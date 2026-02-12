//! hall_effect_keyboard — Daisy Seed + CD74HC4051 + A1302
#![no_main]
#![no_std]

#[rtic::app(
    device = stm32h7xx_hal::stm32,
    peripherals = true,
    dispatchers = [DMA1_STR0, DMA1_STR2, DMA1_STR3, DMA1_STR4, DMA1_STR5, DMA1_STR6]
)]
mod app {
    const BLOCK_SIZE: usize = 128;

    use libdaisy::gpio::*;
    use libdaisy::logger;
    use libdaisy::{audio, system};
    // GpioExt brings .split() into scope
    use libdaisy::prelude::_stm32h7xx_hal_gpio_GpioExt;
    // OneShot brings .read() into scope
    use cortex_m::prelude::_embedded_hal_adc_OneShot;

    use libdaisy::hal::{
        adc::{Adc, AdcSampleTime, Resolution},
        gpio::{gpiob::*, gpioc::*, Analog, Output, PushPull},
    };
    use log::info;
    use stm32h7xx_hal::time::MilliSeconds;
    use stm32h7xx_hal::{adc, stm32, timer::Timer};

    // ── Key config ──────────────────────────────────────────────────────────────
    const NUM_KEYS: usize = 8;

    /// ADC counts at first actuation point (~25% travel). Tune after measuring.
    const FIRST_THRESHOLD: u16 = 1500;

    /// ADC counts at second actuation point (~75% travel).
    const SECOND_THRESHOLD: u16 = 2800;

    /// Below this -> key released. Lower than FIRST_THRESHOLD for hysteresis.
    const RELEASE_THRESHOLD: u16 = 1200;

    /// Presses faster than this window get maximum velocity.
    /// Presses slower than this get minimum velocity = 1.
    const VELOCITY_WINDOW_MS: u32 = 200;

    // ── Key state machine ────────────────────────────────────────────────────────

    #[derive(Clone, Copy, Debug)]
    enum KeyPhase {
        Idle,
        FirstActuated { value: u16, tick: u32 },
        FullyActuated { velocity: u8 },
    }

    #[derive(Clone, Copy)]
    struct KeyState {
        phase: KeyPhase,
    }

    impl KeyState {
        const fn new() -> Self {
            Self {
                phase: KeyPhase::Idle,
            }
        }

        fn update(&mut self, adc_value: u16, tick: u32) -> Option<KeyEvent> {
            match self.phase {
                KeyPhase::Idle => {
                    if adc_value >= FIRST_THRESHOLD {
                        self.phase = KeyPhase::FirstActuated {
                            value: adc_value,
                            tick,
                        };
                    }
                    None
                }

                KeyPhase::FirstActuated { tick: t1, .. } => {
                    if adc_value >= SECOND_THRESHOLD {
                        let elapsed = tick.saturating_sub(t1);

                        // Fast press (small elapsed) -> high velocity.
                        // Slow press (elapsed >= window) -> velocity 1.
                        // elapsed == 0 is the theoretically instantaneous case -> 127.
                        let velocity = if elapsed == 0 {
                            127u8
                        } else if elapsed >= VELOCITY_WINDOW_MS {
                            1u8
                        } else {
                            // Invert: less time = louder
                            let v = 127u32.saturating_sub((elapsed * 126) / VELOCITY_WINDOW_MS);
                            (v + 1) as u8 // +1 ensures minimum of 1 even on rounding
                        };

                        self.phase = KeyPhase::FullyActuated { velocity };
                        Some(KeyEvent::NoteOn { velocity })
                    } else if adc_value < RELEASE_THRESHOLD {
                        // Released before reaching second threshold -- no note
                        self.phase = KeyPhase::Idle;
                        None
                    } else {
                        None
                    }
                }

                KeyPhase::FullyActuated { .. } => {
                    if adc_value < RELEASE_THRESHOLD {
                        self.phase = KeyPhase::Idle;
                        Some(KeyEvent::NoteOff)
                    } else {
                        None
                    }
                }
            }
        }
    }

    #[derive(Debug)]
    enum KeyEvent {
        NoteOn { velocity: u8 },
        NoteOff,
    }

    // ── RTIC resources ───────────────────────────────────────────────────────────

    #[shared]
    struct Shared {
        tick_ms: u32,
    }

    #[local]
    struct Local {
        audio: audio::Audio,
        adc1: adc::Adc<stm32::ADC1, adc::Enabled>,
        adc_pin: Daisy16<Analog>,
        s0: Daisy0<Output<PushPull>>,
        s1: Daisy1<Output<PushPull>>,
        s2: Daisy2<Output<PushPull>>,
        key_states: [KeyState; NUM_KEYS],
    }

    // ── init ─────────────────────────────────────────────────────────────────────

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        logger::init();

        let mut core = ctx.core;
        let device = ctx.device;

        // ── Clocks ───────────────────────────────────────────────
        let ccdr = system::System::init_clocks(device.PWR, device.RCC, &device.SYSCFG);

        // ── SysTick @ 1 kHz (must be BEFORE system_init!) ────────
        let syst = &mut core.SYST;
        syst.set_reload(480_000 - 1); // 480 MHz / 1 kHz
        syst.clear_current();
        syst.enable_counter();
        syst.enable_interrupt();

        // ── libdaisy system init (moves core/device/ccdr) ────────
        let mut system = libdaisy::system_init!(core, device, ccdr, BLOCK_SIZE);

        // ── GPIO: CD74HC4051 select pins ─────────────────────────
        // D0 = PB8
        let s0 = system
            .gpio
            .daisy0
            .take()
            .expect("Failed to get daisy0")
            .into_push_pull_output();

        // D1 = PB9
        let s1 = system
            .gpio
            .daisy1
            .take()
            .expect("Failed to get daisy1")
            .into_push_pull_output();

        // D2 = PB10
        let s2 = system
            .gpio
            .daisy2
            .take()
            .expect("Failed to get daisy2")
            .into_push_pull_output();

        // ── ADC pin (A0 = PC0) ───────────────────────────────────
        let adc_pin = system
            .gpio
            .daisy16 // A0 on Daisy Seed
            .take()
            .expect("Failed to get A0")
            .into_analog();

        // ── ADC configuration ───────────────────────────────────
        let mut adc1 = system.adc1.enable();
        adc1.set_resolution(adc::Resolution::SixteenBit);
        adc1.set_sample_time(AdcSampleTime::T_64);

        info!("Hall effect keyboard startup done!");

        (
            Shared { tick_ms: 0 },
            Local {
                audio: system.audio,
                adc1,
                adc_pin,
                s0,
                s1,
                s2,
                key_states: [KeyState::new(); NUM_KEYS],
            },
            init::Monotonics(),
        )
    }

    // ── idle ─────────────────────────────────────────────────────────────────────

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    // ── Audio passthrough ────────────────────────────────────────────────────────

    #[task(binds = DMA1_STR1, local = [audio], priority = 8)]
    fn audio_handler(ctx: audio_handler::Context) {
        let audio = ctx.local.audio;
        audio.for_each(|left, right| (left, right));
    }

    // ── 1 ms SysTick ─────────────────────────────────────────────────────────────

    #[task(binds = SysTick, shared = [tick_ms], priority = 15)]
    fn systick(mut ctx: systick::Context) {
        ctx.shared.tick_ms.lock(|t| *t = t.wrapping_add(1));

        // spawn().ok() silently drops if the queue is full -- intentional.
        // Ticks are skipped rather than panicking if scan_keys falls behind.
        scan_keys::spawn().ok();
    }

    // ── Key scanner (spawned every 1 ms by SysTick) ───────────────────────────────

    // capacity = 2: allows one queued scan while another is running.
    #[task(
        local  = [adc1, adc_pin, s0, s1, s2, key_states],
        shared = [tick_ms],
        priority = 2,
        capacity = 2,
    )]
    fn scan_keys(mut ctx: scan_keys::Context) {
        let adc = ctx.local.adc1;
        let adc_pin = ctx.local.adc_pin;
        let s0 = ctx.local.s0;
        let s1 = ctx.local.s1;
        let s2 = ctx.local.s2;
        let states = ctx.local.key_states;

        let now = ctx.shared.tick_ms.lock(|t| *t);

        for key_idx in 0..NUM_KEYS {
            set_mux_channel(key_idx, s0, s1, s2);

            // Allow the mux output to settle before sampling (~10 us).
            // At 480 MHz core clock: 480_000_000 / 1_000_000 * 10 = 4_800 cycles.
            cortex_m::asm::delay(4_800);

            // With SixteenBit resolution the HAL returns u32.
            // Shift down to 12-bit range (0-4095) to match threshold constants.
            let raw: u32 = adc.read(adc_pin).unwrap_or(0);
            let raw = (raw >> 4) as u16;

            if let Some(event) = states[key_idx].update(raw, now) {
                handle_key_event(key_idx, event);
            }
        }
    }

    // ── Helpers ───────────────────────────────────────────────────────────────────

    /// Drive the three CD74HC4051 select lines to choose channel `ch` (0-7).
    fn set_mux_channel(
        ch: usize,
        s0: &mut Daisy0<Output<PushPull>>,
        s1: &mut Daisy1<Output<PushPull>>,
        s2: &mut Daisy2<Output<PushPull>>,
    ) {
        // stm32h7xx-hal OutputPin methods are infallible -- no Result to unwrap.
        if ch & 0b001 != 0 {
            s0.set_high();
        } else {
            s0.set_low();
        }
        if ch & 0b010 != 0 {
            s1.set_high();
        } else {
            s1.set_low();
        }
        if ch & 0b100 != 0 {
            s2.set_high();
        } else {
            s2.set_low();
        }
    }

    /// Respond to a key event. Wire your MIDI UART send or synth trigger here.
    fn handle_key_event(key_idx: usize, event: KeyEvent) {
        match event {
            KeyEvent::NoteOn { velocity } => {
                let note = 60u8 + key_idx as u8; // middle C + offset
                info!("NoteOn  key={} note={} vel={}", key_idx, note, velocity);
                // TODO: midi_uart::note_on(note, velocity);
            }
            KeyEvent::NoteOff => {
                let note = 60u8 + key_idx as u8;
                info!("NoteOff key={} note={}", key_idx, note);
                // TODO: midi_uart::note_off(note);
            }
        }
    }
}
