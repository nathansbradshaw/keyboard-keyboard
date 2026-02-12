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

    use core::fmt::Write;
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
    const SECOND_THRESHOLD: u16 = 2200;

    /// Below this -> key released. Lower than FIRST_THRESHOLD for hysteresis.
    const RELEASE_THRESHOLD: u16 = 1000;

    /// Presses faster than this window get maximum velocity.
    /// Presses slower than this get minimum velocity = 1.
    const VELOCITY_WINDOW_MS: u32 = 200;

    // ── Debug config ───────────────────────────────────────────────────────────
    /// Enable continuous ADC output
    const DEBUG_ADC_OUTPUT: bool = false;
    /// Output interval in milliseconds
    const DEBUG_OUTPUT_INTERVAL_MS: u32 = 50;
    /// Comma-separated list of channels to output (empty = all channels)
    const DEBUG_CHANNELS: &[usize] = &[];

    // ── Key state machine ────────────────────────────────────────────────────────

    #[derive(Clone, Copy, Debug)]
    enum KeyPhase {
        Idle,
        FirstActuated { value: u16, tick: u32 },
        FullyActuated { velocity: u8 },
    }

    #[derive(Clone, Copy)]
    pub struct KeyState {
        phase: KeyPhase,
        last_adc: u16,
    }

    impl KeyState {
        const fn new() -> Self {
            Self {
                phase: KeyPhase::Idle,
                last_adc: 0,
            }
        }

        fn update(&mut self, adc_value: u16, tick: u32) -> Option<KeyEvent> {
            self.last_adc = adc_value;

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

                        let velocity = if elapsed == 0 {
                            127u8
                        } else if elapsed >= VELOCITY_WINDOW_MS {
                            1u8
                        } else {
                            let v = 127u32.saturating_sub((elapsed * 126) / VELOCITY_WINDOW_MS);
                            (v + 1) as u8
                        };

                        self.phase = KeyPhase::FullyActuated { velocity };
                        Some(KeyEvent::NoteOn { velocity })
                    } else if adc_value < RELEASE_THRESHOLD {
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
        last_debug_tick: u32,
        key_states: [KeyState; NUM_KEYS],
    }

    #[local]
    struct Local {
        audio: audio::Audio,
        adc1: adc::Adc<stm32::ADC1, adc::Enabled>,
        adc_pin: Daisy15<Analog>,
        s0: Daisy0<Output<PushPull>>,
        s1: Daisy1<Output<PushPull>>,
        s2: Daisy2<Output<PushPull>>,
        timer2: Timer<stm32::TIM2>, // Add timer
    }

    // ── init ─────────────────────────────────────────────────────────────────────

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        logger::init();

        let mut core = ctx.core;
        let device = ctx.device;

        // ── Clocks ───────────────────────────────────────────────
        let ccdr = system::System::init_clocks(device.PWR, device.RCC, &device.SYSCFG);

        // ── libdaisy system init ────────────────────────────────
        let mut system = libdaisy::system_init!(core, device, ccdr, BLOCK_SIZE);

        // ── GPIO: CD74HC4051 select pins ─────────────────────────
        let mut s0 = system
            .gpio
            .daisy0
            .take()
            .expect("Failed to get daisy0")
            .into_push_pull_output();

        let mut s1 = system
            .gpio
            .daisy1
            .take()
            .expect("Failed to get daisy1")
            .into_push_pull_output();

        let mut s2 = system
            .gpio
            .daisy2
            .take()
            .expect("Failed to get daisy2")
            .into_push_pull_output();

        // ── ADC pin (A0 = PC0) ───────────────────────────────────
        let mut adc_pin = system
            .gpio
            .daisy15
            .take()
            .expect("Failed to get A0")
            .into_analog();

        // ── ADC configuration ───────────────────────────────────
        let mut adc1 = system.adc1.enable();
        adc1.set_resolution(adc::Resolution::SixteenBit);
        adc1.set_sample_time(AdcSampleTime::T_64);

        // ── Timer2 @ 1kHz ───────────────────────────────────────
        let mut timer2 = stm32h7xx_hal::timer::TimerExt::timer(
            device.TIM2,
            MilliSeconds::from_ticks(100).into_rate(),
            ccdr.peripheral.TIM2,
            &ccdr.clocks,
        );
        timer2.listen(stm32h7xx_hal::timer::Event::TimeOut);

        timer2.set_freq(MilliSeconds::from_ticks(5).into_rate());

        info!("Hall effect keyboard startup done!");

        // Test the multiplexer
        info!("Testing 74HC4051...");

        // Test all combinations of select pins
        for s2_val in 0..2 {
            for s1_val in 0..2 {
                for s0_val in 0..2 {
                    if s0_val == 0 {
                        s0.set_low();
                    } else {
                        s0.set_high();
                    }
                    if s1_val == 0 {
                        s1.set_low();
                    } else {
                        s1.set_high();
                    }
                    if s2_val == 0 {
                        s2.set_low();
                    } else {
                        s2.set_high();
                    }

                    cortex_m::asm::delay(48_000);

                    let raw: u32 = adc1.read(&mut adc_pin).unwrap_or(0);
                    let raw = (raw >> 4) as u16;

                    let ch = (s2_val << 2) | (s1_val << 1) | s0_val;
                    info!(
                        "S2={} S1={} S0={} -> Ch{}: {}",
                        s2_val, s1_val, s0_val, ch, raw
                    );
                }
            }
        }

        (
            Shared {
                tick_ms: 0,
                last_debug_tick: 0,
                key_states: [KeyState::new(); NUM_KEYS],
            },
            Local {
                audio: system.audio,
                adc1,
                adc_pin,
                s0,
                s1,
                s2,
                timer2, // Add timer to locals
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

    // ── 1 ms Timer2 ─────────────────────────────────────────────────────────────

    #[task(binds = TIM2, local = [timer2], shared = [tick_ms, last_debug_tick], priority = 15)]
    fn timer_handler(mut ctx: timer_handler::Context) {
        // Clear the interrupt flag
        ctx.local.timer2.clear_irq();

        // Increment tick counter
        let now = ctx.shared.tick_ms.lock(|t| {
            *t = t.wrapping_add(1);
            *t
        });

        // Spawn key scanner
        scan_keys::spawn().ok();

        // Spawn debug output task periodically
        if DEBUG_ADC_OUTPUT {
            let last_debug = ctx.shared.last_debug_tick.lock(|t| *t);
            if now.wrapping_sub(last_debug) >= DEBUG_OUTPUT_INTERVAL_MS {
                ctx.shared.last_debug_tick.lock(|t| *t = now);
                output_adc_values::spawn().ok();
            }
        }
    }

    // ── Key scanner ───────────────────────────────────────────────────────────────

    #[task(
        local  = [adc1, adc_pin, s0, s1, s2],
        shared = [tick_ms, key_states],
        priority = 2,
        capacity = 2,
    )]
    fn scan_keys(mut ctx: scan_keys::Context) {
        let adc = ctx.local.adc1;
        let adc_pin = ctx.local.adc_pin;
        let s0 = ctx.local.s0;
        let s1 = ctx.local.s1;
        let s2 = ctx.local.s2;

        let now = ctx.shared.tick_ms.lock(|t| *t);

        for key_idx in 0..NUM_KEYS {
            set_mux_channel(key_idx, s0, s1, s2);
            cortex_m::asm::delay(4_800);

            let raw: u32 = adc.read(adc_pin).unwrap_or(0);
            let raw = (raw >> 4) as u16;

            ctx.shared.key_states.lock(|states| {
                if let Some(event) = states[key_idx].update(raw, now) {
                    handle_key_event(key_idx, event);
                }
            });
        }
    }

    // ── Debug: Output ADC values ─────────────────────────────────────────────────

    #[task(
        shared = [key_states],
        priority = 1,
        capacity = 2,
    )]
    fn output_adc_values(mut ctx: output_adc_values::Context) {
        info!("output adc values");
        ctx.shared.key_states.lock(|states| {
            let mut output = heapless::String::<128>::new();

            if DEBUG_CHANNELS.is_empty() {
                output.push_str("ADC: ").ok();
                for (i, state) in states.iter().enumerate() {
                    if i > 0 {
                        output.push_str(", ").ok();
                    }
                    write!(&mut output, "{}:{}", i, state.last_adc).ok();
                }
            } else {
                output.push_str("ADC (selected): ").ok();
                for (j, &ch) in DEBUG_CHANNELS.iter().enumerate() {
                    if ch < NUM_KEYS {
                        if j > 0 {
                            output.push_str(", ").ok();
                        }
                        write!(&mut output, "{}:{}", ch, states[ch].last_adc).ok();
                    }
                }
            }

            info!("{}", output);
        });
    }

    // ── Helpers ───────────────────────────────────────────────────────────────────

    fn set_mux_channel(
        ch: usize,
        s0: &mut Daisy0<Output<PushPull>>,
        s1: &mut Daisy1<Output<PushPull>>,
        s2: &mut Daisy2<Output<PushPull>>,
    ) {
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

    fn handle_key_event(key_idx: usize, event: KeyEvent) {
        match event {
            KeyEvent::NoteOn { velocity } => {
                let note = 60u8 + key_idx as u8;
                info!("NoteOn  key={} note={} vel={}", key_idx, note, velocity);
            }
            KeyEvent::NoteOff => {
                let note = 60u8 + key_idx as u8;
                info!("NoteOff key={} note={}", key_idx, note);
            }
        }
    }
}
