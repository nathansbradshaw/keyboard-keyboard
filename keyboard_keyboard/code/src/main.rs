//! hall_effect_keyboard — Daisy Seed + CD74HC4051 + A1302
//! Relative-threshold scanning with per-channel baseline calibration
//! MIDI output over USART1 at 31250 baud
#![no_main]
#![no_std]

use panic_rtt_target as _;

mod midi_sender;

#[rtic::app(
    device = stm32h7xx_hal::stm32,
    peripherals = true,
    dispatchers = [DMA1_STR2, DMA1_STR3, DMA1_STR4, DMA1_STR5, DMA1_STR6]
)]
mod app {
    const BLOCK_SIZE: usize = 128;
    const NUM_KEYS: usize = 8;
    const NUM_ACTIVE_KEYS: usize = 6;

    // Map each mux channel to a MIDI note number.
    const KEY_TO_NOTE: [u8; NUM_KEYS] = [48, 50, 52, 55, 57, 60, 62, 64];

    use crate::midi_sender::MidiSender;
    use libdaisy::gpio::*;
    use libdaisy::logger;
    use libdaisy::{audio, system};
    use stm32h7xx_hal::time::MilliSeconds;

    use libdaisy::hal::{
        adc::{self, Adc, AdcSampleTime, Resolution},
        gpio::{Analog, Output, PushPull},
        prelude::*,
        serial::{config::Config as SerialConfig, SerialExt},
        stm32,
        time::U32Ext,
        timer,
    };
    use log::info;

    // ── ADC config ─────────────────────────────────────────────────────────────
    const ADC_SAMPLE_TIME: AdcSampleTime = AdcSampleTime::T_16;
    const ADC_RESOLUTION: Resolution = Resolution::TwelveBit;

    // ── Calibration ────────────────────────────────────────────────────────────
    const CALIBRATION_SAMPLES: usize = 64;

    // ── Key thresholds (relative to per-channel baseline) ─────────────────────
    const FIRST_DELTA: u16 = 200;
    const SECOND_DELTA: u16 = 400;
    const RELEASE_DELTA: u16 = 150;

    // ── Debounce ───────────────────────────────────────────────────────────────
    // How many consecutive ticks a threshold must be exceeded before we act.
    // At 1kHz scan rate, 3 ticks = 3ms — filters out single-sample noise
    // spikes while adding negligible latency.
    const DEBOUNCE_TICKS: u8 = 3;

    // ── Moving average filter ──────────────────────────────────────────────────
    // Number of samples to average per channel. Must be a power of 2 for
    // efficient division. 4 samples = 4ms of smoothing at 1kHz.
    const FILTER_SIZE: usize = 4;
    const FILTER_SHIFT: u32 = 2; // log2(FILTER_SIZE)

    // ── Velocity ───────────────────────────────────────────────────────────────
    const VELOCITY_WINDOW_MS: u32 = 80;

    // ── Diagnostic logging ─────────────────────────────────────────────────────
    // Set to true to log raw ADC values every LOG_INTERVAL_MS.
    // Use this to see what your channels are doing at rest and find noise.
    const DIAG_LOGGING: bool = true;
    const LOG_INTERVAL_MS: u32 = 500;

    // ── Per-channel filter state ───────────────────────────────────────────────
    #[derive(Clone, Copy)]
    pub struct ChannelFilter {
        ring: [u16; FILTER_SIZE],
        index: usize,
        sum: u32,
    }

    impl ChannelFilter {
        const fn new() -> Self {
            Self {
                ring: [0; FILTER_SIZE],
                index: 0,
                sum: 0,
            }
        }

        /// Feed a new raw sample, return the filtered (averaged) value.
        fn feed(&mut self, raw: u16) -> u16 {
            // Subtract the oldest sample, add the new one
            self.sum -= self.ring[self.index] as u32;
            self.sum += raw as u32;
            self.ring[self.index] = raw;
            self.index = (self.index + 1) % FILTER_SIZE;
            (self.sum >> FILTER_SHIFT) as u16
        }

        /// Prime all slots with the same value (used after calibration).
        fn prime(&mut self, value: u16) {
            for slot in self.ring.iter_mut() {
                *slot = value;
            }
            self.sum = value as u32 * FILTER_SIZE as u32;
            self.index = 0;
        }
    }

    // ── Key state machine ──────────────────────────────────────────────────────
    #[derive(Clone, Copy, Debug)]
    pub enum KeyPhase {
        Idle,
        FirstActuated { tick: u32 },
        FullyActuated { velocity: u8 },
    }

    #[derive(Clone, Copy)]
    pub struct KeyState {
        phase: KeyPhase,
        pub last_adc: u16,
        /// Counts consecutive ticks above/below a threshold for debouncing.
        debounce_count: u8,
    }

    impl KeyState {
        const fn new() -> Self {
            Self {
                phase: KeyPhase::Idle,
                last_adc: 0,
                debounce_count: 0,
            }
        }

        fn update(
            &mut self,
            adc_value: u16,
            baseline: u16,
            tick: u32,
            key_idx: usize,
        ) -> Option<KeyEvent> {
            self.last_adc = adc_value;
            let delta = adc_value.saturating_sub(baseline);

            match self.phase {
                KeyPhase::Idle => {
                    if delta >= FIRST_DELTA {
                        self.debounce_count = self.debounce_count.saturating_add(1);
                        if self.debounce_count >= DEBOUNCE_TICKS {
                            info!(
                                "key={} FirstActuated: delta={} adc={} baseline={}",
                                key_idx, delta, adc_value, baseline
                            );
                            self.phase = KeyPhase::FirstActuated { tick };
                            self.debounce_count = 0;
                        }
                    } else {
                        self.debounce_count = 0;
                    }
                    None
                }

                KeyPhase::FirstActuated { tick: t1 } => {
                    if delta >= SECOND_DELTA {
                        self.debounce_count = self.debounce_count.saturating_add(1);
                        if self.debounce_count >= DEBOUNCE_TICKS {
                            let elapsed = tick.saturating_sub(t1);
                            info!(
                                "key={} fully actuated: delta={} elapsed={}ms",
                                key_idx, delta, elapsed
                            );

                            let velocity = if elapsed == 0 {
                                127u8
                            } else if elapsed >= VELOCITY_WINDOW_MS {
                                1u8
                            } else {
                                let v = 127u32.saturating_sub((elapsed * 126) / VELOCITY_WINDOW_MS);
                                (v + 1).min(127) as u8
                            };

                            self.phase = KeyPhase::FullyActuated { velocity };
                            self.debounce_count = 0;
                            Some(KeyEvent::NoteOn { velocity })
                        } else {
                            None
                        }
                    } else if delta < RELEASE_DELTA {
                        self.debounce_count = self.debounce_count.saturating_add(1);
                        if self.debounce_count >= DEBOUNCE_TICKS {
                            self.phase = KeyPhase::Idle;
                            self.debounce_count = 0;
                        }
                        None
                    } else {
                        // In between thresholds — reset debounce
                        self.debounce_count = 0;
                        None
                    }
                }

                KeyPhase::FullyActuated { .. } => {
                    if delta < RELEASE_DELTA {
                        self.debounce_count = self.debounce_count.saturating_add(1);
                        if self.debounce_count >= DEBOUNCE_TICKS {
                            self.phase = KeyPhase::Idle;
                            self.debounce_count = 0;
                            Some(KeyEvent::NoteOff)
                        } else {
                            None
                        }
                    } else {
                        self.debounce_count = 0;
                        None
                    }
                }
            }
        }
    }

    #[derive(Debug)]
    pub enum KeyEvent {
        NoteOn { velocity: u8 },
        NoteOff,
    }

    // ── RTIC resources ──────────────────────────────────────────────────────────
    #[shared]
    struct Shared {
        tick_ms: u32,
        key_states: [KeyState; NUM_KEYS],
        baselines: [u16; NUM_KEYS],
        event_queue: heapless::spsc::Queue<(usize, KeyEvent), 32>,
    }

    #[local]
    struct Local {
        audio: audio::Audio,
        adc: Adc<stm32::ADC1, adc::Enabled>,
        adc_pin: Daisy15<Analog>,
        s0: Daisy0<Output<PushPull>>,
        s1: Daisy1<Output<PushPull>>,
        s2: Daisy2<Output<PushPull>>,
        timer2: timer::Timer<stm32::TIM2>,
        adc_buffer: [u32; NUM_KEYS],
        midi_sender: MidiSender,
        filters: [ChannelFilter; NUM_KEYS],
    }

    // ── init ────────────────────────────────────────────────────────────────────
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        logger::init();

        let mut core = ctx.core;
        let device = ctx.device;

        let ccdr = system::System::init_clocks(device.PWR, device.RCC, &device.SYSCFG);
        let mut system = libdaisy::system_init!(core, device, ccdr, BLOCK_SIZE);

        // ── GPIO (mux select lines) ─────────────────────────────────
        let mut s0 = system.gpio.daisy0.take().unwrap().into_push_pull_output();
        let mut s1 = system.gpio.daisy1.take().unwrap().into_push_pull_output();
        let mut s2 = system.gpio.daisy2.take().unwrap().into_push_pull_output();
        let mut adc_pin = system.gpio.daisy15.take().unwrap().into_analog();

        // ── ADC ──────────────────────────────────────────────────────
        let mut adc = system.adc1.enable();
        adc.set_resolution(ADC_RESOLUTION);
        adc.set_sample_time(ADC_SAMPLE_TIME);

        // Wait for power supply and sensors to fully stabilize
        cortex_m::asm::delay(480 * 50_000); // 50ms startup delay

        // ── Baseline calibration ─────────────────────────────────────
        let mut baselines = [0u16; NUM_KEYS];
        let mut filters = [ChannelFilter::new(); NUM_KEYS];

        for ch in 0..NUM_ACTIVE_KEYS {
            set_mux_channel(ch, &mut s0, &mut s1, &mut s2);
            cortex_m::asm::delay(480 * 200); // 200µs — longer settle for calibration

            let mut accumulator: u32 = 0;
            let mut sample_count = 0u32;
            for i in 0..CALIBRATION_SAMPLES {
                let result: Result<u32, _> = adc.read(&mut adc_pin);
                if let Ok(raw) = result {
                    accumulator += raw;
                    sample_count += 1;
                    if ch == 0 && i < 5 {
                        info!("calibration ch=0 sample {} raw={}", i, raw);
                    }
                }
                cortex_m::asm::delay(480 * 10);
            }
            let avg = if sample_count > 0 {
                accumulator / sample_count
            } else {
                0
            };
            baselines[ch] = avg as u16;

            // Prime the moving average filter with the baseline value
            // so it doesn't ramp up from zero on first scan.
            filters[ch].prime(avg as u16);

            info!(
                "baseline ch={} value={} (from {} samples)",
                ch, baselines[ch], sample_count
            );
        }

        // ── MIDI UART (USART1 @ 31250 baud) ─────────────────────────
        let midi_tx_pin = system
            .gpio
            .daisy13
            .take()
            .expect("Failed to get daisy13 for MIDI TX")
            .into_alternate::<7>();

        let midi_rx_pin = system
            .gpio
            .daisy14
            .take()
            .expect("Failed to get daisy14 for MIDI RX")
            .into_alternate::<7>();

        let mut midi_config = SerialConfig::default();
        midi_config.baudrate = 31_250_u32.bps();

        let midi_serial = device
            .USART1
            .serial(
                (midi_tx_pin, midi_rx_pin),
                midi_config,
                ccdr.peripheral.USART1,
                &ccdr.clocks,
            )
            .unwrap();

        let (midi_tx, _midi_rx) = midi_serial.split();
        let midi_sender = MidiSender::new(midi_tx, 0);

        // ── Timer2 @ 1 kHz ───────────────────────────────────────────
        let mut timer2 = stm32h7xx_hal::timer::TimerExt::timer(
            device.TIM2,
            MilliSeconds::from_ticks(1).into_rate(),
            ccdr.peripheral.TIM2,
            &ccdr.clocks,
        );
        timer2.listen(timer::Event::TimeOut);

        set_mux_channel(0, &mut s0, &mut s1, &mut s2);

        info!(
            "Hall effect keyboard startup done! (DIAG_LOGGING={})",
            DIAG_LOGGING
        );

        (
            Shared {
                tick_ms: 0,
                key_states: [KeyState::new(); NUM_KEYS],
                baselines,
                event_queue: heapless::spsc::Queue::new(),
            },
            Local {
                audio: system.audio,
                adc,
                adc_pin,
                s0,
                s1,
                s2,
                timer2,
                adc_buffer: [0; NUM_KEYS],
                midi_sender,
                filters,
            },
            init::Monotonics(),
        )
    }

    // ── idle ────────────────────────────────────────────────────────────────────
    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    // ── Audio ──────────────────────────────────────────────────────────────────
    #[task(binds = DMA1_STR1, priority = 8, local = [audio])]
    fn audio_handler(ctx: audio_handler::Context) {
        ctx.local.audio.for_each(|left, right| (left, right));
    }

    // ── Timer @ 1 kHz ─────────────────────────────────────────────────────────
    #[task(
        binds = TIM2,
        local  = [timer2, adc, adc_pin, s0, s1, s2, adc_buffer, filters],
        shared = [tick_ms, key_states, baselines, event_queue],
        priority = 15
    )]
    fn timer_handler(mut ctx: timer_handler::Context) {
        ctx.local.timer2.clear_irq();

        let now = ctx.shared.tick_ms.lock(|t| {
            *t = t.wrapping_add(1);
            *t
        });

        let mut pending: heapless::Vec<(usize, KeyEvent), 8> = heapless::Vec::new();
        let baselines = ctx.shared.baselines.lock(|b| *b);

        for ch in 0..NUM_ACTIVE_KEYS {
            set_mux_channel(ch, ctx.local.s0, ctx.local.s1, ctx.local.s2);
            // Increased settle time: 10µs instead of 2µs.
            // This gives the mux output + ADC input capacitance time to
            // settle to the new channel's voltage, preventing crosstalk
            // from the previous channel bleeding into this reading.
            cortex_m::asm::delay(480 * 10);

            let result: Result<u32, _> = ctx.local.adc.read(ctx.local.adc_pin);
            if let Ok(raw) = result {
                ctx.local.adc_buffer[ch] = raw;

                // Feed through the moving average filter before threshold comparison.
                let filtered = ctx.local.filters[ch].feed(raw as u16);

                ctx.shared.key_states.lock(|states| {
                    if let Some(event) = states[ch].update(filtered, baselines[ch], now, ch) {
                        pending.push((ch, event)).ok();
                    }
                });
            }
        }

        // ── Diagnostic logging ───────────────────────────────────────
        // Every LOG_INTERVAL_MS, dump the filtered delta for every active
        // channel so you can see what's happening at rest.
        if DIAG_LOGGING && now % LOG_INTERVAL_MS == 0 {
            for ch in 0..NUM_ACTIVE_KEYS {
                let filtered = (ctx.local.filters[ch].sum >> FILTER_SHIFT) as u16;
                let delta = filtered.saturating_sub(baselines[ch]);
                let raw = ctx.local.adc_buffer[ch];
                info!(
                    "DIAG ch={} raw={} filtered={} baseline={} delta={}",
                    ch, raw, filtered, baselines[ch], delta
                );
            }
        }

        if !pending.is_empty() {
            ctx.shared.event_queue.lock(|queue| {
                for item in pending {
                    queue.enqueue(item).ok();
                }
            });
        }

        process_events::spawn().ok();
    }

    // ── Process events → send MIDI ────────────────────────────────────────────
    #[task(shared = [event_queue], local = [midi_sender], priority = 1, capacity = 32)]
    fn process_events(mut ctx: process_events::Context) {
        ctx.shared.event_queue.lock(|queue| {
            while let Some((key_idx, event)) = queue.dequeue() {
                let note = KEY_TO_NOTE[key_idx];

                match event {
                    KeyEvent::NoteOn { velocity } => {
                        info!("TX NoteOn  key={} note={} vel={}", key_idx, note, velocity);
                        ctx.local.midi_sender.note_on(note, velocity);
                    }
                    KeyEvent::NoteOff => {
                        info!("TX NoteOff key={} note={}", key_idx, note);
                        ctx.local.midi_sender.note_off(note, 0);
                    }
                }
            }
        });
    }

    // ── Helpers ───────────────────────────────────────────────────────────────
    #[inline(always)]
    fn set_mux_channel(
        ch: usize,
        s0: &mut Daisy0<Output<PushPull>>,
        s1: &mut Daisy1<Output<PushPull>>,
        s2: &mut Daisy2<Output<PushPull>>,
    ) {
        if ch & 0b001 != 0 {
            s0.set_high()
        } else {
            s0.set_low()
        }
        if ch & 0b010 != 0 {
            s1.set_high()
        } else {
            s1.set_low()
        }
        if ch & 0b100 != 0 {
            s2.set_high()
        } else {
            s2.set_low()
        }
    }
}
