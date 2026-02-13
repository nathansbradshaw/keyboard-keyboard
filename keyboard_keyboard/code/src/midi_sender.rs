//! midi_sender — Encode and transmit MIDI messages over UART
//!
//! Output counterpart to MidiReceiver. Sends standard MIDI messages
//! over USART at 31250 baud using the same HAL types as the rest of
//! the project.

use stm32h7xx_hal::{nb, prelude::*, serial::Tx, stm32};

/// MIDI channel (0–15). Channel 1 in musician-speak is 0 here.
pub const DEFAULT_CHANNEL: u8 = 0;

/// Encodes and sends MIDI messages over UART at 31250 baud.
pub struct MidiSender {
    tx: Tx<stm32::USART1>,
    channel: u8,
}

impl MidiSender {
    pub fn new(tx: Tx<stm32::USART1>, channel: u8) -> Self {
        Self {
            tx,
            channel: channel & 0x0F,
        }
    }

    /// Send a Note On message (status 0x90 | channel, note, velocity).
    pub fn note_on(&mut self, note: u8, velocity: u8) {
        let status = 0x90 | self.channel;
        self.send_byte(status);
        self.send_byte(note & 0x7F);
        self.send_byte(velocity & 0x7F);
    }

    /// Send a Note Off message (status 0x80 | channel, note, velocity).
    pub fn note_off(&mut self, note: u8, velocity: u8) {
        let status = 0x80 | self.channel;
        self.send_byte(status);
        self.send_byte(note & 0x7F);
        self.send_byte(velocity & 0x7F);
    }

    /// Send a Control Change message.
    pub fn control_change(&mut self, controller: u8, value: u8) {
        let status = 0xB0 | self.channel;
        self.send_byte(status);
        self.send_byte(controller & 0x7F);
        self.send_byte(value & 0x7F);
    }

    /// Send a Pitch Bend message.
    /// `value` is 14-bit: 0x2000 = center, 0x0000 = full down, 0x3FFF = full up.
    pub fn pitch_bend(&mut self, value: u16) {
        let status = 0xE0 | self.channel;
        let lsb = (value & 0x7F) as u8;
        let msb = ((value >> 7) & 0x7F) as u8;
        self.send_byte(status);
        self.send_byte(lsb);
        self.send_byte(msb);
    }

    /// Send All Notes Off (CC #123, value 0).
    pub fn all_notes_off(&mut self) {
        self.control_change(123, 0);
    }

    /// Set the MIDI channel (0–15).
    pub fn set_channel(&mut self, channel: u8) {
        self.channel = channel & 0x0F;
    }

    /// Blocking send of a single byte.
    /// At 31250 baud each byte takes ~320µs, so a 3-byte message ≈ 1ms.
    fn send_byte(&mut self, byte: u8) {
        nb::block!(self.tx.write(byte)).ok();
    }
}
