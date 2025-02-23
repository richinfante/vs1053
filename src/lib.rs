#![no_std]
#![feature(inherent_associated_types)]

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::SpiBus;

mod vs1053_patches;

pub mod regs {
    pub const SCI_MODE: u8 = 0x0;
    pub const SCI_STATUS: u8 = 0x1;
    pub const SCI_BASS: u8 = 0x2;
    pub const SCI_CLOCKF: u8 = 0x3;
    pub const SCI_DECODE_TIME: u8 = 0x4;
    pub const SCI_AUDATA: u8 = 0x5;
    pub const SCI_WRAM: u8 = 0x6;
    pub const SCI_WRAMADDR: u8 = 0x7;
    pub const SCI_HDAT0: u8 = 0x8;
    pub const SCI_HDAT1: u8 = 0x9;
    pub const SCI_AIADDR: u8 = 0xa;
    /// maximum volume is 0x0000 and total silence is 0xFEFE
    /// Setting SCI_VOL to 0xFFFF will activate analog powerdown mode
    pub const SCI_VOL: u8 = 0xb;
    pub const SCI_AICTRL0: u8 = 0xc;
    pub const SCI_AICTRL1: u8 = 0xd;
    pub const SCI_AICTRL2: u8 = 0xe;
    pub const SCI_AICTRL3: u8 = 0xf;
    pub const SCI_NUM_REGISTERS: u8 = 0xf;
}

pub mod sci_mode {
    pub const SM_DIFF: u8 = 0;
    pub const SM_LAYER12: u8 = 1;
    pub const SM_RESET: u8 = 2;
    pub const SM_CANCEL: u8 = 3;
    pub const SM_EARSPEAKER_LO: u8 = 4;
    pub const SM_TESTS: u8 = 5;
    pub const SM_STREAM: u8 = 6;
    pub const SM_EARSPEAKER_HI: u8 = 7;
    pub const SM_DACT: u8 = 8;
    pub const SM_SDIORD: u8 = 9;
    pub const SM_SDISHARE: u8 = 10;
    pub const SM_SDINEW: u8 = 11;
    pub const SM_ADPCM: u8 = 12;
    pub const SM_ADCPM_HP: u8 = 13;
    pub const SM_LINE1: u8 = 14;
    pub const SM_CLK_RANGE: u8 = 15;
}

const VS1053_CHUNK_SIZE: u8 = 32;
const END_FILL_BYTE: u16 = 0x1e06;

// TODO i2s?
const _ADDR_REG_GPIO_VAL_R: u16 = 0xc018;
const _ADDR_REG_I2S_CONFIG_RW: u16 = 0xc040;

const ADDR_REG_GPIO_DDR_RW: u16 = 0xc017;
const ADDR_REG_GPIO_ODATA_RW: u16 = 0xc019;

pub const VOLUME_MAX: u16 = 0;
pub const VOLUME_MIN: u16 = 0xfefe;
pub const VOLUME_OFF: u16 = 0xffff;

fn map_volume(x: u8, in_min: i16, in_max: i16, out_min: i16, out_max: i16) -> u8 {
    ((x as i16 - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min) as u8
}

#[derive(Clone, Copy, Debug)]
pub enum SpiError<BUS, CS, DC> {
    Spi(BUS),
    Cs(CS),
    Dc(DC),
}

#[derive(Clone, Copy, Debug)]
pub struct VS1053<BUS, CS, DC, REQ, DELAY> {
    spi: BUS,
    cs: CS,
    dc: DC,
    req: REQ,
    delay: DELAY,
    end_fill_byte: u8,
}

impl<BUS, CS, DC, REQ, DELAY> VS1053<BUS, CS, DC, REQ, DELAY>
where
    BUS: SpiBus,
    CS: OutputPin,
    DC: OutputPin,
    REQ: InputPin,
    DELAY: DelayNs,
{
    pub type Error = SpiError<BUS::Error, CS::Error, DC::Error>;

    /// Create new interface
    pub fn new(spi: BUS, cs: CS, dc: DC, req: REQ, delay: DELAY) -> Self {
        Self {
            spi,
            cs,
            dc,
            req,
            delay,
            end_fill_byte: 0,
        }
    }

    fn await_data_request(&mut self) {
        while self.req.is_low().unwrap() {
            self.delay.delay_us(100);
        }
    }

    pub fn data_request(&mut self) -> bool {
        self.req.is_high().unwrap()
    }

    fn control_mode_on(&mut self) -> Result<(), Self::Error> {
        self.dc.set_high().map_err(SpiError::Dc)?;
        self.cs.set_low().map_err(SpiError::Cs)?;
        Ok(())
    }

    fn control_mode_off(&mut self) -> Result<(), Self::Error> {
        self.cs.set_high().map_err(SpiError::Cs)?;
        Ok(())
    }

    fn data_mode_on(&mut self) -> Result<(), Self::Error> {
        self.cs.set_high().map_err(SpiError::Cs)?;
        self.dc.set_low().map_err(SpiError::Dc)?;
        Ok(())
    }

    fn data_mode_off(&mut self) -> Result<(), Self::Error> {
        self.dc.set_high().map_err(SpiError::Dc)?;
        Ok(())
    }

    fn read_register(&mut self, register: u8) -> Result<u16, Self::Error> {
        let write_buffer = [3, register];
        let mut read_bufer: [u8; 2] = [0u8; 2];

        self.control_mode_on()?;
        self.spi.write(&write_buffer).map_err(SpiError::Spi)?;
        self.spi.read(&mut read_bufer).map_err(SpiError::Spi)?;
        let value = u16::from_be_bytes(read_bufer);

        self.await_data_request();
        self.control_mode_off()?;
        Ok(value)
    }

    fn write_register(&mut self, register: u8, value: u16) -> Result<(), Self::Error> {
        let buf: [u8; 2] = u16::to_be_bytes(value);
        let write_buffer = [2, register, buf[0], buf[1]];

        self.control_mode_on()?;
        self.spi.write(&write_buffer).map_err(SpiError::Spi)?;
        self.spi.flush().map_err(SpiError::Spi)?;
        self.await_data_request();
        self.control_mode_off()?;
        Ok(())
    }

    fn test_comm(&mut self, fast: bool) -> Result<bool, Self::Error> {
        // Test the communication with the VS1053 module.  The result wille be returned.
        // If DREQ is low, there is problably no VS1053 connected.  Pull the line HIGH
        // in order to prevent an endless loop waiting for this signal.  The rest of the
        // software will still work, but readbacks from VS1053 will fail.
        let mut cnt = 0;
        let delta = if fast { 30 } else { 300 }; // 3 for fast SPI

        if !self.req.is_high().unwrap() {
            return Ok(false);
        }
        // Further TESTING.  Check if SCI bus can write and read without errors.
        // We will use the volume setting for this.
        // Will give warnings on serial output if DEBUG is active.
        // A maximum of 20 errors will be reported.

        for mut i in 0..0xFFFF {
            if cnt > 20 {
                break;
            }
            i += delta;

            self.write_register(regs::SCI_VOL, i)?; // Write data to SCI_VOL
            let r1 = self.read_register(regs::SCI_VOL)?; // Read back for the first time
            let r2 = self.read_register(regs::SCI_VOL)?; // Read back a second time
            if r1 != r2 || i != r1 || i != r2
            // Check for 2 equal reads
            {
                self.delay.delay_ms(10);
            }
            cnt += 1;
        }
        Ok(cnt == 0)
    }

    pub fn init(&mut self) -> Result<bool, Self::Error> {
        // spi mode should be slow at this point or you will get errors
        self.dc.set_high().map_err(SpiError::Dc)?;
        self.cs.set_high().map_err(SpiError::Cs)?;

        self.delay.delay_ms(100);

        self.dc.set_low().map_err(SpiError::Dc)?;
        self.cs.set_low().map_err(SpiError::Cs)?;

        self.delay.delay_ms(500);

        self.dc.set_high().map_err(SpiError::Dc)?;
        self.cs.set_high().map_err(SpiError::Cs)?;

        self.delay.delay_ms(500);

        if let Ok(_) = self.test_comm(true) {
            self.write_register(regs::SCI_VOL, 0)?;
            self.write_register(regs::SCI_AUDATA, 44101)?;
            self.write_register(regs::SCI_CLOCKF, 6 << 12)?; // Normal clock settings multiplyer 3.0 = 12.2 MHz
            self.write_register(
                regs::SCI_MODE,
                1 << sci_mode::SM_SDINEW | 1 << sci_mode::SM_LINE1,
            )?;

            self.delay.delay_ms(10);
            self.await_data_request();

            if let Ok(value) = self.wram_read(END_FILL_BYTE) {
                self.end_fill_byte = (value & 0xFF) as u8;
            }
            self.delay.delay_ms(100);
            Ok(true)
        } else {
            Ok(false)
        }
    }

    pub fn write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        self.data_mode_on()?;

        for chunk in buf.chunks(VS1053_CHUNK_SIZE as usize) {
            self.await_data_request();

            self.spi.write(chunk).map_err(SpiError::Spi)?;
            self.spi.flush().map_err(SpiError::Spi)?;
        }

        self.data_mode_off()?;

        Ok(())
    }

    pub fn write_end_bytes(&mut self, len: u16) -> Result<(), Self::Error> {
        self.data_mode_on()?;

        for _ in [0..len].chunks(VS1053_CHUNK_SIZE as usize) {
            self.await_data_request();
            self.spi
                .write(&[self.end_fill_byte])
                .map_err(SpiError::Spi)?;
            self.spi.flush().map_err(SpiError::Spi)?;
        }

        self.data_mode_off()?;

        Ok(())
    }

    fn _get_volume_raw(&mut self) -> Result<u16, Self::Error> {
        self.read_register(regs::SCI_VOL)
    }

    fn _set_volume_raw(&mut self, vol: u16) -> Result<(), Self::Error> {
        self.write_register(regs::SCI_VOL, vol)
    }

    pub fn set_volume(&mut self, vol: u8) -> Result<(), Self::Error> {
        // Set volume.  Both left and right.
        // Input value is 0..100.  100 is the loudest.
        let value_left = map_volume(vol, 0, 100, 0xFE, 0); // 0..100% to left channel
        let value_right = map_volume(vol, 0, 100, 0xFE, 0); // 0..100% to right channel

        let buf: [u8; 2] = [value_left, value_right];

        let value = u16::from_be_bytes(buf);
        self.write_register(regs::SCI_VOL, value)
    }

    fn write_cancel(&mut self) -> Result<(), Self::Error> {
        self.write_register(
            regs::SCI_MODE,
            1 << sci_mode::SM_SDINEW | 1 << sci_mode::SM_CANCEL,
        )
    }

    fn read_cancel(&mut self) -> Result<u8, Self::Error> {
        let value = self.read_register(regs::SCI_MODE)?;
        let res = value & 1 << sci_mode::SM_CANCEL;
        Ok(if res == 1 << sci_mode::SM_CANCEL {
            1
        } else {
            0
        })
    }

    fn soft_reset(&mut self) -> Result<(), Self::Error> {
        self.write_register(
            regs::SCI_MODE,
            1 << sci_mode::SM_SDINEW | 1 << sci_mode::SM_RESET,
        )
    }

    pub fn stop_play(&mut self) -> Result<(), Self::Error> {
        self.write_end_bytes(2052)?;
        self.delay.delay_ms(10);
        self.write_cancel()?;

        // TODO
        for _ in 0..200 {
            self.write_end_bytes(32)?;

            if let Ok(res) = self.read_cancel() {
                if res == 0 {
                    self.write_end_bytes(2052)?;
                    return Ok(());
                }
            }
            self.delay.delay_ms(10);
        }
        // TODO error handling
        Ok(())
    }

    pub fn get_chip_connected(&mut self) -> Result<bool, Self::Error> {
        let status = self.read_register(regs::SCI_STATUS)?;

        Ok(!(status == 0 || status == 0xFFFF))
    }

    /**
     * get the Version Number for the VLSI chip
     * VLSI datasheet: 0 for VS1001, 1 for VS1011, 2 for VS1002, 3 for VS1003, 4 for VS1053 and VS8053,
     * 5 for VS1033, 7 for VS1103, and 6 for VS1063.
     */
    pub fn get_chip_version(&mut self) -> Result<u16, Self::Error> {
        let status = self.read_register(regs::SCI_STATUS)?;

        Ok((status & 0x00F0) >> 4)
    }

    fn wram_write(&mut self, address: u16, data: u16) -> Result<(), Self::Error> {
        self.write_register(regs::SCI_WRAMADDR, address)?;
        self.write_register(regs::SCI_WRAM, data)?;
        Ok(())
    }

    fn wram_read(&mut self, address: u16) -> Result<u16, Self::Error> {
        self.write_register(regs::SCI_WRAMADDR, address)?;
        let value = self.read_register(regs::SCI_WRAM)?;
        Ok(value)
    }

    pub fn set_mp3_mode_on(&mut self) -> Result<(), Self::Error> {
        self.wram_write(ADDR_REG_GPIO_DDR_RW, 3)?; // GPIO DDR = 3
        self.wram_write(ADDR_REG_GPIO_ODATA_RW, 0)?; // GPIO ODATA = 0
        self.delay.delay_ms(100);
        self.soft_reset()?;
        Ok(())
    }

    pub fn set_stream_mode_on(&mut self) -> Result<(), Self::Error> {
        self.write_register(
            regs::SCI_MODE,
            1 << sci_mode::SM_SDINEW | 1 << sci_mode::SM_STREAM,
        )?;
        self.delay.delay_ms(10);
        self.await_data_request();
        Ok(())
    }

    pub fn set_stream_mode_off(&mut self) -> Result<(), Self::Error> {
        self.write_register(regs::SCI_MODE, 1 << sci_mode::SM_SDINEW)?;
        self.delay.delay_ms(10);
        self.await_data_request();
        Ok(())
    }

    pub fn load_user_code(&mut self, patches: &[u16; 4667]) -> Result<(), Self::Error> {
        let mut idx = 0;
        while idx < patches.len() {
            let addr = patches[idx].to_be_bytes();
            idx += 1;
            let mut n = patches[idx];
            idx += 1;
            if n & 0x8000 != 0 {
                /* RLE run, replicate n samples */
                n &= 0x7FFF;
                let val = patches[idx];
                idx += 1;
                for _ in 0..n {
                    self.write_register(addr[1], val)?;
                }
            } else {
                /* Copy run, copy n samples */
                for _ in 0..n {
                    let val = patches[idx];
                    idx += 1;
                    self.write_register(addr[1], val)?;
                }
            }
        }
        Ok(())
    }

    pub fn load_default_patches(&mut self) -> Result<(), Self::Error> {
        self.load_user_code(&vs1053_patches::PATCHES)
    }
}
