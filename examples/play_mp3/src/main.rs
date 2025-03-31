#![no_std]
#![no_main]

use esp_hal::delay::Delay;
use esp_hal::gpio::{Input, Level, Output, Pull};
use esp_hal::spi::master::Spi;
use esp_hal::spi;
use esp_hal::main;
use esp_backtrace as _;
use vs1053::VS1053;

mod sample_mp3;
use crate::sample_mp3::SAMPLE_MP3;

#[main]
fn main()  -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let spi3_miso_pin = peripherals.GPIO17;
    let spi3_mosi_pin = peripherals.GPIO18;
    let spi3_scl_pin = peripherals.GPIO14;
    let spi_mp3_cs_pin = peripherals.GPIO7;
    let spi_mp3_dc_pin = peripherals.GPIO10;
    let spi_mp3_req_pin = peripherals.GPIO6;
    let spi_mp3_reset_pin = peripherals.GPIO12;

    // test for valid speed might need 400.Khz during setup
    let spi3 = Spi::new(
        peripherals.SPI3,
        spi::master::Config::default(),
    )
        .unwrap()
        .with_sck(spi3_scl_pin)
        .with_mosi(spi3_mosi_pin)
        .with_miso(spi3_miso_pin);

    let mp3_cs = Output::new(spi_mp3_cs_pin, Level::Low);
    let mp3_req = Input::new(spi_mp3_req_pin, Pull::Up);
    let mp3_dc = Output::new(spi_mp3_dc_pin, Level::Low);
    let _mp3_reset = Output::new(spi_mp3_reset_pin, Level::High);
    let mut delay = Delay::new();

    let mut vs1053 = VS1053::new(spi3, mp3_cs, mp3_dc, mp3_req, &mut delay);

    match vs1053.init() {
        Ok(_) => {
            // MUST be before loading patches cause
            // this will call soft_reset which undo patches
            let _ = vs1053.set_mp3_mode_on();

            if vs1053.get_chip_version().unwrap() == 4 {
                let _ = vs1053.load_default_patches();
            }
            let _ = vs1053.set_volume(100);

            let delay = Delay::new();

            loop {
                let _ = vs1053.write(&SAMPLE_MP3);
                delay.delay_millis(3000);
            }
        }
        Err(err) => {
            panic!("VS1053::init failed {:?}", err)
        }
    }
}