//! Rainbow effect color wheel using the onboard NeoPixel on an Waveshare RP2040 Zero board
//!
//! This flows smoothly through various colors on the onboard NeoPixel.
//! Uses the `ws2812_pio` driver to control the NeoPixel, which in turns uses the
//! RP2040's PIO block.
#![no_std]
#![no_main]

use core::iter::once;
use defmt_rtt as _;
use embedded_graphics::{
    image::{Image, ImageRaw},
    pixelcolor::BinaryColor,
    prelude::*,
};
use embedded_hal::timer::CountDown;
use fugit::ExtU32;
use fugit::RateExtU32;
use panic_halt as _;
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::{UsbDeviceBuilder, UsbVidPid};
use usbd_serial::SerialPort;
use usbd_serial::UsbError;
use usbd_serial::USB_CLASS_CDC;
use waveshare_rp2040_zero::entry;
use waveshare_rp2040_zero::{
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        i2c::I2C,
        pac,
        pio::PIOExt,
        timer::Timer,
        usb::UsbBus,
        watchdog::Watchdog,
        Sio,
    },
    Pins, XOSC_CRYSTAL_FREQ,
};
use ws2812_pio::Ws2812;

const FRAMES: &[ImageRaw<BinaryColor>] = &[
    ImageRaw::new(include_bytes!("../video/frame1.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame2.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame3.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame4.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame5.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame6.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame7.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame8.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame9.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame10.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame11.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame12.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame13.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame14.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame15.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame16.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame17.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame18.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame19.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame20.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame21.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame22.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame23.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame24.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame25.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame26.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame27.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame28.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame29.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame30.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame31.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame32.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame33.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame34.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame35.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame36.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame37.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame38.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame39.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame40.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame41.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame42.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame43.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame44.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame45.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame46.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame47.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame48.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame49.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame50.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame51.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame52.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame53.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame54.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame55.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame56.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame57.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame58.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame59.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame60.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame61.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame62.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame63.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame64.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame65.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame66.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame67.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame68.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame69.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame70.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame71.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame72.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame73.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame74.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame75.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame76.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame77.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame78.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame79.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame80.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame81.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame82.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame83.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame84.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame85.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame86.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame87.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame88.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame89.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame90.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame91.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame92.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame93.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame94.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame95.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame96.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame97.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame98.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame99.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame100.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame101.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame102.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame103.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame104.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame105.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame106.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame107.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame108.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame109.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame110.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame111.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame112.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame113.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame114.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame115.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame116.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame117.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame118.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame119.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame120.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame121.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame122.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame123.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame124.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame125.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame126.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame127.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame128.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame129.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame130.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame131.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame132.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame133.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame134.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame135.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame136.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame137.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame138.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame139.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame140.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame141.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame142.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame143.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame144.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame145.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame146.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame147.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame148.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame149.raw"), 128),
    ImageRaw::new(include_bytes!("../video/frame150.raw"), 128),
];

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut delay = timer.count_down();

    // Configure display
    let i2c = I2C::i2c1(
        pac.I2C1,
        pins.gp14.into_function(), // sda
        pins.gp15.into_function(), // scl
        400.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
    );
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let raw: ImageRaw<BinaryColor> = ImageRaw::new(include_bytes!("../video/frame1.raw"), 128);

    let im = Image::new(&raw, Point::new(0, 13));

    im.draw(&mut display).unwrap();

    display.flush().unwrap();

    // Configure the addressable LED
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        // The onboard NeoPixel is attached to GPIO pin #16 on the Feather RP2040.
        pins.neopixel.into_function(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    // Configure USB serial
    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .product("Serial port")
        .device_class(USB_CLASS_CDC)
        .build();

    // enter loop
    let mut n = 0;
    loop {
        let Some(raw) = FRAMES.get(n) else {
            n = 0;
            continue;
        };

        let im = Image::new(raw, Point::new(0, 13));
        im.draw(&mut display).unwrap();
        display.flush().unwrap();

        delay.start(50.millis());
        let _ = nb::block!(delay.wait());

        n = n + 1;

        /*
        // magic wheel
        ws.write(brightness(once(wheel(n)), 32)).unwrap();
        n = n.wrapping_add(1);

        delay.start(50.millis());
        let _ = nb::block!(delay.wait());

        // skip if no usb device connected
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        // read and discard any data sent to us
        let mut buf = [0u8; 64];
        serial.read(&mut buf[..]).ok();

        // send some data
        match serial.write(b"hello!\r\n") {
            Ok(_count) => {
                // count bytes were written
            }
            // no data could be written
            Err(UsbError::WouldBlock) => (),
            // an error occurred
            Err(_err) => (),
        };
        */
    }
}

/// Convert a number from `0..=255` to an RGB color triplet.
///
/// The colours are a transition from red, to green, to blue and back to red.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        // No green in this sector - red and blue only
        (255 - (wheel_pos * 3), 0, wheel_pos * 3).into()
    } else if wheel_pos < 170 {
        // No red in this sector - green and blue only
        wheel_pos -= 85;
        (0, wheel_pos * 3, 255 - (wheel_pos * 3)).into()
    } else {
        // No blue in this sector - red and green only
        wheel_pos -= 170;
        (wheel_pos * 3, 255 - (wheel_pos * 3), 0).into()
    }
}
