#![no_main]
#![no_std]

extern crate panic_semihosting;
// extern crate panic_halt;
use cortex_m_rt::entry;
use cortex_m_semihosting::{dbg, heprintln};

#[allow(unused_imports)]
use hal::prelude::*;
#[allow(unused_imports)]
use lpc55_hal as hal;

// use hal::{reg_read, reg_modify};

use usbd_serial::{CdcAcmClass, /*SerialPort*/};
use usb_device::device::{UsbDeviceBuilder, UsbVidPid};
use hal::drivers::{
    pins,
    UsbBus,
    Timer,
};
use hal::traits::wg::digital::v2::ToggleableOutputPin;

use funnel::{funnel, Drain, info};

funnel!(NVIC_PRIO_BITS = hal::raw::NVIC_PRIO_BITS, {
    0: 1024,
    1: 1024,
});

pub fn drain_logs(cdc: 
    &mut CdcAcmClass<
        hal::drivers::usbd::UsbBus<
            hal::peripherals::usbhs::Usbhs<
                hal::typestates::init_state::Enabled, lpc55_hal::typestates::usbhs_mode::Device,
            >
        >
    >) {
    let mut buf = [0u8; 1024];

    let drains = Drain::get_all();

    for (_, drain) in drains.iter().enumerate() {
        'l: loop {
            let n = drain.read(&mut buf).len();
            if n == 0 {
                break 'l;
            }

            let mut need_zlp = false;

            match cdc.write_packet(&buf[..n]) {
                Ok(_count) => {
                    need_zlp = n == 8;
                },
                _ => {}
            }

            if need_zlp {
                match cdc.write_packet(&[]) {
                    Ok(count) => {
                        assert!(count == 0);
                    },
                    _ => {}
                }
            }
        }
    }
}

#[allow(dead_code)]
fn print_type_of<T>(_: &T) {
    heprintln!("{}", core::any::type_name::<T>()).ok();
}


#[derive(Copy, Clone)]
enum Commands {
    Blink = 0x61,       // 'a'
    Reboot = 0x62,      // 'b'
    DFU    = 0x63,      // 'c'
}

#[entry]
fn main() -> ! {

    let hal = hal::new();

    let mut anactrl = hal.anactrl;
    let mut pmc = hal.pmc;
    let mut syscon = hal.syscon;

    let mut gpio = hal.gpio.enabled(&mut syscon);
    let mut iocon = hal.iocon.enabled(&mut syscon);

    let mut red_led = pins::Pio1_6::take().unwrap()
        .into_gpio_pin(&mut iocon, &mut gpio)
        .into_output(hal::drivers::pins::Level::High); // start turned off

    let usb0_vbus_pin = pins::Pio0_22::take().unwrap()
        .into_usb0_vbus_pin(&mut iocon);

    iocon.disabled(&mut syscon).release(); // save the environment :)


    let clocks = hal::ClockRequirements::default()
        .system_frequency(96.mhz())
        .support_usbhs()
        .configure(&mut anactrl, &mut pmc, &mut syscon)
        .unwrap();

    let mut delay_timer = Timer::new(hal.ctimer.0.enabled(&mut syscon));


    let usb_peripheral = hal.usbhs.enabled_as_device(
        &mut anactrl,
        &mut pmc,
        &mut syscon,
        &mut delay_timer,
        clocks.support_usbhs_token()
                        .unwrap()
    );

    let usb_bus = UsbBus::new(usb_peripheral, usb0_vbus_pin);

    let mut cdc_acm = CdcAcmClass::new(&usb_bus, 8);
    // print_type_of(&cdc_acm);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0xcc1d))
        .manufacturer("nickray")
        .product("Demo Demo Demo")
        .serial_number("2019-10-10")
        .device_release(0x0123)
        // Must be 64 bytes for HighSpeed
        .max_packet_size_0(64)
        .build();

    let mut buf = [0u8; 8];

    loop {
        if !usb_dev.poll(&mut [&mut cdc_acm]) {
            continue;
        }

        match cdc_acm.read_packet(&mut buf) {
            Ok(_count) => {
                match buf[0] {
                    x if (Commands::Blink as u8) == x => {
                        info!("Blink").unwrap();
                        red_led.toggle().ok();
                    }
                    x if (Commands::Reboot as u8) == x => {
                        info!("Reboot").unwrap();
                        hal::raw::SCB::sys_reset();
                        
                    }
                    x if (Commands::DFU as u8) == x => {
                        info!("DFU").unwrap();
                        pmc.boot_to_dfu();
                    }
                    _ => {
                        info!("Invalid command {}", buf[0]).unwrap();
                    }
                }
            },
            _ => {}
        }

        drain_logs(&mut cdc_acm);

    }

}