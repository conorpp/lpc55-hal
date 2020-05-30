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
    flash::FlashGordon,
};
use hal::traits::wg::digital::v2::ToggleableOutputPin;

use funnel::{funnel, Drain, info};

funnel!(NVIC_PRIO_BITS = hal::raw::NVIC_PRIO_BITS, {
    0: 1024,
    1: 1024,
});

pub trait HexRepresentation2 {
    fn hex(self) -> [u8; 2];
}

impl HexRepresentation2 for u8 {
    fn hex(self) -> [u8; 2] {
        let mut hex = [0x30, 0x30];

        for i in 0 .. 2 {
            let nibble = (self >> (i * 4)) & 0xf;
            hex[1-i] = if  nibble < 0x0a {
                nibble + 0x30
            }
            else
            {
                nibble + 0x41 - 0x0A
            }
        }
        hex
    }
}

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
                    need_zlp = n == 64;
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
    Blink  = 0x61,       // 'a'
    Reboot = 0x62,      // 'b'
    DFU    = 0x63,      // 'c'
    SetErrorCounter = 0x64,      // 'd'
    ReadId = 0x69,      // 'i'
    ReadBootloader = 0x6d,      // 'm'
    DumpBootloader = 0x6f,      // 'o'
}

const BOOTROM_TREE_ADDR: u32 = 0x130010f0;
fn boot_to_dfu(flash: &mut FlashGordon) {

    // Currently just erasing the first flash page and reseting will force DFU..
    flash.erase_page(0);

    hal::raw::SCB::sys_reset();

    // UM 9.3.4
    // let ptr: *const () = 0x1301fe00 as *const ();
    // let bootrom_enter_location_addr: &u32 = unsafe { core::mem::transmute(BOOTROM_TREE_ADDR) };
    // let bootrom_enter_location: u32 = *bootrom_enter_location_addr;
    // let enter_bootrom_tree_code: extern "C" fn(u32)->u32 = unsafe { core::mem::transmute(bootrom_enter_location) };

    // let arg = 0xEB110000;

    // enter_bootrom_tree_code(arg)
}

#[entry]
fn main() -> ! {

    // boot_to_dfu();

    let hal = hal::new();

    let mut anactrl = hal.anactrl;
    let mut pmc = hal.pmc;
    let mut syscon = hal.syscon;

    let mut gpio = hal.gpio.enabled(&mut syscon);
    let mut iocon = hal.iocon.enabled(&mut syscon);

    let mut flash = hal::FlashGordon::new(hal.flash.enabled(&mut syscon));

    let mut red_led = pins::Pio1_21::take().unwrap()
        .into_gpio_pin(&mut iocon, &mut gpio)
        .into_output(hal::drivers::pins::Level::High); // start turned off

    // let mut _green_led = pins::Pio1_21::take().unwrap()
    //     .into_gpio_pin(&mut iocon, &mut gpio)
    //     .into_output(hal::drivers::pins::Level::Low); // start turned on



    let usb0_vbus_pin = pins::Pio0_22::take().unwrap()
        .into_usb0_vbus_pin(&mut iocon);

    iocon.disabled(&mut syscon).release(); // save the environment :)


    let clocks = hal::ClockRequirements::default()
        .system_frequency(96.mhz())
        .support_usbhs()
        .configure(&mut anactrl, &mut pmc, &mut syscon)
        .unwrap();

    let mut delay_timer = Timer::new(hal.ctimer.0.enabled(&mut syscon));

    // do a blink
    red_led.toggle().ok();
    delay_timer.start(150.ms()); nb::block!(delay_timer.wait()).ok();

    red_led.toggle().ok();

    let mut usb_peripheral = hal.usbhs.enabled_as_device(
        &mut anactrl,
        &mut pmc,
        &mut syscon,
        &mut delay_timer,
        clocks.support_usbhs_token()
                        .unwrap()
    );

    let usb_bus = UsbBus::new(usb_peripheral, usb0_vbus_pin);

    let mut cdc_acm = CdcAcmClass::new(&usb_bus, 64);
    // print_type_of(&cdc_acm);

    // Get uuid of device and convert to hex string
    let uuid = hal::uuid();
    let mut uuidHex = [0x0u8; 32];
    for i in 0..16 {
        let byteHex = uuid[i].hex();
        for j in 0 .. 2 {
            uuidHex[i * 2 + j] = byteHex[j];
        }
    }

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0xbeee))
        .manufacturer("nickray")
        .product("Demo Demo Demo")
        .serial_number(unsafe{core::str::from_utf8_unchecked(&uuidHex)})
        .device_release(0x0123)
        // Must be 64 bytes for HighSpeed
        .max_packet_size_0(64)
        .build();

    let mut buf = [0u8; 64];

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
                        boot_to_dfu(&mut flash);
                    }
                    x if (Commands::SetErrorCounter as u8) == x => {
                        info!("Set error counter and reset").unwrap();
                        pmc.boot_to_dfu();
                    }
                    x if (Commands::ReadId as u8) == x => {

                        let uuid = hal::uuid();

                        info!("ID (dec): ").ok();
                        drain_logs(&mut cdc_acm);
                        info!("{} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {}",
                            uuid[0], uuid[1], uuid[2], uuid[3],
                            uuid[4], uuid[5], uuid[6], uuid[7],
                            uuid[8], uuid[9], uuid[10], uuid[11],
                            uuid[12], uuid[13], uuid[14], uuid[15],
                        ).unwrap();
                        drain_logs(&mut cdc_acm);
                    }

                    x if (Commands::ReadBootloader as u8) == x => {
                        let mut uuid = [0u8; 16];
                        // same UUID can be read from boot rom interface
                        // UM: 48.8 UUID
                        flash.read(BOOTROM_TREE_ADDR as usize, &mut uuid);

                        info!("header (dec): ").ok();
                        drain_logs(&mut cdc_acm);
                        info!("{} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {}",
                            uuid[0], uuid[1], uuid[2], uuid[3],
                            uuid[4], uuid[5], uuid[6], uuid[7],
                            uuid[8], uuid[9], uuid[10], uuid[11],
                            uuid[12], uuid[13], uuid[14], uuid[15],
                        ).unwrap();
                        drain_logs(&mut cdc_acm);

                        let bootrom_enter_location_addr: &u32 = unsafe { core::mem::transmute(BOOTROM_TREE_ADDR) };
                        // let bootrom_string_addr: &&[u8;16] = unsafe { core::mem::transmute(BOOTROM_TREE_ADDR + 20) };
                        let bootrom_enter_location: u32 = *bootrom_enter_location_addr;
                        // let enter_bootrom_tree_code: extern "C" fn(&u32)->u32 = unsafe { core::mem::transmute(bootrom_enter_location) };

                        // let boot_string = unsafe{ core::str::from_utf8_unchecked(*bootrom_string_addr) };
                        info!("BOOTROM_TREE_ADDR: {}", BOOTROM_TREE_ADDR).ok();
                        drain_logs(&mut cdc_acm);
                        info!("enter_location: {}", bootrom_enter_location).ok();
                        drain_logs(&mut cdc_acm);
                        // info!("s: {}", boot_string).ok();
                        drain_logs(&mut cdc_acm);

                        // let v1: &u32 = unsafe { core::mem::transmute(BOOTROM_TREE_ADDR+4) };
                        // let v2: &u32 = unsafe { core::mem::transmute(BOOTROM_TREE_ADDR+8) };
                        // let v3: &u32 = unsafe { core::mem::transmute(BOOTROM_TREE_ADDR+12) };
                        // let v4: &u32 = unsafe { core::mem::transmute(BOOTROM_TREE_ADDR+16) };
                        // info!("v: {}.{}.{}.{}", v1,v2,v3,v4).ok();
                        // drain_logs(&mut cdc_acm);

                    }

                    x if (Commands::DumpBootloader as u8) == x => {
                        let mut uuid = [0u8; 16];
                        // same UUID can be read from boot rom interface
                        // UM: 48.8 UUID
                        //            0x0300_ff90
                        for addr in  (0x1300_0000 .. 0x1302_0000).step_by(16) {
                            for byte_addr in addr .. addr + 16 {
                                let byte_pointer: &u8 = unsafe { core::mem::transmute(byte_addr) };
                                uuid[(byte_addr - addr) as usize] = *byte_pointer;
                            }
                            // flash.read(addr, &mut uuid);
                            info!("{} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {}",
                                uuid[0], uuid[1], uuid[2], uuid[3],
                                uuid[4], uuid[5], uuid[6], uuid[7],
                                uuid[8], uuid[9], uuid[10], uuid[11],
                                uuid[12], uuid[13], uuid[14], uuid[15],
                            ).unwrap();
                            drain_logs(&mut cdc_acm);
                        }

                        info!("done.").ok();

                    }

                    _ => {
                        info!("Invalid command.").unwrap();
                    }
                }
            },
            _ => {}
        }

        drain_logs(&mut cdc_acm);

    }

}
