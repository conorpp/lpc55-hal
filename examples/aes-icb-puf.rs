#![no_main]
#![no_std]

///
/// Compare also with: https://github.com/Ko-/aes-armcortexm
///
extern crate panic_semihosting;
use cortex_m_rt::entry;

use core::convert::TryInto;

#[allow(unused_imports)]
use hal::prelude::*;
#[allow(unused_imports)]
use lpc55_hal as hal;

use aes_soft::block_cipher::{BlockCipher, NewBlockCipher};

use block_cipher::BlockCipher as _;
use generic_array::GenericArray;

use cortex_m_semihosting::{dbg, hprintln};

#[entry]
fn main() -> ! {
    let dp = hal::raw::Peripherals::take().unwrap();
    let mut syscon = hal::Syscon::from(dp.SYSCON);
    let mut hashcrypt = hal::Hashcrypt::from(dp.HASHCRYPT).enabled(&mut syscon);

    let mut icb_vector_key: [u8; 16]
        = [0xef, 0xa5, 0xb7, 0x42, 0x9c, 0xd1, 0x53, 0xbf, 0x00, 0x86, 0xde, 0xf9, 0x00, 0xc0, 0xf2, 0x35];
    let mut icb_vector_iv: [u8; 16]
        // = [0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01];
        = [0x90, 0x36, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
    let mut icb_vector_pt: [u8; 16]
        = [0xe7, 0xf6, 0x1e, 0x01, 0x2f, 0x4f, 0x32, 0x55, 0x31, 0x2b, 0xa6, 0x8b, 0x1d, 0x2f, 0xda, 0xbf];
//       = [
//     218,
//     71,
//     12,
//     221,
//     61,
//     176,
//     125,
//     137,
//     204,
//     78,
//     47,
//     229,
//     2,
//     242,
//     195,
//     117,
// ];
    let mut icb_vector_ct: [u8; 16]
        = [0xea, 0x6e, 0x09, 0xac, 0x2f, 0xb9, 0x7e, 0x10, 0x2d, 0x8c, 0xa6, 0x4c, 0x1c, 0xbc, 0x0c, 0x0c];

    for i in 0..16 {
        // Changing the key does change the cipher text a lot.
        // icb_vector_key[i] = icb_vector_key[i] ^ 0;

        // This will apply to the resulting cipher text...
        // icb_vector_pt[i] = icb_vector_pt[i] ^ 1;
    }

    // icb_vector_key.reverse();
    // icb_vector_iv.reverse();
    // icb_vector_pt.reverse();
    // icb_vector_ct.reverse();

    let mut icb_block = GenericArray::clone_from_slice(&icb_vector_pt);

    hprintln!("Checking icb vector").ok();
    let cipher = hashcrypt.aes128_icb(&icb_vector_key, &icb_vector_iv);
    // cipher.prime_for_encryption();

    dbg!(icb_block);
    dbg!("running hw encrypt");
    let (icb_cyc_enc, _) = hal::count_cycles(|| {

        cipher.encrypt_block(&mut icb_block);
    });
    dbg!(icb_cyc_enc);

    dbg!(icb_block);
    // dbg!(icb_vector_ct);

    // attempt at dumping the beginning of ICB's keystream..
    let mut icb_xor = [0u8; 16];
    for i in 0..16 {
        icb_xor[i] = icb_block[i] ^ icb_vector_pt[i];
    }
    dbg!(icb_xor);


    let hashcrypt = hashcrypt.release();
    let config = hashcrypt.config.read().bits();
    hprintln!("config = {:08X}", config).ok();


    // DONE
    dbg!("all done");
    loop { continue; }
}
