#![no_std]
#![no_main]

use embedded_graphics as eg;
use panic_halt as _;
use wio_terminal as wio;

use wio::hal::clock::GenericClockController;
use wio::hal::delay::Delay;
use wio::pac::{CorePeripherals, Peripherals};
use wio::prelude::*;
use wio::wifi_prelude::*;
use wio::wifi_rpcs as rpc;
use wio::wifi_types::Security;
use wio::{entry, wifi_singleton, Pins, Sets};

use core::fmt::Write;
use cortex_m::interrupt::free as disable_interrupts;
use eg::fonts::{Font6x12, Text};
use eg::pixelcolor::Rgb565;
use eg::prelude::*;
use eg::style::TextStyle;
use heapless::{consts::U256, consts::U4096, String};



#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();

    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.MCLK,
        &mut peripherals.OSC32KCTRL,
        &mut peripherals.OSCCTRL,
        &mut peripherals.NVMCTRL,
    );
    let mut delay = Delay::new(core.SYST, &mut clocks);
    let mut sets: Sets = Pins::new(peripherals.PORT).split();

    // Set up the display so we can print out APs.
    let (mut display, _backlight) = sets
        .display
        .init(
            &mut clocks,
            peripherals.SERCOM7,
            &mut peripherals.MCLK,
            &mut sets.port,
            24.mhz(),
            &mut delay,
        )
        .unwrap();
    clear(&mut display);
    let mut textbuffer = String::<U256>::new();

    let mut user_led = sets.user_led.into_open_drain_output(&mut sets.port);
    user_led.set_low().unwrap();

    // Initialize the wifi peripheral.
    let args = (
        sets.wifi,
        peripherals.SERCOM0,
        &mut clocks,
        &mut peripherals.MCLK,
        &mut sets.port,
        &mut delay,
    );
    let nvic = &mut core.NVIC;
    disable_interrupts(|cs| unsafe {
        wifi_init(cs, args.0, args.1, args.2, args.3, args.4, args.5).unwrap();
        WIFI.as_mut().map(|wifi| {
            wifi.enable(cs, nvic);
        });
    });

    let version = unsafe {
        WIFI.as_mut()
            .map(|wifi| wifi.blocking_rpc(rpc::GetVersion {}).unwrap())
            .unwrap()
    };
    writeln!(textbuffer, "fw: {}", version).unwrap();
    write(
        &mut display,
        textbuffer.as_str(),
        Point::new(320 - (3 + version.len() * 12) as i32, 3),
    );
    textbuffer.truncate(0);

    let mac = unsafe {
        WIFI.as_mut()
            .map(|wifi| wifi.blocking_rpc(rpc::GetMacAddress {}).unwrap())
            .unwrap()
    };
    writeln!(textbuffer, "mac: {}", mac).unwrap();
    write(&mut display, textbuffer.as_str(), Point::new(3, 3));
    textbuffer.truncate(0);

    let ip_info = unsafe {
        WIFI.as_mut()
            .map(|wifi| {
                wifi.connect_to_ap(
                    &mut delay,
                    "+++",
                    "***",
                    Security::WPA2_SECURITY | Security::AES_ENABLED,
                )
                .unwrap()
            })
            .unwrap()
    };
    user_led.set_high().ok();
    writeln!(textbuffer, "ip = {}", ip_info.ip).unwrap();
    write(&mut display, textbuffer.as_str(), Point::new(3, 20));
    textbuffer.truncate(0);

    //connect 
    let ip = 0xBA02A8C0;//192.168.2.186
    let port = 0x3d22; //8765;
    let timeout = 100*1000; //100ms
    unsafe {
        WIFI.as_mut()
            .map(|wifi| {
                let r = wifi.connect(ip, port, timeout);
                match r{
                    Ok(_) => {

                            writeln!(textbuffer, "Connect OK").unwrap();
                            write(&mut display, textbuffer.as_str(), Point::new(3, 34));
                            textbuffer.truncate(0);
                    },
                    Err(_) => {

                            writeln!(textbuffer, "Err").unwrap();
                            write(&mut display, textbuffer.as_str(), Point::new(3, 34));
                            textbuffer.truncate(0);
                    },
                };
                let ret = r.unwrap();
                ret
            })
            .unwrap()
    };

    //send message
    let msg = "GET / HTTP/1.1\r\n\r\n";

    unsafe {
        WIFI.as_mut()
        .map(|wifi| {
            let r = wifi.send(&msg);
            match r{
                Ok(_) => {
                    writeln!(textbuffer, "Ok, msg: {}", msg).unwrap();
                    write(&mut display, textbuffer.as_str(), Point::new(3, 44));
                    textbuffer.truncate(0);
                },
                Err(_) => {

                        writeln!(textbuffer, "Err").unwrap();
                        write(&mut display, textbuffer.as_str(), Point::new(3, 44));
                        textbuffer.truncate(0);
                },
            };
            let ret = r.unwrap();
            ret
        }).unwrap()
    };


    //recv message
    let mut text= String::<U4096>::new();
    let mut countdown = 7u32;
    let mut body_length = 0;
    loop {
        unsafe {
            WIFI.as_mut()
            .map(|wifi| {
                let r = wifi.recv();
                match r{
                    Ok(a) => {
                        let t= String::from_utf8(a).unwrap();
                        text.push_str(t.as_str()).ok();
                    },
                    Err(_) => {
                        writeln!(textbuffer, "Err").unwrap();
                        write(&mut display, textbuffer.as_str(), Point::new(3, 54));
                        textbuffer.truncate(0);
                    },
                };
            }).unwrap()
        };
        countdown-=1;

        if body_length == 0 {
            let ret = find_content_length(&text);
            match ret{
                Ok(a) => {                    
                    body_length = a;
                    countdown = (a+511)/512;
                },
                Err(_) => {}
            }
        }

;
    
        if countdown == 0 {break;}
    }

    writeln!(textbuffer, "index : {}", body_length).unwrap();
    write(&mut display, textbuffer.as_str(), Point::new(3, 54));
    textbuffer.truncate(0);

    write(&mut display, text.as_str(), Point::new(3, 64));
    textbuffer.truncate(0);

    writeln!(textbuffer, "fin recv").unwrap();
    write(&mut display, textbuffer.as_str(), Point::new(3, 224));
    textbuffer.truncate(0);

    loop {
        user_led.toggle();
        delay.delay_ms(200u8);
    }
}

wifi_singleton!(WIFI);

fn clear(display: &mut wio::LCD) {
    display.clear(Rgb565::BLACK).ok().unwrap();
}

fn write<'a, T: Into<&'a str>>(display: &mut wio::LCD, text: T, pos: Point) {
    Text::new(text.into(), pos)
        .into_styled(TextStyle::new(Font6x12, Rgb565::WHITE))
        .draw(display)
        .ok()
        .unwrap();
}

fn find_content_length(text : &heapless::String<U4096>) -> Result<u32, ()>{
    let s: &str = "Content-Length:";
    let mut j : usize= 0;
    let mut p : Option<u32> = None;
    for i in 0..text.len() as usize{
        if text.as_bytes()[i] == s.as_bytes()[j]{
            j+=1;
        }else if text.as_bytes()[i] == s.as_bytes()[0]{
            j=1;
        }else{
            j=0;
        }

        if j==s.len() {
            p = Some(i as u32 + 1);
            break;
        }
    }

    if p==None{
        return Err(());
    }

    let p_start = p.unwrap() as usize;

    //find CRLF
    let mut p_end = 0;
    for i in p_start..text.len() as usize{
        let t = text.as_bytes()[i];
        if t=='\r' as u8 || t=='\n' as u8 {
            p_end = i;
            break;
        }
    }

    // parse num
    let mut n = 1u32;
    let mut ret = 0;
    for i in (p_start .. p_end).rev() {
        let t = text.as_bytes()[i];
        if 0x30u8 <= t && t<= 0x39{
            ret += n*(t-0x30u8) as u32;
            n*=10;
        }
    }

    Ok(ret)
}
