
//use atsamd_hal::clock::GenericClockController;
//use atsamd_hal::delay::Delay;
use atsamd_hal::gpio::*;
use atsamd_hal::prelude::*;
//use atsamd_hal::target_device::{interrupt, MCLK};

use atsamd_hal::sercom::{PadPin, Sercom0Pad0, Sercom0Pad2, UART0};
//use atsamd_hal::target_device::SERCOM0;
//use atsamd_hal::time::Hertz;

//use bbqueue;
//use bbqueue::{
//    consts::{U128, U512, U64},
//    BBBuffer, Consumer, Producer,
//};
use atsamd_hal::{
    clock::GenericClockController,
    delay::Delay,
    ehal::blocking::delay::DelayMs,
    ehal::digital::v2::OutputPin,
    ehal::serial::{Read, Write},
    pac::{interrupt, MCLK, SERCOM0},
    prelude::nb,
    sercom::v2::{uart, IoSet2, Sercom0},
    time::{Hertz, U32Ext},
};
use bbqueue::{self, BBBuffer, Consumer, Producer};
use heapless::consts::*;

use cortex_m::interrupt::CriticalSection;
use cortex_m::peripheral::NVIC;

pub use erpc::rpcs;
use seeed_erpc as erpc;

use super::pins::aliases::*;

use crate::WIFI_UART_BAUD;

//const value for socket
const AF_INET :i32 = 2;
const SOCK_STREAM :i32 = 1;
const F_GETFL :i32= 3;
const F_SETFL :i32=  4;
const O_NON_BLOCK :i32= 0x0001;
const MSG_DONTWAIT : i32= 0x08;

/// The set of pins which are connected to the RTL8720 in some way
pub struct WifiPins {
    pub pwr: WifiPwrReset,
    pub rxd: WifiRxdReset,
    pub txd: WifiTxdReset,
    pub mosi: WifiTxReset,
    pub clk: WifiClkReset,
    pub miso: WifiRxReset,
    pub cs: WifiCsReset,
    pub ready: WifiReadyReset,
    pub dir: WifiDirReset,
}

/// eRPC-based protocol to the RTL8720 chip
pub struct Wifi {
    _pwr: WifiPwr,
    uart: WifiUart,

    rx_buff_isr: Producer<'static, 512>,
    rx_buff_input: Consumer<'static, 512>,
    tx_buff_isr: Consumer<'static, 128>,
    tx_buff_input: Producer<'static, 128>,

    sequence: u32,

    sock_fd : Option<i32>,
}

/// UART pads for the labelled RX & TX pins
pub type WifiUartPads = uart::Pads<Sercom0, IoSet2, WifiRx, WifiTx>;

/// UART device for the labelled RX & TX pins
pub type WifiUart = uart::Uart<uart::Config<WifiUartPads>, uart::Duplex>;

impl Wifi {
    pub fn init(
        pins: WifiPins,
        sercom0: SERCOM0,
        clocks: &mut GenericClockController,
        mclk: &mut MCLK,
        delay: &mut Delay,
        rx_buff: &'static BBBuffer<512>,
        tx_buff: &'static BBBuffer<128>,
    ) -> Wifi {
        let gclk0 = clocks.gclk0();

        let pads = uart::Pads::default().rx(pins.miso).tx(pins.mosi);
        let uart = uart::Config::new(
            mclk,
            sercom0,
            pads,
            clocks.sercom0_core(&gclk0).unwrap().freq(),
        )
        .baud(
            WIFI_UART_BAUD.hz(),
            uart::BaudMode::Fractional(uart::Oversampling::Bits16),
        )
        .enable();

        delay.delay_ms(10u8);

        // Reset the RTL8720 MCU.
        let mut pwr: WifiPwr = pins.pwr.into();
        pwr.set_low().ok();
        delay.delay_ms(100u8);
        pwr.set_high().ok();
        delay.delay_ms(200u8);

        let (rx_buff_isr, rx_buff_input) = rx_buff.try_split().unwrap();
        let (tx_buff_input, tx_buff_isr) = tx_buff.try_split().unwrap();

        let sequence = 0;

        Wifi {
            _pwr: pwr,
            uart,
            rx_buff_isr,
            rx_buff_input,
            tx_buff_isr,
            tx_buff_input,
            sequence,
            sock_fd:None,
        }
    }

    /// Turns on internal interrupts. Call this after you have finished
    /// initializing the rest of your peripherals but before you start
    /// issuing RPCs against the wifi chip.
    pub fn enable(&mut self, _cs: &CriticalSection, nvic: &mut NVIC) {
        unsafe {
            nvic.set_priority(interrupt::SERCOM0_0, 1);
            NVIC::unmask(interrupt::SERCOM0_0);
            nvic.set_priority(interrupt::SERCOM0_2, 1);
            NVIC::unmask(interrupt::SERCOM0_2);
        }

        self.uart.enable_interrupts(uart::Flags::RXC);
    }

    /// Convenience function to connection an access point with the given
    /// network name and security parameters, and request an IP via DHCP.
    pub fn connect_to_ap<S: Into<heapless::String<U64>>, P: Into<heapless::String<U64>>>(
        &mut self,
        delay: &mut Delay,
        ssid: S,
        pw: P,
        security: erpc::Security,
    ) -> Result<erpc::IPInfo, erpc::Err<()>> {
        self.blocking_rpc(rpcs::AdapterInit {})?;
        self.blocking_rpc(rpcs::DHCPClientStop {
            interface: erpc::L3Interface::Station,
        })?;
        self.blocking_rpc(rpcs::WifiOff {})?;

        delay.delay_ms(35u8);

        self.blocking_rpc(rpcs::WifiOn {
            mode: erpc::WifiMode::Station,
        })?;

        self.blocking_rpc(rpcs::WifiConnect {
            ssid: ssid.into(),
            password: pw.into(),
            security: security,
            semaphore: 0,
        })?;

        self.blocking_rpc(rpcs::DHCPClientStart {
            interface: erpc::L3Interface::Station,
        })?;

        delay.delay_ms(25u8);
        self.blocking_rpc(rpcs::GetIPInfo {
            interface: erpc::L3Interface::Station,
        })
        .map_err(|_| erpc::Err::RPCErr(()))
    }

    /// Called from ISR: Handles the signal that the UART has recieved
    /// a byte that needs to be read.
    pub fn _handle_rx(&mut self) {
        match self.uart.read() {
            Ok(b) => {
                if let Ok(mut wgr) = self.rx_buff_isr.grant_exact(1) {
                    wgr[0] = b;
                    wgr.commit(1);
                } else {
                    panic!("overrun");
                }
            }
            Err(nb::Error::Other(e)) => {
                panic!("unrecoverable read error");
            }
            // Skip WouldBlock
            Err(nb::Error::WouldBlock) => (),
        };
    }

    /// Called from ISR: Handles the signal that the outgoing UART buffer
    /// has room for the next byte.
    pub fn _handle_data_empty(&mut self) {
        if let Ok(rgr) = self.tx_buff_isr.read() {
            let buf = rgr.buf();
            self.uart.write(buf[0]).ok();
            rgr.release(1);
        } else {
            self.uart.disable_interrupts(uart::Flags::DRE);
        }
    }

    /// Issues an RPC, blocking till a response is recieved.
    pub fn blocking_rpc<'a, RPC: erpc::RPC>(
        &mut self,
        mut rpc: RPC,
    ) -> Result<RPC::ReturnValue, erpc::Err<RPC::Error>> {
        // Transmit the request.
        let mut tx_buff = heapless::Vec::new();
        tx_buff
            .extend_from_slice(&rpc.header(self.next_seq()).as_bytes())
            .map_err(|_| erpc::Err::TXErr)?;
        rpc.args(&mut tx_buff);
        self.write_frame(&tx_buff).map_err(|_| erpc::Err::TXErr)?;

        loop {
            let result = self.recieve_rpc_response(&mut rpc);
            if let Err(erpc::Err::NotOurs) = result {
                continue;
            };
            break result;
        }
    }


    pub fn get_host_address(&mut self, host : &str) -> Result<i8, erpc::Err<()>> {
        //TODO : not working
        const LWIP_DNS_ADDRTYPE_IPV4 : u8 = 0;
        let mut serveraddr = erpc::IpAddrType{
            addr : [0u32;4],
            t : 0 ,
            len : 0,
        };
        
        let mut hostname = heapless::Vec::new();
        hostname.extend_from_slice(host.as_bytes()).ok();
        let mut c = heapless::Vec::new();
        let arg = 1u32;
        c.extend_from_slice(&arg.to_le_bytes()).ok();
        c.extend_from_slice(&arg.to_le_bytes()).ok();
        let callback = Some(c);
        let dns_addrtype = LWIP_DNS_ADDRTYPE_IPV4;

        let ret = self.blocking_rpc(rpcs::GethostbynameAddrtype{
            hostname: hostname,
            ip_addr: &mut serveraddr,
            found: 1u32,
            callback_arg : callback,
            dns_addrtype : dns_addrtype,
        });
        ret
    }

    pub fn connect(&mut self, ip_addr : u32, port : u16, timeout :i64) -> Result<i32, erpc::Err<()>> {

        let saddr = erpc::InAddr{s_addr : ip_addr};
        let serveraddr = erpc::SockaddrIn {
            sin_len : 0,
            sin_family : AF_INET as u8,
            sin_addr : saddr,
            sin_port : port,
            sin_zero : [0i8; 8], 
        };
        // create socket 
        let sock = self.blocking_rpc(rpcs::Socket{
            domain : AF_INET as i32,
            t : SOCK_STREAM,
            protocol : 0,
        }).unwrap();

        let val = self.blocking_rpc(rpcs::Fcntl{
            s : sock,
            cmd : F_GETFL,
            val: 0,
        }).unwrap();
        self.blocking_rpc(rpcs::Fcntl{
            s : sock,
            cmd : F_SETFL,
            val : val | O_NON_BLOCK,
        }).unwrap();
        let l = core::mem::size_of::<erpc::SockaddrIn>() as u32;

        //connect
        self.blocking_rpc(rpcs::Connect{
            s : sock,
            name : serveraddr,
            namelen: l,
        }).unwrap();

        //connected set socket
        self.sock_fd = Some(sock);

        //select
        let mut writeset = erpc::FdSet::new();
        writeset.set(sock as usize);
        let time = Some(erpc::TimeVal{
            tv_sec : 0,
            tv_usec: timeout,
        });
        self.blocking_rpc(rpcs::Select{
            s : sock+1,
            readset : None,
            writeset: Some(writeset),
            exceptset: None,
            timeval: time,
        })
    }

    pub fn send(&mut self, msg : &str) -> Result<i32, erpc::Err<()>> {
        let sock = self.sock_fd.unwrap();
        let mut data= heapless::Vec::new();
        let flag = MSG_DONTWAIT;
        let b: &[u8] = msg.as_bytes();
        data.extend_from_slice(&b).ok();
        self.blocking_rpc(rpcs::Send{
            s: sock,
            data: data,
            flag: flag,
        })
    }

    pub fn recv(&mut self) -> Result<heapless::Vec<u8, U512>, erpc::Err<()>> {
        let sock = self.sock_fd.unwrap();
        let flag = 0i32;
        let mut data = heapless::Vec::new();
        let res = self.blocking_rpc(rpcs::Recv{
            s: sock,
            len: 512,
            timeout : 10000*1000,
            mem: &mut data,
            flag: flag,
        });
        let mut ret = Ok(data);
        match res{
            Ok(r) => {
                if r<0{
                    ret = Err(erpc::Err::NotOurs);
                }
            },
            Err(_) => {
                ret = Err(erpc::Err::NotOurs);
            },
        }
        ret
    }

    pub fn close(&mut self) -> Result<i32, erpc::Err<()>> {
        let sock = self.sock_fd.unwrap();

        let ret = self.blocking_rpc(rpcs::Close{
            s: sock,
        });
        
        //connected set socket
        self.sock_fd = None;

        ret
    }

    fn recieve_rpc_response<'a, RPC: erpc::RPC>(
        &mut self,
        rpc: &mut RPC,
    ) -> Result<RPC::ReturnValue, erpc::Err<RPC::Error>> {
        let fh = self.recieve_frame_header(rpc)?; // Read the frame header

        // Read the payload, check CRC, hand off to underlying trait to decode
        let mut buffer = [0u8; 2048];
        let sz = fh.msg_length as usize;
        self.recieve_bytes(&mut buffer[..sz]);

        fh.check_crc(&buffer[..sz])?;
        rpc.parse(&buffer[..sz])
    }

    fn recieve_frame_header<'a, RPC: erpc::RPC>(
        &mut self,
        _rpc: &mut RPC,
    ) -> Result<erpc::FrameHeader, erpc::Err<RPC::Error>> {
        let mut buffer = [0u8; 4];
        self.recieve_bytes(&mut buffer);

        match erpc::FrameHeader::parse(&buffer[..]) {
            Err(e) => Err(erpc::Err::Parsing(e)),
            Ok(fh) => Ok(fh.1),
        }
    }

    fn recieve_bytes(&mut self, mut buffer: &mut [u8]) {
        while buffer.len() > 0 {
            let r = match self.rx_buff_input.read() {
                Ok(r) => r,
                Err(_) => {
                    continue;
                }
            };
            let b = r.buf();
            let copy_amt = if b.len() < buffer.len() {
                b.len()
            } else {
                buffer.len()
            };

            for (i, b) in b[..copy_amt].iter().enumerate() {
                buffer[i] = *b;
            }
            buffer = &mut buffer[copy_amt..];

            r.release(copy_amt);
        }
    }

    fn write_frame(&mut self, msg: &heapless::Vec<u8, U64>) -> Result<(), ()> {
        let header = erpc::FrameHeader::new_from_msg(msg);
        self.tx(header.as_bytes().iter().chain(msg));
        Ok(())
    }

    fn tx<'a, D: Iterator<Item = &'a u8>>(&mut self, data: D) {
        for b in data {
            if let Ok(mut wgr) = self.tx_buff_input.grant_exact(1) {
                wgr[0] = *b;
                wgr.commit(1);
            }
            self.uart.enable_interrupts(uart::Flags::DRE);
        }
    }

    fn next_seq(&mut self) -> u32 {
        self.sequence += 1;
        self.sequence
    }

}

/// Imports necessary for using `wifi_singleton`.
pub mod wifi_prelude {
    pub use crate::wifi::*;
    pub use atsamd_hal::gpio::Port;
    pub use atsamd_hal::pac::SERCOM0;
    pub use atsamd_hal::pac::{interrupt, MCLK};
    pub use atsamd_hal::sercom::{Sercom0Pad0, Sercom0Pad2, UART0};
    pub use bbqueue::{BBBuffer, Producer};

    pub use cortex_m::interrupt::CriticalSection;
}

/// Declares static globals for the wifi controller, and wires up interrupts.
#[macro_export]
macro_rules! wifi_singleton {
    ($global_name:ident) => {
        static mut $global_name: Option<Wifi> = None;
        static WIFI_RX: BBBuffer<512> = BBBuffer::new();
        static WIFI_TX: BBBuffer<128> = BBBuffer::new();

        /// Initializes the wifi controller from within an interrupt-free context.
        unsafe fn wifi_init(
            _cs: &CriticalSection,
            pins: WifiPins,
            sercom0: SERCOM0,
            clocks: &mut GenericClockController,
            mclk: &mut MCLK,
            delay: &mut Delay,
        ) {
            unsafe {
                $global_name = Some(Wifi::init(
                    pins, sercom0, clocks, mclk, delay, &WIFI_RX, &WIFI_TX,
                ));
            }
        }

        #[interrupt]
        fn SERCOM0_0() {
            // Data Register Empty interrupt.
            unsafe {
                $global_name.as_mut().map(|wifi| {
                    wifi._handle_data_empty();
                });
            }
        }

        #[interrupt]
        fn SERCOM0_2() {
            // Recieve Complete interrupt.
            unsafe {
                $global_name.as_mut().map(|wifi| {
                    wifi._handle_rx();
                });
            }
        }
    };
}
