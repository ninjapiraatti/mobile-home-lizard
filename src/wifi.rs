extern crate alloc;

use alloc::string::String;
use esp_hal::time::{Duration, Instant};
use esp_println::println;
use esp_wifi::wifi::{
    ClientConfiguration, Configuration, WifiController, WifiDevice,
};
use smoltcp::{
    iface::{Config, Interface, SocketSet, SocketStorage},
    socket::{dhcpv4, tcp},
    time::Instant as SmoltcpInstant,
    wire::{EthernetAddress, HardwareAddress, IpAddress},
};
use embedded_nal::TcpClientStack;
use smoltcp_nal::NetworkStack;

use crate::network_clock::EspClock;

/// Wraps `NetworkStack` and calls `poll()` before every `TcpClientStack` operation.
pub struct PolledStack<'a, D, C>(pub NetworkStack<'a, D, C>)
where
    D: smoltcp::phy::Device,
    C: embedded_time::Clock,
    u32: From<C::T>;

impl<'a, D, C> TcpClientStack for PolledStack<'a, D, C>
where
    NetworkStack<'a, D, C>: TcpClientStack,
    D: smoltcp::phy::Device,
    C: embedded_time::Clock,
    u32: From<C::T>,
{
    type TcpSocket = <NetworkStack<'a, D, C> as TcpClientStack>::TcpSocket;
    type Error = <NetworkStack<'a, D, C> as TcpClientStack>::Error;

    fn socket(&mut self) -> Result<Self::TcpSocket, Self::Error> {
        self.0.poll().ok();
        self.0.socket()
    }

    fn connect(
        &mut self,
        socket: &mut Self::TcpSocket,
        remote: core::net::SocketAddr,
    ) -> embedded_nal::nb::Result<(), Self::Error> {
        self.0.poll().ok();
        self.0.connect(socket, remote)
    }

    fn send(
        &mut self,
        socket: &mut Self::TcpSocket,
        buffer: &[u8],
    ) -> embedded_nal::nb::Result<usize, Self::Error> {
        self.0.poll().ok();
        self.0.send(socket, buffer)
    }

    fn receive(
        &mut self,
        socket: &mut Self::TcpSocket,
        buffer: &mut [u8],
    ) -> embedded_nal::nb::Result<usize, Self::Error> {
        self.0.poll().ok();
        self.0.receive(socket, buffer)
    }

    fn close(&mut self, socket: Self::TcpSocket) -> Result<(), Self::Error> {
        self.0.poll().ok();
        self.0.close(socket)
    }
}

// Static socket buffers
static mut TCP_RX_0: [u8; 1536] = [0u8; 1536];
static mut TCP_TX_0: [u8; 1536] = [0u8; 1536];
static mut TCP_RX_1: [u8; 1536] = [0u8; 1536];
static mut TCP_TX_1: [u8; 1536] = [0u8; 1536];
static mut TCP_RX_2: [u8; 1536] = [0u8; 1536];
static mut TCP_TX_2: [u8; 1536] = [0u8; 1536];

static mut SOCKET_STORAGE: [SocketStorage<'static>; 4] = [
    SocketStorage::EMPTY,
    SocketStorage::EMPTY,
    SocketStorage::EMPTY,
    SocketStorage::EMPTY,
];

fn smoltcp_now() -> SmoltcpInstant {
    let us = esp_hal::time::Instant::now()
        .duration_since_epoch()
        .as_micros();
    SmoltcpInstant::from_micros(us as i64)
}

/// Connect to WiFi and wait for a DHCP address.
pub fn connect<'d>(
    controller: &mut WifiController<'d>,
    device: WifiDevice<'d>,
    ssid: &str,
    password: &str,
) -> PolledStack<'static, WifiDevice<'d>, EspClock> {
    // 1. Configure STA mode
    let client_config = Configuration::Client(ClientConfiguration {
        ssid: String::from(ssid).try_into().unwrap(),
        password: String::from(password).try_into().unwrap(),
        ..Default::default()
    });
    controller.set_configuration(&client_config).expect("WiFi config failed");

    // 2. Start radio and connect to AP
    controller.start().expect("WiFi start failed");
    println!("WiFi started, connecting...");

    loop {
        match controller.connect() {
            Ok(_) => break,
            Err(e) => {
                println!("WiFi connect error: {:?}, retrying...", e);
                let t = Instant::now();
                while t.elapsed() < Duration::from_millis(1000) {}
            }
        }
    }

    // 3. Wait for association
    println!("Waiting for WiFi association...");
    loop {
        if matches!(controller.is_connected(), Ok(true)) {
            println!("WiFi connected!");
            break;
        }
        let t = Instant::now();
        while t.elapsed() < Duration::from_millis(100) {}
    }

    // 4. Build smoltcp Interface from the STA device
    let mut device = device;
    let mac = device.mac_address();
    let iface = Interface::new(
        Config::new(HardwareAddress::Ethernet(EthernetAddress(mac))),
        &mut device,
        smoltcp_now(),
    );

    // 5. Build SocketSet with static storage
    let stack = unsafe {
        let mut socket_set = SocketSet::new(&mut SOCKET_STORAGE[..]);

        socket_set.add(dhcpv4::Socket::new());

        socket_set.add(tcp::Socket::new(
            tcp::SocketBuffer::new(&mut TCP_RX_0[..]),
            tcp::SocketBuffer::new(&mut TCP_TX_0[..]),
        ));
        socket_set.add(tcp::Socket::new(
            tcp::SocketBuffer::new(&mut TCP_RX_1[..]),
            tcp::SocketBuffer::new(&mut TCP_TX_1[..]),
        ));
        socket_set.add(tcp::Socket::new(
            tcp::SocketBuffer::new(&mut TCP_RX_2[..]),
            tcp::SocketBuffer::new(&mut TCP_TX_2[..]),
        ));

        NetworkStack::new(iface, device, socket_set, EspClock)
    };

    // 6. Poll until DHCP gives us an IP address
    PolledStack(wait_for_dhcp(stack))
}

fn wait_for_dhcp<'d>(
    mut stack: NetworkStack<'static, WifiDevice<'d>, EspClock>,
) -> NetworkStack<'static, WifiDevice<'d>, EspClock> {
    println!("Waiting for DHCP...");
    loop {
        stack.poll().ok();

        let has_ip = stack.interface().ip_addrs().iter().any(|cidr: &smoltcp::wire::IpCidr| {
            !matches!(cidr.address(), IpAddress::Ipv4(a) if a.is_unspecified())
        });
        if has_ip {
            for cidr in stack.interface().ip_addrs() {
                println!("IP assigned: {}", cidr);
            }
            break;
        }

        let t = Instant::now();
        while t.elapsed() < Duration::from_millis(10) {}
    }
    stack
}
