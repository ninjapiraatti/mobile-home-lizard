#![no_std]
#![no_main]

use embedded_hal::pwm::SetDutyCycle;
use embedded_hal::digital::OutputPin;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use core::cell::RefCell;
use critical_section::Mutex;
use esp_hal::clock::CpuClock;
use esp_hal::main;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use esp_hal::{
    delay::Delay,
    gpio::{Level, Input, InputConfig, Event, Output, Io, OutputConfig, Pull},
    time::Rate,
    rmt::Rmt,
    handler,
    ram,
    i2c::master::{I2c, Config},
};
use esp_hal_smartled::{smartLedBuffer, SmartLedsAdapter};
use smart_leds::{
    brightness, gamma,
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite,
};
use rotary_encoder_hal::{Direction, Rotary};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

static BUTTON: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));
// Flag to indicate a button press event occurred in the interrupt
static BUTTON_PRESSED: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

#[main]
fn main() -> ! {
    // generator version: 0.3.1

    let mut delay = Delay::new();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    //let peripherals = esp_hal::init(esp_hal::Config::default());
    let mut io = Io::new(peripherals.IO_MUX);
    
    // Interrupt stuff
    let config = InputConfig::default().with_pull(Pull::Up);
    let switch = peripherals.GPIO20;
    io.set_interrupt_handler(interrupt_handler);
    let mut button = Input::new(switch, config);
    critical_section::with(|cs| {
        button.listen(Event::LowLevel);
        BUTTON.borrow_ref_mut(cs).replace(button)
    });
    println!("Did interrupt stuff");

    // Rotary encoder
    let clk = Input::new(peripherals.GPIO22, config);
    let dt = Input::new(peripherals.GPIO21, config);
    let mut encoder = Rotary::new(clk, dt);
    println!("Did rotary encoder stuff");

    //let mut test_pin = Output::new(peripherals.GPIO7, Level::Low, OutputConfig::default());
    //let mut led_pin_a = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());
    
    // LED stuff
    let led_pin = peripherals.GPIO8;
    let led_freq = Rate::from_mhz(80);
    let rmt = Rmt::new(peripherals.RMT, led_freq).unwrap();
    let rmt_buffer = smartLedBuffer!(1);
    let mut led = SmartLedsAdapter::new(rmt.channel0, led_pin, rmt_buffer);
    let mut color = Hsv {
        hue: 0,
        sat: 255,
        val: 255,
    };
    let mut data;
    println!("Did RGB LED stuff");


    // OLED stuff
    let mut reset = Output::new(peripherals.GPIO13, Level::Low, OutputConfig::default());
    let mut i2c = I2c::new(
        peripherals.I2C0,
        Config::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO19)
    .with_scl(peripherals.GPIO18);
    println!("Did half of OLED stuff");
    println!("Starting I2C scan...");
    for addr in 0..=127 {
        let result = i2c.write(addr, &[0]);
        if result.is_ok() {
            println!("I2C device found at address: 0x{:02X}", addr);
        } else {
            println!("No device found");
        }
    }
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(
        interface,
        DisplaySize128x32,
        DisplayRotation::Rotate0,
    ).into_buffered_graphics_mode();
    //display.reset(&mut reset, &mut delay).unwrap();
    display.init().unwrap();
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Mobile home lizard", Point::new(0, 4), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    display.flush().unwrap();
    println!("Did OLED display stuff");

    // From the template
    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let _init = esp_wifi::init(
        timg0.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();
    println!("Did stuff from the template");

    let mut pos: isize = 0;

    loop {
        color.hue = pos as u8;
        data = [hsv2rgb(color)];
        led.write(brightness(gamma(data.iter().cloned()), 10)).unwrap();

        // Check if the button was pressed in the interrupt handler
        let button_was_pressed = critical_section::with(|cs| {
            let was_pressed = *BUTTON_PRESSED.borrow_ref(cs);
            if was_pressed {
                // Reset the flag after we've detected it
                *BUTTON_PRESSED.borrow_ref_mut(cs) = false;
            }
            was_pressed
        });

        if button_was_pressed {
            println!("Button pressed");
        }

        //println!("Pos: {:?}", pos);
        //println!("Button state: {:?}", button.is_interrupt_set());
        match encoder.update().unwrap() {
            Direction::Clockwise => {
                pos += 1;
                println!("Pos: {:?}", pos);
                display.clear(BinaryColor::Off).unwrap();
                Text::with_baseline("Mobile home lizard", Point::new(pos as i32, 4), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();
                display.flush().unwrap();
            }
            Direction::CounterClockwise => {
                pos -= 1;
                println!("Pos: {:?}", pos);
            }
            Direction::None => {}
        }
        // Show something in the OLED
        
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.0/examples/src/bin
}

#[handler]
#[ram]
fn interrupt_handler() {
    if critical_section::with(|cs| {
        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .is_interrupt_set()
    }) {
        let is_button_low = critical_section::with(|cs| {
            BUTTON
                .borrow_ref_mut(cs)
                .as_mut()
                .unwrap()
                .is_low()
        });

        if is_button_low == true {
            critical_section::with(|cs| {
                *BUTTON_PRESSED.borrow_ref_mut(cs) = true;
            });
        }
    } else {
        println!("Button NOT pressed");
    }

    critical_section::with(|cs| {
        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}