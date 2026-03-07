#![no_std]
#![no_main]

use core::cell::RefCell;
use critical_section::Mutex;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;
use esp_hal::main;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{clock::CpuClock, Blocking};
use esp_hal::{
    delay::Delay,
    gpio::{Event, Input, InputConfig, Io, Level, Output, OutputConfig, Pull},
    handler,
    i2c::master::{Config, I2c},
    ram,
    rmt::Rmt,
    time::{Instant, Rate},
};
use esp_hal_smartled::{smart_led_buffer, SmartLedsAdapter};
use esp_println::println;
use rotary_encoder_hal::{Direction, Rotary};
use smart_leds::{
    brightness, gamma,
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite,
};
use ssd1306::{prelude::*, mode::DisplayConfig, I2CDisplayInterface, Ssd1306};
use display_interface::WriteOnlyDataCommand;

struct MenuItem<'a> {
    text: &'a str,
    value: Option<i32>,
}

struct Menu<'a> {
    items: &'a [MenuItem<'a>],
    selected_index: usize,
    scroll_offset: usize,
    max_visible_items: usize,
}

impl<'a> Menu<'a> {
    fn new(items: &'a [MenuItem<'a>], max_visible_items: usize) -> Self {
        Menu {
            items,
            selected_index: 0,
            scroll_offset: 0,
            max_visible_items,
        }
    }

    fn navigate(&mut self, direction: Direction) {
        match direction {
            Direction::Clockwise => {
                if self.selected_index < self.items.len() - 1 {
                    self.selected_index += 1;
                    if self.selected_index >= self.scroll_offset + self.max_visible_items {
                        self.scroll_offset += 1;
                    }
                }
            }
            Direction::CounterClockwise => {
                if self.selected_index > 0 {
                    self.selected_index -= 1;
                    if self.selected_index < self.scroll_offset {
                        self.scroll_offset -= 1;
                    }
                }
            }
            Direction::None => {}
        }
    }

    fn get_selected_item(&self) -> Option<&MenuItem<'a>> {
        self.items.get(self.selected_index)
    }

    fn render<DI>(&self, display: &mut Ssd1306<DI, DisplaySize128x32, ssd1306::mode::BufferedGraphicsMode<DisplaySize128x32>>)
    where
        DI: WriteOnlyDataCommand,
    {
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();

        if let Err(_e) = display.clear(BinaryColor::Off) {
            println!("Failed to clear display");
        }

        let visible_items = self
            .items
            .iter()
            .skip(self.scroll_offset)
            .take(self.max_visible_items);

        const PADDING: i32 = 6; // One letter width (FONT_6X10)

        for (i, item) in visible_items.enumerate() {
            let y_position = (i as i32) * 10 + 2; // 10 pixels per row, 2 pixel padding
            let is_selected = self.scroll_offset + i == self.selected_index;

            if is_selected {
                Text::with_baseline(">", Point::new(PADDING, y_position), text_style, Baseline::Top)
                    .draw(display)
                    .unwrap();
            }

            Text::with_baseline(
                item.text,
                Point::new(PADDING + 8, y_position),
                text_style,
                Baseline::Top,
            )
            .draw(display)
            .unwrap();

            if let Some(value) = item.value {
                let value_text = alloc::format!("{}", value);
                Text::with_baseline(
                    &value_text,
                    Point::new(96, y_position),
                    text_style,
                    Baseline::Top,
                )
                .draw(display)
                .unwrap();
            }
        }

        if self.scroll_offset > 0 {
            Text::with_baseline("^", Point::new(122 - PADDING, 0), text_style, Baseline::Top)
                .draw(display)
                .unwrap();
        }
        if self.scroll_offset + self.max_visible_items < self.items.len() {
            Text::with_baseline("v", Point::new(122 - PADDING, 24), text_style, Baseline::Top)
                .draw(display)
                .unwrap();
        }
        Delay::new().delay_millis(20);
        display.flush().unwrap();
    }
}

fn blink_leds(led1: &mut Output, led2: &mut Output) {
    let delay = Delay::new();
    for _ in 0..4 {
        led1.set_high();
        led2.set_low();
        delay.delay_millis(150);
        led1.set_low();
        led2.set_high();
        delay.delay_millis(150);
    }
    led1.set_low();
    led2.set_low();
}

// Rotary encoder button (GPIO20)
static BUTTON: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));
static BUTTON_PRESSED: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

// Separate button for LED control (GPIO9)
static LED_BUTTON: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));
static LED_BUTTON_PRESSED: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

esp_bootloader_esp_idf::esp_app_desc!();
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
    let input_config = InputConfig::default().with_pull(Pull::Up);
    io.set_interrupt_handler(interrupt_handler);

    // Rotary encoder button (GPIO20)
    let mut button = Input::new(peripherals.GPIO20, input_config);
    critical_section::with(|cs| {
        button.listen(Event::LowLevel);
        BUTTON.borrow_ref_mut(cs).replace(button)
    });

    // LED control button (GPIO9)
    let mut led_button = Input::new(peripherals.GPIO9, input_config);
    critical_section::with(|cs| {
        led_button.listen(Event::LowLevel);
        LED_BUTTON.borrow_ref_mut(cs).replace(led_button)
    });
    println!("Did interrupt stuff");

    // Rotary encoder
    let clk = Input::new(peripherals.GPIO22, input_config);
    let dt = Input::new(peripherals.GPIO21, input_config);
    let mut encoder = Rotary::new(clk, dt);
    println!("Did rotary encoder stuff");

    // Two LEDs for button-triggered blinking
    let mut led1 = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default());
    let mut led2 = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());
    println!("Did LED setup");

    // RGB LED stuff
    let led_pin = peripherals.GPIO8;
    let led_freq = Rate::from_mhz(80);
    let rmt = Rmt::new(peripherals.RMT, led_freq).unwrap();
    let rmt_buffer = smart_led_buffer!(1);
    let mut led = SmartLedsAdapter::new(rmt.channel0, led_pin, rmt_buffer);
    let mut color = Hsv {
        hue: 0,
        sat: 255,
        val: 255,
    };
    let mut data;
    println!("Did RGB LED stuff");

    // OLED stuff
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

    // Initialize the OLED display once
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    println!("Did OLED display stuff");

    // From the template
    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let _init = esp_wifi::init(timg0.timer0, esp_hal::rng::Rng::new(peripherals.RNG)).unwrap();
    println!("Did stuff from the template");

    let mut pos: isize = 0;

    // Menu system initialization
    let menu_items = [
        MenuItem {
            text: "Item 1",
            value: Some(1),
        },
        MenuItem {
            text: "Item 2",
            value: Some(2),
        },
        MenuItem {
            text: "Item 3",
            value: Some(3),
        },
        MenuItem {
            text: "Item 4",
            value: Some(4),
        },
        MenuItem {
            text: "Item 5",
            value: Some(5),
        },
        MenuItem {
            text: "Item 6",
            value: Some(6),
        },
        MenuItem {
            text: "Item 7",
            value: Some(7),
        },
    ];
    let mut menu = Menu::new(&menu_items, 4);

    // Debounce tracking for rotary encoder
    let mut last_encoder_time = Instant::now();
    const ENCODER_DEBOUNCE_MS: u64 = 80;

    loop {
        println!("Loop");
        color.hue = pos as u8;
        data = [hsv2rgb(color)];
        led.write(brightness(gamma(data.iter().cloned()), 10))
            .unwrap();

        // Check rotary encoder button
        let rotary_button_pressed = critical_section::with(|cs| {
            let was_pressed = *BUTTON_PRESSED.borrow_ref(cs);
            if was_pressed {
                *BUTTON_PRESSED.borrow_ref_mut(cs) = false;
            }
            was_pressed
        });

        if rotary_button_pressed {
            println!("Rotary button pressed");
        }

        // Check LED control button (GPIO9)
        let led_button_pressed = critical_section::with(|cs| {
            let was_pressed = *LED_BUTTON_PRESSED.borrow_ref(cs);
            if was_pressed {
                *LED_BUTTON_PRESSED.borrow_ref_mut(cs) = false;
            }
            was_pressed
        });

        if led_button_pressed {
            println!("LED button pressed");
            blink_leds(&mut led1, &mut led2);
        }

        // Debounced encoder handling
        let direction = encoder.update().unwrap();
        if direction != Direction::None {
            let now = Instant::now();
            let elapsed = now.duration_since_epoch().as_millis() - last_encoder_time.duration_since_epoch().as_millis();
            if elapsed > ENCODER_DEBOUNCE_MS {
                menu.navigate(direction);
                last_encoder_time = now;
            }
        }

        menu.render(&mut display);

        // Update hue for RGB LED color cycling
        pos = pos.wrapping_add(1);
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.0/examples/src/bin
}

#[handler]
#[ram]
fn interrupt_handler() {
    // Check rotary encoder button (GPIO20)
    critical_section::with(|cs| {
        if let Some(button) = BUTTON.borrow_ref_mut(cs).as_mut() {
            if button.is_interrupt_set() {
                if button.is_low() {
                    *BUTTON_PRESSED.borrow_ref_mut(cs) = true;
                }
                button.clear_interrupt();
            }
        }
    });

    // Check LED control button (GPIO9)
    critical_section::with(|cs| {
        if let Some(button) = LED_BUTTON.borrow_ref_mut(cs).as_mut() {
            if button.is_interrupt_set() {
                if button.is_low() {
                    *LED_BUTTON_PRESSED.borrow_ref_mut(cs) = true;
                }
                button.clear_interrupt();
            }
        }
    });
}
