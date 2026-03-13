#![no_std]
#![no_main]

use display_interface::WriteOnlyDataCommand;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use esp_hal::clock::CpuClock;
use esp_hal::main;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{
    delay::Delay,
    gpio::{Input, InputConfig, Io, Level, Output, OutputConfig, Pull},
    i2c::master::{Config, I2c},
    time::{Instant, Rate},
};
use esp_println::println;
use rotary_encoder_hal::{Direction, Rotary};
use ssd1306::{mode::DisplayConfig, prelude::*, I2CDisplayInterface, Ssd1306};

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

    fn render<DI>(
        &self,
        display: &mut Ssd1306<
            DI,
            DisplaySize128x32,
            ssd1306::mode::BufferedGraphicsMode<DisplaySize128x32>,
        >,
    ) where
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
                Text::with_baseline(
                    ">",
                    Point::new(PADDING, y_position),
                    text_style,
                    Baseline::Top,
                )
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
            Text::with_baseline(
                "v",
                Point::new(122 - PADDING, 24),
                text_style,
                Baseline::Top,
            )
            .draw(display)
            .unwrap();
        }
        Delay::new().delay_millis(20);
        display.flush().unwrap();
    }
}

/// Light the center LED (pin 6) for the specified duration in milliseconds
fn led_center(led: &mut Output, duration_ms: u32) {
    let delay = Delay::new();
    led.set_high();
    delay.delay_millis(duration_ms);
    led.set_low();
}

/// Light the middle LEDs (pins 5 and 7) for the specified duration in milliseconds
fn led_middle(led_left: &mut Output, led_right: &mut Output, duration_ms: u32) {
    let delay = Delay::new();
    led_left.set_high();
    led_right.set_high();
    delay.delay_millis(duration_ms);
    led_left.set_low();
    led_right.set_low();
}

/// Light the outer LEDs (pins 4 and 8) for the specified duration in milliseconds
fn led_outer(led_left: &mut Output, led_right: &mut Output, duration_ms: u32) {
    let delay = Delay::new();
    led_left.set_high();
    led_right.set_high();
    delay.delay_millis(duration_ms);
    led_left.set_low();
    led_right.set_low();
}

/// Creates a wave pulse effect: center -> middle -> outer LEDs
fn led_pulse(
    led_center_pin: &mut Output,
    led_middle_left: &mut Output,
    led_middle_right: &mut Output,
    led_outer_left: &mut Output,
    led_outer_right: &mut Output,
    duration_ms: u32,
) {
    led_center(led_center_pin, duration_ms);
    led_middle(led_middle_left, led_middle_right, duration_ms);
    led_outer(led_outer_left, led_outer_right, duration_ms);
}


#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

esp_bootloader_esp_idf::esp_app_desc!();
extern crate alloc;

#[main]
fn main() -> ! {
    // generator version: 0.3.1

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    //let peripherals = esp_hal::init(esp_hal::Config::default());
    let _io = Io::new(peripherals.IO_MUX);

    // Input config with pull-up for buttons/encoder
    let input_config = InputConfig::default().with_pull(Pull::Up);

    // Rotary encoder button (GPIO20) - polled
    let button = Input::new(peripherals.GPIO20, input_config);

    // Rotary encoder
    let clk = Input::new(peripherals.GPIO22, input_config);
    let dt = Input::new(peripherals.GPIO21, input_config);
    let mut encoder = Rotary::new(clk, dt);
    println!("Did rotary encoder stuff");

    // Five LEDs: outer(4), middle(5), center(6), middle(7), outer(8)
    let mut led_outer_left = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default());
    let mut led_middle_left = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());
    let mut led_center = Output::new(peripherals.GPIO6, Level::Low, OutputConfig::default());
    let mut led_middle_right = Output::new(peripherals.GPIO7, Level::Low, OutputConfig::default());
    let mut led_outer_right = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());
    println!("Did LED setup");

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

    // Debounce tracking
    let mut last_encoder_time = Instant::now();
    let mut last_button_time = Instant::now();
    const ENCODER_DEBOUNCE_MS: u64 = 80;
    const BUTTON_DEBOUNCE_MS: u64 = 200;
    let mut button_was_pressed = false;

    loop {
        println!("Loop");

        // Check rotary encoder button (polled with debounce)
        let button_is_low = button.is_low();
        if button_is_low && !button_was_pressed {
            let now = Instant::now();
            let elapsed = now.duration_since_epoch().as_millis()
                - last_button_time.duration_since_epoch().as_millis();
            if elapsed > BUTTON_DEBOUNCE_MS {
                println!("Rotary button pressed");
                led_pulse(
                    &mut led_center,
                    &mut led_middle_left,
                    &mut led_middle_right,
                    &mut led_outer_left,
                    &mut led_outer_right,
                    100,
                );
                last_button_time = now;
            }
        }
        button_was_pressed = button_is_low;

        // Debounced encoder handling
        let direction = encoder.update().unwrap();
        if direction != Direction::None {
            let now = Instant::now();
            let elapsed = now.duration_since_epoch().as_millis()
                - last_encoder_time.duration_since_epoch().as_millis();
            if elapsed > ENCODER_DEBOUNCE_MS {
                menu.navigate(direction);
                last_encoder_time = now;
            }
        }

        menu.render(&mut display);
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.0/examples/src/bin
}

