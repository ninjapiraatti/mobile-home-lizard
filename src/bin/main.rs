#![no_std]
#![no_main]

extern crate alloc;

use core::cell::{Cell, RefCell};
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

use mobile_home_lizard::{config, mqtt, wifi};

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

/// Display a status message on the OLED (up to 3 lines)
fn show_status<DI>(
    display: &mut Ssd1306<DI, DisplaySize128x32, ssd1306::mode::BufferedGraphicsMode<DisplaySize128x32>>,
    lines: &[&str],
) where
    DI: WriteOnlyDataCommand,
{
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    display.clear(BinaryColor::Off).ok();

    for (i, line) in lines.iter().take(3).enumerate() {
        let y_position = (i as i32) * 10 + 2;
        Text::with_baseline(line, Point::new(4, y_position), text_style, Baseline::Top)
            .draw(display)
            .unwrap();
    }

    display.flush().unwrap();
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

esp_bootloader_esp_idf::esp_app_desc!();

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

    // Power-on indicator
    led_pulse(
        &mut led_center,
        &mut led_middle_left,
        &mut led_middle_right,
        &mut led_outer_left,
        &mut led_outer_right,
        100,
    );

    // OLED stuff
    let i2c = I2c::new(
        peripherals.I2C0,
        Config::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO19)
    .with_scl(peripherals.GPIO18);
    // Initialize the OLED display
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    println!("OLED init complete");
    show_status(&mut display, &["OLED init OK"]);
    let display = RefCell::new(display);

    // From the template
    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let _init = esp_wifi::init(timg0.timer0, esp_hal::rng::Rng::new(peripherals.RNG)).unwrap();

    // Menu system initialization - Lights control
    let menu_items = [
        MenuItem {
            text: "Lights Off",
            value: None,
        },
        MenuItem {
            text: "Lights On",
            value: None,
        },
    ];
    let menu = RefCell::new(Menu::new(&menu_items, 3));

    // Connect to WiFi
    println!("Connecting to WiFi...");
    show_status(&mut *display.borrow_mut(), &["OLED init OK", "Connecting WiFi.."]);
    let (mut controller, interfaces) = esp_wifi::wifi::new(&_init, peripherals.WIFI).unwrap();
    let network = wifi::connect(
        &mut controller,
        interfaces.sta,
        config::WIFI_SSID,
        config::WIFI_PASSWORD,
    );
    println!("WiFi connected!");
    show_status(&mut *display.borrow_mut(), &["OLED init OK", "WiFi OK", "Connecting MQTT.."]);

    // State for the MQTT callbacks
    let mut last_button_time = Instant::now();
    const BUTTON_DEBOUNCE_MS: u64 = 50;
    const ENCODER_STEPS_PER_DETENT: i8 = 2;
    let mut button_was_pressed = false;
    let mut encoder_count: i8 = 0;
    let mut pending_command: Option<bool> = None;
    let mqtt_connected = Cell::new(false);
    let menu_shown = Cell::new(false);

    // Enter MQTT loop
    mqtt::run(
        network,
        // on_command: called when HA sends a light command
        |on| {
            println!("Received light command from HA: {}", if on { "ON" } else { "OFF" });
            // Could control actual lights/relays here
        },
        // on_connect_change: called when MQTT connection state changes
        |connected| {
            println!("MQTT connection: {}", if connected { "connected" } else { "disconnected" });
            mqtt_connected.set(connected);
            if connected && !menu_shown.get() {
                show_status(&mut *display.borrow_mut(), &["OLED init OK", "WiFi OK", "MQTT Connected"]);
                Delay::new().delay_millis(500);
                menu.borrow().render(&mut *display.borrow_mut());
                menu_shown.set(true);
            } else if !connected {
                menu_shown.set(false);
                show_status(&mut *display.borrow_mut(), &["MQTT Disconnected", "Reconnecting.."]);
            }
        },
        // poll_button: check encoder and button, return light command if button pressed
        || {
            // Only process input if MQTT is connected and menu is shown
            if !mqtt_connected.get() || !menu_shown.get() {
                return None;
            }

            // Handle encoder for menu navigation
            let direction = encoder.update().unwrap();
            match direction {
                Direction::Clockwise => encoder_count += 1,
                Direction::CounterClockwise => encoder_count -= 1,
                Direction::None => {}
            }

            let mut needs_render = false;
            if encoder_count >= ENCODER_STEPS_PER_DETENT {
                menu.borrow_mut().navigate(Direction::Clockwise);
                encoder_count = 0;
                needs_render = true;
            } else if encoder_count <= -ENCODER_STEPS_PER_DETENT {
                menu.borrow_mut().navigate(Direction::CounterClockwise);
                encoder_count = 0;
                needs_render = true;
            }

            if needs_render {
                menu.borrow().render(&mut *display.borrow_mut());
            }

            // Check button press
            let button_is_low = button.is_low();
            if button_is_low && !button_was_pressed {
                let now = Instant::now();
                let elapsed = now.duration_since_epoch().as_millis()
                    - last_button_time.duration_since_epoch().as_millis();
                if elapsed > BUTTON_DEBOUNCE_MS {
                    // Button pressed - determine action based on menu selection
                    let lights_on = menu.borrow().selected_index == 1; // "Lights On" is index 1
                    println!("Button pressed, sending: {}", if lights_on { "ON" } else { "OFF" });

                    led_pulse(
                        &mut led_center,
                        &mut led_middle_left,
                        &mut led_middle_right,
                        &mut led_outer_left,
                        &mut led_outer_right,
                        100,
                    );

                    last_button_time = now;
                    pending_command = Some(lights_on);
                }
            }
            button_was_pressed = button_is_low;

            // Return pending command and clear it
            pending_command.take()
        },
    );
}
