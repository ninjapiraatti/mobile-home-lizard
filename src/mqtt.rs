use core::net::{IpAddr, Ipv4Addr};

use embedded_nal::TcpClientStack;
use embedded_time::Clock;
use heapless::String;
use minimq::{
    broker::{Broker, IpBroker},
    mqtt_client::MqttClient,
    types::TopicFilter,
    ConfigBuilder, Minimq, Publication, QoS, Will,
};

use esp_println::println;

use crate::{config, discovery, network_clock::EspClock};

/// Publish a light command (ON or OFF) to the MQTT broker.
pub fn publish_light_command<S, Clk, B>(client: &mut MqttClient<'_, S, Clk, B>, on: bool)
where
    S: TcpClientStack,
    Clk: Clock,
    B: Broker,
{
    let mut topic: String<64> = String::new();
    let _ = topic.push_str(discovery::BASE_TOPIC);
    let _ = topic.push_str("/switch/lights/command");

    let payload = if on { b"ON".as_ref() } else { b"OFF".as_ref() };
    let _ = client.publish(
        Publication::new(topic.as_str(), payload)
            .retain()
            .qos(QoS::AtMostOnce),
    );
    println!("Published light command: {}", if on { "ON" } else { "OFF" });
}

/// Enter the MQTT poll loop — never returns.
///
/// Call this after WiFi + DHCP are established.
/// `on_command(on)` is called whenever HA sends a light command.
/// `on_connect_change(connected)` is called whenever the MQTT connection state changes.
/// `poll_button()` returns Some(true) for lights on, Some(false) for lights off, None for no action.
pub fn run<S, F, G, H>(network: S, mut on_command: F, mut on_connect_change: G, mut poll_button: H) -> !
where
    S: TcpClientStack,
    F: FnMut(bool),
    G: FnMut(bool),
    H: FnMut() -> Option<bool>,
{
    let broker = IpBroker::new(IpAddr::V4(Ipv4Addr::new(
        config::MQTT_BROKER_IP[0],
        config::MQTT_BROKER_IP[1],
        config::MQTT_BROKER_IP[2],
        config::MQTT_BROKER_IP[3],
    )));

    let mut mqtt_buf = [0u8; 2048];

    let mut will_topic: String<64> = String::new();
    let _ = will_topic.push_str(discovery::BASE_TOPIC);
    let _ = will_topic.push_str("/status");

    let mut mqtt: Minimq<'_, S, EspClock, IpBroker> = Minimq::new(
        network,
        EspClock,
        ConfigBuilder::new(broker, &mut mqtt_buf)
            .client_id(config::MQTT_CLIENT_ID)
            .expect("client ID too long")
            .set_auth(config::MQTT_USERNAME, config::MQTT_PASSWORD)
            .expect("auth config failed")
            .keepalive_interval(30)
            .will(Will::new(will_topic.as_str(), b"offline", &[]).expect("will failed"))
            .expect("will config failed"),
    );

    let mut subscribed = false;
    let mut poll_errors: u32 = 0;
    let mut was_connected = false;

    let ip = config::MQTT_BROKER_IP;
    println!(
        "MQTT loop starting, broker {}.{}.{}.{}:{}",
        ip[0], ip[1], ip[2], ip[3], config::MQTT_BROKER_PORT
    );

    loop {
        // 1. Drive the MQTT state machine; dispatch incoming messages
        if let Err(e) = mqtt.poll(|client, topic, payload, _props| {
            let prefix = {
                let mut p: String<64> = String::new();
                let _ = p.push_str(discovery::BASE_TOPIC);
                let _ = p.push_str("/switch/");
                p
            };

            if let Some(rest) = topic.strip_prefix(prefix.as_str()) {
                if let Some(_switch_id) = rest.strip_suffix("/command") {
                    let on = payload == b"ON";
                    on_command(on);

                    // Echo state back to HA
                    let mut state_topic: String<64> = String::new();
                    let _ = state_topic.push_str(discovery::BASE_TOPIC);
                    let _ = state_topic.push_str("/switch/lights/state");

                    let _ = client.publish(
                        Publication::new(
                            state_topic.as_str(),
                            if on { b"ON".as_ref() } else { b"OFF".as_ref() },
                        )
                        .retain()
                        .qos(QoS::AtMostOnce),
                    );
                }
            }

            None::<()>
        }) {
            poll_errors += 1;
            if poll_errors <= 5 || poll_errors % 100 == 0 {
                println!("MQTT poll error #{}: {:?}", poll_errors, e);
            }
        }

        // 2. Notify on connection state change
        {
            let now_connected = mqtt.client().is_connected();
            if now_connected != was_connected {
                was_connected = now_connected;
                on_connect_change(now_connected);
            }
        }

        // 3. Publish light command when button pressed
        if let Some(lights_on) = poll_button() {
            publish_light_command(mqtt.client(), lights_on);
        }

        // 4. Subscribe and publish discovery once connected
        {
            let client = mqtt.client();

            if client.is_connected() {
                if !subscribed {
                    println!("MQTT connected, publishing discovery...");
                    let mut cmd_filter: String<64> = String::new();
                    let _ = cmd_filter.push_str(discovery::BASE_TOPIC);
                    let _ = cmd_filter.push_str("/switch/+/command");

                    let _ = client.subscribe(
                        &[TopicFilter::new(cmd_filter.as_str())],
                        &[],
                    );

                    publish_discovery(client);

                    let mut status_topic: String<64> = String::new();
                    let _ = status_topic.push_str(discovery::BASE_TOPIC);
                    let _ = status_topic.push_str("/status");

                    let _ = client.publish(
                        Publication::new(status_topic.as_str(), b"online")
                            .retain()
                            .qos(QoS::AtMostOnce),
                    );

                    subscribed = true;
                }
            } else {
                subscribed = false;
            }
        }

        // Small yield to avoid spinning at 100% CPU
        let t = esp_hal::time::Instant::now();
        while t.elapsed() < esp_hal::time::Duration::from_millis(10) {}
    }
}

fn publish_discovery<S, Clk, B>(client: &mut MqttClient<'_, S, Clk, B>)
where
    S: TcpClientStack,
    Clk: Clock,
    B: Broker,
{
    let t = discovery::discovery_topic_switch("lights");
    let p = discovery::switch_payload("lights", "Lights");
    let _ = client.publish(Publication::new(t.as_str(), p.as_bytes()).retain().qos(QoS::AtMostOnce));
}
