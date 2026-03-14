use heapless::String;

use crate::config;

/// HA MQTT Discovery topic prefix.
const DISCOVERY_PREFIX: &str = "homeassistant";

/// Base topic used with the `~` abbreviation in payloads.
pub const BASE_TOPIC: &str = "mobile-home-lizard";

/// Generate discovery topic for a switch entity.
pub fn discovery_topic_switch(object_id: &str) -> String<128> {
    let mut s: String<128> = String::new();
    let _ = s.push_str(DISCOVERY_PREFIX);
    let _ = s.push_str("/switch/");
    let _ = s.push_str(config::HA_DEVICE_ID);
    let _ = s.push('/');
    let _ = s.push_str(object_id);
    let _ = s.push_str("/config");
    s
}

/// Generate discovery payload for a switch entity.
pub fn switch_payload(object_id: &str, display_name: &str) -> String<512> {
    let mut s: String<512> = String::new();
    let _ = s.push_str(r#"{"~":""#);
    let _ = s.push_str(BASE_TOPIC);
    let _ = s.push_str(r#"","name":""#);
    let _ = s.push_str(display_name);
    let _ = s.push_str(r#"","uniq_id":""#);
    let _ = s.push_str(config::HA_DEVICE_ID);
    let _ = s.push('_');
    let _ = s.push_str(object_id);
    let _ = s.push_str(r#"","stat_t":"~/switch/"#);
    let _ = s.push_str(object_id);
    let _ = s.push_str(r#"/state","cmd_t":"~/switch/"#);
    let _ = s.push_str(object_id);
    let _ = s.push_str(r#"/command","dev":{"ids":[""#);
    let _ = s.push_str(config::HA_DEVICE_ID);
    let _ = s.push_str(r#""],"name":""#);
    let _ = s.push_str(config::HA_DEVICE_NAME);
    let _ = s.push_str(r#""}}"#);
    s
}
