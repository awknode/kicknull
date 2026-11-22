//! Test-only: prints every parameter's default (plain) value as preset JSON,
//! used to build factory presets that set every control.
use crate::KickNullParams;
use nice_plug::prelude::*;

#[test]
fn dump_defaults() {
    let p = KickNullParams::default();
    let mut out = std::collections::BTreeMap::new();
    for (id, ptr, _) in p.param_map() {
        let v = unsafe {
            match ptr {
                ParamPtr::FloatParam(x) => serde_json::json!({ "f32": (*x).unmodulated_plain_value() }),
                ParamPtr::IntParam(x) => serde_json::json!({ "i32": (*x).unmodulated_plain_value() }),
                ParamPtr::BoolParam(x) => serde_json::json!({ "bool": (*x).unmodulated_plain_value() }),
                ParamPtr::EnumParam(x) => serde_json::json!({ "i32": (*x).unmodulated_plain_value() }),
            }
        };
        out.insert(id, v);
    }
    println!("DEFAULTS {}", serde_json::to_string(&out).unwrap());
}
