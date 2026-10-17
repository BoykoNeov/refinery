//! THROWAWAY M64.0 probe: the reboiler's duty held, the reflux an unknown.
use std::sync::Mutex;

static HELD_MW: Mutex<Option<f64>> = Mutex::new(None);
pub static LAST: Mutex<String> = Mutex::new(String::new());

/// Hold every heat-balance column's reboiler at `mw` [MW]; `None` restores the
/// shipped behaviour.
pub fn set_held(mw: Option<f64>) {
    *HELD_MW.lock().unwrap() = mw;
}

pub fn held() -> Option<f64> {
    *HELD_MW.lock().unwrap()
}

pub fn last() -> String {
    LAST.lock().unwrap().clone()
}
