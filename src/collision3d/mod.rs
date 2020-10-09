pub mod shapes;
pub use shapes::*;

use nalgebra::*;

use std::cmp::Ordering;

#[inline(always)]
pub fn from_f32(f: f32) -> Vector3<f32> {
    Vector3::new(f, f, f)
}

#[inline(always)]
pub fn get_x(v: &Vector3<f32>) -> f32 {
    unsafe { *v.get_unchecked(0) }
}

#[inline(always)]
pub fn get_y(v: &Vector3<f32>) -> f32 {
    unsafe { *v.get_unchecked(1) }
}

#[inline(always)]
pub fn get_z(v: &Vector3<f32>) -> f32 {
    unsafe { *v.get_unchecked(2) }
}

pub fn f32_ordering(a: f32, b: f32) -> Ordering {
    match a.partial_cmp(&b) {
        Some(o) => o,
        None => Ordering::Equal,
    }
}

#[inline(always)]
pub fn f32_min(a: f32, b: f32) -> f32 {
    if a < b {
        a
    } else {
        b
    }
}

#[inline(always)]
pub fn f32_max(a: f32, b: f32) -> f32 {
    if a > b {
        a
    } else {
        b
    }
}

#[inline(always)]
pub fn clamp(value: f32, min: f32, max: f32) -> f32 {
    assert!(min <= max);
    let mut x = value;
    if x < min {
        x = min;
    }
    if x > max {
        x = max;
    }
    x
}

#[inline(always)]
pub fn is_zero(a: f32) -> bool {
    a.abs() < std::f32::EPSILON
}
