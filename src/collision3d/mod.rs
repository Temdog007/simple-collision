pub mod shapes;
pub use shapes::*;

pub mod collision_resolution;
pub use collision_resolution::*;

pub mod ray;
pub use ray::*;

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

#[inline(always)]
pub fn set_x(v: &mut Vector3<f32>, value: f32) {
    unsafe { *v.get_unchecked_mut(0) = value }
}

#[inline(always)]
pub fn set_y(v: &mut Vector3<f32>, value: f32) {
    unsafe { *v.get_unchecked_mut(1) = value }
}

#[inline(always)]
pub fn set_z(v: &mut Vector3<f32>, value: f32) {
    unsafe { *v.get_unchecked_mut(2) = value }
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

#[inline(always)]
pub fn min_component(v: &Vector3<f32>) -> (usize, &f32) {
    v.iter()
        .enumerate()
        .min_by(|(_, &f1), (_, &f2)| f32_ordering(f1, f2))
        .unwrap()
}

#[inline(always)]
pub fn max_component(v: &Vector3<f32>) -> (usize, &f32) {
    v.iter()
        .enumerate()
        .max_by(|(_, &f1), (_, &f2)| f32_ordering(f1, f2))
        .unwrap()
}

#[inline(always)]
pub fn closest_to_segment(
    start: &Vector3<f32>,
    end: &Vector3<f32>,
    point: &Vector3<f32>,
) -> Vector3<f32> {
    let ab = end - start;
    let t = (point - start).dot(&ab) / ab.dot(&ab);
    start + f32_min(f32_max(t, 0f32), 1f32) * ab
}
