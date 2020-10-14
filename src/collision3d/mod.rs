pub mod shapes;
pub use shapes::*;

pub mod collision_resolution;
pub use collision_resolution::*;

pub mod ray;
pub use ray::*;

use nalgebra::*;
use num_traits::*;

use std::cmp::Ordering;

pub trait PhysicsScalar:
    Scalar
    + Float
    + Pow<Self, Output = Self>
    + One
    + SimdComplexField<SimdRealField = Self>
    + RealField
    + FromPrimitive
    + ToPrimitive
{
}

impl PhysicsScalar for f32 {}
impl PhysicsScalar for f64 {}

#[inline(always)]
pub fn get_x<N: PhysicsScalar>(v: &Vector3<N>) -> N {
    unsafe { *v.get_unchecked(0) }
}

#[inline(always)]
pub fn get_y<N: PhysicsScalar>(v: &Vector3<N>) -> N {
    unsafe { *v.get_unchecked(1) }
}

#[inline(always)]
pub fn get_z<N: PhysicsScalar>(v: &Vector3<N>) -> N {
    unsafe { *v.get_unchecked(2) }
}

#[inline(always)]
pub fn set_x<N: PhysicsScalar>(v: &mut Vector3<N>, value: N) {
    unsafe { *v.get_unchecked_mut(0) = value }
}

#[inline(always)]
pub fn set_y<N: PhysicsScalar>(v: &mut Vector3<N>, value: N) {
    unsafe { *v.get_unchecked_mut(1) = value }
}

#[inline(always)]
pub fn set_z<N: PhysicsScalar>(v: &mut Vector3<N>, value: N) {
    unsafe { *v.get_unchecked_mut(2) = value }
}

pub fn n_ordering<N: PhysicsScalar>(a: N, b: N) -> Ordering {
    match a.partial_cmp(&b) {
        Some(o) => o,
        None => Ordering::Equal,
    }
}

#[inline(always)]
pub fn n_min<N: PhysicsScalar>(a: N, b: N) -> N {
    if a < b {
        a
    } else {
        b
    }
}

#[inline(always)]
pub fn n_max<N: PhysicsScalar>(a: N, b: N) -> N {
    if a > b {
        a
    } else {
        b
    }
}

#[inline(always)]
pub fn clamp<N: PhysicsScalar>(value: N, min: N, max: N) -> N {
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
pub fn is_zero<N: PhysicsScalar>(a: N) -> bool {
    Float::abs(a) < N::epsilon()
}

#[inline(always)]
pub fn min_component<N: PhysicsScalar>(v: &Vector3<N>) -> (usize, &N) {
    v.iter()
        .enumerate()
        .min_by(|(_, &f1), (_, &f2)| n_ordering(f1, f2))
        .unwrap()
}

#[inline(always)]
pub fn max_component<N: PhysicsScalar>(v: &Vector3<N>) -> (usize, &N) {
    v.iter()
        .enumerate()
        .max_by(|(_, &f1), (_, &f2)| n_ordering(f1, f2))
        .unwrap()
}

#[inline(always)]
pub fn closest_to_segment<N: PhysicsScalar>(
    start: &Vector3<N>,
    end: &Vector3<N>,
    point: &Vector3<N>,
) -> Vector3<N> {
    let ab: Vector3<N> = end - start;
    let t: N = (point - start).dot(&ab) / ab.dot(&ab);
    start + ab * n_min(n_max(t, N::zero()), N::one())
}
