pub mod collision3d;

pub mod collision2d;

pub mod collision_resolution;
pub use collision_resolution::*;

use nalgebra::base::dimension::Dim;
use nalgebra::storage::Storage;
use nalgebra::*;
use num_traits::*;

use std::cmp::Ordering;

pub trait PhysicsScalar:
    Scalar
    + ClosedAdd
    + ClosedSub
    + ClosedMul
    + ClosedDiv
    + Signed
    + Copy
    + PartialEq
    + PartialOrd
    + Bounded + Signed
{
}

pub trait FloatingPhysicsScalar:
    PhysicsScalar
    + Float
    + Pow<Self, Output = Self>
    + One
    + SimdComplexField<SimdRealField = Self>
    + RealField
    + FromPrimitive
    + ToPrimitive
{
}

impl PhysicsScalar for i8 {}
impl PhysicsScalar for i16 {}
impl PhysicsScalar for i32 {}
impl PhysicsScalar for i64 {}
impl PhysicsScalar for i128 {}
impl PhysicsScalar for isize {}
impl PhysicsScalar for f32 {}
impl PhysicsScalar for f64 {}

impl FloatingPhysicsScalar for f32 {}
impl FloatingPhysicsScalar for f64 {}

#[inline(always)]
pub(crate) fn n_ordering<N: PhysicsScalar>(a: N, b: N) -> Ordering {
    match a.partial_cmp(&b) {
        Some(o) => o,
        None => Ordering::Equal,
    }
}

#[inline(always)]
pub(crate) fn n_min<N: PhysicsScalar>(a: N, b: N) -> N {
    if a < b {
        a
    } else {
        b
    }
}

#[inline(always)]
pub(crate) fn n_max<N: PhysicsScalar>(a: N, b: N) -> N {
    if a > b {
        a
    } else {
        b
    }
}

#[inline(always)]
pub(crate) fn n_min_iter<
    'a,
    D: Dim,
    S: Storage<N, D> + 'a,
    I: Iterator<Item = &'a Vector<N, D, S>>,
    N: PhysicsScalar,
>(
    iter: I,
    index: usize,
) -> N {
    iter.min_by(|a, b| unsafe {
        match a
            .vget_unchecked(index)
            .partial_cmp(&b.vget_unchecked(index))
        {
            Some(ord) => ord,
            None => std::cmp::Ordering::Equal,
        }
    })
    .map(|v| unsafe { *v.get_unchecked(index) })
    .unwrap()
}

#[inline(always)]
pub(crate) fn n_max_iter<
    'a,
    D: Dim,
    S: Storage<N, D> + 'a,
    I: Iterator<Item = &'a Vector<N, D, S>>,
    N: PhysicsScalar,
>(
    iter: I,
    index: usize,
) -> N {
    iter.max_by(|a, b| unsafe {
        match a
            .vget_unchecked(index)
            .partial_cmp(&b.vget_unchecked(index))
        {
            Some(ord) => ord,
            None => std::cmp::Ordering::Equal,
        }
    })
    .map(|v| unsafe { *v.get_unchecked(index) })
    .unwrap()
}

#[inline(always)]
pub(crate) fn is_zero<N: Float>(a: N) -> bool {
    Float::abs(a) < Float::epsilon()
}

#[inline(always)]
pub(crate) fn min_component<'a, N: PhysicsScalar, S: Storage<N, D> + 'a, D: Dim>(
    v: &'a Vector<N, D, S>,
) -> (usize, &'a N) {
    v.iter()
        .enumerate()
        .min_by(|(_, &f1), (_, &f2)| n_ordering(f1, f2))
        .unwrap()
}

#[allow(dead_code)]
#[inline(always)]
pub(crate) fn max_component<'a, N: PhysicsScalar, S: Storage<N, D> + 'a, D: Dim>(
    v: &'a Vector<N, D, S>,
) -> (usize, &'a N) {
    v.iter()
        .enumerate()
        .max_by(|(_, &f1), (_, &f2)| n_ordering(f1, f2))
        .unwrap()
}

#[inline(always)]
pub(crate) fn closest_to_segment<N: PhysicsScalar>(
    start: &Vector3<N>,
    end: &Vector3<N>,
    point: &Vector3<N>,
) -> Vector3<N> {
    let ab: Vector3<N> = end - start;
    let t: N = (point - start).dot(&ab) / ab.dot(&ab);
    start + ab * n_min(n_max(t, N::zero()), N::one())
}
