use super::*;
use nalgebra::*;

use std::cmp::*;
use std::iter::FromIterator;
use std::iter::Sum;
use std::ops::*;

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct AxisAlignedBoundingBox<N: PhysicsScalar> {
    pub start: Vector2<N>,
    pub end: Vector2<N>,
}

pub type AABB<N> = AxisAlignedBoundingBox<N>;

impl<N: FloatingPhysicsScalar> FromIterator<Vector2<N>> for AxisAlignedBoundingBox<N> {
    fn from_iter<T: IntoIterator<Item = Vector2<N>>>(iter: T) -> Self {
        let mut start = Vector2::from_element(Bounded::max_value());
        let mut end = Vector2::from_element(Bounded::min_value());
        for v in iter {
            for (s, point) in start.iter_mut().zip(v.iter()) {
                *s = n_min(*point, *s);
            }
            for (s, point) in end.iter_mut().zip(v.iter()) {
                *s = n_max(*point, *s);
            }
        }
        AxisAlignedBoundingBox { start, end }
    }
}

impl<'a, N: FloatingPhysicsScalar> FromIterator<&'a Vector2<N>> for AxisAlignedBoundingBox<N> {
    fn from_iter<T: IntoIterator<Item = &'a Vector2<N>>>(iter: T) -> Self {
        let mut start = Vector2::from_element(Bounded::max_value());
        let mut end = Vector2::from_element(Bounded::min_value());
        for v in iter {
            for (s, point) in start.iter_mut().zip(v.iter()) {
                *s = num_traits::clamp_min(*point, *s);
            }
            for (s, point) in end.iter_mut().zip(v.iter()) {
                *s = num_traits::clamp_max(*point, *s);
            }
        }
        AxisAlignedBoundingBox { start, end }
    }
}

impl<N: FloatingPhysicsScalar> Add for AxisAlignedBoundingBox<N> {
    type Output = AxisAlignedBoundingBox<N>;

    fn add(self, rhs: AxisAlignedBoundingBox<N>) -> Self::Output {
        let arr = [self.start, self.end, rhs.start, rhs.end];
        AxisAlignedBoundingBox {
            start: Vector2::new(n_min_iter(arr.iter(), 0), n_min_iter(arr.iter(), 1)),
            end: Vector2::new(n_max_iter(arr.iter(), 0), n_max_iter(arr.iter(), 1)),
        }
    }
}

impl<N: FloatingPhysicsScalar> AddAssign for AxisAlignedBoundingBox<N> {
    fn add_assign(&mut self, rhs: AxisAlignedBoundingBox<N>) {
        *self = *self + rhs
    }
}

impl<N: FloatingPhysicsScalar> Mul<N> for AxisAlignedBoundingBox<N> {
    type Output = Self;
    fn mul(self, rhs: N) -> Self::Output {
        AxisAlignedBoundingBox {
            start: self.start * rhs,
            end: self.end * rhs,
        }
    }
}

impl<N: FloatingPhysicsScalar> MulAssign<N> for AxisAlignedBoundingBox<N> {
    fn mul_assign(&mut self, rhs: N) {
        self.start *= rhs;
        self.end *= rhs;
    }
}

impl<N: FloatingPhysicsScalar> AxisAlignedBoundingBox<N> {
    pub fn width(&self) -> N {
        Float::abs(self.start.x - self.end.x)
    }
    pub fn height(&self) -> N {
        Float::abs(self.start.y - self.end.y)
    }
    pub fn half_width(&self) -> N {
        self.width() * N::from_f64(0.5f64).unwrap()
    }
    pub fn half_height(&self) -> N {
        self.height() * N::from_f64(0.5f64).unwrap()
    }
    pub fn min_max(&self) -> (Vector2<N>, Vector2<N>) {
        let min_vec = Vector2::new(
            n_min(self.start.x, self.end.x),
            n_min(self.start.y, self.end.y),
        );
        let max_vec = Vector2::new(
            n_max(self.start.x, self.end.x),
            n_max(self.start.y, self.end.y),
        );
        (min_vec, max_vec)
    }
    pub fn closest_point(&self, point: &Vector2<N>) -> Vector2<N> {
        let (min_vec, max_vec) = self.min_max();
        Vector2::new(
            clamp(point.x, min_vec.x, max_vec.x),
            clamp(point.y, min_vec.y, max_vec.y),
        )
    }
    pub fn largest_dim(&self) -> (usize, N) {
        [self.width(), self.height()]
            .iter()
            .enumerate()
            .max_by(|(_, &a), (_, &b)| n_ordering(a, b))
            .map(|(i, f)| (i, *f))
            .unwrap()
    }
    pub fn smallest_dim(&self) -> (usize, N) {
        [self.width(), self.height()]
            .iter()
            .enumerate()
            .min_by(|(_, &a), (_, &b)| n_ordering(a, b))
            .map(|(i, f)| (i, *f))
            .unwrap()
    }
    pub fn get_aabb_collision(
        &self,
        aabb: &AxisAlignedBoundingBox<N>,
    ) -> Option<CollisionResolution<Vector2<N>, N>> {
        let n = self.center() - aabb.center();
        let overlap = Vector2::new(
            self.half_width() + aabb.half_width() - Float::abs(n.x),
            self.half_height() + aabb.half_height() - Float::abs(n.y),
        );
        if overlap.x > N::zero() && overlap.y > N::zero() {
            let (index, &penetration) = min_component(&overlap);
            let normal = match index {
                0 => Vector2::new(
                    if n.x < N::zero() { -N::one() } else { N::one() },
                    N::zero(),
                ),
                1 => Vector2::new(
                    N::zero(),
                    if n.y < N::zero() { -N::one() } else { N::one() },
                ),
                _ => panic!("Unexpected min component"),
            };
            Some(CollisionResolution {
                normal,
                penetration,
            })
        } else {
            None
        }
    }
    pub fn corners(&self) -> [Vector2<N>; 4] {
        let (start_x, start_y) = (self.start.x, self.start.y);
        let (end_x, end_y) = (self.end.x, self.end.y);
        [
            Vector2::new(start_x, start_y),
            Vector2::new(start_x, end_y),
            Vector2::new(end_x, start_y),
            Vector2::new(end_x, end_y),
        ]
    }
    pub fn iter(&self) -> impl Iterator<Item = N> + '_ {
        self.start.iter().chain(self.end.iter()).cloned()
    }
    pub fn get_circle_collision(
        &self,
        circle: &Circle<N>,
    ) -> Option<CollisionResolution<Vector2<N>, N>> {
        let n = self.center() - circle.center();
        let extent = Vector2::new(self.half_width(), self.half_height());
        let mut closest = Vector2::new(
            clamp(n.x, -extent.x, extent.x),
            clamp(n.y, -extent.y, extent.y),
        );
        let inside = {
            if n == closest {
                let (index, _) = min_component(&n.abs());
                match index {
                    0 => {
                        closest.x = extent.x
                            * if closest.x > N::zero() {
                                N::one()
                            } else {
                                -N::one()
                            };
                    }
                    1 => {
                        closest.y = extent.y
                            * if closest.y > N::zero() {
                                N::one()
                            } else {
                                -N::one()
                            };
                    }
                    _ => (),
                }
                true
            } else {
                false
            }
        };

        let normal = n - closest;
        let d = normal.magnitude_squared();
        let r = circle.radius;

        if d > r.pow(N::from_usize(2).unwrap()) && !inside {
            return None;
        }

        let d = Float::sqrt(d);
        Some(CollisionResolution {
            normal: n.normalize(),
            penetration: r - d,
        })
    }
}

impl<N: FloatingPhysicsScalar> Shape2D<N> for AxisAlignedBoundingBox<N> {
    fn bounding_aabb(&self) -> AxisAlignedBoundingBox<N> {
        *self
    }
    fn bounding_sphere(&self) -> Circle<N> {
        Circle {
            center: self.center(),
            radius: [self.width(), self.height()]
                .iter()
                .max_by(|a, b| match a.partial_cmp(b) {
                    Some(o) => o,
                    None => Ordering::Equal,
                })
                .cloned()
                .unwrap(),
        }
    }
    fn center(&self) -> Vector2<N> {
        Vector2::new(
            (self.start.x + self.end.x) * N::from_f64(0.5f64).unwrap(),
            (self.start.y + self.end.y) * N::from_f64(0.5f64).unwrap(),
        )
    }
    fn translate(&self, point: &Vector2<N>) -> Self {
        AxisAlignedBoundingBox {
            start: self.start + point,
            end: self.end + point,
        }
    }
    fn translate_mut(&mut self, point: &Vector2<N>) {
        self.start += point;
        self.end += point;
    }
    fn set_center(&self, point: &Vector2<N>) -> Self {
        let offset = Vector2::new(self.half_width(), self.half_height());
        AxisAlignedBoundingBox {
            start: point - offset,
            end: point + offset,
        }
    }
    fn set_center_mut(&mut self, point: &Vector2<N>) {
        let offset = Vector2::new(self.half_width(), self.half_height());
        self.start = point - offset;
        self.end = point + offset;
    }
}

impl<N: FloatingPhysicsScalar> Sum for AxisAlignedBoundingBox<N> {
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        let mut aabb = AABB {
            start: Vector2::<N>::zeros(),
            end: Vector2::<N>::zeros(),
        };
        for a in iter.into_iter() {
            aabb += a;
        }
        aabb
    }
}

impl<'a, N: FloatingPhysicsScalar> Sum<&'a Self> for AxisAlignedBoundingBox<N> {
    fn sum<I: Iterator<Item = &'a Self>>(iter: I) -> Self {
        let mut aabb = AABB {
            start: Vector2::<N>::zeros(),
            end: Vector2::<N>::zeros(),
        };
        for a in iter.into_iter() {
            aabb += *a;
        }
        aabb
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sum_test() {
        let aabb: AxisAlignedBoundingBox<f32> = [
            AxisAlignedBoundingBox {
                start: Vector2::<f32>::new(-10f32, -1f32),
                end: Vector2::<f32>::new(-560f32, 25f32),
            },
            AxisAlignedBoundingBox {
                start: Vector2::<f32>::new(10f32, 505f32),
                end: Vector2::<f32>::new(258f32, 480f32),
            },
        ]
        .iter()
        .sum();

        aabb.start.relative_eq(
            &Vector2::<f32>::new(-560f32, -1f32),
            std::f32::EPSILON,
            std::f32::EPSILON,
        );
        aabb.end.relative_eq(
            &Vector2::<f32>::new(258f32, 505f32),
            std::f32::EPSILON,
            std::f32::EPSILON,
        );
    }
    #[test]
    fn dim_test() {
        let aabb = AxisAlignedBoundingBox {
            start: Vector2::<f32>::new(0f32, 10f32),
            end: Vector2::<f32>::new(-50f32, 100f32),
        };

        assert_eq!(aabb.largest_dim(), (1, 90f32));
        assert_eq!(aabb.smallest_dim(), (0, 50f32));
    }
    #[test]
    fn mul_test() {
        let start = Vector2::<f32>::new(1f32, 2f32);
        let end = Vector2::<f32>::new(0f32, 10f32);

        let aabb = AABB { start, end };
        let aabb2 = aabb * 0.5f32;

        assert_eq!(aabb2.start, start * 0.5f32);
        assert_eq!(aabb2.end, end * 0.5f32);
    }
}
