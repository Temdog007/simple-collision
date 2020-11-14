use super::*;
use nalgebra::*;

use arrayvec::*;
use std::cmp::*;
use std::iter::FromIterator;
use std::iter::Sum;
use std::ops::*;

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct AxisAlignedBoundingBox<N: PhysicsScalar> {
    pub start: Vector3<N>,
    pub end: Vector3<N>,
}

pub type AABB<N> = AxisAlignedBoundingBox<N>;

impl<N: FloatingPhysicsScalar> FromIterator<Vector3<N>> for AxisAlignedBoundingBox<N> {
    fn from_iter<T: IntoIterator<Item = Vector3<N>>>(iter: T) -> Self {
        let mut start = Vector3::from_element(Bounded::max_value());
        let mut end = Vector3::from_element(Bounded::min_value());
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

impl<'a, N: FloatingPhysicsScalar> FromIterator<&'a Vector3<N>> for AxisAlignedBoundingBox<N> {
    fn from_iter<T: IntoIterator<Item = &'a Vector3<N>>>(iter: T) -> Self {
        let mut start = Vector3::from_element(Bounded::max_value());
        let mut end = Vector3::from_element(Bounded::min_value());
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
            start: Vector3::new(
                n_min_iter(arr.iter(), 0),
                n_min_iter(arr.iter(), 1),
                n_min_iter(arr.iter(), 2),
            ),
            end: Vector3::new(
                n_max_iter(arr.iter(), 0),
                n_max_iter(arr.iter(), 1),
                n_max_iter(arr.iter(), 2),
            ),
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

impl<N: FloatingPhysicsScalar> From<AxisAlignedBoundingBox<N>> for Matrix4<N> {
    fn from(aabb: AxisAlignedBoundingBox<N>) -> Matrix4<N> {
        let mut m = Matrix4::identity();
        m.append_nonuniform_scaling_mut(&Vector3::new(aabb.width(), aabb.height(), aabb.depth()));
        m.append_translation_mut(&aabb.center());
        m
    }
}

impl<N: FloatingPhysicsScalar + FromPrimitive> AxisAlignedBoundingBox<N> {
    pub fn half_width(&self) -> N {
        self.width() / N::from_i8(2).unwrap()
    }
    pub fn half_height(&self) -> N {
        self.height() / N::from_i8(2).unwrap()
    }
    pub fn half_depth(&self) -> N {
        self.depth() / N::from_i8(2).unwrap()
    }
}

impl<N: FloatingPhysicsScalar> AxisAlignedBoundingBox<N> {
    pub fn width(&self) -> N {
        Float::abs(self.start.x - self.end.x)
    }
    pub fn height(&self) -> N {
        Float::abs(self.start.y - self.end.y)
    }
    pub fn depth(&self) -> N {
        Float::abs(self.start.z - self.end.z)
    }
    pub fn min_max(&self) -> (Vector3<N>, Vector3<N>) {
        let min_vec = Vector3::new(
            n_min(self.start.x, self.end.x),
            n_min(self.start.y, self.end.y),
            n_min(self.start.z, self.end.z),
        );
        let max_vec = Vector3::new(
            n_max(self.start.x, self.end.x),
            n_max(self.start.y, self.end.y),
            n_max(self.start.z, self.end.z),
        );
        (min_vec, max_vec)
    }
    pub fn closest_point(&self, point: &Vector3<N>) -> Vector3<N> {
        let (min_vec, max_vec) = self.min_max();
        Vector3::new(
            clamp(point.x, min_vec.x, max_vec.x),
            clamp(point.y, min_vec.y, max_vec.y),
            clamp(point.z, min_vec.z, max_vec.z),
        )
    }
    pub fn largest_dim(&self) -> (usize, N) {
        [self.width(), self.height(), self.depth()]
            .iter()
            .enumerate()
            .max_by(|(_, &a), (_, &b)| n_ordering(a, b))
            .map(|(i, f)| (i, *f))
            .unwrap()
    }
    pub fn smallest_dim(&self) -> (usize, N) {
        [self.width(), self.height(), self.depth()]
            .iter()
            .enumerate()
            .min_by(|(_, &a), (_, &b)| n_ordering(a, b))
            .map(|(i, f)| (i, *f))
            .unwrap()
    }
    pub fn get_aabb_collision(
        &self,
        aabb: &AxisAlignedBoundingBox<N>,
    ) -> Option<CollisionResolution<Vector3<N>, N>> {
        let n = self.center() - aabb.center();
        let overlap = Vector3::new(
            self.half_width() + aabb.half_width() - Float::abs(n.x),
            self.half_height() + aabb.half_height() - Float::abs(n.y),
            self.half_depth() + aabb.half_depth() - Float::abs(n.z),
        );
        if overlap.x > N::zero() && overlap.y > N::zero() && overlap.z > N::zero() {
            let (index, &penetration) = min_component(&overlap);
            let normal = match index {
                0 => Vector3::new(
                    if n.x < N::zero() { -N::one() } else { N::one() },
                    N::zero(),
                    N::zero(),
                ),
                1 => Vector3::new(
                    N::zero(),
                    if n.y < N::zero() { -N::one() } else { N::one() },
                    N::zero(),
                ),
                2 => Vector3::new(
                    N::zero(),
                    N::zero(),
                    if n.z < N::zero() { -N::one() } else { N::one() },
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
    pub fn get_sphere_collision(
        &self,
        sphere: &Sphere<N>,
    ) -> Option<CollisionResolution<Vector3<N>, N>> {
        let n = self.center() - sphere.center();
        let extent = Vector3::new(self.half_width(), self.half_height(), self.half_depth());
        let mut closest = Vector3::new(
            clamp(n.x, -extent.x, extent.x),
            clamp(n.y, -extent.y, extent.y),
            clamp(n.z, -extent.z, extent.z),
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
                    2 => {
                        closest.z = extent.z
                            * if closest.z > N::zero() {
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
        let r = sphere.radius;

        if d > r.pow(N::from_usize(2).unwrap()) && !inside {
            return None;
        }

        let d = Float::sqrt(d);
        Some(CollisionResolution {
            normal: n.normalize(),
            penetration: r - d,
        })
    }
    fn get_plane_collision_with_closure<T, Init, Fold>(
        &self,
        plane: &Plane<N>,
        init: Init,
        fold: Fold,
    ) -> Option<T>
    where
        Init: FnOnce() -> T,
        Fold: FnMut(T, (&Vector3<N>, N)) -> T,
    {
        let corners = self.corners();
        let distances: ArrayVec<[N; 8]> = corners
            .iter()
            .map(|corner| plane.distance(corner))
            .collect();

        let value: i32 = distances
            .iter()
            .map(|dist| if *dist > N::zero() { 1 } else { -1 })
            .sum();
        if value.abs() == 8 {
            return None;
        }

        Some(corners.iter().zip(distances.into_iter()).fold(init(), fold))
    }
    pub fn get_plane_collision(
        &self,
        plane: &Plane<N>,
    ) -> Option<CollisionResolution<Vector3<N>, N>> {
        let init = || CollisionResolution {
            normal: plane.normal,
            penetration: N::zero(),
        };
        let fold = |mut result: CollisionResolution<Vector3<N>, N>,
                    (corner, distance): (&Vector3<N>, N)| {
            let point = corner - plane.normal * distance;
            if (point - corner).dot(&plane.normal) > N::zero() {
                result.penetration = n_max(result.penetration, Float::abs(distance));
            }
            result
        };

        match self.get_plane_collision_with_closure(plane, init, fold) {
            Some(result) => {
                if is_zero(result.penetration) {
                    None
                } else {
                    Some(result)
                }
            }
            None => None,
        }
    }
    pub fn get_triangle_collision(
        &self,
        triangle: &Triangle<N>,
    ) -> Option<CollisionResolution<Vector3<N>, N>> {
        let plane = triangle.to_plane();
        let init = || {
            (
                CollisionResolution {
                    normal: plane.normal,
                    penetration: N::zero(),
                },
                false,
            )
        };
        let fold = |(mut result, mut inside): (CollisionResolution<Vector3<N>, N>, bool),
                    (corner, distance): (&Vector3<N>, N)| {
            let point = corner - plane.normal * distance;
            if triangle.contains(&point) {
                inside = true;
            }
            if (point - corner).dot(&plane.normal) > N::zero() {
                result.penetration = n_max(result.penetration, Float::abs(distance));
            }
            (result, inside)
        };
        match self.get_plane_collision_with_closure(&plane, init, fold) {
            Some((result, inside)) => {
                if !inside || is_zero(result.penetration) {
                    None
                } else {
                    Some(result)
                }
            }
            None => None,
        }
    }
    pub fn get_capsule_collision(
        &self,
        capsule: &Capsule<N>,
    ) -> Option<CollisionResolution<Vector3<N>, N>> {
        let center = capsule.closest_point(&self.center());
        self.get_sphere_collision(&Sphere {
            center,
            radius: capsule.radius,
        })
    }
    pub fn corners(&self) -> [Vector3<N>; 8] {
        let (start_x, start_y, start_z) = (self.start.x, self.start.y, self.start.z);
        let (end_x, end_y, end_z) = (self.end.x, self.end.y, self.end.z);
        [
            Vector3::new(start_x, start_y, start_z),
            Vector3::new(start_x, start_y, end_z),
            Vector3::new(start_x, end_y, start_z),
            Vector3::new(start_x, end_y, end_z),
            Vector3::new(end_x, start_y, start_z),
            Vector3::new(end_x, start_y, end_z),
            Vector3::new(end_x, end_y, start_z),
            Vector3::new(end_x, end_y, end_z),
        ]
    }
    pub fn iter(&self) -> impl Iterator<Item = N> + '_ {
        self.start.iter().chain(self.end.iter()).cloned()
    }
}

impl<N: FloatingPhysicsScalar> Shape3D<N> for AxisAlignedBoundingBox<N> {
    fn bounding_aabb(&self) -> AxisAlignedBoundingBox<N> {
        *self
    }
    fn bounding_sphere(&self) -> Sphere<N> {
        Sphere {
            center: self.center(),
            radius: [self.width(), self.height(), self.depth()]
                .iter()
                .max_by(|a, b| match a.partial_cmp(b) {
                    Some(o) => o,
                    None => Ordering::Equal,
                })
                .cloned()
                .unwrap(),
        }
    }
    fn center(&self) -> Vector3<N> {
        Vector3::new(
            (self.start.x + self.end.x) * N::from_f64(0.5f64).unwrap(),
            (self.start.y + self.end.y) * N::from_f64(0.5f64).unwrap(),
            (self.start.z + self.end.z) * N::from_f64(0.5f64).unwrap(),
        )
    }
    fn translate(&self, point: &Vector3<N>) -> Self {
        AxisAlignedBoundingBox {
            start: self.start + point,
            end: self.end + point,
        }
    }
    fn translate_mut(&mut self, point: &Vector3<N>) {
        self.start += point;
        self.end += point;
    }
    fn set_center(&self, point: &Vector3<N>) -> Self {
        let offset = Vector3::new(self.half_width(), self.half_height(), self.half_depth());
        AxisAlignedBoundingBox {
            start: point - offset,
            end: point + offset,
        }
    }
    fn set_center_mut(&mut self, point: &Vector3<N>) {
        let offset = Vector3::new(self.half_width(), self.half_height(), self.half_depth());
        self.start = point - offset;
        self.end = point + offset;
    }
}

impl<N: FloatingPhysicsScalar> Sum for AxisAlignedBoundingBox<N> {
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        let mut aabb = AABB {
            start: Vector3::<N>::zeros(),
            end: Vector3::<N>::zeros(),
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
            start: Vector3::<N>::zeros(),
            end: Vector3::<N>::zeros(),
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
                start: Vector3::<f32>::new(-10f32, -1f32, 500f32),
                end: Vector3::<f32>::new(-560f32, 25f32, 120f32),
            },
            AxisAlignedBoundingBox {
                start: Vector3::<f32>::new(10f32, 505f32, -159f32),
                end: Vector3::<f32>::new(258f32, 480f32, -285f32),
            },
        ]
        .iter()
        .sum();

        aabb.start.relative_eq(
            &Vector3::<f32>::new(-560f32, -1f32, -285f32),
            std::f32::EPSILON,
            std::f32::EPSILON,
        );
        aabb.end.relative_eq(
            &Vector3::<f32>::new(258f32, 505f32, 500f32),
            std::f32::EPSILON,
            std::f32::EPSILON,
        );
    }
    #[test]
    fn dim_test() {
        let aabb = AxisAlignedBoundingBox {
            start: Vector3::<f32>::new(0f32, 10f32, 20f32),
            end: Vector3::<f32>::new(-50f32, 100f32, 100f32),
        };

        assert_eq!(aabb.largest_dim(), (1, 90f32));
        assert_eq!(aabb.smallest_dim(), (0, 50f32));
    }
    #[test]
    fn mul_test() {
        let start = Vector3::<f32>::new(1f32, 2f32, 5f32);
        let end = Vector3::<f32>::new(0f32, 10f32, -2f32);

        let aabb = AABB { start, end };
        let aabb2 = aabb * 0.5f32;

        assert_eq!(aabb2.start, start * 0.5f32);
        assert_eq!(aabb2.end, end * 0.5f32);
    }
}
