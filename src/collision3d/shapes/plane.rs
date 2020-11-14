use nalgebra::*;

use super::*;

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct Plane<N: PhysicsScalar> {
    pub normal: Vector3<N>,
    pub d: N,
}

impl<N: FloatingPhysicsScalar> From<&Triangle<N>> for Plane<N> {
    fn from(triangle: &Triangle<N>) -> Self {
        let normal = triangle.normal();
        let d = -normal.dot(&triangle.point1);
        Plane { normal, d }
    }
}

impl<N: FloatingPhysicsScalar> Plane<N> {
    pub fn from_point(normal: &Vector3<N>, point: &Vector3<N>) -> Self {
        Plane {
            normal: *normal,
            d: -normal.dot(point),
        }
    }
    pub fn distance(&self, point: &Vector3<N>) -> N {
        self.normal.dot(point) + self.d
    }
    pub fn random_point(&self) -> Vector3<N> {
        if !is_zero(self.normal.x) {
            return Vector3::new(-self.d / self.normal.x, N::zero(), N::zero());
        }
        if !is_zero(self.normal.y) {
            return Vector3::new(N::zero(), -self.d / self.normal.y, N::zero());
        }
        if !is_zero(self.normal.z) {
            return Vector3::new(N::zero(), N::zero(), -self.d / self.normal.z);
        }
        panic!("Failed to find random point on plane");
    }
    pub fn closest_point(&self, point: &Vector3<N>) -> Vector3<N> {
        point - self.normal * self.distance(point)
    }
}

impl<N: FloatingPhysicsScalar> Shape3D<N> for Plane<N> {
    fn bounding_aabb(&self) -> AxisAlignedBoundingBox<N> {
        AxisAlignedBoundingBox {
            start: Vector3::from_element(Bounded::min_value()),
            end: Vector3::from_element(Bounded::max_value()),
        }
    }
    fn bounding_sphere(&self) -> Sphere<N> {
        Sphere {
            center: Vector3::from_element(N::zero()),
            radius: Bounded::max_value(),
        }
    }
    fn center(&self) -> Vector3<N> {
        Vector3::from_element(N::zero())
    }
    fn translate_mut(&mut self, _: &Vector3<N>) {}
    fn set_center_mut(&mut self, _: &Vector3<N>) {}
    fn translate(&self, _: &Vector3<N>) -> Self {
        *self
    }
    fn set_center(&self, _: &Vector3<N>) -> Self {
        *self
    }
}
