use super::*;
use nalgebra::*;

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct Sphere<N : PhysicsScalar> {
    pub center: Vector3<N>,
    pub radius: N,
}

impl<N : PhysicsScalar> Sphere<N> {
    pub fn new(center: &Vector3<N>, radius: N) -> Self {
        Sphere {
            center: *center,
            radius,
        }
    }
    pub fn get_sphere_collision(&self, sphere: &Sphere<N>) -> Option<CollisionResolution<N>> {
        let n = self.center - sphere.center;
        let r = (self.radius + sphere.radius).pow(N::from_usize(2).unwrap());

        let len = n.magnitude_squared();
        if len > r {
            return None;
        }

        let d = Float::sqrt(len);
        let (normal, penetration) = if is_zero(d) {
            (Vector3::new(N::one(), N::zero(), N::zero()), self.radius)
        } else {
            let pen = Float::sqrt(r) - d;
            (n.normalize(), pen)
        };
        Some(CollisionResolution {
            normal,
            penetration,
        })
    }
    pub fn get_plane_collision(&self, plane: &Plane<N>) -> Option<CollisionResolution<N>> {
        let dist = plane.distance(&self.center);
        if dist < N::zero() || dist > self.radius {
            None
        } else {
            Some(CollisionResolution {
                normal: plane.normal,
                penetration: self.radius - dist,
            })
        }
    }
    pub fn get_triangle_collision(&self, triangle: &Triangle<N>) -> Option<CollisionResolution<N>> {
        let normal = triangle.normal();
        let dist = (self.center - triangle.point1).dot(&normal);
        if dist < N::zero() || dist > self.radius {
            return None;
        }

        let point = self.center - normal * dist;
        if triangle.contains(&point) {
            Some(CollisionResolution {
                normal,
                penetration: self.radius - dist,
            })
        } else {
            None
        }
    }
}

impl<N : PhysicsScalar> Shape3D<N> for Sphere<N> {
    fn bounding_aabb(&self) -> AxisAlignedBoundingBox<N> {
        AxisAlignedBoundingBox {
            start: self.center() - Vector3::from_element(self.radius),
            end: self.center() + Vector3::from_element(self.radius),
        }
    }
    fn bounding_sphere(&self) -> Sphere<N> {
        *self
    }
    fn center(&self) -> Vector3<N> {
        self.center
    }
    fn translate(&self, point: &Vector3<N>) -> Sphere<N> {
        Sphere {
            center: self.center + point,
            radius: self.radius,
        }
    }
    fn set_center(&self, point: &Vector3<N>) -> Sphere<N> {
        Sphere {
            center: *point,
            radius: self.radius,
        }
    }
    fn translate_mut(&mut self, point: &Vector3<N>) {
        self.center += point
    }
    fn set_center_mut(&mut self, point: &Vector3<N>) {
        self.center = *point;
    }
}
