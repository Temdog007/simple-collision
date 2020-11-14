use super::*;
use crate::collision2d::shapes::Circle;
use nalgebra::*;
use num_traits::Float;

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct Sphere<N: PhysicsScalar> {
    pub center: Vector3<N>,
    pub radius: N,
}

impl<N: FloatingPhysicsScalar> Sphere<N> {
    pub fn get_sphere_collision(
        &self,
        sphere: &Sphere<N>,
    ) -> Option<CollisionResolution<Vector3<N>, N>> {
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
    pub fn get_plane_collision(
        &self,
        plane: &Plane<N>,
    ) -> Option<CollisionResolution<Vector3<N>, N>> {
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
    pub fn get_triangle_collision(
        &self,
        triangle: &Triangle<N>,
        double_sided: bool,
    ) -> Option<CollisionResolution<Vector3<N>, N>> {
        let normal = triangle.normal();
        let dist = (self.center - triangle.point1).dot(&normal);
        if double_sided {
            if Float::abs(dist) > self.radius {
                return None;
            }
        } else {
            if dist < N::zero() || dist > self.radius {
                return None;
            }
        }

        let point = self.center - normal * dist;
        if triangle.contains(&point) {
            return Some(CollisionResolution {
                normal,
                penetration: self.radius - dist,
            });
        }

        let (_, distance) = triangle.closest_point(&self.center);
        if distance < self.radius {
            Some(CollisionResolution {
                normal,
                penetration: self.radius - dist,
            })
        } else {
            None
        }
    }
    pub fn get_capsule_collision(
        &self,
        capsule: &Capsule<N>,
    ) -> Option<CollisionResolution<Vector3<N>, N>> {
        let center = capsule.closest_point(&self.center);
        self.get_sphere_collision(&Sphere {
            center,
            radius: capsule.radius,
        })
    }
    pub fn to_circle(&self, horizontal: Axis, vertical: Axis) -> Circle<N> {
        debug_assert_ne!(horizontal, vertical);
        let x = match horizontal {
            Axis::X => self.center.x,
            Axis::Y => self.center.y,
            Axis::Z => self.center.z,
        };
        let y = match vertical {
            Axis::X => self.center.x,
            Axis::Y => self.center.y,
            Axis::Z => self.center.z,
        };
        Circle {
            center: Vector2::new(x, y),
            radius: self.radius,
        }
    }
}

impl<N: FloatingPhysicsScalar> Shape3D<N> for Sphere<N> {
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
