use super::*;
use nalgebra::*;
use num_traits::Float;

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct Circle<N: PhysicsScalar> {
    pub center: Vector2<N>,
    pub radius: N,
}

impl<N: FloatingPhysicsScalar> Circle<N> {
    pub fn get_circle_collision(
        &self,
        sphere: &Circle<N>,
    ) -> Option<CollisionResolution<Vector2<N>, N>> {
        let n = self.center - sphere.center;
        let r = (self.radius + sphere.radius).pow(N::from_usize(2).unwrap());

        let len = n.magnitude_squared();
        if len > r {
            return None;
        }

        let d = Float::sqrt(len);
        let (normal, penetration) = if is_zero(d) {
            (Vector2::new(N::one(), N::zero()), self.radius)
        } else {
            let pen = Float::sqrt(r) - d;
            (n.normalize(), pen)
        };
        Some(CollisionResolution {
            normal,
            penetration,
        })
    }
}

impl<N: FloatingPhysicsScalar> Shape2D<N> for Circle<N> {
    fn bounding_aabb(&self) -> AxisAlignedBoundingBox<N> {
        AxisAlignedBoundingBox {
            start: self.center() - Vector2::from_element(self.radius),
            end: self.center() + Vector2::from_element(self.radius),
        }
    }
    fn bounding_sphere(&self) -> Circle<N> {
        *self
    }
    fn center(&self) -> Vector2<N> {
        self.center
    }
    fn translate(&self, point: &Vector2<N>) -> Circle<N> {
        Circle {
            center: self.center + point,
            radius: self.radius,
        }
    }
    fn set_center(&self, point: &Vector2<N>) -> Circle<N> {
        Circle {
            center: *point,
            radius: self.radius,
        }
    }
    fn translate_mut(&mut self, point: &Vector2<N>) {
        self.center += point
    }
    fn set_center_mut(&mut self, point: &Vector2<N>) {
        self.center = *point;
    }
}
