use nalgebra::*;

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct CollisionResolution {
    pub normal: Vector3<f32>,
    pub penetration: f32,
}

impl CollisionResolution {
    pub fn new(normal: &Vector3<f32>, penetration: f32) -> CollisionResolution {
        CollisionResolution {
            normal: *normal,
            penetration,
        }
    }
    pub fn total_force(&self) -> Vector3<f32> {
        self.normal * self.penetration
    }
}
