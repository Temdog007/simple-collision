use nalgebra::*;

use super::*;

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct CollisionResolution<N: PhysicsScalar> {
    pub normal: Vector3<N>,
    pub penetration: N,
}

impl<N : PhysicsScalar> CollisionResolution<N> {
    pub fn new(normal: &Vector3<N>, penetration: N) -> Self {
        CollisionResolution {
            normal: *normal,
            penetration,
        }
    }
    pub fn total_force(&self) -> Vector3<N> {
        self.normal * self.penetration
    }
}
