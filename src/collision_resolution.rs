use super::*;

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

use std::ops::*;

#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct CollisionResolution<T: Mul<N> + Copy, N: PhysicsScalar> {
    pub normal: T,
    pub penetration: N,
}

impl<T: Copy + Mul<N, Output = T>, N: PhysicsScalar> CollisionResolution<T, N> {
    pub fn total_force(&self) -> T {
        self.normal * self.penetration
    }
    pub fn flip(&self) -> Self {
        Self {
            normal: self.normal * -N::one(),
            penetration: self.penetration,
        }
    }
}
