use super::*;
use nalgebra::*;

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct Ray {
    point: Vector3<f32>,
    direction: Vector3<f32>,
}

impl Ray {
    pub fn new(point: &Vector3<f32>, direction: &Vector3<f32>) -> Ray {
        Ray {
            point: *point,
            direction: *direction,
        }
    }
    pub fn get_point(&self, dist: f32) -> Vector3<f32> {
        self.point + self.direction * dist
    }
    pub fn to_plane(&self, normal: &Vector3<f32>) -> Plane {
        let d = -normal.dot(&self.point);
        Plane { normal: *normal, d }
    }
}
