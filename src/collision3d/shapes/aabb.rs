use super::*;
use nalgebra::*;

use std::cmp::*;

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct AABB {
    pub start: Vector3<f32>,
    pub end: Vector3<f32>,
}

impl AABB {
    fn width(&self) -> f32 {
        (self.start.x - self.end.x).abs()
    }
    fn half_width(&self) -> f32 {
        self.width() * 0.5f32
    }
    fn height(&self) -> f32 {
        (self.start.y - self.end.y).abs()
    }
    fn half_height(&self) -> f32 {
        self.height() * 0.5f32
    }
    fn depth(&self) -> f32 {
        (self.start.z - self.end.z).abs()
    }
    fn half_depth(&self) -> f32 {
        self.depth() * 0.5f32
    }
}

impl Shape3D for AABB {
    fn bounding_aabb(&self) -> AABB {
        *self
    }
    fn bounding_sphere(&self) -> Sphere {
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
    fn center(&self) -> Vector3<f32> {
        Vector3::new(
            (self.start.x + self.end.x) * 0.5f32,
            (self.start.y + self.end.y) * 0.5f32,
            (self.start.z + self.end.z) * 0.5f32,
        )
    }
    fn translate(&mut self, point: &Vector3<f32>) {
        self.start += point;
        self.end += point;
    }
    fn set_center(&mut self, point: &Vector3<f32>) {
        let offset = Vector3::new(self.half_width(), self.half_height(), self.half_depth());

        self.start = point - offset;
        self.end = point + offset;
    }
}
