use super::*;
use nalgebra::*;

use std::cmp::*;
use std::ops::*;

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct AABB {
    pub start: Vector3<f32>,
    pub end: Vector3<f32>,
}

impl Add for AABB {
    type Output = AABB;

    fn add(self, rhs: AABB) -> Self::Output {
        AABB {
            start: Vector3::new(
                f32_min(get_x(&self.start), get_x(&rhs.start)),
                f32_min(get_y(&self.start), get_y(&rhs.start)),
                f32_min(get_z(&self.start), get_z(&rhs.start)),
            ),
            end: Vector3::new(
                f32_max(get_x(&self.start), get_x(&rhs.start)),
                f32_max(get_y(&self.start), get_y(&rhs.start)),
                f32_max(get_z(&self.start), get_z(&rhs.start)),
            ),
        }
    }
}

impl AddAssign for AABB {
    fn add_assign(&mut self, rhs: AABB) {
        *self = *self + rhs
    }
}

impl Add<Vector3<f32>> for AABB {
    type Output = AABB;

    fn add(self, rhs: Vector3<f32>) -> Self::Output {
        let center = self.center() + rhs;
        let mut v = self;
        v.set_center(&center);
        v
    }
}

impl AddAssign<Vector3<f32>> for AABB {
    fn add_assign(&mut self, rhs: Vector3<f32>) {
        *self = *self + rhs
    }
}

impl From<AABB> for Matrix4<f32> {
    fn from(aabb: AABB) -> Matrix4<f32> {
        let mut m = Matrix4::identity();
        m.append_nonuniform_scaling_mut(&Vector3::new(aabb.width(), aabb.height(), aabb.depth()));
        m.append_translation_mut(&aabb.center());
        m
    }
}

impl AABB {
    pub fn width(&self) -> f32 {
        (self.start.x - self.end.x).abs()
    }
    pub fn half_width(&self) -> f32 {
        self.width() * 0.5f32
    }
    pub fn height(&self) -> f32 {
        (self.start.y - self.end.y).abs()
    }
    pub fn half_height(&self) -> f32 {
        self.height() * 0.5f32
    }
    pub fn depth(&self) -> f32 {
        (self.start.z - self.end.z).abs()
    }
    pub fn half_depth(&self) -> f32 {
        self.depth() * 0.5f32
    }
    pub fn closest_point(&self, point: &Vector3<f32>) -> Vector3<f32> {
        Vector3::new(
            clamp(get_x(point), get_x(&self.start), get_x(&self.end)),
            clamp(get_y(point), get_y(&self.start), get_y(&self.end)),
            clamp(get_z(point), get_z(&self.start), get_z(&self.end)),
        )
    }
    pub fn largest_dim(&self) -> (usize, f32) {
        [self.width(), self.height(), self.depth()]
            .iter()
            .enumerate()
            .max_by(|(_, &a), (_, &b)| f32_ordering(a, b))
            .map(|(i, f)| (i, *f))
            .unwrap()
    }
    pub fn smallest_dim(&self) -> (usize, f32) {
        [self.width(), self.height(), self.depth()]
            .iter()
            .enumerate()
            .min_by(|(_, &a), (_, &b)| f32_ordering(a, b))
            .map(|(i, f)| (i, *f))
            .unwrap()
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
