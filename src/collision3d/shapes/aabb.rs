use super::*;
use nalgebra::*;

use std::cmp::*;
use std::ops::*;

use arrayvec::*;

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
        let (min_vec1, max_vec1) = self.min_max();
        let (min_vec2, max_vec2) = rhs.min_max();
        AABB {
            start: Vector3::new(
                f32_min(get_x(&min_vec1), get_x(&max_vec1)),
                f32_min(get_y(&min_vec1), get_y(&max_vec1)),
                f32_min(get_z(&min_vec1), get_z(&max_vec1)),
            ),
            end: Vector3::new(
                f32_max(get_x(&min_vec2), get_x(&max_vec2)),
                f32_max(get_y(&min_vec2), get_y(&max_vec2)),
                f32_max(get_z(&min_vec2), get_z(&max_vec2)),
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
    pub fn min_max(&self) -> (Vector3<f32>, Vector3<f32>) {
        let min_vec = Vector3::new(
            f32_min(get_x(&self.start), get_x(&self.end)),
            f32_min(get_y(&self.start), get_y(&self.end)),
            f32_min(get_z(&self.start), get_z(&self.end)),
        );
        let max_vec = Vector3::new(
            f32_max(get_x(&self.start), get_x(&self.end)),
            f32_max(get_y(&self.start), get_y(&self.end)),
            f32_max(get_z(&self.start), get_z(&self.end)),
        );
        (min_vec, max_vec)
    }
    pub fn closest_point(&self, point: &Vector3<f32>) -> Vector3<f32> {
        let (min_vec, max_vec) = self.min_max();
        Vector3::new(
            clamp(get_x(point), get_x(&min_vec), get_x(&max_vec)),
            clamp(get_y(point), get_y(&min_vec), get_y(&max_vec)),
            clamp(get_z(point), get_z(&min_vec), get_z(&max_vec)),
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
    pub fn get_aabb_collision(&self, aabb: &AABB) -> Option<CollisionResolution> {
        let n = self.center() - aabb.center();
        let overlap = Vector3::new(
            self.half_width() + aabb.half_width() - get_x(&n).abs(),
            self.half_height() + aabb.half_height() - get_y(&n).abs(),
            self.half_depth() + aabb.half_depth() - get_z(&n).abs(),
        );
        if get_x(&overlap) > 0f32 && get_y(&overlap) > 0f32 && get_z(&overlap) > 0f32 {
            let (index, &penetration) = min_component(&overlap);
            let normal = match index {
                0 => Some(Vector3::new(
                    if get_x(&n) < 0f32 { -1f32 } else { 1f32 },
                    0f32,
                    0f32,
                )),
                1 => Some(Vector3::new(
                    0f32,
                    if get_y(&n) < 0f32 { -1f32 } else { 1f32 },
                    0f32,
                )),
                2 => Some(Vector3::new(
                    0f32,
                    0f32,
                    if get_z(&n) < 0f32 { -1f32 } else { 1f32 },
                )),
                _ => None,
            };
            if let Some(normal) = normal {
                return Some(CollisionResolution::new(&normal, penetration));
            }
        }
        None
    }
    pub fn get_sphere_collision(&self, sphere: &Sphere) -> Option<CollisionResolution> {
        let n = self.center() - sphere.center();
        let extent = Vector3::new(self.half_width(), self.half_height(), self.half_depth());
        let mut closest = Vector3::new(
            clamp(get_x(&n), -get_x(&extent), get_x(&extent)),
            clamp(get_y(&n), -get_y(&extent), get_y(&extent)),
            clamp(get_z(&n), -get_z(&extent), get_z(&extent)),
        );
        let inside = {
            if n == closest {
                let (index, _) = min_component(&n.abs());
                match index {
                    0 => {
                        let value = get_x(&closest);
                        set_x(
                            &mut closest,
                            get_x(&extent) * if value > 0f32 { 1f32 } else { -1f32 },
                        )
                    }
                    1 => {
                        let value = get_y(&closest);
                        set_y(
                            &mut closest,
                            get_y(&extent) * if value > 0f32 { 1f32 } else { -1f32 },
                        )
                    }
                    2 => {
                        let value = get_z(&closest);
                        set_z(
                            &mut closest,
                            get_z(&extent) * if value > 0f32 { 1f32 } else { -1f32 },
                        )
                    }
                    _ => (),
                }
                true
            } else {
                false
            }
        };

        let normal = n - closest;
        let d = normal.magnitude_squared();
        let r = sphere.radius;

        if d > r.powi(2) && !inside {
            return None;
        }

        let d = d.sqrt();
        Some(CollisionResolution {
            normal: n.normalize(),
            penetration: r - d,
        })
    }
    fn get_plane_collision_with_closure<T, Init, Fold>(
        &self,
        plane: &Plane,
        init: Init,
        fold: Fold,
    ) -> Option<T>
    where
        Init: FnOnce() -> T,
        Fold: FnMut(T, (&Vector3<f32>, &f32)) -> T,
    {
        let corners = self.corners();
        let distances: ArrayVec<[f32; 8]> = corners
            .iter()
            .map(|corner| plane.distance(corner))
            .collect();

        let value: i32 = distances
            .iter()
            .map(|dist| if *dist > 0f32 { 1 } else { -1 })
            .sum();
        if value.abs() == 8 {
            return None;
        }

        Some(corners.iter().zip(distances.iter()).fold(init(), fold))
    }
    pub fn get_plane_collision(&self, plane: &Plane) -> Option<CollisionResolution> {
        let init = || CollisionResolution {
            normal: plane.normal,
            penetration: 0f32,
        };
        let fold = |mut result: CollisionResolution, (corner, &distance): (&Vector3<f32>, &f32)| {
            let point = corner - plane.normal * distance;
            if (point - corner).dot(&plane.normal) > 0f32 {
                result.penetration = f32_max(result.penetration, distance.abs());
            }
            result
        };

        match self.get_plane_collision_with_closure(plane, init, fold) {
            Some(result) => {
                if is_zero(result.penetration) {
                    None
                } else {
                    Some(result)
                }
            }
            None => None,
        }
    }
    pub fn get_triangle_collision(&self, triangle: &Triangle) -> Option<CollisionResolution> {
        let plane = triangle.to_plane();
        let init = || {
            (
                CollisionResolution {
                    normal: plane.normal,
                    penetration: 0f32,
                },
                false,
            )
        };
        let fold = |(mut result, mut inside): (CollisionResolution, bool),
                    (corner, &distance): (&Vector3<f32>, &f32)| {
            let point = corner - plane.normal * distance;
            if triangle.contains(&point) {
                inside = true;
            }
            if (point - corner).dot(&plane.normal) > 0f32 {
                result.penetration = f32_max(result.penetration, distance.abs());
            }
            (result, inside)
        };
        match self.get_plane_collision_with_closure(&plane, init, fold) {
            Some((result, inside)) => {
                if !inside || is_zero(result.penetration) {
                    None
                } else {
                    Some(result)
                }
            }
            None => None,
        }
    }
    pub fn corners(&self) -> [Vector3<f32>; 8] {
        let (start_x, start_y, start_z) =
            (get_x(&self.start), get_y(&self.start), get_z(&self.start));
        let (end_x, end_y, end_z) = (get_x(&self.end), get_y(&self.end), get_z(&self.end));
        [
            Vector3::new(start_x, start_y, start_z),
            Vector3::new(start_x, start_y, end_z),
            Vector3::new(start_x, end_y, start_z),
            Vector3::new(start_x, end_y, end_z),
            Vector3::new(end_x, start_y, start_z),
            Vector3::new(end_x, start_y, end_z),
            Vector3::new(end_x, end_y, start_z),
            Vector3::new(end_x, end_y, end_z),
        ]
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
