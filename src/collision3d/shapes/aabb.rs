use super::*;
use nalgebra::*;

use std::cmp::*;
use std::ops::*;

use arrayvec::*;

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct AABB<N : PhysicsScalar> {
    pub start: Vector3<N>,
    pub end: Vector3<N>,
}

impl<N : PhysicsScalar> Add for AABB<N> {
    type Output = AABB<N>;

    fn add(self, rhs: AABB<N>) -> Self::Output {
        let (min_vec1, max_vec1) = self.min_max();
        let (min_vec2, max_vec2) = rhs.min_max();
        AABB {
            start: Vector3::new(
                n_min(get_x(&min_vec1), get_x(&max_vec1)),
                n_min(get_y(&min_vec1), get_y(&max_vec1)),
                n_min(get_z(&min_vec1), get_z(&max_vec1)),
            ),
            end: Vector3::new(
                n_max(get_x(&min_vec2), get_x(&max_vec2)),
                n_max(get_y(&min_vec2), get_y(&max_vec2)),
                n_max(get_z(&min_vec2), get_z(&max_vec2)),
            ),
        }
    }
}

impl<N : PhysicsScalar> AddAssign for AABB<N> {
    fn add_assign(&mut self, rhs: AABB<N>) {
        *self = *self + rhs
    }
}

impl<N : PhysicsScalar> From<AABB<N>> for Matrix4<N> {
    fn from(aabb: AABB<N>) -> Matrix4<N> {
        let mut m = Matrix4::identity();
        m.append_nonuniform_scaling_mut(&Vector3::new(aabb.width(), aabb.height(), aabb.depth()));
        m.append_translation_mut(&aabb.center());
        m
    }
}

impl<N : PhysicsScalar> AABB<N> {
    pub fn width(&self) -> N {
        Float::abs(self.start.x - self.end.x)
    }
    pub fn half_width(&self) -> N {
        self.width() * N::from_f64(0.5f64).unwrap()
    }
    pub fn height(&self) -> N {
        Float::abs(self.start.y - self.end.y)
    }
    pub fn half_height(&self) -> N {
        self.height() * N::from_f64(0.5f64).unwrap()
    }
    pub fn depth(&self) -> N {
        Float::abs(self.start.z - self.end.z)
    }
    pub fn half_depth(&self) -> N {
        self.depth() * N::from_f64(0.5f64).unwrap()
    }
    pub fn min_max(&self) -> (Vector3<N>, Vector3<N>) {
        let min_vec = Vector3::new(
            n_min(get_x(&self.start), get_x(&self.end)),
            n_min(get_y(&self.start), get_y(&self.end)),
            n_min(get_z(&self.start), get_z(&self.end)),
        );
        let max_vec = Vector3::new(
            n_max(get_x(&self.start), get_x(&self.end)),
            n_max(get_y(&self.start), get_y(&self.end)),
            n_max(get_z(&self.start), get_z(&self.end)),
        );
        (min_vec, max_vec)
    }
    pub fn closest_point(&self, point: &Vector3<N>) -> Vector3<N> {
        let (min_vec, max_vec) = self.min_max();
        Vector3::new(
            clamp(get_x(point), get_x(&min_vec), get_x(&max_vec)),
            clamp(get_y(point), get_y(&min_vec), get_y(&max_vec)),
            clamp(get_z(point), get_z(&min_vec), get_z(&max_vec)),
        )
    }
    pub fn largest_dim(&self) -> (usize, N) {
        [self.width(), self.height(), self.depth()]
            .iter()
            .enumerate()
            .max_by(|(_, &a), (_, &b)| n_ordering(a, b))
            .map(|(i, f)| (i, *f))
            .unwrap()
    }
    pub fn smallest_dim(&self) -> (usize, N) {
        [self.width(), self.height(), self.depth()]
            .iter()
            .enumerate()
            .min_by(|(_, &a), (_, &b)| n_ordering(a, b))
            .map(|(i, f)| (i, *f))
            .unwrap()
    }
    pub fn get_aabb_collision(&self, aabb: &AABB<N>) -> Option<CollisionResolution<N>> {
        let n = self.center() - aabb.center();
        let overlap = Vector3::new(
            self.half_width() + aabb.half_width() - Float::abs(get_x(&n)),
            self.half_height() + aabb.half_height() - Float::abs(get_y(&n)),
            self.half_depth() + aabb.half_depth() - Float::abs(get_z(&n))
        );
        if get_x(&overlap) > N::zero() && get_y(&overlap) > N::zero() && get_z(&overlap) > N::zero() {
            let (index, &penetration) = min_component(&overlap);
            let normal = match index {
                0 => Some(Vector3::new(
                    if get_x(&n) < N::zero() { -N::one() } else { N::one() },
                    N::zero(),
                    N::zero(),
                )),
                1 => Some(Vector3::new(
                    N::zero(),
                    if get_y(&n) < N::zero() { -N::one() } else { N::one() },
                    N::zero(),
                )),
                2 => Some(Vector3::new(
                    N::zero(),
                    N::zero(),
                    if get_z(&n) < N::zero() { -N::one() } else { N::one() },
                )),
                _ => None,
            };
            if let Some(normal) = normal {
                return Some(CollisionResolution::new(&normal, penetration));
            }
        }
        None
    }
    pub fn get_sphere_collision(&self, sphere: &Sphere<N>) -> Option<CollisionResolution<N>> {
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
                            get_x(&extent) * if value > N::zero() { N::one() } else { -N::one() },
                        )
                    }
                    1 => {
                        let value = get_y(&closest);
                        set_y(
                            &mut closest,
                            get_y(&extent) * if value > N::zero() { N::one() } else { -N::one() },
                        )
                    }
                    2 => {
                        let value = get_z(&closest);
                        set_z(
                            &mut closest,
                            get_z(&extent) * if value > N::zero() { N::one() } else { -N::one() },
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

        if d > r.pow(N::from_usize(2).unwrap()) && !inside {
            return None;
        }

        let d = Float::sqrt(d);
        Some(CollisionResolution {
            normal: n.normalize(),
            penetration: r - d,
        })
    }
    fn get_plane_collision_with_closure<T, Init, Fold>(
        &self,
        plane: &Plane<N>,
        init: Init,
        fold: Fold,
    ) -> Option<T>
    where
        Init: FnOnce() -> T,
        Fold: FnMut(T, (&Vector3<N>, &N)) -> T,
    {
        let corners = self.corners();
        let distances: ArrayVec<[N; 8]> = corners
            .iter()
            .map(|corner| plane.distance(corner))
            .collect();

        let value: i32 = distances
            .iter()
            .map(|dist| if *dist > N::zero() { 1 } else { -1 })
            .sum();
        if value.abs() == 8 {
            return None;
        }

        Some(corners.iter().zip(distances.iter()).fold(init(), fold))
    }
    pub fn get_plane_collision(&self, plane: &Plane<N>) -> Option<CollisionResolution<N>> {
        let init = || CollisionResolution {
            normal: plane.normal,
            penetration: N::zero(),
        };
        let fold = |mut result: CollisionResolution<N>, (corner, &distance): (&Vector3<N>, &N)| {
            let point = corner - plane.normal * distance;
            if (point - corner).dot(&plane.normal) > N::zero() {
                result.penetration = n_max(result.penetration, Float::abs(distance));
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
    pub fn get_triangle_collision(&self, triangle: &Triangle<N>) -> Option<CollisionResolution<N>> {
        let plane = triangle.to_plane();
        let init = || {
            (
                CollisionResolution{
                    normal: plane.normal,
                    penetration: N::zero(),
                },
                false,
            )
        };
        let fold = |(mut result, mut inside): (CollisionResolution<N>, bool),
                    (corner, &distance): (&Vector3<N>, &N)| {
            let point = corner - plane.normal * distance;
            if triangle.contains(&point) {
                inside = true;
            }
            if (point - corner).dot(&plane.normal) > N::zero() {
                result.penetration = n_max(result.penetration, Float::abs(distance));
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
    pub fn corners(&self) -> [Vector3<N>; 8] {
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

impl<N : PhysicsScalar> Shape3D<N> for AABB<N> {
    fn bounding_aabb(&self) -> AABB<N> {
        *self
    }
    fn bounding_sphere(&self) -> Sphere<N> {
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
    fn center(&self) -> Vector3<N> {
        Vector3::new(
            (self.start.x + self.end.x) * N::from_f64(0.5f64).unwrap(),
            (self.start.y + self.end.y) * N::from_f64(0.5f64).unwrap(),
            (self.start.z + self.end.z) * N::from_f64(0.5f64).unwrap(),
        )
    }
    fn translate(&self, point: &Vector3<N>) -> Self {
        AABB {
            start: self.start + point,
            end: self.end + point,
        }
    }
    fn translate_mut(&mut self, point: &Vector3<N>) {
        self.start += point;
        self.end += point;
    }
    fn set_center(&self, point: &Vector3<N>) -> Self {
        let offset = Vector3::new(self.half_width(), self.half_height(), self.half_depth());
        AABB {
            start: point - offset,
            end: point + offset,
        }
    }
    fn set_center_mut(&mut self, point: &Vector3<N>) {
        let offset = Vector3::new(self.half_width(), self.half_height(), self.half_depth());
        self.start = point - offset;
        self.end = point + offset;
    }
}
