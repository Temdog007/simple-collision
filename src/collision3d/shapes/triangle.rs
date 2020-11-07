use super::*;
use nalgebra::*;

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct Triangle<N: PhysicsScalar> {
    pub point1: Vector3<N>,
    pub point2: Vector3<N>,
    pub point3: Vector3<N>,
}

impl<N: PhysicsScalar> Triangle<N> {
    pub fn x_width(&self) -> N {
        let arr = [self.point1.x, self.point2.x, self.point3.x];
        let min_value = arr.iter().min_by(|&a, &b| n_ordering(*a, *b)).unwrap();
        let max_value = arr.iter().max_by(|&a, &b| n_ordering(*a, *b)).unwrap();
        *max_value - *min_value
    }
    pub fn y_width(&self) -> N {
        let arr = [self.point1.y, self.point2.y, self.point3.y];
        let min_value = arr.iter().min_by(|&a, &b| n_ordering(*a, *b)).unwrap();
        let max_value = arr.iter().max_by(|&a, &b| n_ordering(*a, *b)).unwrap();
        *max_value - *min_value
    }
    pub fn z_width(&self) -> N {
        let arr = [self.point1.z, self.point2.z, self.point3.z];
        let min_value = arr.iter().min_by(|&a, &b| n_ordering(*a, *b)).unwrap();
        let max_value = arr.iter().max_by(|&a, &b| n_ordering(*a, *b)).unwrap();
        *max_value - *min_value
    }
    pub fn normal(&self) -> Vector3<N> {
        (self.point2 - self.point1)
            .cross(&(self.point3 - self.point1))
            .normalize()
    }
    pub fn closest_point(&self, point: &Vector3<N>) -> (Vector3<N>, N) {
        if self.contains(&point) {
            return (*point, N::zero());
        }

        let points = [
            closest_to_segment(&self.point1, &self.point2, point),
            closest_to_segment(&self.point2, &self.point3, point),
            closest_to_segment(&self.point3, &self.point1, point),
        ];

        points
            .iter()
            .map(|p| (point, (point - p).norm_squared()))
            .min_by(|(_, dist1), (_, dist2)| n_ordering(*dist1, *dist2))
            .map(|(&point, p)| (point, p))
            .unwrap()
    }
    pub fn contains(&self, point: &Vector3<N>) -> bool {
        let d0 = self.point3 - self.point1;
        let d1 = self.point2 - self.point1;
        let d2 = point - self.point1;
        let d00 = d0.dot(&d0);
        let d01 = d0.dot(&d1);
        let d02 = d0.dot(&d2);
        let d11 = d1.dot(&d1);
        let d12 = d1.dot(&d2);

        let denom = d00 * d11 - d01 * d01;
        if is_zero(denom) {
            return false;
        }

        let denom = N::one() / denom;
        let u = (d11 * d02 - d01 * d12) * denom;
        let v = (d00 * d12 - d01 * d02) * denom;
        u >= N::zero() && v >= N::zero() && u + v < N::one()
    }
    pub fn to_plane(&self) -> Plane<N> {
        let normal = self.normal();
        Plane {
            normal,
            d: -normal.dot(&self.point1),
        }
    }
    pub fn distance(&self, point: &Vector3<N>) -> N {
        (point - self.point1).dot(&self.normal())
    }
    fn get_x<F: FnMut(N, &N) -> N>(&self, f: F) -> N {
        [self.point1.x, self.point2.x].iter().fold(self.point3.x, f)
    }
    pub fn min_x(&self) -> N {
        self.get_x(|acc, f| n_min(acc, *f))
    }
    pub fn max_x(&self) -> N {
        self.get_x(|acc, f| n_max(acc, *f))
    }
    fn get_y<F: FnMut(N, &N) -> N>(&self, f: F) -> N {
        [self.point1.y, self.point2.y].iter().fold(self.point3.y, f)
    }
    pub fn min_y(&self) -> N {
        self.get_y(|acc, f| n_min(acc, *f))
    }
    pub fn max_y(&self) -> N {
        self.get_y(|acc, f| n_max(acc, *f))
    }
    fn get_z<F: FnMut(N, &N) -> N>(&self, f: F) -> N {
        [self.point1.z, self.point2.z].iter().fold(self.point3.z, f)
    }
    pub fn min_z(&self) -> N {
        self.get_z(|acc, f| n_min(acc, *f))
    }
    pub fn max_z(&self) -> N {
        self.get_z(|acc, f| n_max(acc, *f))
    }
    pub fn iter(&self) -> impl Iterator<Item = N> + '_ {
        self.point1
            .iter()
            .chain(self.point2.iter())
            .chain(self.point3.iter())
            .cloned()
    }
}

impl<N: PhysicsScalar> Shape3D<N> for Triangle<N> {
    fn bounding_aabb(&self) -> AxisAlignedBoundingBox<N> {
        AxisAlignedBoundingBox {
            start: Vector3::new(self.min_x(), self.min_y(), self.min_z()),
            end: Vector3::new(self.max_x(), self.max_y(), self.max_z()),
        }
    }
    fn bounding_sphere(&self) -> Sphere<N> {
        self.bounding_aabb().bounding_sphere()
    }
    fn center(&self) -> Vector3<N> {
        (self.point1 + self.point2 + self.point3) / N::from_usize(3).unwrap()
    }
    fn translate(&self, point: &Vector3<N>) -> Self {
        Triangle {
            point1: self.point1 + point,
            point2: self.point2 + point,
            point3: self.point3 + point,
        }
    }
    fn translate_mut(&mut self, value: &Vector3<N>) {
        self.point1 += value;
        self.point2 += value;
        self.point3 += value;
    }
    fn set_center(&self, point: &Vector3<N>) -> Self {
        let mut t = *self;
        t.set_center_mut(point);
        t
    }
    fn set_center_mut(&mut self, new_center: &Vector3<N>) {
        let center = self.center();
        let dist1 = self.point1 - center;
        let dist2 = self.point2 - center;
        let dist3 = self.point3 - center;
        self.point1 = new_center + dist1;
        self.point2 = new_center + dist2;
        self.point3 = new_center + dist3;
    }
}
