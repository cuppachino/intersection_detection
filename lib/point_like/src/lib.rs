extern crate num;
use num::Float;

/// A number that may be compared for approximate equality.
pub trait ApproxEq<Rhs = Self, Epsilon = Self> {
    fn approx_eq(&self, rhs: Rhs, epsilon: Epsilon) -> bool;
}

impl<F> ApproxEq<Self, Self> for F
where
    F: Float,
{
    fn approx_eq(&self, rhs: Self, epsilon: Self) -> bool {
        (self.sub(rhs)).abs() < epsilon
    }
}

/// A trait for checking if a value is between two other values.
pub trait Between: PartialOrd + Sized {
    /// Returns `true` if self lies between `start` and `end`.
    #[inline]
    fn is_between(self, start: Self, end: Self) -> bool {
        self.ge(&start) && self.le(&end)
    }

    /// Returns `true` if self lies beyond `start` and `end`.
    #[inline]
    fn is_not_between(self, start: Self, end: Self) -> bool {
        self.lt(&start) || self.gt(&end)
    }
}
impl<F: Float> Between for F {}

/// A trait for rounding scalars.
pub trait PrecisionRound {
    /// Precision round a scalar to a number of decimal places.
    fn round(self, precision: usize) -> Self;
}

impl<F: Float> PrecisionRound for F {
    /// Round a scalar to `precision` decimal places.
    ///
    /// # Panics
    ///
    /// Panics if the precision factor cannot be represented by `F`.
    #[inline]
    fn round(self, precision: usize) -> Self {
        let factor = F::from(10.0_f32.powi(precision as i32)).unwrap();
        (self * factor).round() / factor
    }
}

/// A point-like type that can be converted to and from `[F; 2]`.
///
/// Any third component is assumed to be zero.
pub trait FromIntoPointLike<T>: Sized {
    fn from_point_like(t: T) -> Self;
    fn into_point_like(self) -> T;
}

impl<F: Float> FromIntoPointLike<[F; 2]> for [F; 2] {
    #[inline]
    fn from_point_like(t: [F; 2]) -> Self {
        [t[0], t[1]]
    }
    #[inline]
    fn into_point_like(self) -> [F; 2] {
        [self[0], self[1]]
    }
}

impl<F: Float> FromIntoPointLike<[F; 2]> for [F; 3] {
    #[inline]
    fn from_point_like(t: [F; 2]) -> Self {
        [t[0], t[1], F::zero()]
    }
    #[inline]
    fn into_point_like(self) -> [F; 2] {
        [self[0], self[1]]
    }
}

impl<F: Float> FromIntoPointLike<[F; 2]> for (F, F) {
    #[inline]
    fn from_point_like(t: [F; 2]) -> Self {
        (t[0], t[1])
    }
    #[inline]
    fn into_point_like(self) -> [F; 2] {
        [self.0, self.1]
    }
}

impl<F: Float> FromIntoPointLike<[F; 2]> for (F, F, F) {
    #[inline]
    fn from_point_like(t: [F; 2]) -> Self {
        (t[0], t[1], F::zero())
    }
    #[inline]
    fn into_point_like(self) -> [F; 2] {
        [self.0, self.1]
    }
}

/// A point-like type that can be used to define a coordinate in 2D space.
pub trait PointLike<N: Float>: Copy + PartialEq + FromIntoPointLike<[N; 2]> {
    fn x(&self) -> N;
    fn y(&self) -> N;
    fn with_x(&self, x: N) -> Self;
    fn with_y(&self, y: N) -> Self;
    fn with_xy(x: N, y: N) -> Self;
}

impl<T, F: Float> PointLike<F> for T
where
    T: Copy + PartialEq + FromIntoPointLike<[F; 2]>,
{
    fn with_x(&self, x: F) -> Self {
        Self::from_point_like([x, self.y()])
    }

    fn with_xy(x: F, y: F) -> Self {
        Self::from_point_like([x, y])
    }

    fn with_y(&self, y: F) -> Self {
        Self::from_point_like([self.x(), y])
    }

    fn x(&self) -> F {
        self.into_point_like()[0]
    }

    fn y(&self) -> F {
        self.into_point_like()[1]
    }
}

pub trait PointOps<F: Float> {
    fn add(self, rhs: Self) -> Self;

    fn sub(self, rhs: Self) -> Self;

    fn mul(self, rhs: Self) -> Self;
}

impl<F: Float, P: PointLike<F>> PointOps<F> for P {
    #[inline]
    fn add(self, other: Self) -> Self {
        Self::from_point_like([self.x() + other.x(), self.y() + other.y()])
    }

    #[inline]
    fn sub(self, other: Self) -> Self {
        Self::from_point_like([self.x() - other.x(), self.y() - other.y()])
    }

    #[inline]
    fn mul(self, rhs: Self) -> Self {
        Self::from_point_like([self.x() * rhs.x(), self.y() * rhs.y()])
    }
}

pub trait PointOpsExt<F: Float>: PointLike<F> {
    /// Construct a new point using the same value for both components of self.
    #[inline]
    fn splat(value: F) -> Self {
        Self::with_xy(value, value)
    }

    /// Add a scalar to both components of a point.
    #[inline]
    fn add_f(self, scalar: F) -> Self {
        Self::with_xy(self.x() + scalar, self.y() + scalar)
    }

    /// Subtract a scalar from both components of a point.
    #[inline]
    fn sub_f(self, scalar: F) -> Self {
        Self::with_xy(self.x() - scalar, self.y() - scalar)
    }

    /// Multiply a point by a scalar.
    #[inline]
    fn mul_f(self, scalar: F) -> Self {
        Self::with_xy(self.x() * scalar, self.y() * scalar)
    }

    /// Fused multiply-add. Computes (self * a) + b with only one rounding error, yielding a more accurate result than an unfused multiply-add.
    ///
    /// Using mul_add can be more performant than an unfused multiply-add if the target architecture has a dedicated fma CPU instruction.
    #[inline]
    fn mul_add(self, a: Self, b: Self) -> Self {
        Self::with_xy(
            self.x().mul_add(a.x(), b.x()),
            self.y().mul_add(a.y(), b.y()),
        )
    }

    /// Construct a point from the absolute components of self.
    #[inline]
    fn abs(self) -> Self {
        Self::with_xy(self.x().abs(), self.y().abs())
    }

    /// Compare all components in self with `rhs` for approximate equality.
    #[inline]
    fn approx_eq(self, rhs: Self, epsilon: F) -> bool {
        self.sub(rhs).abs().lt(&epsilon)
    }

    /// Compare all components in a point with a scalar for approximate equality.
    #[inline]
    fn approx_eq_f(self, scalar: F, epsilon: F) -> bool {
        self.sub_f(scalar).abs().lt(&epsilon)
    }

    /// Round the components of a point to N decimal places.

    #[inline]
    fn round(self, n: usize) -> Self {
        let factor = F::from(10).unwrap().powi(n as i32);
        let x = (self.x() * factor).round() / factor;
        let y = (self.y() * factor).round() / factor;
        Self::with_xy(x, y)
    }

    /// Returns a point-like containing the maximum values for each element of self and `rhs`.
    ///
    /// In other words this computes `[self.x().max(rhs.x), self.y().max(rhs.y())]`.
    #[inline]
    fn max(self, other: Self) -> Self {
        Self::with_xy(self.x().max(other.x()), self.y().max(other.y()))
    }

    /// Returns a point-like containing the minimum values for each element of self and `rhs`.
    ///
    /// In other words this computes `[self.x.min(rhs.x), self.y.min(rhs.y)]`.
    #[inline]
    fn min(self, other: Self) -> Self {
        Self::with_xy(self.x().min(other.x()), self.y().min(other.y()))
    }

    /// Compute the cross product of two points.
    #[inline]
    fn cross(self, rhs: Self) -> F {
        self.x() * rhs.y() - self.y() * rhs.x()
    }

    /// Computes the dot of self and `rhs`.
    ///
    /// ```maths
    /// a · b = (x₁ · x₂) + (y₁ · y₂)
    /// ```
    #[inline]
    fn dot(self, rhs: Self) -> F {
        self.x() * rhs.x() + self.y() * rhs.y()
    }

    /// Computes the length of self.
    ///
    /// ```maths
    /// c = √(a² + b²)
    /// ```
    #[doc(alias = "magnitude")]
    #[inline]
    fn length(self) -> F {
        self.dot(self).sqrt()
    }

    /// Computes the squared length of self.
    /// This is generally faster than `length` as it avoids the square root.
    #[doc(alias = "magnitude_squared")]
    #[inline]
    fn length_squared(self) -> F {
        self.dot(self)
    }

    /// Computes the Euclidean distance between two points in space.
    ///
    /// This works by relocating one point to the origin,
    /// then computing the length of the resulting vector.
    ///
    /// ```maths
    /// c = √((x₂ - x₁)² + (y₂ - y₁)²)
    /// ```
    #[doc(alias = "length_squared")]
    #[inline]
    fn distance(self, rhs: Self) -> F {
        (self.sub(rhs)).length()
    }

    #[inline]
    fn partial_cmp(&self, other: &F) -> Option<std::cmp::Ordering> {
        let x = self.x().partial_cmp(other);
        let y = self.y().partial_cmp(other);
        match (x, y) {
            (Some(std::cmp::Ordering::Equal), Some(std::cmp::Ordering::Equal)) => {
                Some(std::cmp::Ordering::Equal)
            }
            (Some(std::cmp::Ordering::Equal), _) => y,
            (_, Some(std::cmp::Ordering::Equal)) => x,
            (Some(std::cmp::Ordering::Less), Some(std::cmp::Ordering::Less)) => {
                Some(std::cmp::Ordering::Less)
            }
            (Some(std::cmp::Ordering::Greater), Some(std::cmp::Ordering::Greater)) => {
                Some(std::cmp::Ordering::Greater)
            }
            _ => None,
        }
    }

    #[inline]
    fn lt(&self, other: &F) -> bool {
        self.partial_cmp(other) == Some(std::cmp::Ordering::Less)
    }

    #[inline]
    fn gt(self, other: &F) -> bool {
        self.partial_cmp(other) == Some(std::cmp::Ordering::Greater)
    }

    #[inline]
    fn le(self, other: &F) -> bool {
        self.partial_cmp(other) != Some(std::cmp::Ordering::Greater)
    }

    #[inline]
    fn ge(self, other: &F) -> bool {
        self.partial_cmp(other) != Some(std::cmp::Ordering::Less)
    }

    #[inline]
    fn eq(self, other: &F) -> bool {
        self.partial_cmp(other) == Some(std::cmp::Ordering::Equal)
    }

    #[inline]
    fn ne(self, other: &F) -> bool {
        self.partial_cmp(other) != Some(std::cmp::Ordering::Equal)
    }
}

impl<F: Float, P: PointLike<F>> PointOpsExt<F> for P {}
