/*
* 2D vector struct. Using derive attributes to avoid wierd or complicated code.
*/
use std::ops::{Add, Sub, Mul, Div, Neg, AddAssign, SubAssign, MulAssign};

#[derive(Clone, Copy, Default, PartialEq, Debug)]
pub struct V2D {
    pub x: f64,
    pub y: f64,
}

impl V2D {
    // Constructor
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    // Null vector constructor, for shorter syntax.
    pub const fn null() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
        }
    }

    // Create a new vector from a borrowed vector.
    pub const fn from(v: &V2D) -> Self {
        Self {
            x: v.x,
            y: v.y,
        }
    }

    // Calculate dot product.
    pub fn dot(&self, r: V2D) -> f64 {
        self.x * r.x + self.y * r.y
    }

    // Magnitude squared of the vector.
    pub fn mag2(&self) -> f64 {
        self.x * self.x + self.y * self.y
    }

    // Magnitude of the vector.
    pub fn mag(&self) -> f64 {
        self.mag2().sqrt()
    }

    // Project to another vector.
    pub fn pjt(&self, r: V2D) -> V2D {
        if r.mag2() < 1e-12 { return V2D::null() };
        r * (self.dot(r) / r.mag2())
    }

    // Normalization, get unit vector.
    pub fn unit(self) -> V2D {
        let mag = self.mag();
        if mag < 1e-12 { return V2D:: null() };
        self / mag
    }

    // Compare with another vector.
    pub fn are_equal(&self, c: V2D) -> bool {
        self.x == c.x && self.y == c.y
    }
}

/*
 * The following allows the vectors to use simple operators.
 */
impl Add for V2D {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl Sub for V2D {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl Mul<f64> for V2D {
    type Output = Self;
    fn mul(self, s: f64) -> Self {
        Self {
            x: self.x * s,
            y: self.y * s,
        }
    }
}

impl Div<f64> for V2D {
    type Output = Self;
    fn div(self, s: f64) -> Self {
        Self {
            x: self.x / s,
            y: self.y / s,
        }
    }
}

impl Neg for V2D {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
        }
    }
}

impl AddAssign for V2D {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl SubAssign for V2D {
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl MulAssign for V2D {
    fn mul_assign(&mut self, rhs: Self) {
        self.x *= rhs.x;
        self.y *= rhs.y;
    }
}
