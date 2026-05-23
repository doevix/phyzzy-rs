use crate::v2d::V2D;

/// Masses are the parts of the model that are affected by forces and are subject to acceleration.
/// This type of mass is governed by a verlet integrator, thus the velocity is implicit: v = (p_i - p_o) / dt
#[derive(Clone, Copy, Default, Debug)]
pub struct Mass {
    /// mass of the Mass \[kg\]
    pub m: f64,

    /// radius of the Mass \[m\]
    pub r: f64,

    /// sum of forces applied to Mass \[N\]
    pub f: V2D,

    /// current position of Mass
    pub p_i: V2D,

    /// position of Mass in last simulation frame
    pub p_o: V2D,

    /// When true, indicates whether the model should ignore in calculation step.
    pub fixed: bool,

    // TODO add these when dynamic collisions are enabled.
    // refl: f64,      // Reflection, for mass-mass collisions.
    // mu_s: f64,      // Surface stiction.
    // mu_k: f64,      // Surface sliction
}

impl Mass {
    /// Creates a new mass, with no velocity.
    pub const fn new(m: f64, r: f64, pos: &V2D) -> Self {
        Self {
            m,
            r,
            f: V2D::null(),
            p_i: V2D::from(pos),
            p_o: V2D::from(pos),
            fixed: false,
        }
    }

    /// Creates a new mass with a set implicit velocity.
    pub const fn load(m: f64, r: f64, pos: &V2D, pos_prv: &V2D) -> Self {
        Self {
            m,
            r,
            f: V2D::null(),
            p_i: V2D::from(pos),
            p_o: V2D::from(pos_prv),
            fixed: false,
        }
    }

    /// Calculate implicit velocity (p_i - p_o) / dt
    pub fn vel(&self, dt: f64) -> V2D {
        // self.p_i.sub(&self.p_o).scale(1.0 / dt)
        (self.p_i - self.p_o) / dt
    }

    /// Return the difference between the current and previous positions.
    pub fn diff_p(&self) -> V2D {
        self.p_i - self.p_o
    }

    /// Sets the mass's implicit velocity to a given value. Requires time delta.
    pub fn set_vel(&mut self, vel: V2D, dt: f64) {
        self.p_o = self.p_i - (vel * dt);
    }
}
