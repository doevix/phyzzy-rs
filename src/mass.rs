use crate::v2d::V2D;

/*
 * Masses are the parts of the model that are affected by forces and are
 * subject to acceleration.
 */
#[derive(Clone, Copy, Default, Debug)]
pub struct Mass {
    pub m: f64,         // mass of the Mass [kg]
    pub r: f64,         // radius of the Mass [kg]
    pub f: V2D,         // sum of forces applied to mass [N]
    pub p_i: V2D,       // current position of mass
    pub p_o: V2D,       // position of mass in last simulation frame
    pub fixed: bool,    // Determines if mass is fixed.

    // TODO add these when dynamic collisions are enabled.
    // refl: f64,      // Reflection, for mass-mass collisions.
    // mu_s: f64,      // Surface stiction.
    // mu_k: f64,      // Surface sliction
}

impl Mass {
    // Creates a new mass, with no velocity.
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

    // Creates a mass, with a set velocity.
    pub const fn load(m: f64, r: f64, p_i: &V2D, p_o: &V2D) -> Self {
        Self {
            m,
            r,
            f: V2D::null(),
            p_i: V2D::from(p_i),
            p_o: V2D::from(p_o),
            fixed: false,
        }
    }

    // calculate velocity (p_i - p_o) / dt
    pub fn vel(&self, dt: f64) -> V2D {
        // self.p_i.sub(&self.p_o).scale(1.0 / dt)
        (self.p_i - self.p_o) / dt
    }
}
