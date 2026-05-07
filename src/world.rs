use crate::v2d::V2D;
/*
 * WorldConfig contains the values that the world
 */
#[derive(Clone, Copy, Default)]
pub struct WorldConfig {
    pub gravity: V2D,      // gravitational acceleration [m/s^2]
    pub drag: f64,         // drag coefficient
}

/*
 * The World is the environment the model lives in. Applies boundaries where needed.
 */
pub struct World {
    pub bounds: Vec<Boundary>,
}

impl World {
    pub fn new() -> Self {
        Self {
            bounds: Vec::new()
        }
    }
}

/*
 * Boundaries constrain the model to a surface.
 */
pub struct Boundary {
    pub pos: V2D,       // position where the boundary is located
    pub nrm: V2D,       // surface normal.
    pub refl: f64,      // boundary surface reflection (restitution)
    pub mu_s: f64,      // coefficient of static surface friction (stiction)
    pub mu_k: f64,      // coefficient of sliding surface friction (sliction?)
}

impl Boundary {
    pub fn new(pos: V2D, nrm: V2D, refl: f64, mu_s: f64, mu_k: f64) -> Self {
        Self {
            pos, refl, mu_s, mu_k,
            nrm: nrm.unit(), // Ensure this is a unit vector.
        }
    }
}
