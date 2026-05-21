use crate::v2d::V2D;
use crate::mass::Mass;
use crate::spring::Spring;
use crate::world::{ World, WorldConfig };

#[derive(Debug, Clone, PartialEq)]
pub enum PhyzzyModelError {
    SelfConnection(usize),
    OutOfBounds { spring_ma: usize, spring_mb: usize, n_masses: usize },
}

impl std::fmt::Display for PhyzzyModelError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::SelfConnection(idx) => write!(f, "Spring cannot connect mass {} to itself", idx),
            Self::OutOfBounds { spring_ma, spring_mb, n_masses } => {
                write!(f, "Spring indices ({}, {}) out of bounds (n_masses = {})",
                       spring_ma, spring_mb, n_masses)

            }
        }
    }
}

impl std::error::Error for PhyzzyModelError {}

/*
 * Actuators follow a waveform and change the spring properties.
 */
pub enum SpringActuator {
    SpringClassicMuscle {
        spring: usize,
        phase: f64,
        sense: f64,
    },
    SpringRelaxationMuscle{
        spring: usize,
        phase: f64,
        sense: f64,
    },
}

pub enum MassActuator {
    MassBalloon {
        mass: usize,
        phase: f64,
        sense: f64,
    },
    MassTank {
        mass: usize,
        phase: f64,
        sense: f64,
    },
}

/*
 * The model struct holds the model made of springs and masses.
 */
pub struct Model {
    pub wave_speed: f64,
    pub wave_amplitude: f64,
    pub muscles: Vec<SpringActuator>,
    pub bladders: Vec<MassActuator>,
    masses: Vec<Mass>,
    springs: Vec<Spring>,

}

impl Model {
    pub fn new(wave_speed: f64, wave_amplitude: f64) -> Self {
        Self {
            wave_speed, wave_amplitude,
            muscles: Vec::new(),
            bladders: Vec::new(),
            masses: Vec::new(),
            springs: Vec::new(),
        }
    }

    pub fn new_mass(&mut self, mass: Mass) {
        self.masses.push(mass);
    }

    pub fn new_spring(&mut self, spring: Spring) -> Result<(), PhyzzyModelError> {
        let a = spring.get_ma();
        let b = spring.get_mb();
        let len = self.masses.len();

        if a == b {
            return Err(PhyzzyModelError::SelfConnection(a));
        }
        if a >= len || b >= len {
            return Err(PhyzzyModelError::OutOfBounds {
                spring_ma: a,
                spring_mb: b,
                n_masses: len

            });
        }

        self.springs.push(spring);
        Ok(())

    }

    pub fn del_mass(&mut self, i_m: usize) {
        if i_m >= self.masses.len() { return; }

        let tail_idx = self.masses.len() - 1;
        // swap at index with last element and remove it.
        self.masses.swap_remove(i_m);

        // for clean removal, springs with deleted mass need to be collected
        let mut to_del = Vec::new();
        for (idx, spring) in self.springs.iter().enumerate() {
            if spring.get_ma() == i_m || spring.get_mb() == i_m {
                to_del.push(idx);
            }
        }

        // delete collected springs in reverse order.
        for rem in to_del.into_iter().rev() {
            self.springs.swap_remove(rem);
        }

        // renumber remaining springs.
        for spring in &mut self.springs {
            if spring.get_ma() == tail_idx { spring.set_ma(i_m); }
            if spring.get_mb() == tail_idx { spring.set_mb(i_m); }
        }

    }

    pub fn del_spring(&mut self, i_s: usize) {
        if i_s < self.springs.len() { self.springs.swap_remove(i_s); }
    }

    // Returns the masses, avoid modifying externally.
    pub fn get_masses(&self) -> &[Mass] {
        &self.masses
    }

    // Returns the springs, avoid modifying externally.
    pub fn get_springs(&self) -> &[Spring] {
        &self.springs
    }

    pub fn get_mass(&self, idx: usize) -> Mass {
        self.masses[idx]
    }

    pub fn get_spring(&self, idx: usize) -> Spring {
        self.springs[idx]
    }

    pub fn set_mass_vel(&mut self, idx: usize, vel: V2D, dt: f64) {
        self.masses[idx].set_vel(vel, dt);
    }

    // Simulation step to calculate and update the model.
    pub fn step(&mut self, dt: f64, world: &World, w_cfg: &WorldConfig) {
        let dt2 = dt * dt;

        // Force application.
        self.clear_forces();
        self.apply_spring_f(dt);
        self.apply_world_f(world, w_cfg, dt);

        // Verlet integration
        for mass in &mut self.masses {
            if mass.fixed { continue; }

            // Verlet calculation.
            let p_i_o = mass.p_i;
            mass.p_i = mass.p_i * 2.0 - mass.p_o + (mass.f / mass.m) * dt2;
            mass.p_o = p_i_o;

            // Boundary collisions.
            for bound in &world.bounds {
                // Catch boundary crossing
                let bound_unit = bound.nrm.prp();
                let bound_pos_to_mass = (mass.p_i - bound.pos).pjt(bound_unit);
                let vec_rad = mass.r * -bound.nrm;
                let pos_bm = bound_pos_to_mass + bound.pos;
                let check_side = (mass.p_i + vec_rad - pos_bm).dot(bound.nrm);

                if check_side < 0.0 {
                    let vel = mass.vel(dt); // Velocity before reflection.
                    // Correct positions.
                    mass.p_i = pos_bm + (mass.r * bound.nrm);
                    mass.p_o = mass.p_i;

                    // Apply reflections.
                    let v_pjt_b = vel.pjt(bound_unit);
                    let v_pjt_n = vel.pjt(bound.nrm);
                    let refl_vel = v_pjt_b - (bound.refl * v_pjt_n);
                    mass.set_vel(refl_vel, dt);
                }

            }


        }

    }
    pub fn clear_forces(&mut self) {
        for mass in &mut self.masses {
            mass.f = V2D::null();
        }
    }

    pub fn apply_spring_f(&mut self, dt: f64) {
        for spring in &mut self.springs {
            let a = &self.masses[spring.get_ma()];
            let b = &self.masses[spring.get_mb()];
            let ab = b.p_i - a.p_i;
            let l = ab.mag();
            let v_a = a.p_i - a.p_o;
            let v_b = b.p_i - b.p_o;

            // Force calculations for springing and dampening.
            let f_spr = (ab / l) * (l - spring.r) * spring.k;
            let f_dmp = ((v_b - v_a) / dt).pjt(ab) * spring.d;

            self.masses[spring.get_ma()].f += f_spr + f_dmp;
            self.masses[spring.get_mb()].f -= f_spr + f_dmp;
        }
    }

    pub fn apply_world_f(&mut self, world: &World, w_cfg: &WorldConfig, dt: f64) {
        for mass in &mut self.masses {
            let f_weight = w_cfg.gravity * mass.m;
            let f_drag = mass.vel(dt) * -w_cfg.drag;

            mass.f += f_weight + f_drag;

            for bound in &world.bounds {
                // TODO: Apply friction and surface-related force calculations here!
                // Detect if mass is touching the boundary.
                // Catch boundary crossing
                let mass_to_boundp = mass.p_i - bound.pos;
                let seg_dist_signed = mass_to_boundp.dot(bound.nrm);
                let seg_dist_abs = seg_dist_signed.abs();
                let penetration = seg_dist_signed - mass.r;


                // Determine if mass is moving or sitting still relative to the boundary.

                // Apply forces.
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::v2d::V2D;

    #[test]
    fn test_add_spring_out_of_bounds() {
        let mut model = Model::new(0.0, 0.0);
        model.new_mass(Mass::new(1.0, 0.5, &V2D::new(0.0, 0.0)));
        let result = model.new_spring(Spring::new(1.0, 0.0, 0.0, 0, 1));

        assert!(result.is_err());
        assert_eq!(
            result.unwrap_err(),
            PhyzzyModelError::OutOfBounds {
                spring_ma: 0,
                spring_mb: 1,
                n_masses: 1
            }
        );
    }

    #[test]
    fn test_add_self_connection() {
        let mut model = Model::new(0.0, 0.0);
        model.new_mass(Mass::new(1.0, 0.5, &V2D::new(0.0, 0.0)));

        let result = model.new_spring(Spring::new(1.0, 0.0, 0.0, 0, 0));

        assert_eq!(result.unwrap_err(), PhyzzyModelError::SelfConnection(0)); // trying to connect to same mass.
    }


    #[test]
    fn test_remove_mass_cleans_springs() {
        let mut model = Model::new(0.0, 0.0);
        model.new_mass(Mass::new(1.0, 0.1, &V2D::new(0.0, 0.0)));
        model.new_mass(Mass::new(1.0, 0.1, &V2D::new(1.0, 0.0)));
        model.new_mass(Mass::new(1.0, 0.1, &V2D::new(2.0, 0.0)));

        let ins_result = model.new_spring(Spring::new(1.0, 50.0, 0.0, 1, 2));


        model.del_mass(1); // removes middle mass

        // Spring should be gone
        assert!(ins_result.is_ok());
        assert_eq!(model.springs.len(), 0);
    }
}
