use crate::v2d::V2D;
use crate::world::{World, WorldConfig};
use crate::model::Model;

/*
 * The Sim struct applies all the mathematical calculations needed to run the model.
 */
pub struct Sim {
    pub dt: f64,
}

impl Sim {
    pub fn new(dt: f64) -> Self {
        Self { dt }
    }

    // Simulation step to calculate and update the model.
    pub fn step(&self, model: &mut Model, world: &World, w_cfg: &WorldConfig) {
        // Force application.
        model.clear_forces();
        model.apply_spring_f(self.dt);
        model.apply_world_f(w_cfg, self.dt);

        // Verlet integration
        let dt2 = self.dt * self.dt;
        for mass in model.masses_mut() {
            if mass.fixed { continue; }

            // Initial Verlet calculation.
            let m_pi_o = mass.p_i; // save p_i before the history shift
            let mut v_new = V2D::null(); // for setting the new velocity on reflection.
            let ver_p = mass.p_i * 2.0 - mass.p_o + (mass.f / mass.m) * dt2;
            let mut collided = false;
            let mut mpos = ver_p;

            // Detect and apply boundary collisions.
            for bound in &world.bounds {
                let seg_mb = mpos - bound.pos;
                // distance from boundary
                let dist_signed = seg_mb.dot(bound.nrm);
                // length of distance.
                let dist = dist_signed.abs();

                // Correct and reflect on overlap.
                if dist < mass.r {
                    let overlap = mass.r - dist;
                    let push_dir = if dist_signed > 0.0 { -bound.nrm } else { bound.nrm };
                    // Correct position.
                    mpos += push_dir * overlap;
                    let v = (mpos - mass.p_o) / self.dt;
                    let v_n = v.dot(bound.nrm);

                    if v_n < 0.0 {
                        let impulse = -(1.0 + bound.refl) * v_n;
                        v_new = v + bound.nrm * impulse;
                        collided = true;
                    }
                }
            }

            mass.p_o = m_pi_o;
            mass.p_i = mpos;

            // If the mass had an interaction with the boundary, correct p_o.
            if collided {
                mass.p_o = mass.p_i - v_new * self.dt;
            }
        }

    }
}
