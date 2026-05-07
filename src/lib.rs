pub mod v2d;

pub use v2d::V2D;

/*
 * Collision notes:
 * reflections and mu_k, mu_s in mass should be obeyed by mass-mass collisions.
 * reflections and mu_k, mu_s in spring should be obeyed by spring-mass collisions.
 * reflections and mu_k, mu_s in boundary should be obeyed by mass-surface collisions.
 *
 * Spring-spring and spring-surface collisions should not be possible thus should not
 * be considered.
 */

/*
 * Masses are the parts of the model that are affected by forces and are
 * subject to acceleration.
 */
#[derive(Clone, Copy, Default)]
struct Mass {
    m: f64,         // mass of the Mass [kg]
    r: f64,         // radius of the Mass [kg]
    f: V2D,         // sum of forces applied to mass [N]
    p_i: V2D,       // current position of mass
    p_o: V2D,       // position of mass in last simulation frame
    fixed: bool,    // Determines if mass is fixed.

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

/*
 * Springs interconnect Masses and apply forces according to Hooke's law with
 * dampening.
 */
#[derive(Clone, Copy, Default)]
struct Spring {
    pub r: f64,         // restlength of the Spring [m]
    pub k: f64,         // Hooke's law constant
    pub d: f64,         // dampening factor
    m_a: usize,         // Mass A index
    m_b: usize,         // Mass B index

    // TODO add these somehow when dynamic collisions are added.
    // pub mu_s: f64,       // Spring stiction when collisions enabled.
    // pub mu_k: f64,       // Spring sliction when collisions enabled.
}

impl Spring {
    pub fn new(r: f64, k: f64, d: f64, m_a: usize, m_b: usize) -> Self {
        Self { r, k, d, m_a, m_b }
    }
}

/*
 * WorldConfig contains the values that the world
 */
#[derive(Clone, Copy, Default)]
struct WorldConfig {
    pub gravity: V2D,      // gravitational acceleration [m/s^2]
    pub drag: f64,         // drag coefficient
}

/*
 * The World is the environment the model lives in. Applies boundaries where needed.
 */
struct World {
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
struct Boundary {
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

/*
 * The model struct holds the model made of springs and masses.
 */
struct Model {
    pub wave_speed: f64,
    pub wave_amplitude: f64,
    masses: Vec<Mass>,
    springs: Vec<Spring>,
}

impl Model {
    pub fn new(wave_speed: f64, wave_amplitude: f64) -> Self {
        Self {
            wave_speed, wave_amplitude,
            masses: Vec::new(),
            springs: Vec::new(),
        }
    }

    pub fn new_mass(&mut self, mass: Mass) {
        self.masses.push(mass);
    }

    pub fn new_spring(&mut self, spring: Spring) {
        self.springs.push(spring);
    }

    pub fn del_mass(&mut self, i_m: usize) {
        if i_m >= self.masses.len() { return; }

        let tail_idx = self.masses.len() - 1;
        // swap at index with last element and remove it.
        self.masses.swap_remove(i_m);

        // for clean removal, springs with deleted mass need to be collected
        let mut to_del = Vec::new();
        for (idx, spring) in self.springs.iter().enumerate() {
            if spring.m_a == i_m || spring.m_b == i_m {
                to_del.push(idx);
            }
        }

        // delete collected springs in reverse order.
        for rem in to_del.into_iter().rev() {
            self.springs.swap_remove(rem);
        }

        // renumber remaining springs.
        for spring in &mut self.springs {
            if spring.m_a == tail_idx { spring.m_a = i_m; }
            if spring.m_b == tail_idx { spring.m_b = i_m; }
        }

    }

    pub fn del_spring(&mut self, i_s: usize) {
        if i_s < self.springs.len() { self.springs.swap_remove(i_s); }
    }

    pub fn clear_forces(&mut self) {
        for mass in &mut self.masses {
            mass.f = V2D::null();
        }
    }

    pub fn apply_spring_f(&mut self, dt: f64) {
        for spring in &mut self.springs {
            let a = &self.masses[spring.m_a];
            let b = &self.masses[spring.m_b];
            let ab = b.p_i - a.p_i;
            let l = ab.mag();
            let v_a = a.p_i - a.p_o;
            let v_b = b.p_i - b.p_o;

            // Force calculations for springing and dampening.
            let f_spr = (ab / l) * (spring.r - l) * spring.k;
            let f_dmp = ((v_b - v_a) / dt).pjt(ab) * spring.d;

            self.masses[spring.m_a].f += f_spr + f_dmp;
            self.masses[spring.m_b].f -= f_spr + f_dmp;
        }
    }

    pub fn apply_world_f(&mut self, w_cfg: &WorldConfig, dt: f64) {
        for mass in &mut self.masses {
            let f_weight = w_cfg.gravity * mass.m;
            let f_drag = mass.vel(dt) * -w_cfg.drag;

            mass.f += f_weight + f_drag;
        }
    }
}

/*
 * The Sim struct applies all the mathematical calculations needed to run the model.
 */
struct Sim {
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
        for mass in &mut model.masses {
            if mass.fixed { continue; }

            // Provisional Verlet calculation.
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
