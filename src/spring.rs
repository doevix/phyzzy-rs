/// Springs interconnect Masses and apply forces according to Hooke's law with dampening.
#[derive(Clone, Copy, Default, Debug)]
pub struct Spring {
    /// Restlength of the Spring \[m\]
    pub r: f64,
    /// Hooke's law constant, or "springyness". \[N/m\]
    pub k: f64,
    /// dampening factor \[Ns/m\]
    pub d: f64,
    /// reflection
    pub refl: f64,

    m_a: usize,         // Mass A index
    m_b: usize,         // Mass B index
    f_spring: f64,      // The springing force applied. For display purposes only.
    f_dampen: f64,      // The dampening force applied. For display purposes only.
    cur_length: f64,    // The current spring length. For display purposes only.

    // TODO add these somehow when dynamic collisions are added.
    pub mu_s: f64,       // Spring stiction when collisions enabled.
    pub mu_k: f64,       // Spring sliction when collisions enabled.
}

impl Spring {
    /// Creates a new spring that connects 2 masses by index in the model.
    pub fn new(r: f64, k: f64, d: f64, m_a: usize, m_b: usize) -> Self {
        Self {
            r, k, d, m_a, m_b,
            f_spring: 0.0,
            f_dampen: 0.0,
            cur_length: 0.0,
            refl: 0.8,
            mu_s: 0.6,
            mu_k: 0.3
        }
    }

    /// Get mass index a.
    pub fn get_ma(&self) -> usize {
        self.m_a
    }
    /// Get mass index b.
    pub fn get_mb(&self) -> usize {
        self.m_b
    }

    /// Display springing force being applied.
    pub fn get_springing(&self) -> f64 {
        self.f_spring
    }

    /// Display dampening force being applied.
    pub fn get_dampening(&self) -> f64 {
        self.f_dampen
    }

    /// Display the spring's current length.
    pub fn get_current_length(&self) -> f64 {
        self.cur_length
    }

    pub(crate) fn set_springing(&mut self, f_spr_mag: f64) {
        self.f_spring = f_spr_mag;
    }

    pub(crate) fn set_dampening(&mut self, f_dmp_mag: f64) {
        self.f_dampen = f_dmp_mag;
    }

    pub(crate) fn set_cur_length(mut self, cur_len: f64) {
        self.cur_length = cur_len;
    }

    // Change mass index a.
    pub(crate) fn set_ma(&mut self, m_a: usize) {
        self.m_a = m_a;
    }
    // Change mass index b.
    pub(crate) fn set_mb(&mut self, m_b: usize) {
        self.m_b = m_b;
    }
}
