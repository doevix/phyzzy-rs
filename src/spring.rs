/*
 * Springs interconnect Masses and apply forces according to Hooke's law with
 * dampening.
 */
#[derive(Clone, Copy, Default, Debug)]
pub struct Spring {
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

    // Get mass index a.
    pub fn get_ma(&self) -> usize {
        self.m_a
    }
    // Get mass index b.
    pub fn get_mb(&self) -> usize {
        self.m_b
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
