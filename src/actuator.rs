use crate::spring::Spring;
use crate::mass::Mass;
/*
 * Actuators follow a waveform and change specific element properties.
 */
pub enum SpringActuatorType {
    ClassicMuscle,
    RelaxationMuscle,
}

pub enum SpringActuator {
    SpringClassicMuscle {
        spring: usize,
        phase: f64,
        sense: f64,
        base_restlength: f64,
    },
    SpringRelaxationMuscle{
        spring: usize,
        phase: f64,
        sense: f64,
        base_springing: f64,
    },
}


impl SpringActuator {
    pub fn new(act_type: SpringActuatorType, spring_idx: usize, spring: &Spring, phase: f64, sense: f64) -> Self {
        match act_type {
            SpringActuatorType::ClassicMuscle => Self::SpringClassicMuscle {
                spring: spring_idx,
                base_restlength: spring.r,
                phase, sense,
            },
            SpringActuatorType::RelaxationMuscle => Self::SpringRelaxationMuscle {
                spring: spring_idx,
                phase, sense,
                base_springing: spring.k,
            },
        }
    }
}

pub trait MuscleActions {
    fn get_idx(&self) -> usize;
    fn get_type(&self) -> SpringActuatorType;
    fn get_base_value(&self) -> f64;
    fn spring_to_base_value(&self, springs: &mut Vec<Spring>);
    fn spring_wave_mut(&self, springs: &mut Vec<Spring>, wave_amplitude: f64, time_passed: f64);
}

impl MuscleActions for SpringActuator {
    fn get_idx(&self) -> usize {
        match self {
            Self::SpringClassicMuscle { spring, phase: _, sense: _, base_restlength: _ } => *spring,
            Self::SpringRelaxationMuscle { spring, phase: _, sense: _, base_springing: _ } => *spring,
        }
    }

    fn get_type(&self) -> SpringActuatorType {
        match self {
            Self::SpringClassicMuscle { spring: _, phase: _, sense: _, base_restlength: _ } => SpringActuatorType::ClassicMuscle,
            Self::SpringRelaxationMuscle { spring: _, phase: _, sense: _, base_springing: _ } => SpringActuatorType::RelaxationMuscle,
        }
    }

    // Returns the original restlength or springing according to the type.
    fn get_base_value(&self) -> f64 {
        match self {
            Self::SpringClassicMuscle { spring: _, phase: _, sense: _, base_restlength } => *base_restlength,
            Self::SpringRelaxationMuscle { spring: _, phase: _, sense: _, base_springing } => *base_springing,
        }
    }

    // Returns the type of actuator if needed.
    fn spring_to_base_value(&self, springs: &mut Vec<Spring>) {
        match self {
            Self::SpringClassicMuscle { spring, phase: _, sense: _, base_restlength } => springs[*spring].r = *base_restlength,
            Self::SpringRelaxationMuscle { spring, phase: _, sense: _, base_springing } => springs[*spring].k = *base_springing,
        }
    }

    fn spring_wave_mut(&self, springs: &mut Vec<Spring>, wave_amplitude: f64, time_passed: f64) {
        match self {
            // Modify spring's restlength according to waveform.
            Self::SpringClassicMuscle { spring, phase, sense, base_restlength } => {
                springs[*spring].r = *base_restlength * (1.0 + (wave_amplitude * *sense) * (time_passed + *phase).sin())
            },
            // Modify spring's springyness according to waveform.
            Self::SpringRelaxationMuscle { spring, phase, sense, base_springing } => {
                springs[*spring].k = *base_springing * (1.0 + (wave_amplitude * *sense) * (time_passed + *phase).sin())
            },
        }
    }
}

pub enum MassActuatorType {
    Balloon,
    Tank,
}

pub enum MassActuator {
    MassBalloon {
        mass: usize,
        phase: f64,
        sense: f64,
        base_radius: f64,
    },
    MassTank {
        mass: usize,
        phase: f64,
        sense: f64,
        base_mass: f64,
    },
}

impl MassActuator {
    pub fn new(act_type: MassActuatorType, mass_idx: usize, mass: &Mass, phase: f64, sense: f64) -> Self {
        match act_type {
            MassActuatorType::Balloon => Self::MassBalloon {
                mass: mass_idx,
                phase, sense,
                base_radius: mass.r,
            },
            MassActuatorType::Tank => Self::MassTank {
                mass: mass_idx,
                phase, sense,
                base_mass: mass.m,
            },
        }
    }
}

pub trait BladderActions {
    fn get_idx(&self) -> usize;
    fn get_type(&self) -> MassActuatorType;
    fn get_base_value(&self) -> f64;
    fn mass_to_base_value(&self, masses: &mut Vec<Mass>);
    fn mass_wave_mut(&self, masses: &mut Vec<Mass>, wave_amplitude: f64, time_passed: f64);
}

impl BladderActions for MassActuator {
    fn get_idx(&self) -> usize {
        match self {
            Self::MassBalloon { mass, phase: _, sense: _, base_radius: _ } => *mass,
            Self::MassTank { mass, phase: _, sense: _, base_mass: _ } => *mass,
        }
    }

    fn get_type(&self) -> MassActuatorType {
        match self {
            Self::MassBalloon { mass: _, phase: _, sense: _, base_radius: _ } => MassActuatorType::Balloon,
            Self::MassTank { mass: _, phase: _, sense: _, base_mass: _ } => MassActuatorType::Tank,
        }
    }

    // Returns the original restlength or springing according to the type.
    fn get_base_value(&self) -> f64 {
        match self {
            Self::MassBalloon { mass: _, phase: _, sense: _, base_radius } => *base_radius,
            Self::MassTank { mass: _, phase: _, sense: _, base_mass } => *base_mass,
        }
    }

    // Returns the type of actuator if needed.
    fn mass_to_base_value(&self, masses: &mut Vec<Mass>) {
        match self {
            Self::MassBalloon { mass, phase: _, sense: _, base_radius } => masses[*mass].r = *base_radius,
            Self::MassTank { mass, phase: _, sense: _, base_mass } => masses[*mass].m = *base_mass,
        }
    }

    fn mass_wave_mut(&self, masses: &mut Vec<Mass>, wave_amplitude: f64, time_passed: f64) {
        match self {
            // Modify spring's restlength according to waveform.
            Self::MassBalloon { mass, phase, sense, base_radius } => {
                masses[*mass].r = *base_radius * (2.0 + (wave_amplitude * *sense) * (time_passed + *phase).sin())
            },
            // Modify spring's springyness according to waveform.
            Self::MassTank { mass, phase, sense, base_mass } => {
                masses[*mass].m = *base_mass * (2.0 + (wave_amplitude * *sense) * (time_passed + *phase).sin())
            },
        }
    }
}
