use crate::spring::Spring;
use crate::mass::Mass;
/*
 * Actuators follow a waveform and change specific element properties.
 */

/// Defines the type of spring actuator to be used.
pub enum SpringActuatorType {
    ClassicMuscle,
    RelaxationMuscle,
}

// The waveform that every actuator must follow
fn waveform(amplitude: f64, sense: f64, angle: f64,  phase: f64) -> f64 {
    1.0 + amplitude * sense * (angle + phase).sin()
}

/// Attached to a spring, allowing changes in specific properties according to a waveform. Called muscles.
pub enum SpringActuator {
    /// Follows a waveform to expand and contract, changing the restlength.
    SpringClassicMuscle {
        spring: usize,
        phase: f64,
        sense: f64,
        base_restlength: f64,
    },

    /// Follows a waveform to tense up and relax, changing the hooke constant.
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
    /// Get the index of the spring the actuator is attached to.
    fn get_idx(&self) -> usize;
    /// Return the actuator type.
    fn get_type(&self) -> SpringActuatorType;
    /// Get the default restlength or springing of the actuator.
    fn get_base_value(&self) -> f64;
    /// Set the spring's value back to the default. Useful for the case in detaching the actuator.
    fn spring_to_base_value(&self, springs: &mut Vec<Spring>);
}

pub(crate) trait MuscleActuation {
    // Apply mutation on the attached spring.
    fn spring_wave_mut(&self, springs: &mut Vec<Spring>, wave_amplitude: f64, angle: f64);
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

}

impl MuscleActuation for SpringActuator {
    fn spring_wave_mut(&self, springs: &mut Vec<Spring>, wave_amplitude: f64, angle: f64) {
        match self {
            // Modify spring's restlength according to waveform.
            Self::SpringClassicMuscle { spring, phase, sense, base_restlength } => {
                springs[*spring].r = *base_restlength * waveform(wave_amplitude, *sense, angle, *phase)
            },
            // Modify spring's springyness according to waveform.
            Self::SpringRelaxationMuscle { spring, phase, sense, base_springing } => {
                springs[*spring].k = *base_springing * waveform(wave_amplitude, *sense, angle, *phase)
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
        multiplier: f64,
    },
    MassTank {
        mass: usize,
        phase: f64,
        sense: f64,
        base_mass: f64,
        multiplier: f64,
    },
}

impl MassActuator {
    pub fn new(act_type: MassActuatorType, mass_idx: usize, mass: &Mass, phase: f64, sense: f64, multiplier: f64) -> Self {
        match act_type {
            MassActuatorType::Balloon => Self::MassBalloon {
                mass: mass_idx,
                phase, sense, multiplier,
                base_radius: mass.r,
            },
            MassActuatorType::Tank => Self::MassTank {
                mass: mass_idx,
                phase, sense, multiplier,
                base_mass: mass.m,
            },
        }
    }
}

pub trait BladderActions {
    /// Get the index of the mass the actuator is attached to.
    fn get_idx(&self) -> usize;
    /// Return the actuator type.
    fn get_type(&self) -> MassActuatorType;
    /// Get the default radius or mass of the actuator.
    fn get_base_value(&self) -> f64;
    /// Set the mass's value back to the default. Useful for the case in detaching the actuator.
    fn mass_to_base_value(&self, masses: &mut Vec<Mass>);
}

pub(crate) trait BladderActuation {
    // Apply mutation on the attached mass.
    fn mass_wave_mut(&self, masses: &mut Vec<Mass>, wave_amplitude: f64, angle: f64);
}

impl BladderActions for MassActuator {
    fn get_idx(&self) -> usize {
        match self {
            Self::MassBalloon { mass, phase: _, sense: _, base_radius: _, multiplier: _ } => *mass,
            Self::MassTank { mass, phase: _, sense: _, base_mass: _, multiplier: _ } => *mass,
        }
    }

    fn get_type(&self) -> MassActuatorType {
        match self {
            Self::MassBalloon { mass: _, phase: _, sense: _, base_radius: _, multiplier: _ } => MassActuatorType::Balloon,
            Self::MassTank { mass: _, phase: _, sense: _, base_mass: _,multiplier: __ } => MassActuatorType::Tank,
        }
    }

    // Returns the original restlength or springing according to the type.
    fn get_base_value(&self) -> f64 {
        match self {
            Self::MassBalloon { mass: _, phase: _, sense: _, base_radius, multiplier: _ } => *base_radius,
            Self::MassTank { mass: _, phase: _, sense: _, base_mass, multiplier: _ } => *base_mass,
        }
    }

    // Returns the type of actuator if needed.
    fn mass_to_base_value(&self, masses: &mut Vec<Mass>) {
        match self {
            Self::MassBalloon { mass, phase: _, sense: _, base_radius, multiplier: _ } => masses[*mass].r = *base_radius,
            Self::MassTank { mass, phase: _, sense: _, base_mass, multiplier: _ } => masses[*mass].m = *base_mass,
        }
    }
}

impl BladderActuation for MassActuator {
    fn mass_wave_mut(&self, masses: &mut Vec<Mass>, wave_amplitude: f64, angle: f64) {
        match self {
            // Modify mass's radius according to waveform.
            Self::MassBalloon { mass, phase, sense, base_radius, multiplier } => {
                masses[*mass].r = *base_radius * (1.0 + *multiplier * waveform(wave_amplitude, *sense, angle, *phase))
            },
            // Modify mass's mass according to waveform.
            Self::MassTank { mass, phase, sense, base_mass , multiplier } => {
                masses[*mass].m = *base_mass * (1.0 + *multiplier * waveform(wave_amplitude, *sense, angle, *phase))
            },
        }
    }
}
