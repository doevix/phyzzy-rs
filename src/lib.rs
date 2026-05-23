pub mod v2d;
pub mod mass;
pub mod spring;
pub mod actuator;
pub mod world;
pub mod model;
pub mod loader;

pub use v2d::V2D;
pub use mass::Mass;
pub use spring::Spring;
pub use actuator::{SpringActuator, MassActuator, SpringActuatorType, MassActuatorType};
pub use world::{World, WorldConfig, Boundary};
pub use model::Model;
pub use loader::{Loader, MassActuatorDataType, SpringActuatorDataType};

/*
 * Notes on collisions:
 * Mass-mass collisions should follow the masses' mu_s and mu_k coefficients.
 * Mass-spring collisions should follow the springs' mu_s and mu_k coefficients.
 * Mass-boundary collisions should follow the boundaries' mu_s and mu_k coefficients.
 * Spring-spring collisions do not exist.
 */
