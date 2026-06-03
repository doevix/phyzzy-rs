use serde::{self, Deserialize, Serialize};
use serde_json;

pub struct Loader;

impl Loader {
    pub fn load_from_json_str(json_string: &String) -> Result<LoaderData, serde_json::Error> {
        serde_json::from_str(&json_string)
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub enum SpringActuatorDataType {
    Classic,
    Relaxation,
}

#[derive(Serialize, Deserialize, Debug)]
pub enum MassActuatorDataType {
    Balloon,
    Tank,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct LoaderData {
    pub meta: MetaData,
    pub model: ModelData,
    pub world_config: WorldConfigData,
    pub world: WorldData,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CollisionLayerData {
    pub masses: Vec<usize>,
    pub springs: Vec<usize>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ModelData {
    pub wave_amplitude: f64,
    pub wave_speed: f64,
    pub angle: f64,
    pub masses: Vec<MassData>,
    pub springs: Vec<SpringData>,
    pub muscles: Vec<SpringActuatorData>,
    pub bladders: Vec<MassActuatorData>,
    pub collision_layers: Vec<CollisionLayerData>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct MetaData {
    pub name: String,
    pub creator: String,
    pub created: String,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct V2DData {
    pub x: f64,
    pub y: f64,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct MassData {
    pub mass: f64,
    pub radius: f64,
    pub pos: V2DData,
    pub vel: V2DData,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct SpringData {
    pub restlength: f64,
    pub springing: f64,
    pub dampening: f64,
    pub m_a: usize,
    pub m_b: usize,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct SpringActuatorData {
    pub muscle_type: SpringActuatorDataType,
    pub phase: f64,
    pub sense: f64,
    pub spring: usize,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct MassActuatorData {
    pub bladder_type: MassActuatorDataType,
    pub phase: f64,
    pub sense: f64,
    pub multiplier: f64,
    pub mass: usize,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct BoundaryData {
    pub pos: V2DData,
    pub nrm: V2DData,
    pub refl: f64,
    pub mu_s: f64,
    pub mu_k: f64,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct WorldConfigData {
    pub gravity: V2DData,
    pub drag: f64,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct WorldData {
    pub area_sz: V2DData,
    pub bounds: Vec<BoundaryData>,
}
