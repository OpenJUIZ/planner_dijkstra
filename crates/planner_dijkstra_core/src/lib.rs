//! planner_dijkstra の中身(juiz に依存しない)。
//!
//! - [`map`] は、ROS map_server と同じ YAML と画像から占有格子を作る
//! - [`dijkstra`] は、占有格子の上でダイクストラ法を解く(ロボットの大きさは考えない)
//! - [`path`] は、格子の列を、世界座標の姿勢の列にする

pub mod dijkstra;
pub mod error;
pub mod map;
pub mod path;

pub use dijkstra::{plan_cells, CellPlan, PlanOptions};
pub use error::{PlanError, PlannerError};
pub use map::{Cell, MapMeta, OccupancyGrid};
pub use path::{plan, Path2D, Pose2D};

/// このクレートの版
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
