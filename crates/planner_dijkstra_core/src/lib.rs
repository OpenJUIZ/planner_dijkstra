//! planner_dijkstra の中身(juiz に依存しない)。
//!
//! - [`map`] は、ROS map_server と同じ YAML と画像から占有格子を作る
//! - [`costmap`] は、障害物をロボットの半径だけ膨らませ、そばの格子の重みを上げる
//! - [`dijkstra`] は、コストマップの上でダイクストラ法を解く
//! - [`path`] は、格子の列を、世界座標の姿勢の列にする

pub mod costmap;
pub mod dijkstra;
pub mod error;
pub mod map;
pub mod path;

pub use costmap::{Costmap, Inflation};
pub use dijkstra::{plan_cells, plan_cells_on, CellPlan, PlanOptions};
pub use error::{PlanError, PlannerError};
pub use map::{Cell, MapMeta, OccupancyGrid};
pub use path::{plan, plan_on, Path2D, Pose2D};

/// このクレートの版
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
