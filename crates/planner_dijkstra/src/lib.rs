//! planner_dijkstra の juiz コンポーネント。
//!
//! コンテナ `dijkstra_planner` が占有格子の地図を持ち、ロボットの姿勢から目標までの経路をダイクストラ法で求める。
//! ロボットの大きさ(フットプリント)は考えない。処理の中身は [`planner::Planner`] にある。
//!
//! | ContainerProcess | 引数 | 戻り値 |
//! | --- | --- | --- |
//! | `planner_set_start` | `pose: Pose2DStamped` | なし(トピック amcl_pose を受ける) |
//! | `planner_set_goal` | `goal: Pose2DStamped` | 経路(トピック goal を受け、path に出す) |
//! | `planner_plan` | `start: Pose2D`, `goal: Pose2D` | 経路 |
//! | `planner_get_path` | なし | 最後に求めた経路 |
//! | `planner_set_map` | `image`, `resolution`, `origin`, `negate`, `occupied_thresh`, `free_thresh` | 地図の情報 |
//! | `planner_load_map` | `map`(YAML のパス) | 地図の情報 |
//! | `planner_get_map` | なし | 地図の画像(障害物 0、空き 254、不明 205) |
//! | `planner_get_map_info` | なし | 解像度・原点・大きさ |
//! | `planner_get_state` | なし | 画面用: 姿勢・目標・経路・地図の情報 |
//!
//! 経路は `{header, ok, message, start, goal, poses: [Pose2D], length, cost, expanded, time_ms}`。
//! 求まらないときは `ok: false` で、`message` に理由が入る(呼び手にエラーは返さない。トピックで動かすため)。

pub mod planner;

use juiz_sdk::anyhow::anyhow;
use juiz_sdk::image::DynamicImage;
use juiz_sdk::prelude::*;
use juiz_std_types::geometry::{Pose2D, Pose2DStamped};
use planner_dijkstra_core::PlanOptions;

pub use planner::Planner;
pub use planner_dijkstra_core::VERSION;

fn err(e: String) -> JuizError {
    JuizError::InvalidValueError { message: e }
}

#[juiz_component_container(
    description = "Global path planner: Dijkstra on an occupancy grid (the robot footprint is ignored)"
    arguments = {
        default = {
            map = "assets/maps/default.yaml"
            frame_id = "map"
        }
        description = {
            map = "Path of the map YAML (ROS map_server). Relative to JUIZ_APP_DIR if set. PLANNER_MAP overrides it"
            frame_id = "frame_id written in the header of the path"
            allow_unknown = "Whether the path may go through unknown cells (default false)"
            diagonal = "Move to the 8 neighbours (true, default) or only the 4 neighbours (false)"
        }
    }
)]
fn dijkstra_planner(
    name: String,
    map: String,
    frame_id: String,
    allow_unknown: Option<Value>,
    diagonal: Option<Value>,
) -> JuizResult<Box<Planner>> {
    let _ = name;
    let flag = |v: Option<Value>, key: &str, default: bool| -> JuizResult<bool> {
        match v {
            None | Some(Value::Null) => Ok(default),
            Some(Value::Bool(b)) => Ok(b),
            Some(other) => Err(anyhow!(err(format!(
                "{key} must be true or false, got {other}"
            )))),
        }
    };
    let options = PlanOptions {
        allow_unknown: flag(allow_unknown, "allow_unknown", false)?,
        diagonal: flag(diagonal, "diagonal", true)?,
    };
    let p = Planner::new(&map, &frame_id, options).map_err(|e| anyhow!(err(e)))?;
    Ok(Box::new(p))
}

#[juiz_component_container_process(
    container_type = "dijkstra_planner"
    description = "Set the robot pose used as the start (subscribe it to the topic amcl_pose)"
    arguments = {
        default = "{\"pose\": {\"header\": {\"stamp\": {\"sec\": 0, \"nanosec\": 0}, \"frame_id\": \"map\"}, \"pose\": {\"position\": {\"x\": 0.0, \"y\": 0.0}, \"theta\": 0.0}}}"
    }
)]
fn planner_set_start(
    container: &mut ContainerImpl<Planner>,
    pose: Pose2DStamped,
) -> JuizResult<()> {
    container.set_start(pose);
    Ok(())
}

#[juiz_component_container_process(
    container_type = "dijkstra_planner"
    description = "Set the goal and plan from the last robot pose. Returns the path (ok: false with a message when there is none)"
    arguments = {
        default = "{\"goal\": {\"header\": {\"stamp\": {\"sec\": 0, \"nanosec\": 0}, \"frame_id\": \"map\"}, \"pose\": {\"position\": {\"x\": 0.0, \"y\": 0.0}, \"theta\": 0.0}}}"
    }
)]
fn planner_set_goal(
    container: &mut ContainerImpl<Planner>,
    goal: Pose2DStamped,
) -> JuizResult<Value> {
    Ok(container.set_goal(goal))
}

#[juiz_component_container_process(
    container_type = "dijkstra_planner"
    description = "Plan from start to goal (both in the map frame). Returns the path (ok: false with a message when there is none)"
    arguments = {
        default = {
            start = { position = { x = 0.0, y = 0.0 }, theta = 0.0 }
            goal = { position = { x = 0.0, y = 0.0 }, theta = 0.0 }
        }
    }
)]
fn planner_plan(
    container: &mut ContainerImpl<Planner>,
    start: Pose2D,
    goal: Pose2D,
) -> JuizResult<Value> {
    Ok(container.plan(&start, &goal))
}

#[juiz_component_container_process(
    container_type = "dijkstra_planner"
    description = "The last path"
)]
fn planner_get_path(container: &ContainerImpl<Planner>) -> JuizResult<Value> {
    Ok(container.path())
}

#[juiz_component_container_process(
    container_type = "dijkstra_planner"
    description = "Replace the map with an image (the results of get_map and get_map_info of mobile_sim or mapper_gmapping can be passed as they are)"
    arguments = {
        default = "{\"resolution\": 0.05, \"origin\": [0.0, 0.0, 0.0], \"negate\": false, \"occupied_thresh\": 0.65, \"free_thresh\": 0.196}"
        description = {
            image = "Map image (black: occupied, white: free, as map_server)"
            resolution = "Meters per pixel"
            origin = "[x, y, yaw] of the lower-left pixel"
            negate = "Swap black and white"
            occupied_thresh = "Occupied above this probability"
            free_thresh = "Free below this probability"
        }
    }
)]
fn planner_set_map(
    container: &mut ContainerImpl<Planner>,
    image: DynamicImage,
    resolution: f64,
    origin: Vec<Value>,
    negate: bool,
    occupied_thresh: f64,
    free_thresh: f64,
) -> JuizResult<Value> {
    let meta = planner::meta_from_args(resolution, &origin, negate, occupied_thresh, free_thresh)
        .map_err(|e| anyhow!(err(e)))?;
    container
        .set_map(&image.to_luma8(), meta)
        .map_err(|e| anyhow!(err(e)))
}

#[juiz_component_container_process(
    container_type = "dijkstra_planner"
    description = "Load another map YAML (ROS map_server)"
    arguments = { default = { map = "assets/maps/default.yaml" } }
)]
fn planner_load_map(container: &mut ContainerImpl<Planner>, map: String) -> JuizResult<Value> {
    container.load_map(&map).map_err(|e| anyhow!(err(e)))
}

#[juiz_component_container_process(
    container_type = "dijkstra_planner"
    description = "Map image (black: occupied, white: free, gray: unknown)"
)]
fn planner_get_map(container: &ContainerImpl<Planner>) -> JuizResult<Capsule> {
    Ok(container.map_image().into())
}

#[juiz_component_container_process(
    container_type = "dijkstra_planner"
    description = "Map metadata: resolution, origin, width and height"
)]
fn planner_get_map_info(container: &ContainerImpl<Planner>) -> JuizResult<Value> {
    Ok(container.map_info())
}

#[juiz_component_container_process(
    container_type = "dijkstra_planner"
    description = "Robot pose, goal, path and map metadata (for the browser view)"
)]
fn planner_get_state(container: &ContainerImpl<Planner>) -> JuizResult<Value> {
    Ok(container.state())
}

juiz_component_manifest!(
    component_name = "planner_dijkstra"
    containers = {
        dijkstra_planner = [
            planner_set_start,
            planner_set_goal,
            planner_plan,
            planner_get_path,
            planner_set_map,
            planner_load_map,
            planner_get_map,
            planner_get_map_info,
            planner_get_state
        ]
    }
);
