//! planner_dijkstra の juiz コンポーネント。
//!
//! コンテナ `dijkstra_planner` が占有格子の地図を持ち、ロボットの姿勢から目標までの経路をダイクストラ法で求める。
//! 障害物はロボットの半径(+ 余裕)だけ膨らませ、そばの格子は重みを上げる(壁から離れた経路にする)。
//! 処理の中身は [`planner::Planner`] にある。
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
//! | `planner_get_costmap` | なし | 膨らませた地図の画像(障害物 0、通れない範囲 96、重みを上げる範囲 160〜250、空き 254、不明 205) |
//! | `planner_set_inflation` | `robot_radius`, `safety_margin`, `inflation_radius`, `inflation_weight`(どれも省略可) | 設定 |
//! | `planner_get_state` | なし | 画面用: 姿勢・目標・経路・地図の情報 |
//!
//! 経路は `{header, ok, message, start, goal, poses: [Pose2D], length, cost, expanded, time_ms, start_moved_to, goal_moved_to}`。
//! 求まらないときは `ok: false` で、`message` に理由が入る(呼び手にエラーは返さない。トピックで動かすため)。
//! 出発や目標が障害物に近すぎるときは近くの通れる格子に寄せ、`start_moved_to`・`goal_moved_to` に寄せた先の `{x, y}` を、
//! `message` にそのことを入れる(`ok: true`)。目標を寄せたときは経路の最後の点が寄せた先になる。

pub mod planner;

use juiz_sdk::anyhow::anyhow;
use juiz_sdk::image::DynamicImage;
use juiz_sdk::prelude::*;
use juiz_std_types::geometry::{Pose2D, Pose2DStamped};
use planner_dijkstra_core::{Inflation, PlanOptions};

pub use planner::Planner;
pub use planner_dijkstra_core::VERSION;

fn err(e: String) -> JuizError {
    JuizError::InvalidValueError { message: e }
}

#[juiz_component_container(
    description = "Global path planner: Dijkstra on an occupancy grid inflated by the robot radius"
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
            robot_radius = "Robot radius [m] (default 0.2, as robot_radius of planner_dwa). Cells closer than robot_radius + safety_margin to an obstacle are not passable"
            safety_margin = "Margin added to robot_radius [m] (default 0.05)"
            inflation_radius = "Cells closer than this to an obstacle [m] cost more, so that the path keeps away from walls (default 0.6)"
            inflation_weight = "How much more those cells cost (default 2.0; 0 only keeps the robot_radius + safety_margin clearance)"
        }
    }
)]
fn dijkstra_planner(
    name: String,
    map: String,
    frame_id: String,
    allow_unknown: Option<Value>,
    diagonal: Option<Value>,
    robot_radius: Option<Value>,
    safety_margin: Option<Value>,
    inflation_radius: Option<Value>,
    inflation_weight: Option<Value>,
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
    let d = Inflation::default();
    let num = |v: Option<Value>, key: &str, default: f64| -> JuizResult<f64> {
        Ok(number(v, key)
            .map_err(|e| anyhow!(err(e)))?
            .unwrap_or(default))
    };
    let options = PlanOptions {
        allow_unknown: flag(allow_unknown, "allow_unknown", false)?,
        diagonal: flag(diagonal, "diagonal", true)?,
        inflation: Inflation {
            robot_radius: num(robot_radius, "robot_radius", d.robot_radius)?,
            safety_margin: num(safety_margin, "safety_margin", d.safety_margin)?,
            inflation_radius: num(inflation_radius, "inflation_radius", d.inflation_radius)?,
            weight: num(inflation_weight, "inflation_weight", d.weight)?,
        },
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

/// 数の引数。無い・null なら None
fn number(v: Option<Value>, key: &str) -> Result<Option<f64>, String> {
    match v {
        None | Some(Value::Null) => Ok(None),
        Some(Value::Number(n)) => n
            .as_f64()
            .map(Some)
            .ok_or_else(|| format!("{key} must be a number, got {n}")),
        Some(other) => Err(format!("{key} must be a number, got {other}")),
    }
}

#[juiz_component_container_process(
    container_type = "dijkstra_planner"
    description = "Map inflated by the robot radius (black: occupied, dark gray: too close to obstacles, gray to white: higher to lower cost)"
)]
fn planner_get_costmap(container: &ContainerImpl<Planner>) -> JuizResult<Capsule> {
    Ok(container.costmap_image().into())
}

#[juiz_component_container_process(
    container_type = "dijkstra_planner"
    description = "Change how obstacles are inflated (values not given, or null, are kept). Returns the options"
    arguments = {
        default = "{\"robot_radius\": null, \"safety_margin\": null, \"inflation_radius\": null, \"inflation_weight\": null}"
        description = {
            robot_radius = "Robot radius [m]"
            safety_margin = "Margin added to robot_radius [m]"
            inflation_radius = "Cells closer than this to an obstacle [m] cost more"
            inflation_weight = "How much more those cells cost"
        }
    }
)]
fn planner_set_inflation(
    container: &mut ContainerImpl<Planner>,
    robot_radius: Value,
    safety_margin: Value,
    inflation_radius: Value,
    inflation_weight: Value,
) -> JuizResult<Value> {
    let get = |v: Value, key: &str| number(Some(v), key).map_err(|e| anyhow!(err(e)));
    let (r, m, i, w) = (
        get(robot_radius, "robot_radius")?,
        get(safety_margin, "safety_margin")?,
        get(inflation_radius, "inflation_radius")?,
        get(inflation_weight, "inflation_weight")?,
    );
    container
        .set_inflation(r, m, i, w)
        .map_err(|e| anyhow!(err(e)))
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
            planner_get_costmap,
            planner_set_inflation,
            planner_get_state
        ]
    }
);
