//! コンテナ `dijkstra_planner` の中身。juiz の型と planner_dijkstra_core の受け渡しをする

use std::path::{Path, PathBuf};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use juiz_sdk::image::{DynamicImage, GrayImage};
use juiz_sdk::serde_json::{json, Value};
use juiz_std_types::common::{Header, Time};
use juiz_std_types::geometry::{Point2D, Pose2D, Pose2DStamped};
use planner_dijkstra_core as core;
use planner_dijkstra_core::{MapMeta, OccupancyGrid, PlanOptions};

/// 地図の YAML のパスをこの環境変数で上書きできる
pub const MAP_ENV: &str = "PLANNER_MAP";
/// juiz run が展開先を入れる環境変数。相対パスはここから解決する
pub const APP_DIR_ENV: &str = "JUIZ_APP_DIR";

/// コンテナ `dijkstra_planner` の中身
pub struct Planner {
    grid: OccupancyGrid,
    meta: MapMeta,
    /// 地図をどこから得たか(YAML のパス、または set_map で渡した元の説明)
    map_source: String,
    map_image: DynamicImage,
    /// 経路の header.frame_id(地図の座標系の名前)
    frame_id: String,
    options: PlanOptions,
    /// 最後に受け取ったロボットの姿勢(トピック amcl_pose など)
    start: Option<Pose2DStamped>,
    /// 最後に受け取った目標
    goal: Option<Pose2DStamped>,
    /// 最後に求めた結果(経路が求まらなかったときも)
    last: Option<Value>,
    /// 計画した回数
    count: u64,
    /// 地図を替えた回数(画面が地図を取り直すため)
    map_version: u64,
}

impl Planner {
    /// YAML の地図を読んで作る。`PLANNER_MAP` があればそちらを読む
    pub fn new(map: &str, frame_id: &str, options: PlanOptions) -> Result<Self, String> {
        let path = match std::env::var(MAP_ENV) {
            Ok(p) if !p.is_empty() => PathBuf::from(p),
            _ => resolve_path(Path::new(map)),
        };
        let (grid, meta) = OccupancyGrid::load(&path).map_err(|e| e.to_string())?;
        Ok(Self {
            map_image: DynamicImage::ImageLuma8(grid.render()),
            grid,
            meta,
            map_source: path.to_string_lossy().into_owned(),
            frame_id: frame_id.to_owned(),
            options,
            start: None,
            goal: None,
            last: None,
            count: 0,
            map_version: 0,
        })
    }

    /// 別の YAML の地図に替える
    pub fn load_map(&mut self, map: &str) -> Result<Value, String> {
        let path = resolve_path(Path::new(map));
        let (grid, meta) = OccupancyGrid::load(&path).map_err(|e| e.to_string())?;
        self.replace_map(grid, meta, path.to_string_lossy().into_owned());
        Ok(self.map_info())
    }

    /// 画像で地図を替える(mobile_sim や mapper の get_map・get_map_info の結果をそのまま渡せる)
    pub fn set_map(&mut self, image: &GrayImage, meta: MapMeta) -> Result<Value, String> {
        let grid = OccupancyGrid::from_image(image, &meta).map_err(|e| e.to_string())?;
        self.replace_map(grid, meta, "(image)".to_owned());
        Ok(self.map_info())
    }

    fn replace_map(&mut self, grid: OccupancyGrid, meta: MapMeta, source: String) {
        self.map_image = DynamicImage::ImageLuma8(grid.render());
        self.grid = grid;
        self.meta = meta;
        self.map_source = source;
        self.map_version += 1;
        // 古い地図で求めた経路は捨てる
        self.last = None;
    }

    pub fn map_image(&self) -> DynamicImage {
        self.map_image.clone()
    }

    /// 地図の情報(mobile_sim の get_map_info と同じ名前)
    pub fn map_info(&self) -> Value {
        json!({
            "path": self.map_source,
            "resolution": self.grid.resolution(),
            "origin": self.meta.origin,
            "width": self.grid.width(),
            "height": self.grid.height(),
            "occupied_thresh": self.meta.occupied_thresh,
            "free_thresh": self.meta.free_thresh,
            "negate": self.meta.negate,
            "frame_id": self.frame_id,
            "version": self.map_version,
        })
    }

    pub fn set_start(&mut self, pose: Pose2DStamped) {
        self.start = Some(pose);
    }

    /// 目標を受け取り、最後に受け取ったロボットの姿勢から経路を求める
    pub fn set_goal(&mut self, goal: Pose2DStamped) -> Value {
        self.goal = Some(goal.clone());
        match self.start.clone() {
            Some(start) => self.plan(&start.pose, &goal.pose),
            None => self.fail(
                None,
                &goal.pose,
                "no robot pose yet: publish it to the topic amcl_pose (or call planner_set_start)",
            ),
        }
    }

    /// `start` から `goal` までの経路を求めて覚え、結果を返す。求まらないときは ok: false と理由
    pub fn plan(&mut self, start: &Pose2D, goal: &Pose2D) -> Value {
        self.goal = Some(Pose2DStamped {
            header: self.header(),
            pose: goal.clone(),
        });
        let t0 = Instant::now();
        let result = core::plan(&self.grid, &to_core(start), &to_core(goal), &self.options);
        let time_ms = t0.elapsed().as_secs_f64() * 1e3;
        self.count += 1;
        let v = match result {
            Ok(path) => json!({
                "header": header_json(&self.header()),
                "ok": true,
                "message": "",
                "start": pose_json(start),
                "goal": pose_json(goal),
                "poses": path.poses.iter().map(|p| json!({
                    "position": { "x": p.x, "y": p.y },
                    "theta": p.theta,
                })).collect::<Vec<_>>(),
                "length": path.length,
                "cost": path.cost,
                "expanded": path.expanded,
                "time_ms": time_ms,
            }),
            Err(e) => return self.fail(Some(start), goal, &e.to_string()),
        };
        self.last = Some(v.clone());
        v
    }

    fn fail(&mut self, start: Option<&Pose2D>, goal: &Pose2D, message: &str) -> Value {
        let v = json!({
            "header": header_json(&self.header()),
            "ok": false,
            "message": message,
            "start": start.map(pose_json),
            "goal": pose_json(goal),
            "poses": [],
            "length": 0.0,
            "cost": 0.0,
            "expanded": 0,
            "time_ms": 0.0,
        });
        self.last = Some(v.clone());
        v
    }

    /// 最後に求めた結果。まだ求めていなければ ok: false の空の経路
    pub fn path(&self) -> Value {
        self.last.clone().unwrap_or_else(|| {
            json!({
                "header": header_json(&self.header()),
                "ok": false,
                "message": "no plan yet",
                "start": null,
                "goal": null,
                "poses": [],
                "length": 0.0,
                "cost": 0.0,
                "expanded": 0,
                "time_ms": 0.0,
            })
        })
    }

    /// 画面用: ロボットの姿勢、目標、経路、地図の情報、設定
    pub fn state(&self) -> Value {
        json!({
            "start": self.start.as_ref().map(stamped_json),
            "goal": self.goal.as_ref().map(stamped_json),
            "path": self.path(),
            "map": self.map_info(),
            "options": {
                "allow_unknown": self.options.allow_unknown,
                "diagonal": self.options.diagonal,
            },
            "count": self.count,
        })
    }

    fn header(&self) -> Header {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default();
        Header {
            stamp: Time {
                sec: now.as_secs() as i64,
                nanosec: now.subsec_nanos() as i32,
            },
            frame_id: self.frame_id.clone(),
        }
    }
}

/// 相対パスは JUIZ_APP_DIR(juiz run の展開先)から、無ければカレントディレクトリから
fn resolve_path(p: &Path) -> PathBuf {
    if p.is_absolute() {
        return p.to_owned();
    }
    match std::env::var_os(APP_DIR_ENV) {
        Some(dir) if !dir.is_empty() => PathBuf::from(dir).join(p),
        _ => p.to_owned(),
    }
}

/// 地図の画像と一緒に渡す値から MapMeta を作る
pub fn meta_from_args(
    resolution: f64,
    origin: &[Value],
    negate: bool,
    occupied_thresh: f64,
    free_thresh: f64,
) -> Result<MapMeta, String> {
    if origin.len() < 2 {
        return Err(format!(
            "origin must be [x, y] or [x, y, yaw], got {origin:?}"
        ));
    }
    let mut o = [0.0; 3];
    for (k, v) in origin.iter().take(3).enumerate() {
        o[k] = v
            .as_f64()
            .ok_or_else(|| format!("origin must be numbers, got {origin:?}"))?;
    }
    let meta = MapMeta {
        occupied_thresh,
        free_thresh,
        negate: negate as u8,
        ..MapMeta::new(resolution, o)
    };
    meta.validate().map_err(|e| e.to_string())?;
    Ok(meta)
}

fn to_core(p: &Pose2D) -> core::Pose2D {
    core::Pose2D::new(p.position.x, p.position.y, p.theta)
}

pub fn to_pose(x: f64, y: f64, theta: f64) -> Pose2D {
    Pose2D {
        position: Point2D { x, y },
        theta,
    }
}

fn pose_json(p: &Pose2D) -> Value {
    json!({ "position": { "x": p.position.x, "y": p.position.y }, "theta": p.theta })
}

fn header_json(h: &Header) -> Value {
    json!({
        "stamp": { "sec": h.stamp.sec, "nanosec": h.stamp.nanosec },
        "frame_id": h.frame_id,
    })
}

fn stamped_json(p: &Pose2DStamped) -> Value {
    json!({ "header": header_json(&p.header), "pose": pose_json(&p.pose) })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_map() -> String {
        format!(
            "{}/../../app/assets/maps/default.yaml",
            env!("CARGO_MANIFEST_DIR")
        )
    }

    fn stamped(x: f64, y: f64, theta: f64) -> Pose2DStamped {
        Pose2DStamped {
            header: Header {
                stamp: Time { sec: 0, nanosec: 0 },
                frame_id: "map".to_owned(),
            },
            pose: to_pose(x, y, theta),
        }
    }

    #[test]
    fn plans_on_the_default_map() {
        let mut p = Planner::new(&default_map(), "map", PlanOptions::default()).unwrap();
        // mobile_sim の初期位置(左の部屋)から、仕切り壁の向こうの右の部屋へ
        let v = p.plan(&to_pose(-2.0, 0.0, 0.0), &to_pose(3.0, -2.5, 1.0));
        assert_eq!(v["ok"], true, "{v}");
        let poses = v["poses"].as_array().unwrap();
        assert!(poses.len() > 50);
        assert_eq!(poses.last().unwrap()["theta"], 1.0);
        assert!(v["length"].as_f64().unwrap() > 5.0);
        // 経路の点はどれも空きの格子にある
        let g = &p.grid;
        for q in poses {
            let (x, y) = (
                q["position"]["x"].as_f64().unwrap(),
                q["position"]["y"].as_f64().unwrap(),
            );
            let (i, j) = g.world_to_cell(x, y).unwrap();
            assert!(g.is_passable(i as i64, j as i64, false), "({x}, {y})");
        }
        assert_eq!(p.path(), v);
        assert_eq!(p.state()["count"], 1);
    }

    #[test]
    fn set_goal_uses_the_last_start() {
        let mut p = Planner::new(&default_map(), "map", PlanOptions::default()).unwrap();
        let v = p.set_goal(stamped(0.5, 3.0, 0.0));
        assert_eq!(v["ok"], false);
        assert!(v["message"].as_str().unwrap().contains("amcl_pose"));
        p.set_start(stamped(-2.0, 0.0, 0.0));
        let v = p.set_goal(stamped(0.5, 3.0, 0.0));
        assert_eq!(v["ok"], true, "{v}");
        assert_eq!(v["header"]["frame_id"], "map");
        assert_eq!(p.state()["start"]["pose"]["position"]["x"], -2.0);
    }

    #[test]
    fn reports_a_goal_in_a_wall() {
        let mut p = Planner::new(&default_map(), "map", PlanOptions::default()).unwrap();
        // 外周の壁
        let v = p.plan(&to_pose(-2.0, 0.0, 0.0), &to_pose(-4.99, 0.0, 0.0));
        assert_eq!(v["ok"], false);
        assert_eq!(v["message"], "the goal is on an obstacle");
        assert_eq!(p.path()["ok"], false);
    }

    #[test]
    fn set_map_replaces_the_grid() {
        let mut p = Planner::new(&default_map(), "map", PlanOptions::default()).unwrap();
        p.plan(&to_pose(-2.0, 0.0, 0.0), &to_pose(0.0, 0.0, 0.0));
        let img = GrayImage::from_pixel(20, 10, juiz_sdk::image::Luma([254]));
        let meta = meta_from_args(
            0.1,
            &[json!(0.0), json!(0.0), json!(0.0)],
            false,
            0.65,
            0.196,
        )
        .unwrap();
        let info = p.set_map(&img, meta).unwrap();
        assert_eq!(info["width"], 20);
        assert_eq!(info["path"], "(image)");
        assert_eq!(info["version"], 1);
        assert_eq!(p.path()["message"], "no plan yet");
        let v = p.plan(&to_pose(0.05, 0.05, 0.0), &to_pose(1.95, 0.95, 0.0));
        assert_eq!(v["ok"], true);
        assert!(meta_from_args(0.1, &[json!(0.0)], false, 0.65, 0.196).is_err());
        assert!(meta_from_args(0.1, &[json!("a"), json!(0.0)], false, 0.65, 0.196).is_err());
    }

    #[test]
    fn missing_map_is_an_error() {
        assert!(Planner::new("/nonexistent/map.yaml", "map", PlanOptions::default()).is_err());
    }
}
