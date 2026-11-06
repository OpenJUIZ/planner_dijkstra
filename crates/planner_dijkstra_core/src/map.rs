//! 地図: ROS の map_server と同じ YAML と画像から作る占有格子
//!
//! 座標の決まり(map_server・mobile_sim と同じ):
//! - `origin` は画像の左下の画素の、左下の角の世界座標 [m, m, rad]
//! - 画像の上の行ほど y が大きい
//! - 格子の添字 `(i, j)` は左下の画素を `(0, 0)` とし、`i` が x 方向、`j` が y 方向

use std::path::{Path, PathBuf};

use image::{GrayImage, Luma};
use serde::{Deserialize, Serialize};

use crate::error::PlannerError;

/// 地図の YAML の中身(画像のパス以外は、画像で地図を渡すときにも使う)
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MapMeta {
    /// 画像のパス。相対パスは YAML のあるディレクトリから。画像で渡したときは空
    #[serde(default)]
    pub image: PathBuf,
    /// 1 画素の大きさ [m/px]
    pub resolution: f64,
    /// 左下の画素の世界座標 [x, y, yaw]
    pub origin: [f64; 3],
    #[serde(default = "default_occupied_thresh")]
    pub occupied_thresh: f64,
    #[serde(default = "default_free_thresh")]
    pub free_thresh: f64,
    /// 0 なら黒が障害物、1 なら白が障害物
    #[serde(default)]
    pub negate: u8,
}

fn default_occupied_thresh() -> f64 {
    0.65
}

fn default_free_thresh() -> f64 {
    0.196
}

impl MapMeta {
    /// 画像のパス以外を指定して作る
    pub fn new(resolution: f64, origin: [f64; 3]) -> Self {
        Self {
            image: PathBuf::new(),
            resolution,
            origin,
            occupied_thresh: default_occupied_thresh(),
            free_thresh: default_free_thresh(),
            negate: 0,
        }
    }

    /// YAML の文字列から読む(`path` はエラーの表示用)
    pub fn from_yaml(text: &str, path: &Path) -> Result<Self, PlannerError> {
        let meta: MapMeta = serde_yaml::from_str(text).map_err(|source| PlannerError::MapYaml {
            path: path.to_owned(),
            source,
        })?;
        meta.validate()?;
        Ok(meta)
    }

    pub fn validate(&self) -> Result<(), PlannerError> {
        if !(self.resolution > 0.0 && self.resolution.is_finite()) {
            return Err(PlannerError::InvalidMap(format!(
                "resolution must be positive, got {}",
                self.resolution
            )));
        }
        if !(0.0..=1.0).contains(&self.free_thresh)
            || !(0.0..=1.0).contains(&self.occupied_thresh)
            || self.free_thresh > self.occupied_thresh
        {
            return Err(PlannerError::InvalidMap(format!(
                "thresholds must satisfy 0 <= free_thresh ({}) <= occupied_thresh ({}) <= 1",
                self.free_thresh, self.occupied_thresh
            )));
        }
        if self.negate > 1 {
            return Err(PlannerError::InvalidMap(format!(
                "negate must be 0 or 1, got {}",
                self.negate
            )));
        }
        if !self.origin.iter().all(|v| v.is_finite()) {
            return Err(PlannerError::InvalidMap(format!(
                "origin must be finite, got {:?}",
                self.origin
            )));
        }
        if self.origin[2] != 0.0 {
            return Err(PlannerError::InvalidMap(
                "rotated origin (origin[2] != 0) is not supported".to_owned(),
            ));
        }
        Ok(())
    }
}

/// 格子 1 つの状態
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cell {
    Free,
    Occupied,
    Unknown,
}

/// 占有格子
#[derive(Clone, Debug)]
pub struct OccupancyGrid {
    width: usize,
    height: usize,
    resolution: f64,
    origin_x: f64,
    origin_y: f64,
    /// `j * width + i` に `(i, j)` の状態。`j = 0` が下の行
    cells: Vec<Cell>,
}

impl OccupancyGrid {
    /// YAML のファイルから、画像も読んで作る
    pub fn load(yaml_path: &Path) -> Result<(Self, MapMeta), PlannerError> {
        let text = std::fs::read_to_string(yaml_path).map_err(|source| PlannerError::Io {
            path: yaml_path.to_owned(),
            source,
        })?;
        let mut meta = MapMeta::from_yaml(&text, yaml_path)?;
        if meta.image.as_os_str().is_empty() {
            return Err(PlannerError::InvalidMap(format!(
                "{} has no image",
                yaml_path.display()
            )));
        }
        if meta.image.is_relative() {
            meta.image = yaml_path
                .parent()
                .unwrap_or(Path::new("."))
                .join(&meta.image);
        }
        let img = image::open(&meta.image)
            .map_err(|source| PlannerError::Image {
                path: meta.image.clone(),
                source,
            })?
            .to_luma8();
        Ok((Self::from_image(&img, &meta)?, meta))
    }

    /// グレースケール画像から作る(map_server の trinary モードと同じ判定)
    pub fn from_image(img: &GrayImage, meta: &MapMeta) -> Result<Self, PlannerError> {
        meta.validate()?;
        let (width, height) = (img.width() as usize, img.height() as usize);
        if width == 0 || height == 0 {
            return Err(PlannerError::InvalidMap("map image is empty".to_owned()));
        }
        let mut cells = vec![Cell::Unknown; width * height];
        for (px, py, luma) in img.enumerate_pixels() {
            let v = luma.0[0] as f64 / 255.0;
            let occ = if meta.negate == 0 { 1.0 - v } else { v };
            let cell = if occ > meta.occupied_thresh {
                Cell::Occupied
            } else if occ < meta.free_thresh {
                Cell::Free
            } else {
                Cell::Unknown
            };
            let j = height - 1 - py as usize;
            cells[j * width + px as usize] = cell;
        }
        Ok(Self {
            width,
            height,
            resolution: meta.resolution,
            origin_x: meta.origin[0],
            origin_y: meta.origin[1],
            cells,
        })
    }

    /// 全部 `Free` の格子を作る(テスト用)
    pub fn new_free(width: usize, height: usize, resolution: f64, origin: (f64, f64)) -> Self {
        Self {
            width,
            height,
            resolution,
            origin_x: origin.0,
            origin_y: origin.1,
            cells: vec![Cell::Free; width * height],
        }
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    pub fn resolution(&self) -> f64 {
        self.resolution
    }

    pub fn origin(&self) -> (f64, f64) {
        (self.origin_x, self.origin_y)
    }

    /// 世界座標の点を含む格子。地図の外なら `None`
    pub fn world_to_cell(&self, x: f64, y: f64) -> Option<(usize, usize)> {
        let gx = ((x - self.origin_x) / self.resolution).floor();
        let gy = ((y - self.origin_y) / self.resolution).floor();
        if !gx.is_finite() || !gy.is_finite() || gx < 0.0 || gy < 0.0 {
            return None;
        }
        let (i, j) = (gx as usize, gy as usize);
        (i < self.width && j < self.height).then_some((i, j))
    }

    /// 格子の中心の世界座標
    pub fn cell_center(&self, i: usize, j: usize) -> (f64, f64) {
        (
            self.origin_x + (i as f64 + 0.5) * self.resolution,
            self.origin_y + (j as f64 + 0.5) * self.resolution,
        )
    }

    /// 格子の状態。地図の外なら `None`
    pub fn cell(&self, i: i64, j: i64) -> Option<Cell> {
        if i < 0 || j < 0 || i as usize >= self.width || j as usize >= self.height {
            return None;
        }
        Some(self.cells[j as usize * self.width + i as usize])
    }

    pub fn set_cell(&mut self, i: usize, j: usize, cell: Cell) {
        assert!(i < self.width && j < self.height, "cell out of range");
        self.cells[j * self.width + i] = cell;
    }

    /// 通れるか。地図の外と障害物は通れない。不明は `allow_unknown` のときだけ通れる
    pub fn is_passable(&self, i: i64, j: i64, allow_unknown: bool) -> bool {
        match self.cell(i, j) {
            Some(Cell::Free) => true,
            Some(Cell::Unknown) => allow_unknown,
            _ => false,
        }
    }

    /// 画像にする(障害物 0、空き 254、不明 205。上の行が y の大きい側。mobile_sim の get_map と同じ)
    pub fn render(&self) -> GrayImage {
        let (w, h) = (self.width as u32, self.height as u32);
        GrayImage::from_fn(w, h, |x, y| {
            let j = (h - 1 - y) as i64;
            Luma([match self.cell(x as i64, j) {
                Some(Cell::Occupied) => 0,
                Some(Cell::Free) => 254,
                _ => 205,
            }])
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn meta() -> MapMeta {
        MapMeta::new(0.5, [-1.0, -2.0, 0.0])
    }

    #[test]
    fn image_rows_are_flipped_and_thresholded() {
        // 2x2: 上の行 = 黒・白、下の行 = 灰色・白
        let img = GrayImage::from_raw(2, 2, vec![0, 255, 205, 254]).unwrap();
        let g = OccupancyGrid::from_image(&img, &meta()).unwrap();
        assert_eq!(g.cell(0, 1), Some(Cell::Occupied));
        assert_eq!(g.cell(1, 1), Some(Cell::Free));
        assert_eq!(g.cell(0, 0), Some(Cell::Unknown));
        assert_eq!(g.cell(1, 0), Some(Cell::Free));
        assert_eq!(g.cell(2, 0), None);
        // 画像に戻すと mobile_sim の get_map と同じ値になる
        assert_eq!(g.render().into_raw(), vec![0, 254, 205, 254]);
    }

    #[test]
    fn negate_swaps_black_and_white() {
        let img = GrayImage::from_raw(2, 1, vec![0, 255]).unwrap();
        let mut m = meta();
        m.negate = 1;
        let g = OccupancyGrid::from_image(&img, &m).unwrap();
        assert_eq!(g.cell(0, 0), Some(Cell::Free));
        assert_eq!(g.cell(1, 0), Some(Cell::Occupied));
    }

    #[test]
    fn world_and_cell_coordinates() {
        let g = OccupancyGrid::new_free(4, 2, 0.5, (-1.0, -2.0));
        assert_eq!(g.world_to_cell(-1.0, -2.0), Some((0, 0)));
        assert_eq!(g.world_to_cell(0.9, -1.1), Some((3, 1)));
        assert_eq!(g.world_to_cell(1.0, -2.0), None);
        assert_eq!(g.world_to_cell(-1.01, -2.0), None);
        assert_eq!(g.world_to_cell(f64::NAN, 0.0), None);
        assert_eq!(g.cell_center(1, 1), (-0.25, -1.25));
    }

    #[test]
    fn rejects_bad_meta() {
        let img = GrayImage::from_raw(1, 1, vec![255]).unwrap();
        for m in [
            MapMeta::new(0.0, [0.0; 3]),
            MapMeta::new(0.1, [0.0, 0.0, 0.5]),
            MapMeta {
                free_thresh: 0.9,
                ..MapMeta::new(0.1, [0.0; 3])
            },
            MapMeta {
                negate: 2,
                ..MapMeta::new(0.1, [0.0; 3])
            },
        ] {
            assert!(OccupancyGrid::from_image(&img, &m).is_err(), "{m:?}");
        }
    }

    #[test]
    fn loads_yaml_with_relative_image() {
        let dir = tempfile::tempdir().unwrap();
        GrayImage::from_raw(3, 2, vec![255, 0, 255, 255, 255, 205])
            .unwrap()
            .save(dir.path().join("m.png"))
            .unwrap();
        let yaml = dir.path().join("m.yaml");
        std::fs::write(
            &yaml,
            "image: m.png\nresolution: 0.1\norigin: [1.0, 2.0, 0.0]\n",
        )
        .unwrap();
        let (g, m) = OccupancyGrid::load(&yaml).unwrap();
        assert_eq!((g.width(), g.height()), (3, 2));
        assert_eq!(g.origin(), (1.0, 2.0));
        assert_eq!(m.image, dir.path().join("m.png"));
        assert_eq!(m.occupied_thresh, 0.65);
        assert_eq!(g.cell(1, 1), Some(Cell::Occupied));
        assert_eq!(g.cell(2, 0), Some(Cell::Unknown));
    }

    #[test]
    fn load_reports_missing_files() {
        let dir = tempfile::tempdir().unwrap();
        assert!(matches!(
            OccupancyGrid::load(&dir.path().join("none.yaml")),
            Err(PlannerError::Io { .. })
        ));
        let yaml = dir.path().join("m.yaml");
        std::fs::write(
            &yaml,
            "image: none.png\nresolution: 0.1\norigin: [0, 0, 0]\n",
        )
        .unwrap();
        assert!(matches!(
            OccupancyGrid::load(&yaml),
            Err(PlannerError::Image { .. })
        ));
    }
}
