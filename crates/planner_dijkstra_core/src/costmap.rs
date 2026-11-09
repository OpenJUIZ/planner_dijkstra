//! コストマップ: 障害物をロボットの大きさだけ膨らませた格子(ROS costmap_2d の inflation と同じ考え)
//!
//! 格子の中心から一番近い障害物の格子の中心までの距離 `d` [m] で、格子を 3 つに分ける。
//!
//! - `d <= lethal`(= `robot_radius + safety_margin`): ロボットの中心が入るとぶつかる。通れない
//! - `lethal < d < inflation_radius`: 通れるが、障害物に近いほど高い。1 歩の重みは
//!   `距離 × (1 + weight × ((inflation_radius - d) / (inflation_radius - lethal))²)`
//! - それより遠い: 重みは距離のまま
//!
//! 障害物として膨らませるのは `Occupied` の格子だけ(不明の格子は膨らませない。通れるかは `allow_unknown` で決まる)。

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::error::PlanError;
use crate::map::OccupancyGrid;

/// 障害物の膨らませ方
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Inflation {
    /// ロボットの半径 [m]。planner_dwa の `robot_radius` に合わせる
    pub robot_radius: f64,
    /// 半径に足す余裕 [m]
    pub safety_margin: f64,
    /// コストを上げる範囲 [m](障害物の格子の中心から)。これより遠ければ重みは距離のまま
    pub inflation_radius: f64,
    /// 障害物のそばのコストの重み。0 なら通れない範囲を除いて距離だけで決める
    pub weight: f64,
}

impl Default for Inflation {
    fn default() -> Self {
        Self {
            robot_radius: 0.2,
            safety_margin: 0.05,
            inflation_radius: 0.6,
            weight: 2.0,
        }
    }
}

impl Inflation {
    /// 膨らませない(占有格子をそのまま探索する。0.1.0 と同じ)
    pub const NONE: Self = Self {
        robot_radius: 0.0,
        safety_margin: 0.0,
        inflation_radius: 0.0,
        weight: 0.0,
    };

    /// 障害物の格子の中心からこの距離 [m] 以下の格子は通れない
    pub fn lethal_distance(&self) -> f64 {
        self.robot_radius + self.safety_margin
    }

    pub fn validate(&self) -> Result<(), String> {
        for (name, v) in [
            ("robot_radius", self.robot_radius),
            ("safety_margin", self.safety_margin),
            ("inflation_radius", self.inflation_radius),
            ("weight", self.weight),
        ] {
            if !(v.is_finite() && v >= 0.0) {
                return Err(format!("{name} must be a finite number >= 0, got {v}"));
            }
        }
        Ok(())
    }

    /// 距離 `d` の格子の 1 歩の重みにかける倍率。通れなければ `None`
    fn factor(&self, d: f64) -> Option<f64> {
        let lethal = self.lethal_distance();
        if d <= lethal {
            return None;
        }
        if d >= self.inflation_radius || self.weight == 0.0 {
            return Some(1.0);
        }
        let t = (self.inflation_radius - d) / (self.inflation_radius - lethal);
        Some(1.0 + self.weight * t * t)
    }
}

/// 占有格子と、各格子から一番近い障害物までの距離
#[derive(Clone, Debug)]
pub struct Costmap {
    grid: OccupancyGrid,
    inflation: Inflation,
    /// `j * width + i` に、一番近い `Occupied` の格子の中心までの距離 [m]。調べた範囲より遠ければ無限大
    distance: Vec<f64>,
}

impl Costmap {
    pub fn new(grid: OccupancyGrid, inflation: Inflation) -> Self {
        let (w, h) = (grid.width(), grid.height());
        let res = grid.resolution();
        let mut distance = vec![f64::INFINITY; w * h];
        // 調べる範囲(これより遠い格子の距離は使わない)
        let reach = inflation.inflation_radius.max(inflation.lethal_distance());
        let r = (reach / res).ceil() as i64 + 1;
        let occupied = |i: i64, j: i64| grid.cell(i, j) == Some(crate::map::Cell::Occupied);
        for j in 0..h as i64 {
            for i in 0..w as i64 {
                if !occupied(i, j) {
                    continue;
                }
                distance[j as usize * w + i as usize] = 0.0;
                // まわりが全部障害物の格子からは、外の格子への距離は境目の格子より遠いので飛ばす
                if occupied(i + 1, j)
                    && occupied(i - 1, j)
                    && occupied(i, j + 1)
                    && occupied(i, j - 1)
                {
                    continue;
                }
                for dj in -r..=r {
                    let y = j + dj;
                    if y < 0 || y >= h as i64 {
                        continue;
                    }
                    for di in -r..=r {
                        let x = i + di;
                        if x < 0 || x >= w as i64 {
                            continue;
                        }
                        let d = ((di * di + dj * dj) as f64).sqrt() * res;
                        let k = y as usize * w + x as usize;
                        if d < distance[k] {
                            distance[k] = d;
                        }
                    }
                }
            }
        }
        Self {
            grid,
            inflation,
            distance,
        }
    }

    pub fn grid(&self) -> &OccupancyGrid {
        &self.grid
    }

    pub fn inflation(&self) -> &Inflation {
        &self.inflation
    }

    /// 一番近い障害物の格子の中心までの距離 [m]。地図の外や `inflation_radius` より遠ければ無限大
    pub fn distance(&self, i: i64, j: i64) -> f64 {
        if i < 0 || j < 0 || i as usize >= self.grid.width() || j as usize >= self.grid.height() {
            return f64::INFINITY;
        }
        self.distance[j as usize * self.grid.width() + i as usize]
    }

    /// 占有格子の上で通れるか(膨らませる前)
    pub fn is_free(&self, i: i64, j: i64, allow_unknown: bool) -> bool {
        self.grid.is_passable(i, j, allow_unknown)
    }

    /// ロボットの中心が入ってよいか(膨らませた後)
    pub fn is_passable(&self, i: i64, j: i64, allow_unknown: bool) -> bool {
        self.is_free(i, j, allow_unknown) && self.inflation.factor(self.distance(i, j)).is_some()
    }

    /// 格子に入る 1 歩の重みにかける倍率。通れなければ `None`
    pub fn factor(&self, i: i64, j: i64, allow_unknown: bool) -> Option<f64> {
        if !self.is_free(i, j, allow_unknown) {
            return None;
        }
        self.inflation.factor(self.distance(i, j))
    }

    /// 障害物のそばの通れない格子にいるとき、空きの格子だけをたどって一番近い通れる格子を探す。
    /// たどる長さが `max_distance` [m] を超えたら諦める。`cell` 自体が空きでなければ `None`
    pub fn nearest_passable(
        &self,
        cell: (usize, usize),
        allow_unknown: bool,
        max_distance: f64,
    ) -> Option<(usize, usize)> {
        let (w, h) = (self.grid.width(), self.grid.height());
        if cell.0 >= w || cell.1 >= h || !self.is_free(cell.0 as i64, cell.1 as i64, allow_unknown)
        {
            return None;
        }
        let res = self.grid.resolution();
        let diag = res * std::f64::consts::SQRT_2;
        let moves = [
            (1, 0, res),
            (-1, 0, res),
            (0, 1, res),
            (0, -1, res),
            (1, 1, diag),
            (1, -1, diag),
            (-1, 1, diag),
            (-1, -1, diag),
        ];
        let mut dist = vec![f64::INFINITY; w * h];
        let s = cell.1 * w + cell.0;
        dist[s] = 0.0;
        let mut heap = BinaryHeap::from([Entry {
            cost: 0.0,
            index: s,
        }]);
        while let Some(Entry { cost, index: u }) = heap.pop() {
            if cost > dist[u] {
                continue;
            }
            let (ui, uj) = ((u % w) as i64, (u / w) as i64);
            if self.is_passable(ui, uj, allow_unknown) {
                return Some((ui as usize, uj as usize));
            }
            for &(di, dj, step) in &moves {
                let (vi, vj) = (ui + di, uj + dj);
                if !self.is_free(vi, vj, allow_unknown) {
                    continue;
                }
                let next = cost + step;
                if next > max_distance + 1e-9 {
                    continue;
                }
                let v = vj as usize * w + vi as usize;
                if next < dist[v] {
                    dist[v] = next;
                    heap.push(Entry {
                        cost: next,
                        index: v,
                    });
                }
            }
        }
        None
    }

    /// 画像にする(画面用)。障害物 0、通れない範囲 96、コストを上げる範囲 160〜(遠いほど明るい)、空き 254、不明 205
    pub fn render(&self, allow_unknown: bool) -> image::GrayImage {
        let (w, h) = (self.grid.width() as u32, self.grid.height() as u32);
        image::GrayImage::from_fn(w, h, |x, y| {
            let (i, j) = (x as i64, (h - 1 - y) as i64);
            let v = match self.grid.cell(i, j) {
                Some(crate::map::Cell::Occupied) => 0,
                Some(crate::map::Cell::Unknown) if !allow_unknown => 205,
                _ => match self.inflation.factor(self.distance(i, j)) {
                    None => 96,
                    Some(f) if f > 1.0 && self.inflation.weight > 0.0 => {
                        // f は 1 〜 1 + weight。重いほど暗く
                        let t = (f - 1.0) / self.inflation.weight;
                        (250.0 - 90.0 * t).round() as u8
                    }
                    _ => {
                        if self.grid.cell(i, j) == Some(crate::map::Cell::Unknown) {
                            205
                        } else {
                            254
                        }
                    }
                },
            };
            image::Luma([v])
        })
    }
}

/// 出発や目標が障害物のそばの通れない格子にあるとき、どこまで寄せてよいか [m]
pub fn snap_distance(inflation: &Inflation, resolution: f64) -> f64 {
    2.0 * inflation.lethal_distance() + 2.0 * resolution
}

/// `cell` が通れればそのまま、通れなければ近くの通れる格子。どちらも無ければ `blocked`
pub(crate) fn passable_or_nearest(
    costmap: &Costmap,
    cell: (usize, usize),
    allow_unknown: bool,
    blocked: PlanError,
) -> Result<(usize, usize), PlanError> {
    if costmap.is_passable(cell.0 as i64, cell.1 as i64, allow_unknown) {
        return Ok(cell);
    }
    if !costmap.is_free(cell.0 as i64, cell.1 as i64, allow_unknown) {
        return Err(blocked);
    }
    let max = snap_distance(costmap.inflation(), costmap.grid().resolution());
    costmap
        .nearest_passable(cell, allow_unknown, max)
        .ok_or(match blocked {
            PlanError::StartBlocked => PlanError::StartTooClose,
            _ => PlanError::GoalTooClose,
        })
}

#[derive(Clone, Copy, PartialEq)]
pub(crate) struct Entry {
    pub(crate) cost: f64,
    pub(crate) index: usize,
}

impl Eq for Entry {}

impl Ord for Entry {
    fn cmp(&self, other: &Self) -> Ordering {
        // BinaryHeap は大きい順なので逆にする。同じ重みなら添字で決める(結果を毎回同じにする)
        other
            .cost
            .total_cmp(&self.cost)
            .then_with(|| other.index.cmp(&self.index))
    }
}

impl PartialOrd for Entry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::map::Cell;

    /// 0.1 m の格子で、x = 1.0 m に縦の壁がある 3 m × 1 m の地図
    fn wall_grid() -> OccupancyGrid {
        let mut g = OccupancyGrid::new_free(30, 10, 0.1, (0.0, 0.0));
        for j in 0..10 {
            g.set_cell(10, j, Cell::Occupied);
        }
        g
    }

    #[test]
    fn distance_to_the_nearest_obstacle() {
        let c = Costmap::new(wall_grid(), Inflation::default());
        assert_eq!(c.distance(10, 5), 0.0);
        assert!((c.distance(12, 5) - 0.2).abs() < 1e-12);
        assert!((c.distance(7, 5) - 0.3).abs() < 1e-12);
        // inflation_radius (0.6) より遠いところも、調べた範囲では距離が入る。範囲の外は無限大
        assert_eq!(c.distance(25, 5), f64::INFINITY);
        assert_eq!(c.distance(-1, 0), f64::INFINITY);
    }

    #[test]
    fn lethal_and_inflated_cells() {
        let c = Costmap::new(wall_grid(), Inflation::default());
        // lethal = 0.25 m: 2 格子 (0.2 m) 離れた格子は通れず、3 格子 (0.3 m) なら通れる
        assert!(!c.is_passable(12, 5, false));
        assert!(!c.is_passable(8, 5, false));
        assert!(c.is_passable(13, 5, false));
        assert!(c.is_passable(7, 5, false));
        // 近いほど高い。inflation_radius から先は 1
        let f3 = c.factor(13, 5, false).unwrap();
        let f5 = c.factor(15, 5, false).unwrap();
        assert!(f3 > f5 && f5 > 1.0, "{f3} {f5}");
        assert_eq!(c.factor(17, 5, false), Some(1.0));
        assert!((f3 - (1.0 + 2.0 * (0.3f64 / 0.35).powi(2))).abs() < 1e-9);
    }

    #[test]
    fn no_inflation_is_the_occupancy_grid() {
        let g = wall_grid();
        let c = Costmap::new(g.clone(), Inflation::NONE);
        for j in 0..10 {
            for i in 0..30 {
                assert_eq!(c.is_passable(i, j, false), g.is_passable(i, j, false));
                if g.is_passable(i, j, false) {
                    assert_eq!(c.factor(i, j, false), Some(1.0));
                }
            }
        }
    }

    #[test]
    fn nearest_passable_moves_away_from_the_wall() {
        let c = Costmap::new(wall_grid(), Inflation::default());
        // 壁のすぐ右 (11, 4) から、一番近い通れる格子 (13, 4)
        assert_eq!(c.nearest_passable((11, 4), false, 1.0), Some((13, 4)));
        // 届かなければ None
        assert_eq!(c.nearest_passable((11, 4), false, 0.1), None);
        // 壁の中からは探さない
        assert_eq!(c.nearest_passable((10, 4), false, 1.0), None);
    }

    #[test]
    fn nearest_passable_does_not_go_through_walls() {
        // 壁の左側の、壁と地図の端の間 (幅 0.3 m) は全部通れない。壁を越えて右に出たりはしない
        let mut g = OccupancyGrid::new_free(20, 5, 0.1, (0.0, 0.0));
        for j in 0..5 {
            g.set_cell(0, j, Cell::Occupied);
            g.set_cell(4, j, Cell::Occupied);
        }
        let c = Costmap::new(g, Inflation::default());
        assert_eq!(c.nearest_passable((2, 2), false, 2.0), None);
        assert_eq!(
            passable_or_nearest(&c, (2, 2), false, PlanError::GoalBlocked),
            Err(PlanError::GoalTooClose)
        );
        assert_eq!(
            passable_or_nearest(&c, (0, 2), false, PlanError::GoalBlocked),
            Err(PlanError::GoalBlocked)
        );
    }

    #[test]
    fn render_marks_the_zones() {
        let c = Costmap::new(wall_grid(), Inflation::default());
        let img = c.render(false);
        let px = |i: u32, j: u32| img.get_pixel(i, 9 - j).0[0];
        assert_eq!(px(10, 5), 0);
        assert_eq!(px(12, 5), 96);
        assert!((160..254).contains(&px(13, 5)), "{}", px(13, 5));
        assert_eq!(px(25, 5), 254);
    }

    #[test]
    fn rejects_bad_inflation() {
        assert!(Inflation::default().validate().is_ok());
        let bad = Inflation {
            robot_radius: -0.1,
            ..Default::default()
        };
        assert!(bad.validate().is_err());
        let bad = Inflation {
            weight: f64::NAN,
            ..Default::default()
        };
        assert!(bad.validate().is_err());
    }
}
