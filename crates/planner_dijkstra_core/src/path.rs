//! 世界座標の姿勢から経路を求め、姿勢の列にする

use crate::costmap::Costmap;
use crate::dijkstra::{plan_cells_on, PlanOptions};
use crate::error::PlanError;
use crate::map::OccupancyGrid;

/// 平面の姿勢 [m, m, rad]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose2D {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

impl Pose2D {
    pub fn new(x: f64, y: f64, theta: f64) -> Self {
        Self { x, y, theta }
    }
}

/// 求めた経路
#[derive(Clone, Debug, PartialEq)]
pub struct Path2D {
    /// 出発の姿勢、途中の格子の中心、目標の姿勢の順。途中の向きは次の点へ向かう向き、最後は目標の向き。
    /// 目標を寄せたときは、最後の点は寄せた格子の中心(向きは目標の向き)
    pub poses: Vec<Pose2D>,
    /// 姿勢の点をつないだ長さ [m]
    pub length: f64,
    /// ダイクストラ法のコスト(膨らませなければ格子の上での長さ [m])
    pub cost: f64,
    /// 探索で確定した格子の数
    pub expanded: usize,
    /// 出発が障害物に近すぎて寄せたとき、寄せた先の格子の中心
    pub start_moved_to: Option<(f64, f64)>,
    /// 目標が障害物に近すぎて寄せたとき、寄せた先(経路の最後の点)
    pub goal_moved_to: Option<(f64, f64)>,
}

/// `start` から `goal` までの経路を求める(`options.inflation` でコストマップを作る)
pub fn plan(
    grid: &OccupancyGrid,
    start: &Pose2D,
    goal: &Pose2D,
    options: &PlanOptions,
) -> Result<Path2D, PlanError> {
    let costmap = Costmap::new(grid.clone(), options.inflation);
    plan_on(&costmap, start, goal, options)
}

/// 作っておいたコストマップの上で `start` から `goal` までの経路を求める
pub fn plan_on(
    costmap: &Costmap,
    start: &Pose2D,
    goal: &Pose2D,
    options: &PlanOptions,
) -> Result<Path2D, PlanError> {
    let grid = costmap.grid();
    let s = grid
        .world_to_cell(start.x, start.y)
        .ok_or(PlanError::StartOutside)?;
    let g = grid
        .world_to_cell(goal.x, goal.y)
        .ok_or(PlanError::GoalOutside)?;
    let cells = plan_cells_on(costmap, s, g, options)?;
    let n = cells.cells.len();
    let first = cells.cells[0];
    let last = cells.cells[n - 1];
    let start_moved_to = cells
        .start_moved
        .then(|| grid.cell_center(first.0, first.1));
    let goal_moved_to = cells.goal_moved.then(|| grid.cell_center(last.0, last.1));

    // 両端の格子は、格子の中心の代わりに出発・目標の位置そのものを使う。
    // 寄せたときは、出発の位置から寄せた格子の中心へ向かい、目標は寄せた格子の中心で終える
    let mut points: Vec<(f64, f64)> = Vec::with_capacity(n + 2);
    points.push((start.x, start.y));
    let lo = if cells.start_moved { 0 } else { 1 };
    let hi = if cells.goal_moved { n } else { n - 1 };
    if lo < hi {
        for &(i, j) in &cells.cells[lo..hi] {
            points.push(grid.cell_center(i, j));
        }
    }
    if !cells.goal_moved {
        points.push((goal.x, goal.y));
    } else if points.len() == 1 {
        // 出発と寄せた目標が同じ格子(出発の位置そのものは寄せていない)
        points.push(goal_moved_to.unwrap());
    }

    let mut poses = Vec::with_capacity(points.len());
    let mut length = 0.0;
    for k in 0..points.len() {
        let (x, y) = points[k];
        let theta = if k + 1 < points.len() {
            let (nx, ny) = points[k + 1];
            length += (nx - x).hypot(ny - y);
            if (nx - x).hypot(ny - y) > 1e-12 {
                (ny - y).atan2(nx - x)
            } else {
                goal.theta
            }
        } else {
            goal.theta
        };
        poses.push(Pose2D::new(x, y, theta));
    }
    Ok(Path2D {
        poses,
        length,
        cost: cells.cost,
        expanded: cells.expanded,
        start_moved_to,
        goal_moved_to,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::map::Cell;

    #[test]
    fn ends_are_the_given_poses() {
        let g = OccupancyGrid::new_free(10, 10, 0.1, (0.0, 0.0));
        let start = Pose2D::new(0.12, 0.13, 1.0);
        let goal = Pose2D::new(0.87, 0.14, -2.0);
        let p = plan(&g, &start, &goal, &PlanOptions::default()).unwrap();
        assert_eq!(p.poses.first().map(|q| (q.x, q.y)), Some((0.12, 0.13)));
        assert_eq!(p.poses.last().copied(), Some(goal));
        // 途中はほぼ x 方向
        for q in &p.poses[..p.poses.len() - 1] {
            assert!(q.theta.abs() < 0.2, "{q:?}");
        }
        assert!((p.length - 0.75).abs() < 0.02, "{}", p.length);
        assert!((p.cost - 0.7).abs() < 1e-9);
    }

    #[test]
    fn same_cell_gives_two_poses() {
        let g = OccupancyGrid::new_free(2, 2, 1.0, (0.0, 0.0));
        let p = plan(
            &g,
            &Pose2D::new(0.2, 0.2, 0.0),
            &Pose2D::new(0.2, 0.2, 1.5),
            &PlanOptions::default(),
        )
        .unwrap();
        assert_eq!(p.poses.len(), 2);
        assert_eq!(p.poses[0].theta, 1.5);
        assert_eq!(p.length, 0.0);
    }

    #[test]
    fn errors_for_outside_and_blocked() {
        let mut g = OccupancyGrid::new_free(3, 3, 1.0, (0.0, 0.0));
        g.set_cell(2, 2, Cell::Occupied);
        let o = PlanOptions::default();
        let free = Pose2D::new(0.5, 0.5, 0.0);
        assert_eq!(
            plan(&g, &Pose2D::new(-1.0, 0.0, 0.0), &free, &o),
            Err(PlanError::StartOutside)
        );
        assert_eq!(
            plan(&g, &free, &Pose2D::new(5.0, 0.0, 0.0), &o),
            Err(PlanError::GoalOutside)
        );
        assert_eq!(
            plan(&g, &free, &Pose2D::new(2.5, 2.5, 0.0), &o),
            Err(PlanError::GoalBlocked)
        );
    }

    #[test]
    fn moved_ends_are_reported() {
        // 0.05 m の格子、x = 1.0 m に壁
        let mut g = OccupancyGrid::new_free(40, 20, 0.05, (0.0, 0.0));
        for j in 0..20 {
            g.set_cell(20, j, Cell::Occupied);
        }
        let o = PlanOptions::default();
        // 出発は壁の 0.1 m 左(通れない範囲)、目標は離れたところ
        let start = Pose2D::new(0.92, 0.5, 0.0);
        let goal = Pose2D::new(0.3, 0.5, 2.0);
        let p = plan(&g, &start, &goal, &o).unwrap();
        assert_eq!(p.poses[0].x, 0.92);
        let (mx, _) = p.start_moved_to.unwrap();
        assert!((mx - 0.725).abs() < 1e-9, "{mx}");
        assert_eq!(p.poses[1].x, mx);
        assert_eq!(p.goal_moved_to, None);
        assert_eq!(p.poses.last().copied(), Some(goal));
        // 目標が壁のそば: 最後の点は寄せた格子の中心で、向きは目標の向き
        let p = plan(&g, &goal, &start, &o).unwrap();
        let (gx, gy) = p.goal_moved_to.unwrap();
        let last = p.poses.last().unwrap();
        assert_eq!((last.x, last.y, last.theta), (gx, gy, 0.0));
        assert!(gx < 0.75);
        // 出発と目標が同じ格子で、目標だけ寄せる
        let p = plan(
            &g,
            &Pose2D::new(0.92, 0.5, 0.0),
            &Pose2D::new(0.93, 0.5, 1.0),
            &o,
        )
        .unwrap();
        assert_eq!(p.poses.len(), 2);
    }
}
