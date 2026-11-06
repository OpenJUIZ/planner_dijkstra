//! 世界座標の姿勢から経路を求め、姿勢の列にする

use crate::dijkstra::{plan_cells, PlanOptions};
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
    /// 出発の姿勢、途中の格子の中心、目標の姿勢の順。途中の向きは次の点へ向かう向き、最後は目標の向き
    pub poses: Vec<Pose2D>,
    /// 姿勢の点をつないだ長さ [m]
    pub length: f64,
    /// 格子の上での長さ [m](ダイクストラ法のコスト)
    pub cost: f64,
    /// 探索で確定した格子の数
    pub expanded: usize,
}

/// `start` から `goal` までの経路を求める。ロボットの大きさは考えない
pub fn plan(
    grid: &OccupancyGrid,
    start: &Pose2D,
    goal: &Pose2D,
    options: &PlanOptions,
) -> Result<Path2D, PlanError> {
    let s = grid
        .world_to_cell(start.x, start.y)
        .ok_or(PlanError::StartOutside)?;
    let g = grid
        .world_to_cell(goal.x, goal.y)
        .ok_or(PlanError::GoalOutside)?;
    let cells = plan_cells(grid, s, g, options)?;

    // 両端の格子は、格子の中心の代わりに出発・目標の位置そのものを使う
    let mut points: Vec<(f64, f64)> = Vec::with_capacity(cells.cells.len() + 1);
    points.push((start.x, start.y));
    if cells.cells.len() > 2 {
        for &(i, j) in &cells.cells[1..cells.cells.len() - 1] {
            points.push(grid.cell_center(i, j));
        }
    }
    points.push((goal.x, goal.y));

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
}
