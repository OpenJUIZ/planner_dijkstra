//! 占有格子の上のダイクストラ法
//!
//! - 格子の中心を節点とし、上下左右(と、`diagonal` のときは斜め)の隣へ辺を張る。辺の重みは中心どうしの距離 [m]
//! - 斜めに進むときは、角をかすめないよう、間の上下左右の 2 つの格子も通れるときだけ進める
//! - ロボットの大きさは [`Costmap`] で考える: 障害物のそばの格子には入らず、少し離れた格子は重みを上げる
//!   (壁から離れ、扉の真ん中を通る経路になる)。[`Inflation::NONE`] なら占有格子をそのまま探索する
//! - 出発や目標が障害物のそばの通れない格子にあるときは、空きの格子をたどって一番近い通れる格子に寄せる

use std::collections::BinaryHeap;

use crate::costmap::{passable_or_nearest, Costmap, Entry, Inflation};
use crate::error::PlanError;
use crate::map::OccupancyGrid;

/// 探索の設定
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlanOptions {
    /// 不明の格子を通ってよいか
    pub allow_unknown: bool,
    /// 斜めにも進むか(8 近傍)。false なら上下左右だけ(4 近傍)
    pub diagonal: bool,
    /// 障害物の膨らませ方(ロボットの大きさ)
    pub inflation: Inflation,
}

impl Default for PlanOptions {
    fn default() -> Self {
        Self {
            allow_unknown: false,
            diagonal: true,
            inflation: Inflation::default(),
        }
    }
}

/// 格子の経路
#[derive(Clone, Debug, PartialEq)]
pub struct CellPlan {
    /// 出発の格子から目標の格子までの格子の列(両端を含む)
    pub cells: Vec<(usize, usize)>,
    /// 重みの合計(障害物のそばほど重い。膨らませなければ格子の中心をつないだ長さ [m])
    pub cost: f64,
    /// 取り出して確定した節点の数
    pub expanded: usize,
    /// 出発の格子が障害物に近すぎて、`cells[0]` に寄せたか
    pub start_moved: bool,
    /// 目標の格子が障害物に近すぎて、`cells` の最後に寄せたか
    pub goal_moved: bool,
}

const NONE: usize = usize::MAX;

/// 格子 `start` から格子 `goal` までの最短経路を求める(`options.inflation` でコストマップを作る)
pub fn plan_cells(
    grid: &OccupancyGrid,
    start: (usize, usize),
    goal: (usize, usize),
    options: &PlanOptions,
) -> Result<CellPlan, PlanError> {
    let costmap = Costmap::new(grid.clone(), options.inflation);
    plan_cells_on(&costmap, start, goal, options)
}

/// 作っておいたコストマップの上で、格子 `start` から格子 `goal` までの最短経路を求める
/// (`options.inflation` は使わず、`costmap` の膨らませ方を使う)
pub fn plan_cells_on(
    costmap: &Costmap,
    start: (usize, usize),
    goal: (usize, usize),
    options: &PlanOptions,
) -> Result<CellPlan, PlanError> {
    let grid = costmap.grid();
    let (w, h) = (grid.width(), grid.height());
    let allow = options.allow_unknown;
    let passable = |i: i64, j: i64| costmap.is_passable(i, j, allow);
    if start.0 >= w || start.1 >= h {
        return Err(PlanError::StartOutside);
    }
    if goal.0 >= w || goal.1 >= h {
        return Err(PlanError::GoalOutside);
    }
    let requested = (start, goal);
    let start = passable_or_nearest(costmap, start, allow, PlanError::StartBlocked)?;
    let goal = passable_or_nearest(costmap, goal, allow, PlanError::GoalBlocked)?;

    let res = grid.resolution();
    let diag = res * std::f64::consts::SQRT_2;
    let mut moves: Vec<(i64, i64, f64)> =
        vec![(1, 0, res), (-1, 0, res), (0, 1, res), (0, -1, res)];
    if options.diagonal {
        moves.extend([(1, 1, diag), (1, -1, diag), (-1, 1, diag), (-1, -1, diag)]);
    }

    let n = w * h;
    let mut dist = vec![f64::INFINITY; n];
    let mut prev = vec![NONE; n];
    let mut done = vec![false; n];
    let index = |i: usize, j: usize| j * w + i;
    let s = index(start.0, start.1);
    let g = index(goal.0, goal.1);
    dist[s] = 0.0;
    let mut heap = BinaryHeap::new();
    heap.push(Entry {
        cost: 0.0,
        index: s,
    });
    let mut expanded = 0;

    while let Some(Entry { cost, index: u }) = heap.pop() {
        if done[u] {
            continue;
        }
        done[u] = true;
        expanded += 1;
        if u == g {
            break;
        }
        let (ui, uj) = ((u % w) as i64, (u / w) as i64);
        for &(di, dj, step) in &moves {
            let (vi, vj) = (ui + di, uj + dj);
            if !passable(vi, vj) {
                continue;
            }
            if di != 0 && dj != 0 && !(passable(ui + di, uj) && passable(ui, uj + dj)) {
                continue;
            }
            let Some(factor) = costmap.factor(vi, vj, allow) else {
                continue;
            };
            let v = index(vi as usize, vj as usize);
            let next = cost + step * factor;
            if next < dist[v] {
                dist[v] = next;
                prev[v] = u;
                heap.push(Entry {
                    cost: next,
                    index: v,
                });
            }
        }
    }

    if !done[g] {
        return Err(PlanError::NoPath);
    }
    let mut cells = Vec::new();
    let mut u = g;
    loop {
        cells.push((u % w, u / w));
        if u == s {
            break;
        }
        u = prev[u];
    }
    cells.reverse();
    Ok(CellPlan {
        cells,
        cost: dist[g],
        expanded,
        start_moved: start != requested.0,
        goal_moved: goal != requested.1,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::map::Cell;

    fn grid(rows: &[&str]) -> OccupancyGrid {
        // rows[0] が上(y の大きい側)。'#' 障害物、'?' 不明、ほかは空き
        let h = rows.len();
        let w = rows[0].len();
        let mut g = OccupancyGrid::new_free(w, h, 1.0, (0.0, 0.0));
        for (r, row) in rows.iter().enumerate() {
            for (i, c) in row.chars().enumerate() {
                let cell = match c {
                    '#' => Cell::Occupied,
                    '?' => Cell::Unknown,
                    _ => Cell::Free,
                };
                g.set_cell(i, h - 1 - r, cell);
            }
        }
        g
    }

    fn assert_connected(plan: &CellPlan, g: &OccupancyGrid, options: &PlanOptions) {
        for pair in plan.cells.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            let (di, dj) = (b.0 as i64 - a.0 as i64, b.1 as i64 - a.1 as i64);
            assert!(
                di.abs() <= 1 && dj.abs() <= 1 && (di, dj) != (0, 0),
                "{a:?} -> {b:?}"
            );
            assert!(g.is_passable(b.0 as i64, b.1 as i64, options.allow_unknown));
        }
    }

    #[test]
    fn straight_line_on_free_grid() {
        let g = grid(&[".....", ".....", "....."]);
        let p = plan_cells(&g, (0, 1), (4, 1), &PlanOptions::default()).unwrap();
        assert_eq!(p.cells, vec![(0, 1), (1, 1), (2, 1), (3, 1), (4, 1)]);
        assert!((p.cost - 4.0).abs() < 1e-12);
    }

    #[test]
    fn diagonal_moves_are_shorter() {
        let g = grid(&["....", "....", "....", "...."]);
        let p = plan_cells(&g, (0, 0), (3, 3), &PlanOptions::default()).unwrap();
        assert_eq!(p.cells.len(), 4);
        assert!((p.cost - 3.0 * 2f64.sqrt()).abs() < 1e-12);
        let four = PlanOptions {
            diagonal: false,
            ..Default::default()
        };
        let p = plan_cells(&g, (0, 0), (3, 3), &four).unwrap();
        assert_eq!(p.cells.len(), 7);
        assert!((p.cost - 6.0).abs() < 1e-12);
        assert_connected(&p, &g, &four);
    }

    #[test]
    fn goes_around_a_wall() {
        let g = grid(&[
            ".......", //
            "...#...", //
            "...#...", //
            "...#...", //
            ".......",
        ]);
        let opts = PlanOptions::default();
        let p = plan_cells(&g, (0, 2), (6, 2), &opts).unwrap();
        assert_connected(&p, &g, &opts);
        // 壁の端を回る: 斜め 4 回 + まっすぐ 2 回(どちら回りでも同じ)
        assert!(
            (p.cost - (2.0 + 4.0 * 2f64.sqrt())).abs() < 1e-9,
            "{}",
            p.cost
        );
        assert!(p
            .cells
            .iter()
            .all(|&(i, j)| g.cell(i as i64, j as i64) == Some(Cell::Free)));
    }

    #[test]
    fn does_not_cut_corners() {
        // 斜めの隙間は通れない
        let g = grid(&[
            "#.", //
            ".#",
        ]);
        let opts = PlanOptions::default();
        assert_eq!(
            plan_cells(&g, (0, 0), (1, 1), &opts),
            Err(PlanError::NoPath)
        );
    }

    #[test]
    fn unknown_cells_need_permission() {
        let g = grid(&["..?.."]);
        let opts = PlanOptions::default();
        assert_eq!(
            plan_cells(&g, (0, 0), (4, 0), &opts),
            Err(PlanError::NoPath)
        );
        let allow = PlanOptions {
            allow_unknown: true,
            ..Default::default()
        };
        let p = plan_cells(&g, (0, 0), (4, 0), &allow).unwrap();
        assert_eq!(p.cells.len(), 5);
    }

    #[test]
    fn reports_bad_ends() {
        let g = grid(&[".#.", "..."]);
        let opts = PlanOptions::default();
        assert_eq!(
            plan_cells(&g, (3, 0), (0, 0), &opts),
            Err(PlanError::StartOutside)
        );
        assert_eq!(
            plan_cells(&g, (0, 0), (0, 2), &opts),
            Err(PlanError::GoalOutside)
        );
        assert_eq!(
            plan_cells(&g, (1, 1), (0, 0), &opts),
            Err(PlanError::StartBlocked)
        );
        assert_eq!(
            plan_cells(&g, (0, 0), (1, 1), &opts),
            Err(PlanError::GoalBlocked)
        );
    }

    #[test]
    fn start_equals_goal() {
        let g = grid(&["..."]);
        let p = plan_cells(&g, (1, 0), (1, 0), &PlanOptions::default()).unwrap();
        assert_eq!(p.cells, vec![(1, 0)]);
        assert_eq!(p.cost, 0.0);
    }

    #[test]
    fn optimal_on_a_maze() {
        // 4 近傍の格子では、ダイクストラの長さは幅優先探索の手数と一致する
        let g = grid(&[
            "..#......",
            "..#.####.",
            "..#.#....",
            "....#.##.",
            "#####.#..",
            "......#.#",
        ]);
        let opts = PlanOptions {
            diagonal: false,
            ..Default::default()
        };
        let (s, t) = ((0, 5), (0, 0));
        let p = plan_cells(&g, s, t, &opts).unwrap();
        assert_connected(&p, &g, &opts);
        assert_eq!(p.cost as usize, bfs(&g, s, t).unwrap());
    }

    /// 0.05 m の格子(planner_dwa と同じ大きさで考える)。`wall_x` の縦の壁に、y = [door_lo, door_hi) の扉
    fn door_grid(door_lo: usize, door_hi: usize) -> OccupancyGrid {
        let mut g = OccupancyGrid::new_free(60, 60, 0.05, (0.0, 0.0));
        for j in 0..60 {
            if !(door_lo..door_hi).contains(&j) {
                g.set_cell(30, j, Cell::Occupied);
            }
        }
        g
    }

    #[test]
    fn passes_through_the_middle_of_a_door() {
        // 扉は y = 1.0〜2.0 m。出発と目標は扉の下の端の高さ(y = 1.0 m)
        let g = door_grid(20, 40);
        let (s, t) = ((10, 20), (50, 20));
        let none = PlanOptions {
            inflation: Inflation::NONE,
            ..Default::default()
        };
        let raw = plan_cells(&g, s, t, &none).unwrap();
        let clearance = |p: &CellPlan, c: &Costmap| {
            p.cells
                .iter()
                .map(|&(i, j)| c.distance(i as i64, j as i64))
                .fold(f64::INFINITY, f64::min)
        };
        let c = Costmap::new(g.clone(), Inflation::default());
        // 膨らませないと扉の角をかすめる
        assert!(
            clearance(&raw, &c) <= 0.05 + 1e-9,
            "{}",
            clearance(&raw, &c)
        );
        let opts = PlanOptions::default();
        let p = plan_cells(&g, s, t, &opts).unwrap();
        assert_connected(&p, &g, &opts);
        assert!(!p.start_moved && !p.goal_moved);
        // 膨らませると、扉を通る格子は角から lethal (0.25 m) より離れ、扉の真ん中寄りを通る
        assert!(clearance(&p, &c) > 0.25, "{}", clearance(&p, &c));
        let at_door: Vec<usize> = p.cells.iter().filter(|c| c.0 == 30).map(|c| c.1).collect();
        assert!(!at_door.is_empty());
        assert!(
            at_door.iter().all(|&j| (25..35).contains(&j)),
            "{at_door:?}"
        );
        // plan_cells_on でも同じ
        assert_eq!(plan_cells_on(&c, s, t, &opts).unwrap(), p);
    }

    #[test]
    fn a_door_narrower_than_the_robot_is_closed() {
        // 扉の幅 0.4 m: 半径 0.2 m + 余裕 0.05 m のロボットは通れない
        let g = door_grid(25, 33);
        let opts = PlanOptions::default();
        assert_eq!(
            plan_cells(&g, (10, 29), (50, 29), &opts),
            Err(PlanError::NoPath)
        );
        let small = PlanOptions {
            inflation: Inflation {
                robot_radius: 0.1,
                safety_margin: 0.0,
                ..Default::default()
            },
            ..Default::default()
        };
        assert!(plan_cells(&g, (10, 29), (50, 29), &small).is_ok());
    }

    #[test]
    fn ends_near_a_wall_are_moved_to_free_cells() {
        let g = door_grid(20, 40);
        let opts = PlanOptions::default();
        let c = Costmap::new(g.clone(), opts.inflation);
        // 出発は壁の 0.1 m 左、目標は壁の 0.1 m 右
        let p = plan_cells(&g, (28, 5), (32, 50), &opts).unwrap();
        assert!(p.start_moved && p.goal_moved);
        let (s, t) = (p.cells[0], *p.cells.last().unwrap());
        assert!(c.is_passable(s.0 as i64, s.1 as i64, false));
        assert!(c.is_passable(t.0 as i64, t.1 as i64, false));
        assert_eq!(s, (24, 5));
        assert_eq!(t, (36, 50));
        // 壁の上は寄せずにエラー
        assert_eq!(
            plan_cells(&g, (30, 5), (32, 50), &opts),
            Err(PlanError::StartBlocked)
        );
        assert_eq!(
            plan_cells(&g, (28, 5), (30, 50), &opts),
            Err(PlanError::GoalBlocked)
        );
    }

    fn bfs(g: &OccupancyGrid, s: (usize, usize), t: (usize, usize)) -> Option<usize> {
        use std::collections::VecDeque;
        let mut seen = vec![false; g.width() * g.height()];
        let mut q = VecDeque::from([(s, 0)]);
        seen[s.1 * g.width() + s.0] = true;
        while let Some(((i, j), d)) = q.pop_front() {
            if (i, j) == t {
                return Some(d);
            }
            for (di, dj) in [(1i64, 0i64), (-1, 0), (0, 1), (0, -1)] {
                let (ni, nj) = (i as i64 + di, j as i64 + dj);
                if g.is_passable(ni, nj, false) {
                    let k = nj as usize * g.width() + ni as usize;
                    if !seen[k] {
                        seen[k] = true;
                        q.push_back(((ni as usize, nj as usize), d + 1));
                    }
                }
            }
        }
        None
    }
}
