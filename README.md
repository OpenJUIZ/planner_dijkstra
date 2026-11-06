# planner_dijkstra

[mobile_sim](https://github.com/OpenJUIZ/mobile_sim) の地図の上で、ロボットの姿勢から目標の姿勢までの経路(グローバルパス)を
ダイクストラ法で求める [juiz](https://github.com/OpenJUIZ/juiz) のアプリ(juizapp)です。
最初の版なので、ロボットの大きさ(フットプリント)は考えません。経路は壁すれすれを通ることがあります。

![画面](docs/ui.png)

- 地図: ROS map_server と同じ YAML と画像(既定は mobile_sim と同じ地図)。画面の「mobile_sim の地図を使う」で mobile_sim の地図に替えられる
- 出発: トピック `amcl_pose`(`juiz_std.geometry.Pose2DStamped`、[localization_amcl](https://github.com/OpenJUIZ/localization_amcl) が出す)。画面では mobile_sim の真の姿勢も使える
- 目標: トピック `goal`(Pose2DStamped)、または画面で地図を押して向きの方へドラッグ
- 経路: トピック `path` に `{header, ok, message, start, goal, poses: [Pose2D], length, cost, expanded, time_ms}` を出す。
  求まらないときは `ok: false` と `message`(目標が障害物の上、道が無い、など)

設計とマイルストーンは [docs/plan.md](docs/plan.md)。

## 動かす

juiz 0.2.4 以降と mobile_sim 0.3 系が要ります。

```sh
pip install juiz-cli==0.2.4
juiz run mobile_sim-0.3.3.juizapp        # 先に mobile_sim を動かしておく(http://localhost:8000/)
juiz run planner_dijkstra-0.1.0.juizapp  # 別の端末で
```

http://localhost:8003/ を開き、「目標を置く」を押してから地図の上でドラッグすると、経路を求めて描きます。

### Docker

```sh
docker compose -f docker/compose.yml up
```

mobile_sim(http://localhost:8000/)と planner_dijkstra(http://localhost:8003/)が一緒に動きます。
別の地図は `docker/maps/` に置き、環境変数 `PLANNER_MAP=/maps/<名前>.yaml` か、画面の「ファイルを読む」で読みます。

### 開発

```sh
cargo test --workspace
scripts/dev_plugin.sh                         # app/plugins/<target>/ にプラグインを置く
(cd ../mobile_sim && scripts/dev_plugin.sh && cd app && juiz -f app.conf -d --ipc mobile_sim)
cd app && juiz -f app.conf -d                 # 別の端末で
python3 scripts/integration_test.py --mobile-sim ../mobile_sim --ui ui_test.png   # 結合テストと画面のテスト
scripts/pack.sh                               # dist/ に .juizapp を作る
```

## juiz の ContainerProcess

コンテナ `dijkstra_planner`(app.conf では `planner0`):

| 名前 | 引数 | 戻り値 |
| --- | --- | --- |
| `planner_set_start` | `pose: Pose2DStamped` | なし(トピック `amcl_pose` を受ける) |
| `planner_set_goal` | `goal: Pose2DStamped` | 経路(トピック `goal` を受け、`path` に出す) |
| `planner_plan` | `start: Pose2D`, `goal: Pose2D` | 経路(`path` に出す) |
| `planner_get_path` | なし | 最後の経路 |
| `planner_set_map` | `image`, `resolution`, `origin`, `negate`, `occupied_thresh`, `free_thresh` | 地図の情報 |
| `planner_load_map` | `map`(YAML のパス) | 地図の情報 |
| `planner_get_map` / `planner_get_map_info` | なし | 地図の画像と情報 |
| `planner_get_state` | なし | 出発・目標・経路・地図の情報(画面用) |

コンテナの引数は `map`(YAML のパス。環境変数 `PLANNER_MAP` で上書き)、`frame_id`(既定 `map`)、`allow_unknown`(不明の格子を通るか、既定 false)、`diagonal`(8 近傍か、既定 true)。

## わかっている制約

localization_amcl と planner_dijkstra がどちらも mobile_sim だけを subsystem にした形では、トピック `amcl_pose` が planner_dijkstra に届きません
(juiz のトピックは subsystem の木をたどって届くため)。今は画面で mobile_sim の真の姿勢を使ってください。詳しくは [docs/plan.md](docs/plan.md#わかっている制約)。

## ライセンス

MIT OR Apache-2.0
