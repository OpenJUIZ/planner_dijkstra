# planner_dijkstra の計画

mobile_sim の地図・ロボットの姿勢・目標の姿勢を受け取り、地図の上の経路(グローバルパス)を求める juizapp。
最初の版は、ロボットの大きさ(フットプリント)を考えず、占有格子の上でダイクストラ法を解く。

依頼: ysuga(2026-11-06)「mobile_sim に表示されたマップ情報と、ロボットの自己位置、目標位置姿勢を受け取って、
グローバルなパスプランニングをするアプリを作りたい。ロボットのフットプリントを考えずにダイクストラ法で解くモノを最初に作りたい」

## 方針

| 項目 | 決めたこと | 理由 |
| --- | --- | --- |
| 言語 | Rust(mobile_sim と同じ) | 地図の読み方(ROS map_server の YAML、画像の判定)を mobile_sim とそろえやすい。juiz 0.2 の Rust プラグインは C ABI なので rustc の版を問わない |
| 形 | juizapp、`kind: app`、`depends: mobile_sim >=0.3.1, <0.4` | mapper_gmapping・localization_amcl と同じ。画面が mobile_sim の地図と真の姿勢を使う |
| ポート | HTTP 8003、IPC 名 `planner_dijkstra` | mobile_sim 8000、mapper_gmapping 8001、localization_amcl 8002 の次 |
| 地図 | 自分で YAML を読む(既定は mobile_sim と同じ地図)。`planner_set_map` で画像を渡して差し替える | 画面の「mobile_sim の地図を使う」で mobile_sim の `get_map`・`get_map_info` をそのまま渡せる(localization_amcl の `amcl_set_map` と同じ形) |
| 格子の判定 | map_server の trinary(占有 > occupied_thresh、空き < free_thresh、ほかは不明)。不明は既定で通らない(`allow_unknown`) | mobile_sim・AMCL と同じ |
| 探索 | 8 近傍(`diagonal: false` で 4 近傍)、辺の重みは格子の中心どうしの距離。斜めは角をかすめない(間の 2 つの格子も空きのときだけ) | 壁の角をすり抜けない |
| フットプリント | 0.1.0 は考えない。0.2.0 から障害物を `robot_radius + safety_margin` だけ膨らませ、`inflation_radius` まで重みを上げる(#8) | 0.1.0 の経路は壁すれすれを通り、planner_dwa(半径 0.2 m)が扉の角で stuck になった |
| 出発の姿勢 | トピック `amcl_pose`(Pose2DStamped)で覚える。画面では、来ていなければ mobile_sim の真の姿勢を使う | 自己位置は localization_amcl が出す。AMCL が無くても試せるように |
| 目標 | トピック `goal`(Pose2DStamped)、または画面で地図をクリック・ドラッグ | RViz の 2D Nav Goal と同じ操作 |
| 経路 | トピック `path` に出す。`{header, ok, message, start, goal, poses: [Pose2D], length, cost, expanded, time_ms}` | juiz_std に Path の型が無いので JSON の object。求まらないときは `ok: false` と理由(トピックで動かすのでエラーにしない) |

## ContainerProcess(コンテナ `dijkstra_planner`)

| 名前 | 引数 | 戻り値 | conf でのつなぎ |
| --- | --- | --- | --- |
| `planner_set_start` | `pose: Pose2DStamped` | なし | subscribes `amcl_pose` |
| `planner_set_goal` | `goal: Pose2DStamped` | 経路 | subscribes `goal`、publishes `path` |
| `planner_plan` | `start: Pose2D`, `goal: Pose2D` | 経路 | publishes `path` |
| `planner_get_path` | なし | 最後の経路(readonly) | |
| `planner_set_map` | `image`, `resolution`, `origin`, `negate`, `occupied_thresh`, `free_thresh` | 地図の情報 | |
| `planner_load_map` | `map`(YAML のパス) | 地図の情報 | |
| `planner_get_map` / `planner_get_map_info` | なし | 地図の画像(占有 0、空き 254、不明 205)と情報(readonly) | |
| `planner_get_state` | なし | 画面用: 出発・目標・経路・地図の情報(readonly) | |

コンテナの引数: `map`(YAML、`PLANNER_MAP` で上書き、相対パスは `JUIZ_APP_DIR` から)、`frame_id`(既定 `map`)、`allow_unknown`(既定 false)、`diagonal`(既定 true)。

## マイルストーン

| | 内容 | 確かめ方 |
| --- | --- | --- |
| M0 | 雛形: Cargo workspace(`planner_dijkstra_core`・`planner_dijkstra`)、CI(fmt・clippy・test) | CI |
| M1 | 中身: 占有格子(YAML・画像)、ダイクストラ法、経路を姿勢の列にする | 単体テスト(迷路で幅優先探索と一致、角をかすめない、不明の扱い、端が障害物・地図の外) |
| M2 | juiz コンポーネントと app.conf。mobile_sim とつなぐ | `scripts/integration_test.py`: 真の姿勢から右の部屋へ、仕切り壁の出入口を通る。壁の上の目標、地図の差し替え、トピック amcl_pose・goal |
| M3 | ブラウザ画面(http://localhost:8003/): 地図・経路・ロボットを描き、クリックで目標を置く。地図の差し替え | `scripts/ui_test.js`(Playwright) |
| M4 | `.juizapp` と release.yml(5 ターゲット)。パッケージを juiz run で mobile_sim と動かして試す | release.yml の pack ジョブ |
| M5 | Docker(ghcr.io/openjuiz/planner_dijkstra、amd64・arm64)と compose(mobile_sim と一緒に) | docker.yml が compose で結合テスト |

リリース(タグ v0.1.0)は ysuga が GitHub の画面で Release を作る(スレッドからはタグを打てない)。release.yml が `.juizapp` を載せ、docker.yml が `:0.1.0`・`:latest` を出す。

## わかっている制約

### localization_amcl の amcl_pose が届く System のつなぎ方

juiz のトピックは、subsystem の木(つないだ側とつながれた側)をたどって届く(`topic_request_subscribe`)。
mobile_sim を真ん中に localization_amcl と planner_dijkstra がそれぞれ mobile_sim を subsystem にした形(星形)では、
AMCL の `amcl_pose` は planner_dijkstra に届かない(試した: AMCL の代わりに mobile_sim のプラグインで `amcl_pose` を出す System を動かした)。

- planner_dijkstra → AMCL → mobile_sim の一本の木にすると届くが、planner の画面から mobile_sim の Process が見えなくなる(一覧は 1 段だけ)。
- planner_dijkstra が mobile_sim と AMCL の両方を subsystem にすると輪になり、起動時の `topic_request_subscribe` が行って戻ってきて
  IPC の応答待ちで止まり、planner_dijkstra が終了する(`receiving a response from ipc://… failed: Resource temporarily unavailable`)。

最初の版はこのまま(画面では mobile_sim の真の姿勢を使える。トピック `amcl_pose` は、届けば使う)。
juiz 側で、要求の転送に「通った System」を持たせて輪を止めれば、planner_dijkstra が mobile_sim と AMCL の両方をつなげる(OpenJUIZ/juiz#449)。

解決(2026-11-08、#5): navigation_manager 0.2.0 からは、根の navigation_hub が葉(localization_amcl・planner_dijkstra・planner_dwa)を
subsystem に持ち、葉の出すトピックを `nav_relay` で `nav/...` に中継する。`navigation-manager stack` は planner_dijkstra の conf の
subsystem を消し、`amcl_pose` を `nav/amcl_pose` に替えて起動するので、AMCL の推定姿勢が届く。planner_dijkstra 側の変更は要らない。

### ロボットの大きさ(#8、0.2.0)

- 距離: 各格子の中心から一番近い `Occupied` の格子の中心までの距離 `d`(不明の格子は膨らませない)
- `d <= robot_radius + safety_margin`(既定 0.25 m)は通れない。`d < inflation_radius`(既定 0.6 m)は 1 歩の重みを
  `1 + inflation_weight × ((inflation_radius - d) / (inflation_radius - lethal))²` 倍にする(ROS costmap_2d の inflation と同じ考え。減衰は 2 乗)
- 出発・目標が通れない範囲の空きの格子にあるときは、空きの格子をたどって(壁を越えずに)一番近い通れる格子に寄せる。
  寄せてよい長さは `2 × (robot_radius + safety_margin) + 2 格子`。出発を寄せたときは、出発の位置から寄せた格子へ向かう。
  目標を寄せたときは寄せた格子で終える(向きは目標の向き)。障害物の格子そのものの上はエラーのまま
- 確かめたこと: 既定の地図で左の部屋から (2.5, -3.0) へ。0.1.0 は扉の角から 0.05 m 以内を通ったが、0.2.0 は 0.25 m より離れ、扉の中ほどを通る

## この先(別の issue)

- ~~フットプリント: 地図の障害物をロボットの半径だけ膨らませたコストマップで探索する(壁から離れる重みも)~~ 0.2.0 で入れた(#8)
- A*(ヒューリスティック)と、経路の間引き・平滑化
- mobile_sim の画面に経路(トピック path)を重ねて描く
- 経路をたどる制御(cmd_vel を出すローカルプランナ)
