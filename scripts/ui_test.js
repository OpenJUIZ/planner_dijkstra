// ブラウザ画面の確認(M3)。mobile_sim と planner_dijkstra を起動しておき、Playwright で開いて操作する
// 使い方: node scripts/ui_test.js [http://localhost:8003/] [スクリーンショットの保存先]
//   mobile_sim は既定の地図・既定の初期位置(-2, 0)で動いていること
const { chromium } = require("playwright");

const url = process.argv[2] || "http://localhost:8003/";
const shot = process.argv[3] || "ui_test.png";

function check(cond, msg) {
  if (!cond) { console.error("FAILED: " + msg); process.exit(1); }
  console.log("ok -", msg);
}

(async () => {
  const browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: 1100, height: 860 } });
  const errors = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.goto(url);
  await page.waitForSelector("#connection.ok", { timeout: 10000 });
  check(true, "connected to juiz");
  check(await page.isVisible("#sim_connection.ok"), "finds mobile_sim through the subsystem");
  check(await page.isVisible("#map_sim"), "offers the map of mobile_sim");
  await page.waitForFunction(() => /^x /.test(document.getElementById("s_truth").textContent));
  await page.waitForFunction(() => /×/.test(document.getElementById("s_map").textContent));

  // 世界座標 → ページ上の位置(地図は 10 m 四方、原点 -5, -5 をキャンバスいっぱいに描いている)
  const box = await page.locator("#map").boundingBox();
  const toPage = (x, y) => [box.x + ((x + 5) / 10) * box.width, box.y + ((5 - y) / 10) * box.height];
  const dragGoal = async (x, y, dx, dy) => {
    await page.click("#set_goal");
    const [sx, sy] = toPage(x, y);
    await page.mouse.move(sx, sy);
    await page.mouse.down();
    await page.mouse.move(sx + dx, sy + dy, { steps: 5 });
    await page.mouse.up();
  };

  // 出発は mobile_sim の真の姿勢にする(結合テストの後だと amcl_pose が来ていて、「自動」ではそちらを使うため)
  await page.selectOption("#start_source", "truth");

  // 仕切り壁の向こうの右の部屋に目標を置く(+y 向き)
  await page.click("#set_goal");
  check(await page.$eval("#set_goal", (b) => b.classList.contains("armed")), "the button arms the goal picker");
  await page.click("#set_goal"); // いったん外す(dragGoal がもう一度押す)
  await dragGoal(3.0, -2.5, 0, -30);
  await page.waitForFunction(() => document.getElementById("s_status").textContent === "経路あり", null, { timeout: 5000 });
  check(true, "a path is found to the right room");
  const len = parseFloat(await page.textContent("#s_length"));
  check(len > 5 && len < 12, `path length is plausible (${len} m)`);
  const goal = await page.textContent("#s_goal");
  check(/x 3\.0\d, y -2\.[45]\d, θ 1\.5\d/.test(goal), `goal pose is where it was dragged (${goal})`);
  const start = await page.textContent("#s_start");
  check(/x -2\.00, y 0\.00/.test(start), `start is the true pose of mobile_sim (${start})`);

  // 経路の青い線が描かれている
  const blue = await page.evaluate(() => {
    const c = document.getElementById("map");
    const d = c.getContext("2d").getImageData(0, 0, c.width, c.height).data;
    let n = 0;
    for (let i = 0; i < d.length; i += 4) if (d[i] < 80 && d[i + 1] > 80 && d[i + 1] < 130 && d[i + 2] > 190) n++;
    return n;
  });
  check(blue > 300, `canvas shows the path (${blue} blue px)`);
  await page.screenshot({ path: shot });

  // 壁の上の目標は「障害物の上」と出す(外周の壁 x = -4.95)
  await dragGoal(-4.97, 0.0, 0, -30);
  await page.waitForFunction(() => document.getElementById("s_status").textContent === "目標が障害物の上です", null, { timeout: 5000 });
  check(await page.$eval("#s_status", (e) => e.classList.contains("bad")), "a goal in a wall is reported");

  // 出発を amcl_pose にすると、プランナが覚えた姿勢から計画する(set_goal)
  const amcl = await page.textContent("#s_amcl");
  if (/^x /.test(amcl)) {
    await page.selectOption("#start_source", "amcl");
    await page.click("#replan_now");
    await page.waitForFunction((a) => document.getElementById("s_start").textContent === a.split("(")[0], amcl, { timeout: 5000 });
    check(true, `start is the amcl_pose (${amcl})`);
    await page.selectOption("#start_source", "truth");
  }

  // mobile_sim の地図を使う
  await page.click("#map_sim");
  await page.waitForFunction(() => document.getElementById("message").textContent.includes("mobile_sim の地図"));
  await page.waitForFunction(() => document.getElementById("s_map").textContent.startsWith("(image)"));
  check(true, "uses the map of mobile_sim");

  // 地図を替えた後も計画できる
  await dragGoal(0.5, 3.0, 30, 0);
  await page.waitForFunction(() => document.getElementById("s_status").textContent === "経路あり", null, { timeout: 5000 });
  check(true, "plans on the map from mobile_sim");

  check(errors.length === 0, `no page errors (${errors.join("; ")})`);
  await browser.close();
  console.log("OK");
})().catch((e) => { console.error(e); process.exit(1); });
