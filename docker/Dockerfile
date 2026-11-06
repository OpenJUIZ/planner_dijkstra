# syntax=docker/dockerfile:1
# planner_dijkstra のイメージ。リポジトリのルートを context にしてビルドする:
#   docker build -f docker/Dockerfile -t planner_dijkstra .
#
# Rust のプラグインは C ABI なので、juiz は pip の juiz-cli をそのまま使う(版は Cargo.toml の juiz_sdk と揃える)。
# mobile_sim のイメージは juiz run ではなく juiz -f で動くので、こちらも conf の HTTP の subsystem でつなぐ。
ARG JUIZ_VERSION=0.2.4

# プラグインをビルドしてテストする
FROM rust:1-bookworm AS build
ARG JUIZ_VERSION
WORKDIR /src
COPY . .
RUN --mount=type=cache,target=/usr/local/cargo/registry \
    --mount=type=cache,target=/src/target \
    grep -qx "juiz_sdk = \"=${JUIZ_VERSION}\"" Cargo.toml \
    && cargo test --workspace --locked \
    && cargo build -p planner_dijkstra --release --locked \
    && target=$(rustc -vV | sed -n 's/^host: //p') \
    && mkdir -p "/plugins/$target" \
    && cp target/release/libplanner_dijkstra.so "/plugins/$target/"

FROM python:3.12-slim-bookworm
ARG JUIZ_VERSION
RUN pip install --no-cache-dir "juiz-cli==${JUIZ_VERSION}"
# app/ と同じ形にする(conf からの相対パスでプラグインとブラウザ画面を探す)
COPY app/app.conf /opt/planner_dijkstra/
COPY app/web /opt/planner_dijkstra/web
COPY app/assets /opt/planner_dijkstra/assets
COPY --from=build /plugins /opt/planner_dijkstra/plugins
COPY LICENSE-MIT LICENSE-APACHE /opt/planner_dijkstra/
COPY docker/entrypoint.sh /usr/local/bin/planner_dijkstra-entrypoint
# conf の地図 (assets/maps/default.yaml) は JUIZ_APP_DIR から探す。ほかの地図は -v ./maps:/maps で渡し、
# 環境変数 PLANNER_MAP=/maps/<名前>.yaml か、画面の「ファイルを読む」で /maps/<名前>.yaml を読む
ENV JUIZ_APP_DIR=/opt/planner_dijkstra
WORKDIR /maps
VOLUME ["/maps"]
# つなぐ mobile_sim(juiz の HTTP)
ENV MOBILE_SIM_URL=http://mobile_sim:8000
EXPOSE 8003
ENTRYPOINT ["planner_dijkstra-entrypoint"]
