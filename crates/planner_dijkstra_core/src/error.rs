use std::path::PathBuf;

use thiserror::Error;

/// 地図を読むときのエラー
#[derive(Debug, Error)]
pub enum PlannerError {
    #[error("cannot read {path}: {source}")]
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
    #[error("invalid map YAML {path}: {source}")]
    MapYaml {
        path: PathBuf,
        source: serde_yaml::Error,
    },
    #[error("cannot read the map image {path}: {source}")]
    Image {
        path: PathBuf,
        source: image::ImageError,
    },
    #[error("invalid map: {0}")]
    InvalidMap(String),
}

/// 経路が求まらないときの理由
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum PlanError {
    #[error("the start is outside the map")]
    StartOutside,
    #[error("the goal is outside the map")]
    GoalOutside,
    #[error("the start is on an obstacle")]
    StartBlocked,
    #[error("the goal is on an obstacle")]
    GoalBlocked,
    #[error("the start is too close to obstacles and no free cell is near it")]
    StartTooClose,
    #[error("the goal is too close to obstacles and no free cell is near it")]
    GoalTooClose,
    #[error("no path from the start to the goal")]
    NoPath,
}
