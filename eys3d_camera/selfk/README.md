# eYs3D Self-Calibration (selfk)

Public API header (`include/eys3d_selfcal.h`) plus a prebuilt archive per
architecture under `lib/<arch>/`. The driver links it automatically when
`EYS3D_WITH_SELFCAL` is ON (the default); see `../CMakeLists.txt`. No build
step is required beyond the normal `colcon build`.
