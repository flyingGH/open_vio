include_directories(
  "include"
  ${CERES_INCLUDE_DIRS}
)

add_library(camera_models
  src/camera_models/Camera.cc
  src/camera_models/CameraFactory.cc
  src/camera_models/PinholeCamera.cc
  src/camera_models/PinholeFullCamera.cc
  src/camera_models/CataCamera.cc
  src/camera_models/EquidistantCamera.cc
  src/sparse_graph/Transform.cc
  src/gpl/gpl.cc
  src/gpl/EigenQuaternionParameterization.cc
)
target_link_libraries(camera_models ${OpenCV_LIBS} ${CERES_LIBRARIES})