add_library(vins_lib
  src/estimator/parameters.cpp
  src/estimator/estimator.cpp
  src/estimator/feature_manager.cpp
  src/factor/pose_local_parameterization.cpp
  src/factor/projectionTwoFrameOneCamFactor.cpp
  src/factor/projectionTwoFrameTwoCamFactor.cpp
  src/factor/projectionOneFrameTwoCamFactor.cpp
  src/factor/marginalization_factor.cpp
  src/utility/utility.cpp
  src/utility/Print.cpp
  src/initial/solve_5pts.cpp
  src/initial/initial_aligment.cpp
  src/initial/initial_sfm.cpp
  src/initial/initial_ex_rotation.cpp
  src/featureTracker/feature_tracker.cpp
)
target_link_libraries(vins_lib camera_models pthread ${OpenCV_LIBS} ${CERES_LIBRARIES})
