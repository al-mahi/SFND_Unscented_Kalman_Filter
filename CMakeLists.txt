cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (ukf_highway src/main.cpp src/ukf.cpp src/tools.cpp src/render/render.cpp)
target_link_libraries (ukf_highway ${PCL_LIBRARIES})

#add_executable(kf1 src/exercises/KF1.cpp)
#add_executable(kf2 src/exercises/KF2/main.cpp src/exercises/KF2/kalman_filter.cpp src/exercises/KF2/tracking.cpp)
#add_executable(jacobian src/exercises/jacobian.cpp)
#add_executable(rmse src/exercises/RMSE.cpp)
#add_executable(sigma_points src/exercises/sigma_points/main.cpp src/exercises/sigma_points/ukf.cpp)
#add_executable(ukf_augmentation src/exercises/ukf_augmentation/main.cpp src/exercises/ukf_augmentation/ukf.cpp)
add_executable(ukf_prediction src/exercises/ukf_prediction/main.cpp src/exercises/ukf_prediction/ukf.cpp)
#add_executable(ukf_prediction_mean_cov src/exercises/ukf_prediction_mean_cov/main.cpp src/exercises/ukf_prediction_mean_cov/ukf.cpp)
#add_executable(prediction_radar_measurement src/exercises/predict_radar_measurement/main.cpp src/exercises/predict_radar_measurement/ukf.cpp)
#add_executable(ukf_update src/exercises/ukf_update/main.cpp src/exercises/ukf_update/ukf.cpp)
