cmake_minimum_required(VERSION 3.8)
project(plan_env)

# C++ 표준 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

# 메시지 파일 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MultipleMasksWithConfidence.msg"
  "msg/SemanticScore.msg"
  "msg/FrontierScan.msg"
  "msg/SkeletonNode.msg"
  "msg/SkeletonEdge.msg"
  "msg/SkeletonGraph.msg"
  "msg/SkeletonFrontierLink.msg"
  "msg/FrontierMemoryCell.msg"
  "msg/FrontierMemoryNode.msg"
  "msg/FrontierMemoryState.msg"
  "msg/TopologyObservation.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs nav_msgs
)

# Include 디렉토리 설정
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

# 라이브러리 생성
add_library(plan_env_lib SHARED
  src/sdf_map2d.cpp
  src/map_ros.cpp
  src/map_ros_memory.cpp
  src/raycast2d.cpp
  src/perception_utils2d.cpp
  src/frontier_map2d.cpp
  src/object_map2d.cpp
  src/value_map2d.cpp
  src/topology_graph.cpp
  src/sparse_topology_observer.cpp
  src/skeleton_route.cpp
  src/skeleton_graph.cpp
  src/skeleton_frontier_links.cpp
  src/skeleton_graph_manager.cpp
)

# Use the already-installed contrib module when available; otherwise the core
# includes a small Zhang-Suen implementation with no new package requirement.
if(TARGET opencv_ximgproc)
  target_compile_definitions(plan_env_lib PRIVATE APEXNAV_HAVE_XIMGPROC)
endif()

target_include_directories(plan_env_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(plan_env_lib
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  message_filters
  pcl_conversions
  tf2
  tf2_ros
)

target_link_libraries(plan_env_lib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# 생성된 메시지 인터페이스 연결
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(plan_env_lib "${cpp_typesupport_target}")

# 설치
install(TARGETS plan_env_lib
  EXPORT plan_env_libExport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Export
ament_export_include_directories(include)
ament_export_libraries(plan_env_lib)
ament_export_targets(plan_env_libExport HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  message_filters
  pcl_conversions
  tf2
  tf2_ros
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(sparse_topology_test ../../../tests/sparse_topology_test.cpp src/topology_graph.cpp)
  add_test(NAME sparse_topology COMMAND sparse_topology_test)
  add_executable(target_object_fusion_test ../../../tests/target_object_fusion.cpp)
  target_link_libraries(target_object_fusion_test plan_env_lib)
  add_test(NAME target_object_fusion COMMAND target_object_fusion_test)
  set_tests_properties(target_object_fusion PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=94" TIMEOUT 30)
  add_executable(skeleton_graph_test tests/skeleton_graph_test.cpp src/skeleton_graph.cpp)
  target_link_libraries(skeleton_graph_test ${OpenCV_LIBRARIES})
  if(TARGET opencv_ximgproc)
    target_compile_definitions(skeleton_graph_test PRIVATE APEXNAV_HAVE_XIMGPROC)
  endif()
  add_test(NAME skeleton_graph COMMAND skeleton_graph_test)
  add_executable(skeleton_graph_fallback_test tests/skeleton_graph_test.cpp src/skeleton_graph.cpp)
  target_link_libraries(skeleton_graph_fallback_test ${OpenCV_LIBRARIES})
  add_test(NAME skeleton_graph_fallback COMMAND skeleton_graph_fallback_test)
  add_executable(skeleton_frontier_links_test tests/skeleton_frontier_links_test.cpp src/skeleton_graph.cpp src/skeleton_frontier_links.cpp)
  target_link_libraries(skeleton_frontier_links_test ${OpenCV_LIBRARIES})
  add_test(NAME skeleton_frontier_links COMMAND skeleton_frontier_links_test)
  add_executable(skeleton_route_test tests/skeleton_route_test.cpp src/skeleton_route.cpp src/skeleton_frontier_links.cpp src/skeleton_graph.cpp)
  target_link_libraries(skeleton_route_test ${OpenCV_LIBRARIES})
  add_test(NAME skeleton_route COMMAND skeleton_route_test)
  add_executable(skeleton_manager_test tests/skeleton_manager_test.cpp)
  target_link_libraries(skeleton_manager_test plan_env_lib)
  add_test(NAME skeleton_manager COMMAND skeleton_manager_test)
  set_tests_properties(skeleton_manager PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=94" TIMEOUT 30)
endif()

ament_package()
