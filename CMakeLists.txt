cmake_minimum_required(VERSION 3.8)
project(servo-monitoring)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(tachimawari_interfaces REQUIRED)
find_package(tachimawari REQUIRED)
find_package(prometheus-cpp CONFIG REQUIRED)

add_library(${PROJECT_NAME} SHARED
  "src/${PROJECT_NAME}/exporter.cpp"
  )
  
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tachimawari_interfaces
  tachimawari
)

install(DIRECTORY "include" DESTINATION ".")

install(TARGETS ${PROJECT_NAME}
EXPORT export_${PROJECT_NAME}
ARCHIVE DESTINATION "lib"
LIBRARY DESTINATION "lib"
RUNTIME DESTINATION "bin")

add_executable(jalan src/main-monitor.cpp)
target_include_directories(jalan PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
$<INSTALL_INTERFACE:include>)
target_link_libraries(jalan ${PROJECT_NAME})

target_link_libraries(jalan #ROS2 library
                        prometheus-cpp::push
                        prometheus-cpp::pull
                        prometheus-cpp::core)

install(TARGETS
      jalan
      DESTINATION lib/${PROJECT_NAME}
)

ament_package()