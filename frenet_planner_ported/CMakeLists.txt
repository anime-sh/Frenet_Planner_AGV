cmake_minimum_required(VERSION 3.5)
project(frenet_planner_ported)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(costmap_2d REQUIRED)
#find_package(base_local_planner REQUIRED)
#find_package(costmap_converter REQUIRED)

add_executable(frenet_ros_obst  src/frenet_ros_obst.cpp  src/frenet_planner.cpp src/cubic_spline_planner.cpp src/polynomials.cpp)
ament_target_dependencies(frenet_ros_obst rclcpp geometry_msgs nav_msgs rclpy std_msgs)

install(TARGETS
  frenet_ros_obst
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

SET (PYTHON_PATH "/usr/include/python2.7/")       #imp to include python.h in matplotlibcpp.h

SET (TBB_DIR  ${PROJECT_SOURCE_DIR}/TBBConfig.cmake) 
find_package(TBB)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${CMAKE_MODULE_PATH}")
find_package(Boost REQUIRED COMPONENTS system thread graph)


find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})	
endif()

include_directories ($ {rclcpp_INCLUDE_DIRS}
                    $ {geometry_msgs_INCLUDE_DIRS}
                    $ {nav_msgs_INCLUDE_DIRS}
                    $ {rclcpp_INCLUDE_DIRS}
                    $ {rclpy_INCLUDE_DIRS}
                    $ {std_msgs_INCLUDE_DIRS}
                    $ {costmap_2d_INCLUDE_DIRS}
                    $ {base_local_planner_INCLUDE_DIRS}
                    $ {costmap_converter_INCLUDE_DIRS}
                    ./include
                    ./launch
                    ./config
                                                      )

                
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(PythonLibs 2.7)
target_include_directories(frenet_ros_obst PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(frenet_ros_obst ${PYTHON_LIBRARIES})

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

ament_package()
