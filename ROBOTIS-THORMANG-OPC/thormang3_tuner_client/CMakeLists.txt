##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(thormang3_tuner_client)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS
  rqt_gui
  rqt_gui_cpp
  roscpp
  roslib
  std_msgs
  sensor_msgs
  geometry_msgs
  thormang3_tuning_module_msgs
  eigen_conversions
  robotis_math
)

find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

set(thormang3_tuner_client_SRCS
  src/main.cpp
  src/main_window.cpp
  src/qnode.cpp
)

set(thormang3_tuner_client_HDRS
  include/thormang3_tuner_client/main_window.hpp
  include/thormang3_tuner_client/qnode.hpp
)

set(thormang3_tuner_client_UIS
  ui/main_window.ui
)

set(thormang3_tuner_client_RESOURCES
  resources/images.qrc
)

set(thormang3_tuner_client_INCLUDE_DIRECTORIES
  include
  include/${PROJECT_NAME}
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}"
)
if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}")
endif()

# Use this to define what the package will export (e.g. libs, headers).
# exporting anything. 
catkin_package(
  INCLUDE_DIRS ${thormang3_tuner_client_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    rqt_gui
    rqt_gui_cpp
    roscpp
    roslib
    std_msgs
    sensor_msgs
    geometry_msgs
    thormang3_tuning_module_msgs
    eigen_conversions
    robotis_math
  DEPENDS Boost EIGEN3
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(thormang3_tuner_client_MOCS ${thormang3_tuner_client_HDRS})
else()
  qt4_wrap_cpp(thormang3_tuner_client_MOCS ${thormang3_tuner_client_HDRS})
endif()
# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}")
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_ui(thormang3_tuner_client_UIS_H ${thormang3_tuner_client_UIS})
else()
  qt4_wrap_ui(thormang3_tuner_client_UIS_H ${thormang3_tuner_client_UIS})
endif()
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_add_resources(thormang3_tuner_client_RCC ${thormang3_tuner_client_RESOURCES})
else()
  qt4_add_resources(thormang3_tuner_client_RCC ${thormang3_tuner_client_RESOURCES})
endif()


include_directories(
  ${thormang3_tuner_client_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME} ${thormang3_tuner_client_SRCS} ${thormang3_tuner_client_MOCS} ${thormang3_tuner_client_UIS_H} ${thormang3_tuner_client_RCC})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets ${Eigen3_LIBRARIES} ${YAML_CPP_LIBRARIES})
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${Eigen3_LIBRARIES} ${YAML_CPP_LIBRARIES})
endif()

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

