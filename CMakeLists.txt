cmake_minimum_required(VERSION 2.8.3)
project(roboy_control_center)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g")

find_package(catkin REQUIRED COMPONENTS common_utilities roscpp roslint rqt_gui rqt_gui_cpp std_msgs roboy_middleware_msgs)
FIND_PACKAGE(yaml-cpp REQUIRED)

include_directories(include qcustomplot)

## System dependencies are found with CMake's conventions

if ("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
    find_package(Qt5Widgets REQUIRED)
else ()
    find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
    include(${QT_USE_FILE})
endif ()

catkin_python_setup()

roslint_cpp()

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp rqt_gui_cpp std_msgs roboy_middleware_msgs common_utilities
        DEPENDS YAML_CPP)

if ("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
    qt5_wrap_cpp(roboy_control_center_MOCS include/roboy_control_center/roboy_control_center.hpp qcustomplot/qcustomplot.h)
    qt5_wrap_ui(roboy_control_center_UIS_H src/roboy_control_center.ui)
else ()
    qt4_wrap_cpp(roboy_control_center_MOCS include/roboy_control_center/roboy_control_center.hpp qcustomplot/qcustomplot.h)
    qt4_wrap_ui(roboy_control_center_UIS_H src/roboy_control_center.ui)
endif ()

include_directories(
        include/
        include/roboy_control_center/
        ${CMAKE_CURRENT_BINARY_DIR}/..
        ${catkin_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
)

add_library(roboy_control_center src/roboy_control_center.cpp ${roboy_control_center_MOCS} ${roboy_control_center_UIS_H}
        qcustomplot/qcustomplot.cpp)

add_dependencies(roboy_control_center roboy_middleware_msgs_gencpp)

target_link_libraries(roboy_control_center ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} -ldl)

if ("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
    target_link_libraries(roboy_control_center Qt5::Widgets)
else ()
    target_link_libraries(roboy_control_center ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif ()

#############
## Install ##
#############

install(TARGETS roboy_control_center
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/roboy_rqt_plugins
        )

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )