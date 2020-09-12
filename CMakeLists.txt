cmake_minimum_required(VERSION 3.5)
project(airbus-fbw-bench LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if (WIN32)
    set(CMAKE_PREFIX_PATH "C:\\Qt\\5.15.0\\msvc2019_64")
endif ()

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5 COMPONENTS Widgets Charts REQUIRED)

include_directories(
        ${CMAKE_SOURCE_DIR}/SimConnect
)
link_directories(
        ${CMAKE_SOURCE_DIR}/SimConnect
)

add_executable(
        airbus-fbw-bench WIN32
        main.cpp
        Display/MainWindow.h
        Display/MainWindow.cpp
        Display/MainWindow.ui
        Controller/MainController.h
        Controller/MainController.cpp
        Controller/DataDefinition.h
        PidController/pid.h
        PidController/pid.cpp
        Display/ActualDemandChart.cpp
        Display/ActualDemandChart.h
        Pitch/LawPitch.cpp Pitch/LawPitch.h)

add_library(
        SimConnect.lib;
        user32.lib;
        shlwapi.lib;
        Ws2_32.lib;
)

target_link_libraries(
        airbus-fbw-bench PRIVATE
        Qt5::Widgets
        Qt5::Charts
        SimConnect
)

add_custom_command(
        TARGET airbus-fbw-bench
        POST_BUILD
        COMMAND "windeployqt" --verbose 0 --no-compiler-runtime --no-translations --no-angle --no-opengl-sw "$<TARGET_FILE:airbus-fbw-bench>"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${PROJECT_SOURCE_DIR}/SimConnect/SimConnect.dll" $<TARGET_FILE_DIR:airbus-fbw-bench>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${PROJECT_SOURCE_DIR}/SimConnect/SimConnect.cfg" $<TARGET_FILE_DIR:airbus-fbw-bench>
)
