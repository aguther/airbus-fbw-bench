cmake_minimum_required(VERSION 3.5)
project(airbus-fbw-bench LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if (WIN32)
    set(CMAKE_PREFIX_PATH "C:\\Qt\\5.15.0\\msvc2019_64")
endif ()

add_subdirectory(src-model)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5 COMPONENTS Widgets Charts REQUIRED)

include_directories(
        AFTER
        "$ENV{MSFS_SDK}/SimConnect SDK/include"
        "${CMAKE_SOURCE_DIR}/../simconnect-toolbox/sim-connect-interface/include"
        "${CMAKE_SOURCE_DIR}/src-model/src"
)

link_directories(
        "$ENV{MSFS_SDK}/SimConnect SDK/lib"
        "${CMAKE_SOURCE_DIR}/../simconnect-toolbox/build/sim-connect-interface/Release"
)

add_compile_definitions(
        NOGDI
        NOMINMAX
        WIN32_LEAN_AND_MEAN
)

add_executable(
        airbus-fbw-bench WIN32
        src/main.cpp
        src/Display/MainWindow.h
        src/Display/MainWindow.cpp
        src/Display/MainWindow.ui
        src/Controller/MainController.h
        src/Controller/MainController.cpp
        src/Display/ActualDemandChart.cpp
        src/Display/ActualDemandChart.h
)

target_link_libraries(
        airbus-fbw-bench PRIVATE
        Qt5::Widgets
        Qt5::Charts
        SimConnectInterface
        SimConnect
        model
)

add_custom_command(
        TARGET airbus-fbw-bench
        POST_BUILD
        COMMAND "windeployqt" --verbose 0 --no-compiler-runtime --no-translations --no-angle --no-opengl-sw "$<TARGET_FILE:airbus-fbw-bench>"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "$ENV{MSFS_SDK}/SimConnect SDK/lib/SimConnect.dll" "$<TARGET_FILE_DIR:airbus-fbw-bench>"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_SOURCE_DIR}/external/SimConnect/SimConnect.cfg" "$<TARGET_FILE_DIR:airbus-fbw-bench>"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_SOURCE_DIR}/../simconnect-toolbox/matlab/SimConnectInterface.dll" "$<TARGET_FILE_DIR:airbus-fbw-bench>"
)
