set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

include_directories(
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

add_compile_definitions(
        _CONSOLE
        _WIN32_WINNT=0x0501
        NDEBUG
        NOGDI
        WIN32_LEAN_AND_MEAN
)

add_library(
        model STATIC
        src/FlyByWire.cpp
        src/FlyByWire.h
        src/FlyByWire_data.cpp
        src/FlyByWire_private.h
        src/FlyByWire_types.h
        src/rt_nonfinite.cpp
        src/rt_nonfinite.h
        src/rtGetInf.cpp
        src/rtGetInf.h
        src/rtGetNaN.cpp
        src/rtGetNaN.h
        src/rtwtypes.h
)

set_target_properties(
        model PROPERTIES
        OUTPUT_NAME "FlyByWireModel"
)
