set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

include_directories(
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

file(
        STRINGS
        src/defines.txt
        MODEL_DEFINES
)

add_compile_definitions(
        _CONSOLE
        _WIN32_WINNT=0x0501
        NDEBUG
        NOGDI
        WIN32_LEAN_AND_MEAN
        ${MODEL_DEFINES}
)

add_library(
        model STATIC
        src/fbw.cpp
        src/fbw.h
        src/fbw_data.cpp
        src/fbw_private.h
        src/fbw_types.h
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
