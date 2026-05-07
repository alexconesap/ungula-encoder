cmake_minimum_required(VERSION 3.16)
project(ungula_encoder_lib_tests)

set(CMAKE_CXX_STANDARD 17)

set(FETCHCONTENT_BASE_DIR ${CMAKE_CURRENT_LIST_DIR}/vendor)

include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
FetchContent_MakeAvailable(googletest)

# ---------------------------------------------------------------------------
# Dependencies. UngulaCore (time API), UngulaHal (multiplexer interface,
# I2C master, GPIO) and EmblogX (logging from inside i_encoder.cpp /
# as5600.cpp) — all sibling libraries during local development.
# ---------------------------------------------------------------------------
option(USE_LOCAL_DEPS "Use local sibling lib*/ directories" ON)

if(USE_LOCAL_DEPS)
    set(LIB_CORE_SRC    ${CMAKE_CURRENT_LIST_DIR}/../../lib/src)
    set(LIB_HAL_SRC     ${CMAKE_CURRENT_LIST_DIR}/../../lib_hal/src)
    set(LIB_EMBLOGX_SRC ${CMAKE_CURRENT_LIST_DIR}/../../lib_emblogx/src)
else()
    FetchContent_Declare(
        ungula_core
        GIT_REPOSITORY https://github.com/alexconesap/ungula-core.git
        GIT_TAG main
    )
    FetchContent_MakeAvailable(ungula_core)
    set(LIB_CORE_SRC ${ungula_core_SOURCE_DIR}/src)

    FetchContent_Declare(
        ungula_hal
        GIT_REPOSITORY https://github.com/alexconesap/ungula-hal.git
        GIT_TAG main
    )
    FetchContent_MakeAvailable(ungula_hal)
    set(LIB_HAL_SRC ${ungula_hal_SOURCE_DIR}/src)

    FetchContent_Declare(
        emblogx
        GIT_REPOSITORY https://github.com/alexconesap/emblogx.git
        GIT_TAG main
    )
    FetchContent_MakeAvailable(emblogx)
    set(LIB_EMBLOGX_SRC ${emblogx_SOURCE_DIR}/src)
endif()

set(LIB_ENCODER_SRC ${CMAKE_CURRENT_LIST_DIR}/../src)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}
    ${LIB_CORE_SRC}
    ${LIB_HAL_SRC}
    ${LIB_EMBLOGX_SRC}
    ${LIB_ENCODER_SRC}
)

add_compile_definitions(
    EMBLOGX_BACKEND_STDIO
    EMBLOGX_LOG_PREFIX="[TEST] "
    EMBLOGX_ENABLE_SINK_CONSOLE=1
    EMBLOGX_ENABLE_SINK_MEMORY=0
    EMBLOGX_ENABLE_SINK_HTTP=0
    EMBLOGX_ENABLE_SINK_SD=0
)

# ---------------------------------------------------------------------------
# Encoder interface tests — pure host. Uses EncoderFake + MultiplexerFake
# to cover both deployment modes (with/without multiplexer) and to catch
# any drift in the IEncoder interface contract.
# ---------------------------------------------------------------------------
add_executable(test_ungula_encoder
    test_encoder.cpp
    ${LIB_ENCODER_SRC}/ungula/encoder/i_encoder.cpp
    ${LIB_HAL_SRC}/ungula/hal/multiplexer/i_multiplexer.cpp
    ${LIB_EMBLOGX_SRC}/emblogx/logger_core.cpp
    ${LIB_EMBLOGX_SRC}/emblogx/async_dispatcher.cpp
    ${LIB_EMBLOGX_SRC}/emblogx/sinks/console_sink.cpp
)
target_link_libraries(test_ungula_encoder gtest_main)

# ---------------------------------------------------------------------------
# AS5600 driver compile-and-link smoke test through the desktop HAL stubs.
# Verifies the driver compiles without warnings, links against the i2c
# stub, and stays a valid IEncoder.
# ---------------------------------------------------------------------------
add_executable(test_ungula_encoder_as5600
    test_as5600.cpp
    ${LIB_ENCODER_SRC}/ungula/encoder/i_encoder.cpp
    ${LIB_ENCODER_SRC}/ungula/encoder/drivers/as5600.cpp
    ${LIB_HAL_SRC}/ungula/hal/multiplexer/i_multiplexer.cpp
    ${LIB_HAL_SRC}/ungula/hal/i2c/platforms/i2c_master_default.cpp
    ${LIB_EMBLOGX_SRC}/emblogx/logger_core.cpp
    ${LIB_EMBLOGX_SRC}/emblogx/async_dispatcher.cpp
    ${LIB_EMBLOGX_SRC}/emblogx/sinks/console_sink.cpp
)
target_link_libraries(test_ungula_encoder_as5600 gtest_main)

enable_testing()
include(GoogleTest)
gtest_discover_tests(test_ungula_encoder)
gtest_discover_tests(test_ungula_encoder_as5600)
