cmake_minimum_required(VERSION 3.25)
project(autobus LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
include(FetchContent)

FetchContent_Declare(
    nlohmann_json
    URL      https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
)
FetchContent_MakeAvailable(nlohmann_json)

FetchContent_Declare(
    fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt.git
    GIT_TAG        10.2.1
)
FetchContent_MakeAvailable(fmt)

# ---------------------------------------------------------------------------
# Targets
# ---------------------------------------------------------------------------
add_library(autobus_core STATIC
    source/pwm_controller.cpp
    source/speed_controller.cpp
    source/servo.cpp
)
target_include_directories(autobus_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(autobus_core PUBLIC
    nlohmann_json::nlohmann_json
    fmt::fmt
)
target_compile_options(autobus_core PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wconversion
    -Wsign-conversion
    -Wnull-dereference
    -Wdouble-promotion
    -Wformat=2
    $<$<CONFIG:Debug>:-g -O0>
    $<$<CONFIG:Release>:-O2>
)

add_executable(busctld source/main.cpp)
target_include_directories(busctld PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(busctld PRIVATE autobus_core)
target_compile_options(busctld PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wconversion
    -Wsign-conversion
    -Wnull-dereference
    -Wdouble-promotion
    -Wformat=2
    $<$<CONFIG:Debug>:-g -O0>
    $<$<CONFIG:Release>:-O2>
)

add_executable(pwmcli tools/pwmcli.cpp)
target_link_libraries(pwmcli PRIVATE autobus_core)
target_compile_options(pwmcli PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wconversion
    -Wsign-conversion
    -Wnull-dereference
    -Wdouble-promotion
    -Wformat=2
    $<$<CONFIG:Debug>:-g -O0>
    $<$<CONFIG:Release>:-O2>
)
