cmake_minimum_required(VERSION 3.14)
project(dm_imu_usb VERSION 1.0.0 LANGUAGES CXX)

# ── C++17 ──────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Build type default ────────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ── Compiler flags ────────────────────────────────────────────────
add_compile_options(-Wall -Wextra -Wpedantic)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O2)
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g -O0)
endif()

# ─────────────────────────────────────────────────────────────────
#  Library: dm_imu  (header-only interface + impl)
# ─────────────────────────────────────────────────────────────────
add_library(dm_imu_lib STATIC
    src/bsp_crc.cpp
    src/imu_driver.cpp
)

target_include_directories(dm_imu_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# pthread (Linux)
find_package(Threads REQUIRED)
target_link_libraries(dm_imu_lib PUBLIC Threads::Threads)

# ─────────────────────────────────────────────────────────────────
#  Executable: dm_imu_demo
# ─────────────────────────────────────────────────────────────────
add_executable(dm_imu_demo src/main.cpp)
target_link_libraries(dm_imu_demo PRIVATE dm_imu_lib)

# ─────────────────────────────────────────────────────────────────
#  Install rules (tuỳ chọn)
# ─────────────────────────────────────────────────────────────────
install(TARGETS dm_imu_lib dm_imu_demo
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# ─────────────────────────────────────────────────────────────────
#  Thông tin
# ─────────────────────────────────────────────────────────────────
message(STATUS "──────────────────────────────────────")
message(STATUS "  Project  : ${PROJECT_NAME} v${PROJECT_VERSION}")
message(STATUS "  Build    : ${CMAKE_BUILD_TYPE}")
message(STATUS "  Compiler : ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "──────────────────────────────────────")

# ── Calibration tool ─────────────────────────────────────────────
add_executable(dm_imu_calibrate src/calibrate.cpp)
target_link_libraries(dm_imu_calibrate PRIVATE dm_imu_lib)
target_include_directories(dm_imu_calibrate PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
