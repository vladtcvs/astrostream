cmake_minimum_required(VERSION 3.16)
project(astrostream)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JPEG REQUIRED libjpeg)
pkg_check_modules(CFITSIO REQUIRED cfitsio)
pkg_check_modules(OBS REQUIRED libobs)
pkg_check_modules(INDI REQUIRED libindi)

add_library(astrostream SHARED src/plugin.cc src/indi_client.cc src/no_signal.c)
target_include_directories(astrostream PUBLIC ${JPEG_INCLUDE_DIRS})
target_include_directories(astrostream PUBLIC ${CFITSIO_INCLUDE_DIRS})
target_include_directories(astrostream PUBLIC ${OBS_INCLUDE_DIRS})
target_include_directories(astrostream PUBLIC ${INDI_INCLUDE_DIRS})

target_link_libraries(astrostream
    pthread
    stdc++
    indiclient
    ${OBS_LIBRARIES}
    ${CFITSIO_LIBRARIES}
    ${JPEG_LIBRARIES}
)
set_target_properties(astrostream PROPERTIES PREFIX "" SUFFIX ".so")
