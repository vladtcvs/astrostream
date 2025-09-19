cmake_minimum_required(VERSION 3.16)
project(indilib_obs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Пути к OBS и INDI
set(OBS_INCLUDE_DIR "/usr/include/obs" CACHE PATH "OBS includes")
set(OBS_LIB_DIR "/usr/lib" CACHE PATH "OBS libs")
set(INDI_INCLUDE_DIR "/usr/local/include/libindi" CACHE PATH "INDI includes")
set(INDI_LIB_DIR "/usr/lib" CACHE PATH "INDI libs")

include_directories(${OBS_INCLUDE_DIR})
include_directories(${INDI_INCLUDE_DIR})
link_directories(${OBS_LIB_DIR})
link_directories(${INDI_LIB_DIR})

add_library(indiobs SHARED src/plugin.cc src/indi_client.cc src/no_signal.c)

# На Linux
target_link_libraries(indiobs
    obs
    indiclient
    pthread
    stdc++
    cfitsio
    jpeg
)
set_target_properties(indiobs PROPERTIES PREFIX "" SUFFIX ".so")


# На Windows нужно будет линковать obs.lib, indi.lib
