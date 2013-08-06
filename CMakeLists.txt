cmake_minimum_required(VERSION 2.8)
project(vlc-vaapi-enc)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules" ${CMAKE_MODULE_PATH})

set(vlc-vaapi-enc_SOURCES
    src/plugin.cpp
    src/encoder.cpp
    src/copy.c)
set(vlc-vaapi-enc_HEADERS
    src/encoder.h
    src/copy.h)

set(VlcPlugin_MODULE_STRING vaapi-enc)

find_package(Libva REQUIRED X11 DRM)
find_package(VlcPlugin REQUIRED)
find_package(CXX11 REQUIRED)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${CXX11_FLAGS})
set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} -std=gnu99)
add_definitions(${Libva_DEFINITIONS} ${VlcPlugin_DEFINITIONS})
include_directories(${Libva_INCLUDE_DIRS} ${VlcPlugin_INCLUDE_DIRS} src)

add_library(vaapi_enc_plugin SHARED ${vlc-vaapi-enc_SOURCES} ${vlc-vaapi-enc_HEADERS})
target_link_libraries(vaapi_enc_plugin ${Libva_LIBRARIES} ${VlcPlugin_LIBRARIES})

install_targets(/lib/vlc/plugins/codec vaapi_enc_plugin)
