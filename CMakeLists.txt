cmake_minimum_required(VERSION 2.6)
project(CarAI)

#set(CMAKE_MSVCIDE_RUN_PATH "ext/assimp-4.0.1/lib")

set(CMAKE_MSVCIDE_RUN_PATH "${PROJECT_SOURCE_DIR}/ext/assimp-4.0.1/lib"
    CACHE STATIC "MSVC IDE Run path" FORCE)

file(GLOB_RECURSE src_files ./src/*.cpp ./src/*.h)
file(GLOB_RECURSE ext_files ./ext/*.cpp ./ext/*.c ./ext/*.h)

add_executable(CarAI ${src_files} ${ext_files})

target_include_directories(CarAI PUBLIC "ext/AntTweakBar/include")
target_include_directories(CarAI PUBLIC "ext/assimp-4.0.1/include")
target_include_directories(CarAI PUBLIC "ext/bullet3-2.87/src")
target_include_directories(CarAI PUBLIC "ext/glad/include")
target_include_directories(CarAI PUBLIC "ext/glfw-3.2.1/include")
target_include_directories(CarAI PUBLIC "ext/glm-0.9.9-a1")
target_include_directories(CarAI PUBLIC "ext/inih/cpp")
target_include_directories(CarAI PUBLIC "ext/lodepng-20170917")

#link_directories("${PROJECT_SOURCE_DIR}ext/AntTweakBar/lib" "${PROJECT_SOURCE_DIR}ext/assimp-4.0.1/lib" "${PROJECT_SOURCE_DIR}ext/glfw-3.2.1/lib")
#target_link_libraries(CarAI AntTweakBar64.lib glfw3.lib assimp-vc140-mt.lib)
target_link_libraries(CarAI ${PROJECT_SOURCE_DIR}/ext/glfw-3.2.1/lib/glfw3.lib)
target_link_libraries(CarAI ${PROJECT_SOURCE_DIR}/ext/assimp-4.0.1/lib/assimp-vc140-mt.lib)
target_link_libraries(CarAI ${PROJECT_SOURCE_DIR}/ext/AntTweakBar/lib/AntTweakBar64.lib)




source_group("ext"  FILES ${ext_files})
