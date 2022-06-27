#############################
# CMake - Serial Bridge - root
#############################

cmake_minimum_required(VERSION 3.5.1)

project("AMM_Serial_Bridge")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

set(VERSION_TYPE "beta" CACHE STRING "version type" FORCE)
set(PROJECT_VERSION_MAJOR 1)
set(PROJECT_VERSION_MINOR 1)
set(PROJECT_VERSION_PATCH 2)
site_name(VERSION_HOST) # read hostname to VERSION_HOST
set(VERSION_HOST "${VERSION_HOST}" CACHE STRING "host of build" FORCE)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)

if (MSVC)
    add_compile_options(/std:c++14 /W3)
    add_definitions(-DNOMINMAX)
    add_definitions(-DWIN32_LEAN_AND_MEAN)
    add_definitions(-DBOOST_ALL_NO_LIB)
    add_definitions(-DBOOST_ALL_DYN_LINK)
    add_definitions(-DBOOST_UUID_FORCE_AUTO_LINK)
    add_definitions(-DBOOST_DATE_TIME_STATIC_LIB)
    add_definitions(-D_WIN32_WINNT=0x0A00)
    add_definitions(-D_DEBUG)
    add_definitions(-D_ITERATOR_DEBUG_LEVEL=2)
    #set(Boost_NO_SYSTEM_PATHS true)
    set(Boost_DETAILED_FAILURE_MSG ON)
    set(Boost_THREADAPI win32)
    set(Boost_USE_STATIC_LIBS ON)
    set(Boost_USE_DEBUG_LIBS ON)
    set(Boost_USE_RELEASE_LIBS ON)
    set(Boost_USE_MULTITHREADED ON)
    set(Boost_USE_STATIC_RUNTIME OFF)
    if (CMAKE_SIZEOF_VOID_P MATCHES "8")
        set(CMAKE_INSTALL_PREFIX "C:/Program Files/${CMAKE_PROJECT_NAME}")
    else ()
        set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/${CMAKE_PROJECT_NAME}")
    endif ()
else ()
    add_compile_options(-std=c++14)
    add_compile_options(-O0)
endif ()

FIND_PACKAGE(Boost 1.67.0 REQUIRED COMPONENTS thread system)

if (MSVC)
    find_package(TinyXML2 REQUIRED NO_SYSTEM_ENVIRONMENT_PATH)
else ()
    find_package(TinyXML2 REQUIRED)
endif ()

find_package(amm_std REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${TinyXML2_INCLUDE_DIRS})

add_subdirectory(src)

file(COPY config DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

message(STATUS "")
message(STATUS "    == Final overview for ${PROJECT_NAME} ==")
message(STATUS "Version:              ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH} ${VERSION_TYPE} @ ${VERSION_HOST}")
message(STATUS "Install prefix:       ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Output:               ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "Compiler:             ${CMAKE_CXX_COMPILER}")
message(STATUS "CMAKE_BUILD_TYPE:     ${CMAKE_BUILD_TYPE}")
message(STATUS "")

include(Packing)
