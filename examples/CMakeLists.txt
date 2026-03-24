cmake_minimum_required(VERSION 3.10)

project(example LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost 1.56.0)
if (NOT Boost_FOUND)
    set(CMAKE_CXX_STANDARD 20)
    set(Boost_INCLUDE_DIRS "")
endif ()

find_package(PkgConfig REQUIRED)
pkg_check_modules(BABELTRACE REQUIRED babeltrace2)

add_library(epitome SHARED epitome.cpp)
set_target_properties(epitome PROPERTIES
        # Do not use the default preprocessor symbol for shared libraries
        DEFINE_SYMBOL ""

        # Do not prefix the name of the library
        PREFIX ""
)
target_include_directories(epitome PRIVATE ../include ${Boost_INCLUDE_DIRS} ${BABELTRACE_INCLUDE_DIRS})
target_compile_options(epitome PRIVATE ${BABELTRACE_CFLAGS_OTHER})
target_link_libraries(epitome PRIVATE ${BABELTRACE_LIBRARIES})
