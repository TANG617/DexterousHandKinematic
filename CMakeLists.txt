cmake_minimum_required(VERSION 3.10)

project(kinematic LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCE_FILE
        main.c
        dexterous_hand.c
        dexterous_hand.h

)

add_executable(kinematic ${SOURCE_FILE})
