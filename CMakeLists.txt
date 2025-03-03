cmake_minimum_required(VERSION 3.10)

project(kinematic LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 指定源文件
set(SOURCE_FILE
        main.c
        dexterous_hand.c
        dexterous_hand.h

)

# 生成可执行文件
add_executable(kinematic ${SOURCE_FILE})

# 可选：添加编译选项
#target_compile_options(my_program PRIVATE -Wall -Wextra -Wpedantic)

# 可选：如果需要链接库，可以在这里添加
# target_link_libraries(my_program PRIVATE some_library)