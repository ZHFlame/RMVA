cmake_minimum_required(VERSION 3.8)
project(shooter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc videoio dnn)

add_executable(shooter ../shooter_node.cpp
        calculate.cpp
)

target_link_libraries(shooter PRIVATE ${OpenCV_LIBS}) # Link OpenCV libraries


