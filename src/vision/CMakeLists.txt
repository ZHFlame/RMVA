cmake_minimum_required(VERSION 3.8)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc videoio dnn)

add_executable(vision main.cpp
        armor.cpp
        detector.cpp
        sphere.cpp
        dynamic.cpp
        digit_detector.cpp
)

target_link_libraries(vision PRIVATE ${OpenCV_LIBS}) # Link OpenCV libraries


