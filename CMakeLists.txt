cmake_minimum_required(VERSION 3.5)

project(XBOTCON  CXX)

add_executable(${PROJECT_NAME}
    "main.cpp"
    "src/distancedetection.cpp"
    "src/kalman_filter.cpp"
    "src/ImgPreProcess.cpp"
    "src/objClassifier.cpp"
    "src/getTarget2dPosition.cpp"
    "src/color_picker.cpp"
)

find_package(OpenCV 4 REQUIRED)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}

)

target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE
        ${OpenCV_LIBS}
)