cmake_minimum_required(VERSION 3.5)


project(XBOTCON  CXX)
set(OpenCV_DIR "/home/pi/Downloads/opencv")
set(Eigen3_DIR "/home/pi/Downloads/eigen-3.4.0/cmake")
set(API_DIR "/home/pi/Desktop/VL53L0X_code/VL53L0X_1.0.4/Api")
add_definitions( -lwiringPi  )
find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
#find_library(libVL53L0X_Rasp.a ./)
message("WARNING" "OpenCV_INCLUDE_DIRS:" ${OpenCV_INCLUDE_DIRS})
message("WARNING" "OpenCV_LIBS:" ${OpenCV_LIBS})
add_executable(${PROJECT_NAME}
    "main.cpp"
    "src/distancedetection.cpp"
    "src/kalman_filter.cpp"
    "src/ImgPreProcess.cpp"
    "src/objClassifier.cpp"
    "src/getTarget2dPosition.cpp"
	"src/threads.cpp"
	"src/serial.cpp"
)

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED) 
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
		${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(
    ${PROJECT_NAME}
	PRIVATE
        ${OpenCV_LIBS}
		${WIRINGPI_LIBRARIES}  -lpthread
)
include_directories (
		"/home/pi/Desktop/xbotcon/include"
)


