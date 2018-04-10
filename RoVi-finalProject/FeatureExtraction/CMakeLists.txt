cmake_minimum_required(VERSION 2.8)
project( FeatureExtraction )
set(CMAKE_CXX_STANDARD 11)
find_package( OpenCV 3 REQUIRED )
add_executable(FeatureExtraction src/main.cpp src/Markers.cpp src/Markers.h src/FeatureExtraction.cpp src/FeatureExtraction.h)
target_link_libraries( FeatureExtraction ${OpenCV_LIBS})

