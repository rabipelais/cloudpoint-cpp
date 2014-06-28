add_executable(cloudpoint-cpp main.cpp)
target_link_libraries(cloudpoint-cpp ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${PCL_KDTREE_LIBRARIES})
