find_package(PCL 1.3 REQUIRED COMPONENTS common io features kdtree)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

FIND_PACKAGE( Boost COMPONENTS program_options thread system REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )


add_executable(cloudpoint-cpp main.cpp)
target_link_libraries(cloudpoint-cpp ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${PCL_KDTREE_LIBRARIES} pthread mongoclient ssl crypto ${Boost_LIBRARIES})
