find_package(PCL 1.3 REQUIRED COMPONENTS common io features kdtree surface)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

FIND_PACKAGE( Boost COMPONENTS program_options thread system REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g" )

add_executable(cloudpoint-cpp main.cpp featurebag.cpp)
target_link_libraries(cloudpoint-cpp ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${PCL_KDTREE_LIBRARIES} ${PCL_SURFACE_LIBRARIES} pthread mongoclient ssl crypto ${Boost_LIBRARIES})
