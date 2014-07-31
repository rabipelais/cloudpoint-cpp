find_package(PCL 1.3 REQUIRED COMPONENTS common io features kdtree surface)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

FIND_PACKAGE( Boost COMPONENTS program_options thread system filesystem REQUIRED )
SET( EIGEN3_INCLUDE_DIR "/usr/include/eigen3" )
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

add_executable(cloudpoint-cpp main.cpp featurebag.cpp gdiam.cpp)
target_link_libraries(cloudpoint-cpp pthread mongoclient ssl crypto ${Boost_LIBRARIES})
