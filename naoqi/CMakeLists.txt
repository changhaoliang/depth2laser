
message("Toolchain file: ${CMAKE_TOOLCHAIN_FILE}")
message("Current source dir: ${CMAKE_CURRENT_SOURCE_DIR}")

set (CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
MESSAGE( STATUS "CMAKE_CXX_FLAGS: " ${CMAKE_CXX_FLAGS} )

include_directories(../src)

#Creating depth2laser_library for naoqi
qi_create_lib(depth2laser_library
  SHARED
  ../src/depth2laser.cpp
)
qi_use_lib(depth2laser_library
  OPENCV2_CORE
  EIGEN3
)
qi_stage_lib(depth2laser_library)

#Creating executable
qi_create_bin(depth2laser_naoqi depth2laser_naoqi.cpp)
qi_create_bin(depth2laser_naoqi_gui depth2laser_naoqi_gui.cpp)
qi_use_lib(depth2laser_naoqi ALCOMMON ALPROXIES BOOST depth2laser_library OPENCV2_HIGHGUI)
qi_use_lib(depth2laser_naoqi_gui ALCOMMON ALPROXIES BOOST depth2laser_library OPENCV2_HIGHGUI)
qi_create_bin(pepper_head_control pepper_head_control.cpp)
qi_use_lib(pepper_head_control ALCOMMON ALPROXIES BOOST)
