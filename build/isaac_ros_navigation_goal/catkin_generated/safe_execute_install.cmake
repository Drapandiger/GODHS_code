execute_process(COMMAND "/home/liding/ws_thesis_lzq/build/isaac_ros_navigation_goal/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/liding/ws_thesis_lzq/build/isaac_ros_navigation_goal/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
