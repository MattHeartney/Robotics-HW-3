execute_process(COMMAND "/home/matt/Documents/ros_sandbox/hw3/build/baxter_interface/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/matt/Documents/ros_sandbox/hw3/build/baxter_interface/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
