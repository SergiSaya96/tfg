# locate the necessary dependencies
# add the necessary include directories
FIND_PACKAGE(iriutils)
FIND_PACKAGE(comm)
FIND_PACKAGE(dynamixel)
FIND_PACKAGE(dynamixel_motor_cont)

INCLUDE_DIRECTORIES(.)
# application source files
IF(iriutils_FOUND)
  INCLUDE_DIRECTORIES(${iriutils_INCLUDE_DIR})
ENDIF(iriutils_FOUND)
IF(comm_FOUND)
  INCLUDE_DIRECTORIES(${comm_INCLUDE_DIR})
ENDIF(comm_FOUND)
IF(dynamixel_FOUND)
  INCLUDE_DIRECTORIES(${dynamixel_INCLUDE_DIR})
ENDIF(dynamixel_FOUND)
IF(dynamixel_motor_cont_FOUND)
  INCLUDE_DIRECTORIES(${dynamixel_motor_cont_INCLUDE_DIR})
ENDIF(dynamixel_motor_cont_FOUND)
SET(sources test_out.cpp)
# application header files
SET(headers test_out.h)
# create the executable file
ADD_EXECUTABLE(test_out ${sources})
# link necessary libraries
IF(iriutils_FOUND)
  TARGET_LINK_LIBRARIES(test_out ${iriutils_LIBRARY})
ENDIF(iriutils_FOUND)
IF(comm_FOUND)
  TARGET_LINK_LIBRARIES(test_out ${comm_LIBRARY}) 
ENDIF(comm_FOUND)
IF(dynamixel_FOUND)
  TARGET_LINK_LIBRARIES(test_out ${dynamixel_LIBRARY})
ENDIF(dynamixel_FOUND)
IF(dynamixel_motor_cont_FOUND)
  TARGET_LINK_LIBRARIES(test_out ${dynamixel_motor_cont_LIBRARY})
ENDIF(dynamixel_motor_cont_FOUND)


