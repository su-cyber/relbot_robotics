cmake_minimum_required(VERSION 3.10)
project(xrf2test)

add_compile_options(-Wno-pointer-arith)

# SET THIS TO WHERE YOUR XRF2 FRAMEWORK IS:
set(locationFramework "${CMAKE_SOURCE_DIR}/XRF2/XenoRosFramework/")

# Xenomai and Common includes
set(locationXenoSrc "${locationFramework}Xenomai/src/")
set(locationXenoInclude "${locationFramework}Xenomai/include")
set(locationCommonInclude "${locationFramework}Common/include")

# Parse timestep from XML for 20-sim model (used by controller, not needed now)
set(XML_FILE_PATH "${CMAKE_SOURCE_DIR}/controller/LoopController_tokens.xml")
execute_process(
    COMMAND python3 ${locationFramework}Common/python_script/extracte_token_time_step.py ${XML_FILE_PATH}
    OUTPUT_VARIABLE TIME_STEP_SIZE_VALUE
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
message("TIME_STEP_SIZE_VALUE = " ${TIME_STEP_SIZE_VALUE})

# Dependencies
find_package(Threads REQUIRED)
find_package(xrf2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Get controller and source files
file(GLOB CONTROLLER_SRC "controller/common/*.cpp")
file(GLOB SRC "src/*.cpp")

# Xeno source from framework
set(XENO  
    ${locationXenoSrc}FullStateMachine.cpp
    ${locationXenoSrc}IcoIo.cpp
    ${locationXenoSrc}XenoFileHandler.cpp
    ${locationXenoSrc}XenoFrtLogger.cpp
    ${locationXenoSrc}XenoFrtLoop.cpp
    ${locationXenoSrc}XenoFrtMonitor.cpp
    ${locationXenoSrc}XenoFrtRosIco.cpp
    ${locationXenoSrc}XenoRosCommunication.cpp
    ${locationXenoSrc}XenoRosCommandBridge.cpp
    ${locationXenoSrc}XenoRosDataBridge.cpp
    ${locationXenoSrc}XenoSpiControllerHandler.cpp
    ${locationXenoSrc}XenoFrt20Sim.cpp
)

<<<<<<< HEAD
#Real-Time Executable
=======
<<<<<<< HEAD
# 🔧 Real-Time Executable
=======
#Real-Time Executable
>>>>>>> 470dd26 (separeting_packages)
>>>>>>> a5ff0ad (Packages_Separated)
add_executable(xrf2test 
    main.cpp
    ${XENO}
    ${SRC}
    controller/LoopController.cpp
    ${CONTROLLER_SRC}
)

# Define timestep as a macro
target_compile_definitions(xrf2test PRIVATE TIME_STEP_SIZE=${TIME_STEP_SIZE_VALUE})

# Link dependencies to FRT executable
ament_target_dependencies(xrf2test PUBLIC rclcpp xrf2_msgs std_msgs)
<<<<<<< HEAD

<<<<<<< HEAD
=======
# Install the ROS 2 test node
install(TARGETS
  xrf2test
  DESTINATION lib/${PROJECT_NAME}
)
=======
>>>>>>> 470dd26 (separeting_packages)

>>>>>>> a5ff0ad (Packages_Separated)
# Include paths for all headers
target_include_directories(xrf2test BEFORE PUBLIC 
    /usr/evl/include 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${locationXenoInclude}
    ${locationCommonInclude}
    controller/
    controller/common/
)

# Link Xenomai EVL user-space library
target_link_directories(xrf2test PUBLIC /usr/evl/lib)
target_link_libraries(xrf2test PRIVATE evl)
<<<<<<< HEAD
=======

# Install the real-time target
install(TARGETS
  xrf2test
  DESTINATION lib/${PROJECT_NAME}
)
>>>>>>> 470dd26 (separeting_packages)

# Install the real-time target
install(TARGETS
  xrf2test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
