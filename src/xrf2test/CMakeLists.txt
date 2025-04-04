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
list(REMOVE_ITEM SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/test_node.cpp")  # <- prevent double main()

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

# 🔧 Real-Time Executable
add_executable(template_20sim 
    main.cpp
    ${XENO}
    ${SRC}
    controller/LoopController.cpp
    ${CONTROLLER_SRC}
)

# 🧪 ROS 2 Test Publisher
add_executable(test_node src/test_node.cpp)
ament_target_dependencies(test_node rclcpp xrf2_msgs)

# Define timestep as a macro
target_compile_definitions(template_20sim PRIVATE TIME_STEP_SIZE=${TIME_STEP_SIZE_VALUE})

# Link dependencies to FRT executable
ament_target_dependencies(template_20sim PUBLIC rclcpp xrf2_msgs std_msgs)

# Install the ROS 2 test node
install(TARGETS
  test_node
  DESTINATION lib/${PROJECT_NAME}
)

# Include paths for all headers
target_include_directories(template_20sim BEFORE PUBLIC 
    /usr/evl/include 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${locationXenoInclude}
    ${locationCommonInclude}
    controller/
    controller/common/
)

# Link Xenomai EVL user-space library
target_link_directories(template_20sim PUBLIC /usr/evl/lib)
target_link_libraries(template_20sim PRIVATE evl)

ament_package()
