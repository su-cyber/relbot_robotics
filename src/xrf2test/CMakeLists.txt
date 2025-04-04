cmake_minimum_required(VERSION 3.10)
project(template_20sim)

# Optional: Uncomment to force C++11
#set(CMAKE_CXX_STANDARD 11)

add_compile_options(-Wno-pointer-arith)

# SET THIS TO THE ACTUAL PATH WHERE YOUR XRF2 FOLDER IS:
set(locationFramework "${CMAKE_SOURCE_DIR}/XRF2/XenoRosFramework/")

# Xenomai and Common includes
set(locationXenoSrc "${locationFramework}Xenomai/src/")
set(locationXenoInclude "${locationFramework}Xenomai/include")
set(locationCommonInclude "${locationFramework}Common/include")

# XML Token parsing for 20sim controller
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

# Source Files
file(GLOB CONTROLLER_SRC "controller/common/*.cpp")
file(GLOB SRC "src/*.cpp")

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

# Main Real-Time FRT executable
add_executable(template_20sim 
    main.cpp 
    ${XENO}
    ${SRC}
    controller/LoopController.cpp
    ${CONTROLLER_SRC} 
)

# ROS2 test publisher executable
add_executable(test_node src/test_node.cpp)
ament_target_dependencies(test_node rclcpp xrf2_msgs)

# Apply TIME_STEP_SIZE from parsed XML
target_compile_definitions(template_20sim PRIVATE TIME_STEP_SIZE=${TIME_STEP_SIZE_VALUE})

ament_target_dependencies(template_20sim PUBLIC rclcpp xrf2_msgs std_msgs)

# Install only the test_node (no need to install template_20sim for now)
install(TARGETS
  test_node
  DESTINATION lib/${PROJECT_NAME}
)

# Headers for everything (20sim, Xeno, ROS bridge, etc.)
target_include_directories(template_20sim BEFORE PUBLIC 
    /usr/evl/include 
    include/
    ${locationXenoInclude}
    ${locationCommonInclude}
    controller/
    controller/common/
)

# Link against the Xenomai EVL user-space lib
target_link_directories(template_20sim PUBLIC /usr/evl/lib)
target_link_libraries(template_20sim PRIVATE evl)
