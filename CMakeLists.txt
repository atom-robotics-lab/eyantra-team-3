cmake_minimum_required(VERSION 3.8)
project(eyantra_hb_task_6)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# Adding executable for hello_publisher
# add_executable(${CMAKE_PROJECT_NAME}_hello_publisher
#   src/hello_publisher_node.cpp
# )

# target_include_directories(${CMAKE_PROJECT_NAME}_hello_publisher PRIVATE
#   include/${CMAKE_PROJECT_NAME}
# )

# ament_target_dependencies(${CMAKE_PROJECT_NAME}_hello_publisher
#   rclcpp
#   std_msgs
# )

# # Adding executable for hello_subscriber
# add_executable(${CMAKE_PROJECT_NAME}_hello_subscriber
#   src/hello_subscriber_node.cpp
# )

# target_include_directories(${CMAKE_PROJECT_NAME}_hello_subscriber PRIVATE
#   include/${CMAKE_PROJECT_NAME}  
# )

# ament_target_dependencies(${CMAKE_PROJECT_NAME}_hello_subscriber
#   rclcpp
#   std_msgs
# )

# # Install executables
# install(TARGETS
#   ${CMAKE_PROJECT_NAME}_hello_publisher
#   ${CMAKE_PROJECT_NAME}_hello_subscriber
#   DESTINATION lib/${PROJECT_NAME}
# )

# # Install the header files
# install(DIRECTORY include/
#   DESTINATION include/
# )

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

install(
    DIRECTORY
    launch
    scripts
  #   worlds
  # env-hooks
    DESTINATION
    share/${PROJECT_NAME}/
)

# Install Python executables
install(PROGRAMS
  scripts/bot_1_6a_controller.py
  scripts/bot_2_6a_controller.py
  scripts/bot_3_6a_controller.py
  scripts/feedback.py
  scripts/bots_reached.py
  scripts/stop_service.py
  scripts/stop_client.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
