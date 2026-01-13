cmake_minimum_required(VERSION 3.8)
project(sensor_data_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

#Adding Global Files Variables

# Automatically find and install all Python and shell files in the dls directory
file(GLOB PYTHON_FILES "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py")

if(PYTHON_FILES)
  install(PROGRAMS
    ${PYTHON_FILES}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install launch directory if it exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)

  find_package(ament_cmake_pytest REQUIRED)
  #find_package(ament_lint_auto REQUIRED)

  # Automatically discover and add pytest tests
  file(GLOB TEST_FILES "${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.py")
  foreach(TEST_FILE ${TEST_FILES})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
    ament_add_pytest_test(
      ${TEST_NAME}
      ${TEST_FILE}
      PYTHON_EXECUTABLE ${PYTHON_EXECUTABLE}
      TIMEOUT 60
    )
  endforeach()
  # Skip the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()

ament_package()
