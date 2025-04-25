cmake_minimum_required(VERSION 3.8)
project(my_package)

# enable pedantic warnings on GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find core ament build system
find_package(ament_cmake REQUIRED)

# optionally uncomment to lint during tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# install your Python demo node
install(PROGRAMS
  psuedo_rplidar_node.py
  world_maker_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# install any config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/config
)

# install URDFs
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/urdf
)

# install meshes
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/worlds
)

# ament meta-target
ament_package()
