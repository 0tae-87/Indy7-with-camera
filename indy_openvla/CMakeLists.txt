cmake_minimum_required(VERSION 3.8)
project(indy_openvla)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED) # Python launch 파일 설치를 위해 필요

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 🌟 수정: YAML 컨트롤러 파일을 명시적으로 설치 (파일 로드 오류 해결) 🌟
# Error opening YAML file 오류는 이 파일이 install 폴더에 없어서 발생했습니다.
install(FILES
  config/indy_openvla_controllers.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# URDF (XACRO) 파일이 설치되도록 합니다.
# (config 파일을 FILES로 명시했으므로, urdf는 별도로 설치합니다.)
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()