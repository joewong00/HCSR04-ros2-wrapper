cmake_minimum_required(VERSION 3.8)
project(libHCSR04)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

add_library(libHCSR04 src/libHCSR04.cpp)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_library(WIRINGPI_LIBS NAMES wiringPi HINTS /usr/local/lib)
ament_export_targets(libHCSR04 HAS_LIBRARY_TARGET)
target_link_libraries(libHCSR04 ${WIRINGPI_LIBS})

install(
  DIRECTORY include/libHCSR04
  DESTINATION include 
)

install(
  TARGETS libHCSR04
  EXPORT libHCSR04
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
