add_executable(simple_2d_example ./simple_2d_example.cpp)
target_link_libraries(simple_2d_example osqp-cpp)
# Include their headers
target_include_directories(simple_2d_example PRIVATE
      ${CMAKE_SOURCE_DIR}/lib/osqp-cpp
      )

