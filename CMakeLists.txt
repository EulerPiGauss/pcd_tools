cmake_minimum_required(VERSION 2.8)
project(pcd_tools)
set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.8)
INCLUDE_DIRECTORIES(include/
        ${PCL_INCLUDE_DIRS})

list(APPEND TOOL_UTILS_SRC src/tool_utils.cpp src/io_utils.cpp src/visualize_utils.cpp)
list(APPEND TOOL_UTILS_INC include/tool_utils.h include/io_utils.h include/visualize_utilis.h)


add_executable(txt2pcd src/txt2pcd.cpp ${TOOL_UTILS_SRC}  ${TOOL_UTILS_INC})
target_link_libraries(txt2pcd ${PCL_LIBRARIES})

add_executable(tool_cropper src/tool_cropper.cpp ${TOOL_UTILS_SRC}  ${TOOL_UTILS_INC})
target_link_libraries(tool_cropper ${PCL_LIBRARIES})

add_executable(tool_remove_bg src/tool_remove_bg.cpp ${TOOL_UTILS_SRC}  ${TOOL_UTILS_INC})
target_link_libraries(tool_remove_bg ${PCL_LIBRARIES})

add_executable(preprocessing src/preprocessing_test.cpp ${TOOL_UTILS_SRC}  ${TOOL_UTILS_INC})
target_link_libraries(preprocessing ${PCL_LIBRARIES})