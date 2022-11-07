cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

set(CMSISDSP CMSISDSP)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/src/Source)

target_link_libraries(${PROJECT_NAME} PRIVATE CMSISDSP)
