cmake_minimum_required(VERSION 2.8.3)

foreach(library ${USER_LIBRARIES})
    add_subdirectory(${library})
endforeach()