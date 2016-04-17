cmake_minimum_required(VERSION 2.8.3)

foreach(library ${USER_LIBRARIES})
    add_subdirectory(${library})
    set(${library}_INCLUDE_DIRS
      ${${library}_INCLUDE_DIRS}
      PARENT_SCOPE
      )
endforeach()