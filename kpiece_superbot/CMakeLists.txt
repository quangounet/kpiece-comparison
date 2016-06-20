cmake_minimum_required (VERSION 2.6.0)
project (cppexamples)

set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../..) # only used for cppexamples!

find_package(OpenRAVE REQUIRED)
find_package(OMPL REQUIRED)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT COMPONENTS iostreams python thread system)

include_directories(${OpenRAVE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} )
if( Boost_INCLUDE_DIRS )
  include_directories(${Boost_INCLUDE_DIRS})
endif()

link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

macro(build_openrave_executable name)
  add_executable(${name} ${name}.cpp)
  set_target_properties(${name} PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS} -I/usr/local/include")
  set_target_properties(${name} PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS} -L/usr/local/lib")
  target_link_libraries(${name} ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ompl ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY})
  install(TARGETS ${name} DESTINATION . )
endmacro(build_openrave_executable)

macro(build_openrave_plugin name)
  add_library(${name} SHARED ${name}.cpp)
  set_target_properties(${name} PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}")
  set_target_properties(${name} PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
  target_link_libraries(${name} ${OpenRAVE_LIBRARIES})
  install(TARGETS ${name} DESTINATION . )
endmacro(build_openrave_plugin)

build_openrave_executable(SuperBotBenchmark_AllDOFs)
