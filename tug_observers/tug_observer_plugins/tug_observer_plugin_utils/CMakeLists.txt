cmake_minimum_required(VERSION 2.8.3)
project(tug_observer_plugin_utils)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    find_package(catkin REQUIRED COMPONENTS
            roscpp
            rospy
            tug_yaml
            roslint
            tug_testing
            )
else()
    find_package(catkin REQUIRED COMPONENTS
            roscpp
            rospy
            tug_yaml
            )
endif()

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS thread math)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES observer_plugin_utils
  CATKIN_DEPENDS tug_yaml
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(observer_plugin_utils   src/hypothesis_check/singe_value_hypothesis_check/nominal_value/NominalValue.cpp
        src/hypothesis_check/singe_value_hypothesis_check/nominal_value/NominalValueFactory.cpp
        src/hypothesis_check/singe_value_hypothesis_check/nominal_value/GaussNominalValue.cpp
        src/filter/value_filter/ValueFilter.cpp
        src/filter/value_filter/ValueFilterFactory.cpp src/filter/value_filter/MeanValueFilter.cpp
        src/filter/value_filter/MedianValueFilter.cpp src/filter/value_filter/KMeansValueFilter.cpp
        src/filter/value_filter/EWMAValueFilter.cpp src/filter/value_filter/NoValueFilter.cpp
        src/hypothesis_check/singe_value_hypothesis_check/nominal_value/ExactNominalValue.cpp
        src/hypothesis_check/singe_value_hypothesis_check/nominal_value/NotNominalValue.cpp
        src/hypothesis_check/singe_value_hypothesis_check/nominal_value/GreaterThanNominalValue.cpp
        src/hypothesis_check/singe_value_hypothesis_check/nominal_value/LessThanNominalValue.cpp
        src/hypothesis_check/singe_value_hypothesis_check/nominal_value/InBetweenNominalValue.cpp
        src/hypothesis_check/singe_value_hypothesis_check/nominal_value/NotInBetweenNominalValue.cpp
        src/filter/deviation_filter/DeviationFilter.cpp src/filter/Filter.cpp src/filter/FilteState.cpp
        src/filter/deviation_filter/MinMaxDeviationFilter.cpp
        src/filter/deviation_filter/DeviationFilterFactory.cpp
        src/filter/deviation_filter/StandartDeviationFilter.cpp
        src/filter/value_filter/ewma_value_filter/EWMAValueFilterWithBuffer.cpp
        src/filter/value_filter/ewma_value_filter/EWMAValueFilterWithoutBuffer.cpp
        src/filter/value_filter/mean_value_filter/MeanValueFilterWithBuffer.cpp
        src/filter/value_filter/mean_value_filter/MeanValueFilterWithoutBuffer.cpp
        src/filter/deviation_filter/standart_deviation_filter/StandartDeviationFilterWithBuffer.cpp
        src/filter/deviation_filter/standart_deviation_filter/StandartDeviationFilterWithoutBuffer.cpp
        src/filter/deviation_filter/min_max_deviation_filter/MinMaxDeviationFilterWithBuffer.cpp
        src/filter/deviation_filter/min_max_deviation_filter/MinMaxDeviationFilterWithoutBuffer.cpp
        src/hypothesis_check/singe_value_hypothesis_check/SingleValueHypothesisCheck.cpp
        src/hypothesis_check/singe_value_hypothesis_check/NominalValueHypothesis.cpp
        src/hypothesis_check/singe_value_hypothesis_check/SingleValueHypothesisCheckFactory.cpp
        src/hypothesis_check/singe_value_hypothesis_check/StudentsTSingleHypothesis.cpp
        src/interpolation/Interpolation.cpp src/interpolation/LinearInterpolation.cpp
        src/differentiation/Differentiation.cpp src/differentiation/SingleSideDifferentiation.cpp
        src/differentiation/TwoSideDifferentiation.cpp src/differentiation/SimpleLinearRegression.cpp
        src/integration/Integration.cpp src/integration/SimpleSummBasedIntegration.cpp
        )

## Declare a cpp executable
# add_executable(tug_observer_plugins_cpp_node src/tug_observer_plugins_cpp_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(tug_observer_plugins_cpp_node tug_observer_plugins_cpp_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(observer_plugin_utils
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS tug_observer_plugins_cpp tug_observer_plugins_cpp_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tug_observer_plugins_cpp.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

if(CATKIN_ENABLE_TESTING)
    roslint_cpp()
    roslint_add_test()

    catkin_add_gtest(test_linear_interpolatioin test/TestLinearInterpolation.cpp)
    target_link_libraries(test_linear_interpolatioin ${catkin_LIBRARIES} observer_plugin_utils)

    catkin_add_gtest(test_single_side_differentation test/TestSingleSideDifferentiation.cpp)
    target_link_libraries(test_single_side_differentation ${catkin_LIBRARIES} observer_plugin_utils)

    catkin_add_gtest(test_two_side_differentation test/TestTwoSideDifferentiation.cpp)
    target_link_libraries(test_two_side_differentation ${catkin_LIBRARIES} observer_plugin_utils)

    catkin_add_gtest(test_simple_linear_regression test/TestSimpleLinearRegression.cpp)
    target_link_libraries(test_simple_linear_regression ${catkin_LIBRARIES} observer_plugin_utils)

    catkin_add_gtest(test_no_value_filter test/TestNoValueFilter.cpp)
    target_link_libraries(test_no_value_filter ${catkin_LIBRARIES} observer_plugin_utils)

    add_rostest_gtest(test_mean_filter test/launch/test_mean_filter.test test/TestMeanFilter.cpp)
    target_link_libraries(test_mean_filter ${catkin_LIBRARIES} observer_plugin_utils)

    add_rostest_gtest(test_median_filter test/launch/test_median_filter.test test/TestMedianFilter.cpp)
    target_link_libraries(test_median_filter ${catkin_LIBRARIES} observer_plugin_utils)

    add_rostest_gtest(test_ewma_filter test/launch/test_ewma_filter.test test/TestEWMAFilter.cpp)
    target_link_libraries(test_ewma_filter ${catkin_LIBRARIES} observer_plugin_utils)

    add_rostest_gtest(test_kmeans_filter test/launch/test_kmeans_filter.test test/TestKMeansFilter.cpp)
    target_link_libraries(test_kmeans_filter ${catkin_LIBRARIES} observer_plugin_utils)
endif()
