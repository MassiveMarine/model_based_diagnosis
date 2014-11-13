cmake_minimum_required(VERSION 2.8.3)
project(tug_ist_diagnosis_msgs)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs)


add_message_files(
  FILES
  BoardActionFeedback.msg  BoardActionResult.msg  BoardResult.msg        Diagnosis.msg         SystemDescription.msg          SystemModelAction.msg        SystemModelGoal.msg
  BoardActionGoal.msg      BoardFeedback.msg      Channel.msg            DiagnosisResults.msg  SystemModelActionFeedback.msg  SystemModelActionResult.msg  SystemModelResult.msg
  BoardAction.msg          BoardGoal.msg          DBoardMeasurments.msg  Observations.msg      SystemModelActionGoal.msg      SystemModelFeedback.msg
)

add_action_files(
  FILES
  DiagnosisRepair.action
)

generate_messages(
    DEPENDENCIES actionlib actionlib_msgs
)

# catkin_package parameters:
catkin_package(
    CATKIN_DEPENDS actionlib actionlib_msgs
)