cmake_minimum_required(VERSION 2.8.3)
project(neato_node)

find_package(catkin REQUIRED message_generation std_msgs)

add_message_files(
   FILES
   Button.msg
   Sensor.msg
   Movement.msg
   Encoder.msg
 )

add_service_files(
   FILES
	 SetLed.srv
	 PlaySound.srv
 )
 
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(DEPENDS message_runtime )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY nodes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
