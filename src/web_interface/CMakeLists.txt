cmake_minimum_required(VERSION 3.8)
project(web_interface)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY web/ DESTINATION share/${PROJECT_NAME}/web)

ament_package()
