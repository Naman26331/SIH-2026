cmake_minimum_required(VERSION 3.8)
project(fleet_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds maps models scripts
        DESTINATION share/${PROJECT_NAME})

ament_package()
