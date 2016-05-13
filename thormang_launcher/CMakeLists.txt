cmake_minimum_required(VERSION 2.8.3)
project(thormang_launcher)


find_package(catkin REQUIRED roslaunch)
roslaunch_add_file_check(launch)
