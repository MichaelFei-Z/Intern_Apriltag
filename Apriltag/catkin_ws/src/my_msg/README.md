<!-- 创建功能包 -->
catkin_create_pkg my_msg roscpp rospy std_msgs geometry_msgs message_generation

<!-- 创建自定义消息文件 -->
cd my_msg && mkdir msg
gedit detection.msg   填入自定义消息

<!-- 修改CMakeLists.txt -->
# 从 msg 文件夹中生成消息
add_message_files( FILES detection.msg)
# 添加生成消息的依赖
generate_messages( DEPENDENCIES geometry_msgs std_msgs)
# 添加编译需要的依赖
catkin_package( CATKIN_DEPENDS geometry_msgs message_generation roscpp rospy std_msgs)
# 添加编译需要的头文件
include_directories( ${catkin_INCLUDE_DIRS})
