# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project( OpenCV_install_test )

# 设置编译模式
set( CMAKE_BUILD_TYPE "Debug" )

#添加OPENCV库
#指定OpenCV版本，代码如下
#find_package(OpenCV 4.4 REQUIRED)
#如果不需要指定OpenCV版本，代码如下
find_package(OpenCV REQUIRED)

#添加OpenCV头文件
include_directories(${OpenCV_INCLUDE_DIRS})

#显示OpenCV_INCLUDE_DIRS的值
message(${OpenCV_INCLUDE_DIRS})

# 添加一个可执行程序
# 语法：add_executable( 程序名 源代码文件 ）
add_executable( opencv_test test_2.cpp )

# 将库文件链接到可执行程序上
target_link_libraries( opencv_test  ${OpenCV_LIBS})