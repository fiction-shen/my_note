# Ros学习——C++发布器publisher和订阅器subscriber

## 1.编写发布器

**初始化 ROS 系统**

**在 ROS 网络内广播我们将要在 chatter 话题上发布 std_msgs/String 类型的消息**

**以每秒 10 次的频率在 chatter 上发布消息**

在 beginner_tutorials package 里创建 src/talker.cpp 文件

```
#include "ros/ros.h"　　　　　　//一个实用的头文件，它引用了ROS 系统中大部分常用的头文件
#include "std_msgs/String.h"  //引用了 std_msgs/String 消息, 它存放在 std_msgs package 里，是由 String.msg 文件自动生成的头文件。

#include <sstream>

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");　　//初始化 ROS.可指定节点的名称。节点的名称必须唯一(名称内不能包含 / 等符号)

  
  ros::NodeHandle n;　　//为这个进程的节点创建一个句柄。
　　　　　　　　　　　　　　//第一个创建的 NodeHandle 会为节点进行初始化，最后一个销毁的 NodeHandle 则会释放该节点所占用的所有资源
  
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);//告诉master 我们将要在 chatter（话题名） 上发布 std_msgs/String 消息类型的消息。
　　　　　　　　　　　　　//如果我们发布的消息的频率太高，缓冲区中的消息在大于 1000 个的时候就会开始丢弃先前发布的消息。　　　　　　　　　　　　　//advertise返回一个 ros::Publisher 对象,它有两个作用： 1) 它有一个 publish() 成员函数可以让你在topic上发布消息； 2) 如果消息类型不对,它会拒绝发布。

  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())　　　　　　//ros::ok()
  {

    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());//ROS_INFO 和其他类似的函数可以用来代替 printf/cout 等函数。

    chatter_pub.publish(msg);//向所有订阅 chatter 话题的节点发送消息。

    ros::spinOnce();　　　　　　//ros::spinOnce()这一语句，否则你的回调函数就永远也不会被调用

    loop_rate.sleep();　　　　//调用 ros::Rate 对象来休眠一段时间以使得发布频率为 10Hz。
    ++count;
  }


  return 0;
}
```

## 2.编写订阅器



**初始化ROS系统**

**订阅 chatter 话题**

**进入自循环，等待消息的到达**

**当消息到达，调用 chatterCallback() 函数**

在 beginner_tutorials package 目录下创建 src/listener.cpp 文件：

```
#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)//是一个回调函数，当接收到 chatter 话题的时候就会被调用。
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
//告诉 master 我们要订阅 chatter 话题上的消息。当有消息发布到这个话题时，ROS 就会调用 chatterCallback() 函数。第二个参数是队列大小，以防我们处理消息的速度不够快，当缓存达到 1000 条消息后，再有新的消息到来就将开始丢弃先前接收的消息。

 ros::spin();//ros::spin() 进入自循环，可以尽可能快的调用消息回调函数。return 0; }
```

## 3.编译

### 1.确认CMakeLists.txt文件

```
cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES Num.msg)
add_service_files(DIRECTORY srv FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
```

### 2.在 CMakeLists.txt 文件末尾加入几条语句:

```
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)
```

1-2会生成两个可执行文件, talker 和 listener, 默认存储到 devel space 目录下,具体在~/catkin_ws/devel/lib/<package name> 中.

### 3.如果在 *Groovy* 版本下，你可以使用下边的这个变量来添加对所有必须的文件依赖:

```
add_dependencies(talker ${catkin_EXPORTED_TARGETS})
```