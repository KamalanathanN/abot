# Behavior Tree Setup:

## Behavior Tree 🌳:
A Behavior Tree (BT) is a way to structure the switching between different tasks in an autonomous agent, such as a robot or a virtual entity in a computer game.

Checkout offical Documentation for more: [BehaviorTree.CPP](https://www.behaviortree.dev/)

## Groot 🌱:
Editing a BehaviorTree is as simple as editing a XML file in your favourite text editor or just [Groot](https://github.com/BehaviorTree/Groot) it.🙂

## Install BT:
```bash
    sudo apt install ros-noetic-behaviortree-cpp-v3
```
## Get Groot:
Install dependencies for groot:
```bash
    sudo apt install qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev
```
Run following inside ```your_ros_ws/src/```
```bash
    git clone https://github.com/BehaviorTree/Groot.git
    cd ..
    rosdep install --from-paths src --ignore-src
    catkin_make 
```
Now you can run Groot with:
```bash
    source your_ros_ws/devel/setup.bash
    rosrun groot Groot
```
## Groot <-- ? --> ROS
Groot and ROS are two separate applications, in order for them to communicate there must be some intermediate program. To do this task we use a plugin called “[ZeroMQ](https://zeromq.org/get-started/)”.

```bash
    # dependency packages

    sudo apt install libtool pkg-config build-essential autoconf automake

    # Libsodium is a crypto library used by ZeroMQ
    
    wget https://download.libsodium.org/libsodium/releases/libsodium-1.0.18-stable.tar.gz
    tar -xvf libsodium-1.0.18-stable.tar.gz
    cd libsodium-stable/
    ./autogen.sh
    ./configure && make check
    sudo make install
    sudo ldconfig

    # Install ZeroMQ

    wget https://github.com/zeromq/libzmq/releases/download/v4.3.4/zeromq-4.3.4.tar.gz
    tar -xvf zeromq-4.3.4.tar.gz
    cd zeromq-4.3.4/
    ./autogen.sh
    ./configure && make check
    sudo make install
    sudo ldconfig
```
## Groot <--ZeroMQ--> ROS
Groot has an inbuild ZeroMQ client so we only need to create a ZeroMQ publisher inside our ROS package to publish information about the behavior tree at runtime.

Just create an instance of the ZeroMQ publisher in the class where we initiate our behavior tree and pass our behavior tree to that class as an argument.

### Minimal template:
```cpp
    /* Include this header file to get the ZMQ publisher*/ 
    #include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

    using namespace BT;

    /* main class where you will initialize the behavior tree */
    int main(int argc, char **argv)
    {
    
    /* Initialize your tree */
    /* Pass the initialized tree to the ZMQ publisher */
    /* By adding following line you will be able to
        create a link between Groot and ROS */
    
    PublisherZMQ publisher_zmq(tree);
    
    return 0;
    }
```


