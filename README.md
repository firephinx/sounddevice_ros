# sounddevice_ros

This ROS Catkin package is a wrapper around the sounddevice python package located here: https://python-sounddevice.readthedocs.io/en/0.3.13/

## Installation Instructions

1. Install sounddevice:
    ```bash
    pip install sounddevice --user
    ```
2. Install soundfile:
    ```bash
    pip install pysoundfile --user
    ```
3. Clone this repository into your catkin_ws/src folder:
    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/firephinx/sounddevice_ros.git
    ```
4. Run catkin_make in your catkin_ws folder:
    ```bash
    cd ..
    catkin_make
    ```

## Run Instructions

1. Source the devel/setup.bash and start the sound device publisher:
    ```bash
    source devel/setup.bash
    rosrun sounddevice_ros sounddevice_ros_publisher_node.py
    ```
2. Open a new terminal, source the devel/setup.bash, and start the sound device subscriber:
    ```bash
    source ~/catkin_ws/devel/setup.bash
    rosrun sounddevice_ros sounddevice_ros_subscriber_node.py
    ```

## Options

### Sounddevice ROS Publisher Node

1. You can list available devices by first running:
    ```bash
    rosrun sounddevice_ros sounddevice_ros_publisher_node.py -l
    ```
2. You can select your desired input device by using the -d flag:
    ```bash
    rosrun sounddevice_ros sounddevice_ros_publisher_node.py -d <device id or substring>
    ```
3. You can set the sample rate by using the -r flag:
    ```bash
    rosrun sounddevice_ros sounddevice_ros_publisher_node.py -r <sample_rate>
    ```
4. You can set the number of channels by using the -c flag (default is 1):
    ```bash
    rosrun sounddevice_ros sounddevice_ros_publisher_node.py -c <num_channels>
    ```
5. You can set whether to save the audio to a wav file using the -s flag (default is False):
    ```bash
    rosrun sounddevice_ros sounddevice_ros_publisher_node.py -s True
    ```
6. If you want to save the audio to a wave file, you can set the filename (default is a random temp filename):
    ```bash
    rosrun sounddevice_ros sounddevice_ros_publisher_node.py -s True [Filename]
    ```
7. If you want to set the subtype for soundfile saving, you can use the -t flag:
    ```bash
    rosrun sounddevice_ros sounddevice_ros_publisher_node.py -t PCM_24
    ```

### Sounddevice ROS Subscriber Node

1. You can set the filename that the audio will be saved to (default is a random temp filename):
    ```bash
    rosrun sounddevice_ros sounddevice_ros_subscriber_node.py [Filename]
    ```
2. If you did not already set the subtype for soundfile saving when starting the publisher node, you can use the -t flag:
    ```bash
    rosrun sounddevice_ros sounddevice_ros_publisher_node.py -t PCM_24
    ```