# Axrosbag <!-- omit in toc -->

When bad things happen, it's often too late to start recording with `rosbag record`.

`axrosbag` serve as a [accident data recorder](https://en.wikipedia.org/wiki/Accident_data_recorder).
It will rotatedly record last-N seconds of messages only in memory.
Only when requested, it will dump saved messages into a [ROS bag file](http://wiki.ros.org/Bags).

- [Command Line Interface](#command-line-interface)
  - [1. Daemon Mode](#1-daemon-mode)
  - [2. Write to File](#2-write-to-file)
  - [3. Pause & Resume](#3-pause--resume)
- [ROS Services](#ros-services)

## Command Line Interface

### 1. Daemon Mode

Record topics in memory.

Sample commands:

```bash
# record all topics with default buffer length
axrosbag daemon -a
# record selected topics
axrosbag daemon /imu /odom #
# record all topics with a 60-seconds buffer
axrosbag daemon -a --limit 60

# show help
axrosbag daemon --help
```

### 2. Write to File

Sample command:

```bash
# write recorded messages into a file.bag
axrosbag write -f file.bag
# show help
axrosbag daemon --help
```

### 3. Pause & Resume

```bash
# pause recording of some topics
axrosbag pause /imu /odom

# resume recording of all topics
axrosbag resume --all
```

## ROS Services

- `/axrosbag/write` - Write ROS bag file.
- `/axrosbag/pause` - Pause or resume recoding of some/all topics.
