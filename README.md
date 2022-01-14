# Axrosbag <!-- omit in toc -->

When bad things happen, it's often too late to start recording with `rosbag record`.

`axrosbag` acts as an [accident data recorder](https://en.wikipedia.org/wiki/Accident_data_recorder).
It will rotatedly record last-N seconds of messages only in memory.
Only when requested, it shall dump saved messages into a [ROS bag file](http://wiki.ros.org/Bags).

- [Command Line Interface](#command-line-interface)
  - [Daemon Mode](#daemon-mode)
  - [Write to File](#write-to-file)
  - [Pause & Resume](#pause--resume)
- [ROS Services](#ros-services)

## Command Line Interface

### Daemon Mode

```bash
# record all topics with default buffer size
axrosbag daemon -a
# record selected topics
axrosbag daemon /imu /odom
# record all topics with a 60-seconds buffer
axrosbag daemon -a --limit 60

# show help
axrosbag daemon --help
```

### Write to File

```bash
# write recorded messages into file.bag
axrosbag write -f file.bag
# show help
axrosbag daemon --help
```

### Pause & Resume

```bash
# pause recording of some topics
axrosbag pause /imu /odom

# resume recording of all topics
axrosbag resume --all
```

## ROS Services

- `/axrosbag/write` - Write ROS bag file.
- `/axrosbag/pause` - Pause or resume recoding of some/all topics.
