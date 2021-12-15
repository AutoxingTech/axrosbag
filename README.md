### Axrosbag

When bad things happen, it's often too late to start recording with `rosbag record`.

`axrosbag` will quitely work in the background, recording everthing only in memory.
When needed, you can make a request to save last-N-seconds of topics into a bag file.

### Daemon Mode

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

### Write to File

Sample command:

```bash
# write recorded messages into a file.bag
axrosbag write -f file.bag
# show help
axrosbag daemon --help
```
