### Axrosbag

When bad things happen, it's often too late to start recording with `rosbag record`.

`axrosbag` will record last-N seconds(rotated) of messages only in memory.
Only when needed, you can make a request to dump saved messages into a bag file.

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
