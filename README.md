### Axrosbag

When bad things happen, it's often too late to start recording with `rosbag record`.

`axrosbag` will quitely work in the background, recording everthing only in memory.
When needed, you can make a request to save last-N-seconds of topics into a bag file.

### Daemon Mode

Record topics in memory.
参数 -a(--all) 表示记录所有 topic， 如果未加 -a 选项, 必须指定记录哪些 topic; 默认 buffer 时长为 300s，也支持参数 --limit 60, 即指定 buffer 时长 60s.

Sample commands:

```bash
# record all topics with default buffer length
axrosbag daemon -a
# record selected topics
axrosbag daemon /imu /odom # 
# record all topics with a 60-seconds buffer
axrosbag daemon -a --limit 60
```

### Write to File

Sample command:

```bash
axrosbag write -f file.bag
```
