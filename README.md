> **Warning:** A work in progress. Please don't use it until the first version is release.

### Axrosbag

When bad things happen, it's usually too late to start recording with `rosbag record`.

`axrosbag` will quitely work in the background, recording topics only in memory.
When needed, you can make a request to save last-N-seconds of topics into a bag file.

```bash
catkin_make install

source install/setup.bash
```

### Daemon Command

参数 -a(--all) 表示记录所有 topic， 如果未加 -a 选项, 必须指定记录哪些 topic; 默认 buffer 时长为 300s，也支持参数 --limit 60, 即指定 buffer 时长 60s.

```bash
rosrun axrosbag_ros axrosbag daemon -a

rosrun axrosbag_ros axrosbag daemon /imu /odom

rosrun axrosbag_ros axrosbag daemon -a --limit 60
```

or

```bash
axrosbag daemon -a
```

### Write Command

参数 -f 必须指定完整的保存路径和 bag 名; 默认以 lz4 格式压缩 bag， 也支持 bz2 格式.

```bash
rosrun axrosbag_ros axrosbag write -f /path/to/file.bag # 保存到指定路径

rosrun axrosbag_ros axrosbag write -f file.bag # 保存到当前所在路径

rosrun axrosbag_ros axrosbag write -f /path/to/file.bag --bz2  # 建议采用默认的lz4格式，写文件速度更快
```
