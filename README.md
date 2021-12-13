> **Warning:** A work in progress. Please don't use it until the first version is release.

### Axrosbag

When bad things happen, it's usually too late to start recording with `rosbag record`.

`axrosbag` will quitely work in the background, recording topics only in memory.
When needed, you can make a request to save last-N-seconds of topics into a bag file.

### Daemon Command

参数 -a(--all) 表示记录所有 topic， 如果未加 -a 选项, 必须指定记录哪些 topic; 默认 buffer 时长为 300s，也支持参数 --limit 60, 即指定 buffer 时长 60s.

```bash
axrosbag daemon -a

axrosbag daemon /imu /odom

axrosbag daemon -a --limit 60
```

### Write Command

参数 -f 必须指定完整的保存路径和 bag 名; 默认以 lz4 格式压缩 bag， 也支持 bz2 格式.

```bash
axrosbag write -f /path/to/file.bag

axrosbag write -f /path/to/file.bag --bz2  # 建议采用默认的lz4格式，写文件速度更快
```
