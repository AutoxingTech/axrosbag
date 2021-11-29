### Axrosbag

The axrosbag node(daemon command) may run in the background, and you can save buffer data from the recent past to disk via write command.

### Daemon Command

参数 -a(--all) 表示记录所有 topic， 如果未加 -a 选项, 必须指定记录哪些topic; 默认 buffer 时长为 300s，也支持参数 --limit 60, 即指定buffer时长60s.

```bash
rosrun axrosbag axrosbag daemon -a

rosrun axrosbag axrosbag daemon /imu /odom

rosrun axrosbag axrosbag daemon -a --limit 60
```
or
```bash
./devel/lib/axrosbag/axrosbag daemon -a
```

### Write Command

参数 -f 必须指定完整的保存路径和bag名; 默认以 lz4 格式压缩 bag， 也支持 bz2 格式.

```bash
rosrun axrosbag axrosbag write -f /path/to/file.bag

rosrun axrosbag axrosbag write -f /path/to/file.bag --bz2  # 建议采用默认的lz4格式，写文件速度更快
```
