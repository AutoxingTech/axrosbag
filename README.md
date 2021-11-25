### Axrosbag

The axrosbag node may run in the background, and you can save buffer data from the recent past to disk via command line or ros service.

### Run

本程序默认记录所有 topic，默认以 lz4 格式压缩 bag，默认 buffer 时长为 300s;同时，也支持 --topic, --lz4 (bz2), -d 指定参数

```bash
rosrun axrosbag axrosbag
```
or
```bash
./devel/lib/axrosbag/axrosbag
```

### Command line interface

1. 保存以当前的日期和时间命名的 bag, 默认保存在当前目录

```bash
rosrun axrosbag axrosbag -t
```

2. 保存指定名称的 bag 到指定的目录下

```bash
rosrun axrosbag axrosbag -t -O /path/to/file.bag
```
