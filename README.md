### Axrosbag

The axrosbag node may run in the background, and you can save buffer data from the recent past to disk via command line or ros service.

### Run

在launch文件中设置buffer duration，默认记录所有topics

```bash
roslaunch axrosbag record.launch
```

### Command line interface

1. 保存以当前的日期和时间命名的bag

```bash
rosrun axrosbag axrosbag -t
```

2. 保存指定名称的bag到指定的目录下

```bash
rosrun axrosbag axrosbag -t -O /path/to/file.bag
```

### Call ros service

该服务包含如下几个参数：

filename - 设置bag包的名称和保存的目录

topics - 若为空，则记录所有的topic

start_time - 设置记录的起始ros time

stop_time - 设置记录的终止ros time

```bash
$ rosservice call /trigger_record "filename: '/path/to/file.bag'
topics:
- ''
start_time:
  secs: 0
  nsecs: 0
stop_time:
  secs: 0
  nsecs: 0"
```
