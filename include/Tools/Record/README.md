# Record（头文件定义）

比赛录制接口。

## record.h

- `DatasetRecorder`
  - `start(record_dir, max_fps)`：建目录、启动异步写线程
  - `try_push(frame)`：按 `max_fps` 限流入队；队列满丢最旧帧
  - `stop()`：停线程并 join
  - `enabled()`：是否已开启录制
