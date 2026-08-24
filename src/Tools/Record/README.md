# Record（实现功能）

## record.cpp

- 异步队列写 YOLO 输入 BGR 为 `record_<时间戳>.mp4`
- 多 fourcc 尝试打开 `VideoWriter`
- 限帧推送、队列满丢帧统计；无有效帧时删除空文件
