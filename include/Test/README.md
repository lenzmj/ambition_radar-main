# Test

test 模式回放：视频 / 照片喂帧，供主循环做与实机相同的检测与解算。

## 子文件

### playback.h / playback.cpp

- `TestPlaybackState`：总帧、当前帧、seek、暂停
- `TestPlayer`
  - 照片循环 / 视频回放（gain、loop、进度条 seek）
  - 暂停切换、存原图快照
  - 通过回调发布帧，不直接依赖海康/串口
