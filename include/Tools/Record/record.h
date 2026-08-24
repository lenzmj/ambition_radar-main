#ifndef TOOLS_RECORD_H
#define TOOLS_RECORD_H

#include <opencv2/opencv.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

/** hik 比赛录制：异步将 YOLO 输入 BGR 写入 MP4 */
class DatasetRecorder {
public:
    DatasetRecorder() = default;
    ~DatasetRecorder();

    DatasetRecorder(const DatasetRecorder&) = delete;
    DatasetRecorder& operator=(const DatasetRecorder&) = delete;

    /** 创建目录并启动写线程；max_fps<=0 表示不限帧。失败返回 false */
    bool start(const std::string& record_dir, double max_fps);

    bool enabled() const { return enabled_; }

    /** 按 max_fps 限流入队；队列满则丢最旧帧 */
    void try_push(const cv::Mat& frame);

    /** 停止写线程并 join */
    void stop();

private:
    void writer_loop();
    static bool open_mp4_writer(cv::VideoWriter& writer, const std::string& path, double fps,
                                const cv::Size& size);
    static std::string make_timestamp();
    static std::string make_video_path(const std::string& base_dir, const std::string& stamp);

    bool enabled_ = false;
    std::atomic<bool> running_{false};
    std::string video_path_;
    double video_fps_ = 30.0;
    double min_interval_s_ = 0.0;

    std::mutex mtx_;
    std::deque<cv::Mat> queue_;
    std::condition_variable cv_;
    std::atomic<uint64_t> drops_{0};
    static constexpr size_t kQueueMax = 16;

    std::thread worker_;
    std::chrono::steady_clock::time_point last_push_{};
};

#endif
