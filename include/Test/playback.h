#ifndef TEST_PLAYBACK_H
#define TEST_PLAYBACK_H

#include <opencv2/opencv.hpp>
#include <atomic>
#include <functional>
#include <string>
#include <thread>

/** test 模式回放状态：进度条 seek / 暂停（主线程与回放线程共享） */
struct TestPlaybackState {
    std::atomic<int> total_frames{0};
    std::atomic<int> current_frame{0};
    std::atomic<int> seek_request{-1};
    std::atomic<bool> trackbar_programmatic{false};
    std::atomic<bool> paused{false};
};

/** test 视频/照片回放 */
class TestPlayer {
public:
    TestPlaybackState state;

    static bool is_image_path(const std::string& path);
    static void on_trackbar(int pos, void* userdata);

    void sync_trackbar(const std::string& window_name = "Pikachu View");

    /**
     * 启动回放线程。
     * @param publish 写入共享帧（由 main 提供）
     * @param running 全局运行标志；失败时会置 false
     */
    void start(const std::string& media_path, bool loop, float gain, double image_fps,
               std::atomic<bool>& running, std::function<void(const cv::Mat&)> publish);

    void join();

    void toggle_pause();
    bool is_paused() const;

    /** 等待视频 total_frames 就绪，返回帧数（至少 1） */
    int wait_total_frames(std::atomic<bool>& running, int timeout_ms = 2000);

    /** 保存无绘制的 BGR 原图 */
    static void save_snapshot(const std::string& save_dir, const cv::Mat& clean_bgr);

private:
    std::thread worker_;
};

#endif
