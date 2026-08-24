#include "Test/playback.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace cv;
using namespace std;

namespace {

string to_lower_ascii(string s) {
    transform(s.begin(), s.end(), s.begin(),
              [](unsigned char c) { return static_cast<char>(tolower(c)); });
    return s;
}

void apply_test_gain(Mat& frame, float gain) {
    if (frame.empty() || gain <= 0.f || abs(gain - 1.f) < 1e-3f)
        return;
    frame.convertTo(frame, -1, static_cast<double>(gain), 0.0);
}

void image_playback_loop(const string& image_path, bool loop, float gain, double fps,
                         TestPlaybackState* pb, atomic<bool>& running,
                         function<void(const Mat&)> publish) {
    Mat image = imread(image_path, IMREAD_COLOR);
    if (image.empty()) {
        cerr << "[test] 无法打开照片: " << image_path << "\n";
        running = false;
        return;
    }
    if (fps <= 1.0 || fps > 240.0)
        fps = 10.0;
    const auto frame_period = chrono::duration_cast<chrono::steady_clock::duration>(
        chrono::duration<double>(1.0 / fps));
    auto next_tick = chrono::steady_clock::now();

    if (pb) {
        pb->total_frames.store(1, memory_order_relaxed);
        pb->current_frame.store(0, memory_order_relaxed);
    }

    cout << "[test] 照片 " << image_path << " (" << image.cols << "x" << image.rows << " @ " << fps
         << " fps";
    if (abs(gain - 1.f) >= 1e-3f)
        cout << ", gain=" << gain;
    if (loop)
        cout << ", 循环";
    cout << ")\n";

    Mat frame;
    while (running) {
        if (pb && pb->paused.load(memory_order_relaxed)) {
            this_thread::sleep_for(chrono::milliseconds(10));
            next_tick = chrono::steady_clock::now();
            continue;
        }
        image.copyTo(frame);
        apply_test_gain(frame, gain);
        publish(frame);
        if (!loop)
            break;
        next_tick += frame_period;
        this_thread::sleep_until(next_tick);
    }
}

void video_playback_loop(const string& video_path, bool loop, float gain, TestPlaybackState* pb,
                         atomic<bool>& running, function<void(const Mat&)> publish) {
    VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        cerr << "[test] 无法打开视频: " << video_path << "\n";
        running = false;
        return;
    }
    double fps = cap.get(CAP_PROP_FPS);
    if (fps <= 1.0 || fps > 240.0)
        fps = 30.0;
    const auto frame_period = chrono::duration_cast<chrono::steady_clock::duration>(
        chrono::duration<double>(1.0 / fps));
    auto next_tick = chrono::steady_clock::now();

    int total = static_cast<int>(cap.get(CAP_PROP_FRAME_COUNT));
    if (total <= 0)
        total = 1;
    if (pb) {
        pb->total_frames.store(total, memory_order_relaxed);
        pb->current_frame.store(0, memory_order_relaxed);
    }

    cout << "[test] 回放 " << video_path << " (" << cap.get(CAP_PROP_FRAME_WIDTH) << "x"
         << cap.get(CAP_PROP_FRAME_HEIGHT) << " @ " << fps << " fps, " << total << " 帧";
    if (abs(gain - 1.f) >= 1e-3f)
        cout << ", gain=" << gain;
    if (loop)
        cout << ", 循环";
    cout << ")\n";

    while (running) {
        bool did_seek = false;
        if (pb) {
            const int seek = pb->seek_request.exchange(-1, memory_order_relaxed);
            if (seek >= 0) {
                cap.set(CAP_PROP_POS_FRAMES, seek);
                pb->current_frame.store(seek, memory_order_relaxed);
                did_seek = true;
            }
        }

        if (pb && pb->paused.load(memory_order_relaxed) && !did_seek) {
            this_thread::sleep_for(chrono::milliseconds(10));
            next_tick = chrono::steady_clock::now();
            continue;
        }

        Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            if (!loop)
                break;
            cap.set(CAP_PROP_POS_FRAMES, 0);
            if (pb)
                pb->current_frame.store(0, memory_order_relaxed);
            if (!cap.read(frame) || frame.empty())
                break;
        }

        if (pb) {
            const int pos = static_cast<int>(cap.get(CAP_PROP_POS_FRAMES));
            const int shown = (pos > 0) ? pos - 1 : 0;
            pb->current_frame.store(min(shown, total - 1), memory_order_relaxed);
        }

        apply_test_gain(frame, gain);
        publish(frame);

        if (did_seek) {
            next_tick = chrono::steady_clock::now();
        } else {
            next_tick += frame_period;
            this_thread::sleep_until(next_tick);
        }
    }
}

}  // namespace

bool TestPlayer::is_image_path(const string& path) {
    const string ext = to_lower_ascii(filesystem::path(path).extension().string());
    return ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp" || ext == ".tif" ||
           ext == ".tiff" || ext == ".webp";
}

void TestPlayer::on_trackbar(int pos, void* userdata) {
    auto* st = static_cast<TestPlaybackState*>(userdata);
    if (!st->trackbar_programmatic.load(memory_order_relaxed)) {
        const int total = st->total_frames.load(memory_order_relaxed);
        const int frame = max(0, min(pos, max(0, total - 1)));
        st->seek_request.store(frame, memory_order_relaxed);
    }
}

void TestPlayer::sync_trackbar(const string& window_name) {
    const int total = state.total_frames.load(memory_order_relaxed);
    if (total <= 1)
        return;
    const int cur =
        max(0, min(state.current_frame.load(memory_order_relaxed), total - 1));
    state.trackbar_programmatic.store(true, memory_order_relaxed);
    setTrackbarPos("进度", window_name, cur);
    state.trackbar_programmatic.store(false, memory_order_relaxed);
}

void TestPlayer::start(const string& media_path, bool loop, float gain, double image_fps,
                       atomic<bool>& running, function<void(const Mat&)> publish) {
    if (is_image_path(media_path)) {
        worker_ = thread(image_playback_loop, media_path, loop, gain, image_fps, &state, ref(running),
                         publish);
    } else {
        worker_ = thread(video_playback_loop, media_path, loop, gain, &state, ref(running), publish);
    }
}

void TestPlayer::join() {
    if (worker_.joinable())
        worker_.join();
}

void TestPlayer::toggle_pause() {
    const bool now_paused = !state.paused.load(memory_order_relaxed);
    state.paused.store(now_paused, memory_order_relaxed);
    cout << "[test] " << (now_paused ? "暂停 (s=保存当前原图)" : "继续") << "\n";
}

bool TestPlayer::is_paused() const {
    return state.paused.load(memory_order_relaxed);
}

int TestPlayer::wait_total_frames(atomic<bool>& running, int timeout_ms) {
    const int steps = max(1, timeout_ms / 10);
    for (int i = 0; i < steps && state.total_frames.load(memory_order_relaxed) <= 0 && running.load();
         ++i)
        this_thread::sleep_for(chrono::milliseconds(10));
    return max(1, state.total_frames.load(memory_order_relaxed));
}

void TestPlayer::save_snapshot(const string& save_dir, const Mat& clean_bgr) {
    if (clean_bgr.empty()) {
        cerr << "[test] 无可用帧可保存\n";
        return;
    }
    filesystem::create_directories(save_dir);

    static int save_seq = 0;
    ++save_seq;

    const auto now = chrono::system_clock::now();
    const time_t t = chrono::system_clock::to_time_t(now);
    tm tm_buf{};
    localtime_r(&t, &tm_buf);

    ostringstream name;
    name << save_dir << '/' << put_time(&tm_buf, "%y%m%d_%H%M_%S") << 'f' << save_seq << ".jpg";
    const string path = name.str();
    if (!imwrite(path, clean_bgr)) {
        cerr << "[test] 保存帧失败: " << path << "\n";
        return;
    }
    cout << "[test] 已保存原图(无绘制): " << path << "\n";
}
