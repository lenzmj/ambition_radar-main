#include "Tools/Record/record.h"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace cv;
using namespace std;

DatasetRecorder::~DatasetRecorder() {
    stop();
}

bool DatasetRecorder::open_mp4_writer(VideoWriter& writer, const string& path, double fps,
                                      const Size& size) {
    const int codecs[] = {
        VideoWriter::fourcc('m', 'p', '4', 'v'),
        VideoWriter::fourcc('a', 'v', 'c', '1'),
        VideoWriter::fourcc('H', '2', '6', '4'),
    };
    for (int fourcc : codecs) {
        if (writer.open(path, fourcc, fps, size, true))
            return true;
    }
    return false;
}

string DatasetRecorder::make_timestamp() {
    const auto now = chrono::system_clock::now();
    const time_t t = chrono::system_clock::to_time_t(now);
    tm tm_buf{};
    localtime_r(&t, &tm_buf);
    ostringstream oss;
    oss << put_time(&tm_buf, "%Y%m%d_%H%M%S");
    return oss.str();
}

string DatasetRecorder::make_video_path(const string& base_dir, const string& stamp) {
    return base_dir + "/record_" + stamp + ".mp4";
}

bool DatasetRecorder::start(const string& record_dir, double max_fps) {
    if (enabled_)
        return true;

    filesystem::create_directories(record_dir);
    video_path_ = make_video_path(record_dir, make_timestamp());
    min_interval_s_ = (max_fps > 0.0) ? (1.0 / max_fps) : 0.0;
    video_fps_ = (max_fps > 0.0) ? max_fps : 30.0;

    running_.store(true);
    enabled_ = true;
    worker_ = thread(&DatasetRecorder::writer_loop, this);
    cout << "[dataset] 比赛录制（YOLO 输入）-> " << video_path_ << "\n";
    return true;
}

void DatasetRecorder::try_push(const Mat& frame) {
    if (!enabled_ || frame.empty())
        return;

    const auto now_tick = chrono::steady_clock::now();
    if (min_interval_s_ > 0.0 &&
        chrono::duration<double>(now_tick - last_push_).count() < min_interval_s_) {
        return;
    }

    Mat rec_frame;
    frame.copyTo(rec_frame);
    {
        lock_guard<mutex> lock(mtx_);
        if (queue_.size() >= kQueueMax) {
            queue_.pop_front();
            drops_.fetch_add(1);
        }
        queue_.push_back(std::move(rec_frame));
    }
    cv_.notify_one();
    last_push_ = now_tick;
}

void DatasetRecorder::stop() {
    if (!enabled_)
        return;
    running_.store(false);
    cv_.notify_all();
    if (worker_.joinable())
        worker_.join();
    enabled_ = false;
}

void DatasetRecorder::writer_loop() {
    VideoWriter writer;
    bool writer_ready = false;
    size_t frames_written = 0;

    while (true) {
        Mat frame;
        {
            unique_lock<mutex> lock(mtx_);
            cv_.wait(lock, [this] { return !queue_.empty() || !running_.load(); });
            if (queue_.empty() && !running_.load())
                break;
            if (queue_.empty())
                continue;
            frame = std::move(queue_.front());
            queue_.pop_front();
        }

        if (!writer_ready) {
            if (!open_mp4_writer(writer, video_path_, video_fps_, frame.size())) {
                cerr << "[dataset] 无法创建 MP4: " << video_path_ << "\n";
                return;
            }
            writer_ready = true;
            cout << "[dataset] MP4: " << video_path_ << " (" << frame.cols << "x" << frame.rows
                 << " @ " << video_fps_ << " fps)\n";
        }
        writer.write(frame);
        ++frames_written;
    }

    if (writer_ready)
        writer.release();

    const uint64_t drops = drops_.exchange(0);
    if (drops > 0) {
        cerr << "[dataset] 录像队列满，丢弃 " << drops << " 帧\n";
    }
    if (frames_written > 0) {
        cout << "[dataset] 录像结束，共 " << frames_written << " 帧 -> " << video_path_ << "\n";
    } else if (writer_ready) {
        filesystem::remove(video_path_);
    }
}
