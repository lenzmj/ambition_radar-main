#include "Detect/Detector/detector.h"
#include "yaml.hpp"
#include <algorithm>
#include <cctype>
#include <iostream>

using namespace cv;
using namespace std;

static string to_lower_ascii(string s) {
    for (char& c : s) {
        c = static_cast<char>(tolower(static_cast<unsigned char>(c)));
    }
    return s;
}

/**
 * 数据集固定：0=blue 1=red 2=white（换边不改编名，换模型训敌方主色）。
 * 我方红打蓝 -> 优先 0；我方蓝打红 -> 优先 1；都可 fallback white=2。
 */
static int primary_class_from_our_side(const string& our_side) {
    if (to_lower_ascii(our_side) == "blue") {
        return 1;  // 敌方红
    }
    return 0;  // 敌方蓝
}

/** 在指定类别得分通道上找最高置信候选；通道非法返回 false */
static bool best_on_score_channel(const float* data, int rows, int score_channel, int dimensions,
                                  float conf, float& best_score, int& best_idx) {
    if (score_channel < 4 || score_channel >= dimensions - 1) {
        return false;
    }
    best_score = -1.0f;
    best_idx = -1;
    for (int i = 0; i < rows; ++i) {
        const float score = data[score_channel * rows + i];
        if (score > conf && score > best_score) {
            best_score = score;
            best_idx = i;
        }
    }
    return best_idx != -1;
}

struct YoloPick {
    const float* data = nullptr;
    int dimensions = 0;
    int rows = 0;
    int class_id = -1;
    int best_idx = -1;
    float score = -1.0f;
};

static bool yolo_output_valid(const YoloInferOutput& out) {
    return out.data && out.dimensions > 0 && out.rows > 0;
}

/** 单模型：优先主色，否则同模型 white 通道 */
static YoloPick pick_primary_then_white(const YoloInferOutput& out, int primary_class_id,
                                        int fallback_class_id, float primary_conf,
                                        float white_conf) {
    YoloPick pick;
    if (!yolo_output_valid(out)) {
        return pick;
    }

    pick.data = out.data;
    pick.dimensions = out.dimensions;
    pick.rows = out.rows;

    const int num_classes = out.dimensions - 5;
    const int primary_ch = (num_classes <= 1) ? 4 : (4 + primary_class_id);
    const int fallback_ch = 4 + fallback_class_id;

    if (best_on_score_channel(out.data, out.rows, primary_ch, out.dimensions, primary_conf,
                              pick.score, pick.best_idx)) {
        pick.class_id = primary_class_id;
    } else if (num_classes > fallback_class_id &&
               best_on_score_channel(out.data, out.rows, fallback_ch, out.dimensions, white_conf,
                                     pick.score, pick.best_idx)) {
        pick.class_id = fallback_class_id;
    }
    return pick;
}

/** 双模型主路：只检敌方主色 */
static YoloPick pick_primary_only(const YoloInferOutput& out, int primary_class_id,
                                  float primary_conf) {
    YoloPick pick;
    if (!yolo_output_valid(out)) {
        return pick;
    }

    pick.data = out.data;
    pick.dimensions = out.dimensions;
    pick.rows = out.rows;

    const int num_classes = out.dimensions - 5;
    const int primary_ch = (num_classes <= 1) ? 4 : (4 + primary_class_id);

    if (best_on_score_channel(out.data, out.rows, primary_ch, out.dimensions, primary_conf,
                              pick.score, pick.best_idx)) {
        pick.class_id = primary_class_id;
    }
    return pick;
}

/** 双模型白靶路：只检 white */
static YoloPick pick_white_only(const YoloInferOutput& out, int white_class_id, float white_conf) {
    YoloPick pick;
    if (!yolo_output_valid(out)) {
        return pick;
    }

    const int num_classes = out.dimensions - 5;
    const int white_ch = 4 + white_class_id;
    if (num_classes <= white_class_id) {
        return pick;
    }

    pick.data = out.data;
    pick.dimensions = out.dimensions;
    pick.rows = out.rows;

    if (best_on_score_channel(out.data, out.rows, white_ch, out.dimensions, white_conf, pick.score,
                              pick.best_idx)) {
        pick.class_id = white_class_id;
    }
    return pick;
}

static DetectResult build_detect_result(const YoloPick& pick, float scale_x, float scale_y) {
    DetectResult current;
    const int best_idx = pick.best_idx;
    const float* data = pick.data;
    const int dimensions = pick.dimensions;
    const int rows = pick.rows;

    const float cx_net = data[0 * rows + best_idx];
    const float cy_net = data[1 * rows + best_idx];
    const float w_net = data[2 * rows + best_idx];
    const float h_net = data[3 * rows + best_idx];

    const float angle_rad = data[(dimensions - 1) * rows + best_idx];
    const float angle_deg = angle_rad * 180.0f / CV_PI;

    RotatedRect rrect_net(Point2f(cx_net, cy_net), Size2f(w_net, h_net), angle_deg);
    Point2f pts_net[4];
    rrect_net.points(pts_net);

    current.corners.clear();
    for (int j = 0; j < 4; j++) {
        Point2f pt_original;
        pt_original.x = pts_net[j].x * scale_x;
        pt_original.y = pts_net[j].y * scale_y;
        current.corners.push_back(pt_original);
    }

    current.box = boundingRect(current.corners);
    current.score = pick.score;
    current.class_id = pick.class_id;
    return current;
}

Detector::Detector() {
    auto& cfg = ConfigManager::getInstance();
    const string backend = cfg.get<string>("hardware.inference_backend", "openvino");
    const string model_path = cfg.get<string>("hardware.model_path", "");
    // 注释掉 / 不写 / 空字符串 → 不加载第二模型，run_yolo 只推一次
    string white_model_path = cfg.get<string>("hardware.white_model_path", "");
    while (!white_model_path.empty() &&
           isspace(static_cast<unsigned char>(white_model_path.front()))) {
        white_model_path.erase(white_model_path.begin());
    }
    while (!white_model_path.empty() &&
           isspace(static_cast<unsigned char>(white_model_path.back()))) {
        white_model_path.pop_back();
    }
    // yaml-cpp 旧行为兜底：缺省键曾被读成字面量 "null"
    if (white_model_path == "null" || white_model_path == "~" || white_model_path == "Null") {
        white_model_path.clear();
    }
    const string our_side = cfg.get<string>("hardware.our_side", "red");
    primary_class_id_ = primary_class_from_our_side(our_side);
    fallback_class_id_ = 2;  // white
    use_white_backend_ = false;
    try {
        backend_ = create_yolo_infer_backend(backend, model_path);
    } catch (const exception& e) {
        cerr << "[Detector 错误] 主模型初始化失败: " << e.what() << endl;
        return;
    }
    if (!white_model_path.empty()) {
        try {
            white_backend_ = create_yolo_infer_backend(backend, white_model_path);
            use_white_backend_ = static_cast<bool>(white_backend_);
        } catch (const exception& e) {
            cerr << "[Detector 警告] 白靶模型加载失败，回退单模型: " << e.what() << endl;
            white_backend_.reset();
            use_white_backend_ = false;
        }
    }
    const char* primary_name =
        (primary_class_id_ == 1) ? "red" : (primary_class_id_ == 0) ? "blue" : "?";
    cout << "[Detector] 推理后端: " << backend << endl;
    cout << "[Detector] 我方: " << our_side << "，主模型: " << model_path << endl;
    if (use_white_backend_) {
        cout << "[Detector] 白靶模型: " << white_model_path << endl;
    } else {
        cout << "[Detector] 未配置 white_model_path → 单模型（主色+同模型 white，无额外推理）"
             << endl;
    }
    cout << "[Detector] 优先类别 id=" << primary_class_id_ << "（" << primary_name
         << "），无目标时 fallback id=" << fallback_class_id_ << "（white）" << endl;
    const float conf = cfg.get<float>("params.conf_threshold", 0.6f);
    const float white_conf = cfg.get<float>("params.white_conf_threshold", conf);
    const int det_lost = cfg.get<int>("params.det_lost", 2);
    cout << "[Detector] conf_threshold=" << conf << "，white_conf_threshold=" << white_conf << endl;
    cout << "[Detector] det_lost=" << det_lost << endl;
}

void Detector::clear_track() {
    has_history_ = false;
    det_miss_cnt_ = 0;
    white_sticky_ = false;
}

vector<DetectResult> Detector::run_yolo(Mat& frame) {
    auto& cfg = ConfigManager::getInstance();
    const float yaml_conf = cfg.get<float>("params.conf_threshold", 0.6f);
    const float white_conf = cfg.get<float>("params.white_conf_threshold", yaml_conf);
    const float yaml_alpha = cfg.get<float>("params.det_alpha", 0.1f);
    // 补帧 + 白靶 sticky 共用：连续未检出最多保留 det_lost 帧
    const int det_lost = cfg.get<int>("params.det_lost", 2);

    last_detection_fresh_ = false;
    vector<DetectResult> final_results;
    if (frame.empty() || !backend_) {
        return final_results;
    }

    Mat blob;
    dnn::blobFromImage(frame, blob, 1.0 / 255.0, Size(640, 640), Scalar(), true, false);

    // ----- 1) 推理：选主色 / 白靶 -----
    YoloPick pick;
    if (use_white_backend_) {
        if (white_sticky_) {
            // sticky：已锁定白靶，本帧只推白模型（补帧期内也保持）
            const YoloInferOutput white_out = white_backend_->infer(blob);
            pick = pick_white_only(white_out, fallback_class_id_, white_conf);
        } else {
            // 串行：主色 → 未中再白靶
            const YoloInferOutput primary_out = backend_->infer(blob);
            pick = pick_primary_only(primary_out, primary_class_id_, yaml_conf);
            if (pick.best_idx < 0) {
                const YoloInferOutput white_out = white_backend_->infer(blob);
                pick = pick_white_only(white_out, fallback_class_id_, white_conf);
            }
        }
    } else {
        const YoloInferOutput yolo_out = backend_->infer(blob);
        pick = pick_primary_then_white(yolo_out, primary_class_id_, fallback_class_id_, yaml_conf,
                                       white_conf);
    }

    const float scale_x = static_cast<float>(frame.cols) / 640.0f;
    const float scale_y = static_cast<float>(frame.rows) / 640.0f;

    // ----- 2) 跟踪：检出更新 / 未检出补帧或清空 -----
    if (pick.best_idx != -1) {
        DetectResult current = build_detect_result(pick, scale_x, scale_y);

        if (!has_history_) {
            last_res = current;
            has_history_ = true;
        } else {
            for (int j = 0; j < 4; j++) {
                last_res.corners[j].x =
                    yaml_alpha * current.corners[j].x + (1 - yaml_alpha) * last_res.corners[j].x;
                last_res.corners[j].y =
                    yaml_alpha * current.corners[j].y + (1 - yaml_alpha) * last_res.corners[j].y;
            }
            last_res.box = current.box;
            last_res.score = current.score;
            last_res.class_id = current.class_id;
        }
        det_miss_cnt_ = 0;
        last_detection_fresh_ = true;
        final_results.push_back(last_res);

        // 双模型：检出 white → sticky；检出主色 → 退出 sticky
        white_sticky_ = use_white_backend_ && (current.class_id == fallback_class_id_);
        return final_results;
    }

    // 未检出：最多补 det_lost 帧；期间 sticky 不变；超限清空（sticky 一并清）
    if (has_history_ && det_lost > 0 && det_miss_cnt_ < det_lost) {
        det_miss_cnt_++;
        final_results.push_back(last_res);  // 补帧：fresh=false
    } else {
        clear_track();
    }

    return final_results;
}
