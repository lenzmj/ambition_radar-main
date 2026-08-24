#ifndef CONFIG_MANAGER_HPP
#define CONFIG_MANAGER_HPP

#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <iostream>
#include <mutex>
#include <memory>

/**
 * @brief ConfigManager 单例类
 * 支持通过 "a.b.c" 这种 key 路径访问 YAML 嵌套节点
 */
class ConfigManager {
public:
    // 获取单例实例 (C++11 局部静态变量保证线程安全)
    static ConfigManager& getInstance() {
        static ConfigManager instance;
        return instance;
    }

    // 初始化：加载 YAML 文件
    void init(const std::string& config_path) {
        std::lock_guard<std::mutex> lock(mtx_);
        try {
            config_ = YAML::LoadFile(config_path);
            path_ = config_path;
            std::cout << "[Config] 配置文件加载成功: " << config_path << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[Config 错误] 无法加载配置文件: " << e.what() << std::endl;
        }
    }

    /**
     * @brief 获取配置参数
     * @param key 键值路径，例如 "params.conf_threshold"
     * @param default_val 默认值（如果键不存在或解析失败）
     */
    template <typename T>
    T get(const std::string& key, T default_val) {
        std::lock_guard<std::mutex> lock(mtx_);
        try {
            YAML::Node node = findNode(key);
            // Null 节点（键不存在）对 string 会 as<> 成 "null"，必须排除
            if (node && node.IsDefined() && !node.IsNull()) {
                return node.as<T>();
            }
        } catch (...) {
            // 解析失败时返回默认值
        }
        return default_val;
    }

    // 针对 vector 的特化支持（方便读取内参等数组）
    template <typename T>
    std::vector<T> getVector(const std::string& key) {
        std::lock_guard<std::mutex> lock(mtx_);
        try {
            YAML::Node node = findNode(key);
            if (node && node.IsSequence()) {
                return node.as<std::vector<T>>();
            }
        } catch (const std::exception& e) {
            std::cerr << "[Config 警告] 读取列表 " << key << " 失败: " << e.what() << std::endl;
        }
        return {};
    }

private:
    ConfigManager() = default;
    ~ConfigManager() = default;
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;

    // 解析 "a.b.c" 形式的路径
    YAML::Node findNode(const std::string& key) {
        YAML::Node current = YAML::Clone(config_);
        std::string segment;
        std::stringstream ss(key);

        while (std::getline(ss, segment, '.')) {
            if (current[segment]) {
                current = current[segment];
            } else {
                return YAML::Node(); // 未找到节点
            }
        }
        return current;
    }

    YAML::Node config_;
    std::string path_;
    std::mutex mtx_; // 保证多线程读取安全
};

#endif // CONFIG_MANAGER_HPP