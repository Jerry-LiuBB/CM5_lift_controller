#include "lift_controller/can_interface.hpp"
#include <cstring>
#include <cerrno>
#include <iostream>
#include <unistd.h>

namespace lift_controller {

CanInterface::CanInterface(const std::string& interface)
    : interface_(interface), socket_fd_(-1), connected_(false), running_(false) {}

CanInterface::~CanInterface() { shutdown(); }

bool CanInterface::init(int bitrate) {
    (void)bitrate;  // SocketCAN 中波特率由 ip link 配置,此处忽略
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (connected_) return true;

        socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0) {
            std::cerr << "[CAN] Failed to create socket: " << strerror(errno) << std::endl;
            return false;
        }

        struct ifreq ifr;
        std::strcpy(ifr.ifr_name, interface_.c_str());
        if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
            std::cerr << "[CAN] Failed to get interface index: " << strerror(errno) << std::endl;
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "[CAN] Failed to bind socket: " << strerror(errno) << std::endl;
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        int flags = fcntl(socket_fd_, F_GETFL, 0);
        if (flags < 0) flags = 0;
        fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

        connected_ = true;
        std::cout << "[CAN] Connected to " << interface_ << std::endl;
    }

    // 自动启动接收线程(在锁外启动,避免与 receiveLoop 死锁)
    startReceiveThread();
    return true;
}

void CanInterface::shutdown() {
    stopReceiveThread();
    std::lock_guard<std::mutex> lock(mutex_);
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
    connected_ = false;
    // 唤醒可能在等待的 sendAndWait
    sdo_cv_.notify_all();
}

bool CanInterface::send(const CanFrame& frame) {
    if (!connected_) return false;

    struct can_frame can_fr;
    std::memset(&can_fr, 0, sizeof(can_fr));
    can_fr.can_id = frame.id;
    can_fr.can_dlc = frame.dlc;
    std::memcpy(can_fr.data, frame.data, frame.dlc);

    std::lock_guard<std::mutex> lock(mutex_);
    return write(socket_fd_, &can_fr, sizeof(can_fr)) == sizeof(can_fr);
}

bool CanInterface::receive(CanFrame& frame, int timeout_ms) {
    if (!connected_) return false;

    struct pollfd pfd;
    pfd.fd = socket_fd_;
    pfd.events = POLLIN;

    if (poll(&pfd, 1, timeout_ms) <= 0) return false;

    struct can_frame can_fr;
    int nbytes = read(socket_fd_, &can_fr, sizeof(can_fr));
    if (nbytes != sizeof(can_fr)) return false;

    frame.id = can_fr.can_id & CAN_EFF_MASK;
    frame.dlc = can_fr.can_dlc;
    std::memcpy(frame.data, can_fr.data, can_fr.can_dlc);
    gettimeofday(&frame.timestamp, nullptr);
    return true;
}

bool CanInterface::sendAndWait(const CanFrame& tx_frame, CanFrame& rx_frame,
                                uint32_t expected_id, int timeout_ms) {
    // 序列化整个发送+接收配对,避免并发调用者互相消费对方的 SDO 响应
    std::lock_guard<std::mutex> lock(cmd_mutex_);

    // 清除队列中的陈旧 SDO 响应
    {
        std::lock_guard<std::mutex> qlock(sdo_mutex_);
        sdo_queue_.clear();
    }

    if (!send(tx_frame)) {
        std::cerr << "[CAN] sendAndWait: send failed, id=0x" << std::hex << tx_frame.id
                  << std::dec << std::endl;
        return false;
    }

    std::cerr << "[CAN] sendAndWait: sent id=0x" << std::hex << tx_frame.id
              << ", expecting id=0x" << expected_id << std::dec << std::endl;

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (true) {
        std::unique_lock<std::mutex> qlock(sdo_mutex_);

        // 在队列中查找匹配的响应帧
        for (auto it = sdo_queue_.begin(); it != sdo_queue_.end(); ++it) {
            if (it->id == expected_id) {
                rx_frame = *it;
                sdo_queue_.erase(it);
                return true;
            }
        }

        // 等待新 SDO 响应到达
        if (sdo_cv_.wait_until(qlock, deadline) == std::cv_status::timeout) {
            // 超时后最终检查一次
            for (auto it = sdo_queue_.begin(); it != sdo_queue_.end(); ++it) {
                if (it->id == expected_id) {
                    rx_frame = *it;
                    sdo_queue_.erase(it);
                    return true;
                }
            }
            std::cerr << "[CAN] sendAndWait: TIMEOUT waiting for id=0x" << std::hex
                      << expected_id << std::dec << std::endl;
            return false;
        }
    }
}

void CanInterface::setReceiveCallback(std::function<void(const CanFrame&)> callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    receive_callback_ = callback;
}

void CanInterface::startReceiveThread() {
    if (running_) return;
    running_ = true;
    receive_thread_ = std::thread(&CanInterface::receiveLoop, this);
}

void CanInterface::stopReceiveThread() {
    if (!running_) return;
    running_ = false;
    sdo_cv_.notify_all();  // 唤醒可能在等待的 sendAndWait
    if (receive_thread_.joinable()) receive_thread_.join();
}

void CanInterface::receiveLoop() {
    // 接收线程:分发所有 CAN 帧
    // - SDO 响应(0x580-0x5FF)入队,供 sendAndWait 用 condition_variable 等待
    // - TPDO / EMCY / SYNC / 心跳等走 receive_callback_
    while (running_) {
        CanFrame frame;
        if (receive(frame, 5)) {
            if (frame.id >= 0x580 && frame.id < 0x600) {
                // SDO 响应帧 → 入队
                std::lock_guard<std::mutex> qlock(sdo_mutex_);
                sdo_queue_.push_back(frame);
                sdo_cv_.notify_one();
            } else {
                // TPDO / 其他帧 → 回调
                std::function<void(const CanFrame&)> cb;
                {
                    std::lock_guard<std::mutex> cb_lock(callback_mutex_);
                    cb = receive_callback_;
                }
                if (cb) cb(frame);
            }
        }
    }
}

} // namespace lift_controller
