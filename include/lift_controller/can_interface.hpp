#ifndef LIFT_CONTROLLER_CAN_INTERFACE_HPP_
#define LIFT_CONTROLLER_CAN_INTERFACE_HPP_

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/time.h>
#include <string>
#include <functional>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <cstring>
#include <iostream>
#include <deque>
#include <condition_variable>

namespace lift_controller {

struct CanFrame {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    struct timeval timestamp;
};

class CanInterface {
public:
    explicit CanInterface(const std::string& interface = "can1");
    ~CanInterface();

    bool init(int bitrate = 1000000);
    void shutdown();
    bool is_connected() const { return connected_.load(); }

    bool send(const CanFrame& frame);
    bool receive(CanFrame& frame, int timeout_ms = 0);
    // 发送 SDO 请求并等待响应(通过内部 SDO 响应队列,不直接 poll socket)
    bool sendAndWait(const CanFrame& tx_frame, CanFrame& rx_frame,
                     uint32_t expected_id, int timeout_ms = 200);

    void setReceiveCallback(std::function<void(const CanFrame&)> callback);
    void startReceiveThread();
    void stopReceiveThread();

private:
    void receiveLoop();  // 接收线程:分发所有帧,SDO 响应入队,TPDO/其他走回调

    std::string interface_;
    int socket_fd_;
    std::atomic<bool> connected_;
    std::atomic<bool> running_;
    std::mutex mutex_;          // 保护 socket 写/关闭
    std::mutex cmd_mutex_;      // 序列化 sendAndWait(发送+接收配对)
    std::function<void(const CanFrame&)> receive_callback_;
    std::mutex callback_mutex_;  // 保护 receive_callback_
    std::thread receive_thread_;

    // SDO 响应队列:接收线程入队,sendAndWait 出队等待
    std::mutex sdo_mutex_;
    std::condition_variable sdo_cv_;
    std::deque<CanFrame> sdo_queue_;
};

} // namespace lift_controller

#endif
