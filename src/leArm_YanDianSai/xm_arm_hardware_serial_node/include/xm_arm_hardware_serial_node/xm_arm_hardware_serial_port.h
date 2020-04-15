#ifndef XM_ARM_HARDWARE_SERIAL_PORT_H_
#define XM_ARM_HARDWARE_SERIAL_PORT_H_

#include <xm_arm_msgs/xm_ArmSerialDatagram.h>
#include <iomanip>
#include <vector>
#include <queue>
#include <stdio.h>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/make_shared.hpp>

using namespace std;
using namespace boost;
using namespace boost::asio;

namespace xm_serial_node {
class SerialParams
{
public:
    //默认构造函数
    SerialParams() :
        serial_port_(),
        baud_rate_(9600),
        flow_control_(0),
        parity_bits_(0),
        stop_bits_(0)
    {
    }
    //有参构造函数
    SerialParams(
        string       serial_port,
        unsigned int baud_rate,
        unsigned int flow_control,
        unsigned int parity_bits,
        unsigned int stop_bits
        ) :
        serial_port_(serial_port),
        baud_rate_(baud_rate),
        flow_control_(flow_control),
        parity_bits_(parity_bits),
        stop_bits_(stop_bits)
    {
    }
public:
    string       serial_port_;//串口
    unsigned int baud_rate_;//波特率
    unsigned int flow_control_;//流控制
    unsigned int parity_bits_;//
    unsigned int stop_bits_;//停止位
};

typedef vector<u_int8_t> byte_vector;

typedef boost::shared_ptr<byte_vector> ptr_byte_vector;

//这个类是serialNode的成员类
class SerialPort
{
public:
    SerialPort();
    virtual ~SerialPort();
public:
    //将serialNode设置好的serialParam传给自己
    void setSerialParams(const SerialParams &params);
    //普通传参
    void setTimeOut(int timeout);
    //
    bool startThread();
    //
    bool stopThread();
    //绑定回调函数????
    void setCallbackFunc(
        const boost::function<void(xm_arm_msgs::xm_ArmSerialDatagramPtr)> &func);
    //订阅数据,写到buffer
    bool writeDataGram(const xm_arm_msgs::xm_ArmSerialDatagram &datagram);
    bool writeRaw(const byte_vector &raw_data);
private:
    void runMain();
    void startOneRead();
    void startOneWrite();
    void readHandler(const system::error_code &ec, size_t bytes_trans);
    void writeHandler(const system::error_code &ec);
    void timeoutHandler(const system::error_code &ec);
private:
    boost::shared_ptr<deadline_timer> ptr_timer_;//
    boost::shared_ptr<io_service>     ptr_io_service_;//
    boost::shared_ptr<serial_port>    ptr_serial_;//
    mutex                      serial_mutex_;

    enum {HEADER_LEN = 2};
    // enum STATE {
    //     WAITING_FF, WAITING_FF2, READING_HEAD, READING_DATA, READING_CHECKSUM
    // } state_;//要根据下位机的协议修改
    enum STATE {
        WAITING_H1, WAITING_H2,GET_LENGTH,GET_CMD, READING_DATA, READING_CHECKSUM
    } state_;//根据下位机的协议修改

    SerialParams           serial_params_;//
    int                    timeout_;//
    byte_vector            temp_buffer_;//缓存,异步读取后masyn_read_some会将数据存储到这个数据结构里
    byte_vector            current_header_;//当前头
    byte_vector            current_data_;//当前的数据
    size_t                 header_bytes_read_;//数据头的个数
    size_t                 data_bytes_read_;//数据的个数
    queue<ptr_byte_vector> write_queue_;//写入数据的队列
    boost::mutex           write_queue_mutex_;//写入数据线程锁.
    boost::thread          thread_;
    boost::function<void(xm_arm_msgs::xm_ArmSerialDatagramPtr)> data_callback_func_;//返回值为空的函数对象
    boost::function<void()>                                     error_callback_func_;//返回值和参数都为空的函数对象
};

} // namespace xm_serial_node

#endif // XM_ARM_HARDWARE_SERIAL_PORT_H_
