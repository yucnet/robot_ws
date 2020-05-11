#include <xm_arm_hardware_serial_node/xm_arm_hardware_serial_port.h>
#include <boost/function.hpp>
using namespace std;

namespace xm_serial_node {
 
SerialPort::SerialPort()
{
    cout << "SerialPort Object created!" << endl;
    temp_buffer_.resize(1024, 0);
}
SerialPort::~SerialPort()
{
    ptr_io_service_->stop();
    thread_.join();
}

void SerialPort::startOneRead()
{
    cout << "SerialPort::startOneRead called !!" << endl;
    mutex::scoped_lock lock(serial_mutex_);
    ptr_serial_->async_read_some(buffer(temp_buffer_),
        boost::bind(&SerialPort::readHandler, this, boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
}

void SerialPort::startOneWrite()
{
    mutex::scoped_lock lock(serial_mutex_);
    boost::asio::async_write(*ptr_serial_, buffer(*(write_queue_.front())),
            boost::bind(&SerialPort::writeHandler, this, boost::asio::placeholders::error));
}

void SerialPort::runMain()
{
    cout << "SerialPort mainThread STARTED!" << endl;
    state_ = WAITING_H1;
    current_header_.resize(4, 0);
    startOneRead();
    ptr_io_service_->run();//异步执行
    cout << "SerialPort mainThread EXITED!" << endl;
}

void SerialPort::readHandler(const system::error_code &ec, size_t bytes_trans)
{
    //ec:函数调用产生错误抛出的异常,bytes_trans:数据大小(字节)
    if (ec)
    {
        cout << "SerialPort read error !!" << endl;
        return;
    }

    //逐字节读取数据
    for (size_t i = 0; i < bytes_trans; i++)
    {
        u_int8_t byte = temp_buffer_.at(i);
        printf("State=%d, Processing byte: 0x%X \n", state_, byte);

        switch (state_)
        {
            case WAITING_H1:
                if (byte == (u_int8_t)0x55)
                {
                    state_ = WAITING_H2;
                    //设置读取超时100ms
                    ptr_timer_.reset(new deadline_timer(*ptr_io_service_,
                        posix_time::milliseconds(timeout_)));
                    ptr_timer_->async_wait(boost::bind(&SerialPort::timeoutHandler,
                        this, boost::asio::placeholders::error));
                }
            break;
            case WAITING_H2:
                if (byte == (u_int8_t)0x55)
                {
                    cout << "Get a new datagram header !!" << endl;
                    //header_bytes_read_ = 0;
                    state_ = GET_LENGTH;
                }
                else
                    state_ = WAITING_H1;
            break;
            case GET_LENGTH:
                current_header_[header_bytes_read_] = byte;
                header_bytes_read_++;

                if (HEADER_LEN == header_bytes_read_)
                {
                    uint16_t param_len = ((uint16_t)(current_header_[2]) << 8) +
                        current_header_[3];
                if (param_len > 0)
                {
                    current_data_.resize(param_len, 0);
                    data_bytes_read_ = 0;
                    state_ = READING_DATA;
                }
                else
                    state_ = WAITING_H1;
                }
                break;
            case READING_DATA:
                current_data_[data_bytes_read_++] = byte;

                if (current_data_.size() == data_bytes_read_)
                    state_ = READING_CHECKSUM;
                break;
            case READING_CHECKSUM:
                ptr_timer_->cancel();
                ptr_timer_.reset();
                uint32_t byte_sum = 0;

                for (size_t k = 0; k < current_header_.size(); k++)
                    byte_sum += current_header_.at(k);
                for (size_t k = 0; k < current_data_.size(); k++)
                    byte_sum += current_data_.at(k);

                if ((u_int8_t)(byte_sum % 255) == byte)
                {
                    xm_arm_msgs::xm_ArmSerialDatagramPtr ptr_new_diagram =
                        boost::make_shared<xm_arm_msgs::xm_ArmSerialDatagram>();
                    ptr_new_diagram->sender     = current_header_.at(0);
                    ptr_new_diagram->receiver   = current_header_.at(1);
                    ptr_new_diagram->data       = current_data_;
                    cout << "A new datagram received !!" << endl;
                    data_callback_func_(ptr_new_diagram);
                }
                else
                    cout << "checksum error !!" << endl;

                state_ = WAITING_H1;
                break;
        }
    }
    startOneRead();
}

void SerialPort::writeHandler(const system::error_code &ec)
{
    if (ec)
    {
        cout << "Serial write error!" << endl;
        return ;
    }
    else
    {
        mutex::scoped_lock lock(write_queue_mutex_);
        write_queue_.pop();

        if (write_queue_.empty()==false)
            startOneWrite();
    }
}

void SerialPort::timeoutHandler(const system::error_code &ec)
{
    if (!ec)
    {
        cout << "Time Out !" << endl;
        state_ = WAITING_H1;
    }
}

void SerialPort::setSerialParams(const SerialParams &params)
{
    serial_params_ = params;
}

void SerialPort::setTimeOut(int timeout)
{
    timeout_ = timeout;
}

bool SerialPort::startThread()
{
    cout << "SerialPort::startThread() called!" << endl;
    ptr_io_service_ = boost::make_shared<io_service>();

    //设置参数,serial_params_中的参数来自serial_node中的用户写入
    try
    {
        ptr_serial_ = boost::make_shared<serial_port>(boost::ref(*ptr_io_service_),
        serial_params_.serial_port_);
        ptr_serial_->set_option(
            serial_port::baud_rate(serial_params_.baud_rate_));//波特率
        ptr_serial_->set_option(
            serial_port::flow_control((
            serial_port::flow_control::type)serial_params_.flow_control_));//流量控制
        ptr_serial_->set_option(
            serial_port::parity((
            serial_port::parity::type)serial_params_.parity_bits_));//奇偶校验
        ptr_serial_->set_option(
            serial_port::stop_bits((
            serial_port::stop_bits::type)serial_params_.stop_bits_));//停止位
        ptr_serial_->set_option(serial_port::character_size(8));//数据位
    }
    catch (std::exception &e)
    {
        cout << "Failed to open serial port !" << endl;
        cout << "Error Info: " << e.what() << endl;
        return false;
    }

    try
    {
        //线程执行runmain()函数的调用
        thread_ = boost::thread(bind(&SerialPort::runMain, this));
    }
    catch (std::exception &e)
    {
        cout << "Failed to create thread !" << endl;
        cout << "Error Info: " << e.what() << endl;
        return false;
    }

    return true;
}

bool SerialPort::stopThread()
{
    ptr_io_service_->stop();
    return true;
}

void SerialPort::setCallbackFunc(
    const boost::function<void(xm_arm_msgs::xm_ArmSerialDatagramPtr)> &func)
{
    data_callback_func_ = func;
}

bool SerialPort::writeDataGram(
    const xm_arm_msgs::xm_ArmSerialDatagram &datagram)
{
    uint32_t byte_sum = 0;
    const size_t param_len = datagram.data.size();//传输字长


    //数据buffer
    byte_vector buffer_to_send(HEADER_LEN + param_len + 5, 0);//都为零
    buffer_to_send[0] = buffer_to_send[1] = 0x55;//数据协议头,若下位机收到两个0x55则开始接收数据
    buffer_to_send[2] = 0;//数据长度N
    buffer_to_send[3] = 0x03;//指令名 CMD_MULT_SERVO_MOVE
    buffer_to_send[4] = 0x06;//电机个数
    buffer_to_send[5] = 0xFF;//时间,控制速度
    buffer_to_send[6] = 0xFF;


    //把接收到的msg中的数据写到buffer中
    for (size_t i = 0; i < param_len; i++)
        buffer_to_send[6 + i] = datagram.data.at(i);

    for (size_t i = 2; i < buffer_to_send.size() - 1; i++)
        byte_sum += buffer_to_send.at(i);

    buffer_to_send[buffer_to_send.size() - 1] = (u_int8_t)(byte_sum % 255);
    cout << "Sending Bytes: " << endl;

    for (size_t i = 0; i < buffer_to_send.size(); i++)
        printf("%02X ", buffer_to_send.at(i));

    cout << endl;
    return writeRaw(buffer_to_send);
}

bool SerialPort::writeRaw(const byte_vector &raw_data)
{
    mutex::scoped_lock lock(write_queue_mutex_);
    bool empty = write_queue_.empty();
    ptr_byte_vector data(new byte_vector(raw_data));
    write_queue_.push(data);

    if (empty)
        startOneWrite();

    return true;
}

} /* namespace xm_serial_node */
