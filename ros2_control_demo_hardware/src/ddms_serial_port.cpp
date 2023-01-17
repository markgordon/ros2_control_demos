
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>

#include "ros2_control_demo_hardware/ddms_serial_port.hpp"

using namespace ddms_diff;

DDMSSerial::DDMSSerial()
    : serial_port_(-1)
    , rx_frame_length_(0)
    , rx_frame_escape_(false)
    , tx_frame_length_(0)
{

}

DDMSSerial::~DDMSSerial()
{
    close();
}

ddms_diff::return_type DDMSSerial::open(const std::string & port_name)
{
    serial_port_ = ::open(port_name.c_str(), O_RDWR | O_NOCTTY);

    if (serial_port_ < 0) {
        fprintf(stderr, "Failed to open serial port: %s (%d)\n", strerror(errno), errno);
        return return_type::ERROR;
    }

    struct termios tty_config{};
    if (::tcgetattr(serial_port_, &tty_config) != 0) {
        fprintf(stderr, "Failed to get serial port configuration: %s (%d)\n", strerror(errno), errno);
        close();
        return return_type::ERROR;
    }

    memset(&tty_config, 0, sizeof(termios));
    tty_config.c_cflag = B9600 | CRTSCTS | CS8 | CLOCAL | CREAD;
    tty_config.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty_config.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty_config.c_cflag &= ~CSIZE;    // Clear bits per byte
    tty_config.c_cflag |=  CS8;      // 8 bit per byte
    tty_config.c_iflag = IGNPAR;
    tty_config.c_oflag = OPOST;
    tty_config.c_lflag = 0;
    tty_config.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty_config.c_cc[VMIN] = 0;
    tcflush(serial_port_, TCIFLUSH);

    /*
    if (::cfsetispeed(&tty_config, B9600) != 0 || ::cfsetospeed(&tty_config, B9600) != 0) {
        fprintf(stderr, "Failed to set serial port speed: %s (%d)\n", strerror(errno), errno);
        close();
        return return_type::ERROR;
    }
    */

    if (::tcsetattr(serial_port_, TCSANOW, &tty_config) != 0) {
        fprintf(stderr, "Failed to set serial port configuration: %s (%d)\n", strerror(errno), errno);
        close();
        return return_type::ERROR;
    }

    return return_type::SUCCESS;
}

ddms_diff::return_type DDMSSerial::close()
{
    if (is_open()) {
        ::close(serial_port_);
        serial_port_ = -1;
    }
    return return_type::SUCCESS;
}
return_type DDMSSerial::motor_command(uint8_t ID, double commanded_value)
{
    //write velocity to motor
    command_motor command;
    command.ID = ID;
    int16_t integer_velocity = (int)32767/commanded_value;
    command.commanded_state[0] = integer_velocity >>8;
    command.commanded_state[1] = integer_velocity;
    return write_frame((uint8_t *)(&command));
}

return_type DDMSSerial::read_frame(uint8_t * frame)
{
    // Read data from the serial port
    const ssize_t num_bytes = ::read(serial_port_, rx_buffer_, 200);
    if (num_bytes == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("DDMSSerialPort"),"Failed to read serial port data: %s (%d)\n", strerror(errno), errno);
        return return_type::ERROR;
    }
    if(num_bytes >=9){
        memcpy(frame,rx_buffer_,9);
        return return_type::SUCCESS;
    }
    return return_type::ERROR;
}
std::vector<double> DDMSSerial::get_wheel_state(uint8_t ID,double velocity)
{
        //write velocity to motor
        std::vector<double> states;
        return_type retval = motor_command(ID,velocity);
    if(retval == return_type::SUCCESS){
        command_motor_reply reply;
        if(read_frame((uint8_t * )(&reply)) == return_type::SUCCESS){
            //velocity 
            states.push_back(((reply.velocity[0] << 8 )+ reply.velocity[1])/(double)32767);
            //position
            states.push_back((((reply.position[0] << 8) + reply.position[1])/(double)32767 )* 2* M_PI);
        }
    }else{
          RCLCPP_ERROR(rclcpp::get_logger("DDMSSerialPort"), "Failed to read state ID:%i, error %i",ID,(uint)retval);

    }
    return states;
}

ddms_diff::return_type DDMSSerial::write_frame(uint8_t* data)
{
    if (!is_open()) {
        return return_type::ERROR;
    }
    
    // Generate the frame
    uint8_t crc = crc_update(data);
    data[DDMS_SERIAL_SERIAL_FRAME_SIZE -1] = crc;
    if (::write(serial_port_, data, DDMS_SERIAL_SERIAL_FRAME_SIZE) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("DDMSSerialPort"),"Failed to write serial port data: %s (%d)\n", strerror(errno), errno);
        return return_type::ERROR;
    }

    return return_type::SUCCESS;
}

bool DDMSSerial::is_open() const
{
    return serial_port_ >= 0;
}

// CRC-8/MAXIM
uint8_t DDMSSerial::crc_update(const uint8_t* data)
{
    uint8_t crc, i;
    crc = 0x00;
    i=0;
    while(i < DDMS_SERIAL_SERIAL_FRAME_SIZE - 1)//fixed frame size
    {
        crc = crc ^ *data++;
        for(i = 0;i < 9;i++)
        {
            if(crc & 0x01)
            {
                crc = (crc >> 1) ^ 0x8c;
            }
                else crc >>= 1;
        }
    }
    return crc;
}