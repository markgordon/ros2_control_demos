
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
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
    //make it exclusive
    ioctl(serial_port_, TIOCEXCL);
    struct termios tty_config{};
    if (::tcgetattr(serial_port_, &tty_config) != 0) {
        fprintf(stderr, "Failed to get serial port configuration: %s (%d)\n", strerror(errno), errno);
        close();
        return return_type::ERROR;
    }

    memset(&tty_config, 0, sizeof(termios));
    tty_config.c_cflag = B115200 ;//| CRTSCTS | CS8 | CLOCAL | CREAD;
    tty_config.c_cflag &= ~PARENB;//clear parity bit (no parity)
    tty_config.c_cflag &= ~CSTOPB;//Stop bits = 1
    tty_config.c_cflag &= ~CSIZE;//clears the mask
    tty_config.c_cflag |= CS8; //set data bits = 8
    tty_config.c_cflag &= ~CRTSCTS; //turn off hardwar based flow ctrl
    tty_config.c_cflag |= CREAD | CLOCAL;//Turn on the reciever
    tty_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
    tty_config.c_cc[VTIME] = 1;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty_config.c_cc[VMIN] = 0;
    tcflush(serial_port_, TCIFLUSH);

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


ddms_diff::return_type DDMSSerial::get_wheel_state(uint8_t ID,double velocity,std::vector<double> & states)
{
    //write velocity to motor

    return_type retval = motor_command(ID,velocity);

    if(retval == return_type::SUCCESS){
        command_motor_reply reply;

        if((retval = read_frame((uint8_t * )(&reply))) == return_type::SUCCESS){
            //velocity 
            //position
            if(ID==0){
                states.push_back(-((reply.velocity[0] << 8 )+ reply.velocity[1])/(double)32767);
                int pos = 32767 - ((reply.position[0] << 8) + reply.position[1]);
                states.push_back((pos/(double)32767 )* 2* M_PI);
            }else{
                states.push_back(((reply.velocity[0] << 8 )+ reply.velocity[1])/(double)32767);
                states.push_back((((reply.position[0] << 8) + reply.position[1])/(double)32767 )* 2* M_PI);
            }
        }else{
            return retval;
        }
    }else{
        RCLCPP_ERROR(rclcpp::get_logger("DDMSSerialPort"), "Failed to read state ID:%i, error %i",ID,(uint)retval);
        return retval;
    }
    //RCLCPP_INFO(rclcpp::get_logger("DDMSSerialPort"),"command %f %f", states[0],states[1]);

    return ddms_diff::return_type::SUCCESS;
}
return_type DDMSSerial::motor_command(uint8_t ID, double commanded_value)
{
    //write velocity to motor
    command_motor command;
    command.ID = ID;
    int16_t integer_velocity = (int)32767/commanded_value;
    command.commanded_state[0] = integer_velocity >>8;
    command.commanded_state[1] = integer_velocity;
    //RCLCPP_INFO(rclcpp::get_logger("DDMSSerialPort"),"command %d %d %d", command.ID,command.commanded_state[0],command.commanded_state[1]);

    return write_frame((uint8_t *)(&command));
}
return_type DDMSSerial::read_frame(uint8_t * frame)
{
    // Read data from the serial port
    ssize_t num_bytes=0,retval=0;
    uint8_t rx_buffer[11];
    //having some trouble here, dont' want to block but sometimes it doesn't seem to answer, so if we don't get 10 bytes, just dump out after a .2 seconds
    uint tries=0;
    do{
        retval = ::read(serial_port_, rx_buffer, 10-num_bytes);
        if(retval > 0) memcpy(frame + num_bytes,rx_buffer,retval);
        num_bytes+= retval;
        tries++;
    }while(retval > -1 && num_bytes < 10 && tries < 2);

    if (retval == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("DDMSSerialPort"),"Failed to read serial port data: %s (%d)\n", strerror(errno), errno);
        return return_type::ERROR;
    }
    return return_type::SUCCESS;
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
    unsigned crc = 0;
    for (size_t i = 0; i < DDMS_SERIAL_SERIAL_FRAME_SIZE-1; i++) {
        crc ^= data[i];
        for (unsigned k = 0; k < 8; k++)
            crc = crc & 1 ? (crc >> 1) ^ 0x8c : crc >> 1;
    }
    return crc;
}