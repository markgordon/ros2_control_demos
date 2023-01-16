
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>

#include "ddms_serial_port.hpp"

using namespace ddms_diff;

ddsm_diff::DDMSSerial::DDMSSerial()
    : serial_port_(-1)
    , rx_frame_length_(0)
    , rx_frame_crc_(HDLC_CRC_INIT_VALUE)
    , rx_frame_escape_(false)
    , tx_frame_length_(0)
    , tx_frame_crc_(HDLC_CRC_INIT_VALUE)
{

}

ddms_diff::DDMSSerial::~DDMSSerial()
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

ddms_diff::return_type DDMSSerial::read_frames(uint8_t * frame)
{
    // Read data from the serial port
    const ssize_t num_bytes = ::read(serial_port_, rx_buffer_, 256);
    if (num_bytes == -1) {
        fprintf(stderr, "Failed to read serial port data: %s (%d)\n", strerror(errno), errno);
        return return_type::ERROR;
    }
    return return_type::SUCCESS;
}

ddms_diff::return_type DDMSSerial::write_frame(const uint8_t* data)
{
    if (!is_open()) {
        return return_type::ERROR;
    }
    
    // Generate the frame
    uint8_t crc = crc_update(data);
    data[DDMS_SERIAL_SERIAL_FRAME_SIZE -1] = crc;
    if (::write(serial_port_, data, DDMS_SERIAL_SERIAL_FRAME_SIZE) == -1) {
        fprintf(stderr, "Failed to write serial port data: %s (%d)\n", strerror(errno), errno);
        return return_type::ERROR;
    }

    return return_type::SUCCESS;
}

bool DDMSSerial::is_open() const
{
    return serial_port_ >= 0;
}

void DDMSSerial::encode_byte(uint8_t data)
{

}

void DDMSSerial::decode_byte(uint8_t data, std::vector<SerialHdlcFrame>& frames)
{

}
// CRC-8/MAXIM
uint8_t DDMSSerial::crc_update(uint8_t* data)
{
    uint8_t crc, i;
    crc = 0x00;

    while(i < DDMS_SERIAL_SERIAL_FRAME_SIZE - 1)) //fixed frame size
    {
        crc ^ = *data++;
        for(i = 0;i < 8;i++)
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