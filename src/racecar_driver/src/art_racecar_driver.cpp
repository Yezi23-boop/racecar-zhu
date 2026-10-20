#include <fcntl.h>
#include <iostream>
#include <string.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <cstdint>
// Global variables for serial port
static int serial_fd = -1;

extern "C" {

// Initialize serial port for racecar
int art_racecar_init(int speed, char *dev) {
  if (serial_fd >= 0) {
    close(serial_fd);
  }

  serial_fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_fd < 0) {
    std::cerr << "Failed to open serial port: " << dev << std::endl;
    return -1;
  }

  // Set non-blocking mode
  fcntl(serial_fd, F_SETFL, 0);

  // Configure serial port
  struct termios options;
  tcgetattr(serial_fd, &options);

  // Set baud rate
  speed_t baud;
  switch (speed) {
  case 9600:
    baud = B9600;
    break;
  case 19200:
    baud = B19200;
    break;
  case 38400:
    baud = B38400;
    break;
  case 57600:
    baud = B57600;
    break;
  case 115200:
    baud = B115200;
    break;
  default:
    baud = B38400;
    break;
  }

  cfsetispeed(&options, baud);
  cfsetospeed(&options, baud);

  // 8N1 (8 bits, no parity, 1 stop bit)
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  // Disable hardware flow control
  options.c_cflag &= ~CRTSCTS;

  // Enable receiver, ignore control lines
  options.c_cflag |= (CREAD | CLOCAL);

  // Raw input
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  // Raw output
  options.c_oflag &= ~OPOST;

  tcsetattr(serial_fd, TCSANOW, &options);

  std::cout << "Serial port initialized: " << dev << " at " << speed << " baud"
            << std::endl;
  return 0;
}

// Send command to motor and servo
unsigned char send_cmd(uint16_t motor_pwm, uint16_t servo_pwm) {
  if (serial_fd < 0) {
    std::cerr << "Serial port not initialized" << std::endl;
    return 1;
  }

  // Create command packet in MCU-expected format (7 bytes):
  // [0]=0xAA (header)
  // [1]=motor low, [2]=motor high
  // [3]=servo low, [4]=servo high
  // [5]=checksum (sum of bytes 1..4 mod 256)
  // [6]=0x55 (tail)
  unsigned char cmd[7] = {0};
  cmd[0] = 0xAA;
  cmd[1] = motor_pwm & 0xFF;        // motor low
  cmd[2] = (motor_pwm >> 8) & 0xFF; // motor high
  cmd[3] = servo_pwm & 0xFF;        // servo low
  cmd[4] = (servo_pwm >> 8) & 0xFF; // servo high

  // Calculate checksum as sum of bytes 1..4 (matching MCU code)
  unsigned int sum = 0;
  for (int i = 1; i <= 4; ++i) {
    sum += cmd[i];
  }
  cmd[5] = sum & 0xFF;
  cmd[6] = 0x55;

  // Send command
  ssize_t written = write(serial_fd, cmd, 7);
  if (written != 7) {
    std::cerr << "Failed to send command, wrote=" << written << std::endl;
    return 1;
  }

  return 0;
}

} // extern "C"
