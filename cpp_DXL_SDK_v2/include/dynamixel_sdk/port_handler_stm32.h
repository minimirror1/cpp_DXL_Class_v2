/*
 * port_handler_stm32.h
 *
 *  Created on: 2023. 6. 20.
 *      Author: minim
 */

#ifndef INCLUDE_DYNAMIXEL_SDK_PORT_HANDLER_STM32_H_
#define INCLUDE_DYNAMIXEL_SDK_PORT_HANDLER_STM32_H_


#include "port_handler.h"

#include "../../../../cpp_Uart_Class_v2/UART_Class.h"//외부 종속성

namespace dynamixel
{

////////////////////////////////////////////////////////////////////////////////
/// @brief The class for control port in Arduino
////////////////////////////////////////////////////////////////////////////////
class PortHandlerSTM32 : public PortHandler
{
 private:
  int     socket_fd_;
  int     baudrate_;
  char    port_name_[100];

  double  packet_start_time_;
  double  packet_timeout_;
  double  tx_time_per_byte;

#if defined(__OPENCM904__)
  UARTClass *p_dxl_serial;
#endif
  Serial *p_serial;

  bool    setupPort(const int cflag_baud);

  double  getCurrentTime();
  double  getTimeSinceStart();

  int     checkBaudrateAvailable(int baudrate);

  void    setPowerOn();
  void    setPowerOff();
  void    setTxEnable();
  void    setTxDisable();

 public:
  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that initializes instance of PortHandler and gets port_name
  /// @description The function initializes instance of PortHandler and gets port_name.
  ////////////////////////////////////////////////////////////////////////////////
  PortHandlerSTM32(const char *port_name, Serial *serial_port);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that closes the port
  /// @description The function calls PortHandlerSTM32::closePort() to close the port.
  ////////////////////////////////////////////////////////////////////////////////
  virtual ~PortHandlerSTM32() { closePort(); }

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that opens the port
  /// @description The function calls PortHandlerSTM32::setBaudRate() to open the port.
  /// @return communication results which come from PortHandlerSTM32::setBaudRate()
  ////////////////////////////////////////////////////////////////////////////////
  bool    openPort();

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that closes the port
  /// @description The function closes the port.
  ////////////////////////////////////////////////////////////////////////////////
  void    closePort();

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that clears the port
  /// @description The function clears the port.
  ////////////////////////////////////////////////////////////////////////////////
  void    clearPort();

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that sets port name into the port handler
  /// @description The function sets port name into the port handler.
  /// @param port_name Port name
  ////////////////////////////////////////////////////////////////////////////////
  void    setPortName(const char *port_name);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that returns port name set into the port handler
  /// @description The function returns current port name set into the port handler.
  /// @return Port name
  ////////////////////////////////////////////////////////////////////////////////
  char   *getPortName();

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that sets baudrate into the port handler
  /// @description The function sets baudrate into the port handler.
  /// @param baudrate Baudrate
  /// @return false
  /// @return   when error was occurred during port opening
  /// @return or true
  ////////////////////////////////////////////////////////////////////////////////
  bool    setBaudRate(const int baudrate);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that returns current baudrate set into the port handler
  /// @description The function returns current baudrate set into the port handler.
  /// @return Baudrate
  ////////////////////////////////////////////////////////////////////////////////
  int     getBaudRate();

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that checks how much bytes are able to be read from the port buffer
  /// @description The function checks how much bytes are able to be read from the port buffer
  /// @description and returns the number.
  /// @return Length of read-able bytes in the port buffer
  ////////////////////////////////////////////////////////////////////////////////
  int     getBytesAvailable();

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that reads bytes from the port buffer
  /// @description The function gets bytes from the port buffer,
  /// @description and returns a number of bytes read.
  /// @param packet Buffer for the packet received
  /// @param length Length of the buffer for read
  /// @return -1
  /// @return   when error was occurred
  /// @return or Length of bytes read
  ////////////////////////////////////////////////////////////////////////////////
  int     readPort(uint8_t *packet, int length);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that writes bytes on the port buffer
  /// @description The function writes bytes on the port buffer,
  /// @description and returns a number of bytes which are successfully written.
  /// @param packet Buffer which would be written on the port buffer
  /// @param length Length of the buffer for write
  /// @return -1
  /// @return   when error was occurred
  /// @return or Length of bytes written
  ////////////////////////////////////////////////////////////////////////////////
  int     writePort(uint8_t *packet, int length);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that sets and starts stopwatch for watching packet timeout
  /// @description The function sets the stopwatch by getting current time and the time of packet timeout with packet_length.
  /// @param packet_length Length of the packet expected to be received
  ////////////////////////////////////////////////////////////////////////////////
  void    setPacketTimeout(uint16_t packet_length);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that sets and starts stopwatch for watching packet timeout
  /// @description The function sets the stopwatch by getting current time and the time of packet timeout with msec.
  /// @param packet_length Length of the packet expected to be received
  ////////////////////////////////////////////////////////////////////////////////
  void    setPacketTimeout(double msec);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that checks whether packet timeout is occurred
  /// @description The function checks whether current time is passed by the time of packet timeout from the time set by PortHandlerSTM32::setPacketTimeout().
  ////////////////////////////////////////////////////////////////////////////////
  bool    isPacketTimeout();
};

}

#endif /* INCLUDE_DYNAMIXEL_SDK_PORT_HANDLER_STM32_H_ */
