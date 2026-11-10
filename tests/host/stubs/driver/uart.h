#pragma once
#include <cstddef>
#include <cstdint>
using uart_port_t=int;
constexpr int UART_NUM_2=2, UART_DATA_8_BITS=8, UART_PARITY_EVEN=1,
 UART_STOP_BITS_2=2, UART_HW_FLOWCTRL_DISABLE=0, UART_SCLK_DEFAULT=0,
 UART_PIN_NO_CHANGE=-1, UART_SIGNAL_RXD_INV=1;
struct uart_config_t { int baud_rate,data_bits,parity,stop_bits,flow_ctrl,source_clk; };
inline int uart_param_config(int,const uart_config_t*) { return 0; }
inline int uart_set_pin(int,int,int,int,int) { return 0; }
inline int uart_set_line_inverse(int,int) { return 0; }
inline int uart_driver_install(int,int,int,int,void*,int) { return 0; }
int uart_read_bytes(int,void*,uint32_t,uint32_t);

inline int uart_flush_input(int) { return 0; }
