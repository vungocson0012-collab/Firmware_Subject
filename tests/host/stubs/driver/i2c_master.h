#pragma once
#include <cstddef>
#include <cstdint>
using i2c_master_bus_handle_t=void*;
using i2c_master_dev_handle_t=void*;
using gpio_num_t=int;
constexpr int I2C_NUM_0=0,I2C_CLK_SRC_DEFAULT=0,I2C_ADDR_BIT_LEN_7=7;
struct i2c_master_bus_config_t { int i2c_port,sda_io_num,scl_io_num,clk_source,glitch_ignore_cnt; struct { bool enable_internal_pullup; } flags; };
struct i2c_device_config_t { int dev_addr_length,device_address,scl_speed_hz; };
inline int i2c_new_master_bus(const i2c_master_bus_config_t*,i2c_master_bus_handle_t*) { return 0; }
inline int i2c_master_bus_add_device(i2c_master_bus_handle_t,const i2c_device_config_t*,i2c_master_dev_handle_t*) { return 0; }
inline int i2c_master_transmit(i2c_master_dev_handle_t,const uint8_t*,size_t,int) { return 0; }
int i2c_master_transmit_receive(i2c_master_dev_handle_t,const uint8_t*,size_t,uint8_t*,size_t,int);
