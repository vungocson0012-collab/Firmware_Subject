#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include "../../src/imu_icm20602.cpp"
uint32_t mock_ms=10000;
static bool zeros=false,bus_fail=false,noise=false,acc_noise=false;
static bool chip_alive=false;
static int gyro_raw=164,reads=0;
int spi_device_polling_transmit(spi_device_handle_t,spi_transaction_t*t) {
  if(bus_fail)return 1;
  if(!t->rx_buffer)return 0;
  auto tx=static_cast<const uint8_t*>(t->tx_buffer);
  auto rx=static_cast<uint8_t*>(t->rx_buffer);std::memset(rx,0,t->length/8);
  if(chip_alive && (tx[0]&0x7f)==REG_WHO_AM_I)rx[1]=ICM20602_WHO_AM_I_VALUE;
  if(!zeros && (tx[0]&0x7f)==REG_ACCEL_XOUT_H) {
    int z=2048+(acc_noise ? (reads%2?102:-102):0);
    rx[5]=(z>>8)&255;rx[6]=z&255;
  }
  if(!zeros && (tx[0]&0x7f)==REG_GYRO_XOUT_H) {
    int raw=noise?(reads%2?16:-16):gyro_raw;
    rx[1]=(raw>>8)&255;rx[2]=raw&255;++reads;
  }
  return 0;
}
int main(int argc,char**argv) {
  assert(argc==2);std::string test=argv[1];
  if(test=="moving_calibration") {
    float mean[3];assert(!imu_collect_gyro_bias(mean));assert(mock_ms==11000);
  } else if(test=="stationary_calibration") {
    gyro_raw=8;imu_calibrate_gyro();
    assert(mock_ms>=12000 && fabsf(gyro_bias_roll_dps-8/16.4f)<0.001f);
  } else if(test=="noisy_calibration") {
    noise=true;float mean[3];assert(!imu_collect_gyro_bias(mean));
  } else if(test=="accel_motion") {
    gyro_raw=0;acc_noise=true;float mean[3];assert(!imu_collect_gyro_bias(mean));
  } else if(test=="zero_sample" || test=="bus_error") {
    assert(imu_update());float before=imu_get_roll_deg();
    if(test=="zero_sample")zeros=true;else bus_fail=true;
    assert(!imu_update());assert(imu_get_roll_deg()==before && std::isfinite(before));
    zeros=false;bus_fail=false;assert(imu_update());assert(std::isfinite(imu_get_roll_deg()));
  } else if(test=="low_g") {
    zeros=true;chip_alive=true;assert(imu_update());
    assert(!accel_angle_valid && std::isfinite(imu_get_roll_deg()));
    zeros=false;assert(imu_update());assert(accel_angle_valid);
  } else if(test=="recover_filter") {
    att_roll_deg=std::numeric_limits<float>::quiet_NaN();assert(imu_update());
    assert(std::isfinite(imu_get_roll_deg()));
  } else return 2;
  printf("PASS IMU %s\n",test.c_str());
}
