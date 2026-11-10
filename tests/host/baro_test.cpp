#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include "../../src/baro_bmp388.cpp"
uint32_t mock_ms=10000;
static bool data_ready=false,bus_fail=false;
int i2c_master_transmit_receive(i2c_master_dev_handle_t,const uint8_t*tx,size_t,uint8_t*rx,size_t n,int) {
  if(bus_fail)return 1;
  std::memset(rx,0,n);
  if(*tx==BMP388_REG_CHIP_ID)rx[0]=BMP388_CHIP_ID_VALUE;
  if(*tx==BMP388_REG_CALIB) {
    rx[6]=rx[8]=0x40; // p1=p2=16384 -> scaled coefficients zero
    rx[11]=12666&255;rx[12]=12666>>8; // pressure constant 101328 Pa
  }
  if(*tx==BMP388_REG_STATUS) {
    rx[0]=BMP388_STATUS_CMD_RDY | (data_ready?BMP388_STATUS_DRDY_PRESS:0);
    if(n==7) { rx[1]=1;rx[4]=1; }
  }
  return 0;
}
int main(int argc,char**argv) {
  assert(argc==2);std::string test=argv[1];
  if(test=="no_samples") { assert(!baro_init());assert(!baro_ready); }
  else {
    data_ready=true;assert(baro_init());assert(baro_update());
    assert(std::isfinite(baro_get_altitude_m()) && fabsf(baro_get_altitude_m())<0.001f);
    float before=baro_get_altitude_m();
    if(test=="bus_error") { bus_fail=true;assert(!baro_update()); }
    else if(test=="invalid_pressure") { calib.p5=-1;assert(!baro_update()); }
    else if(test=="nan") { calib.p5=std::numeric_limits<double>::quiet_NaN();assert(!baro_update()); }
    else if(test!="normal")return 2;
    assert(baro_get_altitude_m()==before);
  }
  printf("PASS BARO %s\n",test.c_str());
}
