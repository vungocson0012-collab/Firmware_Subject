#include <cassert>
#include <cstdio>
#include <limits>
#include <string>
#include "../../src/control_task.cpp"
uint32_t mock_ms=10000;
static bool deny_rc=false, deny_baro=false, rc_producer=true, baro_producer=true, imu_valid=true, imu_nan=false;
static int motor[4];
struct StepDone {};
int xSemaphoreTake(SemaphoreHandle_t m, TickType_t) {
  return !((deny_rc && m==mtx_rc) || (deny_baro && m==mtx_baro));
}
int xSemaphoreGive(SemaphoreHandle_t) { return pdTRUE; }
void vTaskDelayUntil(TickType_t*,TickType_t t) { mock_ms+=t;throw StepDone{}; }
bool imu_update() { return imu_valid; }
float imu_get_acc_x_g() { return 0; }
float imu_get_acc_y_g() { return 0; }
float imu_get_acc_z_g() { return 1; }
float imu_get_rate_roll_dps() { return 0; }
float imu_get_rate_pitch_dps() { return 0; }
float imu_get_rate_yaw_dps() { return 0; }
float imu_get_roll_deg() { return imu_nan ? std::numeric_limits<float>::quiet_NaN() : 0; }
float imu_get_pitch_deg() { return 0; }
void esc_write(int a,int b,int c,int d) {
  motor[0]=a;motor[1]=b;motor[2]=c;motor[3]=d;
  for(int m:motor)assert(m>=ESC_IDLE && m<=ESC_MAX);
}
void step() {
  if(rc_producer && shared_rc_ok)shared_rc_frame_ms=mock_ms;
  if(baro_producer && shared_baro_valid)shared_baro_sample_ms=mock_ms;
  try { control_task(nullptr); } catch(const StepDone&) {}
}
void arm_normally() { step();shared_rc_ch[CH_ARM]=2000;step();assert(armed); }
int main(int argc,char**argv) {
  assert(argc==2);const std::string test=argv[1];
  mtx_rc=(void*)1;mtx_baro=(void*)2;mtx_tlm=(void*)3;
  sensor_present={true,true};shared_rc_ok=true;shared_baro_valid=true;
  for(int i=1;i<=16;++i)shared_rc_ch[i]=1500;
  shared_rc_ch[CH_THROTTLE]=1000;shared_rc_ch[CH_ARM]=1000;shared_rc_ch[CH_MODE]=1000;
  alt_kf_init();pid_reset_all();
  if(test=="normal") {
    shared_rc_ch[CH_ARM]=2000;step();assert(!armed);
    shared_rc_ch[CH_ARM]=1000;arm_normally();
    shared_rc_ch[CH_ARM]=1000;step();assert(!armed && motor[0]==800);
  } else if(test=="mutex") {
    arm_normally();deny_rc=true;step();assert(armed && rc_ch[CH_ARM]==2 && motor[0]==900);
    deny_rc=false;step();assert(armed);
  } else if(test=="alt_without_baro") {
    sensor_present.baro=false;arm_normally();shared_rc_ch[CH_MODE]=2000;step();
    assert(armed && flight_mode==MODE_ANGLE);
  } else if(test=="mode_transition" || test=="failsafe_transition" || test=="high_throttle_transition") {
    arm_normally();shared_rc_ch[CH_THROTTLE]=test=="high_throttle_transition"?2000:1500;
    for(int i=0;i<1000;++i)step();
    int before=motor[0];
    if(test=="mode_transition")shared_rc_ch[CH_MODE]=2000;
    else shared_rc_ok=false;
    step();assert(armed && std::abs(motor[0]-before)<=1);
    int previous=motor[0];
    for(int i=0;i<100;++i) { step();assert(std::abs(motor[0]-previous)<=1);previous=motor[0]; }
  } else if(test=="landing_timer" || test=="landing_wrap") {
    arm_normally();shared_rc_ok=false;
    if(test=="landing_wrap")mock_ms=UINT32_MAX-500;
    step();
    for(int i=0;i<1100;++i) { altitude_throttle=1200;step();assert(armed && !landing_candidate); }
    altitude_throttle=900;step();assert(armed && landing_candidate);
    for(int i=0;i<500;++i)step();assert(armed);
    altitude_throttle=1200;step();assert(armed && !landing_candidate);
    altitude_throttle=900;step();assert(armed && landing_candidate);
    while(mock_ms-landing_since_ms<LANDING_CONFIRM_MS)step();
    assert(armed);step();assert(!armed && motor[0]==800);
  } else if(test=="stale_baro" || test=="baro_mutex_timeout") {
    arm_normally();shared_rc_ch[CH_THROTTLE]=1500;shared_rc_ch[CH_MODE]=2000;step();
    assert(flight_mode==MODE_ALT_HOLD);
    if(test=="baro_mutex_timeout")deny_baro=true;else baro_producer=false;
    for(int i=0;i<150;++i)step();
    assert(armed && !baro_healthy && flight_mode==MODE_ANGLE);
  } else if(test=="rc_stall" || test=="rc_mutex_timeout" || test=="rc_wrap") {
    if(test=="rc_wrap")mock_ms=UINT32_MAX-100;
    arm_normally();shared_rc_ch[CH_THROTTLE]=1500;step();rc_producer=false;
    if(test=="rc_mutex_timeout")deny_rc=true;
    for(int i=0;i<150;++i)step();
    assert(!rc_ok && armed && flight_mode==MODE_FAILSAFE);
  } else if(test=="no_rc_no_baro") {
    arm_normally();shared_rc_ok=false;shared_baro_valid=false;step();
    assert(!armed && motor[0]==800);
  } else if(test=="imu_invalid" || test=="imu_nan") {
    arm_normally();if(test=="imu_invalid")imu_valid=false;else imu_nan=true;
    step();assert(!armed && motor[0]==800);
    imu_valid=true;imu_nan=false;step();assert(!armed);
    shared_rc_ch[CH_ARM]=1000;arm_normally();
  } else if(test=="baro_nan") {
    arm_normally();shared_rc_ch[CH_MODE]=2000;
    shared_baro_alt_m=std::numeric_limits<float>::quiet_NaN();step();
    assert(!baro_healthy && flight_mode==MODE_ANGLE && std::isfinite(alt_kf_get_altitude_cm()));
  } else return 2;
  printf("PASS CTRL %s\n",test.c_str());
}
