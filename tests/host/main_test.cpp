#include <cassert>
#include <string>
#include <vector>
#include "../../src/main.cpp"
uint32_t mock_ms=10000;
static bool deny=false,receiver_ok=true,baro_new=true;
static uint32_t frame_time=9900;
static std::vector<std::string> calls;
static int mutex_count=0,task_count=0;
struct StepDone {};
int xSemaphoreTake(SemaphoreHandle_t,TickType_t) { return !deny; }
int xSemaphoreGive(SemaphoreHandle_t) { return pdTRUE; }
SemaphoreHandle_t xSemaphoreCreateMutex() { ++mutex_count;return &mutex_count; }
int xTaskCreatePinnedToCore(void (*fn)(void*),const char*name,uint32_t stack,void*,unsigned priority,void*,int core) {
  assert(mutex_count==3 && mtx_rc && mtx_baro && mtx_tlm);
  if(std::string(name)=="RC")assert(fn==rc_task && stack==3072 && priority==4 && core==0);
  else if(std::string(name)=="BARO")assert(fn==baro_task && stack==4096 && priority==2 && core==0);
  else { assert(std::string(name)=="CTRL");assert(fn==control_task && stack==8192 && priority==5 && core==1); }
  ++task_count;return pdPASS;
}
void vTaskDelayUntil(TickType_t*,TickType_t t) { mock_ms+=t;throw StepDone{}; }
void esc_init() { calls.push_back("esc"); }
void esc_write(int a,int b,int c,int d) { assert(a==ESC_IDLE && b==a && c==a && d==a);calls.push_back("idle"); }
void imu_init_bus() { calls.push_back("imu_bus"); }
bool imu_is_present() { return true; }
void imu_init() { calls.push_back("imu_calibration"); }
bool baro_init() { calls.push_back("baro_calibration");return true; }
void alt_kf_init() { calls.push_back("kalman"); }
void rc_init() { calls.push_back("rc"); }
bool rc_update() { return receiver_ok; }
uint32_t rc_last_valid_frame_ms() { return frame_time; }
int rc_get_channel(int index) { return 1000+index; }
bool baro_update() { return baro_new; }
float baro_get_altitude_m() { return 2.5f; }
void control_task(void*) {}
void run_one(void(*task)(void*)) { try { task(nullptr); } catch(const StepDone&) {} }
int main(int argc,char**argv) {
  assert(argc==2);const std::string test=argv[1];
  if(test=="startup_order") {
    app_main();
    const std::vector<std::string> expected={"esc","idle","imu_bus","imu_calibration","baro_calibration","kalman","rc"};
    assert(calls==expected && task_count==3);
  } else if(test=="rc_publish") {
    run_one(rc_task);assert(shared_rc_ok && shared_rc_frame_ms==9900 && shared_rc_ch[16]==1016);
    receiver_ok=false;run_one(rc_task);assert(!shared_rc_ok && shared_rc_frame_ms==9900);
    receiver_ok=true;frame_time=mock_ms;deny=true;run_one(rc_task);assert(!shared_rc_ok && shared_rc_frame_ms==9900);
  } else if(test=="baro_publish") {
    sensor_present.baro=true;run_one(baro_task);
    assert(shared_baro_valid && shared_baro_alt_m==2.5f && shared_baro_sample_ms==10000);
    baro_new=false;run_one(baro_task);assert(shared_baro_sample_ms==10000);
    baro_new=true;deny=true;run_one(baro_task);assert(shared_baro_sample_ms==10000);
  } else return 2;
  printf("PASS TASKS %s\n",test.c_str());
}
