#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>
#include "../../src/pid.cpp"
int main(int argc,char**argv) {
  assert(argc==2);std::string test=argv[1];
  if(test=="reset_no_kick") {
    pid_rate_roll(0,100);pid_reset_all();
    float out=pid_rate_roll(0,10);
    assert(fabsf(out-(KP_RATE_ROLL*10+KI_RATE_ROLL*10*DT_CTRL))<0.001f);
  } else if(test=="climb_seed") {
    pid_reset_all();pid_climb_seed(400,0,-60);
    assert(fabsf(pid_climb(0,-60)-400)<0.001f);
    for(int i=0;i<10000;++i)assert(std::isfinite(pid_climb(-100,-60)));
  } else return 2;
  printf("PASS PID %s\n",test.c_str());
}
