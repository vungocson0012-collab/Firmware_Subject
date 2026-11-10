#include <array>
#include <cassert>
#include <cstdio>
#include <deque>
#include <string>
#include "../../src/rc_sbus.cpp"
uint32_t mock_ms=10000;
static std::deque<uint8_t> input;
int uart_read_bytes(int,void* output,uint32_t,uint32_t) {
  if(input.empty())return 0;
  *static_cast<uint8_t*>(output)=input.front();input.pop_front();return 1;
}
void append(const std::array<int,16>& channels,uint8_t flags=0) {
  std::array<uint8_t,25> frame{};frame[0]=0x0f;frame[23]=flags;
  for(int c=0;c<16;++c) for(int b=0;b<11;++b)
    if(channels[c] & (1<<b))frame[1+(c*11+b)/8] |= 1<<((c*11+b)%8);
  for(auto byte:frame)input.push_back(byte);
}
int main(int argc,char**argv) {
  assert(argc==2);std::string test=argv[1];rc_init();
  std::array<int,16> neutral;neutral.fill(992);
  if(test=="normal") {
    append(neutral);assert(rc_update());assert(rc_get_channel(1)==1500);
    mock_ms+=201;assert(!rc_update());
  } else if(test=="startup") {
    mock_ms=0;assert(!rc_update());
  } else if(test=="flags") {
    append(neutral);assert(rc_update());
    for(int i=0;i<100;++i) { append(neutral,0x08);assert(!rc_update());mock_ms+=10; }
    append(neutral,0x04);assert(!rc_update());
    append(neutral);assert(rc_update());
  } else if(test=="lost_frame") {
    append(neutral);assert(rc_update());auto timestamp=rc_last_valid_frame_ms();
    for(int i=0;i<20;++i) { mock_ms+=10;append(neutral,0x04);assert(rc_update()); }
    mock_ms+=1;append(neutral,0x04);assert(!rc_update());assert(rc_last_valid_frame_ms()==timestamp);
  } else if(test=="payload_header") {
    std::array<int,16> tricky={1347,1412,1695,495,1137,732,1553,1244,1151,1642,385,199,1790,1536,707,1201};
    for(int i=0;i<100;++i) { append(tricky);assert(rc_update());mock_ms+=10; }
    assert(sbus_raw_ch[1]==1347 && sbus_raw_ch[16]==1201);
  } else if(test=="partial") {
    append(neutral);auto suffix=input;input.clear();
    for(int i=0;i<12;++i) { input.push_back(suffix.front());suffix.pop_front(); }
    assert(!rc_update());mock_ms+=2;input=suffix;assert(rc_update());
  } else if(test=="partial_timeout") {
    input={0x0f,1,2};assert(!rc_update());mock_ms+=11;
    append(neutral);assert(rc_update());
  } else if(test=="bad_footer") {
    append(neutral);input.back()=0x99;append(neutral);assert(rc_update());
  } else if(test=="noise") {
    for(int i=0;i<30;++i)input.push_back(0x55);
    append(neutral);assert(rc_update());
  } else if(test=="wrap") {
    mock_ms=UINT32_MAX-100;append(neutral);assert(rc_update());mock_ms+=201;assert(!rc_update());
  } else if(test=="bounds") {
    neutral[0]=0;neutral[1]=2047;append(neutral);assert(rc_update());
    assert(rc_get_channel(1)==1000 && rc_get_channel(2)==2000);
  } else return 2;
  printf("PASS SBUS %s\n",test.c_str());
}
