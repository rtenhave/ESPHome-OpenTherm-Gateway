// Host unit test for components/opentherm/ch_limit.h
// Build: g++ -std=c++17 -I../../components/opentherm ch_limit_test.cpp -o t && ./t
#include "ch_limit.h"
#include <cstdio>
#include <cstdlib>

using namespace esphome::opentherm::ch_limit;

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } } while (0)

// Reference implementation copied from OpenTherm Library 1.1.4 (buildRequest/parity)
static bool lib_parity(unsigned long frame) { int p = 0; while (frame > 0) { if (frame & 1) p++; frame >>= 1; } return p & 1; }
static uint32_t lib_build(uint8_t type, uint8_t id, uint16_t data) {
  unsigned long r = data; r |= ((unsigned long)type) << 28; r |= ((unsigned long)id) << 16;
  if (lib_parity(r)) r |= (1ul << 31);
  return r;
}
static bool lib_valid_parity(uint32_t f) { return !lib_parity(f); }

int main() {
  bool clamped;

  // 1. Above limit -> clamped to limit, valid parity, same type/id
  uint32_t req = lib_build(MSG_TYPE_WRITE_DATA, DATA_ID_TSET, float_to_f88(55.0f));
  uint32_t out = clamp_request(req, 40.0f, false, clamped);
  CHECK(clamped);
  CHECK(f88_to_float(data(out)) == 40.0f);
  CHECK(data_id(out) == DATA_ID_TSET);
  CHECK(msg_type(out) == MSG_TYPE_WRITE_DATA);
  CHECK(lib_valid_parity(out));
  CHECK(out == lib_build(MSG_TYPE_WRITE_DATA, DATA_ID_TSET, float_to_f88(40.0f)));

  // 2. Below / equal limit -> untouched
  req = lib_build(MSG_TYPE_WRITE_DATA, DATA_ID_TSET, float_to_f88(35.5f));
  CHECK(clamp_request(req, 40.0f, false, clamped) == req && !clamped);
  req = lib_build(MSG_TYPE_WRITE_DATA, DATA_ID_TSET, float_to_f88(40.0f));
  CHECK(clamp_request(req, 40.0f, false, clamped) == req && !clamped);

  // 3. Zero (heating off) passes
  req = lib_build(MSG_TYPE_WRITE_DATA, DATA_ID_TSET, 0);
  CHECK(clamp_request(req, 40.0f, false, clamped) == req && !clamped);

  // 4. READ of TSet untouched
  req = lib_build(0 /*READ_DATA*/, DATA_ID_TSET, 0);
  CHECK(clamp_request(req, 40.0f, false, clamped) == req && !clamped);

  // 5. Other IDs untouched (e.g. DHW setpoint 56 at 60 °C)
  req = lib_build(MSG_TYPE_WRITE_DATA, 56, float_to_f88(60.0f));
  CHECK(clamp_request(req, 40.0f, true, clamped) == req && !clamped);

  // 6. MaxTSet (57) only clamped when enabled
  req = lib_build(MSG_TYPE_WRITE_DATA, DATA_ID_MAX_TSET, float_to_f88(80.0f));
  CHECK(clamp_request(req, 45.0f, false, clamped) == req && !clamped);
  out = clamp_request(req, 45.0f, true, clamped);
  CHECK(clamped && f88_to_float(data(out)) == 45.0f && lib_valid_parity(out) && data_id(out) == 57);

  // 7. Invalid limits -> pass-through (fail open to thermostat behaviour)
  req = lib_build(MSG_TYPE_WRITE_DATA, DATA_ID_TSET, float_to_f88(55.0f));
  CHECK(clamp_request(req, NAN, false, clamped) == req && !clamped);
  CHECK(clamp_request(req, 5.0f, false, clamped) == req && !clamped);
  CHECK(clamp_request(req, 95.0f, false, clamped) == req && !clamped);

  // 8. Fractional limit
  out = clamp_request(req, 42.5f, false, clamped);
  CHECK(clamped && f88_to_float(data(out)) == 42.5f && lib_valid_parity(out));

  // 9. Exhaustive parity check for all data values on TSet
  for (uint32_t d = 0; d <= 0xFFFF; d++) {
    uint32_t f = lib_build(MSG_TYPE_WRITE_DATA, DATA_ID_TSET, 0);
    if (replace_data(f, (uint16_t)d) != lib_build(MSG_TYPE_WRITE_DATA, DATA_ID_TSET, (uint16_t)d)) { failures++; std::printf("parity mismatch %u\n", d); break; }
  }

  // 10. ACK rebuild for masking (type WRITE_ACK keeps original data)
  uint32_t ack = lib_build(MSG_TYPE_WRITE_ACK, DATA_ID_TSET, float_to_f88(40.0f));
  uint32_t masked = replace_data(ack, float_to_f88(55.0f));
  CHECK(masked == lib_build(MSG_TYPE_WRITE_ACK, DATA_ID_TSET, float_to_f88(55.0f)));

  std::printf(failures ? "%d FAILURE(S)\n" : "all ch_limit tests passed\n", failures);
  return failures ? 1 : 0;
}
