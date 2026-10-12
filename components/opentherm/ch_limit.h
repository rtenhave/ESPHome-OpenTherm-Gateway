#pragma once

// Pure helpers for clamping OpenTherm CH setpoint frames.
// No Arduino/ESPHome dependencies, so the logic can be unit-tested on a host.

#include <cmath>
#include <cstdint>

namespace esphome
{
  namespace opentherm
  {
    namespace ch_limit
    {
      // OpenTherm frame layout: bit 31 parity, bits 28-30 msg type, bits 16-23 data-id, bits 0-15 data
      constexpr uint8_t MSG_TYPE_WRITE_DATA = 0b001;
      constexpr uint8_t MSG_TYPE_WRITE_ACK = 0b101;
      constexpr uint8_t DATA_ID_TSET = 1;      // Control setpoint (flow temperature)
      constexpr uint8_t DATA_ID_MAX_TSET = 57; // Max CH water setpoint

      // Sanity range for the limit itself; anything outside is treated as "no limit"
      constexpr float MIN_VALID_LIMIT = 10.0f;
      constexpr float MAX_VALID_LIMIT = 90.0f;

      inline uint8_t msg_type(uint32_t frame) { return (frame >> 28) & 0x7; }
      inline uint8_t data_id(uint32_t frame) { return (frame >> 16) & 0xFF; }
      inline uint16_t data(uint32_t frame) { return frame & 0xFFFF; }

      // f8.8 signed fixed point -> float
      inline float f88_to_float(uint16_t u88)
      {
        return static_cast<float>(static_cast<int16_t>(u88)) / 256.0f;
      }

      // float -> f8.8, rounded to the nearest 1/256 °C, clamped to 0..100 like the OpenTherm library
      inline uint16_t float_to_f88(float t)
      {
        if (t < 0.0f)
          t = 0.0f;
        if (t > 100.0f)
          t = 100.0f;
        return static_cast<uint16_t>(std::lround(t * 256.0f));
      }

      // Recompute the even parity bit (bit 31) over bits 0-30
      inline uint32_t with_parity(uint32_t frame)
      {
        frame &= 0x7FFFFFFFu;
        uint32_t v = frame;
        uint8_t ones = 0;
        while (v)
        {
          ones += v & 1u;
          v >>= 1;
        }
        if (ones & 1u)
          frame |= 0x80000000u;
        return frame;
      }

      // Replace the 16 data bits of a frame, keeping type and id, and fix parity
      inline uint32_t replace_data(uint32_t frame, uint16_t new_data)
      {
        return with_parity((frame & 0x7FFF0000u) | new_data);
      }

      inline bool limit_is_valid(float limit)
      {
        return !std::isnan(limit) && limit >= MIN_VALID_LIMIT && limit <= MAX_VALID_LIMIT;
      }

      // True when this is a thermostat WRITE to a data-id we limit (TSet, and optionally MaxTSet)
      inline bool is_limited_write(uint32_t request, bool limit_max_tset)
      {
        if (msg_type(request) != MSG_TYPE_WRITE_DATA)
          return false;
        uint8_t id = data_id(request);
        return id == DATA_ID_TSET || (limit_max_tset && id == DATA_ID_MAX_TSET);
      }

      // Clamp a thermostat WRITE request. Returns the frame to send to the appliance.
      // `clamped` is set when the value was lowered. Frames that are not limited writes,
      // or an invalid limit, pass through unchanged.
      inline uint32_t clamp_request(uint32_t request, float limit, bool limit_max_tset, bool &clamped)
      {
        clamped = false;
        if (!limit_is_valid(limit) || !is_limited_write(request, limit_max_tset))
          return request;

        float requested = f88_to_float(data(request));
        if (std::isnan(requested) || requested <= limit)
          return request;

        clamped = true;
        return replace_data(request, float_to_f88(limit));
      }

      // ---- Manual mode -------------------------------------------------------

      constexpr uint8_t MSG_TYPE_READ_DATA = 0b000;
      constexpr uint8_t DATA_ID_STATUS = 0;
      constexpr uint16_t MASTER_CH_ENABLE = 0x0100; // Status HB bit 0: CH enable

      // Replace the value of a thermostat TSet WRITE with the manual setpoint (capped by limit when valid).
      // `changed` is set when the frame was modified. Other frames, or an invalid manual value, pass through.
      inline uint32_t apply_manual_setpoint(uint32_t request, float manual, float limit, bool &changed)
      {
        changed = false;
        if (!limit_is_valid(manual) || msg_type(request) != MSG_TYPE_WRITE_DATA || data_id(request) != DATA_ID_TSET)
          return request;

        float target = manual;
        if (limit_is_valid(limit) && target > limit)
          target = limit;

        uint16_t new_data = float_to_f88(target);
        if (new_data == data(request))
          return request;

        changed = true;
        return replace_data(request, new_data);
      }

      // Set the master CH-enable flag in a thermostat Status READ. `changed` is set when it was off.
      inline uint32_t force_ch_enable(uint32_t request, bool &changed)
      {
        changed = false;
        if (msg_type(request) != MSG_TYPE_READ_DATA || data_id(request) != DATA_ID_STATUS)
          return request;
        if (data(request) & MASTER_CH_ENABLE)
          return request;

        changed = true;
        return replace_data(request, data(request) | MASTER_CH_ENABLE);
      }

      // The slave echoes the master status in the high byte of its Status response. Put the thermostat's
      // own flags back, so it never sees the CH enable it did not ask for. The slave flags (low byte) stay.
      inline uint32_t restore_master_status_echo(uint32_t response, uint16_t thermostat_data)
      {
        uint16_t new_data = (thermostat_data & 0xFF00) | (data(response) & 0x00FF);
        return replace_data(response, new_data);
      }

    } // namespace ch_limit
  } // namespace opentherm
} // namespace esphome
