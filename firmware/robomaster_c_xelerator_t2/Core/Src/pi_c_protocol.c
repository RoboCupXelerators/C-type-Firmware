#include "pi_c_protocol.h"
#include <math.h>
#include <string.h>

uint16_t pi_c_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
            crc &= 0xFFFF;
        }
    }
    return crc;
}

void pi_c_parser_init(pi_c_parser_t *p) {
    memset(p, 0, sizeof(*p));
}

int pi_c_parser_feed(pi_c_parser_t *p, const uint8_t *data, size_t len, pi_c_frame_t *out_frame) {
    /* Append data to buffer */
    for (size_t i = 0; i < len; i++) {
        if (p->buf_len >= PI_C_MAX_FRAME_SIZE) {
            /* Buffer full, discard oldest byte */
            memmove(p->buf, p->buf + 1, PI_C_MAX_FRAME_SIZE - 1);
            p->buf_len--;
            p->discard_count++;
        }
        p->buf[p->buf_len++] = data[i];
    }

    /* Try to parse a complete frame */
    while (p->buf_len >= 9) {  /* minimum frame: header(2)+body(4)+crc(2)+tail(1) = 9 */
        /* Find header */
        uint16_t hdr_idx = 0;
        int found = 0;
        for (hdr_idx = 0; hdr_idx + 1 < p->buf_len; hdr_idx++) {
            if (p->buf[hdr_idx] == PI_C_FRAME_HEADER_0 &&
                p->buf[hdr_idx + 1] == PI_C_FRAME_HEADER_1) {
                found = 1;
                break;
            }
        }
        if (!found) {
            if (p->buf[p->buf_len - 1] == PI_C_FRAME_HEADER_0) {
                p->buf[0] = PI_C_FRAME_HEADER_0;
                p->buf_len = 1;
            } else {
                p->discard_count += p->buf_len;
                p->buf_len = 0;
            }
            return 0;
        }

        /* Discard bytes before header */
        if (hdr_idx > 0) {
            memmove(p->buf, p->buf + hdr_idx, p->buf_len - hdr_idx);
            p->buf_len -= hdr_idx;
            p->discard_count += hdr_idx;
        }

        if (p->buf_len < 9) return 0;

        uint8_t version     = p->buf[2];
        uint8_t msg_type    = p->buf[3];
        uint8_t seq         = p->buf[4];
        uint8_t payload_len = p->buf[5];

        if (payload_len > PI_C_MAX_PAYLOAD) {
            memmove(p->buf, p->buf + 1, p->buf_len - 1);
            p->buf_len--;
            p->discard_count++;
            continue;
        }

        uint16_t total = 2 + 4 + payload_len + 2 + 1;
        if (p->buf_len < total) return 0;

        /* Verify CRC */
        uint16_t crc_calc = pi_c_crc16(p->buf, 2 + 4 + payload_len);
        uint16_t crc_recv = (uint16_t)(p->buf[6 + payload_len] |
                                       (p->buf[7 + payload_len] << 8));
        uint8_t tail = p->buf[8 + payload_len];

        /* Copy payload to out_frame BEFORE consuming the buffer.
         * The previous implementation returned a dangling pointer
         * into the buffer after it was consumed — this is a use-after-free. */
        if (out_frame && payload_len > 0) {
            /* Use the version field as a small static copy area.
             * out_frame->payload points to out_frame->payload_buf which
             * the caller provides. We copy into a local area in out_frame. */
            memcpy(out_frame->payload_buf, p->buf + 6, payload_len);
            out_frame->payload = out_frame->payload_buf;
        } else if (out_frame) {
            out_frame->payload = out_frame->payload_buf;
        }

        /* Consume frame */
        memmove(p->buf, p->buf + total, p->buf_len - total);
        p->buf_len -= total;

        if (tail != PI_C_FRAME_TAIL) {
            p->discard_count += total;
            continue;
        }
        if (crc_recv != crc_calc) {
            p->discard_count += total;
            continue;
        }

        out_frame->version = version;
        out_frame->msg_type = msg_type;
        out_frame->seq = seq;
        out_frame->payload_len = payload_len;
        return 1;
    }
    return 0;
}

int pi_c_build_frame(uint8_t *out, size_t out_size, uint8_t msg_type, uint8_t seq,
                     const uint8_t *payload, uint8_t payload_len) {
    uint16_t total = 2 + 4 + payload_len + 2 + 1;
    if (out_size < total || payload_len > PI_C_MAX_PAYLOAD) return 0;

    out[0] = PI_C_FRAME_HEADER_0;
    out[1] = PI_C_FRAME_HEADER_1;
    out[2] = PI_C_PROTOCOL_VERSION;
    out[3] = msg_type;
    out[4] = seq;
    out[5] = payload_len;
    if (payload_len > 0 && payload) {
        memcpy(out + 6, payload, payload_len);
    }
    uint16_t crc = pi_c_crc16(out, 6 + payload_len);
    out[6 + payload_len] = (uint8_t)(crc & 0xFF);
    out[7 + payload_len] = (uint8_t)(crc >> 8);
    out[8 + payload_len] = PI_C_FRAME_TAIL;
    return total;
}

int pi_c_build_ack(uint8_t *out, size_t out_size, uint8_t seq, uint8_t acked_msg_type) {
    uint8_t payload[2] = {seq, acked_msg_type};
    return pi_c_build_frame(out, out_size, MSG_ACK, seq, payload, 2);
}

int pi_c_build_nack(uint8_t *out, size_t out_size, uint8_t seq, uint8_t acked_msg_type, uint8_t reason) {
    uint8_t payload[3] = {seq, acked_msg_type, reason};
    return pi_c_build_frame(out, out_size, MSG_NACK, seq, payload, 3);
}

int pi_c_build_pong(uint8_t *out, size_t out_size, uint8_t seq) {
    uint8_t ver = PI_C_PROTOCOL_VERSION;
    return pi_c_build_frame(out, out_size, MSG_PONG, seq, &ver, 1);
}

int pi_c_decode_chassis_twist(
    const uint8_t *payload,
    size_t payload_len,
    pi_c_chassis_twist_t *out) {
    if (payload == NULL || out == NULL || payload_len != PI_C_CHASSIS_TWIST_PAYLOAD_LEN) return 0;
    out->host_monotonic_ms = (uint32_t)payload[0]
        | ((uint32_t)payload[1] << 8)
        | ((uint32_t)payload[2] << 16)
        | ((uint32_t)payload[3] << 24);
    memcpy(&out->vx_mps, payload + 4, sizeof(float));
    memcpy(&out->vy_mps, payload + 8, sizeof(float));
    memcpy(&out->wz_rad_s, payload + 12, sizeof(float));
    out->valid_for_ms = (uint16_t)payload[16] | ((uint16_t)payload[17] << 8);
    out->control_mode = payload[18];
    out->flags = payload[19];
    if (!isfinite(out->vx_mps) || !isfinite(out->vy_mps) || !isfinite(out->wz_rad_s)) return 0;
    if (out->valid_for_ms == 0U || out->valid_for_ms > 1000U) return 0;
    return out->control_mode == PI_C_CHASSIS_MODE_VELOCITY;
}

int pi_c_encode_imu_status(
    uint8_t *out,
    size_t out_size,
    const cboard_imu_state_t *state) {
    if (out == NULL || state == NULL || out_size < PI_C_IMU_STATUS_PAYLOAD_LEN) return 0;
    const float *vectors[] = {
        state->accel_mps2, state->gyro_rad_s, state->mag,
        state->quaternion_wxyz, state->gyro_bias_rad_s,
        state->calibration_gyro_variance,
    };
    const uint8_t lengths[] = {3U, 3U, 3U, 4U, 3U, 3U};
    for (uint8_t vector = 0U; vector < 6U; ++vector) {
        for (uint8_t index = 0U; index < lengths[vector]; ++index) {
            if (!isfinite(vectors[vector][index])) return 0;
        }
    }
    if (!isfinite(state->roll_rad) || !isfinite(state->pitch_rad) ||
        !isfinite(state->yaw_wrapped_rad) || !isfinite(state->yaw_continuous_rad) ||
        !isfinite(state->gyro_z_rad_s) || !isfinite(state->temperature_c)) return 0;
    memset(out, 0, PI_C_IMU_STATUS_PAYLOAD_LEN);
    memcpy(out + 0, &state->sequence, 4);
    memcpy(out + 4, &state->timestamp_us, 4);
    memcpy(out + 8, &state->age_us, 4);
    memcpy(out + 12, &state->flags, 4);
    memcpy(out + 16, &state->sample_count, 4);
    memcpy(out + 20, &state->error_count, 4);
    memcpy(out + 24, &state->missed_sample_count, 4);
    memcpy(out + 28, &state->spi_error_count, 4);
    memcpy(out + 32, &state->movement_rejection_count, 4);
    memcpy(out + 36, &state->sample_interval_min_us, 4);
    memcpy(out + 40, &state->sample_interval_max_us, 4);
    memcpy(out + 44, &state->update_execution_max_us, 4);
    out[48] = (uint8_t)state->state;
    out[49] = (uint8_t)state->calibration_state;
    out[50] = (uint8_t)state->source;
    out[51] = state->accel_chip_id;
    out[52] = state->gyro_chip_id;
    memcpy(out + 54, &state->calibration_sample_count, 2);
    memcpy(out + 56, state->accel_mps2, 12);
    memcpy(out + 68, state->gyro_rad_s, 12);
    memcpy(out + 80, state->mag, 12);
    memcpy(out + 92, state->quaternion_wxyz, 16);
    memcpy(out + 108, &state->roll_rad, 4);
    memcpy(out + 112, &state->pitch_rad, 4);
    memcpy(out + 116, &state->yaw_wrapped_rad, 4);
    memcpy(out + 120, &state->yaw_continuous_rad, 4);
    memcpy(out + 124, &state->gyro_z_rad_s, 4);
    memcpy(out + 128, &state->temperature_c, 4);
    memcpy(out + 132, state->gyro_bias_rad_s, 12);
    memcpy(out + 144, state->calibration_gyro_variance, 12);
    return (int)PI_C_IMU_STATUS_PAYLOAD_LEN;
}
