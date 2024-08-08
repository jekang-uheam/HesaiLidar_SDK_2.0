#include "sv_p64_parser.h"

#include "udp_protocol_p64.h"

namespace hesai {
namespace lidar {

template <typename T_Point>
SV_P64_Parser<T_Point>::SV_P64_Parser() {
  this->motor_speed_ = 0;
  this->return_mode_ = 0;

  const size_t kBlockOffsetSize = 6;

  block_offset_64_single_.reserve(kBlockOffsetSize);
  // Block 1 (42.58 + 55.56 * 5)
  block_offset_64_single_[0] = 42.58f + (55.56f * (kBlockOffsetSize - 1));
  // Block 2 (42.58 + 55.56 * 4)
  block_offset_64_single_[1] = 42.58f + (55.56f * (kBlockOffsetSize - 2));
  // Block 3 (42.58 + 55.56 * 3)
  block_offset_64_single_[2] = 42.58f + (55.56f * (kBlockOffsetSize - 3));
  // Block 4 (42.58 + 55.56 * 2)
  block_offset_64_single_[3] = 42.58f + (55.56f * (kBlockOffsetSize - 4));
  // Block 5 (42.58 + 55.56 * 1)
  block_offset_64_single_[4] = 42.58f + (55.56f * (kBlockOffsetSize - 5));
  // Block 6 (42.58 + 55.56 * 0)
  block_offset_64_single_[5] = 42.58f + (55.56f * (kBlockOffsetSize - 6));

  block_offset_64_dual_.reserve(kBlockOffsetSize);
  // Block 1 & 2 (42.58 + 55.56 * 2)
  block_offset_64_dual_[0] = 42.58f + (55.56f * (kBlockOffsetSize - 4));
  block_offset_64_dual_[1] = 42.58f + (55.56f * (kBlockOffsetSize - 4));
  // Block 3 & 4 (42.58 + 55.56 * 1)
  block_offset_64_dual_[2] = 42.58f + (55.56f * (kBlockOffsetSize - 5));
  block_offset_64_dual_[3] = 42.58f + (55.56f * (kBlockOffsetSize - 5));
  // Block 5 & 6 (42.58 + 55.56 * 0)
  block_offset_64_dual_[4] = 42.58f + (55.56f * (kBlockOffsetSize - 6));
  block_offset_64_dual_[5] = 42.58f + (55.56f * (kBlockOffsetSize - 6));

  const size_t kFiringOffsetSize = 64;
  firing_offset_64_.reserve(kFiringOffsetSize);
  // Firing Sequence 1 (54.668)
  firing_offset_64_[11] = 3.62f + (1.304f * 15.0f) + (1.968f * 16.0f);
  firing_offset_64_[39] = 3.62f + (1.304f * 15.0f) + (1.968f * 16.0f);
  // Firing Sequence 2 (52.7)
  firing_offset_64_[17] = 3.62f + (1.304f * 15.0f) + (1.968f * 15.0f);
  firing_offset_64_[37] = 3.62f + (1.304f * 15.0f) + (1.968f * 15.0f);
  // Firing Sequence 3 (50.732)
  firing_offset_64_[10] = 3.62f + (1.304f * 15.0f) + (1.968f * 14.0f);
  firing_offset_64_[25] = 3.62f + (1.304f * 15.0f) + (1.968f * 14.0f);
  // Firing Sequence 4 (48.764)
  firing_offset_64_[16] = 3.62f + (1.304f * 15.0f) + (1.968f * 13.0f);
  firing_offset_64_[31] = 3.62f + (1.304f * 15.0f) + (1.968f * 13.0f);
  // Firing Sequence 5 (46.796)
  firing_offset_64_[7] = 3.62f + (1.304f * 15.0f) + (1.968f * 12.0f);
  firing_offset_64_[22] = 3.62f + (1.304f * 15.0f) + (1.968f * 12.0f);
  // Firing Sequence 6 (44.828)
  firing_offset_64_[13] = 3.62f + (1.304f * 15.0f) + (1.968f * 11.0f);
  firing_offset_64_[28] = 3.62f + (1.304f * 15.0f) + (1.968f * 11.0f);
  // Firing Sequence 7 (42.86)
  firing_offset_64_[19] = 3.62f + (1.304f * 15.0f) + (1.968f * 10.0f);
  firing_offset_64_[34] = 3.62f + (1.304f * 15.0f) + (1.968f * 10.0f);
  // Firing Sequence 8 (40.892)
  firing_offset_64_[12] = 3.62f + (1.304f * 15.0f) + (1.968f * 9.0f);
  firing_offset_64_[27] = 3.62f + (1.304f * 15.0f) + (1.968f * 9.0f);
  // Firing Sequence 9 (38.924)
  firing_offset_64_[18] = 3.62f + (1.304f * 15.0f) + (1.968f * 8.0f);
  firing_offset_64_[33] = 3.62f + (1.304f * 15.0f) + (1.968f * 8.0f);
  // Firing Sequence 10 (36.956)
  firing_offset_64_[9] = 3.62f + (1.304f * 15.0f) + (1.968f * 7.0f);
  firing_offset_64_[24] = 3.62f + (1.304f * 15.0f) + (1.968f * 7.0f);
  // Firing Sequence 11 (34.988)
  firing_offset_64_[15] = 3.62f + (1.304f * 15.0f) + (1.968f * 6.0f);
  firing_offset_64_[30] = 3.62f + (1.304f * 15.0f) + (1.968f * 6.0f);
  // Firing Sequence 12 (33.02)
  firing_offset_64_[21] = 3.62f + (1.304f * 15.0f) + (1.968f * 5.0f);
  firing_offset_64_[36] = 3.62f + (1.304f * 15.0f) + (1.968f * 5.0f);
  // Firing Sequence 13 (31.052)
  firing_offset_64_[14] = 3.62f + (1.304f * 15.0f) + (1.968f * 4.0f);
  firing_offset_64_[29] = 3.62f + (1.304f * 15.0f) + (1.968f * 4.0f);
  // Firing Sequence 14 (29.084)
  firing_offset_64_[20] = 3.62f + (1.304f * 15.0f) + (1.968f * 3.0f);
  firing_offset_64_[35] = 3.62f + (1.304f * 15.0f) + (1.968f * 3.0f);
  // Firing Sequence 15 (27.116)
  firing_offset_64_[26] = 3.62f + (1.304f * 15.0f) + (1.968f * 2.0f);
  firing_offset_64_[41] = 3.62f + (1.304f * 15.0f) + (1.968f * 2.0f);
  // Firing Sequence 16 (25.148)
  firing_offset_64_[23] = 3.62f + (1.304f * 15.0f) + (1.968f * 1.0f);
  firing_offset_64_[32] = 3.62f + (1.304f * 15.0f) + (1.968f * 1.0f);
  // Firing Sequence 17 (23.18)
  firing_offset_64_[0] = 3.62f + (1.304f * 15.0f) + (1.968f * 0.0f);
  firing_offset_64_[43] = 3.62f + (1.304f * 15.0f) + (1.968f * 0.0f);
  // Firing Sequence 18 (21.876)
  firing_offset_64_[1] = 3.62f + (1.304f * 14.0f) + (1.968f * 0.0f);
  firing_offset_64_[45] = 3.62f + (1.304f * 14.0f) + (1.968f * 0.0f);
  // Firing Sequence 19 (20.572)
  firing_offset_64_[2] = 3.62f + (1.304f * 13.0f) + (1.968f * 0.0f);
  firing_offset_64_[51] = 3.62f + (1.304f * 13.0f) + (1.968f * 0.0f);
  // Firing Sequence 20 (19.268)
  firing_offset_64_[3] = 3.62f + (1.304f * 12.0f) + (1.968f * 0.0f);
  firing_offset_64_[49] = 3.62f + (1.304f * 12.0f) + (1.968f * 0.0f);
  // Firing Sequence 21 (17.964)
  firing_offset_64_[4] = 3.62f + (1.304f * 11.0f) + (1.968f * 0.0f);
  firing_offset_64_[47] = 3.62f + (1.304f * 11.0f) + (1.968f * 0.0f);
  // Firing Sequence 22 (16.66)
  firing_offset_64_[5] = 3.62f + (1.304f * 10.0f) + (1.968f * 0.0f);
  firing_offset_64_[53] = 3.62f + (1.304f * 10.0f) + (1.968f * 0.0f);
  // Firing Sequence 23 (15.356)
  firing_offset_64_[40] = 3.62f + (1.304f * 9.0f) + (1.968f * 0.0f);
  firing_offset_64_[57] = 3.62f + (1.304f * 9.0f) + (1.968f * 0.0f);
  // Firing Sequence 24 (14.052)
  firing_offset_64_[46] = 3.62f + (1.304f * 8.0f) + (1.968f * 0.0f);
  firing_offset_64_[61] = 3.62f + (1.304f * 8.0f) + (1.968f * 0.0f);
  // Firing Sequence 25 (12.748)
  firing_offset_64_[52] = 3.62f + (1.304f * 7.0f) + (1.968f * 0.0f);
  firing_offset_64_[63] = 3.62f + (1.304f * 7.0f) + (1.968f * 0.0f);
  // Firing Sequence 26 (11.444)
  firing_offset_64_[6] = 3.62f + (1.304f * 6.0f) + (1.968f * 0.0f);
  firing_offset_64_[55] = 3.62f + (1.304f * 6.0f) + (1.968f * 0.0f);
  // Firing Sequence 27 (10.14)
  firing_offset_64_[42] = 3.62f + (1.304f * 5.0f) + (1.968f * 0.0f);
  firing_offset_64_[58] = 3.62f + (1.304f * 5.0f) + (1.968f * 0.0f);
  // Firing Sequence 28 (8.836)
  firing_offset_64_[48] = 3.62f + (1.304f * 4.0f) + (1.968f * 0.0f);
  firing_offset_64_[62] = 3.62f + (1.304f * 4.0f) + (1.968f * 0.0f);
  // Firing Sequence 29 (7.532)
  firing_offset_64_[8] = 3.62f + (1.304f * 3.0f) + (1.968f * 0.0f);
  firing_offset_64_[54] = 3.62f + (1.304f * 3.0f) + (1.968f * 0.0f);
  // Firing Sequence 30 (6.228)
  firing_offset_64_[38] = 3.62f + (1.304f * 2.0f) + (1.968f * 0.0f);
  firing_offset_64_[56] = 3.62f + (1.304f * 2.0f) + (1.968f * 0.0f);
  // Firing Sequence 31 (4.924)
  firing_offset_64_[44] = 3.62f + (1.304f * 1.0f) + (1.968f * 0.0f);
  firing_offset_64_[59] = 3.62f + (1.304f * 1.0f) + (1.968f * 0.0f);
  // Firing Sequence 32 (3.62)
  firing_offset_64_[50] = 3.62f + (1.304f * 0.0f) + (1.968f * 0.0f);
  firing_offset_64_[60] = 3.62f + (1.304f * 0.0f) + (1.968f * 0.0f);
}

template <typename T_Point>
SV_P64_Parser<T_Point>::~SV_P64_Parser() {
  std::cout << "Release SV P64 Parser" << std::endl;
}

template <typename T_Point>
int SV_P64_Parser<T_Point>::DecodePacket(LidarDecodedPacket<T_Point> &output, const UdpPacket &udp_packet) {
  if (udp_packet.buffer[0] != 0xEE || udp_packet.buffer[1] != 0xFF) {
    return -1;
  }

  const HS_LIDAR_L64_Header *header_ptr =
      reinterpret_cast<const HS_LIDAR_L64_Header *>(&(udp_packet.buffer[0]));

  const HS_LIDAR_TAIL_L64 *tail_ptr =
      reinterpret_cast<const HS_LIDAR_TAIL_L64 *>(
          (const unsigned char *)header_ptr + sizeof(HS_LIDAR_L64_Header) +
          (sizeof(HS_LIDAR_BODY_AZIMUTH_L64) + (sizeof(HS_LIDAR_BODY_CHN_UNIT_L64) * header_ptr->GetLaserNum())) * header_ptr->GetBlockNum());

  this->spin_speed_ = tail_ptr->GetMotorSpeed();
  output.spin_speed = tail_ptr->m_u16MotorSpeed;

  this->is_dual_return_ = tail_ptr->IsDualReturn();
  output.work_mode = 2;

  output.return_mode = tail_ptr->GetReturnMode();

  output.host_timestamp = GetMicroTickCountU64();
  output.points_num = header_ptr->GetBlockNum() * header_ptr->GetLaserNum();
  output.scan_complete = false;

  output.sensor_timestamp = tail_ptr->GetMicroLidarTimeU64();
  output.distance_unit = header_ptr->GetDistUnit();

  output.block_num = header_ptr->GetBlockNum();
  output.laser_num = header_ptr->GetLaserNum();

  int index = 0;
  for (uint8_t block_id = 0; block_id < header_ptr->GetBlockNum(); ++block_id) {
    // point to azimuth addr
    const HS_LIDAR_BODY_AZIMUTH_L64 *azimuth_ptr =
        reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_L64 *>(
            (const unsigned char *)header_ptr + sizeof(HS_LIDAR_L64_Header) +
            (sizeof(HS_LIDAR_BODY_AZIMUTH_L64) + (sizeof(HS_LIDAR_BODY_CHN_UNIT_L64) * header_ptr->GetLaserNum())) * block_id);

    // point to channel unit addr
    const HS_LIDAR_BODY_CHN_UNIT_L64 *chn_unit_ptr =
        reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_L64 *>(
            (const unsigned char *)azimuth_ptr + sizeof(HS_LIDAR_BODY_AZIMUTH_L64));

    for (uint8_t i = 0; i < header_ptr->GetLaserNum(); ++i) {
      output.reflectivities[index] = chn_unit_ptr->GetReflectivity();
      output.distances[index] = chn_unit_ptr->GetDistance();
      output.azimuth[index] = azimuth_ptr->GetAzimuth();
      if (this->get_firetime_file_) {
        output.azimuth[index] += (kResolutionInt * this->GetFiretimesCorrection(i, this->spin_speed_));
      }
      ++chn_unit_ptr;
      ++index;
    }

    if (IsNeedFrameSplit(azimuth_ptr->GetAzimuth())) {
      output.scan_complete = true;
    }
    this->last_azimuth_ = azimuth_ptr->GetAzimuth();
  }

  return 0;
}

template <typename T_Point>
int SV_P64_Parser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, LidarDecodedPacket<T_Point> &packet) {
  int azimuth = 0;
  int elevation = 0;

  frame.laser_num = packet.laser_num;
  frame.spin_speed = packet.spin_speed;
  frame.return_mode = packet.return_mode;
  frame.work_mode = packet.work_mode;

  for (uint16_t j = 0; j < packet.block_num; ++j) {
    azimuth = 0;
    elevation = 0;

    for (uint16_t i = 0; i < packet.laser_num; ++i) {
      int block_id = (packet.laser_num * j) + i;
      int point_index = (packet.packet_index * packet.points_num) + block_id;
      float distance = packet.distances[block_id] * packet.distance_unit;
      if (this->get_correction_file_) {
        elevation = this->elevation_correction_[i] * kResolutionInt;
        elevation = (CIRCLE + elevation) % CIRCLE;
        azimuth = static_cast<int>(packet.azimuth[block_id]) + (this->azimuth_collection_[i] * kResolutionInt);
        azimuth = (CIRCLE + azimuth) % CIRCLE;
      }

      float xy_distance = distance * this->cos_all_angle_[elevation];
      float x = xy_distance * this->sin_all_angle_[azimuth];
      float y = xy_distance * this->cos_all_angle_[azimuth];
      float z = distance * this->sin_all_angle_[elevation];
      this->TransformPoint(x, y, z);

      double timestamp = double(packet.sensor_timestamp);
      if (this->is_dual_return_) {
        timestamp -= static_cast<double>(block_offset_64_dual_[j] + firing_offset_64_[i]);
      } else {
        timestamp -= static_cast<double>(block_offset_64_single_[j] + firing_offset_64_[i]);
      }

      setX(frame.points[point_index], x);
      setY(frame.points[point_index], y);
      setZ(frame.points[point_index], z);
      setIntensity(frame.points[point_index], packet.reflectivities[block_id]);
      setTimestamp(frame.points[point_index], timestamp / kMicrosecondToSecond);
      setRing(frame.points[point_index], i);
      setAngle(frame.points[point_index], static_cast<int>(packet.azimuth[block_id]));
      if (distance <= 0.1 || distance > 200.0) {
        // timestamp < 0 == wrong point
        // for checking data, insert check to last.
        setTimestamp(frame.points[point_index], -100);
      } else {
        ++valid_points;
      }
    }
  }

  frame.points_num += packet.points_num;
  frame.valid_points_num += valid_points;
  frame.packet_num = packet.packet_index;
  return 0;
}

template <typename T_Point>
bool SV_P64_Parser<T_Point>::IsNeedFrameSplit(uint16_t azimuth) {
  if (this->last_azimuth_ > azimuth && (this->last_azimuth_ - azimuth > kSplitFrameMinAngle)) {
    return true;
  }
  return false;
}

}  // namespace lidar
}  // namespace hesai