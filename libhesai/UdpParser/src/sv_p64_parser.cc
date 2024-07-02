#include "sv_p64_parser.h"

#include "udp_protocol_p64.h"

namespace hesai {
namespace lidar {

template <typename T_Point>
SV_P64_Parser<T_Point>::SV_P64_Parser() {
  this->motor_speed_ = 0;
  this->return_mode_ = 0;
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

      double timestamp = double(packet.sensor_timestamp) / kMicrosecondToSecond;
      if (this->is_dual_return_) {
        timestamp -= static_cast<double>(kBlockOffsetDual[block_id] + kFiringOffset[i]) / kMicrosecondToSecond;
      } else {
        timestamp -= static_cast<double>(kBlockOffsetSingle[block_id] + kFiringOffset[i]) / kMicrosecondToSecond;
      }

      setX(frame.points[point_index], x);
      setY(frame.points[point_index], y);
      setZ(frame.points[point_index], z);
      setIntensity(frame.points[point_index], packet.reflectivities[block_id]);
      setTimestamp(frame.points[point_index], timestamp);
      setRing(frame.points[point_index], i);
      setAngle(frame.points[point_index], static_cast<int>(packet.azimuth[block_id]));
    }
  }
  frame.points_num += packet.points_num;
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