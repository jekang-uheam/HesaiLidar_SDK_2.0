#include "sv_64ch_parser.h"

#include "general_parser.h"
#include "udp_protocol_p64.h"

namespace hesai {
namespace lidar {

template <typename T_Point>
SV64chParser<T_Point>::SV64chParser()
    : UdpP64Parser<T_Point>() {
  this->motor_speed_ = 0;
  this->return_mode_ = 0;
}
template <typename T_Point>
SV64chParser<T_Point>::~SV64chParser() {
}
template <typename T_Point>
int SV64chParser<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket &udpPacket) {
  if (!this->get_correction_file_) {
    static bool printErrorBool = true;
    if (printErrorBool) {
      LogInfo("No available angle calibration files, prohibit parsing of point cloud packages");
      printErrorBool = false;
    }
    return -1;
  }
  if (udpPacket.buffer[0] != 0xEE || udpPacket.buffer[1] != 0xFF) {
    return -1;
  }
  const HS_LIDAR_L64_Header *pHeader =
      reinterpret_cast<const HS_LIDAR_L64_Header *>(&(udpPacket.buffer[0]));

  const HS_LIDAR_BODY_AZIMUTH_L64 *pAzimuth =
      reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_L64 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_L64_Header));

  const HS_LIDAR_BODY_CHN_UNIT_L64 *pChnUnit =
      reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_L64 *>(
          (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_L64));

  const HS_LIDAR_TAIL_L64 *pTail =
      reinterpret_cast<const HS_LIDAR_TAIL_L64 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_L64_Header) +
          (sizeof(HS_LIDAR_BODY_AZIMUTH_L64) +
           sizeof(HS_LIDAR_BODY_CHN_UNIT_L64) * pHeader->GetLaserNum()) *
              pHeader->GetBlockNum());

  const HS_LIDAR_TAIL_SEQ_NUM_L64 *pSeqNum =
      reinterpret_cast<const HS_LIDAR_TAIL_SEQ_NUM_L64 *>(
          (const unsigned char *)pTail + sizeof(HS_LIDAR_TAIL_L64));

  this->is_dual_return_ = pTail->IsDualReturn();
  frame.work_mode = 2;
  frame.return_mode = pTail->GetReturnMode();

  if (frame.use_timestamp_type == 0) {
    frame.sensor_timestamp[frame.packet_num] = pTail->GetMicroLidarTimeU64();
  } else {
    frame.sensor_timestamp[frame.packet_num] = udpPacket.recv_timestamp;
  }
  this->CalPktLoss(pSeqNum->GetSeqNum());
  this->CalPktTimeLoss(pTail->GetMicroLidarTimeU64());
  this->spin_speed_ = pTail->GetMotorSpeed();
  frame.spin_speed = pTail->GetMotorSpeed();
  frame.lidar_state = pTail->m_u8Shutdown;
  frame.return_mode = pTail->m_u8ReturnMode;
  frame.per_points_num = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  frame.scan_complete = false;
  frame.host_timestamp = GetMicroTickCountU64();
  frame.distance_unit = pHeader->GetDistUnit();
  frame.block_num = pHeader->GetBlockNum();
  frame.laser_num = pHeader->GetLaserNum();

  int index = frame.packet_num * pHeader->GetBlockNum() * pHeader->GetLaserNum();
  uint16_t u16Azimuth = 0;
  for (int blockid = 0; blockid < pHeader->GetBlockNum(); blockid++) {
    u16Azimuth = pAzimuth->GetAzimuth();
    pChnUnit = reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_L64 *>(
        (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_L64));

    pAzimuth = reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_L64 *>(
        (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_L64) +
        sizeof(HS_LIDAR_BODY_CHN_UNIT_L64) * pHeader->GetLaserNum());

    auto elevation = 0;
    for (int i = 0; i < pHeader->GetLaserNum(); i++) {
      if (this->get_firetime_file_) {
        frame.pointData[index].azimuth = u16Azimuth + this->rotation_flag * this->GetFiretimesCorrection(i, this->spin_speed_) * kResolutionFloat;
      } else {
        frame.pointData[index].azimuth = u16Azimuth;
      }
      frame.pointData[index].distances = pChnUnit->GetDistance();
      frame.pointData[index].reflectivities = pChnUnit->GetReflectivity();
      frame.pointData[index].elevation = elevation;
      pChnUnit = pChnUnit + 1;
      index++;
    }
  }
  if (IsNeedFrameSplit(u16Azimuth)) {
    frame.scan_complete = true;
  }
  if (u16Azimuth != this->last_azimuth_) {
    this->last_last_azimuth_ = this->last_azimuth_;
    this->last_azimuth_ = u16Azimuth;
  }
  frame.packet_num++;
  return 0;
}
template <typename T_Point>
int SV64chParser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, int packet_index) {
  for (int blockid = 0; blockid < frame.block_num; blockid++) {
    // T_Point point;
    int elevation = 0;
    int azimuth = 0;

    for (int i = 0; i < frame.laser_num; i++) {
      int point_index = packet_index * frame.per_points_num + blockid * frame.laser_num + i;
      float distance = frame.pointData[point_index].distances * frame.distance_unit;
      int Azimuth = frame.pointData[point_index].azimuth * kFineResolutionInt;
      if (this->get_correction_file_) {
        int azimuth_coll = (int(this->azimuth_collection_[i] * kAllFineResolutionFloat) + CIRCLE) % CIRCLE;
        int elevation_corr = (int(this->elevation_correction_[i] * kAllFineResolutionFloat) + CIRCLE) % CIRCLE;
        if (this->enable_distance_correction_) {
          GetDistanceCorrection(azimuth_coll, elevation_corr, distance, GeometricCenter);
        }
        elevation = elevation_corr;
        azimuth = Azimuth + azimuth_coll;
        azimuth = (CIRCLE + azimuth) % CIRCLE;
      }
      if (frame.config.fov_start != -1 && frame.config.fov_end != -1) {
        int fov_transfer = azimuth / 256 / 100;
        if (fov_transfer < frame.config.fov_start || fov_transfer > frame.config.fov_end) {  // 不在fov范围continue
          memset(&frame.points[point_index], 0, sizeof(T_Point));
          continue;
        }
      }
      float xyDistance = distance * this->cos_all_angle_[(elevation)];
      float x = xyDistance * this->sin_all_angle_[(azimuth)];
      float y = xyDistance * this->cos_all_angle_[(azimuth)];
      float z = distance * this->sin_all_angle_[(elevation)];
      this->TransformPoint(x, y, z);

      double timestamp = static_cast<double>(frame.sensor_timestamp[packet_index]) / kMicrosecondToSecond;
      if (this->is_dual_return_) {
        timestamp -= static_cast<double>(kBlockOffsetDual[blockid] + kFiringOffset[i]) / kMicrosecondToSecond;
      } else {
        timestamp -= static_cast<double>(kBlockOffsetSingle[blockid] + kFiringOffset[i]) / kMicrosecondToSecond;
      }

      setX(frame.points[point_index], x);
      setY(frame.points[point_index], y);
      setZ(frame.points[point_index], z);
      setIntensity(frame.points[point_index], frame.pointData[point_index].reflectivities);
      setTimestamp(frame.points[point_index], timestamp);
      setRing(frame.points[point_index], i);
      setAngle(frame.points[point_index], static_cast<int>(frame.pointData[point_index].azimuth));

      if (frame.first_timestamp_ == 0.0 || (timestamp > 0 && timestamp < frame.first_timestamp_)) {
        frame.first_timestamp_ = timestamp;
      }
    }
  }
  GeneralParser<T_Point>::FrameNumAdd();
  return 0;
}
template <typename T_Point>
bool SV64chParser<T_Point>::IsNeedFrameSplit(uint16_t azimuth) {
  // Determine frame_start_azimuth_ [0,360)
  if (this->frame_start_azimuth_ < 0.0f || this->frame_start_azimuth_ >= 360.0f) {
    this->frame_start_azimuth_ = 0.0f;
  }
  // The first two packet dont have the information of last_azimuth_  and last_last_azimuth, so do not need split frame
  // The initial value of last_azimuth_ is -1
  // Determine the rotation direction and division

  uint16_t division = 0;
  // If last_last_azimuth_ != -1，the packet is the third, so we can determine whether the current packet requires framing
  if (this->last_last_azimuth_ != -1) {
    // Get the division
    uint16_t division1 = abs(this->last_azimuth_ - this->last_last_azimuth_);
    uint16_t division2 = abs(this->last_azimuth_ - azimuth);
    division = division1 > division2 ? division2 : division1;
    // Prevent two consecutive packets from having the same angle when causing an error in framing
    if (division == 0) return false;
    // In the three consecutive angle values, if the angle values appear by the division of the decreasing situation,it must be reversed
    // The same is true for FOV
    if (this->last_last_azimuth_ - this->last_azimuth_ == division || this->last_azimuth_ - azimuth == division) {
      this->rotation_flag = -1;
    } else {
      this->rotation_flag = 1;
    }
  } else {
    // The first  and second packet do not need split frame
    return false;
  }
  if (this->rotation_flag == 1) {
    // When an angle jump occurs
    if (this->last_azimuth_ - azimuth > division) {
      if (uint16_t(this->frame_start_azimuth_ * kResolutionInt) > this->last_azimuth_ || uint16_t(this->frame_start_azimuth_ * kResolutionInt <= azimuth)) {
        return true;
      }
      return false;
    }
    if (this->last_azimuth_ < azimuth && this->last_azimuth_ < uint16_t(this->frame_start_azimuth_ * kResolutionInt) && azimuth >= uint16_t(this->frame_start_azimuth_ * kResolutionInt)) {
      return true;
    }
    return false;
  } else {
    if (azimuth - this->last_azimuth_ > division) {
      if (uint16_t(this->frame_start_azimuth_ * kResolutionInt) <= this->last_azimuth_ || uint16_t(this->frame_start_azimuth_ * kResolutionInt) > azimuth) {
        return true;
      }
      return false;
    }
    if (this->last_azimuth_ > azimuth && this->last_azimuth_ > uint16_t(this->frame_start_azimuth_ * kResolutionInt) && azimuth <= uint16_t(this->frame_start_azimuth_ * kResolutionInt)) {
      return true;
    }
    return false;
  }
}

}  // namespace lidar
}  // namespace hesai