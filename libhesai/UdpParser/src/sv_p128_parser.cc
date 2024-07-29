#include "sv_p128_parser.h"

#include <iomanip>

#include "udp_protocol_v1_4.h"

namespace hesai {
namespace lidar {

template <typename T_Point>
SV_P128_Parser<T_Point>::SV_P128_Parser()
    : section_distance_(0),
      distance_correction_para_a_(1),
      distance_correction_para_b_(0.012),
      distance_correction_para_h_(0.04),
      distance_correction_para_c_(0),
      distance_correction_para_d_(0),
      use_frame_start_azimuth_(true) {
  this->motor_speed_ = 0;
  this->return_mode_ = 0;
  distance_correction_para_c_ = std::sqrt(distance_correction_para_b_ * distance_correction_para_b_ + distance_correction_para_h_ * distance_correction_para_h_);
  distance_correction_para_d_ = std::atan(distance_correction_para_b_ / distance_correction_para_h_);
}

template <typename T_Point>
SV_P128_Parser<T_Point>::~SV_P128_Parser() {
  std::cout << "Release SV P128 Parser" << std::endl;
}

template <typename T_Point>
void SV_P128_Parser<T_Point>::LoadFiretimesFile(std::string firetimes_path) {
  std::ifstream in_file(firetimes_path, std::ios::in);
  if (!in_file.is_open()) {
    std::cout << "Open firetime file failed" << std::endl;
    this->get_firetime_file_ = false;
    return;
  }

  int useless_line = 0;
  int count = 0;
  std::string line_str;
  while (std::getline(in_file, line_str)) {
    std::stringstream ss(line_str);
    std::string str;
    std::vector<std::string> str_list;
    str_list.reserve(20);

    while (std::getline(ss, str, ',')) {
      str_list.push_back(str);
    }

    if (useless_line == 0) {
      std::string dist;
      for (auto e : str_list[0]) {
        if (e == '.' || (e >= '0' && e <= '9')) {
          dist.push_back(e);
        }
      }
      section_distance_ = std::stod(dist);
    }

    if (useless_line < 3) {
      ++useless_line;
      continue;
    }

    if (line_str[line_str.size() - 1] == '\n') {
      line_str = line_str.substr(line_str.size() - 1);
    }

    if (str_list.size() < 17) {
      std::cout << "nvalid input file!" << std::endl;
      this->get_firetime_file_ = false;
      return;
    }

    int idx = std::stoi(str_list[0]) - 1;
    for (int i = 1; i <= 15; i += 2) {
      int a = std::stoi(str_list[i]);
      int b = std::stoi(str_list[i + 1]);
      firetime_section_values[idx].section_values[i / 2].firetime[0] = a;
      firetime_section_values[idx].section_values[i / 2].firetime[1] = b;
    }

    ++count;
    if (count > 128) {
      std::cout << "Invalid input file!" << std::endl;
      this->get_firetime_file_ = false;
      return;
    }
  }

  std::cout << "Open firetime file success!" << std::endl;
  this->get_firetime_file_ = true;
}

template <typename T_Point>
double SV_P128_Parser<T_Point>::GetFiretimesCorrection(int laser_id, double speed, uint8_t opt_mode, uint8_t angle_state, uint16_t dist) {
  int idx = 0;
  switch (opt_mode) {
    case 0:
      idx = angle_state;
      break;
    case 2:
      idx = 4 + angle_state;
      break;
    case 3:
      idx = 6 + angle_state;
      break;

    default:
      return 0.0;
      break;
  }
  int k = (dist >= section_distance_) ? 0 : 1;
  return firetime_section_values[laser_id].section_values[idx].firetime[k] * speed * 6E-9;
}

template <typename T_Point>
int SV_P128_Parser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, LidarDecodedPacket<T_Point> &packet) {
  frame.lidar_state = packet.lidar_state;
  frame.work_mode = packet.work_mode;

  frame.laser_num = packet.laser_num;
  frame.spin_speed = packet.spin_speed;
  frame.return_mode = packet.return_mode;

  int azimuth = 0;
  int elevation = 0;
  for (uint16_t block_id = 0; block_id < packet.block_num; ++block_id) {
    for (uint16_t i = 0; i < packet.laser_num; ++i) {
      int point_index = (packet.packet_index * packet.points_num) + (block_id * packet.laser_num) + i;
      float distance = packet.distances[(block_id * packet.laser_num) + i] * packet.distance_unit;
      if (this->get_correction_file_) {
        elevation = this->elevation_correction_[i] * kResolutionInt;
        elevation = (CIRCLE + elevation) % CIRCLE;
        azimuth = static_cast<int>(packet.azimuth[block_id * packet.laser_num]) + (this->azimuth_collection_[i] * kResolutionInt);
        azimuth = (CIRCLE + azimuth) % CIRCLE;
      }

      if (this->enable_distance_correction_) {
        GetDistanceCorrection(i, distance, azimuth, elevation);
      }

      float xy_distance = distance * this->cos_all_angle_[(elevation)];
      float x = xy_distance * this->sin_all_angle_[(azimuth)];
      float y = xy_distance * this->cos_all_angle_[(azimuth)];
      float z = distance * this->sin_all_angle_[(elevation)];
      this->TransformPoint(x, y, z);

      setX(frame.points[point_index], x);
      setY(frame.points[point_index], y);
      setZ(frame.points[point_index], z);
      setIntensity(frame.points[point_index], packet.reflectivities[block_id * packet.laser_num + i]);
      setTimestamp(frame.points[point_index], double(packet.sensor_timestamp) / kMicrosecondToSecond);
      setRing(frame.points[point_index], i);
      setAngle(frame.points[point_index], static_cast<int>(packet.azimuth[block_id * packet.laser_num]));
    }
  }
  frame.points_num += packet.points_num;
  frame.packet_num = packet.packet_index;
  return 0;
}

template <typename T_Point>
int SV_P128_Parser<T_Point>::DecodePacket(LidarDecodedPacket<T_Point> &output, const UdpPacket &udp_packet) {
  if (udp_packet.buffer[0] != 0xEE || udp_packet.buffer[1] != 0xFF) {
    return -1;
  }
  size_t offset = 0;

  // const HS_LIDAR_PRE_HEADER *pre_header_ptr =
  //    reinterpret_cast<const HS_LIDAR_PRE_HEADER *>(udp_packet.buffer + offset);
  offset += sizeof(HS_LIDAR_PRE_HEADER);

  const HS_LIDAR_HEADER_ME_V4 *header =
      reinterpret_cast<const HS_LIDAR_HEADER_ME_V4 *>(udp_packet.buffer + offset);
  offset += sizeof(HS_LIDAR_HEADER_ME_V4);

  size_t chn_unit_size = 0;
  if (header->HasConfidenceLevel()) {
    chn_unit_size = sizeof(HS_LIDAR_BODY_CHN_UNIT_ME_V4);
  } else {
    chn_unit_size = sizeof(HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4);
  }

  size_t func_safety_size = 0;
  if (header->HasFuncSafety()) {
    func_safety_size = sizeof(HS_LIDAR_BODY_CRC_ME_V4);
  }

  size_t seq_num_size = 0;
  if (header->HasSeqNum()) {
    seq_num_size = sizeof(HS_LIDAR_TAIL_SEQ_NUM_ME_V4);
  }

  size_t imu_size = 0;
  if (header->HasIMU()) {
    imu_size = sizeof(HS_LIDAR_TAIL_IMU_ME_V4);
  }

  size_t cyber_security_size = 0;
  if (header->HasCyberSecurity()) {
    cyber_security_size = sizeof(HS_LIDAR_CYBER_SECURITY_ME_V4);
  }

  const HS_LIDAR_BODY_AZIMUTH_ME_V4 *azimuth =
      reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_ME_V4 *>(udp_packet.buffer + offset);
  if (header->HasConfidenceLevel()) {
    offset += (sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4) + (sizeof(HS_LIDAR_BODY_CHN_UNIT_ME_V4) * header->GetLaserNum())) * header->GetBlockNum();
  } else {
    offset += (sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4) + (sizeof(HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4) * header->GetLaserNum())) * header->GetBlockNum();
  }

  // const HS_LIDAR_BODY_CRC_ME_V4 *body_crc =
  //     reinterpret_cast<const HS_LIDAR_BODY_CRC_ME_V4 *>(udp_packet.buffer + offset);
  offset += sizeof(HS_LIDAR_BODY_CRC_ME_V4);

  if (header->HasFuncSafety()) {
    const HS_LIDAR_FUNC_SAFETY_ME_V4 *function_savety =
        reinterpret_cast<const HS_LIDAR_FUNC_SAFETY_ME_V4 *>(udp_packet.buffer + offset);
    offset += sizeof(HS_LIDAR_FUNC_SAFETY_ME_V4);

    output.lidar_state = function_savety->GetLidarState();
  } else {
    output.lidar_state = -1;
  }

  const HS_LIDAR_TAIL_ME_V4 *tail =
      reinterpret_cast<const HS_LIDAR_TAIL_ME_V4 *>(udp_packet.buffer + offset);
  offset += sizeof(HS_LIDAR_TAIL_ME_V4);

  if (header->HasSeqNum()) {
    const HS_LIDAR_TAIL_SEQ_NUM_ME_V4 *tail_seq_num =
        reinterpret_cast<const HS_LIDAR_TAIL_SEQ_NUM_ME_V4 *>(udp_packet.buffer + offset);
    offset += sizeof(HS_LIDAR_TAIL_SEQ_NUM_ME_V4);

    // skip decode packet if enable packet_loss_tool
    if (this->enable_packet_loss_tool_ == true) {
      this->current_seqnum_ = tail_seq_num->m_u32SeqNum;
      if (this->current_seqnum_ > this->last_seqnum_ && this->last_seqnum_ != 0) {
        this->total_packet_count_ += this->current_seqnum_ - this->last_seqnum_;
      }
      tail_seq_num->CalPktLoss(this->start_seqnum_, this->last_seqnum_, this->loss_count_, this->start_time_, this->total_loss_count_, this->total_start_seqnum_);
    }
  }

  if (header->HasIMU()) {
    // const HS_LIDAR_TAIL_IMU_ME_V4 *tail_imu =
    //    reinterpret_cast<const HS_LIDAR_TAIL_IMU_ME_V4 *>(udp_packet.buffer + offset);
    offset += sizeof(HS_LIDAR_TAIL_IMU_ME_V4);
    /*
        std::cout << "HS_LIDAR_TAIL_IMU_ME_V4: \n"
                  << " Temperature: " << std::setw(10) << tail_imu->GetIMUTemperature() << " / "
                  << " Timestamp  : " << std::setw(10) << tail_imu->GetIMUTimestamp() << "\n"

                  << " AccelUnit  : " << std::setw(10) << tail_imu->GetIMUAccelUnit() << " / "
                  << " AngVelUnit : " << std::setw(10) << tail_imu->GetIMUAngVelUnit() << "\n"

                  << " X Accel    : " << std::setw(10) << tail_imu->GetIMUXAccel() << " / "
                  << " Y Accel    : " << std::setw(10) << tail_imu->GetIMUYAccel() << " / "
                  << " Z Accel    : " << std::setw(10) << tail_imu->GetIMUZAccel() << "\n"
                  << " X AngVel   : " << std::setw(10) << tail_imu->GetIMUXAngVel() << " / "
                  << " Y AngVel   : " << std::setw(10) << tail_imu->GetIMUYAngVel() << " / "
                  << " Z AngVel   : " << std::setw(10) << tail_imu->GetIMUZAngVel() << "\n"
                  << std::endl;
    */
  }

  const HS_LIDAR_TAIL_CRC_ME_V4 *tail_crc =
      reinterpret_cast<const HS_LIDAR_TAIL_CRC_ME_V4 *>(udp_packet.buffer + offset);
  offset += sizeof(HS_LIDAR_TAIL_CRC_ME_V4);

  if (header->HasCyberSecurity()) {
    const HS_LIDAR_CYBER_SECURITY_ME_V4 *cyber_security =
        reinterpret_cast<const HS_LIDAR_CYBER_SECURITY_ME_V4 *>(udp_packet.buffer + offset);
    offset += sizeof(HS_LIDAR_CYBER_SECURITY_ME_V4);
  }

  output.sensor_timestamp = tail->GetMicroLidarTimeU64();
  output.host_timestamp = GetMicroTickCountU64();

  if (this->enable_packet_loss_tool_ == true) {
    return 0;
  }
  this->spin_speed_ = tail->m_u16MotorSpeed;
  this->is_dual_return_ = tail->IsDualReturn();
  output.spin_speed = tail->m_u16MotorSpeed;
  output.work_mode = tail->getOperationMode();

  output.return_mode = tail->GetReturnMode();

  // 如下三条：max min这样的参数一点用都没有
  output.maxPoints = header->GetBlockNum() * header->GetLaserNum();
  // 不填直接崩调，=0界面一个点也没有
  output.points_num = header->GetBlockNum() * header->GetLaserNum();
  // 不填则仅显示很小一部分点云
  output.scan_complete = false;
  output.distance_unit = header->GetDistUnit();

  // clean-up // float minAzimuth = 0;
  // clean-up // float maxAzimuth = 0;
  output.block_num = header->GetBlockNum();
  output.laser_num = header->GetLaserNum();

  if (this->enable_update_monitor_info_) {
    this->monitor_info1_[tail->m_reservedInfo1.m_u8ID] = tail->m_reservedInfo1.m_u16Sts;
    this->monitor_info2_[tail->m_reservedInfo2.m_u8ID] = tail->m_reservedInfo2.m_u16Sts;
    this->monitor_info3_[tail->m_reservedInfo3.m_u8ID] = tail->m_reservedInfo3.m_u16Sts;
  }

  int index = 0;
  uint8_t operation_mode = tail->getOperationMode();
  for (uint8_t i = 0; i < header->GetBlockNum(); ++i) {
    uint8_t angle_state = tail->getAngleState(i);
    uint16_t u16_azimuth = azimuth->GetAzimuth();
    output.azimuths = u16_azimuth;

    if (header->HasConfidenceLevel()) {
      // point to channel unit addr
      const HS_LIDAR_BODY_CHN_UNIT_ME_V4 *chn_unit =
          reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_ME_V4 *>((const unsigned char *)azimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4));

      // point to next block azimuth addr
      azimuth =
          reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_ME_V4 *>((const unsigned char *)azimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4) + (sizeof(HS_LIDAR_BODY_CHN_UNIT_ME_V4) * header->GetLaserNum()));

      for (uint8_t j = 0; j < header->GetLaserNum(); ++j) {
        output.azimuth[index] = u16_azimuth;

        if (this->get_firetime_file_) {
          output.azimuth[index] += (kResolutionInt * GetFiretimesCorrection(i, this->spin_speed_, operation_mode, angle_state, chn_unit->GetDistance()));
        }

        output.reflectivities[index] = chn_unit->GetReflectivity();
        output.distances[index] = chn_unit->GetDistance();
        output.elevation[index] = 0;
        ++chn_unit;
        ++index;
      }
    } else {
      // point to channel unit addr
      const HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4 *chn_unit_no_conf =
          reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4 *>((const unsigned char *)azimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4));

      // point to next block azimuth addr
      azimuth =
          reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_ME_V4 *>((const unsigned char *)azimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4) + (sizeof(HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4) * header->GetLaserNum()));

      for (uint8_t j = 0; j < header->GetLaserNum(); ++j) {
        output.azimuth[index] = u16_azimuth;
        if (this->get_firetime_file_) {
          output.azimuth[index] += (kResolutionInt * GetFiretimesCorrection(i, this->spin_speed_, operation_mode, angle_state, chn_unit_no_conf->GetDistance()));
        }

        output.reflectivities[index] = chn_unit_no_conf->GetReflectivity();
        output.distances[index] = chn_unit_no_conf->GetDistance();
        output.elevation[index] = 0;
        ++chn_unit_no_conf;
        ++index;
      }
    }

    if (IsNeedFrameSplit(u16_azimuth)) {
      output.scan_complete = true;
    }
    this->last_azimuth_ = u16_azimuth;
  }

  return 0;
}

template <typename T_Point>
bool SV_P128_Parser<T_Point>::IsNeedFrameSplit(uint16_t azimuth) {
  if (this->last_azimuth_ > azimuth && (this->last_azimuth_ - azimuth > kSplitFrameMinAngle)) {
    use_frame_start_azimuth_ = true;
  }
  // do not use frame_start_azimuth, split frame near 360 degree
  if (this->frame_start_azimuth_ < 1 || this->frame_start_azimuth_ > 359) {
    if (this->last_azimuth_ > azimuth && (this->last_azimuth_ - azimuth > kSplitFrameMinAngle)) {
      return true;
    }
    return false;
  } else {  // use frame_start_azimuth
    if ((azimuth >= uint16_t(this->frame_start_azimuth_ * kResolutionInt)) && (use_frame_start_azimuth_ == true)) {
      use_frame_start_azimuth_ = false;
      return true;
    }
    return false;
  }
}

template <typename T_Point>
void SV_P128_Parser<T_Point>::GetDistanceCorrection(int laser_id, float distance, int &azimuth, int &elevation) {
  double sin_delt_elevation = distance_correction_para_c_ / distance * this->sin_all_angle_[elevation];
  if (sin_delt_elevation >= -1 && sin_delt_elevation <= 1) {
    elevation -= distance_correction_para_a_ * std::asin(sin_delt_elevation) * kHalfCircleFloat / M_PI;
    elevation = elevation % CIRCLE;
  }
  double sin_delt_azimuth = distance_correction_para_c_ / distance / this->cos_all_angle_[elevation] *
                            this->sin_all_angle_[int((distance_correction_para_d_ + this->azimuth_collection_[laser_id]) * kResolutionInt) % CIRCLE];
  if (sin_delt_azimuth >= -1 && sin_delt_azimuth <= 1) {
    azimuth -= distance_correction_para_a_ * std::asin(sin_delt_azimuth) * kHalfCircleFloat / M_PI;
    azimuth = azimuth % CIRCLE;
  }
}

}  // namespace lidar
}  // namespace hesai