#ifndef SV_P128_PARSER_H_
#define SV_P128_PARSER_H_

#include "general_parser.h"

namespace hesai {
namespace lidar {

typedef struct FiretimeSectionValues_s {
  typedef struct SectionValue_s {
    std::array<int, 2> firetime;
  } SectionValue_t;
  std::array<SectionValue_t, 8> section_values;
} FiretimeSectionValues_t;

// class SV_P128_Parser
// parsers packets and computes points for Pandar128
// you can parser the upd or pcap packets using the DocodePacket fuction
// you can compute xyzi of points using the ComputeXYZI fuction, which uses cpu to compute
template <typename T_Point>
class SV_P128_Parser : public GeneralParser<T_Point> {
 public:
  SV_P128_Parser();
  virtual ~SV_P128_Parser();

  // covert a origin udp packet to decoded packet, the decode function is in UdpParser module
  // udp_packet is the origin udp packet, output is the decoded packet
  virtual int DecodePacket(LidarDecodedPacket<T_Point>& output, const UdpPacket& udp_packet);

  // compute xyzi of points from decoded packet
  // param packet is the decoded packet; xyzi of points after computed is puted in frame
  virtual int ComputeXYZI(LidarDecodedFrame<T_Point>& frame, LidarDecodedPacket<T_Point>& packet);

  // get lidar firetime correction file from local file,and pass to udp parser
  virtual void LoadFiretimesFile(std::string firetimes_path);

  // compute lidar firetime correciton
  double GetFiretimesCorrection(int laser_id, double speed, uint8_t opt_mode, uint8_t angle_state, uint16_t dist);

  // determine whether frame splitting is needed
  bool IsNeedFrameSplit(uint16_t azimuth);

  // compute lidar distance correction
  void GetDistanceCorrection(int laser_id, float distance, int& azimuth, int& elevation);

  // Get angle correction file path
  std::string GetAngleCorrectionFilePath() override { return "/surf/bin/CorrectionFiles/angle_correction/pandar_128e3x_angle_correction.csv"; }
  // Get firetime correction file path
  std::string GetFiretimeCorrectionFilePath() override { return "/surf/bin/CorrectionFiles/firetime_correction/pandar_128e3x_firetime_correction.csv"; }

 private:
  static const int kLaserNum = 128;

  std::vector<float> block_offset_128_single_standard_;
  std::vector<float> block_offset_128_single_high_resolution_;
  float block_offset_128_dual_;

  uint8_t operation_mode_;

  double section_distance_;
  float distance_correction_para_a_;
  float distance_correction_para_b_;
  float distance_correction_para_h_;
  float distance_correction_para_c_;
  float distance_correction_para_d_;
  bool use_frame_start_azimuth_ = true;
  std::array<FiretimeSectionValues_t, kLaserNum> firetime_section_values;
};
}  // namespace lidar
}  // namespace hesai

#include "sv_p128_parser.cc"

#endif  // SV_P128_PARSER_H_
