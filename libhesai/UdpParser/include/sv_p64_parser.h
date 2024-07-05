#ifndef SV_P64_PARSER_H_
#define SV_P64_PARSER_H_

#include "general_parser.h"

namespace hesai {
namespace lidar {

// class SV_P64_Parser
// parsers packets and computes points for Pandar64
// you can parser the upd or pcap packets using the DocodePacket fuction
// you can compute xyzi of points using the ComputeXYZI fuction, which uses cpu to compute
template <typename T_Point>
class SV_P64_Parser : public GeneralParser<T_Point> {
 public:
  SV_P64_Parser();
  virtual ~SV_P64_Parser();

  // covert a origin udp packet to decoded packet, the decode function is in UdpParser module
  // udp_packet is the origin udp packet, output is the decoded packet
  virtual int DecodePacket(LidarDecodedPacket<T_Point>& output, const UdpPacket& udp_packet);

  // compute xyzi of points from decoded packet
  // param packet is the decoded packet; xyzi of points after computed is puted in frame
  virtual int ComputeXYZI(LidarDecodedFrame<T_Point>& frame, LidarDecodedPacket<T_Point>& packet);

  // determine whether frame splitting is needed
  bool IsNeedFrameSplit(uint16_t azimuth);

  // Get angle correction file path
  std::string GetAngleCorrectionFilePath() override { return "/surf/bin/CorrectionFiles/angle_correction/pandar_64_angle_correction.csv"; }
  // Get firetime correction file path
  std::string GetFiretimeCorrectionFilePath() override { return "/surf/bin/CorrectionFiles/firetime_correction/pandar_64_firetime_correction.csv"; }

 private:
  std::vector<float> block_offset_64_single_;
  std::vector<float> block_offset_64_dual_;
  std::vector<float> firing_offset_64_;
};
}  // namespace lidar
}  // namespace hesai

#include "sv_p64_parser.cc"

#endif  // SV_P64_PARSER_H_
