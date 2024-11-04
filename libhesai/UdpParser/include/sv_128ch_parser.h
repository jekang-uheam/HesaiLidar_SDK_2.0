#ifndef SV_128CH_PARSER_H_
#define SV_128CH_PARSER_H_

#include "udp1_4_parser.h"

namespace hesai {
namespace lidar {

template <typename T_Point>
class SV128chParser : public Udp1_4Parser<T_Point> {
 public:
  SV128chParser();
  virtual ~SV128chParser();

  // covert a origin udp packet to decoded data, and pass the decoded data to a frame struct to reduce memory copy
  virtual int DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket &udpPacket) override;

  // compute xyzi of points from decoded packet
  // param packet is the decoded packet; xyzi of points after computed is puted in frame
  virtual int ComputeXYZI(LidarDecodedFrame<T_Point> &frame, int packet_index) override;

  // get lidar firetime correction file from local file,and pass to udp parser
  virtual void LoadFiretimesFile(std::string firetimes_path) override;

  using GeneralParser<T_Point>::GetFiretimesCorrection;

  // compute lidar firetime correciton
  double GetFiretimesCorrection(int laserId, double speed, uint8_t optMode, uint8_t angleState, float dist);

  // determine whether frame splitting is needed
  bool IsNeedFrameSplit(uint16_t azimuth);

  using GeneralParser<T_Point>::GetDistanceCorrection;

 private:
  static const int kLaserNum = 128;
  double section_distance;
  std::array<FiretimeSectionValues, kLaserNum> firetime_section_values;
  float distance_correction_para_a_;
  float distance_correction_para_b_;
  float distance_correction_para_h_;
  float distance_correction_para_c_;
  float distance_correction_para_d_;
  bool use_frame_start_azimuth_ = true;
};

}  // namespace lidar
}  // namespace hesai

#include "sv_128ch_parser.cc"

#endif  // SV_64CH_PARSER_H_