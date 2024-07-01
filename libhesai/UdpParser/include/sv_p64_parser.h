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

 private:
  const float kBlockOffsetSingle[6]{42.58f, 98.14f, 153.70001f, 209.26001f, 264.82f, 320.38f};
  const float kBlockOffsetDual[6]{42.58f, 42.58f, 98.14f, 98.14f, 153.70001f, 153.70001f};
  const float kFiringOffset[64]{23.18f, 21.876f, 20.572f, 19.268f, 17.964f, 16.66f, 11.444f, 46.796f, 7.532f, 36.956f,
                                50.732f, 54.668f, 40.892f, 44.828f, 31.052f, 34.988f, 48.764f, 52.7f, 38.924f, 42.86f,
                                29.084f, 33.02f, 46.796f, 25.148f, 36.956f, 50.732f, 27.116f, 40.892f, 44.828f, 31.052f,
                                34.988f, 48.764f, 25.148f, 38.924f, 42.86f, 29.084f, 33.02f, 52.7f, 6.228f, 54.668f,
                                15.356f, 27.116f, 10.14f, 23.18f, 4.924f, 21.876f, 14.052f, 17.964f, 8.836f, 19.268f,
                                3.62f, 20.572f, 12.748f, 16.66f, 7.532f, 11.444f, 6.228f, 15.356f, 10.14f, 4.924f,
                                3.62f, 14.052f, 8.836f, 12.748f};
};
}  // namespace lidar
}  // namespace hesai

#include "sv_p64_parser.cc"

#endif  // SV_P64_PARSER_H_
