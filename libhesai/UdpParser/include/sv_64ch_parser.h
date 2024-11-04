#ifndef SV_64CH_PARSER_H_
#define SV_64CH_PARSER_H_

#include "udp_p64_parser.h"

namespace hesai {
namespace lidar {

template <typename T_Point>
class SV64chParser : public UdpP64Parser<T_Point> {
 public:
  SV64chParser();
  virtual ~SV64chParser();

  // covert a origin udp packet to decoded data, and pass the decoded data to a frame struct to reduce memory copy
  virtual int DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket &udpPacket) override;
  // compute xyzi of points from decoded packet
  // param packet is the decoded packet; xyzi of points after computed is puted in frame
  virtual int ComputeXYZI(LidarDecodedFrame<T_Point> &frame, int packet_index) override;

  using GeneralParser<T_Point>::GetDistanceCorrection;

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

#include "sv_64ch_parser.cc"

#endif  // SV_64CH_PARSER_H_