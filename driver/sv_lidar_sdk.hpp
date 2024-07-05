#ifndef SV_HESAI_LIDAR_SDK_H_
#define SV_HESAI_LIDAR_SDK_H_

#include "fault_message.h"
#include "lidar.h"

namespace hesai {
namespace lidar {

#ifndef VERSION_MAJOR
#define VERSION_MAJOR 2
#endif  // VERSION_MAJOR

#ifndef VERSION_MINOR
#define VERSION_MINOR 0
#endif  // VERSION_MINOR

#ifndef VERSION_TINY
#define VERSION_TINY 5
#endif  // VERSION_TINY

class SVLidarSdk {
 public:
  SVLidarSdk()
      : runing_thread_ptr_(nullptr), lidar_ptr_(nullptr), is_thread_runing_(false), packet_loss_tool_(false) {
    std::cout << "-------- SV Lidar SDK ver" << VERSION_MAJOR << "." << VERSION_MINOR << "." << VERSION_TINY << " --------" << std::endl;
  }

  ~SVLidarSdk() {
    Stop();
  }

  // init lidar with param. init logger, udp parser, source, ptc client, start receive/parser thread
  bool Init(const DriverParam& param) {
    /****************** Init decoder ******************/
    lidar_ptr_ = new Lidar<SVPoint_t>;
    if (nullptr == lidar_ptr_) {
      std::cout << "[Error] Create Lidar fail" << std::endl;
      return false;
    }

    // init lidar with param
    if (lidar_ptr_) {
      lidar_ptr_->Init(param);
    }

    // set packet_loss_tool
    packet_loss_tool_ = param.decoder_param.enable_packet_loss_tool;
    /****************** Init source *******************/
    if (param.input_param.source_type == 1) {
      u8Array_t s_data;
      for (int i = 0; i < 3; ++i) {
        int ret = lidar_ptr_->ptc_client_->GetCorrectionInfo(s_data);
        std::string correction_str = std::string((char*)s_data.data(), s_data.size());
        if (ret != 0 || lidar_ptr_->udp_parser_->LoadCorrectionString((char*)correction_str.c_str()) != 0) {
          lidar_ptr_->udp_parser_->LoadCorrectionFile(param.input_param.correction_file_path);
        } else {
          break;
        }
      }
    } else {
      lidar_ptr_->udp_parser_->LoadCorrectionFile(param.input_param.correction_file_path);
    }
    /**************************************************/
    return true;
  }

  // start process thread
  void Start() {
    runing_thread_ptr_ = new boost::thread(boost::bind(&SVLidarSdk::Run, this));
  }
  // stop process thread
  void Stop() {
    if (nullptr != runing_thread_ptr_) {
      is_thread_runing_ = false;
      if (runing_thread_ptr_->joinable()) {
        runing_thread_ptr_->join();
      }
      delete runing_thread_ptr_;
      runing_thread_ptr_ = nullptr;
    }

    if (nullptr != lidar_ptr_) {
      delete lidar_ptr_;
      lidar_ptr_ = nullptr;
    }
  }

  // process thread
  void Run() {
    int packet_index = 0;
    UdpFrame_t udp_packet_frame;
    LidarDecodedPacket<SVPoint_t> decoded_packet;
    UdpPacket packet;
    FaultMessageInfo fault_message_info;

    is_thread_runing_ = true;
    while (is_thread_runing_) {
      if (nullptr == lidar_ptr_) {
        std::cout << "[Error] Lidar is crashed" << std::endl;
        is_thread_runing_ = false;
        continue;
      }

      // get one packte from origin_packets_buffer_, which receive data from upd or pcap thread
      if (-1 == lidar_ptr_->GetOnePacket(packet)) {
        continue;
      }

      // get fault message
      if (kFaultMessageLength == packet.packet_len) {
        FaultMessageCallback(packet, fault_message_info);
        continue;
      }

      // get distance azimuth reflection, etc.and put them into decode_packet
      lidar_ptr_->DecodePacket(decoded_packet, packet);

      // do not compute xyzi of points if enable packet_loss_tool_
      if (true == packet_loss_tool_) {
        continue;
      }

      // one frame is receive completely, split frame
      if (decoded_packet.scan_complete) {
        // waiting for parser thread compute xyzi of points in the same frame
        while (!lidar_ptr_->ComputeXYZIComplete(decoded_packet.packet_index)) {
          std::this_thread::sleep_for(std::chrono::microseconds(100));
        }

        // log info, display frame message
        if (kMinPointsOfOneFrame < lidar_ptr_->frame_.points_num) {
          // publish point cloud topic
          if (point_cloud_cb_) {
            point_cloud_cb_(lidar_ptr_->frame_);
          }
          // publish upd packet topic
          if (pkt_cb_) {
            pkt_cb_(udp_packet_frame, lidar_ptr_->frame_.points[0].timestamp);
          }
        }

        // reset frame variable
        lidar_ptr_->frame_.Update();

        // clear udp packet vector
        udp_packet_frame.clear();
        packet_index = 0;

        // if the packet which contains split frame msgs is vaild, it will be the first packet of new frame
        if (decoded_packet.IsDecodedPacketValid()) {
          decoded_packet.packet_index = packet_index;

          lidar_ptr_->ComputeXYZI(decoded_packet);
          udp_packet_frame.emplace_back(packet);

          packet_index++;
        }
      } else {
        // new decoded packet of one frame, put it into decoded_packets_buffer_ and compute xyzi of points
        decoded_packet.packet_index = packet_index;

        lidar_ptr_->ComputeXYZI(decoded_packet);
        udp_packet_frame.emplace_back(packet);

        packet_index++;

        // update status manually if start a new frame failedly
        if (packet_index >= kMaxPacketNumPerFrame) {
          std::cout << "[Warn] fail to start a new frame (" << packet_index << " >= " << static_cast<int>(kMaxPacketNumPerFrame) << ")" << std::endl;

          udp_packet_frame.clear();
          lidar_ptr_->frame_.Update();

          packet_index = 0;
        }
      }
    }
  }
  // assign callback fuction
  void RegRecvDecodedFrameCallback(const std::function<void(const LidarDecodedFrame<SVPoint_t>&)>& callback) {
    point_cloud_cb_ = callback;
  }

  // assign callback fuction
  void RegRecvUdpFrameCallback(const std::function<void(const UdpFrame_t&, double)>& callback) {
    pkt_cb_ = callback;
  }

  // parsar fault message
  void FaultMessageCallback(UdpPacket& udp_packet, FaultMessageInfo& fault_message_info) {
    FaultMessageVersion3* fault_message_ptr = reinterpret_cast<FaultMessageVersion3*>(&(udp_packet.buffer[0]));
    if (nullptr != fault_message_ptr) {
      fault_message_ptr->ParserFaultMessage(fault_message_info);
    }
  }

  const Lidar<SVPoint_t>* lidar_ptr() { return lidar_ptr_; }

 private:
  boost::thread* runing_thread_ptr_;
  Lidar<SVPoint_t>* lidar_ptr_;
  bool is_thread_runing_;
  bool packet_loss_tool_;

  std::function<void(const UdpFrame_t&, double)> pkt_cb_;
  std::function<void(const LidarDecodedFrame<SVPoint_t>&)> point_cloud_cb_;
};

}  // namespace lidar
}  // namespace hesai

#endif  // SV_HESAI_LIDAR_SDK_H_