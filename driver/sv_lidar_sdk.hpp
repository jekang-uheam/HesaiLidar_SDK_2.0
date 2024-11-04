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
      : lidar_ptr_(nullptr), init_thread_ptr_(nullptr), source_type_(DATA_FROM_PCAP), runing_thread_ptr_(nullptr), is_thread_runing_(false), packet_loss_tool_(false), device_ip_address_(0), device_udp_port_(0), device_fault_port_(0), first_timestamp_(0) {
    std::cout << "-------- SV Lidar SDK ver" << VERSION_MAJOR << "." << VERSION_MINOR << "." << VERSION_TINY << " --------" << std::endl;
  }

  ~SVLidarSdk() {
    Stop();
  }

  void Init_thread() {
    packet_loss_tool_ = param_.decoder_param.enable_packet_loss_tool;
    // init lidar with param
    if (nullptr != lidar_ptr_) {
      lidar_ptr_->Init(param_);
      lidar_ptr_->init_finish_[AllInitFinish] = true;
      LogDebug("finish 3: Initialisation all complete !!!");
    }
  }

  // init lidar with param. init logger, udp parser, source, ptc client, start receive/parser thread
  bool Init(const DriverParam& param) {
    /****************** Init decoder ******************/
    lidar_ptr_ = new Lidar<SVPoint_t>;
    if (nullptr == lidar_ptr_) {
      std::cout << "[Error] Create Lidar fail" << std::endl;
      return false;
    }

    source_type_ = param.input_param.source_type;
    param_ = param;
    init_thread_ptr_ = new std::thread(std::bind(&SVLidarSdk::Init_thread, this));
    device_ip_address_ = inet_addr(param.input_param.device_ip_address.c_str());
    if (param.input_param.device_fault_port >= 1024 && param.input_param.device_fault_port <= 65535 && device_ip_address_ != INADDR_NONE) {
      device_fault_port_ = param.input_param.device_fault_port;
    } else {
      device_fault_port_ = 0;
    }
    if (param.input_param.device_udp_src_port >= 1024 && param.input_param.device_udp_src_port <= 65535 && device_ip_address_ != INADDR_NONE) {
      device_udp_port_ = param.input_param.device_udp_src_port;
    } else {
      device_udp_port_ = 0;
    }
    /**************************************************/
    return true;
  }

  // start process thread
  void Start() {
    while (1) {
      if ((source_type_ == DATA_FROM_LIDAR && lidar_ptr_->init_finish_[FaultMessParse]) || lidar_ptr_->init_finish_[AllInitFinish]) {
        runing_thread_ptr_ = new std::thread(std::bind(&SVLidarSdk::Run, this));
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
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

  void onRelease() { is_thread_runing_ = false; }

  // process thread
  void Run() {
    LogInfo("--------begin to prase udp package--------");
    is_thread_runing_ = true;
    UdpFrame_t udp_packet_frame;
    lidar_ptr_->frame_.use_timestamp_type = lidar_ptr_->use_timestamp_type_;
    lidar_ptr_->frame_.config.fov_start = lidar_ptr_->fov_start_;
    lidar_ptr_->frame_.config.fov_end = lidar_ptr_->fov_end_;
    uint32_t packet_index = 0;
    // uint32_t start = GetMicroTickCount();
    UdpPacket packet;
    FaultMessageInfo fault_message_info;

    uint32_t total_packet_count;
    uint32_t total_packet_loss_count;
    while (is_thread_runing_) {
      // get one packte from origin_packets_buffer_, which receive data from upd or pcap thread
      int ret = lidar_ptr_->GetOnePacket(packet);
      if (ret == -1) continue;

      // get fault message
      if (packet.packet_len == kFaultMessageLength) {
        if (device_fault_port_ != 0) {
          if (device_ip_address_ != packet.ip || device_fault_port_ != packet.port) {
            continue;
          }
        }
        ret = lidar_ptr_->ParserFaultMessage(packet, fault_message_info);
        if (ret == 0) {
          if (fault_message_cb_)
            fault_message_cb_(fault_message_info);
        }
        continue;
      }

      // Wait for initialization to complete before starting to parse the point cloud
      if (!lidar_ptr_->init_finish_[PointCloudParse]) continue;

      if (device_udp_port_ != 0 && (packet.is_timeout == false || packet_index == 0)) {
        if (device_ip_address_ != packet.ip || device_udp_port_ != packet.port) {
          continue;
        }
      }
      // get distance azimuth reflection, etc.and put them into decode_packet
      if (lidar_ptr_->DecodePacket(lidar_ptr_->frame_, packet) != 0) {
        continue;
      }

      // do not compute xyzi of points if enable packet_loss_tool_
      //  if(packet_loss_tool_ == true) continue;

      // one frame is receive completely, split frame
      if (lidar_ptr_->frame_.scan_complete) {
        // If it's not a timeout split frame, it will be one more packet
        bool last_packet_is_valid = (lidar_ptr_->frame_.packet_num != packet_index);
        lidar_ptr_->frame_.packet_num = packet_index;

        // waiting for parser thread compute xyzi of points in the same frame
        while (!lidar_ptr_->ComputeXYZIComplete(packet_index)) std::this_thread::sleep_for(std::chrono::microseconds(100));

        // log info, display frame message
        if (lidar_ptr_->frame_.points_num > kMinPointsOfOneFrame) {
          // publish point cloud topic
          if (point_cloud_cb_) point_cloud_cb_(lidar_ptr_->frame_);

          // publish upd packet topic
          if (pkt_cb_) {
            double timestamp = 0;
            getTimestamp(lidar_ptr_->frame_.points[0], timestamp);
            pkt_cb_(udp_packet_frame, timestamp);
          }

          if (pkt_loss_cb_) {
            total_packet_count = lidar_ptr_->udp_parser_->GetGeneralParser()->total_packet_count_;
            total_packet_loss_count = lidar_ptr_->udp_parser_->GetGeneralParser()->seqnum_loss_message_.total_loss_count;
            pkt_loss_cb_(total_packet_count, total_packet_loss_count);
          }
          if (ptp_cb_ && lidar_ptr_->frame_.frame_index % 100 == 1) {
            u8Array_t ptp_status;
            u8Array_t ptp_lock_offset;
            int ret_status = lidar_ptr_->ptc_client_->GetPTPDiagnostics(ptp_status, 1);  // ptp_query_type = 1
            int ret_offset = lidar_ptr_->ptc_client_->GetPTPLockOffset(ptp_lock_offset);
            if (ret_status != 0 || ret_offset != 0) {
              LogInfo("-->%d %d %zu %zu", ret_status, ret_offset, ptp_status.size(), ptp_lock_offset.size());
            } else {
              ptp_cb_(ptp_lock_offset.front(), ptp_status);
            }
          }
          if (correction_cb_ && lidar_ptr_->frame_.frame_index % 1000 == 1) {
            correction_cb_(lidar_ptr_->correction_string_);
          }
        }
        // reset frame variable
        lidar_ptr_->frame_.Update();
        // clear udp packet vector
        udp_packet_frame.clear();
        packet_index = 0;

        // if the packet which contains split frame msgs is valid, it will be the first packet of new frame
        if (last_packet_is_valid) {
          lidar_ptr_->DecodePacket(lidar_ptr_->frame_, packet);
          lidar_ptr_->ComputeXYZI(packet_index);
          lidar_ptr_->frame_.points_num += lidar_ptr_->frame_.per_points_num;
          udp_packet_frame.emplace_back(packet);
          packet_index++;
        }
      } else {
        // new decoded packet of one frame, put it into decoded_packets_buffer_ and compute xyzi of points
        if (lidar_ptr_->frame_.packet_num != packet_index) {
          lidar_ptr_->ComputeXYZI(packet_index);
          lidar_ptr_->frame_.points_num += lidar_ptr_->frame_.per_points_num;
          udp_packet_frame.emplace_back(packet);
          packet_index++;
        }

        // update status manually if start a new frame failedly
        if (packet_index >= kMaxPacketNumPerFrame) {
          packet_index = 0;
          udp_packet_frame.clear();
          lidar_ptr_->ClearPacketBuffer();
          std::this_thread::sleep_for(std::chrono::microseconds(100));
          lidar_ptr_->frame_.Update();
          LogError("fail to start a new frame");
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
  void RegRecvCallback(const std::function<void(const uint32_t&, const uint32_t&)>& callback) {
    pkt_loss_cb_ = callback;
  }
  void RegRecvCallback(const std::function<void(const uint8_t&, const u8Array_t&)>& callback) {
    ptp_cb_ = callback;
  }
  void RegRecvCallback(const std::function<void(const FaultMessageInfo&)>& callback) {
    fault_message_cb_ = callback;
  }

  const Lidar<SVPoint_t>* lidar_ptr() { return lidar_ptr_; }

 private:
  Lidar<SVPoint_t>* lidar_ptr_;
  DriverParam param_;
  std::thread* init_thread_ptr_;
  SourceType source_type_;

  std::thread* runing_thread_ptr_;
  bool is_thread_runing_;
  bool packet_loss_tool_;
  uint32_t device_ip_address_;
  uint16_t device_udp_port_;
  uint16_t device_fault_port_;

  std::function<void(const UdpFrame_t&, double)> pkt_cb_;
  std::function<void(const LidarDecodedFrame<SVPoint_t>&)> point_cloud_cb_;
  std::function<void(const u8Array_t&)> correction_cb_;
  std::function<void(const uint32_t&, const uint32_t&)> pkt_loss_cb_;
  std::function<void(const uint8_t&, const u8Array_t&)> ptp_cb_;
  std::function<void(const FaultMessageInfo&)> fault_message_cb_;

  double first_timestamp_;
};

}  // namespace lidar
}  // namespace hesai

#endif  // SV_HESAI_LIDAR_SDK_H_