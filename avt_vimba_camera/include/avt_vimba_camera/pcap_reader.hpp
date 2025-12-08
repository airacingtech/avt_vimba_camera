/// Copyright (c) 2024
/// PCAP reader for GigE Vision camera packets
///
/// This file provides PCAP replay functionality for Allied Vision GigE cameras

#ifndef PCAP_READER_HPP
#define PCAP_READER_HPP

#include <string>
#include <vector>
#include <memory>
#include <cstdint>
#include <map>
#include <pcap.h>
#include <rclcpp/rclcpp.hpp>

namespace avt_vimba_camera
{

struct GigEFrame {
  std::vector<uint8_t> data;
  uint32_t frame_id;
};

class PcapReader
{
public:
  PcapReader(const std::string& filename, const std::string& camera_ip, rclcpp::Logger logger);
  ~PcapReader();

  bool open();
  bool readNextFrame(GigEFrame& frame);
  bool seekToFrame(int target_frame_index);
  bool isOpen() const { return pcap_handle_ != nullptr; }
  void close();
  
  size_t getTotalFramesRead() const { return frames_read_; }
  
private:
  std::string filename_;
  std::string camera_ip_;
  pcap_t* pcap_handle_;
  rclcpp::Logger logger_;
  
  size_t frames_read_;
  
  // Frame reassembly
  std::map<uint32_t, std::vector<uint8_t>> frame_packets_;
  
  // Frame seeking support
  std::vector<long> frame_positions_;
  bool caching_positions_;
  GigEFrame* pending_frame_;
  bool frame_ready_;
  
  bool parseGVSPPacket(const uint8_t* packet_data, size_t packet_size, GigEFrame& frame);
  bool reassembleFrame(uint32_t frame_id, GigEFrame& frame);
  void clearStaleFrames();
  
  static void packetHandler(u_char* user, const struct pcap_pkthdr* header, const u_char* packet);
};

}  // namespace avt_vimba_camera

#endif  // PCAP_READER_HPP
