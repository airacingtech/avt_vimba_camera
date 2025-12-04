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
#include <fstream>
#include <rclcpp/rclcpp.hpp>

namespace avt_vimba_camera
{

// GigE Vision packet structures
#pragma pack(push, 1)

struct EthernetHeader {
  uint8_t dest_mac[6];
  uint8_t src_mac[6];
  uint16_t ethertype;
};

struct IPv4Header {
  uint8_t version_ihl;
  uint8_t tos;
  uint16_t total_length;
  uint16_t identification;
  uint16_t flags_fragment;
  uint8_t ttl;
  uint8_t protocol;
  uint16_t checksum;
  uint32_t src_ip;
  uint32_t dest_ip;
};

struct UDPHeader {
  uint16_t src_port;
  uint16_t dest_port;
  uint16_t length;
  uint16_t checksum;
};

struct GVSPHeader {
  uint16_t status;
  uint16_t block_id;
  uint8_t format;
  uint32_t packet_id;
};

#pragma pack(pop)

// PCAP file header structures
#pragma pack(push, 1)

struct PcapFileHeader {
  uint32_t magic_number;
  uint16_t version_major;
  uint16_t version_minor;
  int32_t thiszone;
  uint32_t sigfigs;
  uint32_t snaplen;
  uint32_t network;
};

struct PcapPacketHeader {
  uint32_t ts_sec;
  uint32_t ts_usec;
  uint32_t incl_len;
  uint32_t orig_len;
};

#pragma pack(pop)

struct GigEFrame {
  std::vector<uint8_t> data;
  uint32_t width;
  uint32_t height;
  uint32_t pixel_format;
  uint64_t timestamp;
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
  bool isOpen() const { return is_open_; }
  void close();
  
  // Get frame statistics
  size_t getTotalFramesRead() const { return frames_read_; }
  
private:
  std::string filename_;
  std::string camera_ip_;
  std::ifstream pcap_file_;
  bool is_open_;
  rclcpp::Logger logger_;
  
  size_t frames_read_;
  uint32_t current_frame_id_;
  
  // Frame reassembly
  std::map<uint32_t, std::vector<uint8_t>> frame_packets_;
  std::map<uint32_t, uint32_t> frame_packet_count_;
  
  // Frame seeking support
  std::streampos file_start_pos_;
  std::vector<std::streampos> frame_positions_;  // Cache file positions for each frame
  bool caching_positions_;
  
  bool parseGVSPPacket(const uint8_t* packet_data, size_t packet_size, GigEFrame& frame);
  bool reassembleFrame(uint32_t frame_id, GigEFrame& frame);
  uint16_t ntohs_custom(uint16_t val);
  uint32_t ntohl_custom(uint32_t val);
};

}  // namespace avt_vimba_camera

#endif  // PCAP_READER_HPP
