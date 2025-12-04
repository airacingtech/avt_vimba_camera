/// PCAP reader implementation for GigE Vision camera packets

#include "avt_vimba_camera/pcap_reader.hpp"
#include <arpa/inet.h>
#include <cstring>

namespace avt_vimba_camera
{

PcapReader::PcapReader(const std::string& filename, const std::string& camera_ip, rclcpp::Logger logger)
  : filename_(filename), camera_ip_(camera_ip), is_open_(false), logger_(logger), frames_read_(0), current_frame_id_(0),
    caching_positions_(true)
{
}

PcapReader::~PcapReader()
{
  close();
}

bool PcapReader::open()
{
  pcap_file_.open(filename_, std::ios::binary);
  if (!pcap_file_.is_open())
  {
    RCLCPP_ERROR(logger_, "Failed to open PCAP file: %s", filename_.c_str());
    return false;
  }

  PcapFileHeader file_header;
  pcap_file_.read(reinterpret_cast<char*>(&file_header), sizeof(PcapFileHeader));
  
  if (!pcap_file_.good())
  {
    RCLCPP_ERROR(logger_, "Failed to read PCAP file header");
    pcap_file_.close();
    return false;
  }

  if (file_header.magic_number != 0xa1b2c3d4 && file_header.magic_number != 0xd4c3b2a1)
  {
    RCLCPP_ERROR(logger_, "Invalid PCAP file format (bad magic: 0x%08x)", file_header.magic_number);
    pcap_file_.close();
    return false;
  }

  is_open_ = true;
  file_start_pos_ = pcap_file_.tellg();
  RCLCPP_INFO(logger_, "PCAP replay: %s (filtering IP: %s)", 
              filename_.c_str(), camera_ip_.empty() ? "none" : camera_ip_.c_str());
  return true;
}

void PcapReader::close()
{
  if (pcap_file_.is_open())
  {
    pcap_file_.close();
  }
  is_open_ = false;
  frame_packets_.clear();
  frame_packet_count_.clear();
}

bool PcapReader::parseGVSPPacket(const uint8_t* packet_data, size_t packet_size, GigEFrame& frame)
{
  if (packet_size < 50) return false;

  const uint8_t* ip_packet = packet_data + 14;
  const IPv4Header* ip_header = reinterpret_cast<const IPv4Header*>(ip_packet);
  
  if (!camera_ip_.empty())
  {
    uint32_t src_ip = ntohl(ip_header->src_ip);
    char ip_str[16];
    snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d",
             (src_ip >> 24) & 0xFF, (src_ip >> 16) & 0xFF,
             (src_ip >> 8) & 0xFF, src_ip & 0xFF);
    
    if (camera_ip_ != ip_str) return false;
  }
  
  if (ip_header->protocol != 17) return false;
  
  uint8_t ip_header_len = (ip_header->version_ihl & 0x0F) * 4;
  const uint8_t* gvsp_data = ip_packet + ip_header_len + 8;
  size_t gvsp_size = packet_size - 14 - ip_header_len - 8;
  
  if (gvsp_size < 8) return false;
  
  uint16_t block_id = ntohs(*reinterpret_cast<const uint16_t*>(gvsp_data + 2));
  uint8_t packet_type = gvsp_data[4];
  uint32_t frame_id = block_id;
  
  switch (packet_type)
  {
    case 0x01:  // Leader
      frame_packets_[frame_id].clear();
      frame_packet_count_[frame_id] = 0;
      break;
      
    case 0x02:  // Trailer
      if (frame_packets_.find(frame_id) != frame_packets_.end())
      {
        return reassembleFrame(frame_id, frame);
      }
      break;
      
    case 0x03:  // Data
    {
      const uint8_t* payload = gvsp_data + 8;
      size_t payload_size = gvsp_size - 8;
      
      if (frame_packets_.find(frame_id) == frame_packets_.end())
      {
        frame_packets_[frame_id] = std::vector<uint8_t>();
        frame_packet_count_[frame_id] = 0;
      }
      
      frame_packets_[frame_id].insert(frame_packets_[frame_id].end(), 
                                       payload, payload + payload_size);
      frame_packet_count_[frame_id]++;
      break;
    }
    
    case 0x04:  // All-in-one
    {
      const uint8_t* payload = gvsp_data + 8;
      size_t payload_size = gvsp_size - 8;
      frame.data.assign(payload, payload + payload_size);
      frame.frame_id = frame_id;
      return true;
    }
  }
  
  return false;
}

bool PcapReader::reassembleFrame(uint32_t frame_id, GigEFrame& frame)
{
  if (frame_packets_.find(frame_id) == frame_packets_.end()) return false;
  
  frame.data = frame_packets_[frame_id];
  frame.frame_id = frame_id;
  frame.width = 0;
  frame.height = 0;
  frame.pixel_format = 0;
  frame.timestamp = 0;
  
  frame_packets_.erase(frame_id);
  frame_packet_count_.erase(frame_id);
  frames_read_++;
  
  return true;
}

bool PcapReader::readNextFrame(GigEFrame& frame)
{
  if (!is_open_) return false;
  
  // Cache the position at the start of this frame
  std::streampos frame_start = pcap_file_.tellg();
  bool frame_position_cached = false;
  
  while (pcap_file_.good())
  {
    PcapPacketHeader packet_header;
    pcap_file_.read(reinterpret_cast<char*>(&packet_header), sizeof(PcapPacketHeader));
    
    if (!pcap_file_.good())
    {
      if (pcap_file_.eof())
      {
        RCLCPP_INFO(logger_, "PCAP replay complete: %zu frames", frames_read_);
      }
      return false;
    }
    
    std::vector<uint8_t> packet_data(packet_header.incl_len);
    pcap_file_.read(reinterpret_cast<char*>(packet_data.data()), packet_header.incl_len);
    
    if (!pcap_file_.good()) return false;
    
    if (parseGVSPPacket(packet_data.data(), packet_data.size(), frame))
    {
      // Cache this frame's position on first read
      if (caching_positions_ && !frame_position_cached)
      {
        frame_positions_.push_back(frame_start);
        frame_position_cached = true;
      }
      return true;
    }
  }
  
  return false;
}

bool PcapReader::seekToFrame(int target_frame_index)
{
  if (!is_open_) return false;
  if (target_frame_index < 0) return false;
  
  // Clear current frame assembly state
  frame_packets_.clear();
  frame_packet_count_.clear();
  
  // If we have cached position for this frame, use it directly
  if (target_frame_index < static_cast<int>(frame_positions_.size()))
  {
    pcap_file_.clear();
    pcap_file_.seekg(frame_positions_[target_frame_index]);
    frames_read_ = target_frame_index;
    return true;
  }
  
  // Otherwise, seek from the closest cached position or start
  pcap_file_.clear();
  int start_frame = 0;
  
  if (!frame_positions_.empty() && target_frame_index >= static_cast<int>(frame_positions_.size()))
  {
    // Start from last cached position
    start_frame = frame_positions_.size() - 1;
    pcap_file_.seekg(frame_positions_[start_frame]);
    frames_read_ = start_frame;
  }
  else
  {
    // Start from beginning
    pcap_file_.seekg(file_start_pos_);
    frames_read_ = 0;
  }
  
  current_frame_id_ = 0;
  
  // Read frames to reach target
  GigEFrame dummy_frame;
  for (int i = start_frame; i < target_frame_index; ++i)
  {
    if (!readNextFrame(dummy_frame))
    {
      return false;
    }
  }
  
  return true;
}

}  // namespace avt_vimba_camera
