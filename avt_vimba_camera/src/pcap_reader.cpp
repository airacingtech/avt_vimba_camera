/// PCAP reader implementation for GigE Vision camera packets using libpcap

#include "avt_vimba_camera/pcap_reader.hpp"
#include <arpa/inet.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <cstring>

namespace avt_vimba_camera
{

PcapReader::PcapReader(const std::string& filename, const std::string& camera_ip, rclcpp::Logger logger)
  : filename_(filename), camera_ip_(camera_ip), pcap_handle_(nullptr), logger_(logger), 
    frames_read_(0), caching_positions_(true), pending_frame_(nullptr), frame_ready_(false)
{
}

PcapReader::~PcapReader()
{
  close();
}

bool PcapReader::open()
{
  char errbuf[PCAP_ERRBUF_SIZE];
  pcap_handle_ = pcap_open_offline(filename_.c_str(), errbuf);
  
  if (!pcap_handle_)
  {
    RCLCPP_ERROR(logger_, "Failed to open PCAP file: %s - %s", filename_.c_str(), errbuf);
    return false;
  }

  RCLCPP_INFO(logger_, "PCAP replay: %s (filtering IP: %s)", 
              filename_.c_str(), camera_ip_.empty() ? "none" : camera_ip_.c_str());
  return true;
}

void PcapReader::close()
{
  if (pcap_handle_)
  {
    pcap_close(pcap_handle_);
    pcap_handle_ = nullptr;
  }
  frame_packets_.clear();
}

bool PcapReader::parseGVSPPacket(const uint8_t* packet_data, size_t packet_size, GigEFrame& frame)
{
  if (packet_size < 42) return false;  // Ethernet + IP + UDP headers minimum

  const uint8_t* ip_packet = packet_data + 14;  // Skip Ethernet header
  const struct ip* ip_header = reinterpret_cast<const struct ip*>(ip_packet);
  
  if (!camera_ip_.empty())
  {
    char ip_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(ip_header->ip_src), ip_str, INET_ADDRSTRLEN);
    if (camera_ip_ != ip_str) return false;
  }
  
  if (ip_header->ip_p != IPPROTO_UDP) return false;
  
  size_t ip_header_len = ip_header->ip_hl * 4;
  const uint8_t* udp_packet = ip_packet + ip_header_len;
  const uint8_t* gvsp_data = udp_packet + 8;  // Skip UDP header
  size_t gvsp_size = packet_size - 14 - ip_header_len - 8;
  
  // Ensure we have at least 8 bytes for GVSP header
  if (gvsp_size < 8) return false;
  // Ensure packet_size is large enough to access all header bytes safely
  if (packet_size < 14 + ip_header_len + 8 + 8) return false;
  
  uint8_t packet_type = gvsp_data[4];
  
  // For GigE Vision GVSP, use block_id (packet ID) from bytes 2-3 as frame grouping
  // Many cameras use block_id to group packets from the same frame
  uint16_t block_id = (static_cast<uint16_t>(gvsp_data[2]) << 8) | gvsp_data[3];
  uint32_t frame_id = block_id;
  
  switch (packet_type)
  {
    case 0x01:  // Leader
    {
      frame_packets_[frame_id].clear();
      // The image leader carries pixel format and dimensions; cache them so the frame can
      // describe itself instead of relying on a hand-configured resolution.
      const uint8_t* leader = gvsp_data + 8;
      if (gvsp_size >= 8 + 32)
      {
        GigEFrame& geom = frame_geometry_[frame_id];
        geom.pixel_format = (static_cast<uint32_t>(leader[12]) << 24) |
                            (static_cast<uint32_t>(leader[13]) << 16) |
                            (static_cast<uint32_t>(leader[14]) << 8) | leader[15];
        geom.width = (static_cast<uint32_t>(leader[16]) << 24) |
                     (static_cast<uint32_t>(leader[17]) << 16) |
                     (static_cast<uint32_t>(leader[18]) << 8) | leader[19];
        geom.height = (static_cast<uint32_t>(leader[20]) << 24) |
                      (static_cast<uint32_t>(leader[21]) << 16) |
                      (static_cast<uint32_t>(leader[22]) << 8) | leader[23];
      }
      clearStaleFrames();
      break;
    }
      
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
      
      if (payload_size > 0)
      {
        frame_packets_[frame_id].insert(frame_packets_[frame_id].end(), 
                                         payload, payload + payload_size);
      }
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
  auto it = frame_packets_.find(frame_id);
  if (it == frame_packets_.end() || it->second.empty()) return false;
  
  frame.data = std::move(it->second);
  frame.frame_id = frame_id;

  auto geom = frame_geometry_.find(frame_id);
  if (geom != frame_geometry_.end())
  {
    frame.width = geom->second.width;
    frame.height = geom->second.height;
    frame.pixel_format = geom->second.pixel_format;
    frame_geometry_.erase(geom);
  }

  frame_packets_.erase(it);
  frames_read_++;
  
  return true;
}

void PcapReader::clearStaleFrames()
{
  if (frame_packets_.size() > 10)
  {
    frame_packets_.clear();
  }
}

void PcapReader::packetHandler(u_char* user, const struct pcap_pkthdr* header, const u_char* packet)
{
  PcapReader* reader = reinterpret_cast<PcapReader*>(user);
  if (!reader || !reader->pending_frame_) return;
  
  if (reader->parseGVSPPacket(packet, header->len, *reader->pending_frame_))
  {
    reader->frame_ready_ = true;
    if (reader->caching_positions_)
    {
      reader->frame_positions_.push_back(reader->frames_read_);
    }
  }
}

bool PcapReader::readNextFrame(GigEFrame& frame)
{
  if (!pcap_handle_) return false;
  
  pending_frame_ = &frame;
  frame_ready_ = false;
  
  while (!frame_ready_)
  {
    int result = pcap_dispatch(pcap_handle_, 1, packetHandler, reinterpret_cast<u_char*>(this));
    if (result == 0)
    {
      RCLCPP_INFO(logger_, "PCAP replay complete: %zu frames", frames_read_);
      return false;
    }
    else if (result == PCAP_ERROR)
    {
      RCLCPP_ERROR(logger_, "Error reading PCAP: %s", pcap_geterr(pcap_handle_));
      return false;
    }
  }
  
  pending_frame_ = nullptr;
  return true;
}

bool PcapReader::seekToFrame(int target_frame_index)
{
  if (!pcap_handle_) return false;
  if (target_frame_index < 0) return false;
  
  frame_packets_.clear();
  
  if (target_frame_index < static_cast<int>(frame_positions_.size()))
  {
    char errbuf[PCAP_ERRBUF_SIZE];
    pcap_close(pcap_handle_);
    pcap_handle_ = pcap_open_offline(filename_.c_str(), errbuf);
    
    if (!pcap_handle_) return false;
    
    frames_read_ = target_frame_index;
    
    GigEFrame dummy;
    for (int i = 0; i < target_frame_index; ++i)
    {
      if (!readNextFrame(dummy)) return false;
    }
    return true;
  }
  
  GigEFrame dummy;
  int current = frames_read_;
  for (int i = current; i < target_frame_index; ++i)
  {
    if (!readNextFrame(dummy)) return false;
  }
  
  return true;
}

}  // namespace avt_vimba_camera
