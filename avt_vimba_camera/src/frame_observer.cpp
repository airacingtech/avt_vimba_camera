/// Copyright (c) 2014,
/// Systems, Robotics and Vision Group
/// University of the Balearic Islands
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///     * Redistributions of source code must retain the above copyright
///       notice, this list of conditions and the following disclaimer.
///     * Redistributions in binary form must reproduce the above copyright
///       notice, this list of conditions and the following disclaimer in the
///       documentation and/or other materials provided with the distribution.
///     * All advertising materials mentioning features or use of this software
///       must display the following acknowledgement:
///       This product includes software developed by
///       Systems, Robotics and Vision Group, Univ. of the Balearic Islands
///     * Neither the name of Systems, Robotics and Vision Group, University of
///       the Balearic Islands nor the names of its contributors may be used
///       to endorse or promote products derived from this software without
///       specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
/// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
/// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "avt_vimba_camera/frame_observer.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>

FrameObserver::FrameObserver(CameraPtr cam_ptr, Callback callback)
  : IFrameObserver(cam_ptr), cam_ptr_(cam_ptr), callback_(callback)
{
}

void FrameObserver::FrameReceived(const FramePtr vimba_frame_ptr)
{
  VmbFrameStatusType eReceiveStatus;
  VmbErrorType err = vimba_frame_ptr->GetReceiveStatus(eReceiveStatus);
  if (err == VmbErrorSuccess)
  {
    switch (eReceiveStatus)
    {
      case VmbFrameStatusComplete: {
        // Call the callback
        callback_(vimba_frame_ptr);
        break;
      }
      case VmbFrameStatusIncomplete: {
        // Rate-limited on purpose. Incomplete frames arrive in floods exactly when the process is
        // already CPU-starved (GVSP packets get dropped because reassembly cannot keep up), and an
        // unbuffered std::endl per bad frame down a tmux pipe turns the symptom into a second
        // cause. Report a running total once a second instead.
        static std::atomic<uint64_t> incomplete{ 0 };
        static std::atomic<int64_t> next_report{ 0 };
        const uint64_t n = ++incomplete;
        const int64_t now_s = static_cast<int64_t>(
            std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
        int64_t due = next_report.load(std::memory_order_relaxed);
        if (now_s >= due && next_report.compare_exchange_strong(due, now_s + 1))
        {
          std::cout << "ERR: FrameObserver VmbFrameStatusIncomplete (total " << n << ")"
                    << std::endl;
        }
        break;
      }
      case VmbFrameStatusTooSmall: {
        std::cout << "ERR: FrameObserver VmbFrameStatusTooSmall" << std::endl;
        break;
      }
      case VmbFrameStatusInvalid: {
        std::cout << "ERR: FrameObserver VmbFrameStatusInvalid" << std::endl;
        break;
      }
      default: {
        std::cout << "ERR: FrameObserver no known status" << std::endl;
        break;
      }
    }
  }

  cam_ptr_->QueueFrame(vimba_frame_ptr);
}
