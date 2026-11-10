/**
 * @file usb_sim_profiles.hpp
 * @brief 仿真后端的硬件档位 / Hardware profiles of the simulation backend.
 *
 * 每个档位只表达后端经契约暴露的限制：有哪些端点号和方向、槽内存、每个端点的 FIFO/PMA
 * 包长上限、是否支持等时端点、同号 IN/OUT 是否共用类型、TX FIFO 占用数，以及停止是立即
 * 完成还是待决。端点布局取自使用该控制器的 BSP。
 * Each profile expresses only the limits a backend exposes through the contract: the
 * endpoint numbers and directions, the slot memory, each endpoint's FIFO or PMA packet
 * limit, isochronous support, whether IN and OUT of one number share the type, the TX
 * FIFO count, and whether a stop completes at once or stays pending. The endpoint
 * layouts come from BSPs that use the controller.
 *
 * 槽内存按主机的缓存行（64 B）取整，小于它的 BSP 缓冲用包长上限表达 FIFO/PMA 大小。
 * Slot memory is rounded to the host cache line (64 B); a smaller BSP buffer is expressed
 * through the packet limit of its FIFO or PMA.
 */

#pragma once

#include "usb_sim.hpp"

namespace LibXR::USB::Sim
{

/// STM32F103 FSDEV（bsp_stm32f103）：EP1 双向 128 B，EP2 只有 IN 16 B，PMA 包长上限
/// 64/16；不支持等时；同号 IN/OUT 共用 EP_TYPE；停止立即完成。
/// STM32F103 FSDEV (bsp_stm32f103): EP1 both directions with 128 B, EP2 IN only with
/// 16 B, PMA packet limits 64/16; no isochronous; IN and OUT of one number share
/// EP_TYPE; stops complete at once.
inline Profile FsdevF103Profile()
{
  Profile profile;
  profile.sof_policy = SofPolicy::AT_STOPS;
  profile.endpoints = {{1, Endpoint::Direction::BOTH, 128, 0, 64},
                       {2, Endpoint::Direction::IN, 128, 0, 16}};
  profile.ep0_buffer = 64;
  profile.isochronous = false;
  profile.shared_type_per_number = true;
  profile.stop_mode = StopMode::IMMEDIATE;
  return profile;
}

/// STM32F407 OTG FS（bsp-dev-c）：RX FIFO 256 B 供所有 OUT，IN FIFO EP1 128 B、EP2 16 B；
/// FIFO 大小即包长上限；停止待决，由中断确认。
/// STM32F407 OTG FS (bsp-dev-c): a 256 B RX FIFO shared by every OUT, IN FIFOs of
/// 128 B on EP1 and 16 B on EP2; the FIFO size is the packet limit; a stop is pending
/// until an interrupt confirms it.
inline Profile OtgFsDevCProfile()
{
  Profile profile;
  profile.sof_policy = SofPolicy::AT_STOPS;
  profile.endpoints = {{1, Endpoint::Direction::OUT, 128, 0, 256},
                       {1, Endpoint::Direction::IN, 128, 0, 128},
                       {2, Endpoint::Direction::IN, 128, 0, 16}};
  profile.ep0_buffer = 64;
  profile.stop_mode = StopMode::FRAMES;
  profile.stop_frames = 2;
  return profile;
}

/// ESP32-S3 DWC2 FS（XR-DAP-S3）：EP1 4 KiB、EP2 1 KiB 双向，EP3 只有 IN 128 B；包长只受
/// 槽内存限制；同时占用 TX FIFO 的 IN 端点最多 5 个（含 EP0），到总线复位才释放；停止
/// 待决，由中断确认。
/// ESP32-S3 DWC2 FS (XR-DAP-S3): EP1 4 KiB and EP2 1 KiB in both directions, EP3 IN
/// only with 128 B; the packet size is bounded by the slot memory only; at most 5 IN
/// endpoints hold a TX FIFO at a time (EP0 included) until a bus reset; a stop is
/// pending until an interrupt confirms it.
inline Profile Esp32S3Profile()
{
  Profile profile;
  profile.sof_policy = SofPolicy::AT_STOPS;
  profile.endpoints = {{1, Endpoint::Direction::BOTH, 4096, 0},
                       {2, Endpoint::Direction::BOTH, 1024, 0},
                       {3, Endpoint::Direction::IN, 128, 0}};
  profile.ep0_buffer = 128;
  profile.in_endpoint_limit = 5;
  profile.stop_mode = StopMode::FRAMES;
  profile.stop_frames = 2;
  return profile;
}

/// CH32 OTG FS（CH32V30x）：EP1–EP4 双向，包长上限 64；停止立即完成；帧中断在
/// OnControlReady 里打开，之后不再关闭。这个档位不在 Line A 的档位里，是按候选线的 CH32
/// 后端加的。
/// CH32 OTG FS (CH32V30x): EP1 to EP4 in both directions, packet limit 64; a stop
/// completes at once; the frame interrupt is turned on in OnControlReady and never off
/// again. This profile is not among Line A's; it follows the candidate's CH32 backend.
inline Profile Ch32OtgFsProfile()
{
  Profile profile;
  profile.endpoints = {{1, Endpoint::Direction::BOTH, 128, 0, 64},
                       {2, Endpoint::Direction::BOTH, 128, 0, 64},
                       {3, Endpoint::Direction::BOTH, 128, 0, 64},
                       {4, Endpoint::Direction::BOTH, 128, 0, 64}};
  profile.ep0_buffer = 64;
  profile.stop_mode = StopMode::IMMEDIATE;
  profile.sof_policy = SofPolicy::AT_CONTROL_READY;
  return profile;
}

}  // namespace LibXR::USB::Sim
