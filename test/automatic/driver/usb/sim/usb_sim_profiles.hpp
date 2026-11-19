/**
 * @file usb_sim_profiles.hpp
 * @brief 仿真后端的硬件档位 / Hardware profiles of the simulation backend
 *
 * 每个档位只表达后端经契约暴露的东西：端点号和方向、槽内存、FIFO 或 PMA 的包长上限、
 * 硬件是否直接读写槽、硬件上同时几个请求、类型和等时的限制，以及停止怎样停稳。取值来自
 * 本树的后端（HardwareLimits、CheckHardware 和类说明）。
 * Each profile expresses only what a backend exposes through the contract: endpoint
 * numbers and directions, slot memory, the FIFO or PMA packet limit, whether the
 * hardware accesses slots directly, how many requests the hardware holds, the type and
 * isochronous restrictions, and how a stop becomes quiescent. The values come from the
 * backends of this tree (HardwareLimits, CheckHardware and the class notes).
 *
 * 槽内存按主机的缓存行（64 B）取整；比它小的 FIFO 或 PMA 用包长上限表达。
 * Slot memory is rounded to the host cache line (64 B); a smaller FIFO or PMA is
 * expressed through the packet limit.
 */

#pragma once

#include "usb_sim.hpp"

namespace LibXR::USB::Sim
{

/**
 * @brief STM32F103 FSDEV / STM32F103 FSDEV
 * @note EP1 双向 PMA 64 B，EP2 只有 IN、PMA 16 B；CPU 在 PMA 与槽之间拷贝（不直接读写
 *       槽）；同号 IN、OUT 共用 EP_TYPE，等时端点占用整个端点号；停止在下一个 SOF 停稳。
 *       EP1 both directions with 64 B of PMA, EP2 IN only with 16 B; the CPU copies
 *       between PMA and slot (no direct access); IN and OUT of one number share EP_TYPE
 *       and an isochronous endpoint takes the whole number; a stop is quiescent at the
 *       next SOF.
 */
inline Profile FsdevF103Profile()
{
  Profile profile;
  profile.endpoints = {{1, Endpoint::Direction::BOTH, 128, 0, 64, 1},
                       {2, Endpoint::Direction::IN, 128, 0, 16, 1}};
  profile.ep0_buffer = 64;
  profile.max_packet_limit = 1023;
  profile.direct_memory = false;
  profile.shared_type_per_number = true;
  profile.iso_takes_number = true;
  profile.stop_mode = StopMode::FRAMES;
  profile.stop_frames = 1;
  return profile;
}

/**
 * @brief STM32F407 OTG FS，无 DMA / STM32F407 OTG FS without DMA
 * @note 所有 OUT 共用 256 B 的 RX FIFO，IN FIFO：EP1 128 B、EP2 16 B；FIFO 大小即包长
 *       上限；CPU 拷贝；停止等 NAK 生效，按两帧计。
 *       Every OUT shares a 256 B RX FIFO, IN FIFOs of 128 B on EP1 and 16 B on EP2; the
 *       FIFO size is the packet limit; the CPU copies; a stop waits for the NAK to take
 *       effect, counted as two frames.
 */
inline Profile OtgFsDevCProfile()
{
  Profile profile;
  profile.endpoints = {{1, Endpoint::Direction::OUT, 128, 0, 256, 1},
                       {1, Endpoint::Direction::IN, 128, 0, 128, 1},
                       {2, Endpoint::Direction::IN, 128, 0, 16, 1}};
  profile.ep0_buffer = 64;
  profile.direct_memory = false;
  profile.stop_mode = StopMode::FRAMES;
  profile.stop_frames = 2;
  return profile;
}

/**
 * @brief ESP32-S3 DWC2 全速，DMA / ESP32-S3 DWC2 full speed with DMA
 * @note XR-DAP 的布局：EP1 4 KiB、EP2 1 KiB 双向，EP3 只有 IN、128 B；DMA 直接读写槽；
 *       停止等硬件确认，按两帧计。
 *       The XR-DAP layout: EP1 4 KiB and EP2 1 KiB in both directions, EP3 IN only with
 *       128 B; the DMA accesses the slot directly; a stop waits for the hardware's
 *       confirmation, counted as two frames.
 */
inline Profile Esp32S3Profile()
{
  Profile profile;
  profile.endpoints = {{1, Endpoint::Direction::BOTH, 4096, 0, 0, 1},
                       {2, Endpoint::Direction::BOTH, 1024, 0, 0, 1},
                       {3, Endpoint::Direction::IN, 128, 0, 0, 1}};
  profile.ep0_buffer = 128;
  profile.direct_memory = true;
  profile.stop_mode = StopMode::FRAMES;
  profile.stop_frames = 2;
  return profile;
}

/**
 * @brief HPM 高速，两个传输描述符 / HPM high speed with two transfer descriptors
 * @note 每个方向两个请求挂在硬件上；包长 1024，一次请求最多 16 KiB；高速等时和中断每个
 *       微帧最多三个包；DMA 直接读写槽；停止等硬件确认，按两个微帧计。
 *       Two requests on the hardware per direction; packets up to 1024 and 16 KiB per
 *       request; up to three packets per microframe for high-speed isochronous and
 *       interrupt; the DMA accesses the slot directly; a stop waits for the hardware's
 *       confirmation, counted as two microframes.
 */
inline Profile HpmHighSpeedProfile()
{
  Profile profile;
  profile.endpoints = {{1, Endpoint::Direction::BOTH, 8192, 4, 0, 2},
                       {2, Endpoint::Direction::BOTH, 8192, 4, 0, 2},
                       {3, Endpoint::Direction::BOTH, 1024, 0, 0, 2}};
  profile.ep0_buffer = 64;
  profile.max_packet_limit = 1024;
  profile.max_bytes = 16384;
  profile.max_packets_per_interval = 3;
  profile.direct_memory = true;
  profile.stop_mode = StopMode::FRAMES;
  profile.stop_frames = 2;
  return profile;
}

}  // namespace LibXR::USB::Sim
