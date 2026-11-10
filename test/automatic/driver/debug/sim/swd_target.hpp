/**
 * @file swd_target.hpp
 * @brief SWD 目标仿真：ADIv5 SW-DP + AHB-AP + 最小 Cortex-M 调试寄存器 / SWD target
 *        simulation: ADIv5 SW-DP, AHB-AP and minimal Cortex-M debug registers.
 *
 * 目标在引脚级工作：每个 SWCLK 上升沿采样 SWDIO，并在需要时驱动下一位；主机在时钟低
 * 电平阶段读。请求、转向、ACK、数据、校验、线复位、JTAG/SWD 切换序列都按 ADIv5 的
 * 位时序推进，所以主机侧的时序错误（例如空闲周期把 SWDIO 驱动成高电平、转向周期少一个）
 * 会直接表现为错位或总线冲突。
 * The target works at the pin level: on every SWCLK rising edge it samples SWDIO and,
 * where its turn comes, drives the next bit; the host reads in the clock-low phase.
 * Request, turnaround, ACK, data, parity, line reset and the JTAG/SWD switch sequences
 * follow the ADIv5 bit timing, so a timing mistake on the host side (an idle cycle
 * driving SWDIO high, a missing turnaround cycle) shows up as misalignment or bus
 * contention.
 *
 * 三种接法 / Three ways to connect:
 * - `SimSwd`：实现 `Debug::Swd` 接口，按位生成波形，给 DAP 命令层用。
 *   `SimSwd` implements `Debug::Swd` and generates the waveform bit by bit, for the DAP
 *   command layer.
 * - `SimSwclkPin` / `SimSwdioPin`：两个 GPIO，给 `SwdGeneralGPIO` 这类位操作引擎用。
 *   Two GPIOs for bit-banging engines such as `SwdGeneralGPIO`.
 * - remote_bitbang 服务端（阶段 5）：给真实 OpenOCD、pyOCD 校准这个模型。
 *   A remote_bitbang server (stage 5) for real OpenOCD and pyOCD to calibrate the model.
 *
 * 故障注入：WAIT、总线错误（置 STICKYERR，之后 FAULT）、读数据校验错、不应答（掉线）、
 * 目标复位。
 * Fault injection: WAIT, bus error (sets STICKYERR, FAULT afterwards), bad read
 * parity, no response (target gone), target reset.
 */

#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <functional>
#include <map>
#include <vector>

#include "debug/swd.hpp"
#include "gpio.hpp"

namespace LibXR::Debug::Sim
{

// ---------------------------------------------------------------------------
// Address space: RAM, Cortex-M system control space, ROM table
// ---------------------------------------------------------------------------

/// 目标的地址空间 / The target address space
class TargetMemory
{
 public:
  struct Region
  {
    uint32_t base;
    std::vector<uint8_t> bytes;
    bool writable;
  };

  static constexpr uint32_t CPUID = 0xE000ED00u;
  static constexpr uint32_t AIRCR = 0xE000ED0Cu;
  static constexpr uint32_t DHCSR = 0xE000EDF0u;
  static constexpr uint32_t DCRSR = 0xE000EDF4u;
  static constexpr uint32_t DCRDR = 0xE000EDF8u;
  static constexpr uint32_t DEMCR = 0xE000EDFCu;
  static constexpr uint32_t DWT_CTRL = 0xE0001000u;
  static constexpr uint32_t DBGMCU_IDCODE = 0xE0042000u;
  static constexpr uint32_t FP_CTRL = 0xE0002000u;
  static constexpr uint32_t FP_COMP0 = 0xE0002008u;
  static constexpr uint32_t ROM_TABLE = 0xE00FF000u;
  static constexpr uint32_t SCS_BASE = 0xE000E000u;

  TargetMemory()
  {
    AddRegion(0x20000000u, 64u * 1024u, true);    // SRAM
    AddRegion(0x08000000u, 128u * 1024u, false);  // flash: readable, writes fault
    // 启动别名：STM32 把闪存映射到 0；调试器复位后会从这里读向量表和指令
    // Boot alias: STM32 maps flash at 0; debuggers read the vector table and instructions
    // there after a reset.
    AddRegion(0x00000000u, 128u * 1024u, false);
    Reset();
  }

  void AddRegion(uint32_t base, uint32_t size, bool writable)
  {
    regions_.push_back({base, std::vector<uint8_t>(size, 0u), writable});
  }

  /// 系统复位：内核寄存器清零；DEMCR.VC_CORERESET 置位时复位后停核 / System reset:
  /// core registers cleared; halted after reset when DEMCR.VC_CORERESET is set
  void Reset()
  {
    regs_.fill(0u);
    regs_[16] = 0x01000000u;  // xPSR: Thumb
    reset_seen_ = true;
    halted_ = debug_enabled_ && (demcr_ & 1u) != 0u;
    ++resets;
  }

  unsigned resets = 0;
  bool Halted() const { return halted_; }
  uint32_t Reg(unsigned n) const { return regs_.at(n); }

  /// size：0 字节、1 半字、2 字；值在对应的字节通道上 / size: 0 byte, 1 half, 2 word;
  /// the value sits on its byte lanes
  bool Read(uint32_t addr, uint8_t size, uint32_t& lanes)
  {
    if (!Aligned(addr, size)) return false;
    if (Region* r = Find(addr, size))
    {
      uint32_t v = 0;
      std::memcpy(&v, &r->bytes[(addr & ~3u) - r->base], 4);  // word containing addr
      lanes = Mask(v, addr, size);
      return true;
    }
    uint32_t word = 0;
    if (!ReadSystem(addr & ~3u, word)) return false;
    lanes = Mask(word, addr, size);
    return true;
  }

  bool Write(uint32_t addr, uint8_t size, uint32_t lanes)
  {
    if (!Aligned(addr, size)) return false;
    if (Region* r = Find(addr, size))
    {
      if (!r->writable) return false;
      const uint32_t n = 1u << size;
      for (uint32_t i = 0; i < n; ++i)
        r->bytes[addr - r->base + i] =
            static_cast<uint8_t>(lanes >> (((addr & 3u) + i) * 8u));
      return true;
    }
    if (size != 2) return false;  // 系统寄存器只接受字访问 / system registers: words
    return WriteSystem(addr, lanes);
  }

 private:
  static bool Aligned(uint32_t addr, uint8_t size)
  {
    return size <= 2 && (addr & ((1u << size) - 1u)) == 0u;
  }
  static uint32_t Mask(uint32_t word, uint32_t addr, uint8_t size)
  {
    if (size == 2) return word;
    const uint32_t mask = size == 0 ? 0xFFu : 0xFFFFu;
    return word & (mask << ((addr & 3u) * 8u));
  }

  Region* Find(uint32_t addr, uint8_t size)
  {
    for (auto& r : regions_)
      if (addr >= r.base && addr + (1u << size) <= r.base + r.bytes.size()) return &r;
    return nullptr;
  }

  /// CoreSight 组件 ID：PIDR4..7、PIDR0..3、CIDR0..3 在 0xFD0..0xFFC / CoreSight
  /// component ID registers at 0xFD0..0xFFC
  static bool ComponentId(uint32_t offset, const std::array<uint8_t, 12>& id,
                          uint32_t& word)
  {
    if (offset < 0xFD0u || offset > 0xFFCu) return false;
    word = id[(offset - 0xFD0u) / 4u];
    return true;
  }

  bool ReadSystem(uint32_t addr, uint32_t& word)
  {
    switch (addr)
    {
      case CPUID:
        word = 0x410FC241u;  // Cortex-M4 r0p1
        return true;
      case AIRCR:
        word = 0xFA050000u;
        return true;
      case DHCSR:
        word = (debug_enabled_ ? 1u : 0u) | (halted_ ? (1u << 1) : 0u) |
               (1u << 16) |                      // S_REGRDY
               (halted_ ? (1u << 17) : 0u) |     // S_HALT
               (!halted_ ? (1u << 24) : 0u) |    // S_RETIRE_ST
               (reset_seen_ ? (1u << 25) : 0u);  // S_RESET_ST
        reset_seen_ = false;                     // 读清 / cleared by the read
        return true;
      case DCRSR:
        word = 0;
        return true;
      case DCRDR:
        word = dcrdr_;
        return true;
      case DEMCR:
        word = demcr_;
        return true;
      case DBGMCU_IDCODE:
        // STM32G431：DEV_ID 0x468，REV_ID 0x2001；调试器据此识别芯片 / STM32G431: DEV_ID
        // 0x468, REV_ID 0x2001; debuggers identify the chip from it
        word = 0x20016468u;
        return true;
      case DWT_CTRL:
        word = 0;  // 没有比较器 / no comparators
        return true;
      case FP_CTRL:
        // REV=1，6 个代码比较器、2 个文字比较器，ENABLE 可写（同 Cortex-M4）/ REV 1,
        // six code and two literal comparators, ENABLE writable (as on a Cortex-M4)
        word = 0x10000000u | (2u << 8) | (6u << 4) | (fp_enable_ ? 1u : 0u);
        return true;
      default:
        if (addr >= FP_COMP0 && addr < FP_COMP0 + 4u * fp_comp_.size())
        {
          word = fp_comp_[(addr - FP_COMP0) / 4u];
          return true;
        }
        break;
    }
    // ROM 表：SCS、DWT、FPB 三项后结束 / ROM table: SCS, DWT, FPB, then the end marker
    if (addr >= ROM_TABLE && addr < ROM_TABLE + 0x1000u)
    {
      const uint32_t off = addr - ROM_TABLE;
      if (off == 0x000u)
        word = 0xFFF0F003u;  // SCS 0xE000E000
      else if (off == 0x004u)
        word = 0xFFF02003u;  // DWT 0xE0001000
      else if (off == 0x008u)
        word = 0xFFF03003u;  // FPB 0xE0002000
      else if (off == 0xFCCu)
        word = 0x00000001u;  // MEMTYPE: system memory present
      // 顶层 ROM 表的 ID 是芯片厂商的：ST（JEP106 0x020），零件号 = 器件 ID 0x468，
      // 调试器据此识别芯片 / The top ROM table carries the chip vendor's ID: ST (JEP106
      // 0x020), part number = device ID 0x468; debuggers identify the chip from it
      else if (!ComponentId(
                   off, {0x00, 0, 0, 0, 0x68, 0x04, 0x0A, 0x00, 0x0D, 0x10, 0x05, 0xB1},
                   word))
        word = 0;
      return true;
    }
    // SCS、DWT、FPB 的组件 ID / component IDs of the SCS, DWT and FPB
    if ((addr & 0xFFFFF000u) == SCS_BASE)
    {
      if (ComponentId(addr & 0xFFFu,
                      {0x04, 0, 0, 0, 0x0C, 0xB0, 0x0B, 0x00, 0x0D, 0xE0, 0x05, 0xB1},
                      word))
        return true;
      word = 0;  // 其余 SCS 寄存器读 0 / other SCS registers read 0
      return true;
    }
    if ((addr & 0xFFFFF000u) == 0xE0001000u || (addr & 0xFFFFF000u) == 0xE0002000u)
    {
      const bool dwt = (addr & 0xFFFFF000u) == 0xE0001000u;
      if (ComponentId(addr & 0xFFFu,
                      {0x04, 0, 0, 0, static_cast<uint8_t>(dwt ? 0x02 : 0x03), 0xB0, 0x3B,
                       0x00, 0x0D, 0xE0, 0x05, 0xB1},
                      word))
        return true;
      word = 0;
      return true;
    }
    // 私有外设区的其余部分（ITM、TPIU 等）：读 0 / The rest of the private peripheral
    // bus (ITM, TPIU, ...): reads 0.
    if (InPpb(addr))
    {
      word = 0;
      return true;
    }
    return false;
  }

  static bool InPpb(uint32_t addr) { return addr >= 0xE0000000u && addr < 0xE0100000u; }

  bool WriteSystem(uint32_t addr, uint32_t value)
  {
    switch (addr)
    {
      case DHCSR:
        if ((value >> 16) != 0xA05Fu) return true;  // 没有 DBGKEY 的写被忽略
        debug_enabled_ = (value & 1u) != 0u;
        if (!debug_enabled_)
          halted_ = false;
        else
          // C_HALT 停核；停着时 C_STEP 执行一条指令后重新停住（模型不执行指令，保持停核）
          // C_HALT halts; C_STEP while halted executes one instruction and halts again
          // (the model executes nothing and stays halted).
          halted_ = (value & (1u << 1)) != 0u || (halted_ && (value & (1u << 2)) != 0u);
        return true;
      case DCRSR:
      {
        const uint32_t sel = value & 0x7Fu;
        if ((value & (1u << 16)) != 0u)
          regs_[sel] = dcrdr_;
        else
          dcrdr_ = regs_[sel];
        return true;
      }
      case DCRDR:
        dcrdr_ = value;
        return true;
      case FP_CTRL:
        if ((value & 2u) != 0u) fp_enable_ = (value & 1u) != 0u;  // KEY 位 / KEY bit
        return true;
      case FP_COMP0 + 0x00:
      case FP_COMP0 + 0x04:
      case FP_COMP0 + 0x08:
      case FP_COMP0 + 0x0C:
      case FP_COMP0 + 0x10:
      case FP_COMP0 + 0x14:
      case FP_COMP0 + 0x18:
      case FP_COMP0 + 0x1C:
        fp_comp_[(addr - FP_COMP0) / 4u] = value;
        return true;
      case DEMCR:
        demcr_ = value;
        return true;
      case AIRCR:
        if ((value >> 16) == 0x05FAu && (value & 0x5u) != 0u) Reset();
        return true;
      default:
        // 私有外设区的其余写被吸收（例如 ITM 解锁）/ Other writes to the private
        // peripheral bus are absorbed (for example the ITM unlock).
        return InPpb(addr);
    }
  }

  std::vector<Region> regions_;
  std::array<uint32_t, 128> regs_{};
  uint32_t dcrdr_ = 0, demcr_ = 0;
  bool debug_enabled_ = false, halted_ = false, reset_seen_ = false, fp_enable_ = false;
  std::array<uint32_t, 8> fp_comp_{};
};

// ---------------------------------------------------------------------------
// Debug port and access port
// ---------------------------------------------------------------------------

/// ADIv5 SW-DP（DPv1）与一个 AHB-AP / ADIv5 SW-DP (DPv1) with one AHB-AP
class DebugPort
{
 public:
  static constexpr uint32_t DPIDR = 0x2BA01477u;
  static constexpr uint32_t AP_IDR = 0x24770011u;
  static constexpr uint32_t STICKYORUN = 1u << 1, STICKYERR = 1u << 5, READOK = 1u << 6,
                            WDATAERR = 1u << 7;
  using Ack = SwdProtocol::Ack;

  explicit DebugPort(TargetMemory& memory) : memory_(memory) {}

  // ---- 故障注入 / Fault injection ----
  unsigned inject_wait = 0;        ///< 之后 N 次 AP/RDBUFF 访问回 WAIT / N WAITs
  unsigned inject_bus_error = 0;   ///< 之后 N 次 AP 传输出总线错误 / N bus errors
  unsigned inject_bad_parity = 0;  ///< 之后 N 次读数据校验位取反 / N bad read parities
  /// 每 wait_every 次 AP/RDBUFF 访问注入 wait_burst 个 WAIT（0 关闭）/ Every wait_every
  /// AP/RDBUFF accesses inject wait_burst WAITs (0 turns it off)
  unsigned wait_every = 0, wait_burst = 1;

  uint32_t CtrlStat() const
  {
    uint32_t v = ctrl_ & (SwdProtocol::DP_CTRLSTAT_CDBGPWRUPREQ |
                          SwdProtocol::DP_CTRLSTAT_CSYSPWRUPREQ | 1u);
    if (v & SwdProtocol::DP_CTRLSTAT_CDBGPWRUPREQ)
      v |= SwdProtocol::DP_CTRLSTAT_CDBGPWRUPACK;
    if (v & SwdProtocol::DP_CTRLSTAT_CSYSPWRUPREQ)
      v |= SwdProtocol::DP_CTRLSTAT_CSYSPWRUPACK;
    return v | sticky_ | (read_ok_ ? READOK : 0u);
  }

  /// 请求头收齐时决定 ACK；读请求同时给出数据 / Decide the ACK once the header is in;
  /// a read also produces its data
  Ack Header(bool ap, bool rnw, uint8_t a32, uint32_t& rdata)
  {
    const bool rdbuff = !ap && rnw && a32 == 3;
    if ((ap || rdbuff) && wait_every != 0 && ++ap_accesses_ % wait_every == 0)
    {
      inject_wait += wait_burst;
    }
    if ((ap || rdbuff) && inject_wait != 0)
    {
      --inject_wait;
      // 溢出检测打开时 WAIT 置 STICKYORUN / With overrun detection a WAIT sets STICKYORUN
      if (OverrunDetect()) sticky_ |= STICKYORUN;
      return Ack::WAIT;
    }
    if (sticky_ & (STICKYERR | WDATAERR | STICKYORUN))
    {
      // 粘性错误期间只放行 DPIDR、CTRL/STAT、RESEND 读和 ABORT 写 / With a sticky
      // error only DPIDR, CTRL/STAT and RESEND reads and ABORT writes pass.
      const bool allowed = !ap && ((rnw && a32 <= 2) || (!rnw && a32 == 0));
      if (!allowed) return Ack::FAULT;
    }
    pending_ap_ = ap;
    pending_a32_ = a32;
    if (!rnw) return Ack::OK;
    if (!ap)
    {
      switch (a32)
      {
        case 0:
          rdata = DPIDR;
          break;
        case 1:
          rdata = (select_ & 0xFu) == 0u ? CtrlStat() : wcr_;
          break;
        case 2:
          rdata = last_read_;
          break;  // RESEND
        default:
          rdata = ap_buffer_;
          break;  // RDBUFF
      }
    }
    else
    {
      // AP 读是延迟的：这次返回上一次的结果，结果进 RDBUFF / AP reads are posted: this
      // one returns the previous result, the new one goes to RDBUFF.
      rdata = ap_buffer_;
      read_ok_ = ApRead(a32, ap_buffer_);
    }
    last_read_ = rdata;
    return Ack::OK;
  }

  /// 写请求的数据阶段 / The data phase of a write
  void WriteData(uint32_t wdata, bool parity_ok)
  {
    if (!parity_ok)
    {
      sticky_ |= WDATAERR;
      return;
    }
    if (pending_ap_)
    {
      ApWrite(pending_a32_, wdata);
      return;
    }
    switch (pending_a32_)
    {
      case 0:  // ABORT
        if (wdata & SwdProtocol::DP_ABORT_STKERRCLR) sticky_ &= ~STICKYERR;
        if (wdata & SwdProtocol::DP_ABORT_WDERRCLR) sticky_ &= ~WDATAERR;
        if (wdata & SwdProtocol::DP_ABORT_ORUNERRCLR) sticky_ &= ~STICKYORUN;
        break;
      case 1:
        if ((select_ & 0xFu) == 0u)
          ctrl_ = wdata;
        else if ((select_ & 0xFu) == 1u)
          wcr_ = (wdata & 0x300u) | 0x40u;  // WCR：只有 TURNROUND 可写 / only TURNROUND
        break;
      case 2:
        select_ = wdata;
        break;
      default:
        break;  // TARGETSEL 只在多点时有意义 / TARGETSEL: multi-drop only
    }
  }

  /// CTRL/STAT.ORUNDETECT：WAIT/FAULT 之后数据阶段照常存在 / With ORUNDETECT the data
  /// phase is present after WAIT/FAULT as well
  bool OverrunDetect() const { return (ctrl_ & 1u) != 0u; }

  /// WCR.TURNROUND：转向周期数 1..4 / WCR.TURNROUND: 1..4 turnaround cycles
  unsigned Turnaround() const { return ((wcr_ >> 8) & 3u) + 1u; }

  bool BadParity()
  {
    if (inject_bad_parity == 0) return false;
    --inject_bad_parity;
    return true;
  }

  /// 线复位：DP 进入复位状态 / Line reset: the DP enters its reset state
  void LineReset() { read_ok_ = false; }

  uint32_t Select() const { return select_; }
  uint32_t Csw() const { return csw_; }
  uint32_t Tar() const { return tar_; }

 private:
  bool ApRead(uint8_t a32, uint32_t& value)
  {
    if ((select_ >> 24) != 0u)
    {
      value = 0;  // 不存在的 AP 读 0 / a missing AP reads 0
      return true;
    }
    const uint32_t reg = ((select_ & 0xF0u)) | (uint32_t(a32) << 2);
    switch (reg)
    {
      case 0x00:
        value = (csw_ & ~0x30u) | ((csw_ & 0x30u) == 0x10u ? 0x10u : 0u) | 0x40u;
        return true;
      case 0x04:
        value = tar_;
        return true;
      case 0x0C:
        return Transfer(true, tar_, value);
      case 0x10:
      case 0x14:
      case 0x18:
      case 0x1C:
      {
        uint32_t addr = (tar_ & ~0xFu) | (reg & 0xCu);
        return Transfer(true, addr, value, false);
      }
      case 0xF4:
        value = 0;
        return true;
      case 0xF8:
        value = TargetMemory::ROM_TABLE | 3u;
        return true;
      case 0xFC:
        value = AP_IDR;
        return true;
      default:
        value = 0;
        return true;
    }
  }

  void ApWrite(uint8_t a32, uint32_t value)
  {
    if ((select_ >> 24) != 0u) return;
    const uint32_t reg = ((select_ & 0xF0u)) | (uint32_t(a32) << 2);
    uint32_t dummy = value;
    switch (reg)
    {
      case 0x00:
        csw_ = value;
        break;
      case 0x04:
        tar_ = value;
        break;
      case 0x0C:
        (void)Transfer(false, tar_, dummy);
        break;
      case 0x10:
      case 0x14:
      case 0x18:
      case 0x1C:
        (void)Transfer(false, (tar_ & ~0xFu) | (reg & 0xCu), dummy, false);
        break;
      default:
        break;
    }
  }

  /// 一次 MEM-AP 传输，失败置 STICKYERR / One MEM-AP transfer; a failure sets STICKYERR
  bool Transfer(bool read, uint32_t addr, uint32_t& value, bool use_tar = true)
  {
    const uint8_t size = use_tar ? static_cast<uint8_t>(csw_ & 7u) : 2u;
    bool ok;
    if (inject_bus_error != 0)
    {
      --inject_bus_error;
      ok = false;
    }
    else
    {
      ok = read ? memory_.Read(addr, size, value) : memory_.Write(addr, size, value);
    }
    if (!ok)
    {
      sticky_ |= STICKYERR;
      return false;
    }
    // 单次自增，在 4 KiB 边界内回绕：Cortex-M3/M4 的 AHB-AP 是 4 KiB（ADIv5 只保证
    // 1 KiB），OpenOCD 对 Cortex-M 也按 4 KiB 分块。
    // Single increment, wrapping inside 4 KiB: the Cortex-M3/M4 AHB-AP has 4 KiB (ADIv5
    // only guarantees 1 KiB), and OpenOCD splits Cortex-M blocks at 4 KiB too.
    if (use_tar && (csw_ & 0x30u) == 0x10u)
      tar_ = (tar_ & ~0xFFFu) | ((tar_ + (1u << size)) & 0xFFFu);
    return true;
  }

  TargetMemory& memory_;
  uint32_t ctrl_ = 0, select_ = 0, sticky_ = 0, wcr_ = 0x40u;
  uint32_t csw_ = 0x00000002u, tar_ = 0;
  uint32_t ap_buffer_ = 0, last_read_ = 0;
  unsigned ap_accesses_ = 0;
  bool read_ok_ = false;
  bool pending_ap_ = false;
  uint8_t pending_a32_ = 0;
};

// ---------------------------------------------------------------------------
// Wire protocol
// ---------------------------------------------------------------------------

/// SWD 线协议的目标端：在每个 SWCLK 上升沿推进 / Target side of the SWD wire protocol,
/// advanced on every SWCLK rising edge
class WireTarget
{
 public:
  using Ack = SwdProtocol::Ack;

  explicit WireTarget(DebugPort& dp, bool swd_selected = false)
      : dp_(dp), swd_(swd_selected)
  {
  }

  bool disconnected = false;  ///< 目标掉线：不驱动任何位 / Target gone: drives nothing
  unsigned contention = 0;    ///< 双方同时驱动的上升沿数 / Edges with both sides driving
  unsigned protocol_errors = 0;
  unsigned line_resets = 0;
  /// 跟踪：协议错误、线复位、模式切换 / Trace: protocol errors, line resets, mode
  /// switches
  std::function<void(const char* what, uint32_t edge, uint32_t value)> trace;
  bool SwdSelected() const { return swd_; }

  // ---- 主机侧的线状态 / Host side of the line ----
  void SetHostDriving(bool driving) { host_driving_ = driving; }
  void SetHostLevel(bool level) { host_level_ = level; }

  /// 线上电平：目标驱动时是目标的位，否则主机的位，都不驱动时上拉为 1 / The line level:
  /// the target's bit while it drives, otherwise the host's, pulled up to 1 when nobody
  /// drives
  bool Line() const
  {
    if (target_driving_) return target_bit_;
    return host_driving_ ? host_level_ : true;
  }

  /// SWCLK 上升沿 / SWCLK rising edge
  void Rising()
  {
    if (target_driving_ && host_driving_) ++contention;
    ++edges_;
    const bool bit = Line();
    const bool host_bit = !target_driving_;
    target_driving_ = false;

    // 线复位：连续 50 个以上主机驱动的 1 / Line reset: 50 or more host-driven ones.
    if (host_bit && bit)
    {
      if (++ones_ == 50) EnterReset();
    }
    else
    {
      ones_ = 0;
    }

    if (state_ == State::RESET)
    {
      if (bit) return;  // 仍在复位序列里 / still inside the reset sequence
      window_ = true;
      window_bits_ = 0;
      window_value_ = 0;
      state_ = swd_ ? State::IDLE : State::LOCKOUT;
    }
    if (window_)
    {
      // 复位后的 16 位同时检查 JTAG/SWD 切换序列，协议解析照常进行 / The 16 bits after
      // a reset are also checked for a JTAG/SWD switch sequence while parsing goes on.
      window_value_ |= static_cast<uint16_t>(static_cast<uint16_t>(bit) << window_bits_);
      if (++window_bits_ == 16)
      {
        window_ = false;
        if (trace) trace("window", edges_, window_value_);
        if (!swd_ && window_value_ == 0xE79Eu)
        {
          swd_ = true;
          state_ = State::LOCKOUT;  // 切换后要再来一次线复位 / needs another line reset
          return;
        }
        if (swd_ && window_value_ == 0xE73Cu)
        {
          swd_ = false;
          state_ = State::LOCKOUT;
          return;
        }
      }
    }
    if (!swd_) return;

    switch (state_)
    {
      case State::LOCKOUT:
        return;
      case State::IDLE:
        if (bit)
        {
          state_ = State::HEADER;
          header_ = 1;
          header_bits_ = 1;
        }
        return;
      case State::HEADER:
        header_ |= static_cast<uint8_t>(bit) << header_bits_;
        if (++header_bits_ == 8) DecodeHeader();
        return;
      case State::TURN_TO_TARGET:
        if (--turn_left_ != 0) return;
        // 转向周期结束：开始送 ACK / end of the turnaround: start sending the ACK
        if (respond_)
        {
          Drive((static_cast<uint8_t>(ack_) & 1u) != 0u);
          state_ = State::ACK;
          count_ = 1;
        }
        else
        {
          state_ = State::SKIP;
          count_ = 3 + dp_.Turnaround();  // 主机仍会读 3 位 ACK 加转向 / ACK + turn
        }
        return;
      case State::ACK:
        if (count_ < 3)
        {
          Drive(((static_cast<uint8_t>(ack_) >> count_) & 1u) != 0u);
          ++count_;
          return;
        }
        if (ack_ != Ack::OK)
        {
          if (!dp_.OverrunDetect())
          {
            state_ = State::TURN_TO_HOST;
            turn_left_ = dp_.Turnaround();
            return;
          }
          // 溢出检测：数据阶段照常走完，目标不驱动读数据、不采用写数据 / Overrun
          // detection: the data phase still runs; the target neither drives read data
          // nor takes write data.
          dummy_ = true;
        }
        if (rnw_)
        {
          state_ = State::RDATA;
          count_ = 0;
          SendData();
          return;
        }
        state_ = State::TURN_WRITE;
        turn_left_ = dp_.Turnaround();
        return;
      case State::RDATA:
        SendData();
        return;
      case State::TURN_TO_HOST:
      case State::TURN_WRITE:
        if (--turn_left_ != 0) return;
        if (state_ == State::TURN_WRITE)
        {
          state_ = State::WDATA;
          count_ = 0;
          wdata_ = 0;
        }
        else
        {
          state_ = State::IDLE;
        }
        return;
      case State::WDATA:
        if (count_ < 32)
        {
          wdata_ |= static_cast<uint32_t>(bit) << count_;
          ++count_;
          return;
        }
        if (!dummy_)
        {
          if (trace)
            trace(bit == Parity(wdata_) ? "  wdata" : "  wdata BAD PARITY", edges_,
                  wdata_);
          dp_.WriteData(wdata_, bit == Parity(wdata_));
        }
        state_ = State::IDLE;
        return;
      case State::SKIP:
        if (--count_ == 0) state_ = State::IDLE;
        return;
      default:
        return;
    }
  }

 private:
  enum class State : uint8_t
  {
    LOCKOUT,  ///< 等线复位 / waiting for a line reset
    RESET,
    IDLE,
    HEADER,
    TURN_TO_TARGET,
    ACK,
    RDATA,
    TURN_TO_HOST,
    TURN_WRITE,
    WDATA,
    SKIP
  };

  static bool Parity(uint32_t v)
  {
    v ^= v >> 16;
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return (v & 1u) != 0u;
  }

  void Drive(bool bit)
  {
    if (disconnected) return;
    target_driving_ = true;
    target_bit_ = bit;
  }

  void EnterReset()
  {
    if (error_pending_ && edges_ - error_edge_ <= 50u)
    {
      --protocol_errors;
      if (trace) trace("protocol error withdrawn by line reset", edges_, 0);
    }
    if (trace) trace("line reset", edges_, 0);
    error_pending_ = false;
    state_ = State::RESET;
    need_dpidr_ = true;
    ++line_resets;
    dp_.LineReset();
  }

  void DecodeHeader()
  {
    const bool start = header_ & 1u, ap = (header_ >> 1) & 1u, rnw = (header_ >> 2) & 1u;
    const uint8_t a32 = (header_ >> 3) & 3u;
    const bool parity = (header_ >> 5) & 1u, stop = (header_ >> 6) & 1u,
               park = (header_ >> 7) & 1u;
    state_ = State::TURN_TO_TARGET;
    turn_left_ = dp_.Turnaround();
    rnw_ = rnw;
    respond_ = false;
    dummy_ = false;
    if (!start || stop || !park || parity != (ap ^ rnw ^ (a32 & 1u) ^ (a32 >> 1)))
    {
      // 协议错误：不应答，等线复位。复位后 16 位内的错误来自切换序列，不计数；紧跟着线
      // 复位的错误是复位开头的 1 被当成了请求头，复位时撤销。
      // Protocol error: no response until a line reset. Errors within the 16 bits after
      // a reset come from a switch sequence and are not counted; an error right before a
      // line reset is the start of the reset taken as a header and is withdrawn by it.
      if (!window_)
      {
        ++protocol_errors;
        error_edge_ = edges_;
        error_pending_ = true;
      }
      if (trace)
        trace(window_ ? "protocol error (switch window)" : "protocol error", edges_,
              header_);
      state_ = State::LOCKOUT;
      return;
    }
    if (disconnected) return;
    // 线复位后第一条必须是读 DPIDR / The first access after a line reset must read DPIDR.
    if (need_dpidr_ && !(!ap && rnw && a32 == 0)) return;
    rdata_ = 0;
    ack_ = dp_.Header(ap, rnw, a32, rdata_);
    if (trace)
      trace(ap ? (rnw ? "AP read" : "AP write") : (rnw ? "DP read" : "DP write"), edges_,
            (static_cast<uint32_t>(a32) << 2) | (static_cast<uint32_t>(ack_) << 8) |
                (rnw ? 0u : 0u));
    if (trace && rnw && ack_ == Ack::OK) trace("  rdata", edges_, rdata_);
    if (need_dpidr_ && ack_ == Ack::OK) need_dpidr_ = false;
    respond_ = true;
  }

  void SendData()
  {
    if (dummy_ && count_ <= 32)
    {
      ++count_;  // 数据阶段不驱动，线上为上拉 / not driven: the line is pulled up
      return;
    }
    if (count_ < 32)
    {
      Drive(((rdata_ >> count_) & 1u) != 0u);
      ++count_;
      return;
    }
    if (count_ == 32)
    {
      Drive(Parity(rdata_) != dp_.BadParity());
      ++count_;
      return;
    }
    state_ = State::TURN_TO_HOST;  // 释放总线，之后是转向 / release; turnaround follows
    turn_left_ = dp_.Turnaround();
  }

  DebugPort& dp_;
  bool swd_;
  State state_ = State::LOCKOUT;
  bool host_driving_ = true, host_level_ = true;
  bool target_driving_ = false, target_bit_ = false;
  unsigned ones_ = 0;
  uint32_t edges_ = 0, error_edge_ = 0;
  bool error_pending_ = false;
  bool window_ = false;
  uint16_t window_value_ = 0;
  unsigned window_bits_ = 0;
  uint8_t header_ = 0;
  unsigned header_bits_ = 0;
  bool rnw_ = false, respond_ = false, need_dpidr_ = true, dummy_ = false;
  Ack ack_ = Ack::OK;
  uint32_t rdata_ = 0, wdata_ = 0;
  unsigned count_ = 0;
  unsigned turn_left_ = 1;
};

/// 一个完整的仿真目标 / One complete simulated target
struct Target
{
  TargetMemory memory;
  DebugPort dp{memory};
  WireTarget wire{dp};
};

// ---------------------------------------------------------------------------
// Connections
// ---------------------------------------------------------------------------

/// `Debug::Swd` 接口：按 SwdGeneralGPIO 的位时序生成波形 / `Debug::Swd` interface that
/// generates the waveform with the bit timing of SwdGeneralGPIO
class SimSwd : public Swd
{
 public:
  explicit SimSwd(WireTarget& wire) : wire_(wire) {}

  ErrorCode SetClockHz(uint32_t hz) override
  {
    clock_hz = hz;
    return ErrorCode::OK;
  }
  void Close() override {}

  ErrorCode LineReset() override
  {
    for (int i = 0; i < 64; ++i) WriteBit(true);
    return ErrorCode::OK;
  }

  ErrorCode EnterSwd() override
  {
    LineReset();
    WriteByte(0x9E);
    WriteByte(0xE7);
    LineReset();
    WriteByte(0x00);
    return ErrorCode::OK;
  }

  ErrorCode Transfer(const SwdProtocol::Request& req,
                     SwdProtocol::Response& resp) override
  {
    const bool ap = req.port == SwdProtocol::Port::AP;
    const uint8_t a2 = req.addr2b & 1u, a3 = (req.addr2b >> 1) & 1u;
    const uint8_t par = (ap ^ req.rnw ^ a2 ^ a3) & 1u;
    const uint8_t header = static_cast<uint8_t>(
        1u | (ap << 1) | (req.rnw << 2) | (a2 << 3) | (a3 << 4) | (par << 5) | (1u << 7));
    WriteByte(header);
    for (unsigned i = 0; i < Turnaround(); ++i) TurnToTarget();
    uint8_t ack = 0;
    for (int i = 0; i < 3; ++i) ack |= static_cast<uint8_t>(ReadBit()) << i;
    resp.ack = ack == 1   ? SwdProtocol::Ack::OK
               : ack == 2 ? SwdProtocol::Ack::WAIT
               : ack == 4 ? SwdProtocol::Ack::FAULT
               : ack == 0 ? SwdProtocol::Ack::NO_ACK
                          : SwdProtocol::Ack::PROTOCOL;
    resp.rdata = 0;
    resp.parity_ok = true;
    if (resp.ack != SwdProtocol::Ack::OK)
    {
      // 数据阶段（DAP_SWD_Configure 的 data phase，配合 ORUNDETECT）：读是 33 个空读
      // 后转向，写是转向后 33 个空写，同 CMSIS-DAP 参考实现。
      // Data phase (data phase of DAP_SWD_Configure, used with ORUNDETECT): a read clocks
      // 33 dummy cycles and then turns around, a write turns around and clocks 33 dummy
      // cycles, as in the CMSIS-DAP reference.
      const bool data_phase = DataPhase() && (resp.ack == SwdProtocol::Ack::WAIT ||
                                              resp.ack == SwdProtocol::Ack::FAULT);
      if (data_phase && req.rnw)
        for (int i = 0; i < 33; ++i) (void)ReadBit();
      TurnToHost();
      if (data_phase && !req.rnw)
        for (int i = 0; i < 33; ++i) WriteBit(false);
      return ErrorCode::OK;
    }
    if (req.rnw)
    {
      uint32_t data = 0;
      for (int i = 0; i < 32; ++i) data |= static_cast<uint32_t>(ReadBit()) << i;
      const bool parity = ReadBit();
      resp.rdata = data;
      resp.parity_ok = parity == Parity(data);
      TurnToHost();
    }
    else
    {
      TurnToHost();
      for (int i = 0; i < 32; ++i) WriteBit(((req.wdata >> i) & 1u) != 0u);
      WriteBit(Parity(req.wdata) != corrupt_write_parity);
    }
    return ErrorCode::OK;
  }

  void IdleClocks(uint32_t cycles) override
  {
    for (uint32_t i = 0; i < cycles; ++i) WriteBit(false);
  }

  ErrorCode SeqWriteBits(uint32_t cycles, const uint8_t* data) override
  {
    for (uint32_t i = 0; i < cycles; ++i) WriteBit(((data[i / 8] >> (i % 8)) & 1u) != 0u);
    return ErrorCode::OK;
  }

  ErrorCode SeqReadBits(uint32_t cycles, uint8_t* out) override
  {
    std::memset(out, 0, (cycles + 7) / 8);
    wire_.SetHostDriving(false);
    for (uint32_t i = 0; i < cycles; ++i)
      if (ReadBit()) out[i / 8] = static_cast<uint8_t>(out[i / 8] | (1u << (i % 8)));
    return ErrorCode::OK;
  }

  uint32_t clock_hz = 0;
  bool corrupt_write_parity = false;  ///< 写数据校验位取反（测试用）/ flip write parity

  /// 主机侧的转向周期数与数据阶段（DAP_SWD_Configure 的效果），读 `TransferPolicy`。
  /// The host-side turnaround cycles and data phase (what DAP_SWD_Configure sets), read
  /// from `TransferPolicy`.
  uint8_t Turnaround() const { return GetTransferPolicy().turnaround; }
  bool DataPhase() const { return GetTransferPolicy().data_phase; }

 private:
  static bool Parity(uint32_t v)
  {
    v ^= v >> 16;
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return (v & 1u) != 0u;
  }
  void WriteBit(bool bit)
  {
    wire_.SetHostDriving(true);
    wire_.SetHostLevel(bit);
    wire_.Rising();
  }
  void WriteByte(uint8_t b)
  {
    for (int i = 0; i < 8; ++i) WriteBit(((b >> i) & 1u) != 0u);
  }
  bool ReadBit()
  {
    const bool bit = wire_.Line();
    wire_.Rising();
    return bit;
  }
  void TurnToTarget()
  {
    wire_.SetHostDriving(false);
    wire_.Rising();
  }
  void TurnToHost()
  {
    wire_.SetHostDriving(false);
    for (unsigned i = 0; i < Turnaround(); ++i) wire_.Rising();
    wire_.SetHostDriving(true);
  }

  WireTarget& wire_;
};

/// 设置主机侧的转向周期与数据阶段（写进 `TransferPolicy`，DAP_SWD_Configure 的效果）
/// Set the host-side turnaround cycles and data phase (written to `TransferPolicy`, what
/// DAP_SWD_Configure does)
inline void SetWireConfig(Swd& swd, uint8_t turnaround, bool data_phase)
{
  Swd::TransferPolicy policy = swd.GetTransferPolicy();
  policy.turnaround = turnaround;
  policy.data_phase = data_phase;
  swd.SetTransferPolicy(policy);
}

/// SWCLK 引脚：上升沿推进目标 / SWCLK pin: a rising edge advances the target
class SimSwclkPin : public GPIO
{
 public:
  explicit SimSwclkPin(WireTarget& wire) : wire_(wire) {}
  bool Read() override { return level_; }
  void Write(bool value) override
  {
    if (value && !level_) wire_.Rising();
    level_ = value;
  }
  ErrorCode EnableInterrupt() override { return ErrorCode::OK; }
  ErrorCode DisableInterrupt() override { return ErrorCode::OK; }
  ErrorCode SetConfig(Configuration) override { return ErrorCode::OK; }

 private:
  WireTarget& wire_;
  bool level_ = false;
};

/// SWDIO 引脚：输出时是主机驱动，输入时读线上电平 / SWDIO pin: driven by the host as an
/// output, reads the line as an input
class SimSwdioPin : public GPIO
{
 public:
  explicit SimSwdioPin(WireTarget& wire) : wire_(wire) {}
  bool Read() override { return wire_.Line(); }
  void Write(bool value) override { wire_.SetHostLevel(value); }
  ErrorCode EnableInterrupt() override { return ErrorCode::OK; }
  ErrorCode DisableInterrupt() override { return ErrorCode::OK; }
  ErrorCode SetConfig(Configuration config) override
  {
    wire_.SetHostDriving(config.direction != Direction::INPUT);
    return ErrorCode::OK;
  }

 private:
  WireTarget& wire_;
};

}  // namespace LibXR::Debug::Sim
