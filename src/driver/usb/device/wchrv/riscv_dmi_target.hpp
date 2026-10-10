#pragma once

#include <cstdint>

#include "debug/rvswd.hpp"
#include "libxr_def.hpp"

namespace LibXR::USB
{
template <typename RvSwdPort>
class RiscvDmiTarget
{
 public:
  struct WchTargetIdentity
  {
    uint32_t chip_id = 0u;
    uint16_t flash_size_kb = 0u;
    uint32_t uid_word0 = 0u;
    uint32_t uid_word1 = 0u;
  };

  struct WchLinkCompatDescriptor
  {
    uint32_t write_pack_size = 4096u;
    uint16_t data_packet_size = 256u;
    uint8_t rom_ram_class = 0u;
    uint8_t extended_rom_ram_class = 0u;
  };

  struct MemoryWriteSessionState
  {
    bool active = false;
    bool autoexec_supported = false;
    bool autoexec_enabled = false;
    uint32_t abstractauto_saved = 0u;
  };

  enum class RunProgramFailureStage : uint8_t
  {
    NONE = 0u,
    READ_DCSR,
    WRITE_DCSR,
    WRITE_MSTATUS,
    WRITE_MIE,
    WRITE_DPC,
    VERIFY_DPC,
    ACK_HAVE_RESET,
    PRIME_HALTREQ_FIRST,
    PRIME_HALTREQ_SECOND,
    CLEAR_HALTREQ,
    RESUME_REQ,
    WAIT_HALT,
    READ_A0,
  };

  struct RunProgramDebugSnapshot
  {
    uint8_t failure_stage = static_cast<uint8_t>(RunProgramFailureStage::NONE);
    uint8_t resume_ack_seen = 0u;
    uint8_t hart_halted_seen = 0u;
    uint8_t a0_valid = 0u;
    uint32_t dmstatus_before_resume = 0u;
    uint32_t dmstatus_after_resume = 0u;
    uint32_t dmstatus_timeout_last = 0u;
    uint32_t dmstatus_after_halt = 0u;
    uint32_t dmcontrol_after_resume = 0u;
    uint32_t a0_result = 0u;
    uint32_t dcsr = 0u;
    uint32_t dpc = 0u;
    uint32_t mepc = 0u;
    uint32_t mcause = 0u;
  };

  explicit RiscvDmiTarget(RvSwdPort& rvswd_link) : rvswd_(rvswd_link) {}

  ErrorCode DmiRead(uint8_t addr, uint32_t& data, LibXR::Debug::RvSwd::Ack& ack)
  {
    return rvswd_.DmiReadTxn(addr, data, ack);
  }

  ErrorCode DmiWrite(uint8_t addr, uint32_t data, LibXR::Debug::RvSwd::Ack& ack)
  {
    return rvswd_.DmiWriteTxn(addr, data, ack);
  }

  ErrorCode DmiNop(uint8_t addr, uint32_t& data, LibXR::Debug::RvSwd::Ack& ack)
  {
    LibXR::Debug::RvSwd::Response resp = {};
    const ErrorCode RESULT = rvswd_.Transfer({addr, 0u, LibXR::Debug::RvSwd::Op::NOP}, resp);
    data = resp.data;
    ack = resp.ack;
    return RESULT;
  }

  bool ReadDmStatus(uint32_t& dmstatus) { return DmiReadWord(DMI_DMSTATUS, dmstatus); }

  static bool IsDmStatusHalted(uint32_t dmstatus)
  {
    return (dmstatus & (1u << 9u)) != 0u && (dmstatus & (1u << 8u)) != 0u;
  }

  static bool IsDmStatusRunning(uint32_t dmstatus)
  {
    return (dmstatus & (1u << 11u)) != 0u && (dmstatus & (1u << 10u)) != 0u;
  }

  static bool IsDmStatusHaveResetLatched(uint32_t dmstatus)
  {
    return (dmstatus & (1u << 19u)) != 0u || (dmstatus & (1u << 18u)) != 0u;
  }

  static bool IsDmStatusResumeAck(uint32_t dmstatus)
  {
    return (dmstatus & (1u << 17u)) != 0u && (dmstatus & (1u << 16u)) != 0u;
  }

  bool WaitForHartHalted(uint32_t max_polls = 32u, uint32_t* last_dmstatus = nullptr)
  {
    if (last_dmstatus)
    {
      *last_dmstatus = 0u;
    }
    for (uint32_t i = 0u; i < max_polls; ++i)
    {
      uint32_t dmstatus = 0u;
      if (!ReadDmStatus(dmstatus))
      {
        return false;
      }
      if (last_dmstatus)
      {
        *last_dmstatus = dmstatus;
      }
      if (IsDmStatusHalted(dmstatus))
      {
        return true;
      }
    }
    return false;
  }

  bool WaitForHartRunning(uint32_t max_polls = 32u)
  {
    for (uint32_t i = 0u; i < max_polls; ++i)
    {
      uint32_t dmstatus = 0u;
      if (!ReadDmStatus(dmstatus))
      {
        return false;
      }
      if (IsDmStatusRunning(dmstatus))
      {
        return true;
      }
    }
    return false;
  }

  bool RequestHartHalt()
  {
    if (!DmiWriteWord(DMI_DMCONTROL, 0x80000001u))
    {
      return false;
    }
    if (!WaitForHartHalted())
    {
      return false;
    }
    return DmiWriteWord(DMI_DMCONTROL, 0x00000001u);
  }

  bool RequestHartResume()
  {
    if (!DmiWriteWord(DMI_DMCONTROL, 0x40000001u))
    {
      return false;
    }
    const bool RUNNING = WaitForHartRunning();
    (void)DmiWriteWord(DMI_DMCONTROL, 0x00000001u);
    return RUNNING;
  }

  bool ReadWordByAbstract(uint32_t addr, uint32_t& data)
  {
    EndMemoryWriteSession();
    if (!DmiWriteWord(DMI_PROGBUF0, 0x0002A303u))  // lw x6, 0(x5)
    {
      return false;
    }
    if (!DmiWriteWord(DMI_PROGBUF1, 0x00100073u))  // ebreak
    {
      return false;
    }
    if (!DmiWriteWord(DMI_DATA0, addr))
    {
      return false;
    }
    if (!ClearAbstractCommandError())
    {
      return false;
    }
    if (!RunAbstractCommand(0x00271005u))  // x5 <- data0 with postexec
    {
      return false;
    }
    if (!RunAbstractCommand(0x00221006u))  // data0 <- x6
    {
      return false;
    }
    return DmiReadWord(DMI_DATA0, data);
  }

  bool WriteWordByAbstract(uint32_t addr, uint32_t data)
  {
    EndMemoryWriteSession();
    if (WriteWordByWchFast(addr, data))
    {
      return true;
    }

    if (!DmiWriteWord(DMI_PROGBUF0, 0x0072A023u))  // sw x7, 0(x5)
    {
      return false;
    }
    if (!DmiWriteWord(DMI_PROGBUF1, 0x00100073u))  // ebreak
    {
      return false;
    }

    if (!DmiWriteWord(DMI_DATA0, addr))
    {
      return false;
    }
    if (!ClearAbstractCommandError())
    {
      return false;
    }
    if (!RunAbstractCommand(0x00231005u))  // x5 <- data0
    {
      return false;
    }

    if (!DmiWriteWord(DMI_DATA0, data))
    {
      return false;
    }
    if (!ClearAbstractCommandError())
    {
      return false;
    }
    return RunAbstractCommand(0x00271007u);  // x7 <- data0 with postexec
  }

  bool WriteByteByAbstract(uint32_t addr, uint8_t data)
  {
    EndMemoryWriteSession();
    if (!DmiWriteWord(DMI_PROGBUF0, 0x00728023u))  // sb x7, 0(x5)
    {
      return false;
    }
    if (!DmiWriteWord(DMI_PROGBUF1, 0x00100073u))  // ebreak
    {
      return false;
    }

    if (!DmiWriteWord(DMI_DATA0, addr))
    {
      return false;
    }
    if (!ClearAbstractCommandError())
    {
      return false;
    }
    if (!RunAbstractCommand(0x00231005u))  // x5 <- data0
    {
      return false;
    }

    if (!DmiWriteWord(DMI_DATA0, static_cast<uint32_t>(data)))
    {
      return false;
    }
    if (!ClearAbstractCommandError())
    {
      return false;
    }
    return RunAbstractCommand(0x00271007u);  // x7 <- data0 with postexec
  }

  bool ReadWordByWchFast(uint32_t addr, uint32_t& data)
  {
    if (memory_write_session_.active)
    {
      EndMemoryWriteSession();
    }
    if ((addr & 0x3u) != 0u)
    {
      return false;
    }

    return DmiWriteWord(DMI_DATA1, addr) && DmiWriteWord(DMI_COMMAND, COMMAND_WCH_READ_MEM32) &&
           DmiReadWord(DMI_DATA0, data);
  }

  bool WriteWordByWchFast(uint32_t addr, uint32_t data)
  {
    if (memory_write_session_.active)
    {
      EndMemoryWriteSession();
    }
    if ((addr & 0x3u) != 0u)
    {
      return false;
    }

    return DmiWriteWord(DMI_DATA1, addr) && DmiWriteWord(DMI_DATA0, data) &&
           DmiWriteWord(DMI_COMMAND, COMMAND_WCH_WRITE_MEM32);
  }

  bool WriteWordByWchFastVerified(uint32_t addr, uint32_t data)
  {
    uint32_t readback = 0u;
    return WriteWordByWchFast(addr, data) && ReadWordByWchFast(addr, readback) &&
           readback == data;
  }

  bool RunWchCustomCommand(uint32_t data1, uint32_t command)
  {
    if (memory_write_session_.active)
    {
      EndMemoryWriteSession();
    }
    return DmiWriteWord(DMI_DATA1, data1) && DmiWriteWord(DMI_COMMAND, command) &&
           WaitAbstractCommandDone();
  }

  void RecoverDebugModuleAfterFault()
  {
    EndMemoryWriteSession();
    (void)DmiWriteWord(DMI_DMCONTROL, 0x00000000u);
    (void)DmiWriteWord(DMI_DMCONTROL, 0x00000001u);
    (void)DmiWriteWord(DMI_DMCONTROL, 0x00000003u);
    (void)DmiWriteWord(DMI_DMCONTROL, 0x00000001u);
    (void)ClearAbstractCommandError();
    (void)DmiWriteWord(DMI_DMCONTROL, 0x80000001u);
    (void)DmiWriteWord(DMI_DMCONTROL, 0x00000001u);
    (void)ClearAbstractCommandError();
  }

  bool RunWchPostEraseSettle(uint16_t dmstatus_polls)
  {
    EndMemoryWriteSession();
    if (!DmiWriteWord(DMI_DMCONTROL, 0x40000001u) ||
        !DmiWriteWord(DMI_DMCONTROL, 0x80000001u) ||
        !DmiWriteWord(DMI_DMCONTROL, 0x80000001u))
    {
      return false;
    }

    for (uint16_t poll = 0u; poll < dmstatus_polls; ++poll)
    {
      uint32_t dmstatus = 0u;
      (void)DmiReadWord(DMI_DMSTATUS, dmstatus);
    }
    return true;
  }

  bool ReadWordWithTemporaryHalt(uint32_t addr, uint32_t& data)
  {
    bool need_resume = false;
    if (!EnsureHartHaltedForProbe(need_resume))
    {
      return false;
    }

    const bool READ_OK = ReadWordByAbstract(addr, data);
    TryResumeHartAfterProbe(need_resume);
    return READ_OK;
  }

  bool BeginHartHaltSession(bool& need_resume)
  {
    return EnsureHartHaltedForProbe(need_resume);
  }

  void EndHartHaltSession(bool need_resume)
  {
    EndMemoryWriteSession();
    TryResumeHartAfterProbe(need_resume);
  }

  bool BeginMemoryWriteSession(uint32_t addr)
  {
    EndMemoryWriteSession();

    uint32_t abstractcs = 0u;
    if (!DmiReadWord(DMI_ABSTRACTCS, abstractcs))
    {
      return false;
    }

    const uint8_t PROGBUF_SIZE = static_cast<uint8_t>((abstractcs >> 24u) & 0x1Fu);
    const uint8_t DATA_COUNT = static_cast<uint8_t>(abstractcs & 0xFu);
    if (PROGBUF_SIZE < 3u || DATA_COUNT == 0u)
    {
      return false;
    }

    if (!DmiWriteWord(DMI_PROGBUF0, 0x0062A023u))  // sw x6, 0(x5)
    {
      return false;
    }
    if (!DmiWriteWord(DMI_PROGBUF1, 0x00428293u))  // addi x5, x5, 4
    {
      return false;
    }
    if (!DmiWriteWord(DMI_PROGBUF2, 0x00100073u))  // ebreak
    {
      return false;
    }
    if (!DmiWriteWord(DMI_DATA0, addr))
    {
      return false;
    }
    if (!ClearAbstractCommandError())
    {
      return false;
    }
    if (!RunAbstractCommand(COMMAND_WRITE_X5))
    {
      return false;
    }

    uint32_t saved_abstractauto = 0u;
    const bool AUTOEXEC_READY = DmiReadWord(DMI_ABSTRACTAUTO, saved_abstractauto);

    memory_write_session_.active = true;
    memory_write_session_.autoexec_enabled = false;
    memory_write_session_.autoexec_supported = AUTOEXEC_READY;
    memory_write_session_.abstractauto_saved = AUTOEXEC_READY ? saved_abstractauto : 0u;
    return true;
  }

  bool WriteMemoryWordStreaming(uint32_t data)
  {
    if (!memory_write_session_.active)
    {
      return false;
    }

    if (!DmiWriteWord(DMI_DATA0, data))
    {
      return false;
    }

    if (!memory_write_session_.autoexec_enabled)
    {
      if (!ClearAbstractCommandError())
      {
        return false;
      }
      if (!RunAbstractCommand(COMMAND_WRITE_X6_POSTEXEC))
      {
        return false;
      }

      if (memory_write_session_.autoexec_supported &&
          TryEnableMemoryWriteAutoexec(memory_write_session_.abstractauto_saved))
      {
        memory_write_session_.autoexec_enabled = true;
      }
      else
      {
        memory_write_session_.autoexec_supported = false;
      }
      return true;
    }

    return WaitAbstractCommandDone();
  }

  void EndMemoryWriteSession()
  {
    if (!memory_write_session_.active)
    {
      return;
    }

    // Always restore ABSTRACTAUTO so a stale autoexec bit left by an earlier
    // session cannot poison later abstract reads on a fresh probe attach.
    (void)DmiWriteWord(DMI_ABSTRACTAUTO, memory_write_session_.abstractauto_saved);
    memory_write_session_ = {};
    (void)ClearAbstractCommandError();
  }

  bool WriteMemoryBlock(uint32_t addr, const uint8_t* data, uint32_t len)
  {
    if (!data || len == 0u)
    {
      return true;
    }

    uint32_t off = 0u;
    bool streaming = false;
    while (off < len)
    {
      const uint32_t CUR_ADDR = addr + off;
      if ((CUR_ADDR & 0x3u) == 0u && (len - off) >= 4u)
      {
        while (off < len)
        {
          const uint32_t STREAM_ADDR = addr + off;
          if ((STREAM_ADDR & 0x3u) != 0u || (len - off) < 4u)
          {
            break;
          }

          const uint32_t WORD = static_cast<uint32_t>(data[off]) |
                                (static_cast<uint32_t>(data[off + 1u]) << 8u) |
                                (static_cast<uint32_t>(data[off + 2u]) << 16u) |
                                (static_cast<uint32_t>(data[off + 3u]) << 24u);
          if (!streaming && WriteWordByWchFast(STREAM_ADDR, WORD))
          {
            off += 4u;
            continue;
          }

          if (!streaming)
          {
            streaming = BeginMemoryWriteSession(STREAM_ADDR);
          }
          if (!streaming || !WriteMemoryWordStreaming(WORD))
          {
            EndMemoryWriteSession();
            return false;
          }
          off += 4u;
        }
        continue;
      }

      if (streaming)
      {
        EndMemoryWriteSession();
        streaming = false;
      }
      if (!WriteByteByAbstract(CUR_ADDR, data[off]))
      {
        return false;
      }
      ++off;
    }

    if (streaming)
    {
      EndMemoryWriteSession();
    }
    return true;
  }

  bool WriteCpuRegister(uint16_t regno, uint32_t value)
  {
    return AccessCpuRegister(regno, value, true);
  }

  bool ReadCpuRegister(uint16_t regno, uint32_t& value)
  {
    return AccessCpuRegister(regno, value, false);
  }

  bool RunProgramAndWaitForHalt(
      uint32_t pc, uint32_t& a0_result, uint32_t max_halt_polls = 1024u,
      RunProgramDebugSnapshot* debug_snapshot = nullptr)
  {
    if (debug_snapshot)
    {
      *debug_snapshot = {};
    }

    auto record_failure = [&](RunProgramFailureStage stage) {
      if (debug_snapshot)
      {
        debug_snapshot->failure_stage = static_cast<uint8_t>(stage);
      }
    };
    auto capture_dmcontrol_after_resume = [&]() {
      if (!debug_snapshot)
      {
        return;
      }

      uint32_t dmcontrol = 0u;
      if (DmiReadWord(DMI_DMCONTROL, dmcontrol))
      {
        debug_snapshot->dmcontrol_after_resume = dmcontrol;
      }
    };
    auto capture_halt_snapshot = [&]() {
      if (!debug_snapshot)
      {
        return;
      }

      uint32_t dmstatus = 0u;
      if (!ReadDmStatus(dmstatus))
      {
        return;
      }

      debug_snapshot->dmstatus_after_halt = dmstatus;
      if (IsDmStatusResumeAck(dmstatus))
      {
        debug_snapshot->resume_ack_seen = 1u;
      }
      if (!IsDmStatusHalted(dmstatus))
      {
        return;
      }

      debug_snapshot->hart_halted_seen = 1u;

      uint32_t reg_value = 0u;
      if (ReadCpuRegister(REG_DCSR, reg_value))
      {
        debug_snapshot->dcsr = reg_value;
      }
      if (ReadCpuRegister(REG_A0, reg_value))
      {
        debug_snapshot->a0_result = reg_value;
        debug_snapshot->a0_valid = 1u;
      }
      if (ReadCpuRegister(REG_DPC, reg_value))
      {
        debug_snapshot->dpc = reg_value;
      }
      if (ReadCpuRegister(REG_MEPC, reg_value))
      {
        debug_snapshot->mepc = reg_value;
      }
      if (ReadCpuRegister(REG_MCAUSE, reg_value))
      {
        debug_snapshot->mcause = reg_value;
      }
    };

    uint32_t saved_dcsr = 0u;
    if (!ReadCpuRegister(REG_DCSR, saved_dcsr))
    {
      record_failure(RunProgramFailureStage::READ_DCSR);
      return false;
    }

    const uint32_t DCSR_WITH_EBREAKM = saved_dcsr | DCSR_EBREAK_M;
    const bool RESTORE_DCSR = DCSR_WITH_EBREAKM != saved_dcsr;
    if (RESTORE_DCSR && !WriteCpuRegister(REG_DCSR, DCSR_WITH_EBREAKM))
    {
      record_failure(RunProgramFailureStage::WRITE_DCSR);
      return false;
    }
    if (RESTORE_DCSR)
    {
      uint32_t dcsr_verify = 0u;
      bool dcsr_ready = ReadCpuRegister(REG_DCSR, dcsr_verify) && (dcsr_verify & DCSR_EBREAK_M) != 0u;
      if (!dcsr_ready)
      {
        if (!WriteCpuRegister(REG_DCSR, DCSR_WITH_EBREAKM) ||
            !ReadCpuRegister(REG_DCSR, dcsr_verify) || (dcsr_verify & DCSR_EBREAK_M) == 0u)
        {
          record_failure(RunProgramFailureStage::WRITE_DCSR);
          return false;
        }
      }
    }

    uint32_t saved_mstatus = 0u;
    uint32_t saved_mie = 0u;
    bool restore_mstatus = false;
    bool restore_mie = false;

    auto restore_dcsr = [&]() {
      if (restore_mie)
      {
        (void)WriteCpuRegister(REG_MIE, saved_mie);
      }
      if (restore_mstatus)
      {
        (void)WriteCpuRegister(REG_MSTATUS, saved_mstatus);
      }
      if (RESTORE_DCSR)
      {
        (void)WriteCpuRegister(REG_DCSR, saved_dcsr);
      }
    };

    // WCH-Link clears mstatus before running the RAM flash algorithm; keeping
    // WCH-specific interrupt bits set lets pending PFIC interrupts preempt it.
    restore_mstatus = ReadCpuRegister(REG_MSTATUS, saved_mstatus);
    if (!WriteCpuRegister(REG_MSTATUS, 0u))
    {
      record_failure(RunProgramFailureStage::WRITE_MSTATUS);
      restore_dcsr();
      return false;
    }
    restore_mie = ReadCpuRegister(REG_MIE, saved_mie);
    if (!WriteCpuRegister(REG_MIE, 0u))
    {
      record_failure(RunProgramFailureStage::WRITE_MIE);
      restore_dcsr();
      return false;
    }

    if (!WriteCpuRegister(REG_DPC, pc))
    {
      record_failure(RunProgramFailureStage::WRITE_DPC);
      restore_dcsr();
      return false;
    }
    {
      uint32_t dpc_verify = 0u;
      bool dpc_ready = ReadCpuRegister(REG_DPC, dpc_verify) && dpc_verify == pc;
      if (!dpc_ready)
      {
        if (!WriteCpuRegister(REG_DPC, pc) || !ReadCpuRegister(REG_DPC, dpc_verify) ||
            dpc_verify != pc)
        {
          if (debug_snapshot)
          {
            debug_snapshot->dpc = dpc_verify;
          }
          record_failure(RunProgramFailureStage::VERIFY_DPC);
          restore_dcsr();
          return false;
        }
      }
    }

    uint32_t dmstatus_before_resume = 0u;
    if (debug_snapshot && ReadDmStatus(dmstatus_before_resume))
    {
      debug_snapshot->dmstatus_before_resume = dmstatus_before_resume;
    }

    if (!DmiWriteWord(DMI_DMCONTROL, 0x40000001u))
    {
      record_failure(RunProgramFailureStage::RESUME_REQ);
      capture_halt_snapshot();
      restore_dcsr();
      return false;
    }

    if (debug_snapshot)
    {
      for (uint8_t i = 0u; i < 32u; ++i)
      {
        uint32_t dmstatus = 0u;
        if (!ReadDmStatus(dmstatus))
        {
          break;
        }
        debug_snapshot->dmstatus_after_resume = dmstatus;
        if (IsDmStatusResumeAck(dmstatus))
        {
          debug_snapshot->resume_ack_seen = 1u;
        }
        if (IsDmStatusRunning(dmstatus) || IsDmStatusResumeAck(dmstatus))
        {
          break;
        }
      }
      capture_dmcontrol_after_resume();
    }

    uint32_t dmstatus_after_halt = 0u;
    if (!WaitForHartHalted(max_halt_polls, &dmstatus_after_halt))
    {
      record_failure(RunProgramFailureStage::WAIT_HALT);
      if (debug_snapshot)
      {
        debug_snapshot->dmstatus_timeout_last = dmstatus_after_halt;
        debug_snapshot->dmstatus_after_halt = dmstatus_after_halt;
        if (IsDmStatusResumeAck(dmstatus_after_halt))
        {
          debug_snapshot->resume_ack_seen = 1u;
        }
        if (IsDmStatusHalted(dmstatus_after_halt))
        {
          debug_snapshot->hart_halted_seen = 1u;
        }
      }
      (void)RequestHartHalt();
      capture_halt_snapshot();
      restore_dcsr();
      return false;
    }

    if (debug_snapshot)
    {
      debug_snapshot->dmstatus_timeout_last = dmstatus_after_halt;
      debug_snapshot->dmstatus_after_halt = dmstatus_after_halt;
      debug_snapshot->hart_halted_seen = 1u;
      if (IsDmStatusResumeAck(dmstatus_after_halt))
      {
        debug_snapshot->resume_ack_seen = 1u;
      }
      uint32_t reg_value = 0u;
      if (ReadCpuRegister(REG_DCSR, reg_value))
      {
        debug_snapshot->dcsr = reg_value;
      }
    }

    if (!ReadCpuRegister(REG_A0, a0_result))
    {
      record_failure(RunProgramFailureStage::READ_A0);
      capture_halt_snapshot();
      restore_dcsr();
      return false;
    }

    if (debug_snapshot)
    {
      debug_snapshot->a0_result = a0_result;
      debug_snapshot->a0_valid = 1u;
    }

    restore_dcsr();
    (void)DmiWriteWord(DMI_DMCONTROL, 0x00000001u);
    return true;
  }

  bool ReadWchChipId(uint32_t& chip_id)
  {
    return ReadWordWithTemporaryHalt(WCH_CHIP_ID_ADDRESS, chip_id) && chip_id != 0u &&
           chip_id != 0xFFFFFFFFu;
  }

  bool ReadWchFlashSizeKb(uint16_t& flash_size_kb)
  {
    uint32_t flash_size_raw = 0u;
    if (!ReadWordWithTemporaryHalt(WCH_FLASH_SIZE_ADDRESS, flash_size_raw))
    {
      return false;
    }

    flash_size_kb = static_cast<uint16_t>(flash_size_raw & 0xFFFFu);
    return IsValidWchFlashSizeKb(flash_size_kb);
  }

  bool ReadWchUidWords(uint32_t& uid_word0, uint32_t& uid_word1)
  {
    return ReadWordWithTemporaryHalt(WCH_UID_WORD0_ADDRESS, uid_word0) &&
           ReadWordWithTemporaryHalt(WCH_UID_WORD1_ADDRESS, uid_word1);
  }

  bool ReadWchTargetIdentity(WchTargetIdentity& identity, bool include_uid = false)
  {
    if (!ReadWchChipId(identity.chip_id) || !ReadWchFlashSizeKb(identity.flash_size_kb))
    {
      return false;
    }
    if (!include_uid)
    {
      return true;
    }
    return ReadWchUidWords(identity.uid_word0, identity.uid_word1);
  }

  static uint8_t DetectWchChipFamilyFromChipId(uint32_t chip_id)
  {
    switch (chip_id & 0xFFF00000u)
    {
      case 0x20300000u:
      case 0x20800000u:
        return 0x05u;
      case 0x30300000u:
      case 0x30500000u:
      case 0x30700000u:
        return 0x06u;
      case 0x31700000u:
        return 0x86u;
      default:
        return 0u;
    }
  }

  static WchLinkCompatDescriptor BuildWchLinkCompatDescriptor(uint8_t chip_family,
                                                              uint16_t flash_size_kb)
  {
    WchLinkCompatDescriptor compat = {};
    switch (chip_family)
    {
      case 0x01u:
        compat.data_packet_size = 128u;
        break;
      case 0x09u:
      case 0x49u:
        compat.data_packet_size = 64u;
        break;
      default:
        compat.data_packet_size = 256u;
        break;
    }

    switch (chip_family)
    {
      case 0x09u:
      case 0x49u:
      case 0x4Eu:
        compat.write_pack_size = 1024u;
        break;
      default:
        compat.write_pack_size = 4096u;
        break;
    }

    switch (chip_family)
    {
      case 0x05u:
        switch (flash_size_kb)
        {
          case 128u:
            compat.rom_ram_class = 0u;
            break;
          case 144u:
            compat.rom_ram_class = 1u;
            break;
          case 160u:
            compat.rom_ram_class = 2u;
            break;
          default:
            break;
        }
        break;
      case 0x06u:
        switch (flash_size_kb)
        {
          case 192u:
            compat.rom_ram_class = 0u;
            compat.extended_rom_ram_class = 0u;
            break;
          case 224u:
            compat.rom_ram_class = 1u;
            compat.extended_rom_ram_class = 2u;
            break;
          case 256u:
            compat.rom_ram_class = 2u;
            compat.extended_rom_ram_class = 4u;
            break;
          case 288u:
            compat.rom_ram_class = 3u;
            compat.extended_rom_ram_class = 7u;
            break;
          default:
            break;
        }
        break;
      case 0x86u:
        switch (flash_size_kb)
        {
          case 128u:
            compat.extended_rom_ram_class = 6u;
            break;
          case 192u:
            compat.extended_rom_ram_class = 0u;
            break;
          case 224u:
            compat.extended_rom_ram_class = 2u;
            break;
          case 256u:
            compat.extended_rom_ram_class = 4u;
            break;
          case 288u:
            compat.extended_rom_ram_class = 7u;
            break;
          default:
            break;
        }
        break;
      default:
        break;
    }

    return compat;
  }

 private:
  static bool IsValidWchFlashSizeKb(uint32_t flash_size_kb)
  {
    return flash_size_kb != 0u && flash_size_kb != 0xFFFFu && flash_size_kb <= 1024u;
  }

  bool DmiReadWord(uint8_t addr, uint32_t& data)
  {
    LibXR::Debug::RvSwd::Ack ack = LibXR::Debug::RvSwd::Ack::PROTOCOL;
    const ErrorCode RESULT = rvswd_.DmiReadTxn(addr, data, ack);
    return RESULT == ErrorCode::OK && ack == LibXR::Debug::RvSwd::Ack::OK;
  }

  bool DmiWriteWord(uint8_t addr, uint32_t data)
  {
    LibXR::Debug::RvSwd::Ack ack = LibXR::Debug::RvSwd::Ack::PROTOCOL;
    const ErrorCode RESULT = rvswd_.DmiWriteTxn(addr, data, ack);
    return RESULT == ErrorCode::OK && ack == LibXR::Debug::RvSwd::Ack::OK;
  }

  bool ClearAbstractCommandError() { return DmiWriteWord(DMI_ABSTRACTCS, 0x00000700u); }

  static uint32_t MakeRegisterAccessCommand(uint16_t regno, bool write, bool postexec = false)
  {
    return 0x00200000u | (postexec ? 0x00040000u : 0u) | 0x00020000u |
           (write ? 0x00010000u : 0u) | static_cast<uint32_t>(regno);
  }

  bool TryEnableMemoryWriteAutoexec(uint32_t saved_abstractauto)
  {
    const uint32_t REQUESTED = saved_abstractauto | ABSTRACTAUTO_EXEC_DATA0;
    if (!DmiWriteWord(DMI_ABSTRACTAUTO, REQUESTED))
    {
      return false;
    }

    uint32_t confirmed_abstractauto = 0u;
    if (!DmiReadWord(DMI_ABSTRACTAUTO, confirmed_abstractauto) ||
        (confirmed_abstractauto & ABSTRACTAUTO_EXEC_DATA0) == 0u)
    {
      (void)DmiWriteWord(DMI_ABSTRACTAUTO, saved_abstractauto);
      return false;
    }
    return true;
  }

  bool WaitAbstractCommandDone()
  {
    for (uint8_t i = 0u; i < 32u; ++i)
    {
      uint32_t abstractcs = 0u;
      if (!DmiReadWord(DMI_ABSTRACTCS, abstractcs))
      {
        return false;
      }
      if ((abstractcs & (1u << 12u)) != 0u)
      {
        continue;
      }
      if (((abstractcs >> 8u) & 0x7u) != 0u)
      {
        (void)ClearAbstractCommandError();
        return false;
      }
      return true;
    }
    return false;
  }

  bool RunAbstractCommand(uint32_t command)
  {
    if (!DmiWriteWord(DMI_COMMAND, command))
    {
      return false;
    }
    return WaitAbstractCommandDone();
  }

  bool AccessCpuRegister(uint16_t regno, uint32_t& value, bool write)
  {
    EndMemoryWriteSession();
    if (write && !DmiWriteWord(DMI_DATA0, value))
    {
      return false;
    }
    if (!ClearAbstractCommandError() || !RunAbstractCommand(MakeRegisterAccessCommand(regno, write)))
    {
      return false;
    }
    if (write)
    {
      return true;
    }
    return DmiReadWord(DMI_DATA0, value);
  }

  bool EnsureHartHaltedForProbe(bool& need_resume)
  {
    need_resume = false;
    uint32_t dmstatus = 0u;
    if (!DmiReadWord(DMI_DMSTATUS, dmstatus))
    {
      return false;
    }
    if (IsDmStatusHalted(dmstatus))
    {
      return true;
    }

    if (!RequestHartHalt())
    {
      return false;
    }
    need_resume = true;
    return true;
  }

  void TryResumeHartAfterProbe(bool need_resume)
  {
    if (!need_resume)
    {
      return;
    }

    (void)RequestHartResume();
  }

 private:
  static constexpr uint8_t DMI_DATA0 = 0x04u;
  static constexpr uint8_t DMI_DATA1 = 0x05u;
  static constexpr uint8_t DMI_DMCONTROL = 0x10u;
  static constexpr uint8_t DMI_DMSTATUS = 0x11u;
  static constexpr uint8_t DMI_ABSTRACTCS = 0x16u;
  static constexpr uint8_t DMI_COMMAND = 0x17u;
  static constexpr uint8_t DMI_ABSTRACTAUTO = 0x18u;
  static constexpr uint8_t DMI_PROGBUF0 = 0x20u;
  static constexpr uint8_t DMI_PROGBUF1 = 0x21u;
  static constexpr uint8_t DMI_PROGBUF2 = 0x22u;
  static constexpr uint32_t COMMAND_WCH_READ_MEM32 = 0x02200000u;
  static constexpr uint32_t COMMAND_WCH_WRITE_MEM32 = 0x02210000u;
  static constexpr uint32_t COMMAND_WRITE_X5 = 0x00231005u;
  static constexpr uint32_t COMMAND_WRITE_X6_POSTEXEC = 0x00271006u;
  static constexpr uint32_t ABSTRACTAUTO_EXEC_DATA0 = 0x00000001u;
  static constexpr uint16_t REG_A0 = 0x100Au;
  static constexpr uint16_t REG_MSTATUS = 0x0300u;
  static constexpr uint16_t REG_MIE = 0x0304u;
  static constexpr uint16_t REG_DCSR = 0x07B0u;
  static constexpr uint16_t REG_DPC = 0x07B1u;
  static constexpr uint16_t REG_MEPC = 0x0341u;
  static constexpr uint16_t REG_MCAUSE = 0x0342u;
  static constexpr uint32_t DCSR_EBREAK_M = 0x00008000u;
  static constexpr uint32_t WCH_CHIP_ID_ADDRESS = 0x1FFFF704u;
  static constexpr uint32_t WCH_FLASH_SIZE_ADDRESS = 0x1FFFF7E0u;
  static constexpr uint32_t WCH_UID_WORD0_ADDRESS = 0x1FFFF7E8u;
  static constexpr uint32_t WCH_UID_WORD1_ADDRESS = 0x1FFFF7ECu;

  RvSwdPort& rvswd_;
  MemoryWriteSessionState memory_write_session_ = {};
};

}  // namespace LibXR::USB
