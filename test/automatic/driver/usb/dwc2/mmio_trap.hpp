/**
 * @file mmio_trap.hpp
 * @brief 把一段固定地址的内存变成带读写语义的寄存器 / Memory at a fixed address that
 *        behaves like a register file.
 *
 * 后端源码按真实地址直接读写寄存器（`core->GINTSTS = x`、FIFO 的 `volatile uint32_t&`），
 * 测试不能改它。这里把寄存器区映射成不可访问的页：每次访问触发 SIGSEGV，处理函数把
 * 该寄存器当前的读出值放进内存、放开这一页并单步执行这条指令，随后在 SIGTRAP 里把页
 * 重新保护起来，再把写入值交给模型（或执行读的副作用，例如弹出接收 FIFO）。这样
 * 写 1 清除、自清位、读即弹出和同一地址的连续 FIFO 读写都按硬件语义发生，而后端
 * 源码原样编译。只支持 Linux x86-64，访问按 32 位字对齐处理。
 *
 * The backend source accesses registers at their real addresses (`core->GINTSTS = x`, a
 * `volatile uint32_t&` for a FIFO) and the test must not change it. The register area is
 * mapped as inaccessible pages: every access raises SIGSEGV, the handler stores the
 * current read value of that register in the memory, opens the page and single-steps the
 * instruction; the SIGTRAP that follows protects the page again and hands the written
 * value to the model (or runs the side effect of the read, such as popping the receive
 * FIFO). Write-one-to-clear bits, self-clearing bits, read-to-pop and repeated FIFO
 * accesses at one address therefore follow the hardware, with the backend source compiled
 * as it is. Linux x86-64 only; accesses are handled as aligned 32-bit words.
 *
 * 单步用的是处理器的陷阱标志，所以这样的程序不能在 gdb 里运行（gdb 自己占用单步陷阱）。
 * 不属于寄存器区的非法访问会打印地址、指令地址和调用栈再终止。
 * The single step uses the trap flag of the processor, so such a program does not run
 * under gdb (which takes the single-step trap for itself). An invalid access outside the
 * register area prints its address, the instruction address and a backtrace before the
 * program dies.
 */

#pragma once

#if !defined(__linux__) || !defined(__x86_64__)
#error "mmio_trap.hpp needs Linux on x86-64"
#endif

#include <execinfo.h>
#include <signal.h>
#include <sys/mman.h>
#include <ucontext.h>
#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace MmioTrap
{

/// 寄存器模型 / A register model behind a trapped region
class Device
{
 public:
  virtual ~Device() = default;
  /// 读出值，不带副作用 / The value a read returns, without its side effect
  virtual uint32_t MmioPeek(uint32_t offset) = 0;
  /// 一次读完成后的副作用 / Side effect of a completed read
  virtual void MmioAfterRead(uint32_t offset) = 0;
  /// 一次写完成：value 是写后整个字的内容 / A completed write: value is the whole word
  virtual void MmioWrite(uint32_t offset, uint32_t value) = 0;
};

namespace Detail
{
struct Region
{
  uintptr_t base = 0;
  size_t size = 0;
  Device* device = nullptr;
};

struct Pending
{
  Region* region = nullptr;
  uint32_t offset = 0;
  uintptr_t page = 0;
  bool write = false;
};

inline constexpr size_t REGION_COUNT = 4;
inline constexpr uintptr_t PAGE = 4096;
inline Region regions[REGION_COUNT];
inline Pending pending;
inline struct sigaction previous_segv;
inline struct sigaction previous_trap;
inline bool installed = false;
inline uintptr_t last_pc = 0;

[[noreturn]] inline void Fatal(const char* text)
{
  std::fprintf(stderr, "mmio trap: %s\n", text);
  std::abort();
}

inline Region* Find(uintptr_t address)
{
  for (auto& region : regions)
    if (region.device != nullptr && address >= region.base &&
        address < region.base + region.size)
      return &region;
  return nullptr;
}

inline void Chain(const struct sigaction& previous, int number, siginfo_t* info,
                  void* context)
{
  if ((previous.sa_flags & SA_SIGINFO) != 0 && previous.sa_sigaction != nullptr)
  {
    previous.sa_sigaction(number, info, context);
    return;
  }
  if (previous.sa_handler != SIG_DFL && previous.sa_handler != SIG_IGN &&
      previous.sa_handler != nullptr)
  {
    previous.sa_handler(number);
    return;
  }
  // Not ours and nobody else wants it: report where, then die with the default action.
  auto* user = static_cast<ucontext_t*>(context);
  std::fprintf(stderr, "mmio trap: signal %d at address %p, instruction %p\n", number,
               info->si_addr, reinterpret_cast<void*>(user->uc_mcontext.gregs[REG_RIP]));
  void* frames[32];
  backtrace_symbols_fd(frames, backtrace(frames, 32), STDERR_FILENO);
  signal(number, SIG_DFL);
  raise(number);
}

inline void OnSegv(int number, siginfo_t* info, void* raw)
{
  auto* context = static_cast<ucontext_t*>(raw);
  const auto address = reinterpret_cast<uintptr_t>(info->si_addr);
  Region* region = Find(address);
  if (region == nullptr || pending.region != nullptr)
  {
    Chain(previous_segv, number, info, raw);
    return;
  }
  pending.region = region;
  pending.offset = static_cast<uint32_t>((address - region->base) & ~uintptr_t{3});
  pending.page = address & ~(PAGE - 1U);
  // Bit 1 of the page fault error code: the access was a write. An instruction that
  // reads and then writes the same word faults as a write and sees the staged value.
  pending.write = (context->uc_mcontext.gregs[REG_ERR] & 2) != 0;
  last_pc = static_cast<uintptr_t>(context->uc_mcontext.gregs[REG_RIP]);
  const uint32_t value = region->device->MmioPeek(pending.offset);
  if (mprotect(reinterpret_cast<void*>(pending.page), PAGE, PROT_READ | PROT_WRITE) != 0)
    Fatal("mprotect failed");
  *reinterpret_cast<uint32_t*>(region->base + pending.offset) = value;
  context->uc_mcontext.gregs[REG_EFL] |= 0x100;  // Trap flag: one instruction.
}

inline void OnTrap(int number, siginfo_t* info, void* raw)
{
  auto* context = static_cast<ucontext_t*>(raw);
  if (pending.region == nullptr)
  {
    Chain(previous_trap, number, info, raw);
    return;
  }
  context->uc_mcontext.gregs[REG_EFL] &= ~static_cast<greg_t>(0x100);
  const Pending done = pending;
  const uint32_t value = *reinterpret_cast<uint32_t*>(done.region->base + done.offset);
  if (mprotect(reinterpret_cast<void*>(done.page), PAGE, PROT_NONE) != 0)
    Fatal("mprotect failed");
  pending.region = nullptr;
  if (done.write)
    done.region->device->MmioWrite(done.offset, value);
  else
    done.region->device->MmioAfterRead(done.offset);
}

inline void Install()
{
  if (installed) return;
  struct sigaction action = {};
  sigemptyset(&action.sa_mask);
  action.sa_flags = SA_SIGINFO;
  action.sa_sigaction = OnSegv;
  if (sigaction(SIGSEGV, &action, &previous_segv) != 0) Fatal("sigaction failed");
  action.sa_sigaction = OnTrap;
  if (sigaction(SIGTRAP, &action, &previous_trap) != 0) Fatal("sigaction failed");
  installed = true;
}
}  // namespace Detail

/// 把 [base, base + size) 交给 device；地址被占用时终止 / Hand [base, base + size) to
/// the device; aborts when the address range is taken
inline void Map(uintptr_t base, size_t size, Device* device)
{
  Detail::Install();
  for (auto& region : Detail::regions)
  {
    if (region.size != 0U && region.base == base)
    {
      if (region.size != size) Detail::Fatal("region mapped with another size");
      region.device = device;
      return;
    }
  }
  for (auto& region : Detail::regions)
  {
    if (region.size != 0U) continue;
    void* got = mmap(reinterpret_cast<void*>(base), size, PROT_NONE,
                     MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
    if (got != reinterpret_cast<void*>(base)) Detail::Fatal("register address in use");
    region = {base, size, device};
    return;
  }
  Detail::Fatal("too many regions");
}

/// 模型离开：地址保持映射，之后的访问按普通的非法访问处理 / The model goes away: the
/// address stays mapped and a later access is an ordinary invalid access
inline void Unmap(uintptr_t base, const Device* device)
{
  for (auto& region : Detail::regions)
    if (region.size != 0U && region.base == base && region.device == device)
      region.device = nullptr;
}

/// 最近一次被拦截访问的指令地址 / Instruction address of the latest trapped access
inline uintptr_t LastPc() { return Detail::last_pc; }

/**
 * @brief 固定低地址的内存区，按顺序分配 / Memory at a fixed low address, handed out in
 *        order
 *
 * 控制器的 DMA 地址寄存器只有 32 位，后端把指针截成 32 位写进去。DMA 能到达的对象（端点
 * 槽、SETUP 缓冲、设备对象）都放在 4 GB 以下的这块内存里，寄存器里的值就是真实地址。
 * The DMA address registers of the controller are 32 bits wide and the backend writes
 * pointers truncated to 32 bits. Every object the DMA reaches (endpoint slots, SETUP
 * buffers, the device object) lives in this memory below 4 GB, so a register value is the
 * real address.
 */
class LowArena
{
 public:
  LowArena(uintptr_t base, size_t size) : base_(base), size_(size)
  {
    void* got = mmap(reinterpret_cast<void*>(base), size, PROT_READ | PROT_WRITE,
                     MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
    if (got != reinterpret_cast<void*>(base)) Detail::Fatal("arena address in use");
  }
  LowArena(const LowArena&) = delete;
  LowArena& operator=(const LowArena&) = delete;

  void* Alloc(size_t size, size_t align)
  {
    uintptr_t at = (base_ + used_ + align - 1U) & ~(static_cast<uintptr_t>(align) - 1U);
    if (at + size > base_ + size_) Detail::Fatal("arena exhausted");
    used_ = at + size - base_;
    return reinterpret_cast<void*>(at);
  }
  /// 之前分出去的内存全部作废 / Everything handed out so far is given up
  void Reset() { used_ = 0; }
  bool Contains(uintptr_t address, size_t size) const
  {
    return address >= base_ && size <= size_ && address - base_ <= size_ - size;
  }
  uintptr_t Base() const { return base_; }
  size_t Size() const { return size_; }

 private:
  uintptr_t base_;
  size_t size_;
  size_t used_ = 0;
};

}  // namespace MmioTrap
