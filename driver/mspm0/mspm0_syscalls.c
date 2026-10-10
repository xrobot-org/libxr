// GCC 链接 MSPM0 SDK 工程时使用 -nostartfiles 和 --specs=nosys.specs（与 SDK 例程的
// gcc makefile 相同），C++ 运行时需要的以下符号因此没有定义。这里补齐它们；弱定义允许
// 工程自行提供。TI Arm Clang 使用自己的启动代码和运行库，不编译这些定义。
// GCC links MSPM0 SDK projects with -nostartfiles and --specs=nosys.specs (as the SDK
// example gcc makefiles do), which leaves the following symbols of the C++ runtime
// undefined. They are provided here as weak definitions a project can replace. TI Arm
// Clang has its own startup code and runtime library and does not compile them.
#if defined(__GNUC__) && !defined(__clang__)

#include <errno.h>

// 编译器登记静态存储期对象的析构函数时把 __dso_handle 传给 __cxa_atexit；它通常由
// crtbegin.o 定义，-nostartfiles 不链接该文件。main() 不返回，析构函数不会执行。
// The compiler passes __dso_handle to __cxa_atexit when it registers the destructor of
// an object with static storage. crtbegin.o defines it, and -nostartfiles drops that
// file. main() does not return, so the destructors never run.
__attribute__((weak)) void* __dso_handle = 0;

// abort()（std::terminate、纯虚函数调用）会调用 _getpid 和 _kill。libnosys 中的版本
// 行为相同，但会让链接器输出 "is not implemented" 警告。
// abort() (std::terminate, pure virtual calls) reaches _getpid and _kill. The libnosys
// versions behave the same, but make the linker print "is not implemented" warnings.
__attribute__((weak)) int _getpid(void) { return 1; }

__attribute__((weak)) int _kill(int process_id, int signal)
{
  (void)process_id;
  (void)signal;
  errno = EINVAL;
  return -1;
}

#endif
