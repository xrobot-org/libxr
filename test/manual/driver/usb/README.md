# USB / DAP USB/IP 端到端手动测试 / Manual USB/IP End-to-End Tests

通过 Linux 内核 `vhci-hcd` 与 `usbip` 驱动将仿真 USB 设备和 CMSIS-DAP v1/v2 探针挂接到真实主机协议栈，端到端验证各设备类（CDC、HID、gs_usb、DFU、UAC）及真实调试器（OpenOCD、pyOCD、probe-rs）。这些程序默认 `EXCLUDE_FROM_ALL`，不参与常规 `LIBXR_TEST_BUILD` 构建与 CI。

Attaches the simulated USB device and CMSIS-DAP v1/v2 probe to the Linux kernel host stack via `vhci-hcd` and `usbip` to verify all device classes (CDC, HID, gs_usb, DFU, UAC) and real host debuggers (OpenOCD, pyOCD, probe-rs) end to end. The targets are `EXCLUDE_FROM_ALL` and omitted from default CI builds.

## 依赖 / Prerequisites

- Linux 主机，具备 `sudo`、`vhci-hcd` 内核模块与 `usbip` 工具。
  Linux host with `sudo`, `vhci-hcd` kernel module, and `usbip`.
- 类检查依赖：`dfu-util`、`python3-usb`、`arecord`。
  Class checks: `dfu-util`, `python3-usb`, `arecord`.
- DAP 检查依赖：`openocd`，以及可选 `pyocd`、`probe-rs`。
  DAP checks: `openocd`, and optional `pyocd`, `probe-rs`.

## 构建与运行 / Build and Run

在仓库根目录配置并显式构建三个手动目标，再从本目录或 [../debug/](../debug/) 运行脚本：

Configure at the repository root, explicitly build the three manual targets, and run the scripts from this directory or [../debug/](../debug/):

```sh
cmake -S . -B build -DLIBXR_TEST_BUILD=ON -DLIBXR_DEV_ASSERT_BUILD=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel 8 --target usb_sim_usbip usb_sim_classes dap_sim_usbip

bash test/manual/driver/usb/classes_e2e.sh build
bash test/manual/driver/usb/plug_cycle.sh build 20
bash test/manual/driver/debug/dap_e2e.sh build [pyocd_path] [probe_rs_path]
```
