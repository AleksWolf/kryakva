# rkaiq for the IMX219 camera

`rkaiq_3A_server` and `librkaiq.so` are built from Rockchip camera_engine_rkaiq v6.0x9.0
(https://github.com/TaishanPi-Rockchip/linux.external.camera_engine_rkaiq, commit 216ab11, Apache-2.0), the version that matches
rkisp 2.9.0 of the Armbian 6.1.115-vendor-rk35xx kernel.

Change: `rkaiq-sched-other.patch`. The statistics threads are started with SCHED_OTHER instead of SCHED_RR: the kernel has
CONFIG_RT_GROUP_SCHED=y and gives no real-time priority to services, so rkaiq stops with `isp_3a_stats_poll create fail`.
The patch also turns off -Werror and the demo and tool sub-projects.

Build: cross-compiler aarch64-linux-gnu, CMake + Ninja, `ARCH=aarch64`, `RKAIQ_TARGET_SOC=rk356x`, `ISP_HW_VERSION=-DISP_HW_V21`,
`CMAKE_BUILD_TYPE=MinSizeRel`.

| File | SHA-256 |
|---|---|
| `librkaiq.so` | bf9fbd0629cc6c5dbfce79b6ae917f412b9b159af5a1d5600eb66794bbeb4511 |
| `rkaiq_3A_server` | 4f73bbe2f80270472887cc07c4b96ac67951c058220433b5f151ba3fc2b138a8 |
