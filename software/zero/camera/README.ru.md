# rkaiq для камеры IMX219

`rkaiq_3A_server` и `librkaiq.so` собраны из camera_engine_rkaiq Rockchip v6.0x9.0
(https://github.com/TaishanPi-Rockchip/linux.external.camera_engine_rkaiq, коммит 216ab11, Apache-2.0) — версия под rkisp 2.9.0
ядра Armbian 6.1.115-vendor-rk35xx.

Изменение — `rkaiq-sched-other.patch`. Потоки статистики запускаются с SCHED_OTHER вместо SCHED_RR: в ядре включён
CONFIG_RT_GROUP_SCHED, службам приоритет реального времени не дают, и rkaiq останавливается с ошибкой `isp_3a_stats_poll create fail`.
Патч также отключает -Werror и демо- и служебные подпроекты.

Сборка: кросс-компилятор aarch64-linux-gnu, CMake + Ninja, `ARCH=aarch64`, `RKAIQ_TARGET_SOC=rk356x`, `ISP_HW_VERSION=-DISP_HW_V21`,
`CMAKE_BUILD_TYPE=MinSizeRel`.

| Файл | SHA-256 |
|---|---|
| `librkaiq.so` | bf9fbd0629cc6c5dbfce79b6ae917f412b9b159af5a1d5600eb66794bbeb4511 |
| `rkaiq_3A_server` | 4f73bbe2f80270472887cc07c4b96ac67951c058220433b5f151ba3fc2b138a8 |
