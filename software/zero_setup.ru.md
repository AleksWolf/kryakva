# Настройка Radxa Zero 3W

Система — Armbian (ядро 6.1.115-vendor-rk35xx), загрузка с microSD. Пользователь в примерах — `duck`.

## Образ

`Armbian_26.8.1_Radxa-zero3_trixie_vendor_6.1.115_minimal.img.xz` с dl.armbian.com/radxa-zero3/Trixie_vendor_minimal.
Сверить с файлом `.sha` рядом с образом и записать на microSD программой Armbian Imager; в её профиле задать Wi-Fi и пользователя `duck`.
Wi-Fi — сразу сеть 5 ГГц (раздел 8).

После первой загрузки адрес платы — в списке устройств роутера. Вход с компьютера: `ssh duck@<адрес Zero>`.
Папки репозитория копируются на плату с компьютера через scp, например:

```
scp -r software/zero duck@<адрес Zero>:/home/duck/zero
```

Если файлы скопированы с Windows, у скриптов и программ пропадает право на запуск. Вернуть его после копирования папки:

```
chmod +x /home/duck/zero/*.sh /home/duck/zero/*/*.sh                         # software/zero
chmod +x /home/duck/duck-bench/bin/* /home/duck/duck-bench/standalone/*.sh   # software/robotd
```

## 1. Порты, overlay и служба усилителя

Папку [software/zero/](zero/) скопировать на плату в `/home/duck/zero` и запустить оттуда скрипт `setup_zero.sh`:

- ставит пакеты `i2c-tools alsa-utils v4l-utils wget gpiod gcc make device-tree-compiler dkms linux-headers-vendor-rk35xx`;
- собирает `tof_check` — проверку дальномера, её берёт страница утки;
- в `/boot/armbianEnv.txt` включает overlay `i2c3-m0` (IMU, пины 3/5), `uart2-m0` (шина серв, пины 8/10) и `radxa-zero3-rpi-camera-v2` (камера),
  убирает консоль ядра с порта серв (`console=display`, `serial-getty@ttyS2` выключен);
- собирает и включает свой overlay `duck-bench.dts`: I2S3 для усилителя и микрофона, I2C4 и I2C5 для дальномера;
- ставит службу `duck-amp`: держит вывод SD усилителя (пин 13) в нуле и поднимает его только на время воспроизведения;
- ставит службу отката: если за 5 минут после перезагрузки не выполнить
  ```
  sudo systemctl disable --now bench-rollback
  ```
  плата вернёт прежний `/boot/armbianEnv.txt` и перезагрузится.

После скрипта перезагрузить плату (`sudo reboot`) и в течение 5 минут после загрузки зайти по ssh и выполнить команду выше.

Проверка после перезагрузки: `i2cdetect -y 5` видит ToF (0x29), `i2cdetect -y 3` — IMU (0x6a), `aplay -l` — карту `duck-i2s3`,
`ls /dev/video0` — камеру.

**Калибровка ToF.** Без неё дальномер показывает всё, что ближе 60 см, ближе, чем есть. Сделать один раз, на собранной утке:

1. Поставить утку в 600 мм от светлой ровной стены, ToF смотрит прямо на неё. Других предметов в поле зрения датчика быть не должно.
2. Запустить калибровку, она сохранит `xtalk.bin` рядом с `tof_check`:
   ```
   cd /home/duck/zero && TOF_CAL="80 600" ./tof_check 5 20
   ```
3. Проверка: `TOF_XTALK=1 ./tof_check 5 20` показывает центр около 600 мм.

Страница утки загружает `xtalk.bin` сама.

Звуковой файл начинается с тишины не короче 50 мс и заканчивается нулями.

## 2. Геймпад DualSense

Настройки этого раздела — для геймпада Sony DualSense.

Драйвера `hid-playstation` в этом ядре нет. Он ставится DKMS-модулем из [software/zero/hid-playstation-dkms/](zero/hid-playstation-dkms/)
(исходники armbian/linux-rockchip той же версии ядра):

```
cd /home/duck/zero
sudo cp -r hid-playstation-dkms /usr/src/hid-playstation-6.1.115-rk
sudo dkms install hid-playstation/6.1.115-rk
```

Модуль грузится сам, когда подключается пульт (по USB или Bluetooth); перезагрузка не нужна.

Пакеты пульта должны идти через ядро, а не через `bluetoothd`: иначе `bluetoothd` постоянно грузит процессор и плата греется.
В `/etc/bluetooth/input.conf` поставить `UserspaceHID=false` и перезагрузить плату. Перезапуск bluetooth вместо перезагрузки не подходит:
адаптер зависает.

Пара по Bluetooth — скрипт `pair_ds.sh`:

1. Сбросить пульт скрепкой (отверстие сзади, ~5 с).
2. Включить режим пары: Create + PS до частого синего мигания.
3. MAC пульта: пока пульт мигает, на плате `bluetoothctl --timeout 30 scan on` — адрес в строке `DualSense Wireless Controller`.
4. На плате, в `/home/duck/zero`: `sudo ./pair_ds.sh <MAC пульта>` — скрипт включает `bondable`/`connectable`, ищет пульт, делает pair, trust, connect.

Дальше пульт подключается сам по кнопке PS. В системе — три устройства: геймпад, Motion Sensors, Touchpad.

## 3. Камера

Автоэкспозиция и баланс белого камеры — rkaiq Rockchip, собранный под это ядро ([software/zero/camera/](zero/camera/README.ru.md)):

```
sudo /home/duck/zero/camera/setup_camera.sh
```

Скрипт кладёт rkaiq в `/opt/rkaiq-v9`, скачивает файл настройки камеры Radxa в `/etc/iqfiles/` и включает службу `rkaiq`.
rkaiq подхватывает камеру при старте потока: если поток шёл до запуска rkaiq, его надо перезапустить.

## 4. Программа робота

Программа — robotd Pollen (`pollen-robotics/microduck`) с патчами под сервы Feetech HD-1910 и IMU на I²C.
Комплект — в [software/robotd/](robotd/README.ru.md).

1. Папку `software/robotd` скопировать на плату в `/home/duck/duck-bench`. В ней `bin/robotd`, `bin/robotctl`, `bin/padd`, `robotd.toml`,
   `policies/`, `hear/libonnxruntime.so`, `models/pickup_detector.onnx` и `standalone/`.
2. Поставить службы `robotd` и `padd`:
   ```
   sudo /home/duck/duck-bench/standalone/install.sh
   ```
   Скрипт ставит службы и переводит журнал systemd в ОЗУ. Ничего не включает и не запускает.
3. Проверить руками:
   ```
   sudo systemctl start robotd
   /home/duck/duck-bench/bin/robotctl --robot-socket /run/duck/robot.sock health --json
   sudo systemctl start padd
   ```
   При старте robotd снимает момент со всех 15 серв: утка мягкая.
4. Журнал такта robotd: пишется в ОЗУ, раз в 60 с и при выключении копируется на карту в `/var/lib/duck/ticks/`, не больше 2 ГБ:
   ```
   sudo /home/duck/zero/tick_journal/install.sh
   ```
5. Запуск при загрузке:
   ```
   sudo systemctl enable robotd padd
   ```

Пульт:

| Кнопка | Что делает |
|---|---|
| PS | стоп: момент снят со всех серв |
| Options | утка встаёт и ходит на месте; лёжа — сначала защитное падение и подъём |
| левый стик | вперёд и назад, вбок |
| правый стик по горизонтали | поворот |
| Create, держать 4 с | снять момент и выключить плату |

Если связь с пультом пропала, примерно через 2 с robotd сам снимает момент.

## 5. Страница утки

Страница состояния и управления: `http://<адрес Zero>:8082`. Работает поверх robotd, ставится после него —
[software/duck_page](duck_page/README.ru.md).

## 6. Пульт серв

Пульт серв можно запускать и на утке, по порту серв `/dev/ttyS2`, — [software/servo_console](servo_console/README.ru.md).

## 7. Карта microSD

Чтобы карта не портилась при резком отключении питания, журналы пишутся в ОЗУ:

- `/var/log` — в zram (так в Armbian по умолчанию, служба `armbian-ramlog`);
- `/tmp` — tmpfs;
- журнал systemd — в ОЗУ (его переводит `install.sh` robotd, раздел 4);
- журнал такта robotd — в ОЗУ, на карту копируется сжатым раз в 60 с и при выключении (раздел 4).

Проверка: `findmnt /var/log /tmp` — zram и tmpfs; `journalctl --header | grep "File path"` — путь в `/run/log/journal`.

## 8. Wi-Fi

Сеть для утки — на 5 ГГц: когда работает пульт, Wi-Fi 2.4 ГГц пропадает. Проще всего задать сеть 5 ГГц сразу в профиле Armbian Imager;
на готовой плате — в `/etc/netplan/*.yaml`.

## 9. Выключение

Сервы питаются от батареи и держат момент и после выключения Zero. Поэтому сначала снять момент с серв, потом `sudo poweroff`.
Момент снимать только у лежащей утки — стоящая упадёт.
