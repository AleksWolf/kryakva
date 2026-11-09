#!/bin/sh
# robotd tick journal: always written to RAM (/tmp/duck), copied to SD /var/lib/duck/ticks every 60 s and at shutdown.
set -e
cd "$(dirname "$0")"
sudo install -m 0755 duck-tickflush.py /usr/local/bin/duck-tickflush.py
sudo cp duck-tickflush.service duck-tickflush.timer duck-tickflush-final.service /etc/systemd/system/
sudo cp duck-tmp.conf /etc/tmpfiles.d/duck-tmp.conf
sudo systemd-tmpfiles --create /etc/tmpfiles.d/duck-tmp.conf
sudo install -D -m 0644 robotd-journal.conf /etc/systemd/system/robotd.service.d/journal.conf
sudo systemctl daemon-reload
sudo systemctl enable --now duck-tickflush.timer duck-tickflush-final.service
