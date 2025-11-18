#!/usr/bin/env bash


set -e
apt-get update && apt-get install -y python3-venv python3-full >/dev/null || true
python3 -m venv .venv --system-site-packages
.venv/bin/pip install -U pip wheel
if [ -f requirements.txt ]; then .venv/bin/pip install -U -r requirements.txt; fi
if [ -f requirements.user.txt ]; then .venv/bin/pip install -U -r requirements.user.txt; fi
