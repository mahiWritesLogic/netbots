#!/usr/bin/env bash
# Reproducible setup on Ubuntu 24.04 (native or WSL2). Re-runnable.
#   NS3_DIR   where ns-3 lives          (default ~/ns-3.48)
#   VENV      Python virtual env path   (default ~/.venvs/netbots)
set -euo pipefail
NS3_VERSION=3.48
NS3_DIR=${NS3_DIR:-$HOME/ns-$NS3_VERSION}
VENV=${VENV:-$HOME/.venvs/netbots}
HERE=$(cd "$(dirname "$0")" && pwd)

need=()
for c in g++ cmake; do command -v $c >/dev/null || need+=($c); done
if ((${#need[@]})); then
  echo "Missing ${need[*]}. Run: sudo apt-get install -y g++ cmake ninja-build python3-venv" >&2
  exit 1
fi

# ns-3 (source archive of the official release tag)
if [ ! -x "$NS3_DIR/ns3" ]; then
  curl -sSL --retry 5 -o /tmp/ns-$NS3_VERSION.tar.gz \
    https://gitlab.com/nsnam/ns-3-dev/-/archive/ns-$NS3_VERSION/ns-3-dev-ns-$NS3_VERSION.tar.gz
  tar xzf /tmp/ns-$NS3_VERSION.tar.gz -C "$(dirname "$NS3_DIR")"
  mv "$(dirname "$NS3_DIR")/ns-3-dev-ns-$NS3_VERSION" "$NS3_DIR"
fi
cp "$HERE/network/netbots.cc" "$NS3_DIR/scratch/netbots.cc"
cd "$NS3_DIR"
[ -f cmake-cache/CMakeCache.txt ] || ./ns3 configure --build-profile=optimized \
  --enable-modules "core;network;internet;applications" --disable-examples --disable-tests
./ns3 build scratch/netbots

# Python env (pip bootstrapped without needing python3-pip)
if [ ! -x "$VENV/bin/pip" ]; then
  python3 -m venv "$VENV" 2>/dev/null || {
    python3 -m venv --without-pip "$VENV"
    curl -sSL --retry 5 https://bootstrap.pypa.io/get-pip.py | "$VENV/bin/python" -
  }
fi
"$VENV/bin/pip" install -q -r "$HERE/requirements.txt"
echo "OK: ns-3 at $NS3_DIR, venv at $VENV"
