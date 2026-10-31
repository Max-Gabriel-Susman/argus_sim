#!/usr/bin/env bash
# install_bindings.sh -- build the argus_sim_py wheel and install it for
# /usr/bin/python3.
#
#   tools/install_bindings.sh
#
# Builds crates/argus-sim-py in release with maturin (one abi3 wheel for
# Python 3.10 and later), installs it into the user site of /usr/bin/python3,
# the interpreter the launch file and the tools run under, then imports it
# and prints its version and where it was loaded from. Run it again after
# changing the bindings: the install replaces the previous one.
#
# Needs maturin 1.16 on PATH and the Rust toolchain from rust-toolchain.toml.
# Installs no dependencies; the module needs none.
set -euo pipefail

PYTHON=/usr/bin/python3

# Everything relative to the repository, wherever it is checked out.
cd "$(dirname "${BASH_SOURCE[0]}")/.."

command -v maturin >/dev/null || {
  echo "install_bindings: maturin not found; install maturin 1.16 first" >&2
  exit 1
}

out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT

# --compatibility linux: a wheel for this machine, not a manylinux one, since
# it is installed here and never published.
maturin build --release -m crates/argus-sim-py/Cargo.toml --out "$out" \
  --compatibility linux --interpreter "$PYTHON"

wheels=("$out"/argus_sim_py-*.whl)
if [ "${#wheels[@]}" -ne 1 ] || [ ! -f "${wheels[0]}" ]; then
  echo "install_bindings: expected one wheel in $out, found: ${wheels[*]}" >&2
  exit 1
fi

# --no-deps: the wheel has none, and pip must not touch anything else.
# --force-reinstall: the version stays 2.0.0 across rebuilds, so pip would
# otherwise keep the old build.
"$PYTHON" -m pip install --user --force-reinstall --no-deps "${wheels[0]}"

# Import from outside the repository (the build's scratch directory), so it
# is the installed module that loads and not anything in the checkout.
cd "$out"
"$PYTHON" -c 'import argus_sim_py; print("argus_sim_py", argus_sim_py.__version__, argus_sim_py.__file__)'
