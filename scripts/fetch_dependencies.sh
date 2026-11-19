#!/usr/bin/env bash
set -e
flapping_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
flapping_driver="$flapping_root/src/nlink_parser2"
flapping_revision=efbd56437c524d95487ed866df1ce93099880eb8
if [[ ! -d "$flapping_driver" ]]; then
  git -c credential.helper= clone --no-checkout \
    https://github.com/nooploop-dev/nlink_parser2.git "$flapping_driver"
fi
if [[ ! -d "$flapping_driver/.git" ]]; then
  echo "$flapping_driver is not the expected standalone Git checkout" >&2
  exit 1
fi
if [[ -n "$(git -C "$flapping_driver" status --porcelain --untracked-files=no)" ]]; then
  # A freshly cloned --no-checkout repository has no worktree yet.
  if [[ -f "$flapping_driver/CMakeLists.txt" ]]; then
    echo "Official driver contains changes; refusing to replace them" >&2
    exit 1
  fi
fi
git -C "$flapping_driver" checkout --detach "$flapping_revision"
git -C "$flapping_driver" -c credential.helper= submodule update --init --recursive
git -C "$flapping_driver" rev-parse HEAD
git -C "$flapping_driver" submodule status --recursive

flapping_mavlink="$flapping_root/third_party/mavlink_c"
flapping_mavlink_revision=085c8344fa721a3d80bf574ae77c4d873f4985f3
if [[ ! -d "$flapping_mavlink" ]]; then
  mkdir -p "$flapping_root/third_party"
  git -c credential.helper= clone --depth 1 --no-checkout \
    https://github.com/mavlink/c_library_v2.git "$flapping_mavlink"
fi
if [[ ! -d "$flapping_mavlink/.git" ]]; then
  echo "$flapping_mavlink is not the expected standalone Git checkout" >&2
  exit 1
fi
if [[ -f "$flapping_mavlink/common/mavlink.h" &&
  -n "$(git -C "$flapping_mavlink" status --porcelain)" ]]; then
  echo 'MAVLink dependency contains changes; refusing to replace them' >&2
  exit 1
fi
if ! git -C "$flapping_mavlink" cat-file -e "$flapping_mavlink_revision^{commit}" 2>/dev/null; then
  git -C "$flapping_mavlink" -c credential.helper= fetch --depth 1 origin "$flapping_mavlink_revision"
fi
git -C "$flapping_mavlink" checkout --detach "$flapping_mavlink_revision"
