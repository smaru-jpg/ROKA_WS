#!/usr/bin/env bash
# Resolve and install real-robot apt dependencies in small batches.

set -Eeuo pipefail

script_directory="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd "$script_directory/.." && pwd)"
batch_size="${ROKA_APT_BATCH_SIZE:-6}"
rosdep_update_timeout="${ROKA_ROSDEP_UPDATE_TIMEOUT_SEC:-180}"

[[ "$batch_size" =~ ^[1-9][0-9]*$ ]] || {
  printf '[ROKA-DEPS][FAIL] ROKA_APT_BATCH_SIZE must be a positive integer\n' >&2
  exit 2
}
[[ "$rosdep_update_timeout" =~ ^[1-9][0-9]*$ ]] || {
  printf '[ROKA-DEPS][FAIL] ROKA_ROSDEP_UPDATE_TIMEOUT_SEC must be a positive integer\n' >&2
  exit 2
}
[[ -r /opt/ros/humble/setup.bash ]] || {
  printf '[ROKA-DEPS][FAIL] Install ROS 2 Humble first.\n' >&2
  exit 1
}
[[ -r /etc/os-release ]] || {
  printf '[ROKA-DEPS][FAIL] /etc/os-release is missing\n' >&2
  exit 1
}
# shellcheck disable=SC1091
source /etc/os-release
[[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "22.04" ]] || {
  printf '[ROKA-DEPS][FAIL] Ubuntu 22.04 required; found %s %s\n' \
    "${ID:-unknown}" "${VERSION_ID:-unknown}" >&2
  exit 1
}

# Ignore user/global pip copies when selecting the Ubuntu/ROS build tools.
# This prevents a pre-existing Flake8/colcon/setuptools stack from changing the
# result on another laptop.
export PYTHONNOUSERSITE=1
export PATH="/usr/sbin:/usr/bin:/sbin:/bin:/usr/local/sbin:/usr/local/bin:$HOME/.local/bin"

# A desktop ROS installation does not guarantee that its build/dependency
# utilities or pip are present. Bootstrap only the missing Ubuntu tools before
# asking rosdep for the much larger runtime dependency set.
bootstrap_packages=()
[[ -x /usr/bin/rosdep ]] || bootstrap_packages+=(python3-rosdep)
[[ -x /usr/bin/colcon ]] || bootstrap_packages+=(python3-colcon-common-extensions)
/usr/bin/python3 -m pip --version >/dev/null 2>&1 || bootstrap_packages+=(python3-pip)
apt_index_updated=false
if ((${#bootstrap_packages[@]})); then
  printf '[ROKA-DEPS] Installing prerequisite tools: %s\n' "${bootstrap_packages[*]}"
  sudo -v
  sudo apt-get update
  apt_index_updated=true
  sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y \
    --no-install-recommends "${bootstrap_packages[@]}"
fi
[[ -x /usr/bin/rosdep ]] && [[ -x /usr/bin/colcon ]] && \
  /usr/bin/python3 -m pip --version >/dev/null 2>&1 || {
    printf '[ROKA-DEPS][FAIL] rosdep, colcon or pip bootstrap failed.\n' >&2
    exit 1
  }

# Generated caches, simulator-only packages and archived reference packages are
# deliberately absent from the real-robot dependency transaction.
source_roots=(
  "$workspace_root/src/common"
  "$workspace_root/src/common_msgs"
  "$workspace_root/src/drivers"
  "$workspace_root/src/localization"
  "$workspace_root/src/manipulation"
  "$workspace_root/src/mission"
  "$workspace_root/src/mission_vision"
  "$workspace_root/src/navigation"
  "$workspace_root/src/terrain"
  "$workspace_root/src/vendor/fast_lio"
  "$workspace_root/src/vendor/lidar_imu_init"
  "$workspace_root/src/vendor/sc_pgo"
  "$workspace_root/src/vendor/velodyne"
)

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION PYTHONPATH
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

if [[ ! -r /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
# This handoff targets Humble only. Avoid fetching every currently supported
# distribution, which can make a fresh low-spec PC appear to hang on unrelated
# rosdistro endpoints.
rosdep_cache="${ROS_HOME:-$HOME/.ros}/rosdep/sources.cache/index"
if ! timeout "$rosdep_update_timeout" rosdep update --rosdistro humble; then
  if [[ -s "$rosdep_cache" ]]; then
    printf '%s\n' \
      '[ROKA-DEPS][WARN] rosdep update failed or timed out; using the existing Humble cache.' \
      'Check internet/DNS later and rerun this installer to refresh it.' >&2
  else
    printf '%s\n' \
      '[ROKA-DEPS][FAIL] rosdep could not download the Humble dependency index.' \
      'Check internet/DNS, then rerun this installer. No usable cache exists.' >&2
    exit 1
  fi
fi

mapfile -t rosdep_keys < <(
  rosdep keys --from-paths "${source_roots[@]}" --ignore-src \
    --rosdistro humble | sort -u
)
apt_packages=()
unsupported=()
for key in "${rosdep_keys[@]}"; do
  # Ubuntu Jammy's rosdep database does not expose the Python build backend as
  # an installable system key; colcon proves it when those packages build.
  [[ "$key" == "ament_python" ]] && continue
  resolution="$(rosdep resolve --rosdistro humble "$key" 2>/dev/null || true)"
  installer="$(sed -n 's/^#//p' <<<"$resolution" | head -n 1)"
  if [[ "$installer" != "apt" ]]; then
    unsupported+=("$key:$installer")
    continue
  fi
  while IFS= read -r package; do
    [[ -n "$package" && "$package" != \#* ]] && apt_packages+=("$package")
  done <<<"$resolution"
done
mapfile -t apt_packages < <(printf '%s\n' "${apt_packages[@]}" | sort -u)

if ((${#unsupported[@]})); then
  printf '[ROKA-DEPS][FAIL] Non-apt rosdep rules require manual review:\n' >&2
  printf '  %s\n' "${unsupported[@]}" >&2
  exit 1
fi

printf '[ROKA-DEPS] %d apt packages; installing in batches of %d.\n' \
  "${#apt_packages[@]}" "$batch_size"
sudo -v
if [[ "$apt_index_updated" != true ]]; then
  sudo apt-get update
fi
for ((offset=0; offset<${#apt_packages[@]}; offset+=batch_size)); do
  batch=("${apt_packages[@]:offset:batch_size}")
  end=$((offset + ${#batch[@]}))
  printf '[ROKA-DEPS] apt %d/%d: %s\n' "$end" "${#apt_packages[@]}" "${batch[*]}"
  sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y \
    --no-install-recommends "${batch[@]}"
done
sudo apt-get clean

rosdep check --from-paths "${source_roots[@]}" --ignore-src \
  --skip-keys=ament_python --rosdistro humble
printf '[ROKA-DEPS][PASS] Real-robot system dependencies are complete.\n'
