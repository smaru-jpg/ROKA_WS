#!/usr/bin/env bash
# Install an extracted ROKA handoff without inheriting another ROS overlay.

set -Eeuo pipefail

script_directory="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd "$script_directory/.." && pwd)"
command_name="roka"
domain_id="31"
camera_serial=""
led_serial=""
domain_id_explicit=false
camera_serial_explicit=false
led_serial_explicit=false
install_arm_ml=true
build_workspace=true
run_verification=true
force_link=false
build_simulation=false

usage() {
  cat <<'EOF'
Usage: ./tools/install_handoff.sh [options]

Options:
  --command-name NAME   Installed command name (default: roka)
  --domain-id ID       ROS 2 domain for this robot/team, 0..232 (default: 31)
  --camera-serial ID   D435 serial; omit when exactly one D435 is connected
  --led-serial PATH    Nano stable /dev/serial/by-id path; omit to keep LED off
  --without-arm        Do not install the isolated YOLO/PyTorch runtime
                       (full verification is skipped if ML is unavailable)
  --with-simulation    Also build the optional MuJoCo packages
  --no-build           Register only; skip colcon build
  --no-verify          Skip the software regression suite
  --force-link         Replace an existing command symlink
  -h, --help           Show this help

Builds are deliberately sequential (one compiler job) for low-memory PCs.
This script never installs apt packages and never uses sudo. Run
install_system_dependencies_low_memory.sh first, then run this as the operator.
EOF
}

while (($#)); do
  case "$1" in
    --command-name)
      (($# >= 2)) || { printf 'missing value for %s\n' "$1" >&2; exit 2; }
      command_name="$2"; shift 2 ;;
    --domain-id)
      (($# >= 2)) || { printf 'missing value for %s\n' "$1" >&2; exit 2; }
      domain_id="$2"; domain_id_explicit=true; shift 2 ;;
    --camera-serial)
      (($# >= 2)) || { printf 'missing value for %s\n' "$1" >&2; exit 2; }
      camera_serial="$2"; camera_serial_explicit=true; shift 2 ;;
    --led-serial)
      (($# >= 2)) || { printf 'missing value for %s\n' "$1" >&2; exit 2; }
      led_serial="$2"; led_serial_explicit=true; shift 2 ;;
    --without-arm) install_arm_ml=false; shift ;;
    --with-simulation) build_simulation=true; shift ;;
    --no-build) build_workspace=false; shift ;;
    --no-verify) run_verification=false; shift ;;
    --force-link) force_link=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$command_name" =~ ^[a-zA-Z0-9_-]+$ ]] || {
  printf '[ROKA-INSTALL][FAIL] invalid command name: %s\n' "$command_name" >&2
  exit 2
}

config_directory="${XDG_CONFIG_HOME:-$HOME/.config}/roka"
config_file="$config_directory/${command_name}.env"
if [[ -r "$config_file" ]]; then
  while IFS= read -r config_line || [[ -n "$config_line" ]]; do
    [[ -z "$config_line" || "$config_line" == \#* ]] && continue
    config_key="${config_line%%=*}"
    config_value="${config_line#*=}"
    case "$config_key" in
      ROKA_ROS_DOMAIN_ID)
        [[ "$domain_id_explicit" == true ]] || domain_id="$config_value" ;;
      ROKA_ARM_CAMERA_SERIAL)
        [[ "$camera_serial_explicit" == true ]] || camera_serial="$config_value" ;;
      ROKA_LED_SERIAL)
        [[ "$led_serial_explicit" == true ]] || led_serial="$config_value" ;;
      *)
        printf '[ROKA-INSTALL][FAIL] unsupported setting in %s: %s\n' \
          "$config_file" "$config_key" >&2
        exit 2 ;;
    esac
  done <"$config_file"
fi
[[ "$domain_id" =~ ^[0-9]+$ ]] && ((domain_id >= 0 && domain_id <= 232)) || {
  printf '[ROKA-INSTALL][FAIL] ROS domain must be 0..232: %s\n' "$domain_id" >&2
  exit 2
}
[[ -z "$camera_serial" || "$camera_serial" =~ ^[A-Za-z0-9_-]+$ ]] || {
  printf '[ROKA-INSTALL][FAIL] invalid camera serial: %s\n' "$camera_serial" >&2
  exit 2
}
[[ -z "$led_serial" || "$led_serial" == /dev/serial/by-id/* ]] || {
  printf '[ROKA-INSTALL][FAIL] LED path must use /dev/serial/by-id: %s\n' \
    "$led_serial" >&2
  exit 2
}

[[ -r /etc/os-release ]] || {
  printf '[ROKA-INSTALL][FAIL] /etc/os-release is missing\n' >&2; exit 1;
}
# shellcheck disable=SC1091
source /etc/os-release
[[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "22.04" ]] || {
  printf '[ROKA-INSTALL][FAIL] Ubuntu 22.04 required; found %s %s\n' \
    "${ID:-unknown}" "${VERSION_ID:-unknown}" >&2
  exit 1
}
[[ -r /opt/ros/humble/setup.bash ]] || {
  printf '[ROKA-INSTALL][FAIL] ROS 2 Humble is not installed\n' >&2; exit 1;
}
[[ -x "$workspace_root/roka" && -d "$workspace_root/src" ]] || {
  printf '[ROKA-INSTALL][FAIL] incomplete handoff at %s\n' "$workspace_root" >&2
  exit 1
}

# Do not let a sourced student/team workspace affect rosdep or colcon package
# selection. Keep non-ROS system paths (including NVIDIA libraries) intact.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION PYTHONPATH
unset ROKA_WS_ROOT ROS_DOMAIN_ID
export PYTHONNOUSERSITE=1
export PATH="/usr/sbin:/usr/bin:/sbin:/bin:/usr/local/sbin:/usr/local/bin:$HOME/.local/bin"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$workspace_root/env/velodyne_build_env.sh"
set -u

printf '[ROKA-INSTALL] workspace=%s command=%s domain=%s\n' \
  "$workspace_root" "$command_name" "$domain_id"

memory_mib="$(awk '/MemTotal:/ {print int($2 / 1024)}' /proc/meminfo)"
swap_mib="$(awk '/SwapTotal:/ {print int($2 / 1024)}' /proc/meminfo)"
available_gib="$(df -Pk "$workspace_root" | awk 'NR==2 {print int($4 / 1048576)}')"
printf '[ROKA-INSTALL] memory=%sMiB swap=%sMiB free-disk=%sGiB build-jobs=1\n' \
  "$memory_mib" "$swap_mib" "$available_gib"
if ((memory_mib + swap_mib < 7500)); then
  printf '%s\n' \
    '[ROKA-INSTALL][WARN] RAM+swap is below 7.5 GiB.' \
    'Add swap using the handoff guide before compiling C++ or installing PyTorch.' >&2
fi
if ((available_gib < 20)); then
  printf '[ROKA-INSTALL][FAIL] At least 20 GiB free disk is required.\n' >&2
  exit 1
fi

real_source_roots=(
  "$workspace_root/src/common" "$workspace_root/src/common_msgs"
  "$workspace_root/src/drivers" "$workspace_root/src/localization"
  "$workspace_root/src/manipulation" "$workspace_root/src/mission"
  "$workspace_root/src/mission_vision" "$workspace_root/src/navigation"
  "$workspace_root/src/terrain" "$workspace_root/src/vendor/fast_lio"
  "$workspace_root/src/vendor/lidar_imu_init" "$workspace_root/src/vendor/sc_pgo"
  "$workspace_root/src/vendor/velodyne"
)
if ! rosdep check --from-paths "${real_source_roots[@]}" --ignore-src \
  --skip-keys=ament_python --rosdistro humble \
  >/tmp/roka_rosdep_check.log 2>&1; then
  cat /tmp/roka_rosdep_check.log >&2
  printf '%s\n' \
    '[ROKA-INSTALL][FAIL] system dependencies are missing.' \
    'Run ./tools/install_system_dependencies_low_memory.sh, then retry.' >&2
  exit 1
fi

python_runtime="$workspace_root/.roka_python"
if [[ "$build_workspace" == true ]]; then
  cd "$workspace_root"
  export CMAKE_BUILD_PARALLEL_LEVEL=1
  export MAKEFLAGS=-j1
  build_stage() {
    local label="$1"
    shift
    printf '[ROKA-INSTALL][BUILD] %s: %s\n' "$label" "$*"
    colcon build --executor sequential --symlink-install \
      --packages-select "$@" --cmake-args -DCMAKE_BUILD_TYPE=Release
    # shellcheck disable=SC1091
    set +u
    source "$workspace_root/install/setup.bash"
    set -u
  }
  build_stage '1/7 interfaces/common' \
    roka_interfaces roka_robot_description roka_nav_utils
  build_stage '2/7 VLP-16 driver' \
    velodyne_msgs velodyne_driver velodyne_laserscan velodyne_pointcloud velodyne
  build_stage '3/7 motor/IMU drivers' roka_md_driver roka_imu_driver
  build_stage '4/7 vision/arm' mission_vision roka_arm
  build_stage '5/7 localization' \
    fast_lio sc_pgo lidar_imu_init roka_map_localization roka_fast_lio_localization
  build_stage '6/7 terrain/navigation' \
    roka_terrain_mapping roka_terrain_supervisor roka_nav2_bringup
  build_stage '7/7 mission integration' roka_mission
  if [[ "$build_simulation" == true ]]; then
    build_stage 'optional simulation' mujoco_ros2_control roka_mujoco
  fi
fi

# Install large ML wheels only after C++ compilation has released its memory.
# Resolve the pinned stack once: installing every direct requirement separately
# downloads Torch/CUDA repeatedly and lets unrelated ~/.local packages affect
# resolution. Build in staging and only replace a previous runtime after import
# and version validation succeeds.
validate_ml_runtime() {
  local runtime="$1"
  [[ -d "$runtime" ]] || return 1
  PYTHONNOUSERSITE=1 PYTHONPATH="$runtime" /usr/bin/python3 - <<'PY'
from importlib.metadata import version

expected = {
    "setuptools": "79.0.1",
    "numpy": "1.26.4",
    "opencv-python": "4.11.0.86",
    "torch": "2.13.0",
    "torchvision": "0.28.0",
    "ultralytics": "8.4.131",
}
for package, wanted in expected.items():
    actual = version(package)
    if actual != wanted:
        raise RuntimeError(f"{package}: expected {wanted}, found {actual}")

import cv2  # noqa: F401,E402
import numpy  # noqa: F401,E402
import torch  # noqa: F401,E402
import torchvision  # noqa: F401,E402
import ultralytics  # noqa: F401,E402
PY
}

if [[ "$install_arm_ml" == true ]]; then
  export PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1
  ml_requirements="$workspace_root/src/manipulation/roka_arm/requirements-ml.txt"
  if validate_ml_runtime "$python_runtime" >/dev/null 2>&1; then
    printf '[ROKA-INSTALL][ML] Reusing the validated isolated ML runtime.\n'
  else
    python_staging="${python_runtime}.staging"
    rm -rf -- "$python_staging"
    mkdir -p "$python_staging"
    printf '[ROKA-INSTALL][ML] Installing the constrained ML stack once.\n'
    PYTHONNOUSERSITE=1 /usr/bin/python3 -m pip install \
      --ignore-installed --no-cache-dir \
      --constraint "$ml_requirements" --requirement "$ml_requirements" \
      --target "$python_staging"
    validate_ml_runtime "$python_staging" || {
      printf '[ROKA-INSTALL][FAIL] Isolated ML runtime validation failed.\n' >&2
      exit 1
    }
    rm -rf -- "$python_runtime"
    mv "$python_staging" "$python_runtime"
  fi
fi

[[ -r "$workspace_root/install/setup.bash" ]] || {
  printf '[ROKA-INSTALL][FAIL] install/setup.bash is missing after build\n' >&2
  exit 1
}

mkdir -p "$config_directory" "$HOME/.local/bin"
cat >"$config_file" <<EOF
# Generated by $workspace_root/tools/install_handoff.sh
ROKA_ROS_DOMAIN_ID=$domain_id
ROKA_ARM_CAMERA_SERIAL=$camera_serial
EOF
if [[ -n "$led_serial" ]]; then
  printf 'ROKA_LED_SERIAL=%s\n' "$led_serial" >>"$config_file"
fi
chmod 600 "$config_file"

command_link="$HOME/.local/bin/$command_name"
if [[ -e "$command_link" || -L "$command_link" ]]; then
  current_target="$(readlink -f "$command_link" 2>/dev/null || true)"
  if [[ "$current_target" != "$workspace_root/roka" && "$force_link" != true ]]; then
    printf '[ROKA-INSTALL][FAIL] %s already points to %s\n' \
      "$command_link" "${current_target:-a non-symlink file}" >&2
    printf 'Use another --command-name or pass --force-link intentionally.\n' >&2
    exit 1
  fi
fi
ln -sfn "$workspace_root/roka" "$command_link"

completion_line="source \"$workspace_root/tools/roka_completion.bash\"; complete -F _roka_complete $command_name"
aliases_file="$HOME/.bash_aliases"
touch "$aliases_file"
if ! grep -Fqx "$completion_line" "$aliases_file"; then
  # Replace a stale path for this command while leaving unrelated aliases and
  # completion registrations untouched.
  sed -i "\|/tools/roka_completion\\.bash.*complete -F _roka_complete $command_name$|d" \
    "$aliases_file"
  printf '%s\n' "$completion_line" >>"$aliases_file"
fi

# Keep shell setup usable on both stock Ubuntu and minimal Bash profiles.
# Every line is guarded so rerunning the installer does not duplicate config.
bashrc_file="$HOME/.bashrc"
touch "$bashrc_file"
ros_setup_line='[[ -r /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash'
local_bin_line='case ":$PATH:" in *":$HOME/.local/bin:"*) ;; *) export PATH="$HOME/.local/bin:$PATH";; esac'
aliases_source_line='[[ -r "$HOME/.bash_aliases" ]] && source "$HOME/.bash_aliases"'
if ! grep -Fqx "$ros_setup_line" "$bashrc_file" && \
   ! grep -Eq '^[[:space:]]*(source|\.)[[:space:]]+/opt/ros/humble/setup\.bash' "$bashrc_file"; then
  printf '%s\n' "$ros_setup_line" >>"$bashrc_file"
fi
if ! grep -Fqx "$local_bin_line" "$bashrc_file" && \
   ! grep -Eq '^[^#]*\$HOME/\.local/bin' "$bashrc_file"; then
  printf '%s\n' "$local_bin_line" >>"$bashrc_file"
fi
if ! grep -Eq '^[[:space:]]*(source|\.)[[:space:]].*\.bash_aliases' "$bashrc_file" && \
   ! grep -Fq 'source "$HOME/.bash_aliases"' "$bashrc_file"; then
  printf '%s\n' "$aliases_source_line" >>"$bashrc_file"
fi

export ROKA_WS_ROOT="$workspace_root"
export ROS_DOMAIN_ID="$domain_id"
set +u
# shellcheck disable=SC1091
source "$workspace_root/install/setup.bash"
set -u
if [[ -d "$python_runtime" ]]; then
  export PYTHONPATH="$python_runtime${PYTHONPATH:+:$PYTHONPATH}"
fi
overlay_check_args=(--workspace "$workspace_root")
if [[ "$build_simulation" == true ]]; then
  overlay_check_args+=(--include-simulation)
fi
python3 "$workspace_root/tools/check_overlay.py" "${overlay_check_args[@]}"

if [[ "$run_verification" == true ]]; then
  if [[ "$install_arm_ml" != true ]] && \
     ! /usr/bin/python3 -c 'import torch, ultralytics' >/dev/null 2>&1; then
    printf '%s\n' \
      '[ROKA-INSTALL][WARN] Full regression verification was skipped because' \
      '--without-arm was selected and no existing YOLO/PyTorch runtime is available.' \
      'Install the ML requirements, then run the installed command with: verify' >&2
  else
    ROKA_CONFIG_FILE="$config_file" "$workspace_root/roka" verify
  fi
fi

printf '%s\n' \
  "[ROKA-INSTALL][PASS] restart the terminal, then run: $command_name help" \
  "[ROKA-INSTALL] per-PC config: $config_file" \
  "[ROKA-INSTALL] other robots on the same network must use a different --domain-id"
