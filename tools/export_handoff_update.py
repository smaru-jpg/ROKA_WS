#!/usr/bin/env python3
"""Build a verified incremental update for an already-installed ROKA handoff."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from hashlib import sha256
import os
from pathlib import Path
import shutil
import stat
import sys
import tempfile
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parent))
import export_handoff


ARCHIVE_ROOT = "roka_ws_update"


def digest_bytes(data: bytes) -> str:
    return sha256(data).hexdigest()


def digest_file(path: Path) -> str:
    hasher = sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            hasher.update(block)
    return hasher.hexdigest()


def tree_entry(path: Path) -> bytes:
    if path.is_symlink():
        return os.readlink(path).encode("utf-8")
    return path.read_bytes()


def snapshot_entries(root: Path) -> dict[str, tuple[str, Path]]:
    result = {}
    for path in sorted(root.rglob("*")):
        if path.is_file() or path.is_symlink():
            relative = str(path.relative_to(root))
            result[relative] = (digest_bytes(tree_entry(path)), path)
    return result


def archive_entries(path: Path) -> tuple[dict[str, str], str | None]:
    with zipfile.ZipFile(path) as archive:
        names = [name for name in archive.namelist() if not name.endswith("/")]
        prefix = "roka_ws/" if any(name.startswith("roka_ws/") for name in names) else ""
        result = {
            name[len(prefix):]: digest_bytes(archive.read(name))
            for name in names
            if name.startswith(prefix)
        }
    return result, result.get("HANDOFF_MANIFEST.sha256")


def copy_entry(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if source.is_symlink():
        destination.symlink_to(os.readlink(source))
    else:
        shutil.copy2(source, destination)


def shell_array(values: list[str]) -> str:
    return " ".join(f"'{value}'" for value in values)


def apply_script(
    accepted_manifests: list[str],
    target_manifest: str,
) -> str:
    accepted = shell_array(accepted_manifests)
    return f"""#!/usr/bin/env bash
set -euo pipefail

update_root="$(cd "$(dirname "${{BASH_SOURCE[0]}}")" && pwd)"
target="${{1:-$HOME/roka_ws}}"
shift || true
force=false
build_after=false
for option in "$@"; do
  case "$option" in
    --force) force=true ;;
    --build) build_after=true ;;
    *) printf '[ROKA-UPDATE][FAIL] unknown option: %s\n' "$option" >&2; exit 2 ;;
  esac
done

target="$(realpath -m "$target")"
[[ -f "$target/roka" && -d "$target/src" ]] || {{
  printf '[ROKA-UPDATE][FAIL] roka workspace not found: %s\n' "$target" >&2
  exit 2
}}

(cd "$update_root" && sha256sum -c UPDATE_MANIFEST.sha256 >/dev/null)

target_manifest='{target_manifest}'
accepted_manifests=({accepted})
current_manifest=''
if [[ -f "$target/HANDOFF_MANIFEST.sha256" ]]; then
  current_manifest="$(sha256sum "$target/HANDOFF_MANIFEST.sha256" | awk '{{print $1}}')"
fi
if [[ "$current_manifest" == "$target_manifest" ]]; then
  if (cd "$target" && sha256sum -c HANDOFF_MANIFEST.sha256 >/dev/null); then
    printf '[ROKA-UPDATE][PASS] update is already applied and verified: %s\n' "$target"
    exit 0
  fi
  printf '[ROKA-UPDATE][FAIL] current-version files were modified after installation\n' >&2
  printf 'Restore the workspace or inspect it manually before retrying.\n' >&2
  exit 3
fi

baseline_ok=false
for expected in "${{accepted_manifests[@]}}"; do
  [[ -n "$current_manifest" && "$current_manifest" == "$expected" ]] && baseline_ok=true
done
if [[ -z "$current_manifest" && -f "$target/docs/ROKA_시스템_개발상태.txt" && \
      -d "$target/maps/section_1" ]]; then
  baseline_ok=true
  printf '[ROKA-UPDATE][WARN] legacy 0829 handoff detected; map layout will also migrate\n'
fi
if [[ "$baseline_ok" != true && "$force" != true ]]; then
  printf '[ROKA-UPDATE][FAIL] target is not an accepted 0829 handoff\n' >&2
  printf 'Use --force only after manually confirming the workspace.\n' >&2
  exit 3
fi
if [[ -n "$current_manifest" && "$force" != true ]] && \
   ! (cd "$target" && sha256sum -c HANDOFF_MANIFEST.sha256 >/dev/null); then
  printf '[ROKA-UPDATE][FAIL] 0829 baseline files were modified or are missing\n' >&2
  printf 'Keep a separate copy of local changes; use --force only after manual review.\n' >&2
  exit 3
fi

stamp="$(date +%Y%m%d_%H%M%S)"
backup="$target/.roka_updates/$stamp/backup"
mkdir -p "$backup"

backup_existing() {{
  local relative="$1" source="$target/$1" saved="$backup/$1"
  if [[ -e "$source" || -L "$source" ]]; then
    mkdir -p "$(dirname "$saved")"
    mv -- "$source" "$saved"
  fi
}}

while IFS= read -r relative; do
  [[ -n "$relative" ]] || continue
  backup_existing "$relative"
done < "$update_root/DELETE_PATHS.txt"

while IFS= read -r -d '' source; do
  relative="${{source#"$update_root/overlay/"}}"
  backup_existing "$relative"
  mkdir -p "$(dirname "$target/$relative")"
  cp -a -- "$source" "$target/$relative"
done < <(find "$update_root/overlay" \( -type f -o -type l \) -print0 | sort -z)

if ! (cd "$target" && sha256sum -c HANDOFF_MANIFEST.sha256 >/dev/null); then
  printf '[ROKA-UPDATE][FAIL] post-update manifest verification failed\n' >&2
  printf '[ROKA-UPDATE] backup: %s\n' "$backup" >&2
  exit 4
fi

printf '[ROKA-UPDATE][PASS] files updated and verified\n'
printf '[ROKA-UPDATE] backup: %s\n' "$backup"
if [[ "$build_after" == true ]]; then
  "$target/tools/install_handoff.sh" --command-name roka --domain-id 31
else
  printf '[ROKA-UPDATE][NEXT] rebuild: cd %s && ./tools/install_handoff.sh --command-name roka --domain-id 31\n' "$target"
fi
"""


def change_summary(
    new_count: int,
    modified_count: int,
    deleted_count: int,
    archive_name: str,
) -> str:
    return f"""ROKA 2026-09-01 증분 업데이트
================================

기준: roka_ws_handoff_20260829.zip을 이미 설치한 roka_ws
변경: 신규 {new_count}개, 수정 {modified_count}개, 정리 {deleted_count}개

주요 변경
---------
1. VLP-16/EBIMU 동적 회전·시간 오프셋 보정 절차와 검증 추가
2. 주행·매핑·임무 중 로봇팔 시작 자세 토크 유지 및 하드웨어 사전검사 보강
3. 섹션 2 평지 지도·센서 rosbag·기록 경로 갱신
4. 지도 저장, 번들 품질 검사, SC-PGO 아카이브 및 재연결 처리 보강
5. 섹션 1 안개 직선 하이브리드 주행과 섹션 5 비학습 선두추종 구조 반영
6. rosbag 결과 재생과 빈 지도부터의 매핑 과정 재구성 RViz 추가
7. Terrain 초록/노랑/빨강/회색 판정, 기록 경로와 실차 폭 표시 추가
8. 구형 lidar_slam_ros2 배포 구조와 중복 문서 정리
9. 시뮬레이터, build/install/log/reports는 계속 배포 제외

적용
----
  unzip {archive_name}
  cd roka_ws_update
  ./apply_update.sh ~/roka_ws --build

적용 전 교체·삭제 파일은 ~/roka_ws/.roka_updates/<시각>/backup에 보관된다.
"""


def write_manifest(root: Path) -> None:
    manifest = root / "UPDATE_MANIFEST.sha256"
    lines = []
    for path in sorted(root.rglob("*")):
        if path.is_file() and not path.is_symlink() and path != manifest:
            lines.append(f"{digest_file(path)}  {path.relative_to(root)}")
    manifest.write_text("\n".join(lines) + "\n", encoding="utf-8")


def archive_tree(root: Path, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    if temporary.exists():
        temporary.unlink()
    with zipfile.ZipFile(temporary, "w", zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(root.rglob("*")):
            if not (path.is_file() or path.is_symlink()):
                continue
            relative = Path(ARCHIVE_ROOT) / path.relative_to(root)
            if path.is_symlink():
                info = zipfile.ZipInfo(str(relative))
                info.create_system = 3
                info.external_attr = (stat.S_IFLNK | 0o777) << 16
                archive.writestr(info, os.readlink(path))
            else:
                archive.write(path, relative)
    temporary.replace(output)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--baseline", action="append", type=Path, required=True)
    args = parser.parse_args()
    baselines = [path.expanduser().resolve() for path in args.baseline]
    for baseline in baselines:
        if not baseline.is_file():
            parser.error(f"baseline not found: {baseline}")

    with tempfile.TemporaryDirectory(prefix="roka_handoff_update_") as directory:
        temporary = Path(directory)
        current_root = export_handoff.copy_handoff(temporary / "snapshot")
        export_handoff.validate_handoff(current_root)
        current = snapshot_entries(current_root)

        old_snapshots = []
        accepted_manifests = []
        for baseline in baselines:
            entries, manifest = archive_entries(baseline)
            old_snapshots.append(entries)
            if manifest:
                accepted_manifests.append(manifest)

        changed = sorted(
            relative for relative, (checksum, _path) in current.items()
            if any(snapshot.get(relative) != checksum for snapshot in old_snapshots)
        )
        new = sorted(
            relative for relative in changed
            if any(relative not in snapshot for snapshot in old_snapshots)
        )
        modified = sorted(set(changed) - set(new))
        deleted = sorted(set().union(*(set(snapshot) for snapshot in old_snapshots)) - set(current))

        update_root = temporary / ARCHIVE_ROOT
        overlay = update_root / "overlay"
        for relative in changed:
            copy_entry(current[relative][1], overlay / relative)

        (update_root / "DELETE_PATHS.txt").write_text(
            "\n".join(deleted) + ("\n" if deleted else ""), encoding="utf-8"
        )
        target_manifest = digest_file(current_root / "HANDOFF_MANIFEST.sha256")
        script = update_root / "apply_update.sh"
        script.write_text(
            apply_script(sorted(set(accepted_manifests)), target_manifest),
            encoding="utf-8",
        )
        script.chmod(0o755)
        (update_root / "CHANGES_KO.txt").write_text(
            change_summary(
                len(new), len(modified), len(deleted), args.output.name
            ),
            encoding="utf-8",
        )
        baseline_lines = [
            f"{digest_file(path)}  {path.name}" for path in baselines
        ]
        (update_root / "BASELINE_ARCHIVES.sha256").write_text(
            "\n".join(baseline_lines) + "\n", encoding="utf-8"
        )
        (update_root / "UPDATE_INFO.txt").write_text(
            "\n".join([
                "ROKA incremental handoff update: 1",
                f"created_utc: {datetime.now(timezone.utc).isoformat(timespec='seconds')}",
                "target: Ubuntu 22.04 / ROS 2 Humble",
                "baselines: " + " ".join(path.name for path in baselines),
                f"new_files: {len(new)}",
                f"modified_files: {len(modified)}",
                f"deleted_files: {len(deleted)}",
                f"target_manifest_sha256: {target_manifest}",
                "simulation: excluded",
                "apply: ./apply_update.sh ~/roka_ws --build",
                "",
            ]),
            encoding="utf-8",
        )
        write_manifest(update_root)
        archive_tree(update_root, args.output.expanduser().resolve())

    output = args.output.expanduser().resolve()
    print(
        f"[ROKA-UPDATE][PASS] {output} "
        f"({output.stat().st_size / 1048576:.1f} MiB; "
        f"new={len(new)}, modified={len(modified)}, deleted={len(deleted)})"
    )


if __name__ == "__main__":
    main()
