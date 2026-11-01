# ROKA YOLO 모델·필터 교체 위치

## 가장 빠르게 찾기

워크스페이스 위치와 관계없이 다음 명령으로 현재 활성 파일을 확인한다.

```bash
roka yolo-paths
```

파일 관리자로 열기:

```bash
xdg-open ~/roka_ws/src/mission_vision/models
xdg-open ~/roka_ws/src/mission_vision/config
xdg-open ~/roka_ws/src/manipulation/roka_arm/models
```

## 섹션별 활성 모델

| 용도 | 활성 PT | 같이 수정할 설정 | 실제 사용 |
|---|---|---|---|
| 섹션 1 피아식별 | `src/mission_vision/models/IFF_0830.pt` | `src/mission_vision/config/spring.yaml` | `roka ... mission1`, `roka manual section1` |
| 섹션 2 팔의 박스 파지 | `src/manipulation/roka_arm/models/box_marker_best.pt` | `src/manipulation/roka_arm/models/model_metadata.yaml`, `roka_arm/arm_config.py` | 자동 파지와 `manual section2` |
| 박스 단독 관찰용 | `src/mission_vision/models/box_0830.pt` | `src/mission_vision/config/summer.yaml` | `roka vision summer`; 팔 파지 모델과 별개 |
| 섹션 2 신호등 | `src/mission_vision/models/traffic_0831.pt` | `src/mission_vision/config/traffic.yaml` | 파지·배치 후 RED 정지/BLUE 재출발 |
| 섹션 3 표식 | `src/mission_vision/models/vision_0831.pt` | `src/mission_vision/config/autumn.yaml` | `mission3`, `manual section3` |
| 섹션 3 롤백 | `src/mission_vision/models/vision_0830.pt` | `src/mission_vision/models/model_metadata.yaml` | 자동 활성화되지 않음 |

섹션 4는 원본 운전 카메라만 사용하고, 섹션 5는 VLP-16 군집 추종이므로 현재 YOLO 파일이 없다.

섹션 1 IFF LED 연결 파일:

```text
src/mission_vision/mission_vision/neopixel_controller.py
src/mission_vision/firmware/roka_iff_neopixel/roka_iff_neopixel.ino
src/mission_vision/firmware/roka_iff_neopixel/README.md
```

펌웨어 사양은 Arduino Nano D6, NeoPixel 16개, 115200 baud이다. IFF 출력 `ENEMY/FRIEND/BOTH/NONE`은 Nano의 `E/F/B/N`과 연결된다.

## PT가 실행까지 연결되는 경로

```text
섹션 1: roka 명령
  -> src/mission/roka_mission/launch/section1_fog_hybrid.launch.py
  -> src/mission_vision/launch/spring.launch.py
  -> src/mission_vision/config/spring.yaml
  -> src/mission_vision/models/IFF_0830.pt

섹션 2 박스: roka 명령
  -> src/mission/roka_mission/launch/section2_box_pick.launch.py
  -> src/manipulation/roka_arm/launch/roka_arm.launch.py
  -> src/manipulation/roka_arm/config/roka_arm.yaml
  -> src/manipulation/roka_arm/models/box_marker_best.pt

섹션 2 신호등: 파지·배치 완료
  -> src/mission_vision/launch/traffic.launch.py
  -> src/mission_vision/config/traffic.yaml
  -> src/mission_vision/models/traffic_0831.pt
  -> src/mission/roka_mission/config/traffic_stop_go.yaml

섹션 3: roka 명령
  -> src/mission/roka_mission/launch/section3_marker_vision.launch.py
  -> src/mission_vision/launch/autumn.launch.py
  -> src/mission_vision/config/autumn.yaml
  -> src/mission_vision/models/vision_0831.pt

섹션별 수동주행 공통 분기:
  src/mission/roka_mission/launch/section_manual.launch.py
```

PT 로더와 필터 실행 흐름을 수정할 때만 다음 코드를 바꾼다.

```text
src/mission_vision/mission_vision/vision_node.py
src/mission_vision/mission_vision/pipeline.py
src/mission_vision/mission_vision/filters/
```

단순 모델 교체나 임계값 튀닝은 위 Python 코드를 건드리지 말고 PT와 해당 YAML만 수정한다.

## 모델 교체 원칙

실수 가능성이 가장 낮은 방법은 **활성 파일명은 그대로 유지**하고 PT 내용만 바꾸는 것이다.

1. 기존 PT는 활성 폴더 바깥에 백업한다.
2. 새 PT를 위 표의 활성 파일명으로 복사한다.
3. 새 체크섬을 계산한다.

```bash
sha256sum src/mission_vision/models/traffic_0831.pt
```

4. 해당 YAML의 `model.sha256`을 새 값으로 바꾼다.
5. `src/mission_vision/models/model_metadata.yaml`의 동일 모델 `sha256`도 바꾼다.
6. 체크포인트의 클래스 번호와 이름을 해당 YAML의 `expected_classes`와 정확히 맞춘다.
7. 모델 파일명을 바꾸었다면 YAML의 `model.path`, 메타데이터의 `file`, 그리고 `test/test_model_assets.py`의 파일 목록도 함께 바꾼다.

`blue/red`는 현재 신호등 모델의 정상 클래스 계약이다. 이 이름을 바꾸면 다음 파일의 주행 판정 라벨도 같이 바꿔야 한다.

```text
src/mission/roka_mission/config/traffic_stop_go.yaml
```

섹션 2 팔 파지 모델은 추가로 아래 두 곳의 기존 SHA-256을 새 값으로 바꾼다.

```text
src/manipulation/roka_arm/roka_arm/arm_config.py
src/manipulation/roka_arm/test/test_model_asset.py
```

## 필터를 바꾸는 위치

우선 Python 코드를 수정하지 말고 해당 프로필 YAML의 `pipeline` 값만 조정한다.

- 1차 YOLO 입력 임계값: `model.yolo_conf_threshold`
- 최종 confidence: `pipeline.detection/ConfidenceFilter/threshold`
- 색상 범위: `HSVRatioFilter`의 `h/s/v`, `min_ratio`, `reject_below`
- 연속 프레임 판정: `TemporalFilter`의 `window_size`, `required_count`
- 섹션 3 재인식/누적: `MarkerCountFilter`의 `disappear_frames`, `max_sequence_length`

필터 구현 파일:

```text
src/mission_vision/mission_vision/filters/confidence_filter.py
src/mission_vision/mission_vision/filters/hsv_ratio_filter.py
src/mission_vision/mission_vision/filters/temporal_filter.py
src/mission_vision/mission_vision/filters/marker_count_filter.py
```

신호등의 HSV는 현재 관찰 전용(`reject_below: false`)이다. 실측 사진으로 범위를 검증하기 전에는 임의로 차단 필터로 바꾸지 않는다.

## 교체 후 저메모리 빌드·검증

```bash
cd ~/roka_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 \
colcon build --executor sequential --symlink-install \
  --packages-select mission_vision roka_arm roka_mission

source install/setup.bash
roka verify
```

카메라만 연결해 구동부 없이 확인한다.

```bash
roka vision spring
roka vision summer
roka vision autumn
roka vision traffic
```

`YOLO checksum mismatch` 또는 `class mapping mismatch`가 나오면 PT 자체 문제가 아니라 위 체크섬/클래스 계약 파일 중 하나가 덜 바뀐 것이다.
