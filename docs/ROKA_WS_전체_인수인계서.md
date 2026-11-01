# ROKA 워크스페이스 전체 인수인계서

## 1. 이 묶음의 기준

- 기준일: 2026-09-04
- 운영체제: Ubuntu 22.04
- ROS: ROS 2 Humble
- 실차 워크스페이스: `~/roka_ws`
- 검증 결과: `roka_ws`가 없는 빈 사용자 경로에서 압축 해제, 21개 실차 패키지 순차 빌드, 새 경로 overlay 확인을 완료했다. 이어서 고정 버전 ML 런타임으로 Python 213개, IMU 7개, terrain supervisor 8개, terrain mapping 25개, SC-PGO 런타임 3개, 실차 런치 47개가 통과했고 rosdep 누락은 0개였다. SC-PGO 수치는 CTest 1개와 그 안의 실제 GTest 2개를 합산한 결과다.
- 실물 미연결 상태의 빌드·소프트웨어 검증이다. 네트워크 다운로드와 실제 USB 장치 수신은 수신 PC에서 수행해야 하며, 포트와 실제 구동 방향도 다시 확인해야 한다.

전체 이식 ZIP은 소스뿐 아니라 현재 `maps`, `datasets`, `reports`, 모델, 실측 rosbag, 보정 결과와 시뮬레이션 참고 소스까지 포함한다. 다음 항목만 의도적으로 제외한다.

- `build/`, `install/`: 현재 PC 경로와 바이너리 ABI가 들어간 재생성 산출물
- `log/`: colcon 임시 빌드 로그
- `.roka_python/`: 현재 PC용 Python 바이너리 캐시
- `__pycache__`, `.pytest_cache`, `.git`: 실행에 불필요한 캐시/저장소 내부 정보

즉 기능 원본과 취득 데이터는 보존하고, 다른 PC에서 문제를 일으킬 수 있는 생성물만 다시 만든다.

## 2. 저사양 PC 준비

권장 여유 공간은 20 GiB 이상이다. RAM 8 GiB 이상이면 순차 빌드로 진행하고, RAM이 8 GiB보다 적거나 swap이 없으면 먼저 8~16 GiB swap을 추가한다.

상태 확인:

```bash
free -h
swapon --show
df -h ~
```

16 GiB swap 예시:

```bash
sudo fallocate -l 16G /swapfile-roka
sudo chmod 600 /swapfile-roka
sudo mkswap /swapfile-roka
sudo swapon /swapfile-roka
echo '/swapfile-roka none swap sw 0 0' | sudo tee -a /etc/fstab
```

이미 충분한 swap이 있으면 위 작업을 반복하지 않는다. 빌드 중 브라우저, RViz, VS Code의 큰 프로젝트 검색, 다른 ROS 워크스페이스를 종료하면 메모리 여유가 커진다.

## 3. 새 PC 설치 순서

### 3.1 압축 해제

인수인계 폴더의 `01_압축해제.sh`를 실행하거나 직접 푼다. 기존 `~/roka_ws`가 있으면 자동으로 덮어쓰지 않는다.

```bash
cd ~/Downloads/ROKA_WS_전체_인수인계_20260904
./01_압축해제.sh
```

`Existing directory will not be overwritten`가 나오면 다음 단계를 실행하지
않는다. 기존 폴더를 보존하려면
`mv ~/roka_ws ~/roka_ws_backup_$(date +%Y%m%d_%H%M%S)`로 옮기고
`01`부터 다시 실행한다.

### 3.2 시스템 패키지 분할 설치

ROS 2 Humble을 먼저 설치한 뒤 실행한다. 빠져 있는 `rosdep`, `colcon`,
`pip` 빌드 도구는 이 단계가 먼저 소규모로 설치한다. 스크립트는 Humble용 rosdep
색인만 갱신하고 결과를 6개씩 apt에 전달하므로 한 번에 큰 설치 목록을
올리지 않는다. 이미 설치된 apt 패키지는 다시 설치하지 않는다. 색인 갱신은
기본 180초 뒤 종료하며, 기존 캐시가 있으면 이를 사용하고 새 PC에 캐시도
없으면 인터넷/DNS를 확인하라는 오류와 함께 중단한다.

```bash
cd ~/roka_ws
./tools/install_system_dependencies_low_memory.sh
```

더 작게 나누려면:

```bash
ROKA_APT_BATCH_SIZE=3 ./tools/install_system_dependencies_low_memory.sh
```

매우 느린 회선에서 rosdep 대기 시간을 5분으로 늘리려면:

```bash
ROKA_ROSDEP_UPDATE_TIMEOUT_SEC=300 ./tools/install_system_dependencies_low_memory.sh
```

### 3.3 순차 빌드와 ML 설치

```bash
cd ~/roka_ws
./tools/install_handoff.sh --command-name roka --domain-id 31
```

이 스크립트는 다음 순서로 한 패키지/컴파일 작업만 실행한다. 시스템에
우연히 설치된 다른 YOLO/PyTorch 버전에 의존하지 않도록 ML 런타임은
워크스페이스의 `.roka_python`에 고정 버전으로 따로 설치한다.

1. 인터페이스·공통 설정
2. VLP-16 드라이버
3. MD200T·IMU 드라이버
4. 카메라 비전·로봇팔
5. FAST-LIO/SC-PGO 위치추정
6. 지형·Nav2
7. 섹션 미션
8. 고정 버전 ML 스택을 한 번만 해석·다운로드하여 격리 경로에 설치
9. 전체 소프트웨어 검증

설치가 중간에 멈추면 전체 폴더를 지우지 말고 같은 명령을 다시 실행한다. 완료된 패키지는 짧게 재확인되고 실패 지점부터 계속 진행할 수 있다.

MuJoCo는 실차에 필요하지 않으므로 기본 빌드에서 제외된다. 필요한 PC에서만 실행한다.

```bash
./tools/install_handoff.sh --with-simulation
```

기본 설치는 `src/simulation` 소스를 보관하되 실차 패키지만 빌드·검증한다. MuJoCo가 필요한 PC에서만 `--with-simulation`을 사용하며, 이때는 시뮬레이션 패키지까지 오버레이 검증 대상에 포함된다.

## 4. 설치 직후 확인

새 터미널을 열고 다음을 실행한다.

```bash
roka help
roka verify
roka sections
roka yolo-paths
```

하드웨어를 연결하지 않은 상태에서는 `roka verify`까지만 수행한다. 실차 구동 명령에 `md_allow_motion:=true`가 들어가므로 바퀴를 지면에서 띄우고 포트·방향 검증 후 사용한다.

## 5. 주요 하드웨어 고정 경로

포트 번호 `/dev/ttyUSB0`를 코드에 넣지 않는다. USB를 다시 꽂아도 유지되는 `/dev/serial/by-id`를 사용한다.

- 왼쪽 MD200T: FTDI `A76BHBER`
- 오른쪽 MD200T: FTDI `A751CYHM`
- 24 V Dynamixel PRO: FTDI `FT89FBMQ`
- 12 V 손목·그리퍼: FTDI `FT78LQH1`
- 섹션 1 IFF LED Nano: 기록된 FTDI `A5069RR4`(교체 시 새 by-id 사용)
- D435: 한 대면 자동 선택, 여러 대면 PC별 설정 파일에 시리얼 지정

확인:

```bash
ls -l /dev/serial/by-id/
rs-enumerate-devices -s
ros2 run joy joy_enumerate_devices
```

다른 로봇이 같은 네트워크에 있으면 ROS domain ID를 다르게 설치한다.

```bash
./tools/install_handoff.sh --domain-id 32 --force-link
```

### 5.1 섹션 1 Arduino Nano·NeoPixel

펌웨어는 `src/mission_vision/firmware/roka_iff_neopixel/` 폴더에 포함된다. 확정 사양은 Nano D6, WS2812/NeoPixel 16개, 115200 baud이다. ROS와 Nano는 단일 바이트 `E/F/B/N`을 사용하며 각각 적군 적색, 아군 녹색, 반반 표시, 소등을 의미한다.

펌웨어 컴파일·업로드는 동봉된 `firmware/roka_iff_neopixel/README.md`를 따른다. LED를 사용할 PC에서는 실제 Nano를 연결하고 다음처럼 설치한다.

```bash
./tools/install_handoff.sh --command-name roka --domain-id 31 \
  --led-serial /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0
```

다른 Nano나 USB 어댑터로 교체했다면 `A5069RR4`를 복사하지 말고 그 PC의 `ls -l /dev/serial/by-id/` 결과를 넣는다. LED가 없는 PC에서는 `--led-serial`을 생략하면 IFF 비전만 실행된다. `spring.yaml`에 Deco PC 포트를 하드코딩하지 않는다.

모터와 로봇팔 없이 D435 IFF와 LED만 확인할 때:

```bash
roka vision spring-led
# 기록된 A5069RR4가 아닌 경우
roka vision spring-led /dev/serial/by-id/실제_NANO_경로
```

## 6. 운영 명령

전체 명령은 `docs/ROKA_통합_사용명령.txt`에 있다. 핵심 명령은 다음과 같다.

```bash
roka mapping section_1
roka process section_1 all
roka review section_1 2d
roka auto section_1 2d

roka track mission1 section_1 fog_1
roka track mission2 section_2
roka track mission3 section_3
roka track mission4 section_4
roka mission5 0.60

roka manual
roka drive
roka drive pro-hold
roka manual section1
roka manual section2
roka manual section3
roka manual section4
roka manual section5
```

섹션 2 신호등은 `RED`에서 기록경로를 취소하고 20 Hz 정지 명령을 유지한다. 검출이 `NONE`이 되어도 정지하며 `BLUE`가 2회 확정되면 현재 위치에서 같은 경로로 재결합한다. Xbox `B` 수동전환 뒤에는 신호가 바뀌어도 자율주행을 다시 켜지 않는다.

## 7. YOLO 교체

먼저 다음 명령으로 활성 파일을 확인한다.

```bash
roka yolo-paths
```

상세 교체 순서와 필터 위치는 `docs/ROKA_YOLO_교체_가이드.md`를 따른다. 특히 섹션 2의 팔 파지 모델과 단독 `summer` 박스 모델은 서로 다른 파일이다. PT만 교체하고 SHA-256과 클래스 계약을 갱신하지 않으면 안전상 실행을 거부한다.

## 8. 기존 작업에서 실제로 발생했던 문제

- 새 사용자 셸에서 `AMENT_TRACE_SETUP_FILES: 바인딩 해제한 변수`가 발생함: 의존성 설치기가 ROS Humble setup을 읽는 동안에만 `set -u`를 해제하고 즉시 다시 활성화하도록 수정했다.
- 수신 PC의 사용자 Python에 OpenCV 4.13·TorchVision 0.25·새 Flake8 등이 이미 있어 격리 설치와 SC-PGO 린터에 섞임: 모든 설치·운용에 `PYTHONNOUSERSITE=1`을 적용하고 Ubuntu의 `/usr/bin`을 사용자·`/usr/local` pip 명령보다 먼저 선택한다. ML 전체는 빈 staging 경로에 한 번만 설치·검증한 뒤 활성화한다.
- SC-PGO에서 `flake8.xunit.xml: 48 failures`가 발생함: 지도 최적화 C++ 오류가 아니라 vendored upstream의 개발용 Python 스타일 검사에 수신 PC의 전역 Flake8/플러그인이 개입한 결과였다. ROS 2 Humble 기본 `ament_flake8.ini`는 따옴표 종류를 강제하지 않으며, 현재 큰따옴표 `sc_pgo.launch.py`는 공식 Humble 검사에서 통과한다. Black과 ROS가 큰따옴표/작은따옴표로 충돌한 것이 아니다. 최종 이식본은 SC-PGO CMake에서 호스트 의존 자동 린터를 등록하지 않고, 과거 XML을 비운 뒤 실제 배포 기능인 `test_loop_constraint`만 실행한다. 정상 표시는 `SC-PGO ... 3 tests, 0 failures`다.
- ML 요구사항을 항목별로 설치하면서 Torch/CUDA 수 GiB가 반복 다운로드됨: 전체 고정 요구사항을 단일 pip transaction으로 변경했다.
- `01`이 기존 `~/roka_ws` 때문에 중단됐는데 `02`, `03`을 계속 실행함: 새 ZIP이 아니라 기존 구버전이 실행된다. `175 passed`, 동시 병렬 빌드 또는 과거 날짜 SC-PGO 결과가 보이면 중단하고 `mv ~/roka_ws ~/roka_ws_backup_$(date +%Y%m%d_%H%M%S)` 후 `01`부터 다시 실행한다.
- PyTorch가 `setuptools 84`를 설치해 Jammy `colcon-core(<80)`와 충돌함: 격리 ML 요구사항에서 양쪽 조건을 만족하는 `setuptools 79.0.1`로 고정한다.
- 다른 ROS 워크스페이스를 source해 `Package not found` 또는 구버전 노드가 실행됨: 새 터미널에서 `roka`만 사용하고 `tools/check_overlay.py` 결과를 확인한다.
- USB를 바꿔 꽂아 `/dev/ttyUSB*` 번호가 변함: 반드시 `/dev/serial/by-id`로 확인한다.
- MD200T PID210 피드백이 없어 주행이 잠김: RS485 TXD+/TXD-, 드라이버 전원, baud/ID, Hall/encoder와 PID 설정을 먼저 확인한다.
- UVW를 반대로 연결해 한쪽만 역회전하거나 과속처럼 보임: 소프트웨어 부호를 바꾸기 전에 UVW/Hall/encoder 결선을 양쪽 동일 기준으로 확인한다.
- Dynamixel 현재 위치가 허용 범위를 벗어나 전체 주행이 차단됨: 팔을 억지로 구동하지 말고 `roka arm-check hardware`와 기구 자세부터 확인한다.
- RealSense를 두 노드가 동시에 열어 영상이 끊김: 제공된 통합 런치에서는 카메라 소유자가 한 개이므로 별도 `realsense2_camera`를 동시에 실행하지 않는다.
- Xbox 수동전환 직후 움직이지 않음: `B` 후 LB를 완전히 놓은 다음 LB+스틱을 다시 사용한다.
- 커널 변경으로 Wi-Fi/GPU가 깨짐: 검증된 6.14 커널을 유지한다. 이 워크스페이스 설치에는 커널 변경이 필요 없다.
- 빌드가 멈추거나 PC가 굳음: 병렬 `colcon build`를 쓰지 말고 본 인수인계 스크립트를 재실행한다. swap과 디스크를 확인한다.

## 9. 최종 인수 확인표

- [ ] ZIP SHA-256 확인
- [ ] Ubuntu 22.04 / ROS 2 Humble 확인
- [ ] 20 GiB 이상 디스크와 RAM+swap 확인
- [ ] 시스템 의존성 분할 설치 완료
- [ ] 순차 빌드 완료
- [ ] `roka verify` PASS
- [ ] 필수 USB by-id 네 개와 사용 시 LED Nano by-id 확인
- [ ] 섹션 1 LED를 쓰면 Nano by-id·D6·16픽셀·E/F/B/N 실물 확인
- [ ] D435와 Xbox 인식 확인
- [ ] 바퀴를 띄우고 좌우 MD 개별 방향·피드백 확인
- [ ] 팔 제한값과 시작 자세 확인
- [ ] 섹션별 지도/경로와 실물 시작 위치 확인
- [ ] YOLO 실측 모델·클래스·필터 재검증
