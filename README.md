# ROKA_WS

Ubuntu 22.04와 ROS 2 Humble 기반의 ROKA 자율주행 로봇 실차 및 MuJoCo 시뮬레이션 워크스페이스입니다.

LiDAR/IMU/휠 오도메트리 기반 SLAM과 지도 주행, 지형 분석, Nav2 자율주행, 카메라 비전, 로봇팔 제어, 섹션 미션을 하나의 작업공간에서 제공합니다.

> 실차 구동 전 반드시 바퀴를 지면에서 띄우고 장치 포트와 구동 방향을 확인하세요. `md_allow_motion:=true`가 포함된 명령은 실제 모터를 구동할 수 있습니다.

## 주요 기능

- 2D/3D 지도 생성 및 기록 경로 주행
- FAST-LIO, SC-PGO, SLAM Toolbox 기반 위치추정과 매핑
- Velodyne LiDAR, EBIMU, MD200T 모터 드라이버 연동
- Nav2, 지형 지도, 전복 및 장애물 안전 감독
- Intel RealSense 기반 비전 및 YOLO 모델 추론
- Dynamixel 로봇팔과 Xbox 수동 조종
- 섹션 1~5 미션 실행
- MuJoCo 기반 자율주행 시뮬레이션과 벤치마크

## 지원 환경

- OS: Ubuntu 22.04
- ROS: ROS 2 Humble
- Python: 3.10
- 빌드 도구: `colcon`, `rosdep`
- 권장 여유 공간: 20 GiB 이상
- 저사양/8 GiB 미만 환경 권장: swap 16 GiB

## 빠른 시작

시스템 의존성 설치 및 ROS 2 Humble 설치 후 실행합니다.

```bash
cd ~/roka_ws

./tools/install_system_dependencies_low_memory.sh
./tools/install_handoff.sh --command-name roka --domain-id 31
```

설치 직후 ROKA 명령과 전체 소프트웨어 검증을 실행합니다.

```bash
roka help
roka verify
roka sections
roka yolo-paths
```

시뮬레이션 패키지까지 설치하려면 다음 옵션을 사용합니다.

```bash
./tools/install_handoff.sh --with-simulation
```

## ROKA 명령

```bash
roka help                    # 전체 실행 명령 안내
roka mapping section_1       # 섹션 1 센서 데이터 기록
roka process section_1 all   # 2D 및 3D 지도 생성
roka auto section_1 2d       # 2D 지도 기반 자율주행
roka auto section_1 3d       # 3D 지도 기반 자율주행
roka manual                  # 차체와 로봇팔 수동조종
roka drive                   # 차체 수동조종
roka manual section1         # 섹션 1 수동 미션
roka manual section2         # 섹션 2 수동 미션
roka manual section3         # 섹션 3 수동 미션
roka manual section4         # 섹션 4 수동 미션
roka manual section5         # 섹션 5 수동 미션
roka vision spring-led       # IFF 카메라 및 LED 시험
roka verify                  # 하드웨어 비구동 소프트웨어 검증
```

## 주요 구성

```text
src/
├── common/                 # 공통 로봇 설명과 URDF
├── common_msgs/            # ROKA 전용 ROS 인터페이스
├── drivers/                # IMU, 모터 드라이버
├── localization/           # FAST-LIO 및 고정 지도 위치추정
├── manipulation/           # 로봇팔 제어
├── mission/                # 섹션 미션
├── mission_vision/         # 카메라 비전과 YOLO 추론
├── navigation/             # Nav2, 지형 안전 감독
├── simulation/             # MuJoCo 시뮬레이션
├── terrain/                # 지형 지도 생성
└── vendor/                 # LiDAR 및 매핑 외부 패키지
```

주요 데이터와 산출물:

- `maps/`: 2D 맵 및 3D 포인트클라우드/경로 데이터
- `datasets/`: rosbag2 원본 및 가공 학습 데이터
- `models/`: YOLO 및 학습 정책 모델
- `reports/`: 검증 로그와 벤치마크 결과
- `third_party/`: 빌드에 필요한 사전 준비 라이브러리
- `tools/`: 설치, 검증, 인수인계 보조 스크립트

## 설치 및 검증

전체 인수인계 절차와 장치 설정은 다음 문서를 참고하세요.

- [전체 인수인계서](docs/ROKA_WS_전체_인수인계서.md)
- [통합 사용 명령](docs/ROKA_통합_사용명령.txt)
- [YOLO 모델 교체 가이드](docs/ROKA_YOLO_교체_가이드.md)
- [시스템 의존성 설치](tools/install_system_dependencies_low_memory.sh)
- [인수인계 설치 스크립트](tools/install_handoff.sh)
- [시스템 상태 점검](tools/check_system.sh)

## 하드웨어 설정

USB 장치 경로는 `/dev/ttyUSB0`처럼 번호로 고정하지 말고 `/dev/serial/by-id` 경로를 사용합니다.

```bash
ls -l /dev/serial/by-id/
rs-enumerate-devices -s
ros2 run joy joy_enumerate_devices
```

대회장이나 다른 로봇과 ROS 2 네트워크가 충돌하지 않도록 로컬 설정으로 domain ID를 분리합니다.

```bash
./tools/install_handoff.sh --domain-id 32 --force-link
```

## 데이터 관리

다음 파일 형식은 Git LFS로 관리됩니다.

- `*.zstd`: rosbag 센서 데이터
- `*.pt`: YOLO PyTorch 모델
- `*.npz`: 처리 데이터와 학습 정책

Git LFS가 없는 환경에서는 먼저 설치해야 합니다.

```bash
git lfs install
git lfs pull
```
