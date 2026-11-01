from collections import Counter, defaultdict, deque


class MarkerCountFilter:
    """
    가을 비전마커용 최근 5개 누적 필터.

    기능
    1. Bounding Box 단위로 같은 클래스의 여러 객체를 모두 센다.
       예: R, O, R -> R:2, O:1

    2. 최근 window_size 프레임 중 required_count 이상
       동일 개수 이상 검출되면 안정적으로 등장했다고 판단한다.

    3. 같은 마커가 계속 보이는 동안에는 다시 추가하지 않는다.

    4. disappear_frames 이상 사라진 뒤 다시 나타나면
       새로운 마커로 다시 추가할 수 있다.

    5. 새롭게 확정된 마커는 detected_sequence에 추가하고,
       deque(maxlen=5)를 이용해 항상 최근 5개만 유지한다.
    """

    def __init__(
        self,
        window_size=10,
        required_count=7,
        disappear_frames=8,
        max_sequence_length=5
    ):
        self.window_size = int(window_size)
        self.required_count = int(required_count)
        self.disappear_frames = int(disappear_frames)

        if self.window_size <= 0:
            raise ValueError(
                "window_size must be greater than 0"
            )

        if self.required_count <= 0:
            raise ValueError(
                "required_count must be greater than 0"
            )

        if self.required_count > self.window_size:
            raise ValueError(
                "required_count cannot be greater than window_size"
            )

        # 클래스별 최근 프레임 검출 개수
        self.histories = defaultdict(
            lambda: deque(maxlen=self.window_size)
        )

        # 현재 활성 상태인 클래스별 객체 개수
        # 예: R이 현재 2개 활성 -> active_counts["R"] = 2
        self.active_counts = Counter()

        # 클래스별 연속 완전 미검출 프레임 수
        self.missing_counts = Counter()

        # 가장 최근에 확정된 마커들
        self.detected_sequence = deque(
            maxlen=int(max_sequence_length)
        )

    def process(self, result, context):

        detections = context.get(
            "filtered_detections",
            []
        )

        # -------------------------------------------------
        # 1. 현재 프레임에서 클래스별 객체 개수 계산
        # -------------------------------------------------
        current_counts = Counter()

        for detection in detections:
            class_name = str(
                detection["class_name"]
            )

            current_counts[class_name] += 1

        all_classes = (
            set(self.histories.keys())
            | set(current_counts.keys())
            | set(self.active_counts.keys())
        )

        # -------------------------------------------------
        # 2. 클래스별 안정 검출 / 사라짐 판정
        # -------------------------------------------------
        for class_name in all_classes:

            current_count = current_counts.get(
                class_name,
                0
            )

            history = self.histories[class_name]
            history.append(current_count)

            # 현재 개수 이상이 최근 window에 몇 번 있었는지
            stable_frames = 0

            if current_count > 0:
                stable_frames = sum(
                    1
                    for count in history
                    if count >= current_count
                )

            # -------------------------------------------------
            # 3. 안정적으로 보이는 경우
            # -------------------------------------------------
            if (
                current_count > 0
                and stable_frames >= self.required_count
            ):
                self.missing_counts[class_name] = 0

                previous_active = self.active_counts[
                    class_name
                ]

                # 현재 안정 검출 개수가 증가했다면
                # 새 마커가 추가로 등장했다고 판단
                if current_count > previous_active:

                    increase = (
                        current_count
                        - previous_active
                    )

                    for _ in range(increase):
                        self.detected_sequence.append(
                            class_name
                        )

                    self.active_counts[
                        class_name
                    ] = current_count

            # -------------------------------------------------
            # 4. 현재 하나도 안 보이는 경우
            # -------------------------------------------------
            elif current_count == 0:

                if self.active_counts[class_name] > 0:

                    self.missing_counts[
                        class_name
                    ] += 1

                    # 충분히 오래 사라졌다면
                    # 다음 등장부터 새로운 마커로 다시 카운트 가능
                    if (
                        self.missing_counts[class_name]
                        >= self.disappear_frames
                    ):
                        self.active_counts[
                            class_name
                        ] = 0

                        self.missing_counts[
                            class_name
                        ] = 0

                        # 과거 검출 이력 제거
                        self.histories[
                            class_name
                        ].clear()

        # -------------------------------------------------
        # 5. 항상 최근 max_sequence_length개 반환
        # -------------------------------------------------
        return list(
            self.detected_sequence
        )