
from collections import Counter, deque


class TemporalFilter:
    """
    Generic temporal voting filter.

    Works with arbitrary model class names, so it is not tied to
    Friend/Enemy, Red/Green, Marker1/Marker2, etc.
    """

    def __init__(self, window_size=5, required_count=3):
        self.window_size = int(window_size)
        self.required_count = int(required_count)

        if self.window_size <= 0:
            raise ValueError("window_size must be greater than 0")
        if self.required_count <= 0:
            raise ValueError("required_count must be greater than 0")
        if self.required_count > self.window_size:
            raise ValueError(
                "required_count cannot be greater than window_size"
            )

        # Each entry is a set of class names seen in one frame.
        self.history = deque(maxlen=self.window_size)

    def process(self, result, context):
        detection_mode = str(
            context.get("detection_mode", "single")
        ).lower()

        if detection_mode == "single":
            frame_classes = set()
            if result is not None:
                frame_classes.add(str(result))
        elif detection_mode == "multiple":
            frame_classes = {
                str(class_name)
                for class_name in (result or [])
                if class_name is not None
            }
        else:
            raise ValueError(
                "TemporalFilter requires detection_mode "
                "'single' or 'multiple'"
            )

        self.history.append(frame_classes)

        counts = Counter()
        for classes in self.history:
            counts.update(classes)

        if detection_mode == "single":
            if not counts:
                return None

            best_count = max(counts.values())
            if best_count < self.required_count:
                return None

            candidates = {
                name for name, count in counts.items()
                if count == best_count
            }

            # Tie-breaker: prefer the class seen most recently.
            for classes in reversed(self.history):
                for name in classes:
                    if name in candidates:
                        return name

            return None

        confirmed = [
            name
            for name, count in counts.items()
            if count >= self.required_count
        ]

        confirmed.sort(
            key=lambda name: (-counts[name], name)
        )
        return confirmed
