
class ConfidenceFilter:
    """Removes detections below the configured confidence threshold."""

    def __init__(self, threshold=0.60):
        self.threshold = float(threshold)

    def process(self, detections, context):
        return [
            detection
            for detection in detections
            if float(detection["confidence"]) >= self.threshold
        ]
