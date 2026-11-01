"""Fail-soft OpenCV display shared by mission and arm vision nodes."""

from __future__ import annotations

import os

import cv2


class OpenCvDisplay:
    """Show frames when a desktop is available without risking mission exit."""

    def __init__(self, enabled: bool, window_name: str, logger=None):
        self.window_name = str(window_name)
        self.logger = logger
        self.enabled = bool(enabled)
        self._warned = False
        if self.enabled and not (
            os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
        ):
            self.enabled = False
            self._warn_once(
                "vision display disabled: DISPLAY/WAYLAND_DISPLAY is unavailable"
            )

    def _warn_once(self, message: str) -> None:
        if self._warned:
            return
        self._warned = True
        if self.logger is not None:
            self.logger.warning(message)
        else:
            print(f"[VISION][WARN] {message}", flush=True)

    def show(self, frame) -> int:
        if not self.enabled:
            return -1
        try:
            cv2.imshow(self.window_name, frame)
            return cv2.waitKey(1) & 0xFF
        except cv2.error as error:
            self.enabled = False
            self._warn_once(f"vision display disabled after OpenCV error: {error}")
            return -1

    def close(self) -> None:
        if not self.enabled:
            return
        try:
            cv2.destroyWindow(self.window_name)
        except cv2.error:
            pass
        self.enabled = False
