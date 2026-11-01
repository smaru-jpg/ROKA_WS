"""Enable native MuJoCo viewer panels and diagnostic overlays on X11."""

import argparse
import ctypes
import ctypes.util
import time


KEY_PRESS_DELAY = 0.04


class X11Keyboard:
    """Small X11/XTest wrapper used only to configure the viewer at startup."""

    def __init__(self):
        x11_path = ctypes.util.find_library("X11")
        xtst_path = ctypes.util.find_library("Xtst")
        if not x11_path or not xtst_path:
            raise RuntimeError("libX11 and libXtst are required")

        self.x11 = ctypes.cdll.LoadLibrary(x11_path)
        self.xtst = ctypes.cdll.LoadLibrary(xtst_path)
        self.x11.XOpenDisplay.restype = ctypes.c_void_p
        self.x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self.x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
        self.x11.XDefaultRootWindow.restype = ctypes.c_ulong
        self.x11.XQueryTree.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)),
            ctypes.POINTER(ctypes.c_uint),
        ]
        self.x11.XFetchName.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.POINTER(ctypes.c_char_p),
        ]
        self.x11.XFree.argtypes = [ctypes.c_void_p]
        self.x11.XRaiseWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.x11.XResizeWindow.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.c_uint,
            ctypes.c_uint,
        ]
        self.x11.XSetInputFocus.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.c_int,
            ctypes.c_ulong,
        ]
        self.x11.XFlush.argtypes = [ctypes.c_void_p]
        self.x11.XStringToKeysym.argtypes = [ctypes.c_char_p]
        self.x11.XStringToKeysym.restype = ctypes.c_ulong
        self.x11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.x11.XKeysymToKeycode.restype = ctypes.c_uint
        self.xtst.XTestFakeKeyEvent.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint,
            ctypes.c_bool,
            ctypes.c_ulong,
        ]

        self.display = self.x11.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError("Cannot connect to the X11 display")

    def close(self):
        if self.display:
            self.x11.XCloseDisplay(self.display)
            self.display = None

    def _title(self, window):
        name = ctypes.c_char_p()
        if not self.x11.XFetchName(self.display, window, ctypes.byref(name)):
            return ""
        if not name.value:
            return ""
        title = name.value.decode("utf-8", errors="replace")
        self.x11.XFree(name)
        return title

    def _children(self, window):
        root = ctypes.c_ulong()
        parent = ctypes.c_ulong()
        children = ctypes.POINTER(ctypes.c_ulong)()
        count = ctypes.c_uint()
        if not self.x11.XQueryTree(
            self.display,
            window,
            ctypes.byref(root),
            ctypes.byref(parent),
            ctypes.byref(children),
            ctypes.byref(count),
        ):
            return ()
        result = tuple(children[index] for index in range(count.value))
        if children:
            self.x11.XFree(children)
        return result

    def find_window(self, title_fragment):
        """Find the first X11 window whose title contains the fragment."""
        root = self.x11.XDefaultRootWindow(self.display)
        pending = [root]
        while pending:
            window = pending.pop()
            if title_fragment in self._title(window):
                return window
            pending.extend(self._children(window))
        return None

    def focus(self, window):
        self.x11.XRaiseWindow(self.display, window)
        self.x11.XSetInputFocus(self.display, window, 1, 0)
        self.x11.XFlush(self.display)

    def resize(self, window, width, height):
        self.x11.XResizeWindow(self.display, window, width, height)
        self.x11.XFlush(self.display)

    def key(self, name, *, shift=False, alt=False):
        """Send one key press with optional Shift or Alt modifiers."""
        key_sym = self.x11.XStringToKeysym(name.encode("ascii"))
        key_code = self.x11.XKeysymToKeycode(self.display, key_sym)
        shift_code = self.x11.XKeysymToKeycode(
            self.display,
            self.x11.XStringToKeysym(b"Shift_L"),
        )
        alt_code = self.x11.XKeysymToKeycode(
            self.display,
            self.x11.XStringToKeysym(b"Alt_L"),
        )
        if not key_code:
            raise RuntimeError(f"Unknown X11 key: {name}")
        if shift:
            self.xtst.XTestFakeKeyEvent(self.display, shift_code, True, 0)
        if alt:
            self.xtst.XTestFakeKeyEvent(self.display, alt_code, True, 0)
        self.xtst.XTestFakeKeyEvent(self.display, key_code, True, 0)
        self.xtst.XTestFakeKeyEvent(self.display, key_code, False, 0)
        if alt:
            self.xtst.XTestFakeKeyEvent(self.display, alt_code, False, 0)
        if shift:
            self.xtst.XTestFakeKeyEvent(self.display, shift_code, False, 0)
        self.x11.XFlush(self.display)
        time.sleep(KEY_PRESS_DELAY)


def parse_arguments():
    """Parse visualization switches while tolerating ROS-added arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--title", default="MuJoCo :")
    parser.add_argument("--timeout", type=float, default=12.0)
    parser.add_argument("--width", type=int, default=800)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--left", action="store_true")
    parser.add_argument("--right", action="store_true")
    parser.add_argument("--joint", action="store_true")
    parser.add_argument("--profiler", action="store_true")
    parser.add_argument("--sensor", action="store_true")
    arguments, _ = parser.parse_known_args()
    return arguments


def main():
    """Wait for MuJoCo and enable the requested native UI elements."""
    args = parse_arguments()
    keyboard = X11Keyboard()
    try:
        deadline = time.monotonic() + args.timeout
        window = None
        while time.monotonic() < deadline:
            window = keyboard.find_window(args.title)
            if window is not None:
                break
            time.sleep(0.1)
        if window is None:
            raise RuntimeError(
                f"MuJoCo window was not found within {args.timeout:.1f}s"
            )
        if args.width < 640 or args.height < 480:
            raise ValueError("MuJoCo window must be at least 640x480")

        keyboard.resize(window, args.width, args.height)
        keyboard.focus(window)
        time.sleep(0.2)
        if args.left:
            keyboard.key("Tab")
        if args.right:
            keyboard.key("Tab", shift=True)
        if args.joint:
            keyboard.key("j", alt=True)
        if args.profiler:
            keyboard.key("F3")
        if args.sensor:
            keyboard.key("F4")
        print("Configured MuJoCo native PK-3A panel layout.")
    finally:
        keyboard.close()


if __name__ == "__main__":
    main()
