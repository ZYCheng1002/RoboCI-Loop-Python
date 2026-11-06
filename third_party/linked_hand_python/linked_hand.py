"""linked_hand Python binding (ctypes only, no third-party dependency).

SocketCAN (backward compatible):
    hand = Hand("can0")

GCAN / USBCAN-II C+ (SDK must be built with LINKED_HAND_ENABLE_GCAN=ON):
    cfg = CanConfig.gcan(channel=0, bitrate=1000000)
    hand = Hand(can_config=cfg)

If liblinked_hand cannot be found, set LINKED_HAND_LIB.
GCAN driver mode Auto prefers the bundled official ECanVci runtime, with native libusb fallback on Linux.
"""

import ctypes as _c
import os as _os

_LIB_SEARCH = ["linked_hand.dll", "liblinked_hand.so", "liblinked_hand.dylib"]


def _preload_packaged_gcan_runtime():
    """Preload GCAN SONAME companions from SDK_ROOT/vendor when present.

    Some GCAN Linux packages require libECanVci.so.1 while applications open
    libECanVci.so. Loading by absolute path alone does not make the sibling
    directory a dependency search path, so preload the companion explicitly.
    """
    here = _os.path.dirname(_os.path.abspath(__file__))
    sdk_root = here
    vendor = _os.path.join(sdk_root, "vendor")
    paths = []
    env = _os.environ.get("LINKED_HAND_GCAN_LIB")
    if env:
        vendor = _os.path.dirname(_os.path.abspath(env)) or vendor
    for name in ("libusb-1.0.so.0", "libusb-1.0.so", "libusb-0.1.so.4", "libusb.so", "libECanVci.so.1"):
        p = _os.path.join(vendor, name)
        if _os.path.isfile(p):
            paths.append(p)
    for p in paths:
        try:
            _c.CDLL(p, mode=getattr(_c, "RTLD_GLOBAL", 0))
        except OSError:
            pass


_preload_packaged_gcan_runtime()


class _LibLoader:
    _lib = None

    @classmethod
    def load(cls):
        if cls._lib is not None:
            return cls._lib
        path = _os.environ.get("LINKED_HAND_LIB")
        candidates = [path] if path else []
        here = _os.path.dirname(_os.path.abspath(__file__))
        sdk_root = here
        packaged_lib_dir = _os.path.join(sdk_root, "lib")
        packaged_win_bin = _os.path.join(sdk_root, "platforms", "windows-x64", "bin")
        candidates += [_os.path.join(here, name) for name in _LIB_SEARCH]
        # Prebuilt SDK layouts.
        if _os.name == "nt":
            if hasattr(_os, "add_dll_directory") and _os.path.isdir(packaged_win_bin):
                _os.add_dll_directory(packaged_win_bin)
            candidates += [_os.path.join(packaged_win_bin, name) for name in _LIB_SEARCH]
        else:
            candidates += [_os.path.join(packaged_lib_dir, name) for name in _LIB_SEARCH]
        candidates += list(_LIB_SEARCH)
        errors = []
        for name in candidates:
            if not name:
                continue
            try:
                cls._lib = _c.CDLL(name)
                return cls._lib
            except OSError as e:
                errors.append("%s -> %s" % (name, e))
        detail = "\n  ".join(errors) if errors else "no candidates"
        raise OSError("cannot load linked_hand library. Attempts:\n  %s" % detail)


_lib = _LibLoader.load()


class Heartbeat(_c.Structure):
    _fields_ = [
        ("node_id", _c.c_uint8),
        ("seq", _c.c_uint8),
        ("calib_timeout", _c.c_uint8),
        ("calib_position_abnormal", _c.c_uint8),
        ("calib_count_abnormal", _c.c_uint8),
        ("calib_success", _c.c_uint8),
        ("calib_failed", _c.c_uint8),
        ("calib_calibrating", _c.c_uint8),
        ("motor_running", _c.c_uint8),
        ("motor_hw_overcurrent", _c.c_uint8),
        ("motor_sw_overcurrent", _c.c_uint8),
        ("motor_bus_voltage_abnormal", _c.c_uint8),
        ("motor_current_over_limit", _c.c_uint8),
        ("motor_position_over_limit", _c.c_uint8),
        ("motor_speed_over_limit", _c.c_uint8),
    ]


class _CanConfigC(_c.Structure):
    _fields_ = [
        ("backend", _c.c_int),
        ("socketcan_ifname", _c.c_char_p),
        ("gcan_device_type", _c.c_uint32),
        ("gcan_device_index", _c.c_uint32),
        ("gcan_channel", _c.c_uint32),
        ("gcan_bitrate", _c.c_uint32),
        ("gcan_acc_code", _c.c_uint32),
        ("gcan_acc_mask", _c.c_uint32),
        ("gcan_filter", _c.c_uint8),
        ("gcan_mode", _c.c_uint8),
        ("gcan_tx_can_id", _c.c_uint32),
        ("gcan_library_path", _c.c_char_p),
        ("gcan_driver_mode", _c.c_int),
    ]


class Finger:
    # Public selector -> wire DATA[1]
    THUMB_ROTATION = 1  # 0x66 拇指旋转端
    INDEX = 2           # 0x67 食指
    MIDDLE = 3          # 0x68 中指
    RING = 4            # 0x69 无名指
    LITTLE = 5          # 0x6A 小指
    THUMB_FLEX = 6      # 0x6B 拇指伸缩端

    # Backward-compatible aliases.
    THUMB_JOINT1 = THUMB_ROTATION
    THUMB_JOINT2 = THUMB_FLEX


def _packaged_gcan_library():
    """Return a vendor ECanVci library placed in SDK_ROOT/vendor, if present."""
    env = _os.environ.get("LINKED_HAND_GCAN_LIB")
    if env:
        return env
    here = _os.path.dirname(_os.path.abspath(__file__))
    sdk_root = here
    if _os.name == "nt":
        candidates = [
            _os.path.join(sdk_root, "platforms", "windows-x64", "bin", "ECanVci64.dll"),
            _os.path.join(sdk_root, "vendor", "windows-x64", "ECanVci64.dll"),
            _os.path.join(sdk_root, "vendor", "ECanVci64.dll"),
        ]
    else:
        candidates = [
            _os.path.join(sdk_root, "vendor", "libECanVci.so"),
            _os.path.join(sdk_root, "vendor", "libECanVci.so.1"),
        ]
    for candidate in candidates:
        if _os.path.isfile(candidate):
            return candidate
    return None


class CanConfig:
    SOCKETCAN = 0
    GCAN = 1
    GCAN_AUTO = 0
    GCAN_NATIVE_USB = 1
    GCAN_ECANVCI = 2

    def __init__(self, backend=SOCKETCAN, interface="can0", *, device_type=4,
                 device_index=0, channel=0, bitrate=1000000,
                 acc_code=0x00000000, acc_mask=0xFFFFFFFF,
                 filter_mode=0, mode=0, tx_can_id=0x64, library_path=None, driver="auto"):
        self.backend = backend
        self.interface = interface
        self.device_type = device_type
        self.device_index = device_index
        self.channel = channel
        self.bitrate = bitrate
        self.acc_code = acc_code
        self.acc_mask = acc_mask
        self.filter_mode = filter_mode
        self.mode = mode
        # Device-level CAN arbitration ID: right hand default 0x64, left hand default 0x65.
        # Historical argument name tx_can_id is kept for compatibility.
        self.tx_can_id = 0x64 if tx_can_id is None else int(tx_can_id)
        if not (1 <= self.tx_can_id <= 0xFF):
            raise ValueError("device CAN ID must be in range 1..255")
        self.library_path = library_path
        if isinstance(driver, str):
            d = driver.lower().replace("-", "_")
            self.driver_mode = {"auto": self.GCAN_AUTO, "native": self.GCAN_NATIVE_USB,
                                "native_usb": self.GCAN_NATIVE_USB, "ecanvci": self.GCAN_ECANVCI,
                                "legacy": self.GCAN_ECANVCI}.get(d, self.GCAN_AUTO)
        else:
            self.driver_mode = int(driver)

    @classmethod
    def socketcan(cls, interface="can0", device_can_id=0x64):
        return cls(cls.SOCKETCAN, interface, tx_can_id=device_can_id)

    @classmethod
    def gcan(cls, *, device_type=4, device_index=0, channel=0,
             bitrate=1000000, tx_can_id=0x64, device_can_id=None, library_path=None, driver="auto"):
        if library_path is None:
            library_path = _packaged_gcan_library()
        if device_can_id is not None:
            tx_can_id = device_can_id
        return cls(cls.GCAN, device_type=device_type, device_index=device_index,
                   channel=channel, bitrate=bitrate, tx_can_id=tx_can_id, library_path=library_path, driver=driver)

    def _to_c(self):
        cfg = _CanConfigC()
        _lib.lh_can_config_init(_c.byref(cfg))
        cfg.backend = self.backend
        cfg.socketcan_ifname = self.interface.encode() if self.interface else None
        cfg.gcan_device_type = self.device_type
        cfg.gcan_device_index = self.device_index
        cfg.gcan_channel = self.channel
        cfg.gcan_bitrate = self.bitrate
        cfg.gcan_acc_code = self.acc_code
        cfg.gcan_acc_mask = self.acc_mask
        cfg.gcan_filter = self.filter_mode
        cfg.gcan_mode = self.mode
        cfg.gcan_tx_can_id = self.tx_can_id
        cfg.gcan_library_path = self.library_path.encode() if self.library_path else None
        cfg.gcan_driver_mode = self.driver_mode
        return cfg


_HeartbeatCb = _c.CFUNCTYPE(None, _c.POINTER(Heartbeat), _c.c_void_p)
_OK = 0


class HandError(Exception):
    pass


class _Api:
    def __init__(self, lib):
        lib.lh_version.restype = _c.c_char_p
        lib.lh_last_error.restype = _c.c_char_p
        lib.lh_gcan_backend_available.restype = _c.c_int
        lib.lh_can_config_init.argtypes = [_c.POINTER(_CanConfigC)]
        lib.lh_can_config_init.restype = None

        lib.lh_create.restype = _c.c_void_p
        lib.lh_create.argtypes = [_c.c_char_p, _HeartbeatCb, _c.c_void_p]
        lib.lh_create_with_config.restype = _c.c_void_p
        lib.lh_create_with_config.argtypes = [_c.POINTER(_CanConfigC), _HeartbeatCb, _c.c_void_p]
        lib.lh_destroy.argtypes = [_c.c_void_p]

        lib.lh_get_limits.argtypes = [_c.c_int, _c.POINTER(_c.c_float), _c.POINTER(_c.c_float)]
        lib.lh_get_limits.restype = _c.c_int
        lib.lh_get_last_heartbeat.argtypes = [_c.c_void_p, _c.c_uint8, _c.POINTER(Heartbeat)]
        lib.lh_get_last_heartbeat.restype = _c.c_int

        for name, argt in [
            ("lh_set_timeout_ms", [_c.c_void_p, _c.c_int]),
            ("lh_enable", [_c.c_void_p, _c.c_int, _c.c_int]),
            ("lh_reset", [_c.c_void_p, _c.c_int]),
            ("lh_clear_error", [_c.c_void_p, _c.c_int]),
            ("lh_encoder_zero", [_c.c_void_p, _c.c_int, _c.c_int]),
            ("lh_start_calibration", [_c.c_void_p, _c.c_int, _c.c_int]),
            ("lh_set_calibration_current", [_c.c_void_p, _c.c_int, _c.c_float]),
            ("lh_set_calibration_offset", [_c.c_void_p, _c.c_int, _c.c_float]),
            ("lh_set_current", [_c.c_void_p, _c.c_int, _c.c_float]),
            ("lh_set_speed", [_c.c_void_p, _c.c_int, _c.c_float]),
            ("lh_set_position", [_c.c_void_p, _c.c_int, _c.c_float]),
            ("lh_get_position", [_c.c_void_p, _c.c_int, _c.POINTER(_c.c_float)]),
            ("lh_get_speed", [_c.c_void_p, _c.c_int, _c.POINTER(_c.c_float)]),
            ("lh_get_current", [_c.c_void_p, _c.c_int, _c.POINTER(_c.c_float)]),
            ("lh_set_heartbeat_interval", [_c.c_void_p, _c.c_int, _c.c_int, _c.c_uint16]),
            ("lh_set_baudrate", [_c.c_void_p, _c.c_int]),
            ("lh_set_can_id", [_c.c_void_p, _c.c_uint8]),
        ]:
            fn = getattr(lib, name)
            fn.argtypes = argt
            fn.restype = _c.c_int


_api = _Api(_lib)


def version():
    return _lib.lh_version().decode("utf-8")


def gcan_backend_available():
    return bool(_lib.lh_gcan_backend_available())


def _last_error():
    p = _lib.lh_last_error()
    return p.decode("utf-8") if p else "unknown error"


def _check(ret):
    if ret != _OK:
        raise HandError(_last_error())


def limits(finger):
    lo, hi = _c.c_float(), _c.c_float()
    _check(_lib.lh_get_limits(finger, _c.byref(lo), _c.byref(hi)))
    return lo.value, hi.value


class Hand:
    """Dexterous-hand driver handle.

    Existing code can keep using Hand("can0"). To use GCAN, pass
    can_config=CanConfig.gcan(...).
    """

    def __init__(self, can_ifname="can0", heartbeat_cb=None, *, can_config=None):
        self._py_cb = heartbeat_cb
        self._cb_holder = _HeartbeatCb(self._cb_bridge)
        self._handle = None

        if can_config is None:
            ptr = _lib.lh_create(can_ifname.encode(), self._cb_holder, None)
        else:
            c_cfg = can_config._to_c()
            ptr = _lib.lh_create_with_config(_c.byref(c_cfg), self._cb_holder, None)
        if not ptr:
            raise HandError(_last_error())
        self._handle = _c.c_void_p(ptr)

    @classmethod
    def gcan(cls, *, device_type=4, device_index=0, channel=0,
             bitrate=1000000, tx_can_id=0x64, device_can_id=None, library_path=None, driver="auto", heartbeat_cb=None):
        return cls(heartbeat_cb=heartbeat_cb,
                   can_config=CanConfig.gcan(device_type=device_type,
                                             device_index=device_index,
                                             channel=channel,
                                             bitrate=bitrate,
                                             tx_can_id=tx_can_id, device_can_id=device_can_id,
                                             library_path=library_path, driver=driver))

    def _cb_bridge(self, hb_ptr, _userdata):
        if self._py_cb is not None and hb_ptr:
            self._py_cb(hb_ptr.contents)

    def close(self):
        if self._handle:
            _lib.lh_destroy(self._handle)
            self._handle = None
            self._cb_holder = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    def _call(self, name, *args):
        _check(getattr(_lib, name)(self._handle, *args))

    def _get_float(self, name, finger):
        out = _c.c_float()
        _check(getattr(_lib, name)(self._handle, finger, _c.byref(out)))
        return out.value

    def set_timeout_ms(self, ms): self._call("lh_set_timeout_ms", ms)
    def enable(self, finger, on): self._call("lh_enable", finger, 1 if on else 0)
    def reset(self, finger): self._call("lh_reset", finger)
    def clear_error(self, finger): self._call("lh_clear_error", finger)
    def encoder_zero(self, finger, on): self._call("lh_encoder_zero", finger, 1 if on else 0)
    def start_calibration(self, finger, on): self._call("lh_start_calibration", finger, 1 if on else 0)
    def set_calibration_current(self, finger, amps): self._call("lh_set_calibration_current", finger, amps)
    def set_calibration_offset(self, finger, deg): self._call("lh_set_calibration_offset", finger, deg)
    def set_current(self, finger, amps): self._call("lh_set_current", finger, amps)
    def set_speed(self, finger, deg_per_sec=None, **kwargs):
        if deg_per_sec is None and "rad_per_sec" in kwargs:
            deg_per_sec = kwargs.pop("rad_per_sec")
        if kwargs:
            raise TypeError("unexpected keyword(s): %s" % ", ".join(kwargs))
        if deg_per_sec is None:
            raise TypeError("deg_per_sec is required")
        self._call("lh_set_speed", finger, deg_per_sec)
    def set_position(self, finger, deg): self._call("lh_set_position", finger, deg)
    def get_position(self, finger): return self._get_float("lh_get_position", finger)
    def get_speed(self, finger): return self._get_float("lh_get_speed", finger)
    def get_current(self, finger): return self._get_float("lh_get_current", finger)
    def set_heartbeat_interval(self, finger, enable, interval_ms):
        self._call("lh_set_heartbeat_interval", finger, 1 if enable else 0, interval_ms)
    def set_baudrate(self, kbps): self._call("lh_set_baudrate", kbps)
    def set_can_id(self, new_id): self._call("lh_set_can_id", new_id)

    def last_heartbeat(self, node_id):
        hb = Heartbeat()
        _check(_lib.lh_get_last_heartbeat(self._handle, node_id, _c.byref(hb)))
        return hb
