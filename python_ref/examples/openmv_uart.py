"""深圳大学 OpenMV 视觉帧：0xFF + xh xl yh yl w h。"""

from __future__ import annotations


def parse_blob_frame(buf: bytes):
    """成功返回 (x, y, w, h)，否则 None。x/y 为 blob 左上角。"""
    if len(buf) < 7:
        return None
    # 找帧头
    i = buf.find(b"\xff")
    if i < 0 or i + 7 > len(buf):
        return None
    x = (buf[i + 1] << 8) | buf[i + 2]
    y = (buf[i + 3] << 8) | buf[i + 4]
    w = buf[i + 5]
    h = buf[i + 6]
    if w == 0 or h == 0:
        return None
    return x, y, w, h
