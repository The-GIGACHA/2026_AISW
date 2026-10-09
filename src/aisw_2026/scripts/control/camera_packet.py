# -*- coding: utf-8 -*-
"""MORAI 카메라 UDP 조각 조립 (ROS 비의존: udp_bridge, tools/test/morai_site_check 공용)"""
import struct


class JpegAssembler:
    """MORAI 카메라 UDP 조각 -> JPEG 프레임.
    datagram: 'MOR'(3) + sec i32 + nsec i32 + chunk_idx i32 + payload_len i32 (=19B) + payload + 잔여바이트 + tail
    tail 'AI'=뒤에 조각 더 있음, 'EI'=마지막 조각. payload_len 뒤 영역은 0 패딩이 아니라 이전 조각의 잔여 바이트이므로
    반드시 payload_len으로 잘라야 함. 같은 프레임 조각은 sec/nsec이 같고 chunk_idx가 0부터 연속."""
    def __init__(self):
        self.parts, self.key, self.broken_key = [], None, None
        self.frames = 0
        self.dropped = 0   # 조각 손실/순서 이상/JPEG 시작 아님으로 버린 프레임 수

    def reset(self):
        self.parts, self.key = [], None

    def _drop(self, key):
        if key != self.broken_key:
            self.dropped += 1; self.broken_key = key
        self.reset()

    def feed(self, raw):
        """완성된 프레임이면 (sec, nsec, jpeg bytes), 아니면 None"""
        if len(raw) < 21 or raw[:3] != b'MOR': return None
        sec, nsec, idx, plen = struct.unpack_from('<iiii', raw, 3)
        key = (sec, nsec)
        if idx == 0:
            if self.parts: self._drop(self.key)   # 이전 프레임의 마지막 조각('EI') 손실
            self.parts, self.key = [], key
        if key != self.key or idx != len(self.parts) or not 0 <= plen <= len(raw) - 21:
            self._drop(key); return None
        self.parts.append(raw[19:19 + plen])
        if raw[-2:] != b'EI': return None
        data = b''.join(self.parts); self.reset()
        if data[:2] != b'\xff\xd8':
            self._drop(key); return None
        self.frames += 1
        return sec, nsec, data
