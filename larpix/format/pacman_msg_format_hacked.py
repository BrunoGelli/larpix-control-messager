"""
Compatibility PACMAN message codec for old larpix-control DAQ.

This module preserves the old public API:
  - format_msg(msg_type, msg_words)
  - parse_msg(msg_bytes)
  - format(packets, msg_type='REQ', ts_pacman=0)
  - parse(msg_bytes, io_group=None)

…but uses the *new* PACMAN wire protocol:
  - 24-byte header (includes uint64 timestamp + n_bytes)
  - 24-byte fixed words
defined by the new `message.py` pack/unpack utilities.

Goal: make old DAQ talk to new firmware/server while touching as little else as possible.
"""

from __future__ import annotations

import time
import struct
from typing import List, Tuple, Any

from larpix import Packet_v2, TriggerPacket, SyncPacket, TimestampPacket

# ---- import the new-format pack/unpack implementation ----
# If your new module is named pacman_message.py, change this import accordingly.
try:
    from . import message as pm
except ImportError:
    import message as pm

#: Codec version marker (old code writes this into rawhdf5 headers etc.)
latest_version = "new-v1-word24"


# -----------------------------------------------------------------------------
# Helpers: pack/unpack payload conversions
# -----------------------------------------------------------------------------

def _now_ts() -> int:
    """Use high-resolution header timestamps (matches your new stack)."""
    return int(time.time_ns())


def _u64_from_packet_bytes(pkt_bytes: bytes) -> int:
    """
    Interpret Packet_v2 bytes (8 bytes) as a little-endian uint64 payload.
    """
    if len(pkt_bytes) != 8:
        raise ValueError(f"expected 8 bytes for Packet_v2 payload, got {len(pkt_bytes)}")
    return struct.unpack("<Q", pkt_bytes)[0]


def _packet_bytes_from_u64(payload: int) -> bytes:
    """Convert uint64 payload back into 8 bytes little-endian."""
    return struct.pack("<Q", payload & 0xFFFFFFFFFFFFFFFF)


def _build_msg(msg_type: str, word_datas: List[Tuple], header_ts: int | None = None, header_pacman: int = 0) -> bytes:
    """
    Build a new-format message using pm.pack_header/pm.pack_word.
    pm.pack_msg() in message.py does not allow passing pacman in the header,
    so we do it manually here.
    """
    if header_ts is None:
        header_ts = _now_ts()

    body_bytes = b"".join(pm.pack_word(*w) for w in word_datas)
    n_bytes = len(body_bytes)

    header_bytes = pm.pack_header(msg_type, n_bytes, header_ts, pacman=header_pacman)
    return header_bytes + body_bytes


# -----------------------------------------------------------------------------
# Old public API: format_msg / parse_msg
# -----------------------------------------------------------------------------

def format_msg(msg_type: str, msg_words: List[Tuple], header_timestamp: int | None = None) -> bytes:
    """
    Old API: msg_words is a list of tuples like ('PING',), ('READ', addr, value), etc.

    We translate those into the new word tuples expected by message.py pack_word.
    """
    word_datas: List[Tuple] = []

    for w in msg_words:
        wtype = w[0]

        if wtype in ("PING", "PONG"):
            # old REP used PONG; new protocol just uses PING word type
            word_datas.append(pm.content_ping(pacman=0))

        elif wtype == "READ":
            # old: ('READ', reg, value)
            _, addr, value = w
            word_datas.append(pm.content_read(addr=addr, value=value, pacman=0))

        elif wtype == "WRITE":
            # old: ('WRITE', reg, value)
            _, addr, value = w
            word_datas.append(pm.content_write(addr=addr, value=value, pacman=0))

        elif wtype in ("DATA", "TX"):
            # old PACMAN words carried the raw 8-byte Packet_v2 payload.
            # new: we encode that 8-byte payload into the 64-bit payload field.
            #
            # w is typically ('TX', io_channel, pkt_bytes) or ('DATA', io_channel, ts, pkt_bytes)
            if wtype == "TX":
                _, chan, pkt_bytes = w
                ts_word = 0
            else:
                _, chan, ts_word, pkt_bytes = w

            payload_u64 = _u64_from_packet_bytes(pkt_bytes)
            word_datas.append(pm.content_data(channel=int(chan), timestamp=int(ts_word), payload=payload_u64, pacman=0))

        elif wtype == "SYNC":
            # old: ('SYNC', sync_type, clk_source, timestamp)
            _, sync_type, clk_src, ts_word = w
            word_datas.append(pm.content_sync(sync_type=int(sync_type), timestamp=int(ts_word), pacman=0, clock_source=int(clk_src), status=0))

        elif wtype == "TRIG":
            # old: ('TRIG', trig_type, timestamp)
            _, trig_type, ts_word = w
            word_datas.append(pm.content_trig(trig_type=int(trig_type), timestamp=int(ts_word), pacman=0, trig_source=0))

        elif wtype == "ERR":
            # old: ('ERR', io_channel?, blob) – but old pacman_msg_format used ('ERR', ch, 14s)
            # new has ('ERR', pacman, timestamp, error_code). We can’t perfectly map without knowing server semantics.
            # Best effort: treat blob as error_code if it looks int-like.
            if len(w) == 2 and isinstance(w[1], int):
                word_datas.append(pm.content_err(error_code=int(w[1]), pacman=0, timestamp=0))
            else:
                # fallback generic error code
                word_datas.append(pm.content_err(error_code=0xEEEE, pacman=0, timestamp=0))

        else:
            raise ValueError(f"Unsupported word type in format_msg: {wtype}")

        msg = _build_msg(msg_type, word_datas, header_ts=header_timestamp)
        print(f"from pacman_msg_format.py -> TX {msg_type}: {msg.hex()}")

    return _build_msg(msg_type, word_datas, header_ts=header_timestamp)


def parse_msg(msg: bytes) -> Tuple[Tuple, List[Tuple]]:
    """
    Old API returned:
      header = ('REQ'|'REP'|'DATA', timestamp, n_words)
      words  = [ ('PING', ...), ('READ', ...), ...]
    """
    header_tuple = pm.unpack_header(msg[:pm.HEADER_LEN])
    hdr = pm.parse_header(header_tuple)

    msg_type = hdr["msg_type"]
    header_ts = hdr["timestamp"]
    n_words = hdr["n_bytes"] // pm.WORD_BYTES

    # decode all words
    words_out: List[Tuple] = []
    for off in range(pm.HEADER_LEN, pm.HEADER_LEN + hdr["n_bytes"], pm.WORD_BYTES):
        w = pm.unpack_word(msg[off:off + pm.WORD_BYTES])
        wtype = w[0]

        # Keep old naming conventions where it matters:
        if msg_type == "REP" and wtype == "PING":
            words_out.append(("PONG",))
            continue

        if wtype == "PING":
            words_out.append(("PING",))
        elif wtype == "READ":
            _, pacman_id, addr, value = w
            words_out.append(("READ", addr, value))
        elif wtype == "WRITE":
            _, pacman_id, addr, value = w
            words_out.append(("WRITE", addr, value))
        elif wtype == "DATA":
            _, pacman_id, chan, ts_word, payload = w
            pkt_bytes = _packet_bytes_from_u64(payload)
            # old DATA word tuple was ('DATA', io_channel, receipt_ts, pkt_bytes)
            words_out.append(("DATA", chan, ts_word, pkt_bytes))
        elif wtype == "CFG":
            _, pacman_id, chan, ts_word, payload = w
            pkt_bytes = _packet_bytes_from_u64(payload)
            # treat CFG like DATA for old DAQ purposes (still an 8-byte payload)
            words_out.append(("DATA", chan, ts_word, pkt_bytes))
        elif wtype == "SYNC":
            _, pacman_id, sync_type, clk_src, ts_word, status = w
            words_out.append(("SYNC", sync_type, clk_src, ts_word))
        elif wtype == "TRIG":
            _, pacman_id, trig_type, trig_src, ts_word = w
            words_out.append(("TRIG", trig_type, ts_word))
        elif wtype == "ERR":
            _, pacman_id, ts_word, err_code = w
            words_out.append(("ERR", err_code))
        else:
            # unknown word type: keep it visible
            words_out.append((wtype,) + tuple(w[1:]))

    header_out = (msg_type, header_ts, n_words)
    print(f"from pacman_msg_format.py -> RX: {msg.hex()}")
    return header_out, words_out


# -----------------------------------------------------------------------------
# Old public API: format / parse for larpix Packet objects
# -----------------------------------------------------------------------------

def _replace_none(obj, attr, default=0):
    v = getattr(obj, attr)
    return v if v is not None else default


def format(packets, msg_type: str = "REQ", ts_pacman: int = 0) -> bytes:
    """
    Old behavior:
      - REQ: only Packet_v2 are transmitted (TX words)
      - DATA: Packet_v2, SyncPacket, TriggerPacket become DATA/SYNC/TRIG words

    New behavior here:
      - REQ: Packet_v2 -> DATA words with timestamp=0 (best guess TX encoding)
      - DATA: Packet_v2 -> DATA words (receipt timestamp if present)
             SyncPacket/TriggerPacket -> SYNC/TRIG words
    """
    msg_words: List[Tuple] = []

    if msg_type == "REQ":
        for pkt in packets:
            if isinstance(pkt, Packet_v2):
                chan = _replace_none(pkt, "io_channel")
                # old TX word carried only bytes; map to DATA with ts=0
                msg_words.append(("TX", chan, pkt.bytes()))
        return format_msg("REQ", msg_words)

    if msg_type == "DATA":
        for pkt in packets:
            if isinstance(pkt, Packet_v2):
                chan = _replace_none(pkt, "io_channel")
                ts_word = getattr(pkt, "receipt_timestamp", ts_pacman)
                msg_words.append(("DATA", chan, ts_word, pkt.bytes()))
            elif isinstance(pkt, SyncPacket):
                msg_words.append(("SYNC",
                                  _replace_none(pkt, "sync_type"),
                                  _replace_none(pkt, "clk_source"),
                                  _replace_none(pkt, "timestamp")))
            elif isinstance(pkt, TriggerPacket):
                msg_words.append(("TRIG",
                                  _replace_none(pkt, "trigger_type"),
                                  _replace_none(pkt, "timestamp")))
        return format_msg("DATA", msg_words)

    # REP is rarely constructed by client side in larpix-control, but keep available
    return format_msg(msg_type, msg_words)


def parse(msg: bytes, io_group=None):
    """
    Old behavior:
      returns [TimestampPacket(header_ts), <decoded packets...>]
    """
    packets = []
    header, words = parse_msg(msg)

    # TimestampPacket: old DAQ expects this first
    ts_pkt = TimestampPacket(timestamp=header[1])
    ts_pkt.io_group = io_group
    packets.append(ts_pkt)

    for w in words:
        wtype = w[0]
        pkt = None

        if wtype == "DATA":
            _, chan, receipt_ts, pkt_bytes = w
            pkt = Packet_v2(pkt_bytes)
            pkt.receipt_timestamp = receipt_ts
            pkt.io_group = io_group
            pkt.io_channel = chan

        elif wtype == "TRIG":
            _, trig_type, ts_word = w
            pkt = TriggerPacket(trigger_type=trig_type, timestamp=ts_word)
            pkt.io_group = io_group

        elif wtype == "SYNC":
            _, sync_type, clk_src, ts_word = w
            pkt = SyncPacket(sync_type=sync_type, clk_source=int(clk_src) & 0x01, timestamp=ts_word)
            pkt.io_group = io_group

        # Ignore pure control words at packet layer (PING/READ/WRITE) – old code didn’t convert those either.
        if pkt is not None:
            packets.append(pkt)

    return packets
