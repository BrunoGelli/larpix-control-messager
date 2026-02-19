'''
A compatibility interface for PACMAN ZMQ message format.

This module preserves the historic ``larpix-control`` API while packing/unpacking
messages using the new PACMAN wire protocol implemented in ``larpix.format.message``:

- 24-byte header (with 64-bit timestamp and n_bytes)
- 24-byte words

Public API compatibility retained:
- format_header / parse_header
- format_word / parse_word
- format_msg / parse_msg
- format / parse
'''

import struct
import time
from bidict import bidict

from larpix import Packet_v2, Packet_v3, TriggerPacket, SyncPacket, TimestampPacket
from . import message as pm

import os

_DEBUG_ZMQ = os.getenv("LARPIX_PACMAN_MSG_DEBUG", "0") not in ("0", "", "false", "False", "no", "No")
_DEBUG_MAX_BYTES = int(os.getenv("LARPIX_PACMAN_MSG_DEBUG_MAX", "192"))


_use_pkt_version = 2
_pkt_versions = {
    2: Packet_v2,
    3: Packet_v3,
}

#: Most up-to-date message format version.
latest_version = 'new-v1-word24'

HEADER_LEN = pm.HEADER_LEN
WORD_LEN = pm.WORD_BYTES

MSG_TYPE_DATA = pm.MSG_TYPE_DATA
MSG_TYPE_REQ = pm.MSG_TYPE_REQ
MSG_TYPE_REP = pm.MSG_TYPE_REP

WORD_TYPE_DATA = pm.WORD_TYPE_DATA
WORD_TYPE_TRIG = pm.WORD_TYPE_TRIG
WORD_TYPE_SYNC = pm.WORD_TYPE_SYNC
WORD_TYPE_PING = pm.WORD_TYPE_PING
WORD_TYPE_WRITE = pm.WORD_TYPE_WRITE
WORD_TYPE_READ = pm.WORD_TYPE_READ
WORD_TYPE_TX = WORD_TYPE_DATA
WORD_TYPE_PONG = WORD_TYPE_PING
WORD_TYPE_ERR = pm.WORD_TYPE_ERR

msg_type_table = bidict([
    ('REQ', MSG_TYPE_REQ),
    ('REP', MSG_TYPE_REP),
    ('DATA', MSG_TYPE_DATA),
])

word_type_table = dict(
    REQ=bidict([
        ('PING', WORD_TYPE_PING),
        ('WRITE', WORD_TYPE_WRITE),
        ('READ', WORD_TYPE_READ),
        ('TX', WORD_TYPE_TX),
    ]),
    REP=bidict([
        ('WRITE', WORD_TYPE_WRITE),
        ('READ', WORD_TYPE_READ),
        ('TX', WORD_TYPE_TX),
        ('PONG', WORD_TYPE_PONG),
        ('ERR', WORD_TYPE_ERR),
    ]),
    DATA=bidict([
        ('DATA', WORD_TYPE_DATA),
        ('TRIG', WORD_TYPE_TRIG),
        ('SYNC', WORD_TYPE_SYNC),
    ]),
)


def _now_timestamp():
    return int(time.time_ns())


def _as_u8(value):
    if isinstance(value, (bytes, bytearray)):
        if len(value) != 1:
            raise ValueError('expected single-byte field, got {} bytes'.format(len(value)))
        return value[0]
    return int(value)


def _u64_from_packet_bytes(pkt_bytes):
    if len(pkt_bytes) != 8:
        raise ValueError('expected 8-byte payload, got {}'.format(len(pkt_bytes)))
    return struct.unpack('<Q', pkt_bytes)[0]


def _packet_bytes_from_u64(payload):
    return struct.pack('<Q', payload & 0xFFFFFFFFFFFFFFFF)


def format_header(msg_type, msg_words, timestamp=None, pacman=0):
    '''
    Generate a header-formatted bytestring of message type ``msg_type`` with
    ``msg_words`` words.
    '''
    if timestamp is None:
        timestamp = _now_timestamp()
    n_bytes = int(msg_words) * WORD_LEN
    return pm.pack_header(msg_type, n_bytes, int(timestamp), pacman=int(pacman))


def parse_header(msg):
    '''
    Returns a tuple of the data contained in the header::

        (<msg type>, <msg time>, <msg words>)
    '''
    header_data = pm.parse_header(pm.unpack_header(msg[:HEADER_LEN]))
    return (
        header_data['msg_type'],
        header_data['timestamp'],
        header_data['n_bytes'] // WORD_LEN,
    )


def format_word(msg_type, word_type, *data):
    '''
    Generate a word-formatted bytestring for ``msg_type`` and ``word_type``.
    '''
    if word_type in ('PING', 'PONG'):
        word_data = pm.content_ping(pacman=0)
    elif word_type in ('DATA', 'TX'):
        if word_type == 'TX':
            io_channel, pkt_bytes = data
            timestamp = 0
        else:
            io_channel, timestamp, pkt_bytes = data
        word_data = pm.content_data(
            channel=int(io_channel),
            timestamp=int(timestamp),
            payload=_u64_from_packet_bytes(pkt_bytes),
            pacman=0,
        )
    elif word_type == 'WRITE':
        addr, value = data
        word_data = pm.content_write(addr=int(addr), value=int(value), pacman=0)
    elif word_type == 'READ':
        addr, value = data
        word_data = pm.content_read(addr=int(addr), value=int(value), pacman=0)
    elif word_type == 'SYNC':
        sync_type, clk_src, timestamp = data
        word_data = pm.content_sync(
            sync_type=_as_u8(sync_type),
            clock_source=_as_u8(clk_src),
            timestamp=int(timestamp),
            pacman=0,
            status=0,
        )
    elif word_type == 'TRIG':
        trig_type, timestamp = data
        word_data = pm.content_trig(
            trig_type=_as_u8(trig_type),
            trig_source=0,
            timestamp=int(timestamp),
            pacman=0,
        )
    elif word_type == 'ERR':
        if len(data) >= 1 and isinstance(data[0], int):
            error_code = data[0]
        elif len(data) >= 2 and isinstance(data[1], int):
            error_code = data[1]
        else:
            error_code = 0
        word_data = pm.content_err(error_code=int(error_code), pacman=0, timestamp=0)
    else:
        raise ValueError('unknown word type: {}'.format(word_type))
    return pm.pack_word(*word_data)


def parse_word(msg_type, word):
    '''
    Returns a tuple of data contained in word, first item is always word type.
    '''
    unpacked = pm.unpack_word(word[:WORD_LEN])
    wtype = unpacked[0]

    if msg_type == 'REP' and wtype == 'PING':
        return ('PONG',)
    if wtype == 'PING':
        return ('PING',)
    if wtype in ('READ', 'WRITE'):
        return (wtype, unpacked[2], unpacked[3])
    if wtype in ('DATA', 'CFG'):
        return ('DATA', unpacked[2], unpacked[3], _packet_bytes_from_u64(unpacked[4]))
    if wtype == 'SYNC':
        return ('SYNC', bytes([unpacked[2]]), unpacked[3], unpacked[4])
    if wtype == 'TRIG':
        return ('TRIG', bytes([unpacked[2]]), unpacked[4])
    if wtype == 'ERR':
        return ('ERR', unpacked[3])

    return (wtype,) + tuple(unpacked[1:])

def format_msg(msg_type, msg_words):
    bytestream = format_header(msg_type, len(msg_words))
    for msg_word in msg_words:
        bytestream += format_word(msg_type, *msg_word)

    if _DEBUG_ZMQ:
        # Extract header fields explicitly
        header_tuple = pm.unpack_header(bytestream[:HEADER_LEN])
        header_dict = pm.parse_header(header_tuple)

        ts = header_dict['timestamp']
        n_words = header_dict['n_bytes'] // WORD_LEN

        print("\n[pacman_msg_format] TX", msg_type)
        print("  header:")
        print(f"    timestamp (64-bit) = {ts}")
        print(f"    timestamp (hex)    = 0x{ts:016x}")
        print(f"    words              = {n_words}")
        print(f"  raw ({len(bytestream)}B): {bytestream.hex()}")

    return bytestream

def parse_msg(msg):
    if _DEBUG_ZMQ:
        header_tuple = pm.unpack_header(msg[:HEADER_LEN])
        header_dict = pm.parse_header(header_tuple)

        ts = header_dict['timestamp']
        n_words = header_dict['n_bytes'] // WORD_LEN

        print("\n[pacman_msg_format] RX")
        print("  header:")
        print(f"    msg_type           = {header_dict['msg_type']}")
        print(f"    timestamp (64-bit) = {ts}")
        print(f"    timestamp (hex)    = 0x{ts:016x}")
        print(f"    words              = {n_words}")
        print(f"  raw ({len(msg)}B): {msg.hex()}")

    header = parse_header(msg)
    words = list()
    for idx in range(HEADER_LEN, len(msg), WORD_LEN):
        words.append(parse_word(header[0], msg[idx:idx + WORD_LEN]))

    if _DEBUG_ZMQ:
        print("  decoded words:")
        for w in words:
            print("   ", w)

            # --- NEW: decode LArPix payload if DATA ---
            if w[0] == 'DATA':
                io_channel = w[1]
                pacman_ts = w[2]
                payload_bytes = w[3]

                try:
                    pkt = _pkt_versions[_use_pkt_version](payload_bytes)

                    print("      ↳ LArPix packet decode:")
                    print(f"         io_channel        = {io_channel}")
                    print(f"         pacman_timestamp  = {pacman_ts}")
                    print(f"         payload_hex       = {payload_bytes.hex()}")

                    # chip id
                    if hasattr(pkt, "chip_id"):
                        print(f"         chip_id           = {pkt.chip_id}")

                    # channel id
                    if hasattr(pkt, "channel_id"):
                        print(f"         channel_id        = {pkt.channel_id}")

                    # packet type
                    if hasattr(pkt, "packet_type"):
                        print(f"         packet_type       = {pkt.packet_type}")

                    # ASIC timestamp (if exists)
                    if hasattr(pkt, "timestamp"):
                        print(f"         asic_timestamp    = {pkt.timestamp}")

                except Exception as e:
                    print(f"      ↳ payload decode failed: {e}")

    return header, words


def _replace_none(obj, attr, default=0):
    return getattr(obj, attr) if getattr(obj, attr) is not None else default


def _packet_data_req(pkt, *args):
    _use_pkt_type = _pkt_versions[_use_pkt_version]
    if isinstance(pkt, _use_pkt_type):
        return ('TX', _replace_none(pkt, 'io_channel'), pkt.bytes())
    return tuple()


def _packet_data_data(pkt, ts_pacman, *args):
    _use_pkt_type = _pkt_versions[_use_pkt_version]
    if isinstance(pkt, _use_pkt_type):
        return (
            'DATA',
            _replace_none(pkt, 'io_channel'),
            pkt.receipt_timestamp if hasattr(pkt, 'receipt_timestamp') else ts_pacman,
            pkt.bytes(),
        )
    if isinstance(pkt, SyncPacket):
        return (
            'SYNC',
            _replace_none(pkt, 'sync_type'),
            _replace_none(pkt, 'clk_source'),
            _replace_none(pkt, 'timestamp'),
        )
    if isinstance(pkt, TriggerPacket):
        return (
            'TRIG',
            _replace_none(pkt, 'trigger_type'),
            _replace_none(pkt, 'timestamp'),
        )
    return tuple()


def format(packets, msg_type='REQ', ts_pacman=0):
    '''
    Converts larpix packets into a single PACMAN message.
    The message header is automatically generated.
    '''
    get_data = _packet_data_req
    if msg_type == 'DATA':
        get_data = _packet_data_data

    word_datas = list()
    for packet in packets:
        word_data = get_data(packet, ts_pacman)
        if len(word_data) == 0:
            continue
        word_datas.append(word_data)
    return format_msg(msg_type, word_datas)


def parse(msg, io_group=None):
    '''
    Converts a PACMAN message into larpix packets.
    '''
    _use_pkt_type = _pkt_versions[_use_pkt_version]
    packets = list()
    header, word_datas = parse_msg(msg)
    packets.append(TimestampPacket(timestamp=header[1]))
    packets[0].io_group = io_group
    for word_data in word_datas:
        packet = None
        if word_data[0] in ('TX', 'DATA'):
            packet = _use_pkt_type(word_data[-1])
            packet.receipt_timestamp = word_data[2]
            packet.io_group = io_group
            packet.io_channel = word_data[1]
        elif word_data[0] == 'TRIG':
            packet = TriggerPacket(trigger_type=word_data[1], timestamp=word_data[2])
            packet.io_group = io_group
        elif word_data[0] == 'SYNC':
            packet = SyncPacket(sync_type=word_data[1], clk_source=word_data[2] & 0x01, timestamp=word_data[3])
            packet.io_group = io_group
        if packet is not None:
            packets.append(packet)
    return packets
