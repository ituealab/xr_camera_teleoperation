"""Minimal ROS 2 Image CDR decoding for MSI without a ROS installation."""
import struct
import zlib


def image_header(cdr, is_compressed=False, compressed_msg=False):
    """Parse only the std_msgs/Header + image metadata from a ROS Image (or CompressedImage) CDR buffer."""
    data = zlib.decompress(cdr) if is_compressed else cdr
    if data[:2] not in (b"\x00\x01", b"\x00\x00"):
        raise ValueError("unsupported ROS CDR encapsulation")
    endian = "<" if data[1] == 1 else ">"
    offset = 4

    def uint():
        nonlocal offset
        offset += (-(offset - 4)) % 4
        value = struct.unpack_from(endian + "I", data, offset)[0]
        offset += 4
        return value

    def string():
        nonlocal offset
        length = uint()
        if length < 1 or offset + length > len(data):
            raise ValueError("invalid CDR string")
        value = data[offset:offset + length - 1].decode("utf-8")
        offset += length
        return value

    sec, nanosec = uint(), uint()
    if sec >= 2**31:
        sec -= 2**32
    frame_id = string()
    if compressed_msg:
        # CompressedImage: header is followed by the format string (e.g. "bgra8; jpeg compressed bgr8").
        return {"source_timestamp_ns": sec * 1_000_000_000 + nanosec, "frame_id": frame_id,
                "height": None, "width": None, "encoding": string()}
    height, width = uint(), uint()
    encoding = string()
    return {"source_timestamp_ns": sec * 1_000_000_000 + nanosec, "frame_id": frame_id,
            "height": height, "width": width, "encoding": encoding}


def decode_image_cdr(compressed, is_compressed=True):
    data = zlib.decompress(compressed) if is_compressed else compressed
    if data[:2] not in (b"\x00\x01", b"\x00\x00"):
        raise ValueError("unsupported ROS CDR encapsulation")
    endian = "<" if data[1] == 1 else ">"
    offset = 4

    def uint():
        nonlocal offset
        offset += (-(offset-4)) % 4
        value = struct.unpack_from(endian+"I", data, offset)[0]
        offset += 4
        return value

    def string():
        nonlocal offset
        length = uint()
        if length < 1 or offset+length > len(data):
            raise ValueError("invalid CDR string")
        value = data[offset:offset+length-1].decode("utf-8")
        offset += length
        return value

    sec, nanosec = uint(), uint()
    if sec >= 2**31:
        sec -= 2**32
    frame_id = string()
    height, width = uint(), uint()
    encoding = string()
    bigendian = data[offset]
    offset += 1
    step, length = uint(), uint()
    if length != height*step or offset+length > len(data):
        raise ValueError("invalid ROS Image payload length")
    return {"stamp_ns": sec*1_000_000_000+nanosec, "frame_id": frame_id,
            "height": height, "width": width, "encoding": encoding,
            "step": step, "is_bigendian": bigendian, "pixels": data[offset:offset+length]}


def image_bgr(packet):
    import numpy as np
    import cv2
    image = (decode_image_cdr(packet["cdr_zlib"]) if "cdr_zlib" in packet
             else decode_image_cdr(packet["cdr"],is_compressed=False))
    channels = {"bgra8": 4, "rgba8": 4, "bgr8": 3, "rgb8": 3, "mono8": 1}
    encoding = image["encoding"]
    if encoding not in channels:
        raise ValueError(f"unsupported RGB encoding: {encoding}")
    count = channels[encoding]
    array = np.frombuffer(image["pixels"], np.uint8).reshape(image["height"], image["step"])
    array = array[:, :image["width"]*count].reshape(image["height"], image["width"], count)
    conversions = {"bgra8": cv2.COLOR_BGRA2BGR, "rgba8": cv2.COLOR_RGBA2BGR,
                   "rgb8": cv2.COLOR_RGB2BGR, "mono8": cv2.COLOR_GRAY2BGR}
    return cv2.cvtColor(array, conversions[encoding]) if encoding in conversions else array.copy()
