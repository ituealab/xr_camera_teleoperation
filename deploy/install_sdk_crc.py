"""Repair SDK wheel package data in the selected env; pinned binaries, no DDS initialization."""
import hashlib
import importlib.util
from pathlib import Path
import platform
import struct
import urllib.request

REF = "814556d15970dd2ecf1c9984e845ca02ab07e206"
BINARIES = {
    "aarch64": ("crc_aarch64.so", "a4db103653db78540d141ff4dc55216b83e5298280e639b2ad54a84ac2fae9a5", 183),
    "x86_64": ("crc_amd64.so", "b136a91a7e99c5cf914f465e131f63c897a17107939184a6f85d1a57cf315ade", 62),
}


def main():
    name, expected, machine = BINARIES[platform.machine()]
    root = Path(importlib.util.find_spec("unitree_sdk2py").origin).parent
    dest = root / "utils/lib" / name
    if not dest.is_file() or hashlib.sha256(dest.read_bytes()).hexdigest() != expected:
        url = f"https://raw.githubusercontent.com/unitreerobotics/unitree_sdk2_python/{REF}/unitree_sdk2py/utils/lib/{name}"
        payload = urllib.request.urlopen(url, timeout=30).read()
        if hashlib.sha256(payload).hexdigest() != expected or payload[:4] != b"\x7fELF" or struct.unpack_from("<H", payload, 18)[0] != machine:
            raise RuntimeError("SDK CRC binary checksum or architecture mismatch")
        dest.parent.mkdir(exist_ok=True)
        temporary = dest.with_suffix(".so.tmp")
        temporary.write_bytes(payload)
        temporary.replace(dest)
    from unitree_sdk2py.utils.crc import CRC
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    value = CRC().Crc(unitree_hg_msg_dds__LowCmd_())
    print(f"SDK CRC verified: {dest}, sha256={expected}, empty HG LowCmd CRC={value}")


if __name__ == "__main__":
    main()
