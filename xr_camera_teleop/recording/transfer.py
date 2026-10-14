"""Finalize/start PC2 archives and verified, retryable transfer to MSI; never delete source."""
import hashlib
import json
import os
from pathlib import Path
import re
import urllib.request

import zmq
from .local_archive import safe_archive_name


def archive_command(context,host,port,operation,request,timeout_ms=18000):
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.LINGER,0)
    socket.connect(f"tcp://{host}:{port}")
    try:
        socket.send_json(dict(request,operation=operation))
        if not socket.poll(timeout_ms):
            raise TimeoutError(f"PC2 archive {port} {operation} timeout")
        answer = socket.recv_json()
        if not answer.get("ok"):
            raise RuntimeError(answer.get("error","archive command rejected"))
        return answer["result"]
    finally:
        socket.close()


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as file:
        for block in iter(lambda:file.read(1024*1024),b""):
            value.update(block)
    return value.hexdigest()


def download_archive(host,port,episode_id,destination):
    if not re.fullmatch(r"episode_[A-Za-z0-9_-]{1,100}",episode_id):
        raise ValueError("invalid episode identifier")
    base = f"http://{host}:{port}/{episode_id}"
    destination = Path(destination)
    destination.mkdir(parents=True,exist_ok=True)
    with urllib.request.urlopen(base+"/manifest.json",timeout=15) as response:
        manifest = json.load(response)
    if not manifest.get("complete") or manifest.get("episode_id")!=episode_id:
        raise ValueError("source archive not complete or wrong episode")
    for item in manifest["files"]:
        name = item["name"]
        if not safe_archive_name(name):
            raise ValueError("unsafe source filename")
        target = destination/name
        target.resolve().relative_to(destination.resolve())
        target.parent.mkdir(parents=True,exist_ok=True)
        if target.exists() and target.stat().st_size==item["bytes"] and digest(target)==item["sha256"]:
            continue
        temporary = destination/(name+".partial")
        checksum = hashlib.sha256()
        size = 0
        with urllib.request.urlopen(base+"/"+name,timeout=60) as response,temporary.open("wb") as file:
            while True:
                block = response.read(1024*1024)
                if not block:
                    break
                file.write(block)
                checksum.update(block)
                size += len(block)
            file.flush()
            os.fsync(file.fileno())
        if size!=item["bytes"] or checksum.hexdigest()!=item["sha256"]:
            raise ValueError("archive checksum mismatch: "+name)
        temporary.replace(target)
    final = dict(manifest,transfer_verified=True,source_retained=True)
    temporary = destination/"manifest.tmp"
    temporary.write_text(json.dumps(final,indent=2,ensure_ascii=False))
    temporary.replace(destination/"manifest.json")
    return final


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode_directory")
    parser.add_argument("--host",default="192.168.123.164")
    args = parser.parse_args()
    episode = Path(args.episode_directory).expanduser()
    request = json.loads((episode/"pc2_archives.json").read_text())
    for source,entry in request["sources"].items():
        if entry.get("started"):
            result = download_archive(args.host,entry["http_port"],episode.name,episode/"pc2"/source)
            print(source,result["samples"],"samples verified")


if __name__=="__main__":
    main()
