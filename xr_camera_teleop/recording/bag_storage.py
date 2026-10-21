"""Read-only inspection of closed stock rosbag2 SQLite bags; no ROS imports."""
import re
import sqlite3
from pathlib import Path

import yaml


def database_paths(directory, require_metadata=False):
    directory = Path(directory)
    metadata = directory/"metadata.yaml"
    if require_metadata and not metadata.is_file():
        raise ValueError("rosbag2 metadata.yaml is missing: recorder did not finalize")
    info = {}
    if metadata.exists():
        info = (yaml.safe_load(metadata.read_text()) or {}).get("rosbag2_bagfile_information",{})
    names = info.get("relative_file_paths") or []
    if names:
        paths = []
        for name in names:
            path = directory/name
            path.resolve().relative_to(directory.resolve())
            if path.suffix!=".db3" or not path.is_file():
                raise ValueError("bag shard missing or unsupported: "+str(name))
            paths.append(path)
        return paths,info
    def natural(path):
        return [int(part) if part.isdigit() else part for part in re.split(r"(\d+)",path.name)]
    return sorted(directory.glob("*.db3"),key=natural),info


def open_readonly(path):
    return sqlite3.connect(Path(path).resolve().as_uri()+"?mode=ro",uri=True,timeout=.3)


def inspect_closed_bag(directory):
    paths,info = database_paths(directory,require_metadata=True)
    if not paths:
        raise ValueError("no SQLite bag shards found")
    counts = {}
    for path in paths:
        connection = open_readonly(path)
        try:
            tables = {row[0] for row in connection.execute("SELECT name FROM sqlite_master WHERE type='table'")}
            if not {"topics","messages"}<=tables:
                raise ValueError("invalid rosbag2 SQLite schema: "+str(path))
            for name,n in connection.execute("SELECT t.name,COUNT(m.id) FROM topics t LEFT JOIN messages m ON t.id=m.topic_id GROUP BY t.id"):
                counts[name] = counts.get(name,0)+n
        finally:
            connection.close()
    total = sum(counts.values())
    if not total:
        raise ValueError("bag contains no messages")
    declared = info.get("message_count")
    if declared is not None and declared!=total:
        raise ValueError("metadata message count does not match SQLite: %s != %s" % (declared,total))
    return {"samples":total,"message_counts":counts,"shards":[p.name for p in paths],
            "container_verified":True,"metadata_count_verified":declared is not None}


def topics_with_messages(directory):
    """Cheap startup check. Does not read payloads or count large live tables."""
    result = set()
    for path in Path(directory).glob("*.db3"):
        try:
            connection = open_readonly(path)
            try:
                for name in connection.execute("SELECT t.name FROM topics t WHERE EXISTS (SELECT 1 FROM messages m WHERE m.topic_id=t.id LIMIT 1)"):
                    result.add(name[0])
            finally:
                connection.close()
        except sqlite3.Error:
            continue  # Database creation/transaction in progress, not ready yet.
    return result
