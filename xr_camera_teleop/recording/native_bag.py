"""Original CDR -> native Foxy rosbag2 writer, with deterministic ownership and EOF close."""
import os
from pathlib import Path
import select
import sqlite3
import struct
import subprocess
import threading
import time


class NativeBagWriter:
    def __init__(self, binary, uri, startup_timeout=10):
        self.binary, self.uri = str(binary), Path(uri)
        self.process = None
        self.counts = {}
        self.log = None
        self.startup_timeout = startup_timeout

    def open(self):
        if not Path(self.binary).is_file():
            raise RuntimeError("Native bag writer missing; run deploy/build_bag_writer.sh")
        self.log = (self.uri.parent/"bag-writer.log").open("ab")
        self.process = subprocess.Popen([self.binary,str(self.uri)],stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,stderr=self.log,bufsize=0,start_new_session=True,close_fds=True)
        end = time.monotonic()+self.startup_timeout
        while time.monotonic()<end:
            if self.process.poll() is not None:
                raise RuntimeError("Native bag writer exited during startup; see bag-writer.log")
            ready,_,_ = select.select([self.process.stdout],[],[],.1)
            if ready:
                line = self.process.stdout.readline()
                if line.strip()==b"READY":
                    threading.Thread(target=self._drain_stdout,daemon=True).start()
                    return self
                self.log.write(line)
        self.abort()
        raise TimeoutError("Native bag writer readiness timeout")

    def _drain_stdout(self):
        try:
            for line in iter(self.process.stdout.readline,b""):
                if self.log and not self.log.closed:
                    self.log.write(line)
        except (ValueError,OSError):
            pass

    def write(self, topic, kind, payload, timestamp_ns):
        if not self.process or self.process.poll() is not None:
            raise RuntimeError("Native bag writer is not running")
        topic,kind = topic.encode(),kind.encode()
        header = struct.pack("<IIIQ",len(topic),len(kind),len(payload),int(timestamp_ns))
        for block in (header,topic,kind,payload):
            view = memoryview(block)
            while view:
                written = self.process.stdin.write(view)
                if not written:
                    raise BrokenPipeError("Native bag writer stopped reading")
                view = view[written:]
        key = topic.decode()
        self.counts[key] = self.counts.get(key,0)+1

    def close(self, timeout=30):
        if self.process is None:
            return
        if self.process.stdin and not self.process.stdin.closed:
            self.process.stdin.close()  # EOF, not a broad process-name kill.
        try:
            code = self.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.abort()
            raise TimeoutError("Native writer did not finalize")
        if self.log:
            self.log.close()
        if code!=0:
            raise RuntimeError("Native writer failed; bag left incomplete")
        if not (self.uri/"metadata.yaml").exists():
            raise RuntimeError("Bag metadata was not finalized")
        actual = sqlite_counts(self.uri)
        if actual!=self.counts:
            raise RuntimeError("Actual SQLite message counts differ from timing index: %r != %r" % (actual,self.counts))

    def abort(self):
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        if self.log:
            self.log.close()


def sqlite_counts(uri):
    result = {}
    for path in Path(uri).glob("*.db3"):
        connection = sqlite3.connect("file:"+str(path)+"?mode=ro",uri=True)
        try:
            for topic,count in connection.execute("SELECT t.name,COUNT(m.id) FROM topics t JOIN messages m ON m.topic_id=t.id GROUP BY t.name"):
                result[topic] = result.get(topic,0)+count
        finally:
            connection.close()
    return result
