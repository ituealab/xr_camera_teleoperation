"""One episode: `ros2 bag record` in the recorder container, verified, stopped and reported."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

from .report import build_report_isolated,topics_with_messages
from .schema import NATIVE_JOINT_NAMES,prompt_directory

CONTAINER = "xrcam-recorder"
CYCLONE = ('<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="%s"/></Interfaces>'
           '<AllowMulticast>spdp</AllowMulticast></General><Internal>'
           '<MinimumSocketReceiveBufferSize>1MB</MinimumSocketReceiveBufferSize></Internal></Domain></CycloneDDS>')


def recorder_command(config,directory):
    return ["docker","run","--rm","--init","--name",CONTAINER,"--net=host","--user","%d:%d" % (os.getuid(),os.getgid()),
            "-e","HOME=/tmp","-e","RMW_IMPLEMENTATION=rmw_cyclonedds_cpp","-e","ROS_DOMAIN_ID=%d" % config["dds_domain"],
            "-e","CYCLONEDDS_URI="+CYCLONE % config["dds_interface"],"-v","%s:/episode" % directory,
            config["recorder_image"],"-o","/episode/bag","-s","mcap","--include-hidden-topics",*config["topics"]]


class EpisodeSession:
    def __init__(self,config,root=None,process_factory=subprocess.Popen,report_builder=build_report_isolated):
        self.config = config
        self.root = Path(root or config["record_root"]).expanduser()
        self.process_factory,self.report_builder = process_factory,report_builder
        self.process = self.directory = self.manifest = self.log = self.started_at = None

    @property
    def recording(self):
        return self.process is not None

    @property
    def alive(self):
        return self.process is not None and self.process.poll() is None

    def stop_stale_recorder(self):
        """A recorder outliving a crashed UI would otherwise keep writing and block the next start."""
        subprocess.run(["docker","kill","--signal=INT",CONTAINER],capture_output=True)

    def _write_manifest(self):
        temporary = self.directory/"manifest.tmp"
        temporary.write_text(json.dumps(self.manifest,indent=2,ensure_ascii=False))
        temporary.replace(self.directory/"manifest.json")

    def start(self,prompt):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.directory = self.root/prompt_directory(prompt)/"raw"/("episode_%s_%s" % (stamp,uuid.uuid4().hex[:8]))
        self.directory.mkdir(parents=True)
        self.manifest = {"format":"xrcam-episode-v1","episode_id":self.directory.name,"prompt":prompt,
                         "complete":False,"topics":self.config["topics"],"created_unix_ns":time.time_ns(),
                         "joint_names_native_order":list(NATIVE_JOINT_NAMES)}
        self._write_manifest()
        self.log = (self.directory/"recorder.log").open("ab")
        self.process = self.process_factory(recorder_command(self.config,self.directory),stdin=subprocess.DEVNULL,
                                            stdout=self.log,stderr=self.log)
        required = set(self.config["required_topics"])
        deadline = time.monotonic()+self.config["start_timeout_seconds"]
        seen = set()
        while time.monotonic()<deadline and self.alive:
            seen = topics_with_messages(self.directory/"bag",required)
            if required<=seen:
                self.started_at = time.monotonic()
                self.manifest.update(ready_unix_ns=time.time_ns())
                self._write_manifest()
                return self.directory
            time.sleep(.1)
        self.stop("start_failed",True)
        raise RuntimeError("no messages on: "+", ".join(sorted(required-seen)))

    def _end_recorder(self):
        self.process.send_signal(signal.SIGINT)  # docker run forwards it; rosbag2 writes the MCAP footer.
        try:
            self.process.wait(timeout=self.config["stop_timeout_seconds"])
        except subprocess.TimeoutExpired:
            subprocess.run(["docker","kill",CONTAINER],capture_output=True)
            self.process.wait(timeout=10)

    def stop(self,reason="manual",incomplete=False):
        self._end_recorder()
        self.log.close()
        try:
            report = self.report_builder(self.directory/"bag",self.config)
        except Exception as exc:
            report = {"closed":False,"incomplete":True,"verdict":"issues","issues":["report: "+str(exc)]}
        (self.directory/"report.json").write_text(json.dumps(report,indent=2))
        self.manifest.update(complete=report["closed"],incomplete=bool(incomplete or report["incomplete"]),
                             stop_reason=reason,closed_unix_ns=time.time_ns(),verdict=report["verdict"],
                             issues=report["issues"])
        self._write_manifest()
        result = dict(self.manifest,path=str(self.directory))
        self.process = self.started_at = None
        return result
