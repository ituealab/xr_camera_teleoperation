"""The PC2 (real robot) SONIC profile: configs/sonic_robot.yaml + sonic_assets_robot.yaml, and the code that reads them."""
import yaml

from xr_camera_teleop.fullbody.node import CONFIG, load_config, run_meta
from xr_camera_teleop.paths import CONFIGS
from xr_camera_teleop.sim import sim_config
from xr_camera_teleop.sonic import assets as A
from xr_camera_teleop.sonic import run_deploy as R

ROBOT = CONFIGS / "sonic_robot.yaml"


def test_robot_manifest_loads_without_a_source():
    m = A.load(CONFIGS / "sonic_assets_robot.yaml")
    assert m.source_root is None and m.python_paths == () and m.dds_domain == 0
    assert m.path("planner").name == "planner_sonic_trt85.onnx"
    assert A.planner_version_problem(m.path("planner"), m.planner_version) is None
    assert "deploy_libs" not in m.assets            # RUNPATH build


def test_robot_manifest_pins_the_same_models_as_the_sim():
    sim = yaml.safe_load((CONFIGS / "sonic_assets.yaml").read_text())["files"]
    rob = yaml.safe_load((CONFIGS / "sonic_assets_robot.yaml").read_text())["files"]
    for name in ("encoder", "decoder", "observation_config"):
        assert rob[name]["sha256"] == sim[name]["sha256"], name


def test_verify_skips_the_source_check_when_none_is_pinned(tmp_path):
    (tmp_path / "bin").write_bytes(b"x")
    path = tmp_path / "m.yaml"
    path.write_text(yaml.safe_dump({
        "roots": {"r": str(tmp_path)},
        "files": {"deploy_binary": {"root": "r", "path": "bin", "sha256": A.sha256_file(tmp_path / "bin")}},
        "deploy": {"dds_domain": 0, "planner_version": "V2"}}))
    checks = A.verify(A.load(path))
    assert [c.name for c in checks] == ["deploy_binary"] and all(c.ok for c in checks)


def test_deploy_configs_name_their_manifest():
    assert A.for_deploy(sim_config.load()["deploy"]) == A.MANIFEST
    assert A.for_deploy(sim_config.load(ROBOT)["deploy"]) == CONFIGS / "sonic_assets_robot.yaml"


def test_robot_argv_eth0_with_crc_and_precision():
    cfg = sim_config.load(ROBOT)
    argv = R.deploy_argv(A.load(A.for_deploy(cfg["deploy"])), cfg)
    assert argv[1] == "eth0" and "--disable-crc-check" not in argv
    assert argv[argv.index("--input-type") + 1] == "zmq_manager"
    assert argv[argv.index("--planner-precision") + 1] == "32"
    assert argv[argv.index("--policy-precision") + 1] == "32"
    assert argv[argv.index("--zmq-port") + 1] == "5556" and argv[argv.index("--zmq-out-port") + 1] == "5557"


def test_sim_argv_unchanged_by_the_robot_options():
    cfg = sim_config.load()
    argv = R.deploy_argv(A.load(), cfg)
    assert "--disable-crc-check" in argv and "--planner-precision" not in argv


def test_robot_profile_refuses_respawn(capsys):
    assert R.main(["--config", str(ROBOT), "--respawn"]) == 2
    assert "respawn" in capsys.readouterr().err


def test_node_sonic_config_override():
    sim = load_config()
    rob = load_config(CONFIG, "sonic_robot.yaml")
    assert rob["fullbody"] == sim["fullbody"]
    assert rob["sonic_deploy"]["assets"] == "sonic_assets_robot.yaml"
    assert (rob["sonic_deploy"]["zmq_port"], rob["sonic_deploy"]["zmq_out_port"]) == (5556, 5557)
    assert load_config(CONFIG, ROBOT)["sonic_deploy"] == rob["sonic_deploy"]   # absolute path too


def test_run_meta_records_the_robot_manifest():
    from xr_camera_teleop.fullbody.node import make_source
    cfg = load_config(CONFIG, "sonic_robot.yaml")
    meta = run_meta(make_source({"kind": "quest"}, cfg["fullbody"]), cfg, CONFIG)
    assert meta["sonic_assets"]["manifest"] == "sonic_assets_robot.yaml"
    assert meta["sonic_assets"]["dds_domain"] == 0 and meta["sonic_assets"]["source_commit"] is None
