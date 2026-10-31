"""The shipped config must describe a recording the report and start check can judge."""
import json
from pathlib import Path

CONFIG = json.loads((Path(__file__).resolve().parents[2]/"configs/recording.json").read_text())


def test_every_checked_topic_is_recorded():
    recorded = set(CONFIG["topics"])
    for key in ("camera_topics","required_topics","state_topics","action_topics"):
        assert set(CONFIG[key])<=recorded,key
    assert CONFIG["tick_topic"] in recorded
    assert set(CONFIG["expected_rates_hz"])<=recorded


def test_both_cameras_block_a_start():
    assert set(CONFIG["camera_topics"])<=set(CONFIG["required_topics"])
    assert len(CONFIG["camera_topics"])==2


def test_every_state_topic_blocks_a_start():
    assert set(CONFIG["state_topics"])<=set(CONFIG["required_topics"])
