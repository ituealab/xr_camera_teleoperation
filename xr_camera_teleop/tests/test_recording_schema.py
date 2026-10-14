from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from xr_camera_teleop.recording.schema import (
    ARM_INDICES, NATIVE_JOINT_NAMES, SampleStamp, joint_permutation,
    prompt_directory, validate_joint_vector,
)


def test_native_order_matches_actual_dds_enums_and_urdf():
    root = Path(__file__).resolve().parents[1]
    # Parse source with AST: importing the controllers would load DDS hardware dependencies.
    import ast
    source = ast.parse((root / "robot/vendor/robot_hand_unitree.py").read_text())
    expected = {
        "Dex3_1_Left_JointIndex": ["Thumb0", "Thumb1", "Thumb2", "Middle0", "Middle1", "Index0", "Index1"],
        "Dex3_1_Right_JointIndex": ["Thumb0", "Thumb1", "Thumb2", "Index0", "Index1", "Middle0", "Middle1"],
    }
    for node in source.body:
        if isinstance(node, ast.ClassDef) and node.name in expected:
            names = [stmt.targets[0].id for stmt in node.body if isinstance(stmt, ast.Assign)]
            assert [name.split("Hand", 1)[1] for name in names] == expected.pop(node.name)
    assert not expected
    urdf = ET.parse(root / "assets/g1/g1_body29_hand14.urdf")
    names = {joint.attrib["name"] for joint in urdf.findall("joint")}
    assert set(NATIVE_JOINT_NAMES) <= names
    assert ARM_INDICES == tuple(range(15, 29))


def test_permutation_is_name_based_and_rejects_ambiguous_reference():
    reference = list(reversed(NATIVE_JOINT_NAMES))
    native = list(range(28))
    assert [native[i] for i in joint_permutation(reference)] == list(reversed(native))
    with pytest.raises(ValueError):
        joint_permutation(list(NATIVE_JOINT_NAMES[:-1]) + [NATIVE_JOINT_NAMES[0]])


def test_prompt_path_cannot_escape_and_sanitization_does_not_collide():
    assert "/" not in prompt_directory("../../Pick / apple")
    assert prompt_directory("Pick / apple") != prompt_directory("Pick ? apple")
    assert prompt_directory("Put the apple into the plate").startswith("Put_the_apple_into_the_plate__")
    with pytest.raises(ValueError):
        prompt_directory("  ")


def test_timestamp_semantics_and_invalid_vectors():
    stamp = SampleStamp.collected("zed_left", "session-1", 0,
                                 source_timestamp_ns=123, source_clock_id="ros_header",
                                 source_time_semantics="header_unverified")
    assert stamp.to_dict()["sequence_semantics"] == "collector_generated"
    with pytest.raises(ValueError):
        SampleStamp.collected("dds_state", "session-1", 0, source_timestamp_ns=123)
    with pytest.raises(ValueError):
        validate_joint_vector([0] * 27 + [float("nan")])
    assert len(validate_joint_vector([0] * 28)) == 28
