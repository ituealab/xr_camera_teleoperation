"""tools/view_latency.py: pairing sent frames with decoded images, stage latencies, head-motion onsets."""
import numpy as np
import pytest

from xr_camera_teleop.tools import view_latency


def test_frames_pair_with_decoded_images_by_size_and_ambiguous_sizes_are_skipped():
    frames = [{"stamp_ns": 1_000_000_000, "t_rx": 1.010, "t_sent": 1.012, "left_bytes": 500, "right_bytes": 501},
              {"stamp_ns": 1_033_000_000, "t_rx": 1.043, "t_sent": 1.045, "left_bytes": 600, "right_bytes": 601},
              {"stamp_ns": 1_066_000_000, "t_rx": 1.076, "t_sent": 1.078, "left_bytes": 600, "right_bytes": 602}]
    bitmaps = [{"size": 500, "t_call": 1020.0, "t_done": 1025.0}, {"size": 501, "t_call": 1020.1, "t_done": 1025.5},
               {"size": 600, "t_call": 1050.0, "t_done": 1055.0}]
    pairs = view_latency.match(frames, bitmaps)
    assert len(pairs) == 1 and pairs[0]["t_done"] == pytest.approx(1.025)
    rep = view_latency.image_path(pairs)
    assert rep["state_to_decoded"]["p50_ms"] == pytest.approx(25.0)
    assert rep["state_to_server"]["p50_ms"] == pytest.approx(10.0) and rep["decode"]["p50_ms"] == pytest.approx(5.0)


def test_motion_onset_finds_the_first_moved_frame():
    rng = np.random.default_rng(0)
    base = rng.integers(0, 255, (40, 60)).astype(np.uint8)
    frames = []
    for n in range(60):                                    # 30 fps; the picture shifts from t = 1.1 s on
        t = n / 30
        img = np.roll(base, 2, axis=0) if t >= 1.1 else base.copy()
        img = np.clip(img.astype(int) + rng.integers(-1, 2, img.shape), 0, 255).astype(np.uint8)
        frames.append((t, img))
    (d,) = view_latency.motion_onsets(frames, [1.0])
    assert d == pytest.approx(0.1, abs=1 / 30 + 1e-9)
    assert view_latency.motion_onsets(frames[:20], [0.5]) == [None]     # nothing moved
