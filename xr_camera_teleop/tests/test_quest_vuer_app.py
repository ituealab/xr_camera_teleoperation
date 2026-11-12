from urllib.parse import parse_qs, urlparse

from xr_camera_teleop.operator.quest.vuer_app import (RECONNECT_ATTEMPTS, RECONNECT_INTERVAL_MS,
                                                      reconnect_redirect)


def test_bare_page_gets_retry_policy():
    q = parse_qs(urlparse(reconnect_redirect({})).query)
    assert q["reconnect"] == [str(RECONNECT_ATTEMPTS)] and q["interval"] == [str(RECONNECT_INTERVAL_MS)]


def test_existing_query_is_kept():
    q = parse_qs(urlparse(reconnect_redirect({"ws": "wss://h:8012"})).query)
    assert q["ws"] == ["wss://h:8012"] and "reconnect" in q


def test_no_redirect_once_policy_present():
    assert reconnect_redirect({"reconnect": "5"}) is None
