from pathlib import Path
import os
import subprocess
import sys

import pytest

from station_manager import StationManager, process_identity


@pytest.mark.skipif(not sys.platform.startswith("linux"), reason="Linux PID identity")
def test_manager_stops_only_owned_process(tmp_path: Path) -> None:
    manager = StationManager(tmp_path)
    unrelated = subprocess.Popen(["sleep", "30"])
    try:
        manager.launch("synthetic", ["sleep", "30"])
        pid = manager.entries[0]["pid"]
        assert process_identity(pid)
        manager.stop()
        assert not process_identity(pid)
        assert unrelated.poll() is None
    finally:
        unrelated.terminate()
        unrelated.wait()


@pytest.mark.skipif(not sys.platform.startswith("linux"), reason="Linux PID identity")
def test_stale_record_cannot_kill_unrelated_process(tmp_path: Path) -> None:
    manager = StationManager(tmp_path)
    manager.entries = [{"name": "stale", "pid": os.getpid(), "identity": "wrong"}]
    manager.stop()
    assert process_identity(os.getpid()) is not None
