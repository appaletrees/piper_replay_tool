import csv
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from app import create_app
from catalog import Catalog
from engine import ReplayEngine, valid_rate
from hardware import Cancelled, DemoRobot, PiperRobot
from video import RecordingReader


def wait_for(predicate, timeout=3):
    deadline = time.monotonic()+timeout
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError("Timed out")
        time.sleep(.005)


class FakeCatalog:
    def __init__(self, length=20):
        self.length = length

    def listing(self):
        return []

    def load(self, source_id, episode, field="action"):
        target = np.repeat((np.arange(self.length)/100)[:, None], 14, axis=1)
        return dict(id=source_id, title="test", episode=episode, field=field, fps=30., frames=self.length,
                    task="test", original_unit="radians", parquet="fixture", videos=[], target=target,
                    timestamps=np.arange(self.length)/30, auxiliary=np.ones(self.length))


class RecorderRobot(DemoRobot):
    def __init__(self):
        super().__init__()
        self.sent = []

    def send(self, target):
        self.sent.append((time.monotonic(), np.asarray(target).copy()))
        return super().send(target)


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.robot = RecorderRobot()
        self.engine = ReplayEngine(FakeCatalog(), self.robot, Path(self.temp.name))

    def tearDown(self):
        self.engine.close()
        self.temp.cleanup()

    def ready(self):
        self.engine.submit("select", dict(dataset="test", episode=0))
        wait_for(lambda: self.engine.episode is not None and not self.engine.busy)
        self.engine.submit("connect")
        wait_for(lambda: self.robot.connected and not self.engine.busy)

    def test_selection_and_connection_send_no_motion(self):
        self.ready()
        self.engine.submit("seek", dict(frame=5))
        wait_for(lambda: self.engine.frame == 5 and not self.engine.busy)
        self.assertEqual(self.robot.sent, [])
        self.assertFalse(self.robot.enabled)

    def test_replay_exact_samples_at_selected_rate_and_record_feedback(self):
        self.ready()
        self.engine.submit("start", dict(hz=60, align_seconds=0))
        wait_for(lambda: self.engine.phase == "completed")
        np.testing.assert_array_equal(np.array([x[1] for x in self.robot.sent]), self.engine.episode["target"])
        elapsed = self.robot.sent[-1][0]-self.robot.sent[0][0]
        self.assertGreaterEqual(elapsed, 19/60*.95)
        self.assertLess(elapsed, 19/60+.15)
        self.assertTrue(self.engine.samples)
        with self.engine.log_path.open() as stream:
            rows = list(csv.DictReader(stream))
        self.assertTrue(rows)
        self.assertIn("feedback_right_j6", rows[0])
        self.assertIn("driver_target_left_gripper", rows[0])

    def test_pause_hold_resume_does_not_drop_or_repeat_dataset_frames(self):
        self.ready()
        self.engine.submit("start", dict(hz=30, align_seconds=0))
        wait_for(lambda: len(self.robot.sent) >= 3)
        self.engine.submit("pause")
        wait_for(lambda: self.engine.phase == "paused")
        count = len(self.robot.sent)
        time.sleep(.08)
        self.assertEqual(len(self.robot.sent), count)
        self.engine.submit("start", dict(hz=100, align_seconds=0))
        wait_for(lambda: self.engine.phase == "completed")
        np.testing.assert_array_equal(np.array([x[1] for x in self.robot.sent]), self.engine.episode["target"])

    def test_cancel_during_enable_prevents_trajectory_commands(self):
        self.ready()
        started = threading.Event()
        def slow_enable(cancel):
            started.set()
            while not cancel.wait(.005):
                pass
            raise Cancelled("cancelled")
        self.robot.enable = slow_enable
        self.engine.submit("start", dict(hz=30, align_seconds=0))
        self.assertTrue(started.wait(1))
        self.engine.submit("disable")
        wait_for(lambda: self.engine.phase == "disabled")
        self.assertEqual(self.robot.sent, [])

    def test_alignment_is_separate_and_pause_cannot_skip_its_remainder(self):
        self.ready()
        self.engine.submit("seek", dict(frame=10))
        wait_for(lambda: self.engine.frame == 10 and not self.engine.busy)
        self.engine.submit("start", dict(hz=60, align_seconds=.2))
        wait_for(lambda: self.engine.phase == "aligning" and len(self.robot.sent)>2)
        self.engine.submit("pause")
        wait_for(lambda: self.engine.phase == "paused")
        self.engine.submit("start", dict(hz=60, align_seconds=.2))
        wait_for(lambda: self.engine.phase == "aligning")
        wait_for(lambda: self.engine.phase == "completed")
        np.testing.assert_array_equal(np.array([v for _, v in self.robot.sent[-10:]]), self.engine.episode["target"][10:])

    def test_rate_and_live_selection_validation(self):
        for value in (0, -2, float("inf"), float("nan"), 101):
            with self.assertRaises(ValueError):
                valid_rate(value)
        self.ready()
        self.engine.submit("start", dict(hz=15, align_seconds=0))
        wait_for(lambda: self.engine.phase == "playing")
        with self.assertRaises(ValueError):
            self.engine.submit("seek", dict(frame=3))
        self.engine.submit("rate", dict(hz=30))
        wait_for(lambda: self.engine.hz == 30)

    def test_completed_disable_restart_and_summary_preservation(self):
        self.ready()
        self.engine.submit("start", dict(hz=100, align_seconds=0))
        wait_for(lambda: self.engine.phase == "completed")
        original_summary = self.engine.log_path.parent / "summary.json"
        saved = original_summary.read_text()
        self.engine.submit("disable")
        wait_for(lambda: self.engine.phase == "disabled")
        self.engine.submit("start", dict(hz=100, align_seconds=.05))
        wait_for(lambda: self.engine.phase == "aligning")
        wait_for(lambda: self.engine.phase == "completed")
        self.assertEqual(self.engine.sent_frames, 20)
        self.assertEqual(original_summary.read_text(), saved)
        self.engine.submit("select", dict(dataset="test", episode=1))
        wait_for(lambda: self.engine.episode["episode"] == 1 and not self.engine.busy)
        self.assertEqual(self.engine.sent_frames, 0)

    def test_single_frame_holds_exact_target_and_continues_feedback_recording(self):
        self.ready()
        self.engine.submit("goto_frame", dict(frame=9, align_seconds=0))
        wait_for(lambda: self.engine.phase == "holding")
        count = len(self.robot.sent)
        samples = self.engine._sequence
        time.sleep(.12)
        self.assertEqual(len(self.robot.sent), count)
        self.assertEqual(self.engine.frame, 9)
        self.assertEqual(self.engine.next_frame, 9)
        np.testing.assert_array_equal(self.engine.target, self.engine.episode["target"][9])
        self.assertGreater(self.engine._sequence, samples)
        self.assertFalse(self.engine.log_handle.closed)
        self.assertEqual(self.engine.last_run["operation"], "single_frame")
        self.engine.submit("start", dict(hz=100, align_seconds=0))
        wait_for(lambda: self.engine.phase == "completed")
        np.testing.assert_array_equal(np.array([v for _,v in self.robot.sent[count:]]),self.engine.episode["target"][9:])

    def test_single_frame_transition_finishes_at_requested_frame_without_autoplay(self):
        self.ready()
        self.engine.submit("goto_frame", dict(frame=19, align_seconds=.1))
        wait_for(lambda: self.engine.phase == "moving")
        with self.assertRaises(ValueError):
            self.engine.submit("goto_frame", dict(frame=2, align_seconds=0))
        wait_for(lambda: self.engine.phase == "holding")
        count=len(self.robot.sent)
        time.sleep(.07)
        self.assertEqual(len(self.robot.sent),count)
        self.assertEqual(self.engine.sent_frames,1)
        np.testing.assert_array_equal(self.robot.sent[-1][1],self.engine.episode["target"][19])
        self.engine.submit("goto_frame", dict(frame=2, align_seconds=.05))
        wait_for(lambda: self.engine.frame==2 and self.engine.phase == "holding")
        np.testing.assert_array_equal(self.engine.target,self.engine.episode["target"][2])

    def test_single_frame_cancel_and_invalid_frame(self):
        self.ready()
        for frame in (-1,20,1.5,True):
            with self.assertRaises(ValueError):
                self.engine.submit("goto_frame", dict(frame=frame, align_seconds=0))
        self.engine.submit("goto_frame", dict(frame=19, align_seconds=.5))
        wait_for(lambda: self.engine.phase == "moving" and len(self.robot.sent)>2)
        self.engine.submit("disable")
        wait_for(lambda: self.engine.phase == "disabled")
        count=len(self.robot.sent)
        time.sleep(.08)
        self.assertEqual(len(self.robot.sent),count)
        self.assertFalse(self.robot.enabled)

    def test_http_requires_token_and_rejects_cross_origin(self):
        frames = Mock(errors={})
        app = create_app(self.engine, frames)
        client = app.test_client()
        self.assertEqual(client.post("/api/command/connect", json={}).status_code, 403)
        token = app.config["REPLAY_TOKEN"]
        self.assertEqual(client.post("/api/command/connect", json={}, headers={"X-Replay-Token":token,"Origin":"http://evil.example"}).status_code,403)
        self.assertFalse(self.robot.connected)
        self.assertEqual(client.post("/api/command/connect", json={}, headers={"X-Replay-Token":token}).status_code,200)
        wait_for(lambda:self.robot.connected)
        self.assertFalse(self.robot.enabled)
        self.assertEqual(client.get("/api/status?after=-1").status_code,200)

    def test_hardware_joint_conversion_and_stop_order_without_real_device(self):
        robot = PiperRobot.__new__(PiperRobot)
        robot.enabled = robot.enable_attempted = True
        events=[]
        robot.controllers=[]
        for name in ("left","right"):
            c=Mock(can_interface=name)
            c.get_target_joint_angles.return_value=np.array([10,20,30,40,50,60],float)
            c.get_gripper_open_value.return_value=.7
            c.latch_emergency_stop.side_effect=lambda name=name:(events.append(name+" latch") or True)
            c.finish_emergency_disable.side_effect=lambda name=name:(events.append(name+" disable") or True)
            robot.controllers.append(c)
        target=np.arange(14,dtype=float)/10
        robot.send(target)
        np.testing.assert_allclose(robot.controllers[1].set_target_joint_angles.call_args.args[0],np.rad2deg(target[7:13]))
        self.assertEqual(robot.controllers[0].set_gripper_open_value.call_args.args[0],target[6])
        robot.disable()
        self.assertEqual(events,["left latch","right latch","left disable","right disable"])


class OriginalDataTests(unittest.TestCase):
    def test_original_field_units_and_auxiliary_preservation(self):
        with tempfile.TemporaryDirectory() as directory:
            project=Path(directory);root=project/'datasets/test'
            (root/'meta').mkdir(parents=True);(root/'data').mkdir()
            manifest=project/'resources/piper/dataset_v3/meta';manifest.mkdir(parents=True)
            spec=dict(root='datasets/test',state_key='state',action_key='actions',joint_unit='degrees',cameras=['a','b','c'])
            (manifest/'piper_interface.json').write_text(json.dumps(dict(sources=[spec])))
            (root/'meta/info.json').write_text(json.dumps(dict(fps=30,chunks_size=1000,data_path='data/{episode_index}.parquet',video_path='{video_key}.mp4')))
            (root/'meta/episodes.jsonl').write_text(json.dumps(dict(episode_index=3,length=2,tasks=['test'])))
            raw=np.ones((2,15))*90;raw[:,[6,13]]=.4;raw[:,14]=7
            pq.write_table(pa.table(dict(state=raw.tolist(),actions=(raw*.5).tolist(),frame_index=[0,1],timestamp=[0.,1/30])),root/'data/3.parquet')
            for name in ('a','b','c'):(root/f'{name}.mp4').touch()
            data=Catalog(project).load('test',3,'action')
            self.assertAlmostEqual(data['target'][0,0],np.pi/4)
            self.assertEqual(data['target'][0,6],.2)
            self.assertEqual(data['auxiliary'][0],3.5)
            self.assertEqual(data['target'].shape,(2,14))

    def test_original_three_video_views_can_seek_forward_and_back(self):
        root=Path('/home/midea/phoenix-mwa')
        if not root.exists():self.skipTest('Local recordings unavailable')
        episode=Catalog(root).load('piper_set_placemat_lerobot',0)
        for path in episode['videos']:
            reader=RecordingReader(path,30)
            try:
                first=reader.get(0).copy()
                self.assertEqual(reader.get(50).shape,(480,640,3))
                np.testing.assert_array_equal(reader.get(0),first)
            finally:reader.close()


if __name__ == '__main__':
    unittest.main()
