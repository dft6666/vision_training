#!/usr/bin/env python3
"""Standard-library unit tests for log auditing; not video recognition tests."""
import copy
import importlib.util
import math
import sys
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / 'tools' / 'finalize_task3.py'
spec = importlib.util.spec_from_file_location('finalize_task3', SCRIPT)
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def fixture():
    source = dict(decoded_frames=16, fps=10.0, width=320, height=240)
    rows = []
    for frame in range(16):
        row = {field: '' for field in audit.FIELDS}
        row.update(frame=str(frame), video_time_s=f'{frame/10:.6f}',
                   r_state='detected', r_reason='OK', valid_candidates='2',
                   target_id='1' if frame < 12 else '2',
                   track_state='detected', event='TRACKED', reason='MATCHED_EXISTING_ID',
                   lost_frames='0', lock_active='1', selected_index=str(frame % 2),
                   r_x='160.000000', r_y='120.000000', target_x='200.000000',
                   target_y='120.000000', target_radius='8.000000',
                   dx_img='40.000000', dy_img='0.000000', distance_px='40.000000',
                   angle_rad='0.000000', angle_deg='0.000000', angle_error_deg='0.000000')
        if frame in (0, 12):
            row.update(event='ACQUIRED', reason='NEW_TRACK_SEGMENT', angle_error_deg='')
        elif frame == 3:
            row['event'] = 'REACQUIRED'
        elif frame == 2 or 4 <= frame <= 11:
            lost = 1 if frame == 2 else frame - 3
            row.update(r_state='lost', r_reason='NO_R_CANDIDATE', valid_candidates='0',
                       track_state='lost', event='LOST' if lost == 1 else 'HOLD',
                       reason='R_LOST', lost_frames=str(lost), selected_index='',
                       r_x='', r_y='', angle_error_deg='')
            for field in audit.OBS_FIELDS:
                row[field] = ''
            if frame == 11:
                row.update(event='RELEASED', lock_active='0')
        rows.append(row)
    return source, rows


class AuditTests(unittest.TestCase):
    def setUp(self):
        self.source, self.rows = fixture()

    def run_rows(self):
        return audit.audit_tracking(self.rows, self.source, 8)

    def test_valid_recovery_release_reacquire(self):
        result = self.run_rows()
        self.assertEqual(result['states']['detected'], 7)
        self.assertEqual(result['states']['lost'], 9)
        self.assertEqual(result['events']['RELEASED'], 1)
        self.assertEqual(result['events']['REACQUIRED'], 1)

    def test_candidate_index_is_not_id(self):
        self.assertGreater(self.run_rows()['index_changes_same_id'], 0)

    def test_truncated_csv(self):
        self.rows.pop()
        with self.assertRaises(audit.AuditError): self.run_rows()

    def test_non_contiguous_frame(self):
        self.rows[5]['frame'] = '50'
        with self.assertRaises(audit.AuditError): self.run_rows()

    def test_stale_position_when_lost(self):
        self.rows[2]['target_x'] = '200'
        with self.assertRaises(audit.AuditError): self.run_rows()

    def test_changed_id_without_release(self):
        self.rows[1]['target_id'] = '2'
        with self.assertRaises(audit.AuditError): self.run_rows()

    def test_wrong_release_boundary(self):
        self.rows[10].update(event='RELEASED', lock_active='0')
        with self.assertRaises(audit.AuditError): self.run_rows()

    def test_wrong_relative_center(self):
        self.rows[1]['dx_img'] = '20'
        with self.assertRaises(audit.AuditError): self.run_rows()

    def test_invalid_numeric_value(self):
        self.rows[1]['angle_rad'] = 'nan'
        with self.assertRaises(audit.AuditError): self.run_rows()

    def test_playback_timestamp_mismatch(self):
        self.rows[1]['video_time_s'] = '0.25'
        with self.assertRaises(audit.AuditError): self.run_rows()

    def test_video_frame_count_mismatch(self):
        a = dict(width=320, height=240, decoded_frames=10)
        b = dict(path='out.mp4', width=320, height=240, decoded_frames=9)
        with self.assertRaises(audit.AuditError): audit.compare_video(a, b)

    def test_same_fps_different_timeline_rejected(self):
        a = dict(width=320, height=240, decoded_frames=3, fps=10,
                 constant_frame_rate=True, timestamp_tolerance=0.0001,
                 normalized_timestamps=[0, .1, .2])
        b = dict(a, path='out.mp4', normalized_timestamps=[0, .1, .21])
        with self.assertRaises(audit.AuditError): audit.compare_video(a, b)

    def test_vfr_not_silently_accepted(self):
        a = dict(width=320, height=240, decoded_frames=3, fps=10,
                 constant_frame_rate=False, timestamp_tolerance=0.0001,
                 normalized_timestamps=[0, .1, .21])
        b = dict(a, path='out.mp4')
        with self.assertRaises(audit.AuditError): audit.compare_video(a, b)

    def test_document_draft_does_not_claim_accuracy(self):
        text = audit.build_report([dict(name='task_3',scene='小能量机关',status='FAIL')])
        self.assertIn('【待人工填写】', text)
        self.assertIn('不是准确率', text)
        self.assertIn('FAIL', text)


if __name__ == '__main__':
    unittest.main(verbosity=2)
