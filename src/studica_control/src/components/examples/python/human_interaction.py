"""Human interaction mode preparation; independent of ROS and hardware.

Only the preparation states are executable until the physical pickup and
slide/rotation calibration have been supplied. No guessed movement is issued.
"""
import json
import math
import os
import tempfile
from pathlib import Path


class HumanInteraction:
    MANUAL = 'MANUAL'
    CAPTURE = 'CAPTURE_DEFAULT'
    READY = 'READY'
    ERROR = 'ERROR'

    def __init__(self, snapshot_path, clock, settle_seconds=0.3,
                 feedback_timeout=0.5, capture_timeout=3.0, slide=None):
        self.slide = dict(slide or {})
        if self.slide:
            if self.slide.get('mode') != 'timed':
                raise ValueError('slide.mode harus timed')
            for direction in ('extend', 'retract'):
                duty = float(self.slide[f'{direction}_duty_percent'])
                duration = float(self.slide[f'{direction}_seconds'])
                if not math.isfinite(duty) or not -100 <= duty <= 100:
                    raise ValueError('duty slide harus -100..100 persen')
                if not math.isfinite(duration) or not 0 <= duration <= 30:
                    raise ValueError('durasi slide harus 0..30 detik')
                self.slide[f'{direction}_duty_percent'] = duty
                self.slide[f'{direction}_seconds'] = duration
        self.path = Path(snapshot_path).expanduser()
        self.clock = clock
        self.settle_seconds = settle_seconds
        self.feedback_timeout = feedback_timeout
        self.capture_timeout = capture_timeout
        self.state = self.MANUAL
        self.message = 'Keyboard manual'
        self.feedback = {}
        self.snapshot = None
        self.started = 0.0
        self.stop_pressed = False
        self.start_pressed = None

    @property
    def owns_control(self):
        return self.state != self.MANUAL

    def observe(self, name, value):
        if math.isfinite(value):
            self.feedback[name] = (float(value), self.clock())

    def capture(self):
        if self.stop_pressed:
            self.message = 'Lepaskan STOP sebelum M'
            return
        # Key repeats must not recapture a prepared mission.
        if self.state not in (self.MANUAL, self.ERROR):
            return
        self.started = self.clock()
        self.state = self.CAPTURE
        self.message = 'Motor berhenti; menunggu feedback baru untuk posisi default'

    def cancel(self):
        self.state = self.MANUAL
        self.message = 'Keyboard manual; default tersimpan tetap ada'

    def button(self, name, pressed):
        if name == 'stop':
            self.stop_pressed = pressed
            if pressed:
                self.cancel()
                self.message = 'STOP fisik aktif'
        else:
            edge = self.start_pressed is False and pressed
            self.start_pressed = pressed
            if edge and self.state == self.READY and not self.stop_pressed:
                # Explicitly inhibit automatic motion until its physical
                # position references and pickup sequence are configured.
                self.message = ('START ditahan: rasio putar, '
                                'target objek dan urutan ambil belum tersedia')

    def tick(self, servo_targets):
        if self.state != self.CAPTURE:
            return
        now = self.clock()
        missing = []
        for name in ('lift', 'rotate'):
            sample = self.feedback.get(name)
            if (sample is None or now - sample[1] > self.feedback_timeout or
                    sample[1] < self.started + self.settle_seconds):
                missing.append(name + ' encoder baru')
        for name in ('wrist', 'gripper'):
            value = servo_targets.get(name)
            if value is None or not math.isfinite(value):
                missing.append(name + ' target (tekan P dahulu)')
        if missing:
            self.message = 'Menunggu: ' + ', '.join(missing)
            if now - self.started >= self.capture_timeout:
                self.state = self.ERROR
            return
        snapshot = {
            'schema_version': 1,
            'lift': {'position': self.feedback['lift'][0],
                     'unit': 'motor_output_revolutions', 'source': 'encoder'},
            'rotate': {'position': self.feedback['rotate'][0],
                       'unit': 'motor_output_revolutions', 'source': 'encoder'},
            'wrist': {'position': servo_targets['wrist'], 'unit': 'degrees',
                      'source': 'commanded_target_not_measured'},
            'gripper': {'position': servo_targets['gripper'], 'unit': 'degrees',
                        'source': 'commanded_target_not_measured'},
            'slide': {'position': None, 'source': 'manual_reference_at_M',
                      'return_accuracy': 'open_loop_not_measured',
                      'timing': self.slide},
            'encoder_reference': 'current_hardware_session_only',
            'restore_gripper_when_holding': False,
        }
        temporary = None
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(mode='w', dir=self.path.parent,
                                             delete=False, encoding='utf-8') as out:
                temporary = out.name
                json.dump(snapshot, out, indent=2, allow_nan=False)
                out.write('\n')
                out.flush()
                os.fsync(out.fileno())
            os.replace(temporary, self.path)
        except (OSError, ValueError) as error:
            self.state = self.ERROR
            self.message = f'Gagal menyimpan default: {error}'
            return
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)
        self.snapshot = snapshot
        self.state = self.READY
        self.message = f'Default OMS tersimpan: {self.path}; menunggu START'
