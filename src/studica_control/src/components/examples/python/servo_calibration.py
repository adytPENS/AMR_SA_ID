#!/usr/bin/env python3
"""Interactive calibration of commanded standard-servo angles, not feedback."""
import argparse
import os
import select
import sys
import tempfile
import termios
import time
import tty
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Float64
from titan_keyboard_teleop import ServoServiceCommand, StandardServoJog
from studica_control.srv import SetData


JOG_KEYS = {'r': ('wrist', 1), 't': ('wrist', -1),
            'y': ('eof', 1), 'u': ('eof', -1)}
MARK_KEYS = {'1': ('wrist', 'left_value'), '2': ('wrist', 'right_value'),
             '3': ('eof', 'left_value'), '4': ('eof', 'right_value')}


def save_positions(path, marks):
    """Preserve unedited settings; replace the file atomically."""
    if not marks:
        raise ValueError('Belum ada posisi ditandai dengan 1/2/3/4')
    path = Path(path)
    config = yaml.safe_load(path.read_text()) if path.exists() else {}
    if config is None:
        config = {}
    if not isinstance(config, dict):
        raise ValueError('YAML harus mapping')
    for (name, side), value in marks.items():
        if not -150 <= value <= 150:
            raise ValueError('Sudut harus -150..150 derajat')
        config.setdefault(name, {})[side] = float(value)
    for name in ('wrist', 'eof'):
        for side in ('left_value', 'right_value'):
            value = float(config.get(name, {}).get(side, float('nan')))
            if not -150 <= value <= 150:
                raise ValueError(f'Posisi {name}.{side} belum valid; tandai dahulu')
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent,
                                         delete=False, encoding='utf-8') as stream:
            temporary = stream.name
            stream.write('# Target sudut derajat: R/T wrist, Y/U EoF.\n')
            yaml.safe_dump(config, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)


class Calibration(Node):
    def __init__(self, args):
        super().__init__('servo_calibration')
        self.positions = {name: StandardServoJog(initial=getattr(args, name + '_start'))
                          for name in ('wrist', 'eof')}
        self.senders = {}
        for name, sensor in (('wrist', args.wrist_sensor), ('eof', args.eof_sensor)):
            self.senders[name] = ServoServiceCommand(
                self.create_client(SetData, f'/{sensor}/set_servo'), self.get_logger(), name)
            self.create_subscription(Float64, f'/{sensor}/state',
                                     lambda msg, name=name: self.positions[name].observe(msg.data), 10)
        self.marks = {}
        self.step = args.step
        self.path = args.config

    def key(self, key):
        if key in JOG_KEYS:
            name, sign = JOG_KEYS[key]
            position = self.positions[name]
            if position.target is None:
                position.initialize()
                print(f'{name}: posisi awal {position.target:g}° (target perintah)')
            else:
                position.target = max(-150, min(150, position.target + sign * self.step))
            self.senders[name].set_target(position.target)
            print(f'{name}: target {position.target:g}°; langkah {self.step:g}°')
        elif key in MARK_KEYS:
            name, side = MARK_KEYS[key]
            sender = self.senders[name]
            target = self.positions[name].target
            if target is None or sender.confirmed != target:
                print('Gerakkan lewat tombol servo lalu tunggu konfirmasi service dahulu.')
                return
            self.marks[name, side] = target
            print(f'Ditandai {name}.{side} = {target:g}°; S untuk simpan file')
        elif key in ('+', '='):
            self.step = min(10, self.step + 1)
            print(f'Langkah {self.step:g}°')
        elif key == '-':
            self.step = max(1, self.step - 1)
            print(f'Langkah {self.step:g}°')
        elif key == 's':
            try:
                save_positions(self.path, self.marks)
                print(f'Tersimpan: {self.path}. Restart keyboard untuk memakai nilai baru.')
            except (OSError, ValueError, TypeError, yaml.YAMLError) as error:
                print(f'Gagal menyimpan: {error}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, help='YAML keyboard_servos.yaml')
    parser.add_argument('--step', type=int, choices=range(1, 11), default=1)
    parser.add_argument('--wrist-sensor', default='oms_wrist')
    parser.add_argument('--eof-sensor', default='oms_gripper')
    parser.add_argument('--wrist-start', type=int, choices=range(-150, 151), default=0)
    parser.add_argument('--eof-start', type=int, choices=range(-150, 151), default=0)
    args = parser.parse_args()
    if not sys.stdin.isatty():
        parser.error('Gunakan terminal interaktif')
    rclpy.init()
    node = Calibration(args)
    terminal = termios.tcgetattr(sys.stdin)
    print('R/T wrist +/- | Y/U EoF +/- | +/- ubah langkah (awal 1°)')
    print('1 wrist kiri | 2 wrist kanan | 3 EoF kiri | 4 EoF kanan')
    print('S simpan YAML | Q keluar; keluar mempertahankan sudut servo, bukan melepas daya.')
    print('Tidak bergerak saat startup. Tombol servo pertama memakai sudut awal jika belum ada state.')
    print('Jangan jalankan kontrol servo lain bersamaan. Tunggu posisi fisik tepat sebelum tandai.')
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            readable, _, _ = select.select([sys.stdin], [], [], 0.01)
            if readable:
                key = sys.stdin.read(1).lower()
                if key == 'q':
                    break
                node.key(key)
            for sender in node.senders.values():
                sender.pump(time.monotonic())
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
