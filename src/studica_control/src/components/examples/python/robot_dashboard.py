#!/usr/bin/env python3
"""Desktop ROS 2 hardware dashboard. Never starts hardware or motion on launch."""
import argparse
import json
import math
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message, get_service
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from std_msgs.msg import Bool, Float64
from studica_control.srv import SetData

WHEELS = ('Front right', 'Rear right', 'Front left', 'Rear left')


def duty_value(percent, direction, inverted=False):
    value = float(percent)
    if not math.isfinite(value) or not 0 <= value <= 100:
        raise ValueError('Duty must be between 0 and 100 percent')
    if direction not in (-1, 1):
        raise ValueError('Direction must be -1 or 1')
    return value / 100 * direction * (-1 if inverted else 1)


def summarize(message):
    """Bound rendering work for images, scans, point clouds and arrays."""
    if hasattr(message, 'data') and hasattr(message, 'width'):
        return f'{message.width} × {message.height}; {getattr(message, "encoding", "point cloud")}'
    if hasattr(message, 'ranges'):
        valid = [v for v in message.ranges if math.isfinite(v) and message.range_min <= v <= message.range_max]
        return f'{len(message.ranges)} samples; minimum {min(valid):.3f} m' if valid else 'No valid range readings'
    def compact(value):
        if hasattr(value, 'get_fields_and_field_types'):
            return {k: compact(getattr(value, k)) for k in value.get_fields_and_field_types()}
        if isinstance(value, (str, int, float, bool)) or value is None:
            return value
        try:
            return [compact(v) for v in value[:12]] + ([f'… ({len(value)} elements)'] if len(value) > 12 else [])
        except (TypeError, AttributeError):
            return str(value)[:300]
    return json.dumps(compact(message), ensure_ascii=False)[:3000]


class Dashboard:
    def __init__(self, root, node, titan):
        self.root, self.node = root, node
        self.titan = '/' + titan.strip('/')
        self.subscriptions, self.latest, self.clients, self.pending = {}, {}, {}, []
        self.services, self.topic_types = {}, {}
        self.active = None
        self.digital_pubs = {}
        self.armed = tk.BooleanVar(value=False)
        self.status = tk.StringVar(value='Monitoring active. Motor control is disabled.')
        self.pubs = [node.create_publisher(Float64, f'{self.titan}/m_{i}/cmd', 1) for i in range(4)]
        root.title('Studica Robot — Monitoring & Control')
        root.geometry('1150x800')
        root.minsize(900, 650)
        style = ttk.Style(root)
        style.theme_use('clam')
        style.configure('Title.TLabel', font=('Sans', 18, 'bold'))
        top = ttk.Frame(root, padding=12)
        top.pack(fill='x')
        ttk.Label(top, text='STUDICA  /  Robot dashboard', style='Title.TLabel').pack(side='left')
        tk.Button(top, text='STOP ALL', bg='#b91c1c', fg='white', font=('Sans', 12, 'bold'), command=self.emergency_stop).pack(side='right')
        ttk.Label(root, textvariable=self.status, padding=(12, 4), wraplength=1080).pack(fill='x')
        tabs = ttk.Notebook(root)
        tabs.pack(fill='both', expand=True, padx=12, pady=12)
        self.motor_tab(tabs)
        self.sensor_tab(tabs)
        self.control_tab(tabs)
        self.service_tab(tabs)
        self.log = tk.Text(root, height=5, state='disabled', font=('Monospace', 9))
        self.log.pack(fill='x', padx=12, pady=(0, 10))
        root.bind_all('<ButtonRelease-1>', lambda event: self.stop_motion())
        root.bind('<FocusOut>', self.focus_out)
        root.bind('<Escape>', lambda event: self.emergency_stop())
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.next_discovery = 0
        self.tick()

    def record(self, text):
        self.status.set(text)
        self.log.configure(state='normal')
        self.log.insert('end', time.strftime('%H:%M:%S ') + text + '\n')
        if int(self.log.index('end-1c').split('.')[0]) > 100:
            self.log.delete('1.0', '2.0')
        self.log.see('end')
        self.log.configure(state='disabled')

    def motor_tab(self, tabs):
        frame = ttk.Frame(tabs, padding=12)
        tabs.add(frame, text='Motors & Encoders')
        ttk.Label(frame, text=f'Controller: {self.titan}  •  Hold CW/CCW to move; release to stop.').pack(anchor='w')
        ttk.Label(frame, text='CW = positive duty before inversion. Check direction from the shaft end. Encoder = distance scaled by dist_per_tick.').pack(anchor='w')
        ttk.Checkbutton(frame, text='Enable manual control (stop other teleop / navigation controllers first)', variable=self.armed, command=self.arm_changed).pack(anchor='w', pady=10)
        grid = ttk.Frame(frame)
        grid.pack(fill='both', expand=True)
        self.feedback, self.duties, self.inverted = [], [], []
        for i, name in enumerate(WHEELS):
            box = ttk.LabelFrame(grid, text=f'M{i} — {name}', padding=12)
            box.grid(row=i // 2, column=i % 2, sticky='nsew', padx=5, pady=5)
            grid.columnconfigure(i % 2, weight=1)
            grid.rowconfigure(i // 2, weight=1)
            duty = tk.DoubleVar(value=10)
            inverted = tk.BooleanVar(value=False)
            self.duties.append(duty)
            self.inverted.append(inverted)
            ttk.Label(box, text='Duty (%)').pack(anchor='w')
            ttk.Spinbox(box, from_=0, to=100, increment=1, textvariable=duty, width=8).pack(anchor='w')
            ttk.Checkbutton(box, text='Invert CW/CCW direction', variable=inverted).pack(anchor='w')
            buttons = ttk.Frame(box)
            buttons.pack(fill='x', pady=8)
            for label, direction in [('↻ CW', 1), ('↺ CCW', -1)]:
                button = ttk.Button(buttons, text=label)
                button.pack(side='left', padx=3)
                button.bind('<ButtonPress-1>', lambda event, m=i, d=direction: self.start_motion(m, d))
            ttk.Button(buttons, text='Stop', command=self.stop_motion).pack(side='left', padx=3)
            ttk.Button(buttons, text='Reset encoder', command=lambda m=i: self.reset_encoder(m)).pack(side='left', padx=3)
            value = tk.StringVar(value='Waiting for encoder / RPM…')
            self.feedback.append(value)
            ttk.Label(box, textvariable=value, justify='left').pack(anchor='w')
            for field in ('encoder', 'rpm', 'angle', 'limit_fwd', 'limit_rev'):
                topic = f'{self.titan}/m_{i}/{field}'
                typ = 'std_msgs/msg/Bool' if field.startswith('limit') else 'std_msgs/msg/Float64'
                self.subscribe(topic, typ)

    def sensor_tab(self, tabs):
        frame = ttk.Frame(tabs, padding=10)
        tabs.add(frame, text='All Sensors')
        ttk.Label(frame, text='Topics discovered automatically • data older than 2 seconds marked STALE • images/point clouds shown as metadata.').pack(anchor='w')
        self.tree = ttk.Treeview(frame, columns=('type', 'value', 'age'), show='tree headings', height=16)
        self.tree.heading('#0', text='Topic')
        for key, label, width in [('type', 'Type', 180), ('value', 'Value', 480), ('age', 'Age', 90)]:
            self.tree.heading(key, text=label)
            self.tree.column(key, width=width)
        self.tree.column('#0', width=240)
        scroll = ttk.Scrollbar(frame, orient='vertical', command=self.tree.yview)
        self.tree.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right', fill='y')
        self.tree.pack(fill='both', expand=True)

    def control_tab(self, tabs):
        frame = ttk.Frame(tabs, padding=16)
        tabs.add(frame, text='Servo / DIO / Lights')
        outputs = ttk.LabelFrame(frame, text='Robot lights & buzzer (DIO)', padding=12)
        outputs.pack(fill='x', pady=8)
        for row, (name, label) in enumerate([
            ('light_control', 'Light control — pin 12'),
            ('light_red', 'Red — pin 13'),
            ('light_green', 'Green — pin 14'),
            ('light_yellow', 'Yellow — pin 15'),
            ('buzzer', 'Buzzer — pin 9'),
        ]):
            ttk.Label(outputs, text=label).grid(row=row, column=0, sticky='w', padx=8, pady=3)
            for col, (text, state) in enumerate([('ON / HIGH', True), ('OFF / LOW', False)], 1):
                ttk.Button(outputs, text=text, command=lambda n=name, v=state: self.digital_write(n, v)).grid(row=row, column=col, padx=6)
            feedback = tk.StringVar(value='Waiting for data')
            ttk.Label(outputs, textvariable=feedback).grid(row=row, column=3, padx=8)
            self.digital_pubs[name] = (self.node.create_publisher(Bool, f'/{name}/cmd', 1), feedback)
            self.subscribe(f'/{name}/state', 'std_msgs/msg/Bool')
        self.quick = {}
        for row, (key, label, suffix, values) in enumerate([
            ('servo', 'Servo — angle / speed according to configuration', '/set_servo', ['0', '45', '90', '135', '180']),
            ('dio', 'Digital output', '/dio_cmd', ['toggle']),
            ('light', 'Light tower', '/set', ['off', 'red', 'green', 'yellow', 'buzzer', 'red:blink', 'green:blink', 'yellow:blink'])]):
            box = ttk.LabelFrame(frame, text=label, padding=15)
            box.pack(fill='x', pady=12)
            target = ttk.Combobox(box, state='readonly', width=40)
            target.pack(side='left', padx=6)
            value = ttk.Combobox(box, values=values, width=16)
            value.set('90' if key == 'servo' else values[0])
            value.pack(side='left', padx=6)
            self.quick[key] = (target, value, suffix)
            ttk.Button(box, text='Send', command=lambda k=key: self.quick_call(k)).pack(side='left', padx=6)
        ttk.Label(frame, text='Devices are listed from active ROS services. Command responses appear in the log below.').pack(anchor='w')

    def service_tab(self, tabs):
        frame = ttk.Frame(tabs, padding=12)
        tabs.add(frame, text='Advanced Services')
        ttk.Label(frame, text='Select a service to load its JSON request. Includes Titan, IMU, encoder and camera configuration.').pack(anchor='w')
        self.service_choice = ttk.Combobox(frame, state='readonly', width=90)
        self.service_choice.pack(fill='x', pady=8)
        self.service_choice.bind('<<ComboboxSelected>>', self.load_request)
        self.request = tk.Text(frame, height=15, font=('Monospace', 11))
        self.request.pack(fill='both', expand=True)
        ttk.Button(frame, text='Call service', command=self.run_service).pack(anchor='e', pady=10)

    def subscribe(self, topic, typ):
        if topic in self.subscriptions:
            return
        try:
            self.subscriptions[topic] = self.node.create_subscription(get_message(typ), topic, lambda msg, t=topic: self.latest.__setitem__(t, (msg, time.monotonic())), qos_profile_sensor_data)
            self.topic_types[topic] = typ
        except (ImportError, AttributeError, ValueError) as error:
            self.node.get_logger().debug(f'{topic}: {error}')

    def discover(self):
        for topic, types in self.node.get_topic_names_and_types():
            if types and not topic.endswith('/cmd') and topic not in ('/rosout', '/parameter_events'):
                self.subscribe(topic, types[0])
        self.services = {name: types[0] for name, types in self.node.get_service_names_and_types() if types}
        self.service_choice.configure(values=sorted(self.services))
        for target, value, suffix in self.quick.values():
            choices = sorted(name for name, typ in self.services.items() if name.endswith(suffix) and typ == 'studica_control/srv/SetData')
            target.configure(values=choices)
            if not target.get() and choices:
                target.set(choices[0])

    def call(self, name, typ, values):
        try:
            if not name:
                raise ValueError('Select a device/service first')
            cls = get_service(typ)
            request = cls.Request()
            set_message_fields(request, values)
            if isinstance(request, SetData.Request) and not 0 <= request.initparams.n_encoder <= 3:
                raise ValueError('Motor/encoder channel must be 0–3')
            client = self.clients.get((name, typ))
            if client is None:
                client = self.node.create_client(cls, name)
                self.clients[(name, typ)] = client
            if not client.service_is_ready():
                raise ValueError(f'Service unavailable: {name}')
            self.pending.append((client.call_async(request), time.monotonic() + 5, name))
            self.record(f'Command sent: {name}')
        except Exception as error:
            self.record(f'Failed: {error}')

    def titan_call(self, command, motor=0):
        self.call(f'{self.titan}/titan_cmd', 'studica_control/srv/SetData', {'params': command, 'initparams': {'n_encoder': motor}})

    def reset_encoder(self, motor):
        self.stop_motion()
        self.titan_call('reset_encoder', motor)

    def arm_changed(self):
        self.stop_motion()
        self.record('Manual control enabled.' if self.armed.get() else 'Manual control disabled.')

    def start_motion(self, motor, direction):
        self.stop_motion()
        if not self.armed.get():
            self.record('Enable manual control first.')
            return
        if self.pubs[motor].get_subscription_count() == 0:
            self.record('Motor controller is not connected.')
            return
        topic = f'{self.titan}/m_{motor}/cmd'
        others = [p.node_name for p in self.node.get_publishers_info_by_topic(topic) if p.node_name != self.node.get_name()]
        if others:
            self.record('Stop other motor publishers: ' + ', '.join(others))
            return
        try:
            duty = duty_value(self.duties[motor].get(), direction, self.inverted[motor].get())
        except (ValueError, tk.TclError) as error:
            self.record(str(error))
            return
        self.active = (motor, duty, time.monotonic() + 3)
        self.pubs[motor].publish(Float64(data=duty))
        self.record(f'M{motor} {WHEELS[motor]}: duty {duty:+.2f}; maximum 3 seconds per press.')

    def stop_motion(self):
        if self.active is not None:
            motor = self.active[0]
            self.active = None
            self.pubs[motor].publish(Float64(data=0.0))
            self.titan_call('stop', motor)

    def emergency_stop(self):
        self.stop_motion()
        self.armed.set(False)
        for pub in self.pubs:
            pub.publish(Float64(data=0.0))
        self.titan_call('disable')
        self.record('STOP sent; check the disable response in the log. Re-enable through the Titan service.')

    def focus_out(self, event):
        self.root.after_idle(lambda: self.stop_motion() if self.root.focus_displayof() is None else None)

    def digital_write(self, name, state):
        pub, _ = self.digital_pubs[name]
        if pub.get_subscription_count() == 0:
            self.record(f'Output /{name}/cmd is unavailable; check the hardware configuration.')
            return
        pub.publish(Bool(data=state))
        self.record(f'{name}: {"HIGH" if state else "LOW"} sent; check the pin feedback.')

    def quick_call(self, key):
        target, value, _ = self.quick[key]
        try:
            if key == 'servo':
                speed = float(value.get())
                if not math.isfinite(speed):
                    raise ValueError('Servo value must be finite')
                values = {'initparams': {'speed': speed}}
            else:
                values = {'params': value.get()}
            self.call(target.get(), 'studica_control/srv/SetData', values)
        except ValueError as error:
            self.record(str(error))

    def load_request(self, event=None):
        try:
            request = get_service(self.services[self.service_choice.get()]).Request()
            self.request.delete('1.0', 'end')
            self.request.insert('1.0', json.dumps(message_to_ordereddict(request), indent=2))
        except Exception as error:
            self.record(f'Unable to load service: {error}')

    def run_service(self):
        try:
            name = self.service_choice.get()
            values = json.loads(self.request.get('1.0', 'end'))
            if not isinstance(values, dict):
                raise ValueError('Request must be a JSON object')
            self.stop_motion()
            self.call(name, self.services[name], values)
        except (ValueError, KeyError) as error:
            self.record(f'Invalid request: {error}')

    def tick(self):
        try:
            for _ in range(30):
                rclpy.spin_once(self.node, timeout_sec=0)
            now = time.monotonic()
            if self.active:
                motor, duty, deadline = self.active
                if now >= deadline:
                    self.stop_motion()
                else:
                    self.pubs[motor].publish(Float64(data=duty))
            if now >= self.next_discovery:
                self.discover()
                self.next_discovery = now + 2
            for item in self.pending[:]:
                future, deadline, name = item
                if future.done():
                    try:
                        self.record(f'{name}: {summarize(future.result())}')
                    except Exception as error:
                        self.record(f'{name}: {error}')
                    self.pending.remove(item)
                elif now > deadline:
                    future.cancel()
                    self.pending.remove(item)
                    self.record(f'{name}: timeout; device status is unconfirmed')
            for name, (_, feedback) in self.digital_pubs.items():
                data = self.latest.get(f'/{name}/state')
                if data:
                    feedback.set(('HIGH' if data[0].data else 'LOW') + (' STALE' if now - data[1] > 2 else ''))
            for i, value in enumerate(self.feedback):
                lines = []
                for field in ('encoder', 'rpm', 'angle', 'limit_fwd', 'limit_rev'):
                    data = self.latest.get(f'{self.titan}/m_{i}/{field}')
                    if data:
                        msg, stamp = data
                        lines.append(f'{field}: {msg.data:.4f}' + ('  STALE' if now - stamp > 2 else ''))
                value.set('\n'.join(lines) or 'Waiting for encoder / RPM…')
            for topic, typ in sorted(self.topic_types.items()):
                data = self.latest.get(topic)
                values = (typ, summarize(data[0]) if data else 'Waiting for data', (f'{now-data[1]:.1f}s' + (' STALE' if now-data[1] > 2 else '')) if data else '—')
                if self.tree.exists(topic):
                    self.tree.item(topic, values=values)
                else:
                    self.tree.insert('', 'end', iid=topic, text=topic, values=values)
        except Exception as error:
            self.stop_motion()
            self.record(f'Dashboard error: {error}')
        self.root.after(50, self.tick)

    def close(self):
        self.stop_motion()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--titan', default='titan0', help='Titan controller name / namespace')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Node('robot_dashboard')
    try:
        root = tk.Tk()
        app = Dashboard(root, node, args.titan)
        try:
            root.mainloop()
        finally:
            app.stop_motion()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
