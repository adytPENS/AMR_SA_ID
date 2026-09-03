#!/usr/bin/env python3
"""Tampilkan occupancy map dan baca koordinat dunia dari klik mouse."""

import argparse
from pathlib import Path
import tkinter as tk

from PIL import Image, ImageTk
import yaml


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('map_yaml', nargs='?',
                        default='/home/vmx/studica_ws/maps/Navigation.yaml')
    args = parser.parse_args()
    yaml_path = Path(args.map_yaml).resolve()
    with yaml_path.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    image_path = (yaml_path.parent / config['image']).resolve()
    source = Image.open(image_path).convert('RGB')
    resolution = float(config['resolution'])
    origin_x, origin_y, _ = map(float, config['origin'])
    scale = max(4, min(8, 700 // max(source.width, source.height)))
    nearest = getattr(getattr(Image, 'Resampling', Image), 'NEAREST')
    shown = source.resize((source.width * scale, source.height * scale), nearest)

    left, top, right, bottom = 75, 25, 25, 60
    root = tk.Tk()
    root.title(f'Koordinat Map - {yaml_path.name}')
    canvas = tk.Canvas(root, width=left + shown.width + right,
                       height=top + shown.height + bottom,
                       background='#dddddd')
    canvas.pack()
    photo = ImageTk.PhotoImage(shown)
    canvas.create_image(left, top, image=photo, anchor='nw')
    max_x = origin_x + source.width * resolution
    max_y = origin_y + source.height * resolution

    value = origin_x
    while value <= max_x + 1e-9:
        px = left + (value - origin_x) / resolution * scale
        canvas.create_line(px, top, px, top + shown.height, fill='#5599cc')
        canvas.create_text(px, top + shown.height + 18,
                           text=f'{value:.2f}', angle=45, anchor='n')
        value += 0.25
    value = origin_y
    while value <= max_y + 1e-9:
        py = top + shown.height - (value - origin_y) / resolution * scale
        canvas.create_line(left, py, left + shown.width, py, fill='#5599cc')
        canvas.create_text(left - 8, py, text=f'{value:.2f}', anchor='e')
        value += 0.25

    canvas.create_text(left + shown.width / 2, top + shown.height + 48,
                       text='X (meter) — klik kiri untuk menandai titik')
    canvas.create_text(15, top + shown.height / 2,
                       text='Y (meter)', angle=90)
    status = tk.StringVar(value='Klik lokasi waypoint pada peta')
    tk.Label(root, textvariable=status, font=('Sans', 12, 'bold')).pack(pady=5)
    point_number = 0

    def clicked(event) -> None:
        nonlocal point_number
        ix, iy = event.x - left, event.y - top
        if not (0 <= ix < shown.width and 0 <= iy < shown.height):
            return
        x = origin_x + (ix / scale) * resolution
        y = origin_y + ((shown.height - iy) / scale) * resolution
        point_number += 1
        label = f'P{point_number}: ({x:.2f}, {y:.2f})'
        canvas.create_oval(event.x - 5, event.y - 5, event.x + 5, event.y + 5,
                           fill='red', outline='white', width=2)
        canvas.create_text(event.x + 7, event.y - 7, text=label,
                           fill='red', anchor='sw', font=('Sans', 9, 'bold'))
        status.set(label)
        print(f'P{point_number}: x={x:.3f}, y={y:.3f}', flush=True)

    canvas.bind('<Button-1>', clicked)
    print(f'Batas X: {origin_x:.2f} .. {max_x:.2f}')
    print(f'Batas Y: {origin_y:.2f} .. {max_y:.2f}')
    root.mainloop()


if __name__ == '__main__':
    main()
