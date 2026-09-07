#!/usr/bin/env python3
"""Navigator khusus full trace dinding kanan dengan belok kiri otomatis."""

# Infrastruktur keselamatan, tombol, lampu, odometri, dan LiDAR digunakan
# bersama hybrid corridor. Perilaku full right-wall diaktifkan hanya oleh
# right_wall_waypoints.yaml melalui motion.auto_corner_turn.
from hybrid_corridor_navigator import main


if __name__ == '__main__':
    main()
