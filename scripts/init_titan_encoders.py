#!/usr/bin/env python3
"""Discover and configure Titan using one persistent ROS client."""
import argparse
import time

import rclpy
from studica_control.srv import SetData


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--wait-only', action='store_true')
    parser.add_argument('--service', action='append')
    parser.add_argument('--timeout', type=float, default=45.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('titan_encoder_init')
    try:
        for service in args.service or ['/titan0/titan_cmd']:
            client = node.create_client(SetData, service)
            deadline = time.monotonic() + args.timeout
            print(f'Menunggu {service} (maksimal {args.timeout:g} detik)...', flush=True)
            while not client.wait_for_service(timeout_sec=min(1.0, max(0.0, deadline - time.monotonic()))):
                if time.monotonic() >= deadline or not rclpy.ok():
                    raise RuntimeError(f'{service} tidak ditemukan; periksa ROS_DOMAIN_ID dan RMW driver.')
            print(f'Service siap: {service}', flush=True)
            if args.wait_only:
                continue
            for motor in range(4):
                for command in ('setup_encoder', 'configure_encoder', 'reset_encoder'):
                    request = SetData.Request()
                    request.params = command
                    request.initparams.n_encoder = motor
                    request.initparams.dist_per_tick = 0.000308604386
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
                    if not future.done():
                        raise RuntimeError(f'{service}: {command} M{motor} timeout')
                    response = future.result()
                    if response is None or not response.success:
                        raise RuntimeError(f'{command} M{motor} gagal: {response}')
                print(f'Encoder M{motor} siap.', flush=True)
        return 0
    except (RuntimeError, KeyboardInterrupt) as error:
        print(f'ERROR: {error}', flush=True)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
