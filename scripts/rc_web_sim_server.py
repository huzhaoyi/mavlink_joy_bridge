#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""HTTP web UI + UDP MAVLink MIXED_IO_DATA (24 ADC V + GPIO input)."""

import argparse
import json
import os
import subprocess
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

try:
    import yaml
except ImportError:
    yaml = None

try:
    from ament_index_python.packages import get_package_share_directory, get_package_prefix
except ImportError:
    get_package_share_directory = None
    get_package_prefix = None

IO_ADC_COUNT = 24
IO_GPIO_COUNT = 24
ADC_DEFAULT_V = 2.5


def resolve_mavlink_send_bin() -> str:
    if get_package_prefix is not None:
        try:
            prefix = get_package_prefix('sealien_mavlink_joy_bridge')
            candidate = os.path.join(
                prefix, 'lib', 'sealien_mavlink_joy_bridge', 'mavlink_mixed_io_udp_send')
            if os.path.isfile(candidate):
                return candidate
        except Exception:
            pass
    script_dir = os.path.dirname(os.path.abspath(__file__))
    dev_candidate = os.path.normpath(
        os.path.join(script_dir, '..', '..', '..', 'build', 'sealien_mavlink_joy_bridge',
                     'mavlink_mixed_io_udp_send'))
    if os.path.isfile(dev_candidate):
        return dev_candidate
    return 'mavlink_mixed_io_udp_send'


def resolve_web_dir() -> str:
    if get_package_share_directory is not None:
        try:
            share = get_package_share_directory('sealien_mavlink_joy_bridge')
            candidate = os.path.join(share, 'web')
            if os.path.isdir(candidate):
                return candidate
        except Exception:
            pass
    script_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(script_dir, '..', 'web'))


def load_config(path: str) -> dict:
    defaults = {
        'web_host': '0.0.0.0',
        'web_port': 8080,
        'udp_target_host': '127.0.0.1',
        'udp_target_port': 14550,
        'send_rate_hz': 50.0,
    }
    if not path or not os.path.isfile(path):
        return defaults
    if yaml is None:
        print('warning: PyYAML not installed, using defaults')
        return defaults
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    params = data.get('rc_web_sim_server', {}).get('ros__parameters', {})
    defaults.update(params)
    return defaults


def gpio_list_to_mask(gpio_list: list) -> int:
    mask = 0
    for i in range(min(IO_GPIO_COUNT, len(gpio_list))):
        if gpio_list[i]:
            mask |= 1 << i
    return mask & 0xFFFFFFFF


def gpio_mask_to_list(mask: int) -> list:
    return [(mask >> i) & 1 for i in range(IO_GPIO_COUNT)]


class IoSimState:
    def __init__(self, cfg: dict) -> None:
        self.lock = threading.Lock()
        self.adc = [ADC_DEFAULT_V] * IO_ADC_COUNT
        self.gpio = [0] * IO_GPIO_COUNT
        self.auto_send = True
        self.seq = 0
        self.web_host = str(cfg['web_host'])
        self.web_port = int(cfg['web_port'])
        self.udp_target_host = str(cfg['udp_target_host'])
        self.udp_target_port = int(cfg['udp_target_port'])
        self.send_rate_hz = float(cfg['send_rate_hz'])


class IoWebSimServer:
    def __init__(self, cfg: dict, web_dir: str) -> None:
        self.state = IoSimState(cfg)
        self.web_dir = web_dir
        self.mavlink_send_bin = resolve_mavlink_send_bin()
        self.running = True
        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self.send_thread.start()

    def stop(self) -> None:
        self.running = False

    def _send_once(self) -> None:
        with self.state.lock:
            adc = list(self.state.adc)
            gpio_lo = gpio_list_to_mask(self.state.gpio)
            host = self.state.udp_target_host
            port = self.state.udp_target_port
            self.state.seq += 1

        cmd = [self.mavlink_send_bin, host, str(port), str(gpio_lo), '0']
        cmd.extend([str(v) for v in adc])
        subprocess.run(cmd, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _send_loop(self) -> None:
        while self.running:
            auto = False
            rate = 50.0
            with self.state.lock:
                auto = self.state.auto_send
                rate = max(1.0, self.state.send_rate_hz)
            if auto:
                self._send_once()
            time.sleep(1.0 / rate)

    def run_http(self) -> None:
        server_obj = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt: str, *args) -> None:
                pass

            def _send_json(self, code: int, payload: dict) -> None:
                body = json.dumps(payload).encode('utf-8')
                self.send_response(code)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(body)

            def _read_json(self) -> dict:
                length = int(self.headers.get('Content-Length', '0'))
                if length <= 0:
                    return {}
                raw = self.rfile.read(length)
                return json.loads(raw.decode('utf-8'))

            def do_OPTIONS(self) -> None:
                self.send_response(HTTPStatus.NO_CONTENT)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')
                self.end_headers()

            def do_GET(self) -> None:
                path = urlparse(self.path).path
                if path == '/api/config':
                    with server_obj.state.lock:
                        cfg = {
                            'udp_target_host': server_obj.state.udp_target_host,
                            'udp_target_port': server_obj.state.udp_target_port,
                            'send_rate_hz': server_obj.state.send_rate_hz,
                            'auto_send': server_obj.state.auto_send,
                            'adc': list(server_obj.state.adc),
                            'gpio': list(server_obj.state.gpio),
                        }
                    self._send_json(HTTPStatus.OK, cfg)
                    return

                if path in ('/', '/index.html'):
                    file_path = os.path.join(server_obj.web_dir, 'index.html')
                else:
                    rel = path.lstrip('/')
                    file_path = os.path.normpath(os.path.join(server_obj.web_dir, rel))
                    if not file_path.startswith(server_obj.web_dir):
                        self.send_error(HTTPStatus.FORBIDDEN)
                        return

                if not os.path.isfile(file_path):
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return

                ext = os.path.splitext(file_path)[1].lower()
                content_types = {
                    '.html': 'text/html; charset=utf-8',
                    '.css': 'text/css; charset=utf-8',
                    '.js': 'application/javascript; charset=utf-8',
                }
                with open(file_path, 'rb') as f:
                    data = f.read()
                self.send_response(HTTPStatus.OK)
                self.send_header(
                    'Content-Type',
                    content_types.get(ext, 'application/octet-stream'))
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_POST(self) -> None:
                path = urlparse(self.path).path
                if path in ('/api/io',):
                    body = self._read_json()
                    with server_obj.state.lock:
                        if 'udp_target_host' in body:
                            server_obj.state.udp_target_host = str(body['udp_target_host'])
                        if 'udp_target_port' in body:
                            server_obj.state.udp_target_port = int(body['udp_target_port'])
                        if 'send_rate_hz' in body:
                            server_obj.state.send_rate_hz = float(body['send_rate_hz'])
                        if 'auto_send' in body:
                            server_obj.state.auto_send = bool(body['auto_send'])
                        if 'adc' in body:
                            adc_in = body['adc']
                            for i in range(min(IO_ADC_COUNT, len(adc_in))):
                                val = float(adc_in[i])
                                server_obj.state.adc[i] = max(0.0, min(5.0, val))
                        if 'gpio' in body:
                            gpio_in = body['gpio']
                            for i in range(min(IO_GPIO_COUNT, len(gpio_in))):
                                server_obj.state.gpio[i] = 1 if gpio_in[i] else 0
                    if body.get('send_once', False):
                        server_obj._send_once()
                    self._send_json(HTTPStatus.OK, {'ok': True})
                    return

                if path == '/api/send_once':
                    server_obj._send_once()
                    self._send_json(HTTPStatus.OK, {'ok': True})
                    return

                self.send_error(HTTPStatus.NOT_FOUND)

        server = ThreadingHTTPServer(
            (self.state.web_host, self.state.web_port), Handler)
        server.daemon_threads = True
        print(
            'Mixed IO web simulator: http://%s:%d/ -> MIXED_IO_DATA %s:%d @ %.1f Hz'
            % (
                self.state.web_host,
                self.state.web_port,
                self.state.udp_target_host,
                self.state.udp_target_port,
                self.state.send_rate_hz,
            ))
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            server.server_close()


def main() -> None:
    parser = argparse.ArgumentParser(description='Sealien Mixed IO web MAVLink simulator')
    parser.add_argument('--config', default='', help='yaml config path')
    parser.add_argument('--web-host', default='')
    parser.add_argument('--web-port', type=int, default=0)
    parser.add_argument('--udp-host', default='')
    parser.add_argument('--udp-port', type=int, default=0)
    args = parser.parse_args()

    cfg = load_config(args.config)
    if args.web_host:
        cfg['web_host'] = args.web_host
    if args.web_port > 0:
        cfg['web_port'] = args.web_port
    if args.udp_host:
        cfg['udp_target_host'] = args.udp_host
    if args.udp_port > 0:
        cfg['udp_target_port'] = args.udp_port

    web_dir = resolve_web_dir()
    sim = IoWebSimServer(cfg, web_dir)
    try:
        sim.run_http()
    finally:
        sim.stop()


if __name__ == '__main__':
    main()
