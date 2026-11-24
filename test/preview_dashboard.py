#!/usr/bin/env python3
"""Local UI fixture: synthetic maps and a fake WebSocket, never ROS or hardware.

python test/preview_dashboard.py --port 18780
"""
import argparse
import copy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import threading
from http.server import ThreadingHTTPServer

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from ugv01_room_explore.map_gallery import list_map_cards, map_name, map_preview

spec = importlib.util.spec_from_file_location('dashboard_server_preview', ROOT / 'web/server.py')
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)

WIDTH, HEIGHT = 120, 88
floor = set()
for x0, y0, x1, y1 in [(12, 14, 43, 39), (35, 26, 98, 42), (78, 32, 109, 76)]:
    floor.update((x, y) for x in range(x0, x1) for y in range(y0, y1))
walls = {(x + dx, y + dy) for x, y in floor for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]} - floor
walls.update((x, y) for x in range(86, 90) for y in range(58, 62))
cells = [100 if (x, y) in walls else 0 if (x, y) in floor else -1 for y in range(HEIGHT) for x in range(WIDTH)]
GRID = {'header': {'frame_id': 'map'}, 'info': {'width': WIDTH, 'height': HEIGHT, 'resolution': .05,
        'origin': {'position': {'x': -3., 'y': -2., 'z': 0.}, 'orientation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}}, 'data': cells}
STATE = {'ok': True, 'mode': 'JOY', 'managed': True, 'map': None, 'stack_mode': 'nav_slam',
         'stack_running': True, 'auto_running': False, 'error': '',
         'navigation': {'stack_mode': 'nav_slam', 'drive_mode': 'JOY', 'actual_mode': 'JOY', 'ready': True,
           'reason': '', 'pose': {'x': -1.2, 'y': -.45, 'yaw': 0.}, 'map_available': True,
           'goal_state': 'idle', 'goal': None, 'distance_remaining': None, 'error': '', 'lifecycle': {'amcl': False}}}
COMMANDS = []


def write_map(folder, name):
    name = map_name(name)
    target = folder / name
    if target.with_suffix('.yaml').exists():
        raise ValueError('Карта с таким именем уже есть; выберите другое имя')
    pixels = bytes(0 if cells[y * WIDTH + x] == 100 else 254 if cells[y * WIDTH + x] == 0 else 205
                   for y in range(HEIGHT - 1, -1, -1) for x in range(WIDTH))
    target.with_suffix('.pgm').write_bytes(f'P5\n{WIDTH} {HEIGHT}\n255\n'.encode() + pixels)
    target.with_suffix('.yaml').write_text(f'image: {name}.pgm\nresolution: 0.05\nfree_thresh: 0.196\noccupied_thresh: 0.65\n', encoding='utf-8')
    return {'ok': True, 'map': name}


class Bridge:
    def read(self): return copy.deepcopy(STATE['navigation'])

    def invoke(self, operation, **args):
        if operation == 'goal':
            STATE['navigation'].update(goal=args, goal_state='navigating', distance_remaining=2.1)
            STATE['mode'] = STATE['navigation']['drive_mode'] = STATE['navigation']['actual_mode'] = 'NAV'
        elif operation == 'initialpose':
            STATE['navigation']['pose'] = args
        return {'ok': True}


class Supervisor:
    def __init__(self, directory):
        self.directory = directory
        self.lock = threading.RLock()
    def maps(self): return [p.stem for p in self.directory.glob('*.yaml')]
    def map_items(self): return list_map_cards(self.directory)
    def map_preview(self, name): return map_preview(self.directory, name)
    def save_map(self, name): return write_map(self.directory, name)
    def status(self): return {k: v for k, v in STATE.items() if k not in ('navigation', 'mode', 'ok')}
    def stop_auto(self): pass
    def stop(self):
        STATE['mode'] = STATE['navigation']['drive_mode'] = STATE['navigation']['actual_mode'] = 'JOY'
        STATE['navigation']['goal_state'] = 'canceled'
    def drive(self, mode):
        self.stop(); STATE['mode'] = STATE['navigation']['drive_mode'] = mode
    def switch(self, mode, selected_map=None):
        self.stop()
        if mode == 'nav_map':
            if map_name(selected_map) not in self.maps(): raise ValueError('Карта не найдена')
        STATE['map'] = selected_map if mode == 'nav_map' else None
        STATE['stack_mode'] = STATE['navigation']['stack_mode'] = mode
        STATE['navigation']['ready'] = mode != 'mapping'
        STATE['navigation']['lifecycle']['amcl'] = mode == 'nav_map'


FAKE_WEBSOCKET = '''<script>
window.WebSocket=class PreviewSocket{
 static OPEN=1;
 constructor(){this.readyState=0;setTimeout(()=>{this.readyState=1;this.onopen?.();this.tick();this.timer=setInterval(()=>this.tick(),750)},40)}
 send(text){const m=JSON.parse(text);if(m.op==='publish'&&m.topic==='/cmd_vel_joy'&&(m.msg.linear.x||m.msg.angular.z))fetch('/preview/commands',{method:'POST',body:text})}
 close(){clearInterval(this.timer);this.readyState=3}
 async tick(){try{const s=await(await fetch('/preview/state')).json();
 const emit=(topic,msg)=>this.onmessage?.({data:JSON.stringify({op:'publish',topic,msg})});
 emit('/map',s.grid);emit('/global_costmap/costmap',s.costmap);
 const p=s.state.navigation.pose;emit('/ugv01/robot_pose',{pose:{position:{x:p.x,y:p.y},orientation:{x:0,y:0,z:Math.sin(p.yaw/2),w:Math.cos(p.yaw/2)}}});
 emit('/cmd_vel',{linear:{x:0},angular:{z:0}});
 if(s.state.navigation.goal){const g=s.state.navigation.goal;emit('/plan',{header:{frame_id:'map'},poses:[p,{x:1.55,y:p.y},{x:1.55,y:g.y},g].map(position=>({pose:{position}}))})}
 }catch(e){}}
};</script>'''


class Handler(server.Handler):
    def log_message(self, *args): pass

    def do_GET(self):
        if self.path == '/preview/state':
            costmap = copy.deepcopy(GRID)
            costmap['data'] = [100 if (x, y) in walls else 55 if (x, y) in floor and any(
                (x + dx, y + dy) in walls for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]) else 0
                for y in range(HEIGHT) for x in range(WIDTH)]
            self.send_json(200, {'state': STATE, 'grid': GRID, 'costmap': costmap}); return
        if self.path == '/preview/commands':
            self.send_json(200, {'commands': COMMANDS}); return
        if self.path in ('/', '/index.html'):
            data = (ROOT / 'web/static/index.html').read_text(encoding='utf-8').replace('</head>', FAKE_WEBSOCKET + '</head>').encode()
            self.send_response(200); self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(data))); self.end_headers(); self.wfile.write(data); return
        super().do_GET()

    def do_POST(self):
        if self.path == '/preview/commands':
            COMMANDS.append(json.loads(self.rfile.read(int(self.headers['Content-Length']))))
            self.send_json(200, {'ok': True}); return
        # No real ROS bridge or hardware is imported or started by this fixture.
        super().do_POST()


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--port', type=int, default=18780); args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='x3-web-preview-') as temporary:
        directory = Path(temporary)
        write_map(directory, 'Первый этаж'); write_map(directory, 'Лаборатория'); write_map(directory, 'Коридор')
        server.RUNTIME = type('Runtime', (), {'node': Bridge()})()
        server.SUPERVISOR = Supervisor(directory)
        server.DASHBOARD_TOKEN = ''
        http = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
        print(f'UI PREVIEW http://127.0.0.1:{args.port} (synthetic data; no hardware)', flush=True)
        try: http.serve_forever()
        except KeyboardInterrupt: pass
        finally: http.server_close()


if __name__ == '__main__': main()
