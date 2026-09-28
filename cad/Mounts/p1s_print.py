"""Start a print on a Bambu Lab P1S over local MQTT from a G-code already on the SD card.

    python3 p1s_print.py status
    python3 p1s_print.py print Mounts/P1S_LiteOn_45W_Brick_v2_PLA.gcode

Talks to the printer's own broker (port 8883, TLS, user bblp, password = access code), asks
for a full status push to learn the serial and current state, then publishes the same
project_file command Bambu Studio sends for a plain G-code on the card. The printer ignores
print commands from LAN MQTT while it is bound to the cloud and not in LAN-only mode, and
simply does not answer; this script reports that instead of pretending.
"""
import json, os, ssl, sys, time
import paho.mqtt.client as mqtt

HOST = os.environ.get('P1S_HOST', '192.168.1.203')
CODE = os.environ.get('P1S_CODE', '17392708')
SERIAL = os.environ.get('P1S_SERIAL', '01P2E2XXDTJJ4D16')   # the printer only reports after a pushall to its own topic

state = {'serial': None, 'print': {}, 'acks': []}


def on_connect(c, u, flags, rc, props=None):
    c.subscribe(f'device/{SERIAL}/report')   # the broker drops the session on any wildcard


def on_message(c, u, msg):
    serial = msg.topic.split('/')[1]
    state['serial'] = serial
    try:
        d = json.loads(msg.payload)
    except Exception:
        return
    if 'print' in d:
        p = d['print']
        state['print'].update({k: p[k] for k in ('gcode_state', 'mc_percent', 'mc_remaining_time', 'subtask_name',
                                                'gcode_file', 'nozzle_temper', 'bed_temper', 'print_error',
                                                'wifi_signal', 'command', 'result', 'reason', 'sequence_id') if k in p})
        if p.get('command') == 'project_file':
            state['acks'].append(p)


def client():
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id='atlas-print', protocol=mqtt.MQTTv311)
    c.username_pw_set('bblp', CODE)
    c.tls_set(cert_reqs=ssl.CERT_NONE)
    c.tls_insecure_set(True)
    c.on_connect = on_connect
    c.on_message = on_message
    c.connect(HOST, 8883, 30)
    c.loop_start()
    time.sleep(1.5)
    c.publish(f'device/{SERIAL}/request', json.dumps({'pushing': {'sequence_id': '1', 'command': 'pushall'}}))
    t0 = time.time()
    while state['serial'] is None and time.time() - t0 < 10:
        time.sleep(0.2)
    if state['serial'] is None:
        raise SystemExit('no report from the printer in 10 s (wrong host/code/serial, or MQTT off)')
    time.sleep(2)
    return c


def show():
    p = state['print']
    print(f'serial {state["serial"]}  state {p.get("gcode_state")}  file {p.get("gcode_file") or p.get("subtask_name")}  '
          f'{p.get("mc_percent")}%  {p.get("mc_remaining_time")} min left  nozzle {p.get("nozzle_temper")}  '
          f'bed {p.get("bed_temper")}  error {p.get("print_error")}')


if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'status'
    c = client()
    show()
    if cmd == 'print':
        path = sys.argv[2]
        if state['print'].get('gcode_state') in ('RUNNING', 'PREPARE', 'PAUSE'):
            raise SystemExit('printer is busy; not sending')
        req = {'print': {'sequence_id': '2000', 'command': 'project_file',
                         'param': 'Metadata/plate_1.gcode' if path.endswith('.3mf') else '',
                         'url': f'file:///sdcard/{path}', 'subtask_name': os.path.basename(path),
                         'use_ams': False, 'timelapse': False, 'bed_leveling': True, 'flow_cali': False,
                         'vibration_cali': False, 'layer_inspect': False, 'bed_type': 'textured_plate',
                         'profile_id': '0', 'project_id': '0', 'subtask_id': '0', 'task_id': '0',
                         'md5': '', 'file': ''}}
        c.publish(f'device/{state["serial"]}/request', json.dumps(req))
        print('sent project_file for', path)
        for i in range(30):
            time.sleep(1)
            if state['acks'] or state['print'].get('gcode_state') in ('RUNNING', 'PREPARE'):
                break
        show()
        if state['acks']:
            print('printer answered:', json.dumps(state['acks'][-1])[:300])
        elif state['print'].get('gcode_state') in ('RUNNING', 'PREPARE'):
            print('print started')
        else:
            print('no ack and state unchanged: the printer is ignoring LAN print commands '
                  '(cloud-bound, not in LAN-only mode). Start it from the screen: Files > SD card > Mounts.')
    c.loop_stop()
