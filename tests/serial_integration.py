"""Exercise production entrypoints over PTYs; frame-loop tests live in Zig."""
import binascii
import ctypes
import errno
import os
import pty
import select
import signal
import socket
import struct
import subprocess
import sys
import termios
import threading
import tempfile
import time

RUNTIME, PENDING_SENSOR, SLOW_SENSOR, INIT_FAIL_SENSOR, FRAME_FAIL_SENSOR, SENSOR, FEATURE, FAST_SENSOR, COMPLETE_SENSOR, DEADLINE_SENSOR = sys.argv[1:]
VALUES = (0.125, -0.25, 0.5, -0.75, 1.0, -1.0)
ZERO = (0.0,) * 6


def reply(kind, extra=b''):
    payload = bytes((kind,)) + extra
    return b'\xa5\x5a' + payload + struct.pack('<H', binascii.crc_hqx(payload, 0xffff))


def wait_for(condition, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return
        time.sleep(.005)
    raise AssertionError('condition timed out')


class Controller:
    def __init__(self, behavior='healthy'):
        self.master, self.slave = pty.openpty()
        self.path = os.ttyname(self.slave)
        self.behavior = behavior
        self.packets = []
        self.raw = []
        self.errors = []
        self.stop = threading.Event()
        self.configured = threading.Event()
        self.ready = threading.Event()
        self.closed = False
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        try:
            while not self.stop.is_set():
                if not termios.tcgetattr(self.slave)[3] & termios.ICANON:
                    break
                time.sleep(.001)
            if self.stop.is_set():
                return
            settings = termios.tcgetattr(self.slave)
            assert settings[4:6] == [termios.B115200] * 2
            assert settings[2] & termios.CSIZE == termios.CS8
            assert not settings[2] & (termios.PARENB | termios.CSTOPB | termios.CRTSCTS)
            assert settings[2] & (termios.CLOCAL | termios.CREAD) == (termios.CLOCAL | termios.CREAD)
            assert not settings[0] & (termios.IXON | termios.IXOFF | termios.IXANY | termios.ICRNL)
            assert not settings[1] & termios.OPOST
            assert not settings[3] & (termios.ICANON | termios.ECHO | termios.ISIG | termios.IEXTEN)
            assert settings[6][termios.VMIN] in (0,b'\0')
            assert settings[6][termios.VTIME] in (0,b'\0')
            self.configured.set()
            # A host must not send a startup/neutral packet while waiting for READY.
            if self.stop.wait(.05):
                return
            assert not select.select([self.master], [], [], 0)[0]
            if self.behavior == 'no_ready':
                self.stop.wait(15)
                return
            if self.behavior == 'disconnect_startup':
                os.close(self.master)
                self.closed = True
                return
            greeting = reply(0x80)
            if self.behavior == 'bad_then_ready':
                greeting = reply(0x80)[:-1] + b'\x00' + b'\x00\xa5' + greeting
            elif self.behavior == 'error_before_ready':
                greeting = reply(0xe0, bytes((2, 3))) + greeting
            for byte in greeting:
                os.write(self.master, bytes((byte,)))
                time.sleep(.001)
            self.ready.set()
            buffer = bytearray()
            last_command = None
            watchdog_sent = False
            while not self.stop.is_set():
                available, _, _ = select.select([self.master], [], [], .01)
                if available:
                    buffer.extend(os.read(self.master, 8192))
                    while len(buffer) >= 29:
                        packet = bytes(buffer[:29])
                        del buffer[:29]
                        assert packet[:3] == b'\xa5\x5a\x06', packet
                        assert struct.unpack('<H', packet[-2:])[0] == binascii.crc_hqx(packet[2:-2], 0xffff)
                        self.raw.append(packet)
                        self.packets.append(struct.unpack('<6f', packet[3:-2]))
                        last_command = time.monotonic()
                        watchdog_sent = False
                        if self.behavior == 'disconnect':
                            os.close(self.master)
                            self.closed = True
                            return
                        if self.behavior == 'firmware_error':
                            os.write(self.master, reply(0xe0, bytes((2, 3))))
                        if self.behavior == 'bad_reply':
                            os.write(self.master, reply(0x80)[:-1] + b'\x00' + reply(0xe0, bytes((2, 3))))
                if self.behavior == 'watchdog' and last_command and not watchdog_sent and time.monotonic() - last_command >= .5:
                    os.write(self.master, reply(0xe0, bytes((9, 0))))
                    watchdog_sent = True
            assert not buffer, buffer
        except OSError as exc:
            if not self.stop.is_set() and exc.errno not in (errno.EIO, errno.EBADF):
                self.errors.append(exc)
        except BaseException as exc:
            self.errors.append(exc)

    def close(self):
        self.stop.set()
        self.thread.join(2)
        if not self.closed:
            os.close(self.master)
        os.close(self.slave)
        assert not self.thread.is_alive()
        assert not self.errors, self.errors


# Byte offsets follow LiveConfigPacket's explicit field serialization, never
# native struct layout. Status contains two complete configuration packets.
CONFIG_FIELDS = [
    ('magic',4), ('wire_version',2), ('camera_forward',12), ('rotation_limit',12),
    ('tam',192), ('tam_len',1), ('odometry',76), ('ibvs',204),
    ('goal_dist_threshold',4), ('thrustor_saturate',4), ('log_freq_div',2),
    ('queue_capacity',2), ('print_stuff',1), ('clear_print_stuff',1),
    ('telemetry_transport',1), ('telemetry_addr',66),
    ('telemetry_log_path',258), ('mission_id',1), ('mission_profile',1),
    ('sensor_source',1), ('sensor_library',258), ('serial_port',258),
    ('sim_robot',66), ('sim_scene',66), ('sim_realtime',1), ('revision',8),
]
OFFSETS = {}
CONFIG_SIZE = 0
for name, size in CONFIG_FIELDS:
    OFFSETS[name] = (CONFIG_SIZE, size)
    CONFIG_SIZE += size


def put(config, name, value):
    offset, size = OFFSETS[name]
    if isinstance(value, str):
        data = value.encode()
        assert len(data) <= size - 2
        value = data.ljust(size-2, b'\0') + struct.pack('<H',len(data))
    elif isinstance(value, int):
        value = value.to_bytes(size, 'little')
    assert len(value) == size
    config[offset:offset+size] = value


def field(config, name):
    offset, size = OFFSETS[name]
    return config[offset:offset+size]


class RuntimeClient:
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(2)
        self.id = int.from_bytes(os.urandom(8), 'little') or 1
        self.request_id = 0
    def request(self, action=0, request_id=None):
        if request_id is None:
            if action: self.request_id += 1
            request_id = self.request_id
        self.socket.sendto(b'ACR1'+struct.pack('<QQB',self.id,request_id,action), ('127.0.0.1',8080))
        return self.receive()
    def receive(self):
        data = self.socket.recv(8192)
        assert data[:4] == b'ACS1' and len(data) == 40 + 2*CONFIG_SIZE + 258 + 8, (len(data), CONFIG_SIZE)
        assert struct.unpack_from('<H', data, 44)[0] == 5, 'unexpected live config wire version'
        state,result,revision,client,request,config_reply,config_rejected,active = struct.unpack_from('<BBQQQQBB', data, 4)
        error = data[40+2*CONFIG_SIZE:40+2*CONFIG_SIZE+258]
        run_id = struct.unpack_from('<Q',data,40+2*CONFIG_SIZE+258)[0]
        return dict(state=state,result=result,revision=revision,client=client,request=request,active=active,config_reply=config_reply,config_rejected=config_rejected,run_id=run_id,
                    config=bytearray(data[40:40+CONFIG_SIZE]), active_config=data[40+CONFIG_SIZE:40+2*CONFIG_SIZE],
                    error=error[:struct.unpack_from('<H',error,256)[0]].decode())
    def apply(self, config):
        self.socket.sendto(config, ('127.0.0.1',8080))
        return self.receive()
    def direct(self, run_id, sequence, values, count=None, client_id=None):
        if count is None: count=len(values)
        if client_id is None: client_id=self.id
        packet=b'AUT1'+struct.pack('<QQQB8f',run_id,client_id,sequence,count,*(tuple(values)+(0.,)*(8-len(values))))
        self.socket.sendto(packet,('127.0.0.1',8080))
    def until(self, predicate, timeout=4):
        deadline = time.monotonic()+timeout
        status = None
        while time.monotonic()<deadline:
            status = self.request()
            if predicate(status): return status
            time.sleep(.02)
        raise AssertionError({k:v for k,v in status.items() if k not in ("config","active_config")})


def runtime_cases():
    trace_dir = tempfile.TemporaryDirectory(prefix='auv-lifecycle-')
    trace_path = os.path.join(trace_dir.name, 'sensor-events')
    def events():
        try:
            with open(trace_path) as trace: return trace.read().splitlines()
        except FileNotFoundError: return []
    env = dict(os.environ, AUV_TEST_SENSOR_TRACE=trace_path)
    process = subprocess.Popen([RUNTIME], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    client = RuntimeClient()
    try:
        for _ in range(20):
            try: status=client.request(); break
            except socket.timeout:
                assert process.poll() is None, process.communicate()[1].decode()
        else: raise AssertionError('runtime control unavailable')
        # Bootstrap failure leaves the listener available for isolated test telemetry.
        client.request(2)
        status=client.until(lambda s:s['state']==0)
        assert field(status['config'],'sensor_source')==b'\x01'
        changed=bytearray(status['config']);put(changed,'sensor_source',0)
        rejected=client.apply(changed)
        assert rejected['config_rejected'] and rejected['error']=='SensorSourceIsCompiled'
        config=status['config']
        put(config,'telemetry_transport',2); put(config,'print_stuff',0)
        put(config,'log_freq_div',65535); put(config,'telemetry_log_path','')
        put(config,'sensor_source',1);put(config,'sensor_library','/absent')
        status=client.apply(config)
        status=client.until(lambda s:s['state']==0 and s['result']==3)
        assert 'FileNotFound' in status['error'] or status['error']=='MissingOutput', status
        # A rejected update cannot replace the accepted configuration.
        config=status['config']; old_revision=status['revision']; put(config,'tam_len',6)
        rejected=client.apply(config); assert rejected['revision']==old_revision
        # Opening a broken publisher is transactional and control stays reachable.
        config=rejected['config'];put(config,'telemetry_transport',0);put(config,'telemetry_addr','invalid')
        rejected=client.apply(config);assert rejected['revision']==old_revision
        # Native eight-channel configuration remains selectable and serial is
        # rejected for that profile before any resources are touched.
        config=rejected['config'];put(config,'serial_port','/absent')
        rejected=client.apply(config);assert rejected['revision']==old_revision
        # Use Proteus geometry for all physical sensor / serial tests.
        def physical_config(status, controller, sensor):
            config=status['config']
            put(config,'mission_id',2);put(config,'mission_profile',1);put(config,'tam_len',6)
            put(config,'sim_robot','proteus');put(config,'sim_scene','robot_only')
            put(config,'sensor_source',1);put(config,'sensor_library',sensor)
            put(config,'serial_port',controller.path)
            rows=((0,1,0,0,0,1),(0,1,0,0,0,-1),(0,0,-1,1,1,0),(0,0,-1,1,-1,0),(0,0,-1,-1,1,0),(0,0,-1,-1,-1,0),(0,)*6,(0,)*6)
            put(config,'tam',struct.pack('<48f',*(v for row in rows for v in row)))
            put(config,'rotation_limit',struct.pack('<3f',.3,0,100))
            return config
        for sensor in (SENSOR,FAST_SENSOR):
            controller=Controller('firmware_error')
            try:
                status=client.request();client.apply(physical_config(status,controller,sensor));client.request(1)
                client.until(lambda s:s['state']==2)
                wait_for(lambda:len(controller.packets)>=2)
                # Keyboard teleop still flows while firmware error replies arrive.
                client.socket.sendto(struct.pack('<7f',0,.8,0,0,0,0,1),('127.0.0.1',8080))
                wait_for(lambda:any(any(abs(v)>.01 for v in p) for p in controller.packets))
                status=client.request(); config=status['config']
                kp=bytearray(field(config,'odometry'));struct.pack_into('<f',kp,4,1.5);put(config,'odometry',kp)
                assert client.apply(config)['revision']==status['revision']+1
                duplicate=client.apply(config);assert duplicate['revision']==status['revision']+1 and not duplicate['config_rejected']
                # Replace telemetry destinations without interrupting serial output.
                with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as telemetry_socket:
                    telemetry_socket.bind(('127.0.0.1',0));telemetry_socket.settimeout(2)
                    config=client.request()['config'];put(config,'telemetry_transport',0)
                    put(config,'telemetry_addr','127.0.0.1:'+str(telemetry_socket.getsockname()[1]))
                    assert client.apply(config)['state']==2
                    wire=telemetry_socket.recv(65535)
                    assert wire[:4]==b'AUV1' and wire[64]==6
                    values=struct.unpack_from('<6f',wire,9284)
                    wait_for(lambda:values in controller.packets)
                    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as next_socket:
                        next_socket.bind(('127.0.0.1',0));next_socket.settimeout(2)
                        config=client.request()['config']
                        put(config,'telemetry_addr','127.0.0.1:'+str(next_socket.getsockname()[1]))
                        assert client.apply(config)['state']==2
                        assert next_socket.recv(65535)[:4]==b'AUV1'
                fd,log_path=tempfile.mkstemp(prefix='auv-runtime-');os.close(fd)
                try:
                    config=client.request()['config'];put(config,'telemetry_transport',2);put(config,'telemetry_log_path',log_path)
                    assert client.apply(config)['state']==2
                    wait_for(lambda:os.path.getsize(log_path)>0)
                    status=client.request();config=status['config'];put(config,'telemetry_log_path','/absent/path/log')
                    assert client.apply(config)['revision']==status['revision']
                    config=client.request()['config'];put(config,'telemetry_log_path','');client.apply(config)
                finally:os.unlink(log_path)
                client.request(2); status=client.until(lambda s:s['state']==0)
                count=len(controller.packets);time.sleep(.15);assert len(controller.packets)==count
                assert not controller.errors,controller.errors
            finally: controller.close()
        # Remain controllable during fragmented READY startup; Stop sends nothing.
        controller=Controller('no_ready')
        try:
            client.apply(physical_config(client.request(),controller,SENSOR));client.request(1)
            wait_for(controller.configured.is_set)
            start=time.monotonic();client.request(2);client.until(lambda s:s['state']==0)
            assert time.monotonic()-start<.5 and not controller.packets
        finally:controller.close()
        # Tuning accepted while waiting for READY must remain active when the
        # mission context finishes initializing.
        controller=Controller('no_ready')
        try:
            client.apply(physical_config(client.request(),controller,SENSOR));client.request(1)
            wait_for(controller.configured.is_set)
            config=client.request()['config']
            threshold=struct.pack('<f',.37)
            put(config,'goal_dist_threshold',threshold)
            assert not client.apply(config)['config_rejected']
            time.sleep(.08)
            os.write(controller.master,reply(0x80))
            status=client.until(lambda s:s['state']==2)
            assert field(status['config'],'goal_dist_threshold')==threshold
            assert field(status['active_config'],'goal_dist_threshold')==threshold
            client.request(2);client.until(lambda s:s['state']==0)
        finally:controller.close()
        controller=Controller('disconnect')
        try:
            client.apply(physical_config(client.request(),controller,SENSOR));client.request(1)
            status=client.until(lambda s:s['state']==0 and s['result']==3)
            assert status['error']=='SerialDisconnected',status['error']
        finally:controller.close()
        # Queue two operational edits while a synchronous SDK init is blocked.
        # Both are drained before unwinding; only the latest replacement opens.
        controller=Controller()
        try:
            before=len(events())
            initial=client.apply(physical_config(client.request(),controller,SLOW_SENSOR))
            wait_for(lambda:'I1' in events()[before:])
            next_config=bytearray(initial['config']);put(next_config,'sensor_library',INIT_FAIL_SENSOR)
            latest_config=bytearray(next_config);put(latest_config,'sensor_library',SENSOR)
            put(latest_config,'revision',initial['revision']+1)
            client.socket.sendto(next_config,('127.0.0.1',8080))
            client.socket.sendto(latest_config,('127.0.0.1',8080))
            first=client.receive();second=client.receive()
            assert not first['config_rejected'] and not second['config_rejected'], (first,second)
            status=client.until(lambda s:s['state']==2)
            assert field(status['config'],'sensor_library')==field(latest_config,'sensor_library')
            assert events()[before:]==['I1','R1','D1'],events()[before:]
            client.request(2);client.until(lambda s:s['state']==0)
        finally:controller.close()
        # Frame acquisition owns its deadline and reports custom errors in the
        # frame. Blocking SDK initialization delays Stop until it returns.
        for sensor in (PENDING_SENSOR,SLOW_SENSOR,INIT_FAIL_SENSOR,FRAME_FAIL_SENSOR):
            controller=Controller()
            try:
                before=len(events())
                started=time.monotonic()
                client.apply(physical_config(client.request(),controller,sensor))
                if sensor in (PENDING_SENSOR,INIT_FAIL_SENSOR,FRAME_FAIL_SENSOR):
                    status=client.until(lambda s:s['state']==0 and s['result']==3)
                    assert 'test ' in status['error'],status
                    mode=0 if sensor==PENDING_SENSOR else 2 if sensor==INIT_FAIL_SENSOR else 3
                    assert events()[before:][0]=='I'+str(mode) and events()[-1]=='D'+str(mode),events()[before:]
                    if sensor==PENDING_SENSOR:
                        assert 'test frame timeout'==status['error'],status['error']
                        assert time.monotonic()-started>=.09
                        assert events()[before:]==['I0','R0','W0','D0']
                        assert not controller.packets
                    if sensor==FRAME_FAIL_SENSOR:
                        assert 'test frame failure'==status['error'],status['error']
                else:
                    mode=0 if sensor==PENDING_SENSOR else 1
                    wait_for(lambda:'I'+str(mode) in events()[before:])
                    start=time.monotonic();client.request(2);client.until(lambda s:s['state']==0)
                    elapsed=time.monotonic()-start
                    if sensor==SLOW_SENSOR:
                        assert .25<elapsed<2,elapsed
                        assert not controller.packets and not controller.configured.is_set()
                    else: assert elapsed<.5,elapsed
                    assert events()[before:]==['I'+str(mode),'R'+str(mode),'D'+str(mode)],events()[before:]
                assert process.poll() is None
            finally:controller.close()
        # A request received during yield is handled after the plugin's compiled
        # deadline expires. Cleanup still generates no command packets.
        controller=Controller()
        try:
            before=len(events())
            client.apply(physical_config(client.request(),controller,PENDING_SENSOR))
            wait_for(lambda:'W0' in events()[before:])
            started=time.monotonic()
            client.request(2);client.until(lambda s:s['state']==0 and s['result']==2)
            assert .03<time.monotonic()-started<.5
            assert events()[before:]==['I0','R0','W0','D0']
            assert not controller.packets
        finally:controller.close()
        # A real finite mission returns through the same scope, cleans up once,
        # and remains idle without repeating commands or reopening the plugin.
        controller=Controller()
        try:
            before=len(events())
            config=physical_config(client.request(),controller,COMPLETE_SENSOR)
            put(config,'mission_id',0)
            put(config,'goal_dist_threshold',struct.pack('<f',100))
            client.apply(config)
            status=client.until(lambda s:s['state']==0 and s['result']==1)
            assert events()[before:]==['I4','R4','D4'],events()[before:]
            assert len(controller.packets)>=3
            count=len(controller.packets);time.sleep(.15)
            assert len(controller.packets)==count and events()[before:]==['I4','R4','D4']
            # Start executes the same completed mission again. Restart also
            # repeats it; duplicate request ids never cause another execution.
            for action in (1,3):
                client.request(action)
                time.sleep(.05);os.write(controller.master,reply(0x80))
                client.until(lambda s:s['state']==0 and s['result']==1)
                count=len(controller.packets)
                duplicate=client.request(action,client.request_id)
                assert duplicate['state']==0 and duplicate['result']==1
                time.sleep(.05);assert len(controller.packets)==count
            assert events()[before:]==['I4','R4','D4']*3,events()[before:]
        finally:controller.close()
        # Recover after a failure, switch mission while running, and explicitly
        # restart the same mission. Cleanup must precede the next initialization.
        controller=Controller()
        try:
            before=len(events())
            config=physical_config(client.request(),controller,COMPLETE_SENSOR)
            put(config,'goal_dist_threshold',struct.pack('<f',.1))
            client.apply(config);client.until(lambda s:s['state']==2)
            client.socket.sendto(struct.pack('<7f',0,.8,0,0,0,0,1),('127.0.0.1',8080))
            wait_for(lambda:any(any(abs(v)>.01 for v in packet) for packet in controller.packets))
            client.request(1) # Start while running does not reopen resources.
            assert events()[before:]==['I4','R4'],events()[before:]
            for mission in (3,0):
                config=client.request()['config'];put(config,'mission_id',mission)
                client.apply(config)
                time.sleep(.05)
                count=len(controller.packets)
                os.write(controller.master,reply(0x80))
                client.until(lambda s:s['state']==2)
                wait_for(lambda:len(controller.packets)>count)
                if mission==3:
                    assert controller.packets[count]==ZERO,controller.packets[count]
            client.request(3)
            time.sleep(.05);os.write(controller.master,reply(0x80))
            client.until(lambda s:s['state']==2)
            client.request(2);client.until(lambda s:s['state']==0)
            assert events()[before:]==['I4','R4','D4']*4,events()[before:]
        finally:controller.close()
        # Direct control bypasses PID, waits for explicit input, and holds only
        # the latest valid command from the owner of the current mission run.
        controller=Controller('bad_then_ready')
        try:
            config=physical_config(client.request(),controller,SENSOR)
            put(config,'mission_id',4)
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as output:
                output.bind(('127.0.0.1',0));output.settimeout(2)
                put(config,'telemetry_transport',0)
                put(config,'telemetry_addr','127.0.0.1:'+str(output.getsockname()[1]))
                client.apply(config)
                status=client.until(lambda s:s['state']==2)
                run_id=status['run_id']; assert run_id
                time.sleep(.1);assert not controller.packets
                client.direct(run_id+1,1,VALUES)
                client.direct(run_id,1,VALUES,count=8)
                client.direct(run_id,1,(float('nan'),0,0,0,0,0))
                time.sleep(.1);assert not controller.packets
                client.direct(run_id,1,VALUES)
                wait_for(lambda:len(controller.packets)>=2)
                assert controller.packets[-1]==VALUES
                wire=output.recv(65535)
                while struct.unpack_from('<6f',wire,9284)!=VALUES: wire=output.recv(65535)
                assert wire[64]==6
                newer=(-.125,.25,-.5,.75,-1.,1.)
                client.direct(run_id,2,newer)
                wait_for(lambda:controller.packets[-1]==newer)
                client.direct(run_id,1,VALUES)
                client.direct(run_id,2,VALUES)
                client.direct(run_id,3,VALUES,client_id=client.id+1)
                time.sleep(.1); assert controller.packets[-1]==newer
                # Repeated identical commands extend the connection; explicit
                # zero is ordinary submitted data, not a cleanup packet.
                client.direct(run_id,3,ZERO)
                wait_for(lambda:controller.packets[-1]==ZERO)
                client.direct(run_id,4,newer)
                wait_for(lambda:controller.packets[-1]==newer)
                hold_until=time.monotonic()+2.3
                while time.monotonic()<hold_until:
                    client.direct(run_id,4,newer)
                    time.sleep(.2)
                assert client.request()['state']==2 and controller.packets[-1]==newer
                start=time.monotonic()
                status=client.until(lambda s:s['state']==0 and s['result']==2,timeout=4)
                assert 1.7 < time.monotonic()-start < 3.5
                assert 'timed out' in status['error']
                time.sleep(.1); count=len(controller.packets)
                time.sleep(.2); assert len(controller.packets)==count and controller.packets[-1]==newer
                client.request(1);time.sleep(.05);os.write(controller.master,reply(0x80))
                status=client.until(lambda s:s['state']==2)
                assert status['run_id']!=run_id
                time.sleep(.1); count=len(controller.packets)
                client.direct(run_id,99,VALUES)
                time.sleep(.15);assert len(controller.packets)==count
                client.direct(status['run_id'],1,VALUES)
                wait_for(lambda:len(controller.packets)>count)
                client.request(2);client.until(lambda s:s['state']==0)
                time.sleep(.1);count=len(controller.packets);time.sleep(.1)
                assert len(controller.packets)==count and controller.packets[-1]==VALUES
        finally:controller.close()
        # READY timeout and disconnect still propagate through mission startup.
        for behavior in ('no_ready','disconnect_startup'):
            controller=Controller(behavior)
            try:
                config=physical_config(client.request(),controller,SENSOR);put(config,'mission_id',4)
                client.apply(config);client.request(1)
                status=client.until(lambda s:s['state']==0 and s['result']==3,timeout=13)
                assert status['error']==('SerialReadyTimeout' if behavior=='no_ready' else 'SerialDisconnected'),status
                assert not controller.packets
            finally:controller.close()
        print('PASS direct thruster mission, run identity, timeout and serial',flush=True)
        # Duplicate lifecycle requests are acknowledged without another attempt.
        config=client.request()['config'];put(config,'sensor_source',1);put(config,'sensor_library',INIT_FAIL_SENSOR)
        client.apply(config);client.until(lambda s:s['state']==0 and s['result']==3)
        status=client.request(3); duplicate_id=client.request_id
        client.until(lambda s:s['state']==0 and s['result']==3)
        status=client.request(3,duplicate_id)
        assert status['request']==duplicate_id and status['state']==0,status
        for path in (SENSOR,FAST_SENSOR):
            plugin=ctypes.CDLL(path)
            for name in ('auv_init','auv_yield_until_next_frame','auv_deinit'):assert getattr(plugin,name)
            for name in ('auv_set_thrustor_values','auv_poll_frame'):assert not hasattr(plugin,name)
        print('PASS persistent runtime, live config, lifecycle, serial data, sensor ABI and cleanup ('+FEATURE+')',flush=True)
        return client.request()['config']
    finally:
        client.socket.close()
        if process.poll() is None:process.send_signal(signal.SIGINT)
        _,stderr=process.communicate(timeout=5)
        trace_dir.cleanup()
        assert process.returncode==-signal.SIGINT,stderr.decode()


def simulated_sensor_cases():
    process=subprocess.Popen([RUNTIME, '--headless'],stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    client=RuntimeClient()
    try:
        for _ in range(20):
            try: status=client.request(2);break
            except socket.timeout:
                assert process.poll() is None,process.communicate()[1].decode()
        else: raise AssertionError('runtime control unavailable')
        status=client.until(lambda s:s['state']==0)
        assert field(status['config'],'sensor_source')==b'\x00'
        changed=bytearray(status['config']);put(changed,'sensor_source',1)
        rejected=client.apply(changed)
        assert rejected['config_rejected'] and rejected['error']=='SensorSourceIsCompiled'
        config=client.request()['config']
        put(config,'mission_id',4);put(config,'mission_profile',0);put(config,'tam_len',8)
        put(config,'serial_port','');put(config,'sim_robot','hydrus')
        put(config,'sim_scene','robot_only');put(config,'sim_realtime',1)
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as output:
            output.bind(('127.0.0.1',0));output.settimeout(2)
            put(config,'telemetry_transport',0);put(config,'telemetry_addr','127.0.0.1:'+str(output.getsockname()[1]))
            assert not client.apply(config)['config_rejected']
            status=client.until(lambda s:s['state']==2)
            initial=output.recv(65535)
            assert initial[64]==8 and struct.unpack_from('<8f',initial,9284)==(0.,)*8
            values=(.1,.2,.3,.4,-.1,-.2,-.3,-.4)
            expected=struct.unpack('<8f',struct.pack('<8f',*values))
            client.direct(status['run_id'],1,values)
            deadline=time.monotonic()+2
            while time.monotonic()<deadline:
                wire=output.recv(65535)
                if struct.unpack_from('<8f',wire,9284)==expected: break
            else: raise AssertionError('missing direct simulator telemetry')
            client.request(2);client.until(lambda s:s['state']==0)

        # Simulator frames and serial output share the same submitted values.
        controller=Controller()
        try:
            config=client.request()['config']
            put(config,'mission_id',4);put(config,'mission_profile',1);put(config,'tam_len',6)
            put(config,'serial_port',controller.path);put(config,'sim_robot','proteus')
            rows=((0,1,0,0,0,1),(0,1,0,0,0,-1),(0,0,-1,1,1,0),(0,0,-1,1,-1,0),(0,0,-1,-1,1,0),(0,0,-1,-1,-1,0),(0,)*6,(0,)*6)
            put(config,'tam',struct.pack('<48f',*(v for row in rows for v in row)))
            put(config,'rotation_limit',struct.pack('<3f',.3,0,100))
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as output:
                output.bind(('127.0.0.1',0));output.settimeout(2)
                put(config,'telemetry_addr','127.0.0.1:'+str(output.getsockname()[1]))
                assert not client.apply(config)['config_rejected']
                status=client.until(lambda s:s['state']==2)
                initial_pose=struct.unpack_from('<3f',output.recv(65535),16)
                assert not controller.packets
                end=time.monotonic()+1.2;next_send=0;matched=False;moved=False
                while time.monotonic()<end:
                    if time.monotonic()>=next_send:
                        client.direct(status['run_id'],1,VALUES);next_send=time.monotonic()+.2
                    wire=output.recv(65535)
                    if struct.unpack_from('<6f',wire,9284)==VALUES: matched=True
                    pose=struct.unpack_from('<3f',wire,16)
                    if sum((a-b)**2 for a,b in zip(pose,initial_pose))>1e-8: moved=True
                wait_for(lambda:VALUES in controller.packets)
                assert matched and moved,(matched,moved)
                client.request(2);client.until(lambda s:s['state']==0)
        finally:controller.close()
        print('PASS compiled simulator sensors, serial fan-out and source rejection',flush=True)
    finally:
        client.socket.close()
        if process.poll() is None:process.send_signal(signal.SIGINT)
        _,stderr=process.communicate(timeout=5)
        assert process.returncode==-signal.SIGINT,stderr.decode()


def frame_deadline_cases():
    # Exercise the production sensor implementation with a compiled frame
    # interval longer than its deadline, then check successful frame acquisition.
    for path in (DEADLINE_SENSOR,SENSOR):
        plugin=ctypes.CDLL(path)
        plugin.auv_init.argtypes=[]
        plugin.auv_init.restype=None
        plugin.auv_yield_until_next_frame.argtypes=[ctypes.c_void_p]
        plugin.auv_yield_until_next_frame.restype=None
        plugin.auv_deinit.argtypes=[]
        plugin.auv_deinit.restype=None
        frame=ctypes.create_string_buffer(11664)
        try:
            plugin.auv_init()
            started=time.monotonic()
            plugin.auv_yield_until_next_frame(frame)
            elapsed=time.monotonic()-started
            code=struct.unpack_from('<I',frame.raw,11396)[0]
            message=frame.raw[11400:11656].split(b'\0',1)[0]
            if path==DEADLINE_SENSOR:
                assert code==5 and b'timed out' in message,(code,message)
                assert 1<=elapsed<2,elapsed
            else: assert code==0,(code,message)
            if path==SENSOR:
                ctypes.memset(frame,0xff,len(frame))
                plugin.auv_yield_until_next_frame(frame)
                assert struct.unpack_from('<I',frame.raw,11396)[0]==0
        finally:
            plugin.auv_deinit()
    print('PASS sensor-owned frame deadlines, error layout and successful recovery',flush=True)


def signal_cases(template):
    # Default signal termination must not wait for the sensor's blocking init
    # to finish and must not depend on a callback into the runtime.
    for signum in (signal.SIGINT,signal.SIGTERM):
        with tempfile.TemporaryDirectory(prefix='auv-signal-') as directory:
            trace_path=os.path.join(directory,'sensor-events')
            process=subprocess.Popen([RUNTIME],env=dict(os.environ,AUV_TEST_SENSOR_TRACE=trace_path),
                                     stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
            client=RuntimeClient()
            try:
                for _ in range(10):
                    try: client.request(2);break
                    except socket.timeout:
                        assert process.poll() is None,process.communicate()[1].decode()
                else: raise AssertionError('runtime control unavailable')
                status=client.until(lambda s:s['state']==0)
                config=bytearray(template)
                put(config,'revision',status['revision'])
                put(config,'sensor_source',1)
                put(config,'sensor_library',SLOW_SENSOR)
                put(config,'serial_port','/unused-during-initialization')
                put(config,'telemetry_transport',2)
                put(config,'telemetry_log_path','')
                assert not client.apply(config)['config_rejected']
                def initialization_started():
                    try:
                        with open(trace_path) as trace:return 'I1' in trace.read().splitlines()
                    except FileNotFoundError:return False
                wait_for(initialization_started)
                started=time.monotonic()
                process.send_signal(signum)
                _,stderr=process.communicate(timeout=1)
                assert process.returncode==-signum,stderr.decode()
                assert time.monotonic()-started<.3
                with open(trace_path) as trace:assert trace.read().splitlines()==['I1']
            finally:
                client.socket.close()
                if process.poll() is None:
                    process.kill();process.communicate(timeout=2)
    print('PASS SIGINT and SIGTERM terminate during blocking sensor initialization',flush=True)


frame_deadline_cases()
if FEATURE=='simulated':
    simulated_sensor_cases()
else:
    signal_cases(runtime_cases())
