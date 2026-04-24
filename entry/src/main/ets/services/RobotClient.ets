/**
 * RobotClient — native WebSocket bridge to teach_monitor.py on the
 * satellite robot arm.
 *
 * Lives in ArkTS (not inside the arm.html WebView) because ArkWeb
 * blocks cleartext ws:// regardless of the module-level
 * `network_security_config`; that metadata only applies to
 * `@ohos.net.*` APIs. The Workshop page forwards every arm-pose
 * frame and joystick rotation through ArmJsBridge here, and this
 * class opens a real WebSocket over the system network stack.
 *
 * Connection lifecycle:
 *   armOn becomes true  → connect()
 *   armOn becomes false → disconnect()
 *   network error       → auto-reconnect with 2s backoff (matches
 *                         what the in-WebView client used to do)
 */

import webSocket from '@ohos.net.webSocket';
import connection from '@ohos.net.connection';
import http from '@ohos.net.http';
import { BusinessError } from '@ohos.base';
import { hilog } from '@kit.PerformanceAnalysisKit';

export type RobotState = 'idle' | 'connecting' | 'linked' | 'active' | 'dead';

type StatusListener = (cls: RobotState, head: string, detail: string) => void;

export class RobotClient {
  private ws: webSocket.WebSocket | null = null;
  private url: string = 'ws://172.20.10.2:8765';
  private state: RobotState = 'idle';
  private active: boolean = false;
  private framesSent: number = 0;
  private reconnectTimer: number = -1;
  private autoReconnect: boolean = false;
  private statusListeners: StatusListener[] = [];

  getUrl(): string { return this.url; }
  setUrl(url: string): void {
    if (!/^wss?:\/\//i.test(url)) return;
    this.url = url;
  }
  getState(): RobotState { return this.state; }

  onStatus(cb: StatusListener): void { this.statusListeners.push(cb); }

  private emitStatus(cls: RobotState, head: string, detail: string): void {
    this.state = cls;
    hilog.info(0x1, 'RobotClient',
      'status %{public}s %{public}s %{public}s', cls, head, detail);
    for (const l of this.statusListeners) {
      try { l(cls, head, detail); } catch (_) {}
    }
  }

  // HarmonyOS picks a "default" net per-app for outbound connections,
  // and on a device that has both WiFi + cellular up it often defaults
  // to cellular. LAN targets like 172.20.10.2 aren't reachable over
  // cellular → connect() fails fast with a netstack-level error that
  // looks like a TCP timeout. Force the whole app to route via WiFi
  // before opening the socket.
  private async bindToWifi(): Promise<void> {
    hilog.info(0x1, 'RobotClient', 'bindToWifi: start');
    try {
      const nets = await connection.getAllNets();
      hilog.info(0x1, 'RobotClient', 'bindToWifi: got %{public}d nets',
        nets.length);
      for (let i = 0; i < nets.length; i++) {
        const n = nets[i];
        try {
          const caps = await connection.getNetCapabilities(n);
          const bearers = caps.bearerTypes || [];
          hilog.info(0x1, 'RobotClient',
            'bindToWifi: net[%{public}d] netId=%{public}d bearers=%{public}s',
            i, n.netId, JSON.stringify(bearers));
          if (bearers.indexOf(1) !== -1) {
            await connection.setAppNet(n);
            hilog.info(0x1, 'RobotClient',
              'bindToWifi: BOUND to WiFi netId=%{public}d', n.netId);
            return;
          }
        } catch (ee) {
          const err = ee as BusinessError;
          hilog.warn(0x1, 'RobotClient',
            'bindToWifi: net[%{public}d] caps err: %{public}s',
            i, err.message || String(err.code));
        }
      }
      hilog.error(0x1, 'RobotClient',
        'bindToWifi: NO WIFI net found — %{public}d total — check phone WiFi + cellular settings',
        nets.length);
    } catch (e) {
      const err = e as BusinessError;
      hilog.error(0x1, 'RobotClient', 'bindToWifi: getAllNets err: %{public}s',
        err.message || String(err.code));
    }
  }

  /**
   * Plain HTTP GET to the robot as a reachability probe. Isolates
   * "can this app's TCP make it to the robot at all" from
   * WebSocket-specific handshake concerns. The websockets Python
   * library replies with HTTP 400 to a plain GET — any response at
   * all proves the TCP path is alive.
   */
  async diagnoseReachability(): Promise<void> {
    // Re-use the ws URL with a http scheme.
    const probeUrl = this.url.replace(/^ws/i, 'http');
    hilog.info(0x1, 'RobotClient', 'probe: GET %{public}s', probeUrl);
    try {
      const req = http.createHttp();
      const resp = await req.request(probeUrl, {
        method: http.RequestMethod.GET,
        connectTimeout: 5000,
        readTimeout: 5000
      });
      hilog.info(0x1, 'RobotClient',
        'probe: responseCode=%{public}d (任何响应 — 即使 400 — 都说明 TCP 通)',
        resp.responseCode);
      req.destroy();
    } catch (e) {
      const err = e as BusinessError;
      hilog.error(0x1, 'RobotClient',
        'probe: FAILED msg=%{public}s code=%{public}d — TCP 根本走不到机器人',
        err.message || '', err.code || 0);
    }
  }

  // Open a new socket and bind listeners. If one was already open we
  // close it first — HarmonyOS's webSocket.WebSocket object is single-
  // use and reconnecting on the same instance drops messages silently.
  async connect(): Promise<void> {
    this.autoReconnect = true;
    if (this.state === 'connecting' || this.state === 'linked' || this.state === 'active') {
      return;
    }
    this.framesSent = 0;
    this.active = false;
    this.emitStatus('connecting', 'ROBOT · CONNECTING', this.url);

    // Route via WiFi — must happen before ws.connect().
    await this.bindToWifi();

    // Reachability probe — plain HTTP GET. Tells us whether the TCP
    // path from this app to the robot works at all, separately from
    // anything WebSocket-specific. Non-blocking so it doesn't hold up
    // the actual ws.connect.
    this.diagnoseReachability();

    hilog.info(0x1, 'RobotClient', 'ws.connect → %{public}s', this.url);

    if (this.ws) {
      try { await this.ws.close(); } catch (_) {}
      this.ws = null;
    }

    const ws = webSocket.createWebSocket();
    this.ws = ws;

    ws.on('open', (err: BusinessError, _v: Object): void => {
      if (err && err.code) {
        this.emitStatus('dead', 'ROBOT · OPEN ERR', err.message || String(err.code));
        return;
      }
      this.emitStatus('linked', 'ROBOT · LINKED', '等待 phone_start 确认…');
      // phone_start — captures the current TCP pose on the robot as the
      // follow-home, and flips the server into streaming mode.
      this.sendRaw('{"cmd":"phone_start"}');
    });
    ws.on('message', (_err: BusinessError, value: string | ArrayBuffer): void => {
      if (typeof value !== 'string') return;
      this.handleIncoming(value);
    });
    ws.on('error', (err: BusinessError): void => {
      // HarmonyOS netstack's error messages are often empty; log every
      // field to hilog so we at least have something to grep.
      const code = err ? err.code : 0;
      const msg = err && err.message ? err.message : '';
      const name = err ? (err.name || '') : '';
      hilog.error(0x1, 'RobotClient',
        'ws error code=%{public}d name=%{public}s msg=%{public}s',
        code, name, msg);
      let detail = msg || ('code ' + code);
      // Code 200 is HarmonyOS's generic "connect failed" — annotate it
      // with a likely cause since the raw code is opaque.
      if (code === 200) detail += ' · TCP 连接失败（路由/防火墙/ping 测过吗？）';
      this.emitStatus('dead', 'ROBOT · ERROR', detail);
    });
    ws.on('close', (_err: BusinessError, v: webSocket.CloseResult): void => {
      const detail = v
        ? 'code=' + v.code + (v.reason ? ' · ' + v.reason : '')
        : 'closed';
      this.emitStatus('dead', 'ROBOT · DISCONNECTED', detail + ' · 2s 后重试');
      this.active = false;
      this.ws = null;
      if (this.autoReconnect) this.scheduleReconnect();
    });

    try {
      const ok = await ws.connect(this.url);
      if (!ok) {
        this.emitStatus('dead', 'ROBOT · CONNECT FAIL',
          'connect() returned false — 检查 IP/端口以及机器人是否在运行 teach_monitor.py');
      }
    } catch (e) {
      const err = e as BusinessError;
      this.emitStatus('dead', 'ROBOT · CONNECT ERR',
        err.message || ('code ' + err.code));
      if (this.autoReconnect) this.scheduleReconnect();
    }
  }

  async disconnect(): Promise<void> {
    this.autoReconnect = false;
    if (this.reconnectTimer >= 0) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = -1;
    }
    // Send an explicit stop so the robot halts motion immediately
    // rather than waiting for the 0.5s watchdog to fire.
    this.sendRaw('{"cmd":"phone_stop"}');
    if (this.ws) {
      try { await this.ws.close(); } catch (_) {}
      this.ws = null;
    }
    this.active = false;
    this.emitStatus('idle', 'ROBOT · IDLE', '');
  }

  private scheduleReconnect(): void {
    if (this.reconnectTimer >= 0) return;
    this.reconnectTimer = setTimeout((): void => {
      this.reconnectTimer = -1;
      if (this.autoReconnect) {
        this.connect();
      }
    }, 2000);
  }

  private handleIncoming(raw: string): void {
    // Server sends ack dicts like {"phone":"enabled","home":[...]},
    // {"phone":"disabled"}, or state dicts after each movel success.
    if (raw.indexOf('"phone":"enabled"') !== -1) {
      this.active = true;
      this.emitStatus('active', 'ROBOT ACTIVE', '已校准 · 帧:' + this.framesSent);
    } else if (raw.indexOf('"phone":"disabled"') !== -1) {
      this.active = false;
      this.emitStatus('linked', 'ROBOT · STOPPED',
        '服务端 phone_stop（超时/跳变/急停）');
    } else if (raw.indexOf('"error"') !== -1) {
      this.emitStatus('dead', 'ROBOT · SERVER ERR', raw.substring(0, 180));
    }
    // State broadcasts ({"x":...,"y":...}) are silently consumed; we
    // don't mirror TCP pose in this UI yet.
  }

  /**
   * Send a raw JSON string to the robot. Returns false if the socket
   * isn't open so the caller can drop silently (jumpy pose frames
   * during disconnect shouldn't queue up anywhere).
   */
  sendRaw(payload: string): boolean {
    const ws = this.ws;
    if (!ws || (this.state !== 'linked' && this.state !== 'active')) return false;
    try {
      ws.send(payload);
      if (payload.indexOf('"cmd":"phone_frame"') !== -1) {
        this.framesSent++;
        // Refresh active label every ~32 frames (~2s at 15fps) so the
        // frame counter visibly ticks without spamming state listeners.
        if (this.active && (this.framesSent & 31) === 1) {
          this.emitStatus('active', 'ROBOT ACTIVE', '已校准 · 帧:' + this.framesSent);
        }
      }
      return true;
    } catch (_) {
      return false;
    }
  }

  // Convenience builders — arm.html calls these via ArmJsBridge.

  sendPhoneFrame(sx: number, sy: number, sz: number,
                 wx: number, wy: number, wz: number,
                 vis: number, ts: number): boolean {
    return this.sendRaw(
      '{"cmd":"phone_frame",' +
      '"ts":' + ts + ',' +
      '"sx":' + sx.toFixed(4) + ',"sy":' + sy.toFixed(4) + ',"sz":' + sz.toFixed(4) + ',' +
      '"wx":' + wx.toFixed(4) + ',"wy":' + wy.toFixed(4) + ',"wz":' + wz.toFixed(4) + ',' +
      '"vis":' + vis.toFixed(3) +
      '}'
    );
  }

  sendPhoneRot(droll: number, dpitch: number, dyaw: number): void {
    this.sendRaw(
      '{"cmd":"phone_rot","droll":' + droll.toFixed(4) +
      ',"dpitch":' + dpitch.toFixed(4) +
      ',"dyaw":' + dyaw.toFixed(4) + '}'
    );
  }

  sendPhoneStop(): void {
    this.sendRaw('{"cmd":"phone_stop"}');
  }

  sendEstop(hard: boolean): void {
    this.sendRaw('{"cmd":"estop","hard":' + (hard ? 'true' : 'false') + '}');
  }
}
