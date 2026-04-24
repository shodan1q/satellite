/**
 * UdpSync — last-resort transport via UDP broadcast over WiFi.
 *
 * HarmonyOS's distributed data APIs (DDO / KVStore) have been flaky on
 * the test devices — softbus doesn't build a real data channel between
 * the two apps even when the trust relationship is fine. This bypasses
 * all of that: plain UDP packets on port 9764, broadcast to the local
 * subnet. Two devices on the same WiFi network exchange state in real
 * time with no pairing, no permissions beyond INTERNET (already held).
 *
 * Protocol: JSON-encoded HandoffPayload. Idempotent — each packet
 * carries the full state the sender wants the receiver to mirror.
 */

import socket from '@ohos.net.socket';
import { BusinessError } from '@ohos.base';
import { hilog } from '@kit.PerformanceAnalysisKit';
import { HandoffPayload } from './Handoff';

const TAG = 'UdpSync';
const PORT = 9764;

export type UdpHandler = (payload: HandoffPayload) => void;

function stringToArrayBuffer(s: string): ArrayBuffer {
  const bytes: number[] = [];
  for (let i = 0; i < s.length; i++) {
    const c = s.charCodeAt(i);
    if (c < 0x80) {
      bytes.push(c);
    } else if (c < 0x800) {
      bytes.push(0xc0 | (c >> 6), 0x80 | (c & 0x3f));
    } else {
      bytes.push(0xe0 | (c >> 12), 0x80 | ((c >> 6) & 0x3f), 0x80 | (c & 0x3f));
    }
  }
  const buf = new ArrayBuffer(bytes.length);
  const view = new Uint8Array(buf);
  for (let i = 0; i < bytes.length; i++) view[i] = bytes[i];
  return buf;
}

function arrayBufferToString(buf: ArrayBuffer): string {
  const view = new Uint8Array(buf);
  let out = '';
  let i = 0;
  while (i < view.length) {
    const b = view[i++];
    if (b < 0x80) {
      out += String.fromCharCode(b);
    } else if (b < 0xe0) {
      const b2 = view[i++] ?? 0;
      out += String.fromCharCode(((b & 0x1f) << 6) | (b2 & 0x3f));
    } else {
      const b2 = view[i++] ?? 0;
      const b3 = view[i++] ?? 0;
      out += String.fromCharCode(((b & 0x0f) << 12) | ((b2 & 0x3f) << 6) | (b3 & 0x3f));
    }
  }
  return out;
}

export class UdpStats {
  bound: boolean = false;
  sent: number = 0;
  received: number = 0;
  lastError: string = '';
  lastSentAt: number = 0;
  lastRecvAt: number = 0;
}

export class UdpSync {
  private static _instance: UdpSync | null = null;
  private udp: socket.UDPSocket | null = null;
  private bound: boolean = false;
  private handler: UdpHandler | null = null;
  private lastSelfNonce: string = '';
  private sentCount: number = 0;
  private recvCount: number = 0;
  private lastError: string = '';
  private lastSentAt: number = 0;
  private lastRecvAt: number = 0;

  static get instance(): UdpSync {
    if (UdpSync._instance === null) UdpSync._instance = new UdpSync();
    return UdpSync._instance as UdpSync;
  }

  getStats(): UdpStats {
    const s = new UdpStats();
    s.bound = this.bound;
    s.sent = this.sentCount;
    s.received = this.recvCount;
    s.lastError = this.lastError;
    s.lastSentAt = this.lastSentAt;
    s.lastRecvAt = this.lastRecvAt;
    return s;
  }

  setHandler(h: UdpHandler): void { this.handler = h; }

  async bind(): Promise<boolean> {
    if (this.bound) return true;
    try {
      this.udp = socket.constructUDPSocketInstance();
      const addr: socket.NetAddress = { address: '0.0.0.0', port: PORT, family: 1 };
      await this.udp.bind(addr);
      // Enable broadcast so send to 255.255.255.255 actually reaches the subnet.
      await this.udp.setExtraOptions({ broadcast: true });
      this.udp.on('message', (value: socket.SocketMessageInfo): void => {
        this.onMessage(value);
      });
      this.bound = true;
      hilog.info(0x1, TAG, 'UDP socket bound on port %{public}d', PORT);
      return true;
    } catch (err) {
      const e = err as BusinessError;
      this.lastError = 'bind: ' + (e.message ?? String(e.code ?? '?'));
      hilog.warn(0x1, TAG, 'bind failed code=%{public}d msg=%{public}s',
        e.code ?? -1, e.message ?? '');
      return false;
    }
  }

  private onMessage(value: socket.SocketMessageInfo): void {
    this.recvCount++;
    this.lastRecvAt = Date.now();
    if (this.handler === null) return;
    try {
      const text = arrayBufferToString(value.message);
      const parsed = JSON.parse(text) as HandoffPayload;
      // Drop our own echoes — every outgoing packet carries a nonce we
      // remember; packets tagged with the same nonce bounce back from
      // the broadcast and we ignore them.
      const nonce = parsed.issuedAt !== undefined ? String(parsed.issuedAt) : '';
      if (nonce.length > 0 && nonce === this.lastSelfNonce) return;
      const p = new HandoffPayload();
      p.kind = parsed.kind ?? 'workshop';
      p.rocketId = parsed.rocketId ?? '';
      p.satelliteId = parsed.satelliteId ?? '';
      p.entryKind = parsed.entryKind ?? 'rocket';
      p.explode = parsed.explode ?? 0;
      p.selectedPart = parsed.selectedPart ?? '';
      p.controlMode = parsed.controlMode ?? 'touch';
      p.issuedAt = parsed.issuedAt ?? 0;
      p.syncTag = parsed.syncTag ?? 'sync';
      this.handler(p);
    } catch (err) { /* drop malformed */ }
  }

  broadcast(payload: HandoffPayload): void {
    if (!this.bound || this.udp === null) {
      this.lastError = 'not bound';
      return;
    }
    try {
      payload.issuedAt = Date.now();
      this.lastSelfNonce = String(payload.issuedAt);
      const json = JSON.stringify(payload);
      const buf = stringToArrayBuffer(json);
      const target: socket.NetAddress = { address: '255.255.255.255', port: PORT, family: 1 };
      const options: socket.UDPSendOptions = { data: buf, address: target };
      this.sentCount++;
      this.lastSentAt = Date.now();
      this.udp.send(options).catch((err: BusinessError): void => {
        this.lastError = 'send: ' + (err.message ?? String(err.code ?? '?'));
        hilog.warn(0x1, TAG, 'send failed code=%{public}d msg=%{public}s',
          err.code ?? -1, err.message ?? '');
      });
    } catch (err) {
      const e = err as BusinessError;
      this.lastError = 'broadcast: ' + (e.message ?? String(e.code ?? '?'));
    }
  }

  async close(): Promise<void> {
    if (this.udp === null) return;
    try { await this.udp.close(); } catch (e) { /* ignore */ }
    this.bound = false;
    this.udp = null;
  }
}
