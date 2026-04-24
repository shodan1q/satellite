/**
 * NebulaSync — reactive state with real multi-device sync.
 *
 * Transport: @ohos.data.distributedKVStore. Each sync'd field is a key
 * in a SingleKVStore with autoSync=true; when the controller writes,
 * HarmonyOS propagates to the peer's copy, and the `dataChange` event
 * fires on the peer so we can replay into the local snapshot.
 *
 * We tried @ohos.data.distributedDataObject first but its change event
 * was unreliable on this build — the KVStore equivalent fires correctly
 * and supports explicit `sync(deviceIds, mode)` as a forced fallback.
 *
 * Public API unchanged: pages still call updateExplode / updateRocket /
 * subscribe / bindSession exactly as before.
 */

import distributedKVStore from '@ohos.data.distributedKVStore';
import distributedDeviceManager from '@ohos.distributedDeviceManager';
import common from '@ohos.app.ability.common';
import { BusinessError } from '@ohos.base';
import { hilog } from '@kit.PerformanceAnalysisKit';
import { HandoffPayload } from './Handoff';
import { UdpStats, UdpSync } from './UdpSync';

const TAG = 'NebulaSync';
const BUNDLE = 'com.yu.satellite';
const STORE_ID = 'nebula_state';

// KV keys — kept simple ASCII strings.
const K_ROCKET = 'currentRocket';
const K_SAT = 'currentSatellite';
const K_EXPLODE = 'explode';
const K_SEL_PART = 'selectedPart';
const K_CTL_MODE = 'controlMode';
const K_CONTROLLER = 'controller';
const K_XFER_AT = 'controllerTransferAt';
const K_BUILD = 'buildJson';
const K_FLIGHT = 'flightJson';
const K_LED = 'ledJson';

export class BuildPart {
  partId: string = '';
  kind: string = '';
  display: string = '';
  color: string = '#FFFFFF';
  height: number = 1;
  radiusTop: number = 0.5;
  radiusBottom: number = 0.5;
}

export class FlightState {
  x: number = 0;
  y: number = 0;
  z: number = 0;
  yaw: number = 0;
  mode: string = 'manual';
}

export class LedState {
  mode: string = 'day';
  brightness: number = 72;
  color: string = '#FFFFFF';
  speed: number = 1;
}

export class DeviceTelemetry {
  batteryPercent: number = 82;
  altitude: number = 0;
  rssi: number = -52;
  temperature: number = 33;
  connected: boolean = false;
}

export class SafetyState {
  locked: boolean = false;
  lowBattery: boolean = false;
  disconnected: boolean = true;
  outOfSafeZone: boolean = false;
}

export class NebulaSnapshot {
  controller: string = 'phone';
  controllerTransferAt: number = 0;
  currentRocket: string = 'falcon-9';
  currentSatellite: string = 'hubble';
  build: BuildPart[] = [];
  flight: FlightState = new FlightState();
  led: LedState = new LedState();
  device: DeviceTelemetry = new DeviceTelemetry();
  safety: SafetyState = new SafetyState();
  role: string = 'solo';
  peerCount: number = 0;
  explode: number = 0;
  selectedPart: string = '';
  controlMode: string = 'touch';
}

export type SyncListener = (snap: NebulaSnapshot) => void;

export class SyncDiagnostics {
  ddoCreated: boolean = false;       // KVStore manager created
  sessionJoined: boolean = false;    // KVStore opened
  peerCount: number = 0;
  lastError: string = '';
  lastPushAt: number = 0;
  lastRecvAt: number = 0;
  lastStatusAt: number = 0;
  lastStatus: string = '';
  // UDP transport — usually the only one that actually works.
  udpBound: boolean = false;
  udpSent: number = 0;
  udpReceived: number = 0;
  udpLastError: string = '';
}

function cloneSnap(s: NebulaSnapshot): NebulaSnapshot {
  const next = new NebulaSnapshot();
  next.controller = s.controller;
  next.controllerTransferAt = s.controllerTransferAt;
  next.currentRocket = s.currentRocket;
  next.currentSatellite = s.currentSatellite;
  next.role = s.role;
  next.peerCount = s.peerCount;
  next.explode = s.explode;
  next.selectedPart = s.selectedPart;
  next.controlMode = s.controlMode;
  next.build = s.build.slice();
  const f = new FlightState();
  f.x = s.flight.x; f.y = s.flight.y; f.z = s.flight.z;
  f.yaw = s.flight.yaw; f.mode = s.flight.mode;
  next.flight = f;
  const l = new LedState();
  l.mode = s.led.mode; l.brightness = s.led.brightness;
  l.color = s.led.color; l.speed = s.led.speed;
  next.led = l;
  const d = new DeviceTelemetry();
  d.batteryPercent = s.device.batteryPercent;
  d.altitude = s.device.altitude;
  d.rssi = s.device.rssi;
  d.temperature = s.device.temperature;
  d.connected = s.device.connected;
  next.device = d;
  const sf = new SafetyState();
  sf.locked = s.safety.locked;
  sf.lowBattery = s.safety.lowBattery;
  sf.disconnected = s.safety.disconnected;
  sf.outOfSafeZone = s.safety.outOfSafeZone;
  next.safety = sf;
  return next;
}

export class NebulaSync {
  private static _instance: NebulaSync | null = null;
  private kvManager: distributedKVStore.KVManager | null = null;
  private kvStore: distributedKVStore.SingleKVStore | null = null;
  private listeners: SyncListener[] = [];
  private snapshot: NebulaSnapshot = new NebulaSnapshot();
  private ready: boolean = false;
  private applyingRemote: boolean = false;
  private lastCtx: common.UIAbilityContext | null = null;
  private peerPollTimer: number = -1;
  private diagDdoCreated: boolean = false;
  private diagSessionJoined: boolean = false;
  private diagLastError: string = '';
  private diagLastPushAt: number = 0;
  private diagLastRecvAt: number = 0;
  private diagLastStatusAt: number = 0;
  private diagLastStatus: string = '';

  private constructor() {
    this.snapshot = new NebulaSnapshot();
    this.snapshot.controllerTransferAt = Date.now();
  }

  static get instance(): NebulaSync {
    if (NebulaSync._instance === null) NebulaSync._instance = new NebulaSync();
    return NebulaSync._instance as NebulaSync;
  }

  static defaultSnapshot(): NebulaSnapshot {
    const s = new NebulaSnapshot();
    s.controllerTransferAt = Date.now();
    return s;
  }

  async init(ctx: common.UIAbilityContext): Promise<void> {
    if (this.ready) return;
    this.lastCtx = ctx;
    this.ready = true;
    this.emit();

    // UDP broadcast sync — doesn't depend on any HarmonyOS distributed
    // service, just needs INTERNET permission + same WiFi. We wire it
    // up at launch so controller/observer don't need to do anything
    // extra. getKVStore still runs via bindSession when the user opens
    // the handoff sheet (best-effort; UDP carries the weight).
    UdpSync.instance.setHandler((payload: HandoffPayload): void => {
      // Only observers apply remote broadcasts — solo and controller
      // ignore incoming (controller is the authority).
      if (this.snapshot.role === 'observer') {
        this.applyRemotePayload(payload);
      }
    });
    UdpSync.instance.bind().then((ok: boolean): void => {
      hilog.info(0x1, TAG, 'UDP transport %{public}s', ok ? 'ready' : 'unavailable');
    });
  }

  subscribe(listener: SyncListener): () => void {
    this.listeners.push(listener);
    listener(this.snapshot);
    return (): void => {
      const i = this.listeners.indexOf(listener);
      if (i >= 0) this.listeners.splice(i, 1);
    };
  }

  private emit(): void {
    const s = this.snapshot;
    for (const l of this.listeners) {
      try { l(s); } catch (e) { /* swallow */ }
    }
  }

  get state(): NebulaSnapshot {
    return this.snapshot;
  }

  getDiagnostics(): SyncDiagnostics {
    const d = new SyncDiagnostics();
    d.ddoCreated = this.diagDdoCreated;
    d.sessionJoined = this.diagSessionJoined;
    d.peerCount = this.snapshot.peerCount;
    d.lastError = this.diagLastError;
    d.lastPushAt = this.diagLastPushAt;
    d.lastRecvAt = this.diagLastRecvAt;
    d.lastStatusAt = this.diagLastStatusAt;
    d.lastStatus = this.diagLastStatus;
    const us: UdpStats = UdpSync.instance.getStats();
    d.udpBound = us.bound;
    d.udpSent = us.sent;
    d.udpReceived = us.received;
    d.udpLastError = us.lastError;
    if (d.udpLastError.length > 0 && d.lastError.length === 0) d.lastError = 'UDP: ' + d.udpLastError;
    return d;
  }

  /**
   * Open the distributed KV store. Call AFTER DISTRIBUTED_DATASYNC is
   * granted — getKVStore on some builds silently fails without it.
   */
  async bindSession(ctx: common.UIAbilityContext | null): Promise<boolean> {
    if (ctx !== null) this.lastCtx = ctx;
    if (this.lastCtx === null) return false;

    // Create the KV manager (cheap, idempotent in spirit).
    if (this.kvManager === null) {
      try {
        const cfg: distributedKVStore.KVManagerConfig = {
          context: this.lastCtx,
          bundleName: BUNDLE
        };
        this.kvManager = distributedKVStore.createKVManager(cfg);
        this.diagDdoCreated = true;
        hilog.info(0x1, TAG, 'KV manager created');
      } catch (err) {
        const e = err as BusinessError;
        this.diagLastError = 'createKVManager: ' + (e.message ?? String(e.code ?? 'unknown'));
        hilog.warn(0x1, TAG, 'createKVManager failed code=%{public}d', e.code ?? -1);
        return false;
      }
    }

    // Open the store — autoSync pushes local writes to peers, securityLevel
    // S1 is the lowest (no encryption) which is fine for non-sensitive
    // state like "which rocket is selected".
    if (this.kvStore === null) {
      try {
        const opts: distributedKVStore.Options = {
          createIfMissing: true,
          encrypt: false,
          backup: false,
          autoSync: true,
          kvStoreType: distributedKVStore.KVStoreType.SINGLE_VERSION,
          securityLevel: distributedKVStore.SecurityLevel.S1
        };
        this.kvStore = await this.kvManager.getKVStore<distributedKVStore.SingleKVStore>(STORE_ID, opts);
        hilog.info(0x1, TAG, 'KV store opened');

        // Remote-only subscription so we don't echo our own writes.
        this.kvStore.on('dataChange',
          distributedKVStore.SubscribeType.SUBSCRIBE_TYPE_REMOTE,
          (data: distributedKVStore.ChangeNotification): void => {
            this.onDataChange(data);
          });

        this.kvStore.on('syncComplete',
          (results: Array<[string, number]>): void => {
            // results is [[deviceId, statusCode], ...]
            this.diagLastStatusAt = Date.now();
            let anyOk = false;
            for (const r of results) {
              if (r[1] === 0) anyOk = true;
              hilog.info(0x1, TAG, 'syncComplete dev=%{public}s code=%{public}d', r[0], r[1]);
            }
            if (anyOk) this.diagLastStatus = 'ok';
            else this.diagLastStatus = 'err';
          });

        this.diagSessionJoined = true;
      } catch (err) {
        const e = err as BusinessError;
        this.diagLastError = 'getKVStore: ' + (e.message ?? String(e.code ?? 'unknown'));
        hilog.warn(0x1, TAG, 'getKVStore failed code=%{public}d msg=%{public}s',
          e.code ?? -1, e.message ?? '');
        return false;
      }
    }

    // SINGLE_VERSION's replication channel activates automatically via
    // on('dataChange', SUBSCRIBE_TYPE_REMOTE, ...) + periodic sync() —
    // no explicit subscribe() method exists on SingleKVStore.
    //
    // Seed all keys locally so observer devices joining late have something
    // to compare against (observer never writes; this only runs on non-obs).
    if (this.snapshot.role !== 'observer') this.republishAll();

    // Force an initial PUSH_PULL to every trusted peer so late joiners
    // catch up without waiting for the next mutation.
    await this.syncToPeers(distributedKVStore.SyncMode.PUSH_PULL);

    this.startPeerPolling();
    return true;
  }


  // ---------- controller-driven writes ----------

  updateBuild(build: BuildPart[]): void {
    const next = cloneSnap(this.snapshot);
    next.build = build.slice();
    this.snapshot = next;
    this.pushField(K_BUILD, JSON.stringify(build));
    this.emit();
  }

  updateRocket(id: string): void {
    const next = cloneSnap(this.snapshot);
    next.currentRocket = id;
    this.snapshot = next;
    this.pushField(K_ROCKET, id);
    this.emit();
  }

  updateSatellite(id: string): void {
    const next = cloneSnap(this.snapshot);
    next.currentSatellite = id;
    this.snapshot = next;
    this.pushField(K_SAT, id);
    this.emit();
  }

  updateFlight(x: number, y: number, z: number, yaw: number, mode: string): void {
    const next = cloneSnap(this.snapshot);
    next.flight.x = x; next.flight.y = y; next.flight.z = z;
    next.flight.yaw = yaw; next.flight.mode = mode;
    this.snapshot = next;
    this.pushField(K_FLIGHT, JSON.stringify(next.flight));
    this.emit();
  }

  updateFlightMode(mode: string): void {
    const f = this.snapshot.flight;
    this.updateFlight(f.x, f.y, f.z, f.yaw, mode);
  }

  updateLedMode(mode: string): void {
    const next = cloneSnap(this.snapshot);
    next.led.mode = mode;
    this.snapshot = next;
    this.pushField(K_LED, JSON.stringify(next.led));
    this.emit();
  }

  updateLedBrightness(brightness: number): void {
    const next = cloneSnap(this.snapshot);
    next.led.brightness = brightness;
    this.snapshot = next;
    this.pushField(K_LED, JSON.stringify(next.led));
    this.emit();
  }

  updateExplode(v: number): void {
    const clamped = Math.max(0, Math.min(1, v));
    if (this.snapshot.explode === clamped) return;
    const next = cloneSnap(this.snapshot);
    next.explode = clamped;
    this.snapshot = next;
    this.pushField(K_EXPLODE, clamped);
    this.emit();
  }

  updateSelectedPart(id: string): void {
    if (this.snapshot.selectedPart === id) return;
    const next = cloneSnap(this.snapshot);
    next.selectedPart = id;
    this.snapshot = next;
    this.pushField(K_SEL_PART, id);
    this.emit();
  }

  updateControlMode(mode: string): void {
    if (mode !== 'touch' && mode !== 'motion') return;
    if (this.snapshot.controlMode === mode) return;
    const next = cloneSnap(this.snapshot);
    next.controlMode = mode;
    this.snapshot = next;
    this.pushField(K_CTL_MODE, mode);
    this.emit();
  }

  transferController(to: string): void {
    const next = cloneSnap(this.snapshot);
    next.controller = to;
    next.controllerTransferAt = Date.now();
    this.snapshot = next;
    this.pushField(K_CONTROLLER, to);
    this.pushField(K_XFER_AT, next.controllerTransferAt);
    this.emit();
  }

  setRole(role: string): void {
    if (role !== 'solo' && role !== 'controller' && role !== 'observer') return;
    if (this.snapshot.role === role) return;
    const next = cloneSnap(this.snapshot);
    next.role = role;
    this.snapshot = next;
    this.emit();
  }

  setPeerCount(count: number): void {
    const safe = Math.max(0, Math.floor(count));
    if (this.snapshot.peerCount === safe) return;
    const next = cloneSnap(this.snapshot);
    next.peerCount = safe;
    this.snapshot = next;
    this.emit();
  }

  // ---------- KV bridge ----------

  private pushField(key: string, value: string | number | boolean): void {
    if (this.applyingRemote) return;
    if (this.snapshot.role === 'observer') return;

    // Primary transport: UDP broadcast. Idempotent — every packet
    // carries the full current state, so observer always converges.
    this.broadcastUdp();
    this.diagLastPushAt = Date.now();

    // Secondary (best-effort): KVStore put. If it happens to work on
    // this build great, otherwise UDP already carried the update.
    if (this.kvStore === null) return;
    const store = this.kvStore;
    store.put(key, value).then((): void => {
      hilog.info(0x1, TAG, 'put %{public}s', key);
      this.syncToPeers(distributedKVStore.SyncMode.PUSH_ONLY).then((): void => {});
    }).catch((err: BusinessError): void => {
      // Don't overwrite diagLastError from UDP; KVStore failing is
      // expected on broken builds, we don't want to confuse the user.
      hilog.warn(0x1, TAG, 'put %{public}s failed code=%{public}d', key, err.code ?? -1);
    });
  }

  private broadcastUdp(): void {
    const s = this.snapshot;
    const p = new HandoffPayload();
    p.kind = 'workshop';
    p.rocketId = s.currentRocket;
    p.satelliteId = s.currentSatellite;
    p.explode = s.explode;
    p.selectedPart = s.selectedPart;
    p.controlMode = s.controlMode;
    p.syncTag = 'sync';
    UdpSync.instance.broadcast(p);
  }

  private async syncToPeers(mode: distributedKVStore.SyncMode): Promise<void> {
    if (this.kvStore === null) return;
    let peerIds: string[] = [];
    try {
      const dm = distributedDeviceManager.createDeviceManager(BUNDLE);
      const raw = dm.getAvailableDeviceListSync();
      for (const d of raw) {
        if (d.networkId !== null && d.networkId !== undefined && d.networkId.length > 0) {
          peerIds.push(d.networkId);
        }
      }
      try { distributedDeviceManager.releaseDeviceManager(dm); } catch (e) { /* ignore */ }
    } catch (e) { /* ignore */ }
    if (peerIds.length === 0) return;
    try {
      this.kvStore.sync(peerIds, mode);
    } catch (err) {
      const e = err as BusinessError;
      hilog.warn(0x1, TAG, 'sync failed code=%{public}d', e.code ?? -1);
    }
  }

  private republishAll(): void {
    if (this.kvStore === null) return;
    const s = this.snapshot;
    this.pushField(K_ROCKET, s.currentRocket);
    this.pushField(K_SAT, s.currentSatellite);
    this.pushField(K_CONTROLLER, s.controller);
    this.pushField(K_XFER_AT, s.controllerTransferAt);
    this.pushField(K_EXPLODE, s.explode);
    this.pushField(K_SEL_PART, s.selectedPart);
    this.pushField(K_CTL_MODE, s.controlMode);
    this.pushField(K_BUILD, JSON.stringify(s.build));
    this.pushField(K_FLIGHT, JSON.stringify(s.flight));
    this.pushField(K_LED, JSON.stringify(s.led));
  }

  /**
   * Remote peer wrote one or more keys — replay into local snapshot.
   */
  private onDataChange(data: distributedKVStore.ChangeNotification): void {
    // Combine inserts + updates; treat identically.
    const next = cloneSnap(this.snapshot);
    let dirty = false;
    const apply = (entries: distributedKVStore.Entry[]): void => {
      for (const e of entries) {
        const k = e.key;
        const v = e.value.value;
        if (k === K_ROCKET) { next.currentRocket = v as string; dirty = true; }
        else if (k === K_SAT) { next.currentSatellite = v as string; dirty = true; }
        else if (k === K_CONTROLLER) { next.controller = v as string; dirty = true; }
        else if (k === K_XFER_AT) { next.controllerTransferAt = v as number; dirty = true; }
        else if (k === K_EXPLODE) { next.explode = v as number; dirty = true; }
        else if (k === K_SEL_PART) { next.selectedPart = v as string; dirty = true; }
        else if (k === K_CTL_MODE) { next.controlMode = v as string; dirty = true; }
        else if (k === K_BUILD) {
          try { next.build = JSON.parse(v as string) as BuildPart[]; dirty = true; } catch (err) { /* skip */ }
        }
        else if (k === K_FLIGHT) {
          try {
            const p = JSON.parse(v as string) as FlightState;
            next.flight.x = p.x ?? 0; next.flight.y = p.y ?? 0; next.flight.z = p.z ?? 0;
            next.flight.yaw = p.yaw ?? 0; next.flight.mode = p.mode ?? 'manual';
            dirty = true;
          } catch (err) { /* skip */ }
        }
        else if (k === K_LED) {
          try {
            const p = JSON.parse(v as string) as LedState;
            next.led.mode = p.mode ?? 'day'; next.led.brightness = p.brightness ?? 72;
            next.led.color = p.color ?? '#FFFFFF'; next.led.speed = p.speed ?? 1;
            dirty = true;
          } catch (err) { /* skip */ }
        }
      }
    };
    apply(data.insertEntries);
    apply(data.updateEntries);
    if (!dirty) return;
    this.diagLastRecvAt = Date.now();
    this.applyingRemote = true;
    try {
      this.snapshot = next;
      this.emit();
    } finally {
      this.applyingRemote = false;
    }
  }

  /**
   * Apply a peer-pushed HandoffPayload (one-shot handoff path). Not used
   * by KVStore live sync but kept for the startAbility handoff path.
   */
  applyRemotePayload(payload: HandoffPayload): void {
    this.applyingRemote = true;
    try {
      const next = cloneSnap(this.snapshot);
      let dirty = false;
      if (payload.rocketId.length > 0 && payload.rocketId !== next.currentRocket) {
        next.currentRocket = payload.rocketId; dirty = true;
      }
      if (payload.satelliteId.length > 0 && payload.satelliteId !== next.currentSatellite) {
        next.currentSatellite = payload.satelliteId; dirty = true;
      }
      if (Math.abs(payload.explode - next.explode) > 0.001) {
        next.explode = Math.max(0, Math.min(1, payload.explode)); dirty = true;
      }
      if (payload.selectedPart !== next.selectedPart) {
        next.selectedPart = payload.selectedPart; dirty = true;
      }
      if (payload.controlMode === 'touch' || payload.controlMode === 'motion') {
        if (payload.controlMode !== next.controlMode) { next.controlMode = payload.controlMode; dirty = true; }
      }
      if (dirty) {
        this.snapshot = next;
        this.diagLastRecvAt = Date.now();
        this.emit();
      }
    } finally {
      this.applyingRemote = false;
    }
  }

  private startPeerPolling(): void {
    if (this.peerPollTimer >= 0) return;
    this.peerPollTimer = setInterval((): void => {
      // DM-based peer count as a backup to any KVStore peer-discovery
      // callback — the UI diagnostic strip reads from snapshot.peerCount.
      let trusted = 0;
      try {
        const dm = distributedDeviceManager.createDeviceManager(BUNDLE);
        const raw = dm.getAvailableDeviceListSync();
        for (const d of raw) {
          if (d.networkId && d.networkId.length > 0) trusted++;
        }
        try { distributedDeviceManager.releaseDeviceManager(dm); } catch (e) { /* ignore */ }
        const oldCount = this.snapshot.peerCount;
        if (trusted !== oldCount) {
          this.setPeerCount(trusted);
          // A peer came online after bindSession — re-subscribe so the
          // replication pipe includes them. Without this, peers that
          // join late never propagate their writes to us.
          if (trusted > oldCount && this.kvStore !== null) {
            // PUSH our current state to the newly-online peer. Also
            // request a pull in case they already had state we missed.
            this.syncToPeers(distributedKVStore.SyncMode.PUSH_PULL).then((): void => {});
          }
        }
      } catch (e) { /* ignore */ }

      // Kick a PULL every few ticks so observers pull recent state
      // even if remote autoSync notifications slip through.
      if (this.kvStore !== null && this.snapshot.role === 'observer') {
        this.syncToPeers(distributedKVStore.SyncMode.PULL_ONLY).then((): void => {
          // no-op
        });
      }
    }, 1500);
  }
}
