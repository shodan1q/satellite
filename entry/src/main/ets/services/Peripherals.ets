/**
 * Peripherals — in-memory simulated state for the Device page's
 * three smart-home-accessory product stubs (all Hisense SoC based):
 *
 *   • 磁悬浮卫星摆件  (Hi3861 · BLE-mesh / Wi-Fi6)
 *   • 漂浮氦气云摆件   (Hi3861 · BLE)
 *   • 生态缸摆件       (Hi3516DV300 · Wi-Fi + camera)
 *
 * The values here drive the UI; real hardware integration goes in a
 * follow-up once the physical products ship. Each device's state lives
 * in a typed class so the UI can bind individual fields without
 * casting, and the whole snapshot is emitted to subscribers on any
 * update so the page rerenders atomically.
 */

export class LevitateState {
  online: boolean = true;
  on: boolean = true;
  rpm: number = 42;              // 0..120 — orbit/spin rate
  mode: string = 'orbit';        // 'orbit' | 'spin' | 'static'
  ledHue: number = 210;          // 0..360 — ring-light hue
  ledBright: number = 75;        // 0..100
  battery: number = 86;          // 0..100, charging dock backup
  levitHeightMm: number = 14;    // 0..30 — calibration target
  firmware: string = '1.4.2';
}

export class CloudState {
  online: boolean = true;
  on: boolean = true;
  brightness: number = 60;       // 0..100
  warmthK: number = 5200;        // 2700..6500 color temp
  breathing: boolean = false;    // slow pulsing effect
  fanSpeed: number = 35;         // 0..100 — gentle updraft to float it
  gasPercent: number = 92;       // 0..100 — helium reserve
  firmware: string = '0.9.7';
}

export class TerrariumState {
  online: boolean = true;
  on: boolean = true;
  tempC: number = 24.5;          // current interior temp
  targetTempC: number = 25.0;    // thermostat target
  humidity: number = 78;         // 0..100 %RH
  dayMode: boolean = true;       // day (grow) vs night (dim)
  autoMist: boolean = true;
  mistMinutes: number = 15;      // misting cadence
  cameraOn: boolean = true;
  firmware: string = '2.1.0';
}

export class PeripheralsSnapshot {
  levitate: LevitateState = new LevitateState();
  cloud: CloudState = new CloudState();
  terrarium: TerrariumState = new TerrariumState();
  pairingCount: number = 3;      // how many connected
  lastSyncAt: number = Date.now();
}

export type PeripheralsListener = (s: PeripheralsSnapshot) => void;

export class Peripherals {
  private static _inst: Peripherals | null = null;
  static get instance(): Peripherals {
    if (Peripherals._inst === null) Peripherals._inst = new Peripherals();
    return Peripherals._inst;
  }

  private snapshot: PeripheralsSnapshot = new PeripheralsSnapshot();
  private listeners: PeripheralsListener[] = [];

  snap(): PeripheralsSnapshot { return this.snapshot; }

  subscribe(cb: PeripheralsListener): () => void {
    this.listeners.push(cb);
    cb(this.snapshot);
    return (): void => {
      const i = this.listeners.indexOf(cb);
      if (i >= 0) this.listeners.splice(i, 1);
    };
  }

  // Mutators — each returns a fresh snapshot object so ArkTS's
  // reference-equality @State diff fires cleanly. Cheap because the
  // snapshot itself is small and nested child objects share refs.
  updateLevitate(fn: (s: LevitateState) => void): void {
    fn(this.snapshot.levitate);
    this.touch();
  }
  updateCloud(fn: (s: CloudState) => void): void {
    fn(this.snapshot.cloud);
    this.touch();
  }
  updateTerrarium(fn: (s: TerrariumState) => void): void {
    fn(this.snapshot.terrarium);
    this.touch();
  }

  private touch(): void {
    this.snapshot.lastSyncAt = Date.now();
    // Emit a shallow-new snapshot so subscribers treat it as a new value.
    const next = new PeripheralsSnapshot();
    next.levitate = this.snapshot.levitate;
    next.cloud = this.snapshot.cloud;
    next.terrarium = this.snapshot.terrarium;
    next.pairingCount = this.snapshot.pairingCount;
    next.lastSyncAt = this.snapshot.lastSyncAt;
    this.snapshot = next;
    for (const l of this.listeners) { try { l(next); } catch (e) { /* swallow */ } }
  }
}
