/**
 * GesturePipeline — hand-gesture detection in the WORLD frame.
 *
 * Key insight: a user flicking the phone "upward" produces an accel peak
 * along device +Y only if the phone is perfectly upright. Tilt the phone
 * forward and the impulse leaks into +Z; hold it flat and it's almost all
 * on +Z. Fixed-axis detection (our old version) therefore misses most
 * real-world flicks.
 *
 * Solution: subscribe to GRAVITY in parallel, use that to build an
 * orthonormal world basis in device coords:
 *     up    = -gravity / |gravity|              (points to sky)
 *     right = (device_X - (device_X · up) * up) / |...|   (horizontal plane)
 *     fwd   = up × right                         (horizontal plane)
 *
 * Project every linear-accel sample onto (up, right) to get worldUp /
 * worldRight. Then flickUp = sustained +worldUp peak, regardless of how
 * the phone is being held.
 *
 * We also emit telemetry (x/y/z raw, magnitude, worldUp, worldRight) so
 * the UI can show a live indicator while the user tunes their motion.
 */

import sensor from '@ohos.sensor';
import { hilog } from '@kit.PerformanceAnalysisKit';

const TAG = 'GesturePipeline';

export class GestureEvent {
  kind: string = '';          // 'flickUp' | 'flickDown' | 'flickLeft' | 'flickRight' | 'shake'
  magnitude: number = 0;
  at: number = 0;
}

export class GestureTelemetry {
  x: number = 0; y: number = 0; z: number = 0;
  mag: number = 0;
  worldUp: number = 0;        // +up / -down
  worldRight: number = 0;     // +right / -left
}

export type GestureListener = (e: GestureEvent) => void;
export type TelemetryListener = (t: GestureTelemetry) => void;

class Sample {
  up: number = 0; right: number = 0; t: number = 0;
}

const WINDOW_MS = 520;
const COOLDOWN_MS = 340;
const FLICK_THRESHOLD = 3.0;
const SHAKE_FLIPS = 3;
const SHAKE_THRESHOLD = 2.4;
const GRAVITY_ALPHA = 0.92;

class Vec3 {
  x: number = 0; y: number = 0; z: number = 0;
}

function vecLen(v: Vec3): number { return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }
function vecDot(a: Vec3, b: Vec3): number { return a.x * b.x + a.y * b.y + a.z * b.z; }

export class GesturePipeline {
  private static _instance: GesturePipeline | null = null;
  private listeners: GestureListener[] = [];
  private telListeners: TelemetryListener[] = [];
  private running: boolean = false;
  private usedAccel: number = -1;
  private usedGravity: boolean = false;
  private buffer: Sample[] = [];
  private lastEmitAt: number = 0;
  // Low-pass gravity (used both for ACCELEROMETER gravity stripping and
  // as a fallback world-up reference if GRAVITY sensor is missing).
  private g: Vec3 = { x: 0, y: 0, z: 9.8 } as Vec3;
  private gravityPrimed: boolean = false;
  private samplesSeen: number = 0;

  static get instance(): GesturePipeline {
    if (GesturePipeline._instance === null) GesturePipeline._instance = new GesturePipeline();
    return GesturePipeline._instance as GesturePipeline;
  }

  subscribe(cb: GestureListener): () => void {
    this.listeners.push(cb);
    if (!this.running) this.start();
    return (): void => {
      const i = this.listeners.indexOf(cb);
      if (i >= 0) this.listeners.splice(i, 1);
      if (this.listeners.length === 0 && this.telListeners.length === 0) this.stop();
    };
  }
  subscribeTelemetry(cb: TelemetryListener): () => void {
    this.telListeners.push(cb);
    if (!this.running) this.start();
    return (): void => {
      const i = this.telListeners.indexOf(cb);
      if (i >= 0) this.telListeners.splice(i, 1);
      if (this.listeners.length === 0 && this.telListeners.length === 0) this.stop();
    };
  }

  private start(): void {
    // Gravity — used to anchor the world frame every sample.
    try {
      sensor.on(
        sensor.SensorId.GRAVITY,
        (data: sensor.GravityResponse): void => {
          this.g.x = data.x; this.g.y = data.y; this.g.z = data.z;
          this.gravityPrimed = true;
        },
        { interval: 40_000_000 }  // 25 Hz is plenty for gravity
      );
      this.usedGravity = true;
      hilog.info(0x1, TAG, 'GRAVITY armed');
    } catch (err) {
      hilog.warn(0x1, TAG, 'GRAVITY unavailable — using low-pass fallback');
    }

    // Primary: LINEAR_ACCELEROMETER (gravity already removed).
    try {
      sensor.on(
        sensor.SensorId.LINEAR_ACCELEROMETER,
        (data: sensor.LinearAccelerometerResponse): void => {
          this.onSample(data.x, data.y, data.z);
        },
        { interval: 20_000_000 }
      );
      this.usedAccel = sensor.SensorId.LINEAR_ACCELEROMETER;
      this.running = true;
      hilog.info(0x1, TAG, 'LINEAR_ACCELEROMETER armed');
      return;
    } catch (err) {
      hilog.warn(0x1, TAG, 'LINEAR_ACCELEROMETER unavailable, falling back');
    }

    // Fallback: raw ACCELEROMETER with gravity stripped ourselves.
    try {
      sensor.on(
        sensor.SensorId.ACCELEROMETER,
        (data: sensor.AccelerometerResponse): void => {
          if (!this.gravityPrimed) {
            this.g.x = data.x; this.g.y = data.y; this.g.z = data.z;
            this.gravityPrimed = true;
          } else if (!this.usedGravity) {
            // No GRAVITY sensor → keep our own estimate via low-pass.
            this.g.x = GRAVITY_ALPHA * this.g.x + (1 - GRAVITY_ALPHA) * data.x;
            this.g.y = GRAVITY_ALPHA * this.g.y + (1 - GRAVITY_ALPHA) * data.y;
            this.g.z = GRAVITY_ALPHA * this.g.z + (1 - GRAVITY_ALPHA) * data.z;
          }
          this.onSample(data.x - this.g.x, data.y - this.g.y, data.z - this.g.z);
        },
        { interval: 20_000_000 }
      );
      this.usedAccel = sensor.SensorId.ACCELEROMETER;
      this.running = true;
      hilog.info(0x1, TAG, 'ACCELEROMETER armed (gravity-stripped)');
    } catch (err) {
      hilog.warn(0x1, TAG, 'no accelerometer at all');
    }
  }

  private stop(): void {
    try { if (this.usedAccel >= 0) sensor.off(this.usedAccel); } catch (e) {}
    try { if (this.usedGravity) sensor.off(sensor.SensorId.GRAVITY); } catch (e) {}
    this.usedAccel = -1;
    this.usedGravity = false;
    this.running = false;
    this.buffer = [];
    this.gravityPrimed = false;
    this.samplesSeen = 0;
  }

  /** Returns { worldUp, worldRight } — components in the world frame. */
  private projectWorld(ax: number, ay: number, az: number): Sample {
    const out = new Sample();
    const gmag = vecLen(this.g);
    if (gmag < 0.1) {
      // No gravity info yet — fall back to raw Y/X as a rough approximation.
      out.up = ay; out.right = ax; out.t = Date.now();
      return out;
    }
    // Up = -gravity normalised.
    const ux = -this.g.x / gmag;
    const uy = -this.g.y / gmag;
    const uz = -this.g.z / gmag;
    const worldUp = ax * ux + ay * uy + az * uz;

    // Right = (deviceX − (deviceX·up) * up) normalised, with deviceX = (1,0,0).
    const dotX = ux; // deviceX.dot(up) = ux*1 + uy*0 + uz*0 = ux
    let rx = 1 - dotX * ux;
    let ry = -dotX * uy;
    let rz = -dotX * uz;
    const rlen = Math.sqrt(rx * rx + ry * ry + rz * rz);
    if (rlen > 0.001) { rx /= rlen; ry /= rlen; rz /= rlen; }
    const worldRight = ax * rx + ay * ry + az * rz;

    out.up = worldUp;
    out.right = worldRight;
    out.t = Date.now();
    return out;
  }

  private onSample(ax: number, ay: number, az: number): void {
    this.samplesSeen++;
    if (this.samplesSeen === 1) {
      hilog.info(0x1, TAG, 'first accel x=%{public}f y=%{public}f z=%{public}f', ax, ay, az);
    }

    const s = this.projectWorld(ax, ay, az);

    if (this.telListeners.length > 0) {
      const t = new GestureTelemetry();
      t.x = ax; t.y = ay; t.z = az;
      t.mag = Math.sqrt(ax * ax + ay * ay + az * az);
      t.worldUp = s.up;
      t.worldRight = s.right;
      for (const l of this.telListeners) {
        try { l(t); } catch (e) { /* swallow */ }
      }
    }

    this.buffer.push(s);
    while (this.buffer.length > 0 && s.t - this.buffer[0].t > WINDOW_MS) this.buffer.shift();

    if (s.t - this.lastEmitAt < COOLDOWN_MS) return;
    if (this.buffer.length < 2) return;

    // Detect.
    let peakUp = 0, peakRight = 0;
    let firstSignUp = 0, firstSignRight = 0;
    let flipsUp = 0;
    let prevSignUp = 0;
    for (const b of this.buffer) {
      if (Math.abs(b.up) > Math.abs(peakUp)) peakUp = b.up;
      if (Math.abs(b.right) > Math.abs(peakRight)) peakRight = b.right;
      if (firstSignUp === 0 && Math.abs(b.up) >= FLICK_THRESHOLD) firstSignUp = b.up > 0 ? 1 : -1;
      if (firstSignRight === 0 && Math.abs(b.right) >= FLICK_THRESHOLD) firstSignRight = b.right > 0 ? 1 : -1;
      // Shake heuristic runs on whichever axis has flips.
      const mix = b.up + b.right;
      if (Math.abs(mix) > SHAKE_THRESHOLD) {
        const sign = mix > 0 ? 1 : -1;
        if (prevSignUp !== 0 && sign !== prevSignUp) flipsUp++;
        prevSignUp = sign;
      }
    }

    if (flipsUp >= SHAKE_FLIPS) {
      this.emit('shake', Math.max(Math.abs(peakUp), Math.abs(peakRight)));
      return;
    }

    const absUp = Math.abs(peakUp);
    const absRight = Math.abs(peakRight);
    if (absUp >= FLICK_THRESHOLD && absUp >= absRight) {
      const sign = firstSignUp !== 0 ? firstSignUp : (peakUp > 0 ? 1 : -1);
      this.emit(sign > 0 ? 'flickUp' : 'flickDown', absUp);
      return;
    }
    if (absRight >= FLICK_THRESHOLD) {
      const sign = firstSignRight !== 0 ? firstSignRight : (peakRight > 0 ? 1 : -1);
      this.emit(sign > 0 ? 'flickRight' : 'flickLeft', absRight);
      return;
    }
  }

  private emit(kind: string, magnitude: number): void {
    const at = Date.now();
    this.lastEmitAt = at;
    this.buffer = [];
    const e = new GestureEvent();
    e.kind = kind; e.magnitude = magnitude; e.at = at;
    hilog.info(0x1, TAG, 'gesture %{public}s mag=%{public}f', kind, magnitude);
    for (const l of this.listeners) {
      try { l(e); } catch (err) { /* swallow */ }
    }
  }
}
