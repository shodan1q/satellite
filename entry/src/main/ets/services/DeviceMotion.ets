/**
 * DeviceMotion — thin wrapper over @ohos.sensor ORIENTATION.
 *
 * Emits yaw/pitch/roll (all in degrees) at ~60 Hz while subscribed. Used by
 * the AR workshop to keep the virtual rocket anchored in world-space while
 * the user rotates the phone around it.
 */

import sensor from '@ohos.sensor';
import { hilog } from '@kit.PerformanceAnalysisKit';

const TAG = 'DeviceMotion';

export class MotionSample {
  yaw: number = 0;
  pitch: number = 0;
  roll: number = 0;
}

export type MotionListener = (sample: MotionSample) => void;

/**
 * Throttles a listener to at most 1 update per `intervalMs`, skipping repeats
 * that are smaller than `epsilon` degrees on every axis. Used to avoid
 * flooding the WebView's runJavaScript queue with every 16 ms sensor sample.
 */
export class ThrottledMotionListener {
  private lastPushAt: number = 0;
  private last: MotionSample = new MotionSample();
  readonly intervalMs: number;
  readonly epsilon: number;
  readonly inner: MotionListener;

  constructor(inner: MotionListener, intervalMs: number, epsilon: number) {
    this.inner = inner;
    this.intervalMs = intervalMs;
    this.epsilon = epsilon;
  }

  handle(sample: MotionSample): void {
    const now = Date.now();
    const due = now - this.lastPushAt >= this.intervalMs;
    const moved =
      Math.abs(sample.yaw - this.last.yaw) >= this.epsilon ||
      Math.abs(sample.pitch - this.last.pitch) >= this.epsilon ||
      Math.abs(sample.roll - this.last.roll) >= this.epsilon;
    if (!due && !moved) return;
    this.last = sample;
    this.lastPushAt = now;
    this.inner(sample);
  }
}

export class DeviceMotion {
  private static _instance: DeviceMotion | null = null;
  private listeners: MotionListener[] = [];
  private running: boolean = false;
  private last: MotionSample = new MotionSample();

  static get instance(): DeviceMotion {
    if (DeviceMotion._instance === null) {
      DeviceMotion._instance = new DeviceMotion();
    }
    return DeviceMotion._instance as DeviceMotion;
  }

  subscribe(listener: MotionListener): () => void {
    this.listeners.push(listener);
    if (!this.running) this.startSensor();
    listener(this.last);
    return (): void => {
      const idx = this.listeners.indexOf(listener);
      if (idx >= 0) this.listeners.splice(idx, 1);
      if (this.listeners.length === 0) this.stopSensor();
    };
  }

  private startSensor(): void {
    try {
      sensor.on(sensor.SensorId.ORIENTATION, (data: sensor.OrientationResponse): void => {
        const s = new MotionSample();
        s.yaw = data.alpha;
        s.pitch = data.beta;
        s.roll = data.gamma;
        this.last = s;
        for (const l of this.listeners) {
          try { l(s); } catch (e) { /* swallow */ }
        }
      }, { interval: 16_000_000 });
      this.running = true;
    } catch (err) {
      hilog.warn(0x1, TAG, 'orientation sensor unavailable');
    }
  }

  private stopSensor(): void {
    try { sensor.off(sensor.SensorId.ORIENTATION); } catch (e) {}
    this.running = false;
  }

  peek(): MotionSample {
    return this.last;
  }
}
