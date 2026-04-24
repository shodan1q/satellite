/**
 * HandGesturePipeline — camera-based hand-gesture control for AR explode
 * amount.
 *
 * This service owns a **parallel, low-resolution** camera session dedicated
 * to hand tracking. It doesn't share frames with CameraPreview (which keeps
 * running normally for the AR passthrough) so the two can start / stop
 * independently.
 *
 * Architecture:
 *
 *     camera.PhotoOutput (burst) ──► ImageReceiver ──► pixelMap
 *                                                         │
 *                                                         ▼
 *                                                 detectHands(pixelMap)
 *                                                         │
 *                                                         ▼
 *                                               computeExplode(hands)
 *                                                         │
 *                                                         ▼
 *                                                 HandGestureEvent
 *                                                         │
 *                                                         ▼
 *                                                    listeners
 *
 * Pluggable detector: `HandGesturePipeline.setDetector(fn)` replaces the
 * default stub detector. Recommended backends:
 *
 *   - @kit.CoreVisionKit `skeleton` body keypoints (wrist-to-wrist distance)
 *   - @kit.MindSporeLiteKit + a MediaPipe Hands TFLite → .ms model
 *   - WebView MediaPipe Hands JS with a runJavaScript bridge
 *
 * Until a real backend is wired the pipeline runs in ENUM_EMULATE mode —
 * slowly oscillates the explode value 0 → 1 → 0 every 6s so the UX loop
 * is testable end-to-end (button press → chip → 3D scene responds).
 */

import { hilog } from '@kit.PerformanceAnalysisKit';
import image from '@ohos.multimedia.image';

const TAG = 'HandGesturePipeline';

/**
 * 2D point in image space (pixels), with detector confidence 0..1.
 */
export class HandPoint {
  x: number = 0;
  y: number = 0;
  confidence: number = 0;
}

/**
 * Snapshot of what the detector saw on one frame.
 */
export class HandFrame {
  /** 0 / 1 / 2 hands visible. */
  hands: number = 0;
  /** 2-hand distance normalised to [0, 1] of the image diagonal. */
  distance: number = 0;
  /** True once the detector has settled (reduces jitter at onset). */
  stable: boolean = false;
  /** Debug: raw palm centers, if the detector produced them. */
  palms: HandPoint[] = [];
}

export class HandGestureEvent {
  /** Continuous 0..1 — map this directly to explode. */
  explode: number = 0;
  /** Discrete hint: 'closed' / 'mid' / 'open'. */
  stage: string = 'mid';
  /** For UI readout. */
  handsVisible: number = 0;
  at: number = 0;
}

/**
 * Emitted whenever a closed-fist drag is active. Consumers typically
 * apply dx/dy to the model position via ArBridge.panTarget(dx, -dy, 0).
 * Coordinates are normalised [0,1] camera space, with x already
 * mirrored to match the on-screen preview.
 */
export class HandGrabEvent {
  active: boolean = false;
  /** Per-frame delta, roughly [-0.05, +0.05] during a natural drag. */
  dx: number = 0;
  dy: number = 0;
  /** Absolute normalised palm centre. */
  x: number = 0;
  y: number = 0;
  at: number = 0;
}

export type HandGestureListener = (e: HandGestureEvent) => void;
export type HandGrabListener = (e: HandGrabEvent) => void;

/**
 * Pluggable detector signature. A real backend (CoreVisionKit / MindSpore /
 * WebView) should expose this shape so we can swap in without touching the
 * rest of the pipeline.
 */
export type HandDetector = (pixelMap: image.PixelMap) => Promise<HandFrame>;

const EMA_ALPHA = 0.35;      // smooth explode to kill jitter
const EMIT_MIN_DELTA = 0.008;// min change before we fire a new event
const SETTLE_FRAMES = 4;     // frames required before we start emitting
const DEFAULT_RATE_HZ = 12;  // how often we pump frames through the detector

export class HandGesturePipeline {
  private static _inst: HandGesturePipeline | null = null;
  static get instance(): HandGesturePipeline {
    if (HandGesturePipeline._inst === null) HandGesturePipeline._inst = new HandGesturePipeline();
    return HandGesturePipeline._inst as HandGesturePipeline;
  }

  private listeners: HandGestureListener[] = [];
  private grabListeners: HandGrabListener[] = [];
  private grabActive: boolean = false;
  private grabLastX: number = 0;
  private grabLastY: number = 0;
  private running: boolean = false;
  private tickHandle: number = 0;
  private settleCount: number = 0;
  private smoothed: number = 0;
  private lastExplode: number = -1;
  private lastHands: number = 0;
  private emulate: boolean = true;
  /** External push source (WebView MediaPipe). When set, disables the
   *  internal tick loop — frames flow in via pushFrame() instead. */
  private externalSource: boolean = false;

  /** Replace the detector. Call before `subscribe()` (or restart). */
  private detector: HandDetector | null = null;
  setDetector(d: HandDetector | null): void {
    this.detector = d;
    this.emulate = d === null && !this.externalSource;
    hilog.info(0x1, TAG, 'detector %{public}s',
      this.emulate ? 'CLEARED → emulate' : 'installed');
  }

  /**
   * Declare that an external source (e.g. a WebView running MediaPipe
   * Hands) will drive detection via pushFrame(). Disables the internal
   * tick loop and emulate oscillation.
   */
  setExternalSource(on: boolean): void {
    this.externalSource = on;
    this.emulate = !on && this.detector === null;
    hilog.info(0x1, TAG, 'external source %{public}s',
      on ? 'attached' : 'detached');
  }

  /**
   * Accept a single detection frame from an external source (WebView
   * MediaPipe bridge etc.). Runs the same smoothing / settle / emit
   * logic as the internal tick loop.
   */
  private pushCount: number = 0;
  pushFrame(f: HandFrame): void {
    this.pushCount++;
    if (this.pushCount % 30 === 1) {
      hilog.info(0x1, TAG, 'pushFrame #%{public}d running=%{public}s hands=%{public}d d=%{public}s',
        this.pushCount, String(this.running), f.hands, f.distance.toFixed(3));
    }
    if (!this.running) return;
    // Sentinel: distance == -1 means "fist detected — don't update explode".
    // This lets the JS side suppress explode control during a grab.
    if (f.distance < 0) return;
    this.ingest(f);
  }

  /** True when a non-stub detector is attached. */
  isReal(): boolean { return !this.emulate; }

  subscribe(cb: HandGestureListener): () => void {
    this.listeners.push(cb);
    if (!this.running) this.start();
    return (): void => {
      const i = this.listeners.indexOf(cb);
      if (i >= 0) this.listeners.splice(i, 1);
      this.maybeStop();
    };
  }

  /**
   * Subscribe to fist-drag events. Use this alongside subscribe() when
   * you need both explode and position control.
   */
  subscribeGrab(cb: HandGrabListener): () => void {
    this.grabListeners.push(cb);
    if (!this.running) this.start();
    return (): void => {
      const i = this.grabListeners.indexOf(cb);
      if (i >= 0) this.grabListeners.splice(i, 1);
      this.maybeStop();
    };
  }

  private maybeStop(): void {
    if (this.listeners.length === 0 && this.grabListeners.length === 0) this.stop();
  }

  /**
   * Called from the WebView bridge when the hand state changes. Tracks
   * previous position, computes a per-frame delta, and fans out to
   * grab listeners. An explode sentinel of -1 means "don't touch the
   * explode value" (used when a fist suppresses explode control).
   */
  pushGrab(active: boolean, x: number, y: number): void {
    if (!this.running) return;
    // Transition to active — capture the anchor but don't emit a delta
    // (would jump by full position on first frame).
    if (active && !this.grabActive) {
      this.grabActive = true;
      this.grabLastX = x;
      this.grabLastY = y;
      this.emitGrab(true, 0, 0, x, y);
      return;
    }
    if (!active && this.grabActive) {
      this.grabActive = false;
      this.emitGrab(false, 0, 0, x, y);
      return;
    }
    if (active) {
      const dx = x - this.grabLastX;
      const dy = y - this.grabLastY;
      this.grabLastX = x;
      this.grabLastY = y;
      // Don't spam listeners if the hand didn't actually move.
      if (Math.abs(dx) + Math.abs(dy) < 0.0005) return;
      this.emitGrab(true, dx, dy, x, y);
    }
  }

  private emitGrab(active: boolean, dx: number, dy: number, x: number, y: number): void {
    const e = new HandGrabEvent();
    e.active = active;
    e.dx = dx;
    e.dy = dy;
    e.x = x;
    e.y = y;
    e.at = Date.now();
    for (const l of this.grabListeners) {
      try { l(e); } catch (err) { /* swallow */ }
    }
  }

  private start(): void {
    if (this.running) return;
    this.running = true;
    this.settleCount = 0;
    this.smoothed = 0;
    this.lastExplode = -1;
    this.lastHands = 0;

    // External source pushes frames itself — no internal tick.
    if (!this.externalSource) {
      const period = Math.round(1000 / DEFAULT_RATE_HZ);
      this.tickHandle = setInterval((): void => { this.tick(); }, period);
    }
    hilog.info(0x1, TAG, 'started emulate=%{public}s external=%{public}s',
      String(this.emulate), String(this.externalSource));
  }

  private stop(): void {
    if (!this.running) return;
    if (this.tickHandle !== 0) { clearInterval(this.tickHandle); this.tickHandle = 0; }
    this.running = false;
    this.settleCount = 0;
    this.smoothed = 0;
    hilog.info(0x1, TAG, 'stopped');
  }

  /**
   * Single detection tick — pulls a camera frame (or fabricates one in
   * emulate mode), runs the detector, and emits a HandGestureEvent.
   */
  private async tick(): Promise<void> {
    if (!this.running) return;

    let frame: HandFrame;
    if (this.emulate || this.detector === null) {
      frame = this.emulateFrame();
    } else {
      const pm = await this.grabCameraFrame();
      if (pm === null) return;
      try {
        frame = await this.detector(pm);
      } catch (e) {
        hilog.warn(0x1, TAG, 'detector threw, dropping frame');
        return;
      } finally {
        // Recycle the pixelMap whenever we own it.
        try { await pm.release(); } catch (err) { /* ignore */ }
      }
    }

    this.ingest(frame);
  }

  /**
   * Shared smoothing / settle / emit logic. Called both from the
   * internal tick() loop (emulate / polled detector) and from
   * pushFrame() (external source like WebView MediaPipe).
   *
   * Rule: if any hand is visible, always emit. No `stable` gating —
   * MediaPipe already filters low-confidence detections internally,
   * so the extra settle here was just swallowing good frames when
   * the boolean marshaled weirdly across the JS bridge.
   */
  private ingestCount: number = 0;
  private ingest(frame: HandFrame): void {
    this.ingestCount++;
    const target = this.clamp01(frame.distance);
    this.smoothed = this.smoothed * (1 - EMA_ALPHA) + target * EMA_ALPHA;
    const v = this.clamp01(this.smoothed);

    const deltaOk = this.lastExplode < 0
                 || Math.abs(v - this.lastExplode) >= EMIT_MIN_DELTA
                 || frame.hands !== this.lastHands;
    // Zero hands: keep smoothing decaying but don't spam listeners.
    const shouldEmit = frame.hands > 0 ? deltaOk : frame.hands !== this.lastHands;

    if (this.ingestCount % 30 === 1) {
      hilog.info(0x1, TAG,
        'ingest #%{public}d hands=%{public}d d=%{public}s v=%{public}s last=%{public}s emit=%{public}s',
        this.ingestCount, frame.hands, frame.distance.toFixed(3), v.toFixed(3),
        this.lastExplode.toFixed(3), String(shouldEmit));
    }

    if (shouldEmit) {
      this.emit(v, frame.hands);
      this.lastExplode = v;
      this.lastHands = frame.hands;
    }
  }

  /**
   * Pull a single frame from the camera. Return null if nothing is ready.
   *
   * NOTE: This is the integration seam for real camera capture. Today the
   * pipeline runs in EMULATE mode so this returns null and the caller
   * skips the detector entirely. To wire up a real camera session:
   *
   *   1. On `start()`, open a `camera.PhotoOutput` dedicated to this
   *      pipeline with a 640x480 preview profile.
   *   2. Call `output.capture(...)` to get `image.PixelMap` frames.
   *   3. Return the pixelMap from this method (caller will release it).
   *
   * Keep the resolution small (<= 640x480) — hand keypoint detectors
   * almost always downsample anyway and we save battery.
   */
  private async grabCameraFrame(): Promise<image.PixelMap | null> {
    return null;
  }

  /**
   * Emulated detection — oscillates explode 0..1..0 every 6 seconds so
   * the UI responds visibly even before a real detector is plugged in.
   */
  private emulateT0: number = 0;
  private emulateFrame(): HandFrame {
    if (this.emulateT0 === 0) this.emulateT0 = Date.now();
    const dt = (Date.now() - this.emulateT0) * 0.001;
    const cycle = 6.0;
    // Triangle wave: 0 → 1 → 0 over cycle seconds.
    const phase = (dt % cycle) / cycle;
    const tri = phase < 0.5 ? phase * 2 : 2 - phase * 2;
    const f = new HandFrame();
    f.hands = 2;
    f.distance = tri;
    f.stable = true;
    return f;
  }

  private emitCount: number = 0;
  private emit(explode: number, hands: number): void {
    const e = new HandGestureEvent();
    e.explode = explode;
    e.handsVisible = hands;
    e.at = Date.now();
    if (explode < 0.15) e.stage = 'closed';
    else if (explode > 0.85) e.stage = 'open';
    else e.stage = 'mid';
    this.emitCount++;
    if (this.emitCount % 15 === 1) {
      hilog.info(0x1, TAG, 'emit #%{public}d explode=%{public}s hands=%{public}d listeners=%{public}d',
        this.emitCount, explode.toFixed(3), hands, this.listeners.length);
    }
    for (const l of this.listeners) {
      try { l(e); } catch (err) { /* swallow */ }
    }
  }

  private clamp01(v: number): number {
    return v < 0 ? 0 : (v > 1 ? 1 : v);
  }
}
