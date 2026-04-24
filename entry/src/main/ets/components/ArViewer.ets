import { webview } from '@kit.ArkWeb';
import { hilog } from '@kit.PerformanceAnalysisKit';
import common from '@ohos.app.ability.common';
import util from '@ohos.util';
import { BuildPart } from '../services/NebulaSync';

const RAWFILE_MODELS = 'three/models/';

export class ArReadyEvent { ready: boolean = false; }
export class ArSelectEvent { partId: string | null = null; }
export class ArBuildEvent { parts: BuildPart[] = []; }
export class ArAnchorEvent {
  x: number = 0; y: number = 0; z: number = 0;
}
export class ArModelEvent {
  ok: boolean = false;
  url: string = '';
  reason: string = '';
}
export class ArSplatEvent {
  status: string = '';   // 'ready' | 'failed' (loading is implicit on toggle)
  reason: string = '';
}

class BridgePayload {
  partId: string = '';
  ready: boolean = false;
  parts: BuildPart[] = [];
  x: number = 0;
  y: number = 0;
  z: number = 0;
  url: string = '';
  reason: string = '';
  status: string = '';
}
class BridgeMessage {
  event: string = '';
  payload: BridgePayload = new BridgePayload();
}
class PendingBundled {
  filename: string = '';
  targetHeight: number = 1.8;
}

class BridgeObject {
  onMsg: (msg: string) => void = (): void => {};
  context: common.UIAbilityContext | null = null;

  postMessage(msg: string): void { this.onMsg(msg); }

  /**
   * Called synchronously from JS (GLTFLoader.parse path). Returns the GLB
   * as a base64 string, or '' on any failure. All work happens inside the
   * ArkTS side — no URL interception, no cross-origin fetching.
   */
  getModelBytes(filename: string): string {
    if (this.context === null) return '';
    try {
      const bytes: Uint8Array = this.context.resourceManager.getRawFileContentSync(RAWFILE_MODELS + filename);
      const helper = new util.Base64Helper();
      return helper.encodeToStringSync(bytes);
    } catch (e) {
      return '';
    }
  }

  /**
   * Generic rawfile reader for JS side. Used by the ar.html fetch
   * polyfill so the Gaussian-splat library can load scenes from
   * `resource://rawfile/...` — the ArkWeb WebView's fetch() doesn't
   * handle that scheme on its own, so the library's internal
   * `fetch(url).then(r => r.arrayBuffer())` silently errors.
   * Returns base64; empty string on failure (caller treats as 404).
   */
  getRawfileBytes(rawfilePath: string): string {
    if (this.context === null) return '';
    try {
      const bytes: Uint8Array = this.context.resourceManager.getRawFileContentSync(rawfilePath);
      const helper = new util.Base64Helper();
      return helper.encodeToStringSync(bytes);
    } catch (e) {
      return '';
    }
  }
}

export class ArBridge {
  controller: webview.WebviewController = new webview.WebviewController();
  ready: boolean = false;

  // Pending state queued up before the page is ready. Flushed on flush().
  private pendingParts: Object[] | null = null;
  private pendingExplode: number | null = null;
  private pendingSelect: string | null = null;
  private pendingSelectCleared: boolean = false;
  private pendingBundled: PendingBundled | null = null;
  private pendingClear: boolean = false;

  private run(js: string): void {
    if (!this.ready) return;
    try { this.controller.runJavaScript(js); } catch (e) { /* ignore */ }
  }

  setParts(parts: Object[]): void {
    this.pendingParts = parts;
    this.pendingBundled = null;
    this.pendingClear = false;
    if (this.ready) this.flushParts();
  }
  setExplode(ratio: number): void {
    this.pendingExplode = ratio;
    if (this.ready) this.flushExplode();
  }
  selectPart(id: string | null): void {
    if (id === null) { this.pendingSelect = null; this.pendingSelectCleared = true; }
    else { this.pendingSelect = id; this.pendingSelectCleared = false; }
    if (this.ready) this.flushSelect();
  }
  setOrientation(yaw: number, pitch: number, roll: number): void {
    this.run(`window.Nebula && window.Nebula.setOrientation(${yaw},${pitch},${roll})`);
  }
  resetAnchor(): void {
    this.run(`window.Nebula && window.Nebula.resetAnchor()`);
  }
  setAnchorDistance(d: number): void {
    this.run(`window.Nebula && window.Nebula.setAnchorDistance(${d})`);
  }
  setControlMode(mode: string): void {
    this.run(`window.Nebula && window.Nebula.setControlMode('${mode}')`);
  }
  orbitBy(dYawDeg: number, dPitchDeg: number): void {
    this.run(`window.Nebula && window.Nebula.orbitBy(${dYawDeg},${dPitchDeg})`);
  }
  recenterCamera(): void {
    this.run(`window.Nebula && window.Nebula.recenterCamera()`);
  }
  setOrbitRadius(r: number): void {
    this.run(`window.Nebula && window.Nebula.setOrbitRadius(${r})`);
  }
  zoomBy(factor: number): void {
    this.run(`window.Nebula && window.Nebula.zoomBy(${factor})`);
  }
  panTarget(dx: number, dy: number, dz: number): void {
    this.run(`window.Nebula && window.Nebula.panTarget(${dx},${dy},${dz})`);
  }
  // Walk forward/right relative to the camera's current XZ heading.
  // Used by the left navigation joystick — values typically in [-1, 1]
  // representing joystick deflection. JS computes the world-space
  // delta from the camera's facing direction so the user always moves
  // "in the direction they're looking" rather than along world axes.
  walkMove(forward: number, right: number): void {
    this.run(`window.Nebula && window.Nebula.walkMove(${forward},${right})`);
  }
  // Vertical translation (Y axis) — used by the up/down arrow column.
  walkClimb(up: number): void {
    this.run(`window.Nebula && window.Nebula.walkClimb(${up})`);
  }
  // Look around in place — rotates the camera's view direction without
  // translating its position. Used by the right navigation joystick.
  // Inputs in [-1, 1]; JS scales to per-frame rotation rate.
  lookBy(yawNorm: number, pitchNorm: number): void {
    this.run(`window.Nebula && window.Nebula.lookBy(${yawNorm},${pitchNorm})`);
  }
  // Toggle the procedural maker-workshop backdrop inside the WebView.
  // When on, the canvas clears opaque and draws the lab scene — page
  // should simultaneously hide the camera passthrough layer. When off,
  // the canvas goes transparent again for AR passthrough.
  setEnvironment(on: boolean): void {
    this.run(`window.Nebula && window.Nebula.setEnvironment(${on ? 'true' : 'false'})`);
  }
  setModel(url: string, targetHeight: number): void {
    if (!this.ready) return;
    try {
      this.controller.runJavaScript(
        `window.Nebula && window.Nebula.setModel('${url}', ${targetHeight})`
      );
    } catch (e) {}
  }
  loadBundledModel(filename: string, targetHeight: number): void {
    const p = new PendingBundled();
    p.filename = filename;
    p.targetHeight = targetHeight;
    this.pendingBundled = p;
    this.pendingParts = null;
    this.pendingClear = false;
    if (this.ready) this.flushBundled();
  }
  /**
   * Inject a GLB as base64, in chunks — HarmonyOS's runJavaScript has a
   * max payload per call (empirically a few hundred KB), so we split large
   * models (Hubble ≈ 6.7 MB base64) into 384 KB pieces. The JS side
   * accumulates them and parses once the last chunk arrives.
   */
  loadModelBase64(base64: string, targetHeight: number, label: string): void {
    if (!this.ready) return;
    this.pushChunks(base64, targetHeight, label);
  }

  private async pushChunks(base64: string, targetHeight: number, label: string): Promise<void> {
    const CHUNK = 384 * 1024;
    const safeLabel = label.replace(/'/g, "\\'");
    const totalChunks = Math.max(1, Math.ceil(base64.length / CHUNK));
    try {
      await this.controller.runJavaScript(
        `window.Nebula && window.Nebula.startModelTransfer('${safeLabel}', ${totalChunks}, ${targetHeight})`
      );
      for (let i = 0; i < totalChunks; i++) {
        const start = i * CHUNK;
        const end = Math.min(start + CHUNK, base64.length);
        const chunk = base64.substring(start, end);
        await this.controller.runJavaScript(
          `window.Nebula && window.Nebula.pushModelChunk(${i}, '${chunk}')`
        );
      }
      await this.controller.runJavaScript(
        `window.Nebula && window.Nebula.finishModelTransfer()`
      );
    } catch (e) { /* ignore — JS side will timeout & fire model-error */ }
  }
  clearModel(): void {
    this.pendingBundled = null;
    this.pendingParts = null;
    this.pendingClear = true;
    if (this.ready) this.flushClear();
  }

  private flushParts(): void {
    if (this.pendingParts === null) return;
    this.run(`window.Nebula && window.Nebula.setParts(${JSON.stringify(this.pendingParts)})`);
    this.pendingParts = null;
  }
  private flushExplode(): void {
    if (this.pendingExplode === null) return;
    this.run(`window.Nebula && window.Nebula.setExplode(${this.pendingExplode})`);
    this.pendingExplode = null;
  }
  private flushSelect(): void {
    if (this.pendingSelect !== null) {
      this.run(`window.Nebula && window.Nebula.selectPart('${this.pendingSelect}')`);
      this.pendingSelect = null;
      this.pendingSelectCleared = false;
      return;
    }
    if (this.pendingSelectCleared) {
      this.run(`window.Nebula && window.Nebula.selectPart(null)`);
      this.pendingSelectCleared = false;
    }
  }

  private flushBundled(): void {
    if (this.pendingBundled === null) return;
    const p = this.pendingBundled;
    const safe = p.filename.replace(/'/g, "\\'");
    this.run(`window.Nebula && window.Nebula.loadBundledModel('${safe}', ${p.targetHeight})`);
    this.pendingBundled = null;
  }
  private flushClear(): void {
    if (!this.pendingClear) return;
    this.run(`window.Nebula && window.Nebula.clearModel()`);
    this.pendingClear = false;
  }

  flushAll(): void {
    this.flushClear();
    this.flushBundled();
    this.flushParts();
    this.flushExplode();
    this.flushSelect();
  }
}

@Component
export struct ArViewer {
  bridge: ArBridge = new ArBridge();
  onReady: (e: ArReadyEvent) => void = (): void => {};
  onSelect: (e: ArSelectEvent) => void = (): void => {};
  onBuildChanged: (e: ArBuildEvent) => void = (): void => {};
  onAnchor: (e: ArAnchorEvent) => void = (): void => {};
  onModel: (e: ArModelEvent) => void = (): void => {};
  onSplat: (e: ArSplatEvent) => void = (): void => {};

  private bridgeObj: BridgeObject = new BridgeObject();

  aboutToAppear(): void {
    this.bridgeObj.onMsg = (msg: string): void => this.handle(msg);
    try {
      const host = this.getUIContext().getHostContext();
      this.bridgeObj.context = host !== undefined && host !== null
        ? (host as common.UIAbilityContext) : null;
    } catch (e) { /* context may not be available yet; getModelBytes returns '' */ }
  }

  private handle(msg: string): void {
    try {
      const parsed: BridgeMessage = JSON.parse(msg) as BridgeMessage;
      if (parsed.event === 'ready') {
        this.markReady();
      } else if (parsed.event === 'select') {
        const e = new ArSelectEvent();
        if (parsed.payload !== null && parsed.payload !== undefined) {
          const pid = parsed.payload.partId;
          e.partId = (pid !== undefined && pid !== null && pid.length > 0) ? pid : null;
        } else {
          e.partId = null;
        }
        this.onSelect(e);
      } else if (parsed.event === 'anchor') {
        const e = new ArAnchorEvent();
        if (parsed.payload !== null && parsed.payload !== undefined) {
          e.x = parsed.payload.x ?? 0;
          e.y = parsed.payload.y ?? 0;
          e.z = parsed.payload.z ?? 0;
        }
        this.onAnchor(e);
      } else if (parsed.event === 'build') {
        const e = new ArBuildEvent();
        e.parts = parsed.payload !== null && parsed.payload !== undefined
          ? (parsed.payload.parts ?? []) : [];
        this.onBuildChanged(e);
      } else if (parsed.event === 'model-loaded') {
        const e = new ArModelEvent();
        e.ok = true;
        if (parsed.payload !== null && parsed.payload !== undefined) {
          e.url = parsed.payload.url ?? '';
        }
        this.onModel(e);
      } else if (parsed.event === 'model-error') {
        const e = new ArModelEvent();
        e.ok = false;
        if (parsed.payload !== null && parsed.payload !== undefined) {
          e.url = parsed.payload.url ?? '';
          e.reason = parsed.payload.reason ?? '';
        }
        this.onModel(e);
      } else if (parsed.event === 'splat') {
        const e = new ArSplatEvent();
        if (parsed.payload !== null && parsed.payload !== undefined) {
          e.status = parsed.payload.status ?? '';
          e.reason = parsed.payload.reason ?? '';
        }
        this.onSplat(e);
      }
    } catch (err) {
      hilog.warn(0x1, 'ArViewer', 'bridge parse failed');
    }
  }

  private markReady(): void {
    if (this.bridge.ready) return;
    this.bridge.ready = true;
    // Flush any state queued while the page was still loading.
    this.bridge.flushAll();
    const e = new ArReadyEvent();
    e.ready = true;
    this.onReady(e);
  }

  build() {
    Web({
      src: $rawfile('three/ar.html'),
      controller: this.bridge.controller
    })
      .domStorageAccess(true)
      .fileAccess(true)
      .javaScriptAccess(true)
      .javaScriptProxy({
        object: this.bridgeObj,
        name: 'nebula',
        methodList: ['postMessage', 'getModelBytes', 'getRawfileBytes'],
        controller: this.bridge.controller
      })
      .onPageEnd((): void => {
        // Authoritative readiness signal — fires once the HTML + inline
        // script have executed, regardless of the postMessage round-trip.
        // Safe again now that ar.html is back to a classic end-of-body
        // <script> tag (synchronous during parse) — the module /
        // importmap experiment made this unsafe because module scripts
        // are deferred, but we're no longer doing that.
        this.markReady();
      })
      // Pipe WebView console + page errors to hilog. Without these
      // hooks every console.log/warn/error from ar.html goes into a
      // black hole and the Gaussian-splat loading failures are
      // impossible to diagnose from the device. Tag filter:
      //   hdc hilog | grep ArViewerWeb
      .onConsole((e): boolean => {
        try {
          hilog.info(0x1, 'ArViewerWeb',
            'console: %{public}s', e.message.getMessage());
        } catch (_) { /* ignore */ }
        return false;
      })
      .onErrorReceive((e): void => {
        try {
          hilog.warn(0x1, 'ArViewerWeb',
            'error: %{public}s', e.error.getErrorInfo());
        } catch (_) { /* ignore */ }
      })
      .backgroundColor(Color.Transparent)
      .width('100%')
      .height('100%')
  }
}
