/**
 * CameraPreview — native back-camera passthrough.
 *
 * The XComponent is only added to the tree while we're actively trying to
 * show a live feed. If permission is denied, no camera exists, or starting
 * the session fails at any step, we unmount the XComponent completely so the
 * page's light gradient stays visible (no black hardware-surface cutout).
 *
 * On tab re-entry this runs fresh every time — a new permission check, a new
 * session — and the gradient holds the screen whenever the camera can't.
 */

import camera from '@ohos.multimedia.camera';
import abilityAccessCtrl, { Permissions, PermissionRequestResult } from '@ohos.abilityAccessCtrl';
import common from '@ohos.app.ability.common';
import { BusinessError } from '@ohos.base';
import { hilog } from '@kit.PerformanceAnalysisKit';

const TAG = 'CameraPreview';

class CameraHandles {
  manager: camera.CameraManager | null = null;
  input: camera.CameraInput | null = null;
  output: camera.PreviewOutput | null = null;
  session: camera.Session | null = null;
  surfaceId: string = '';
  started: boolean = false;
}

@Component
export struct CameraPreview {
  onReady: (ok: boolean) => void = (): void => {};

  // `surfaceMounted` controls whether the XComponent is in the tree at all.
  // `cameraRunning` controls the opacity once it IS mounted.
  @State private surfaceMounted: boolean = false;
  @State private cameraRunning: boolean = false;

  private context: common.UIAbilityContext | null = null;
  private xcController: XComponentController = new XComponentController();
  private handles: CameraHandles = new CameraHandles();
  private permissionGranted: boolean = false;
  // Camera-open retry state. On a cold mount immediately after another
  // component released the hardware (e.g. MediaPipe WebView), the open
  // call can transiently fail. We try up to 4 times with exponential
  // backoff before declaring the feed unavailable.
  private openAttempts: number = 0;
  private maxOpenAttempts: number = 4;
  private retryTimer: number = -1;

  aboutToAppear(): void {
    try {
      const host = this.getUIContext().getHostContext();
      this.context = host !== undefined && host !== null
        ? (host as common.UIAbilityContext) : null;
    } catch (e) {
      this.context = null;
    }
    this.requestPermission();
  }

  aboutToDisappear(): void {
    if (this.retryTimer >= 0) { clearTimeout(this.retryTimer); this.retryTimer = -1; }
    this.stop();
  }

  private requestPermission(): void {
    if (this.context === null) return;
    const atm: abilityAccessCtrl.AtManager = abilityAccessCtrl.createAtManager();
    const ctx: common.UIAbilityContext = this.context;
    const perms: Array<Permissions> = ['ohos.permission.CAMERA'];
    atm.requestPermissionsFromUser(ctx, perms,
      (err: BusinessError, result: PermissionRequestResult): void => {
        if (err !== null && err !== undefined && err.code !== 0) {
          this.bail();
          return;
        }
        if (result.authResults.length === 0 || result.authResults[0] !== 0) {
          this.bail();
          return;
        }
        this.permissionGranted = true;
        // Mount the XComponent now; onLoad will then fire and call start().
        this.surfaceMounted = true;
      }
    );
  }

  private bail(): void {
    this.cameraRunning = false;
    if (this.openAttempts < this.maxOpenAttempts && this.permissionGranted) {
      // Fast exponential retry — 60 / 120 / 240 / 480 ms. Covers the
      // transient window where a WebView just released the camera but
      // the kernel hasn't finished the handoff yet.
      const delay = 60 * Math.pow(2, this.openAttempts);
      this.openAttempts++;
      hilog.info(0x1, TAG, 'open failed, retry #%{public}d in %{public}dms',
        this.openAttempts, delay);
      this.surfaceMounted = false;
      if (this.retryTimer >= 0) clearTimeout(this.retryTimer);
      this.retryTimer = setTimeout((): void => {
        this.retryTimer = -1;
        this.surfaceMounted = true;
      }, delay);
      return;
    }
    hilog.warn(0x1, TAG, 'giving up after %{public}d attempts', this.openAttempts);
    this.permissionGranted = false;
    this.surfaceMounted = false;
    this.openAttempts = 0;
    this.onReady(false);
  }

  private start(): void {
    if (!this.permissionGranted) { this.bail(); return; }
    if (this.handles.started) return;
    if (this.handles.surfaceId.length === 0) return;
    const ctx = this.context as common.UIAbilityContext;
    try {
      const mgr = camera.getCameraManager(ctx);
      this.handles.manager = mgr;
      const devices = mgr.getSupportedCameras();
      if (devices.length === 0) { this.bail(); return; }
      const back = devices.find(d => d.cameraPosition === camera.CameraPosition.CAMERA_POSITION_BACK) ?? devices[0];
      if (back === undefined) { this.bail(); return; }
      this.handles.input = mgr.createCameraInput(back);
      this.handles.input.open((err: BusinessError): void => {
        if (err !== null && err !== undefined && err.code !== 0) { this.bail(); return; }
        this.openSession(back);
      });
    } catch (err) {
      hilog.warn(0x1, TAG, 'init failed: %{public}s', JSON.stringify(err));
      this.bail();
    }
  }

  private openSession(device: camera.CameraDevice): void {
    const mgr = this.handles.manager as camera.CameraManager;
    const input = this.handles.input as camera.CameraInput;
    try {
      const cap = mgr.getSupportedOutputCapability(device, camera.SceneMode.NORMAL_PHOTO);
      if (cap.previewProfiles.length === 0) { this.bail(); return; }
      const preview = cap.previewProfiles[0];
      this.handles.output = mgr.createPreviewOutput(preview, this.handles.surfaceId);
      this.handles.session = mgr.createSession(camera.SceneMode.NORMAL_PHOTO) as camera.Session;
      const s = this.handles.session as camera.Session;
      s.beginConfig();
      s.addInput(input);
      s.addOutput(this.handles.output as camera.PreviewOutput);
      s.commitConfig((commitErr: BusinessError): void => {
        if (commitErr !== null && commitErr !== undefined && commitErr.code !== 0) {
          this.bail(); return;
        }
        s.start((startErr: BusinessError): void => {
          if (startErr !== null && startErr !== undefined && startErr.code !== 0) {
            this.bail(); return;
          }
          this.handles.started = true;
          this.cameraRunning = true;
          this.openAttempts = 0; // reset retry counter on success
          this.onReady(true);
        });
      });
    } catch (err) {
      hilog.warn(0x1, TAG, 'session failed: %{public}s', JSON.stringify(err));
      this.bail();
    }
  }

  private stop(): void {
    const h = this.handles;
    this.cameraRunning = false;
    try {
      if (h.session !== null) {
        const s = h.session;
        s.stop((_e: BusinessError): void => {
          try { s.release((): void => {}); } catch (err) { /* ignore */ }
        });
      }
    } catch (err) { /* ignore */ }
    try {
      if (h.input !== null) h.input.close((): void => {});
    } catch (err) { /* ignore */ }
    h.session = null;
    h.input = null;
    h.output = null;
    h.started = false;
    h.surfaceId = '';
  }

  build() {
    Stack() {
      if (this.surfaceMounted) {
        XComponent({
          id: 'ar-camera',
          type: 'surface',
          controller: this.xcController
        })
          .onLoad((): void => {
            this.handles.surfaceId = this.xcController.getXComponentSurfaceId();
            this.start();
          })
          .opacity(this.cameraRunning ? 1 : 0)
          .width('100%')
          .height('100%')
      }
    }
    .width('100%')
    .height('100%')
  }
}
