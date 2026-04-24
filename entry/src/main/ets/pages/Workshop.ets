/**
 * AR Workshop — full-bleed camera passthrough with transparent 3D overlay.
 *
 * The user rotates the phone around a virtual anchor they tap to place. A
 * pull-up drawer holds the parts palette. All chrome floats as glass over the
 * camera.
 */

import {
  NebulaColor, NebulaFont, NebulaFontFace, NebulaRadius, NebulaShadow, NebulaSpace
} from '../theme/Theme';
import { window } from '@kit.ArkUI';
import { webview } from '@kit.ArkWeb';
import { hilog } from '@kit.PerformanceAnalysisKit';
import common from '@ohos.app.ability.common';
import { NebulaIcon, IconName } from '../components/NebulaIcon';
import { NebulaPill } from '../components/NebulaPill';
import { GlassPanel } from '../components/GlassPanel';
import { CameraPreview } from '../components/CameraPreview';
import {
  ArBridge, ArViewer,
  ArReadyEvent, ArSelectEvent, ArAnchorEvent, ArSplatEvent
} from '../components/ArViewer';
import { Catalog, PartSpec, RocketBlueprint, SatelliteBlueprint, SpecLine } from '../data/Catalog';
import { BuildPart, NebulaSnapshot, NebulaSync } from '../services/NebulaSync';
import { DeviceMotion, MotionSample, ThrottledMotionListener } from '../services/DeviceMotion';
import { AppBus } from '../services/AppBus';
import { UserStats } from '../services/UserStats';
import { ModelStore } from '../services/ModelStore';
import { ViewerOffset, ViewerPart } from '../components/ViewerPart';
import { HandoffPayload } from '../services/Handoff';
import { CardContent, CardSpec, ShareCard, ShareResult } from '../services/ShareCard';
import { GestureEvent, GesturePipeline, GestureTelemetry } from '../services/GesturePipeline';
import { HandFrame, HandGestureEvent, HandGesturePipeline, HandGrabEvent } from '../services/HandGesturePipeline';
import { GestureBurst } from '../components/GestureBurst';
import { JoystickPad } from '../components/JoystickPad';

class PaletteEntry {
  id: string = '';
  kind: string = '';
  label: string = '';
  tone: string = 'brand';
  icon: string = IconName.Rocket;
  factory: () => PartSpec = (): PartSpec => new PartSpec();
}

class Chip {
  id: string = '';
  label: string = '';
}

/**
 * JavaScript proxy object exposed to rawfile/hand.html.
 * Method names must match `methodList` in the Web() javaScriptProxy call.
 */
class HandJsBridge {
  private beatCount: number = 0;
  onHands(count: number, distance: number, stable: boolean): void {
    this.beatCount++;
    if (this.beatCount % 30 === 1) {
      hilog.info(0x1, 'WorkshopHand',
        'bridge tick #%{public}d count=%{public}d d=%{public}s stable=%{public}s',
        this.beatCount, count, distance.toFixed(3), String(stable));
    }
    const f = new HandFrame();
    f.hands = count;
    f.distance = distance;
    f.stable = stable;
    HandGesturePipeline.instance.pushFrame(f);
  }
  onHandGrab(active: boolean, x: number, y: number): void {
    HandGesturePipeline.instance.pushGrab(active, x, y);
  }
}

@Component
export struct WorkshopPage {
  @State private rocketId: string = 'custom-sat';
  @State private parts: PartSpec[] = [];
  @State private selectedId: string = '';
  @State private explode: number = 0;
  @State private anchored: boolean = false;
  @State private drawerOpen: boolean = false;
  @State private sheetOpen: boolean = false;
  @State private cameraOk: boolean = false;
  @State private controller: string = 'phone';
  @State private controlMode: string = 'touch';   // 'touch' | 'motion'
  @State private syncRole: string = 'solo';
  @State private peerCount: number = 0;
  // "体感" — accelerometer-driven flicks/shakes (hand physically moving phone).
  @State private motionOn: boolean = false;
  // "手势" — camera-driven hand-position tracking (pinch/spread in front of lens).
  @State private handOn: boolean = false;
  @State private handStage: string = 'mid';
  @State private handCount: number = 0;
  @State private landscape: boolean = false;
  // "空间" — procedural maker-lab backdrop. When on, the WebView paints
  // a sci-fi workshop (floor grid + light columns + workbenches + truss)
  // behind the model and the camera passthrough layer below is hidden.
  // When off, we fall back to plain AR passthrough through the phone's
  // rear camera. Default on — it's what the Workshop home is designed
  // around and what a fresh launch should showcase.
  @State private envOn: boolean = false;
  // Tracks the ISS splat backdrop loading lifecycle so we can show a
  // "资源较大，加载中" overlay while the 120 MB scene parses. Goes true
  // when the user toggles 空间站 on, back to false when the JS side
  // fires { event: 'splat', status: 'ready' | 'failed' }. Sticky once
  // ready — toggling off/on again is instant from the cache.
  @State private splatLoading: boolean = false;
  @State private splatReady: boolean = false;
  @State private lastGestureLabel: string = '';
  @State private lastGestureAt: number = 0;
  @State private gestureMag: number = 0;
  @State private gestureBurstTrigger: string = '';
  private unsubGesture: () => void = (): void => {};
  private unsubTelemetry: () => void = (): void => {};
  private unsubHand: () => void = (): void => {};
  private unsubHandGrab: () => void = (): void => {};
  private handWebController: webview.WebviewController = new webview.WebviewController();
  // Bridge object exposed to the hand-tracking WebView via javaScriptProxy.
  // `this` capture must be stable between mounts, so we lock it in once.
  private handBridge: HandJsBridge = new HandJsBridge();
  private gestureHintTimer: number = -1;

  private bridge: ArBridge = new ArBridge();
  private rocketOptions: Chip[] = [];
  private unsubMotion: () => void = (): void => {};
  private unsubSync: () => void = (): void => {};
  private unsubBus: () => void = (): void => {};

  aboutToAppear(): void {
    // AR Workshop runs edge-to-edge with rotation enabled — user can hold
    // the phone however they like to frame the model in the room.
    this.enterImmersive();

    const list: Chip[] = [];
    // Put the 自制卫星 starter kit first — it's the Workshop default and
    // should be the leftmost chip so the selection indicator isn't
    // buried in the middle of the scroller.
    for (const s of Catalog.satellites()) {
      const c = new Chip(); c.id = s.id; c.label = s.display; list.push(c);
    }
    for (const r of Catalog.rockets()) {
      const c = new Chip(); c.id = r.id; c.label = r.display; list.push(c);
    }
    this.rocketOptions = list;
    this.loadBlueprint('custom-sat');

    const throttled = new ThrottledMotionListener(
      (s: MotionSample): void => { this.bridge.setOrientation(s.yaw, s.pitch, s.roll); },
      33,   // ~30 Hz — fast enough to feel live, low enough to not queue up
      0.3   // ignore <0.3° jitter
    );
    this.unsubMotion = DeviceMotion.instance.subscribe(
      (s: MotionSample): void => throttled.handle(s)
    );
    this.unsubSync = NebulaSync.instance.subscribe((s: NebulaSnapshot): void => {
      this.controller = s.controller;
      this.syncRole = s.role;
      this.peerCount = s.peerCount;
      // Observer mirrors every sync'd controller field so the screens match.
      // The pushField call-stack is already gated by role, so updating local
      // state here via loadBlueprint / setExplode won't echo back to the peer.
      if (s.role === 'observer') {
        if (s.currentRocket.length > 0 && s.currentRocket !== this.rocketId) {
          this.loadBlueprint(s.currentRocket);
        }
        if (Math.abs(s.explode - this.explode) > 0.001) {
          this.explode = s.explode;
          this.bridge.setExplode(s.explode);
        }
        if (s.selectedPart !== this.selectedId) {
          this.selectedId = s.selectedPart;
          this.bridge.selectPart(s.selectedPart.length > 0 ? s.selectedPart : null);
        }
        if (s.controlMode !== this.controlMode) {
          this.controlMode = s.controlMode;
          this.bridge.setControlMode(s.controlMode);
        }
      }
    });
    this.unsubBus = AppBus.instance.onRocketRequest((id: string): void => {
      this.loadBlueprint(id);
    });
  }

  aboutToDisappear(): void {
    this.unsubMotion();
    this.unsubSync();
    this.unsubBus();
    this.unsubGesture();
    this.unsubTelemetry();
    this.unsubHand();
    this.unsubHandGrab();
    if (this.gestureHintTimer >= 0) { clearTimeout(this.gestureHintTimer); this.gestureHintTimer = -1; }
    this.exitImmersive();
  }

  /**
   * Expand the window to true fullscreen (hide system bars) and allow
   * rotation between portrait and landscape so the user can frame an
   * AR anchor at any angle. Guarded with try/catch so the page still
   * mounts even if the window API rejects the request.
   */
  private enterImmersive(): void {
    try {
      const host = this.getUIContext().getHostContext();
      if (host === undefined || host === null) return;
      const ctx = host as common.UIAbilityContext;
      window.getLastWindow(ctx).then((win: window.Window): void => {
        // Allow both portrait and landscape — user decides.
        win.setPreferredOrientation(window.Orientation.AUTO_ROTATION).catch((): void => {});
        // True fullscreen: hide status + nav bars (AR canvas wants every pixel).
        win.setWindowSystemBarEnable([]).catch((): void => {});
        // Make sure layout extends under any remaining safe-area insets.
        win.setWindowLayoutFullScreen(true).catch((): void => {});
      }).catch((): void => {});
    } catch (e) { /* ignore */ }
  }

  /**
   * Manual orientation flip — enter/exit landscape for the AR canvas.
   * Bypasses the system-wide "auto-rotate" toggle so it works even when
   * the user has disabled that in Settings (common on phones held
   * steady for camera work).
   */
  private toggleLandscape(): void {
    this.landscape = !this.landscape;
    const target = this.landscape
      ? window.Orientation.AUTO_ROTATION_LANDSCAPE
      : window.Orientation.PORTRAIT;
    try {
      const host = this.getUIContext().getHostContext();
      if (host === undefined || host === null) return;
      const ctx = host as common.UIAbilityContext;
      window.getLastWindow(ctx).then((win: window.Window): void => {
        win.setPreferredOrientation(target).catch((): void => {});
      }).catch((): void => {});
    } catch (e) { /* ignore */ }
  }

  /**
   * Restore the rest of the app to portrait-only with system bars showing.
   * Called on aboutToDisappear so switching tabs returns to the regular
   * chrome.
   */
  private exitImmersive(): void {
    try {
      const host = this.getUIContext().getHostContext();
      if (host === undefined || host === null) return;
      const ctx = host as common.UIAbilityContext;
      window.getLastWindow(ctx).then((win: window.Window): void => {
        win.setPreferredOrientation(window.Orientation.PORTRAIT).catch((): void => {});
        win.setWindowSystemBarEnable(['status', 'navigation']).catch((): void => {});
      }).catch((): void => {});
    } catch (e) { /* ignore */ }
  }

  private toggleMotion(): void {
    if (this.motionOn) {
      this.unsubGesture();
      this.unsubTelemetry();
      this.unsubGesture = (): void => {};
      this.unsubTelemetry = (): void => {};
      this.motionOn = false;
      this.lastGestureLabel = '';
      this.gestureMag = 0;
      return;
    }
    this.motionOn = true;
    this.unsubGesture = GesturePipeline.instance.subscribe((e: GestureEvent): void => {
      this.onGesture(e);
    });
    this.unsubTelemetry = GesturePipeline.instance.subscribeTelemetry((t: GestureTelemetry): void => {
      // Track the dominant world-frame component so the hint chip shows
      // how close the current motion is to crossing the flick threshold.
      const upAbs = Math.abs(t.worldUp);
      const rightAbs = Math.abs(t.worldRight);
      this.gestureMag = upAbs > rightAbs ? upAbs : rightAbs;
    });
    // Show a persistent hint on arm so the user knows the sensor is live
    // and gets a quick cheat-sheet of what gestures do what.
    this.lastGestureLabel = '挥动手机 · 上下 = 展开/合拢 · 左右 = 旋转 · 晃动 = 回中';
    this.lastGestureAt = Date.now();
    if (this.gestureHintTimer >= 0) clearTimeout(this.gestureHintTimer);
    this.gestureHintTimer = setTimeout((): void => {
      // Only clear if no gesture has fired since — otherwise the most recent
      // detection label is already showing and we'd be trampling it.
      if (this.lastGestureAt <= Date.now() - 3800) this.lastGestureLabel = '';
      this.gestureHintTimer = -1;
    }, 4000);
  }

  private onGesture(e: GestureEvent): void {
    switch (e.kind) {
      case 'flickUp':
        this.setExplode(Math.min(1, this.explode + 0.28));
        this.lastGestureLabel = '向上挥 · 展开 +';
        break;
      case 'flickDown':
        this.setExplode(Math.max(0, this.explode - 0.28));
        this.lastGestureLabel = '向下拉 · 展开 -';
        break;
      case 'flickLeft':
        this.bridge.orbitBy(-28, 0);
        this.lastGestureLabel = '左挥 · 旋转';
        break;
      case 'flickRight':
        this.bridge.orbitBy(28, 0);
        this.lastGestureLabel = '右挥 · 旋转';
        break;
      case 'shake':
        this.recenter();
        this.lastGestureLabel = '晃动 · 回中';
        break;
      default: return;
    }
    this.lastGestureAt = e.at;
    // Fire the burst animation — unique trigger key per event so the
    // GestureBurst @Watch picks it up even if the same gesture repeats.
    this.gestureBurstTrigger = e.kind + '|' + e.at;
    if (this.gestureHintTimer >= 0) clearTimeout(this.gestureHintTimer);
    this.gestureHintTimer = setTimeout((): void => {
      this.lastGestureLabel = '';
      this.gestureHintTimer = -1;
    }, 1600);
  }

  // --------- hand-gesture pipeline (camera-based) ---------

  private toggleHandGesture(): void {
    if (this.handOn) {
      this.unsubHand();
      this.unsubHandGrab();
      this.unsubHand = (): void => {};
      this.unsubHandGrab = (): void => {};
      HandGesturePipeline.instance.setExternalSource(false);
      // Actively tell hand.html to release the camera BEFORE ArkUI
      // unmounts the Web component. Without this, the next hand mode
      // toggle frequently hits hcamera error 15 because the previous
      // front-camera session is still pending teardown when the new
      // getUserMedia fires. The WebView's own pagehide handler isn't
      // reliable under ArkUI conditional re-renders.
      try {
        this.handWebController.runJavaScript(
          'typeof window.releaseCamera === "function" && window.releaseCamera();'
        );
      } catch (err) { /* WebView already gone — harmless */ }
      this.handOn = false;
      this.handStage = 'mid';
      this.handCount = 0;
      this.lastGestureLabel = '';
      // Restore the workshop backdrop to whatever the user last asked
      // for (envOn). When hand mode was on, the splat canvas had to go
      // transparent so the camera feed + landmarks could show through.
      // handOn is already false at this point, so `!this.handOn` is
      // redundant but kept for symmetry with the other call sites.
      this.bridge.setEnvironment(this.envOn && !this.handOn);
      return;
    }
    // Announce that the WebView MediaPipe bridge will drive detection.
    // Must be set BEFORE subscribe() so start() doesn't spin up the
    // emulate tick loop.
    HandGesturePipeline.instance.setExternalSource(true);
    this.handOn = true;
    this.lastGestureLabel = '手举到摄像头前 · 拇指食指张合=展开 · 握拳=拖动';
    this.lastGestureAt = Date.now();
    if (this.gestureHintTimer >= 0) clearTimeout(this.gestureHintTimer);
    this.gestureHintTimer = setTimeout((): void => {
      if (this.lastGestureAt <= Date.now() - 3800) this.lastGestureLabel = '';
      this.gestureHintTimer = -1;
    }, 4000);
    this.unsubHand = HandGesturePipeline.instance.subscribe((e: HandGestureEvent): void => {
      this.onHandGesture(e);
    });
    this.unsubHandGrab = HandGesturePipeline.instance.subscribeGrab((e: HandGrabEvent): void => {
      this.onHandGrab(e);
    });
    // Suspend the splat backdrop while hand mode is on — ArViewer's
    // canvas must be transparent for the MediaPipe WebView's camera
    // feed and landmark overlay to be visible underneath. envOn is
    // preserved so turning hand mode OFF restores the backdrop.
    this.bridge.setEnvironment(false);
  }

  /**
   * Drag the anchored model in AR space while a fist is held.
   * dx/dy arrive in normalised camera coords (0..1). We scale them
   * to metres via GRAB_SCALE — 1.8 feels 1:1 for a phone held at
   * arm's length. Both axes are inverted: the ArViewer's pan coord
   * system runs opposite to screen coords (empirically — hand right
   * was moving the model left, and hand down was moving it up).
   */
  private onHandGrab(e: HandGrabEvent): void {
    if (e.active && (e.dx !== 0 || e.dy !== 0)) {
      const GRAB_SCALE = 1.8;
      this.bridge.panTarget(-e.dx * GRAB_SCALE, e.dy * GRAB_SCALE, 0);
      this.lastGestureLabel = '握拳拖动 · 移动位置';
      this.lastGestureAt = e.at;
    } else if (e.active) {
      // Just entered grab mode — anchor captured, no move yet.
      this.lastGestureLabel = '已抓取 · 移动手拖动';
      this.lastGestureAt = e.at;
    } else {
      // Released.
      if (this.lastGestureLabel.startsWith('握拳拖动') || this.lastGestureLabel.startsWith('已抓取')) {
        this.lastGestureLabel = '';
      }
    }
  }

  private onHandGestureCount: number = 0;
  private onHandGesture(e: HandGestureEvent): void {
    this.onHandGestureCount++;
    if (this.onHandGestureCount % 15 === 1) {
      hilog.info(0x1, 'WorkshopHand',
        'onHandGesture #%{public}d stage=%{public}s hands=%{public}d explode=%{public}s',
        this.onHandGestureCount, e.stage, e.handsVisible, e.explode.toFixed(3));
    }
    this.handStage = e.stage;
    this.handCount = e.handsVisible;
    // Continuous explode — detector already smoothed via EMA, so we can
    // write through to the bridge on every event without ringing.
    this.setExplode(e.explode);
    // Light label updates to show the pipeline is live without spam.
    if (e.stage === 'closed' && this.handCount > 0) {
      this.lastGestureLabel = '合拢 · 复位';
      this.lastGestureAt = e.at;
    } else if (e.stage === 'open' && this.handCount > 0) {
      this.lastGestureLabel = '张开 · 全展开';
      this.lastGestureAt = e.at;
    }
  }

  private setExplode(v: number): void {
    this.explode = v;
    this.bridge.setExplode(v);
    // Propagate to peer when we are driving the session.
    NebulaSync.instance.updateExplode(v);
  }

  // ---------- palette ----------

  private mkSpec(k: string, v: string): SpecLine {
    const s = new SpecLine(); s.key = k; s.value = v; return s;
  }

  private mkPart(id: string, kind: string, display: string, h: number,
                 rT: number, rB: number, color: string, accent: string,
                 shortDesc: string, fullDesc: string, specs: SpecLine[]): PartSpec {
    const p = new PartSpec();
    p.id = id + '-' + Date.now();
    p.name = id; p.display = display; p.kind = kind;
    p.height = h; p.radiusTop = rT; p.radiusBottom = rB;
    p.color = color; p.accent = accent;
    p.shortDesc = shortDesc; p.fullDesc = fullDesc; p.specs = specs;
    return p;
  }

  private palette(): PaletteEntry[] {
    const mk = (id: string, kind: string, label: string, tone: string, icon: string,
                f: () => PartSpec): PaletteEntry => {
      const e = new PaletteEntry();
      e.id = id; e.kind = kind; e.label = label; e.tone = tone; e.icon = icon;
      e.factory = f; return e;
    };
    return [
      // ---- Satellite kit (default home is 自制卫星 — put these first) ----
      mk('p-bus', 'bus', '平台舱', 'brand', IconName.Satellite,
        (): PartSpec => this.mkPart('p-bus', 'bus', '平台舱', 1.1, 0.58, 0.58,
          '#F6F8FB', '#2E6BF6', '卫星主体平台',
          '承载电源、姿控、推进与载荷。',
          [this.mkSpec('形态', '六面体总装'), this.mkSpec('寿命', '8 – 15 年')])),
      mk('p-solar-l', 'solar-left', '太阳翼 · 左', 'violet', IconName.Sparkle,
        (): PartSpec => this.mkPart('p-solar-l', 'solar-left', '太阳翼 · 左', 2.0, 0.02, 0.6,
          '#1F3A8A', '#7F5CFF', '取电主力 · 左翼',
          '三结砷化镓太阳能板，展开对日取电。',
          [this.mkSpec('功率', '~2 kW')])),
      mk('p-solar-r', 'solar-right', '太阳翼 · 右', 'violet', IconName.Sparkle,
        (): PartSpec => this.mkPart('p-solar-r', 'solar-right', '太阳翼 · 右', 2.0, 0.02, 0.6,
          '#1F3A8A', '#7F5CFF', '取电主力 · 右翼',
          '与左翼对称展开。',
          [this.mkSpec('对日精度', '±1°')])),
      mk('p-antenna', 'antenna', '高增益天线', 'gold', IconName.Signal,
        (): PartSpec => this.mkPart('p-antenna', 'antenna', '高增益天线', 0.55, 0.06, 0.28,
          '#F8FAFE', '#E9A23B', '指向地球的数据天线',
          '用于高码率对地下行。',
          [this.mkSpec('频段', 'Ka')])),
      mk('p-payload-sat', 'payload', '科学载荷', 'cyan', IconName.Orbit,
        (): PartSpec => this.mkPart('p-payload-sat', 'payload', '科学载荷', 0.7, 0.42, 0.42,
          '#DDE4F0', '#11B7D6', '相机 / 光谱仪 / 传感器',
          '载荷舱承载具体任务仪器。',
          [this.mkSpec('可选', '相机 · 雷达 · 光谱仪')])),
      mk('p-thruster', 'engine', '推进器', 'coral', IconName.Flame,
        (): PartSpec => this.mkPart('p-thruster', 'engine', '霍尔推进器', 0.35, 0.4, 0.55,
          '#B0B9CC', '#E5554A', '轨道保持 / 机动',
          '电推用于位保、姿控与轨道机动。',
          [this.mkSpec('类型', 'Hall / Ion')])),
      // ---- Rocket kit ----
      mk('p-nose', 'nose', '尖端锥', 'brand', IconName.Rocket,
        (): PartSpec => this.mkPart('p-nose', 'nose', '气动锥', 0.9, 0.02, 0.42,
          '#F8FAFE', '#2E6BF6', '引导气流、包裹载荷',
          '尖端锥在上升段承担气动整形任务。',
          [this.mkSpec('材料', '铝合金蒙皮')])),
      mk('p-payload', 'payload', '载荷舱', 'violet', IconName.Satellite,
        (): PartSpec => this.mkPart('p-payload', 'payload', '载荷舱', 0.7, 0.38, 0.38,
          '#DDE4F0', '#7F5CFF', '装载卫星或飞船',
          '载荷舱负责承载任务载荷。',
          [this.mkSpec('运力', '可自定义')])),
      mk('p-fairing', 'fairing', '整流罩', 'cyan', IconName.Orbit,
        (): PartSpec => this.mkPart('p-fairing', 'fairing', '整流罩', 1.1, 0.04, 0.52,
          '#F7FAFD', '#11B7D6', '保护载荷穿越大气层',
          '整流罩在高空分离，多为两瓣式设计。',
          [this.mkSpec('分离高度', '~110 km')])),
      mk('p-inter', 'interstage', '级间段', 'gold', IconName.Layers,
        (): PartSpec => this.mkPart('p-inter', 'interstage', '级间段', 0.45, 0.68, 0.68,
          '#8A96B2', '#E9A23B', '连接两级、带分离机构',
          '集成分离机构、栅格翼与电缆路径。',
          [this.mkSpec('材料', '复合材料')])),
      mk('p-stage2', 'stage', '二级', 'brand', IconName.Rocket,
        (): PartSpec => this.mkPart('p-stage2', 'stage', '二级', 1.4, 0.55, 0.55,
          '#F7FAFD', '#2E6BF6', '完成入轨点火',
          '二级负责把载荷送入预定轨道。',
          [this.mkSpec('比冲', '~345 s')])),
      mk('p-stage1', 'stage', '一级芯体', 'neutral', IconName.Rocket,
        (): PartSpec => this.mkPart('p-stage1', 'stage', '一级', 3.0, 0.72, 0.72,
          '#FBFCFE', '#2E6BF6', '主要推力来源',
          '一级承担升空阶段的大部分推力。',
          [this.mkSpec('推力', '7 MN+')])),
      mk('p-boost-l', 'booster', '助推器 · 左', 'mint', IconName.Bolt,
        (): PartSpec => this.mkPart('p-boost-l', 'booster', '助推器 · 左', 2.2, 0.32, 0.32,
          '#F8FAFE', '#11B7D6', '并联绑火', '提升首段推力。',
          [this.mkSpec('推力', '1.2 MN')])),
      mk('p-boost-r', 'booster', '助推器 · 右', 'mint', IconName.Bolt,
        (): PartSpec => this.mkPart('p-boost-r', 'booster', '助推器 · 右', 2.2, 0.32, 0.32,
          '#F8FAFE', '#11B7D6', '与左侧对称', '对称助推器。',
          [this.mkSpec('推力', '1.2 MN')])),
      mk('p-engine', 'engine', '发动机', 'coral', IconName.Flame,
        (): PartSpec => this.mkPart('p-engine', 'engine', '发动机裙摆', 0.38, 0.8, 1.05,
          '#B0B9CC', '#E5554A', '产生推力', '底部发动机裙摆。',
          [this.mkSpec('台数', '5')]))
    ];
  }

  private loadBlueprint(id: string): void {
    // Accept either a rocket or a satellite id. Workshop treats them
    // uniformly — both have `parts`, `display`, and `modelUrl`. The
    // default on first mount is 'beidou-3' (a satellite), so this
    // lookup MUST try both tables.
    const rocket: RocketBlueprint | null = Catalog.rocketById(id);
    const sat: SatelliteBlueprint | null = rocket === null ? Catalog.satelliteById(id) : null;
    if (rocket === null && sat === null) return;
    const parts: PartSpec[] = rocket !== null ? rocket.parts
                            : sat !== null ? sat.parts : [];
    const modelUrl: string = rocket !== null ? rocket.modelUrl
                           : sat !== null ? sat.modelUrl : '';
    const display: string = rocket !== null ? rocket.display
                          : sat !== null ? sat.display : '';
    this.rocketId = id;
    const next: PartSpec[] = [];
    for (const p of parts) next.push(p);
    this.parts = next;
    this.selectedId = '';
    if (modelUrl.length > 0) {
      this.bridge.loadBundledModel(modelUrl, 3.5);
    } else {
      this.sync();
    }
    this.pushBuild();
    UserStats.instance.recordOpenWorkshop(display).then((): void => {});
  }

  private toViewerParts(): ViewerPart[] {
    const list: ViewerPart[] = [];
    for (const p of this.parts) {
      const v = new ViewerPart();
      v.id = p.id; v.kind = p.kind; v.display = p.display;
      v.color = p.color; v.accent = p.accent;
      v.height = p.height; v.radiusTop = p.radiusTop; v.radiusBottom = p.radiusBottom;
      const o = new ViewerOffset(); o.x = 0; o.y = 0.2; o.z = 0;
      v.explodeOffset = o;
      list.push(v);
    }
    return list;
  }

  private sync(): void {
    this.bridge.setParts(this.toViewerParts());
    this.bridge.setExplode(this.explode);
  }

  private pushBuild(): void {
    const build: BuildPart[] = [];
    for (const p of this.parts) {
      const b = new BuildPart();
      b.partId = p.id; b.kind = p.kind; b.display = p.display;
      b.color = p.color; b.height = p.height;
      b.radiusTop = p.radiusTop; b.radiusBottom = p.radiusBottom;
      build.push(b);
    }
    NebulaSync.instance.updateBuild(build);
    NebulaSync.instance.updateRocket(this.rocketId);
  }

  private onReady(_e: ArReadyEvent): void {
    // The first loadBlueprint() runs in aboutToAppear — BEFORE the WebView
    // fires onReady — so its bridge calls are swallowed. Re-apply the
    // blueprint here: load the GLB if one exists, otherwise fall back to
    // the procedural stack. Without this, Workshop always showed the
    // procedural mock even when a real model was wired up. Default id is
    // 'beidou-3' (satellite), so resolve against both tables.
    const rocket: RocketBlueprint | null = Catalog.rocketById(this.rocketId);
    const sat: SatelliteBlueprint | null = rocket === null
      ? Catalog.satelliteById(this.rocketId) : null;
    const modelUrl: string = rocket !== null ? rocket.modelUrl
                           : sat !== null ? sat.modelUrl : '';
    if (modelUrl.length > 0) {
      this.bridge.loadBundledModel(modelUrl, 3.5);
      this.bridge.setExplode(this.explode);
    } else {
      this.sync();
    }
    this.bridge.setControlMode(this.controlMode);
    this.bridge.recenterCamera();
    // Apply the backdrop choice once the bridge is live — before this,
    // setEnvironment() would have been swallowed like the blueprint
    // load was. Hand mode takes precedence (splat canvas must stay
    // transparent so the MediaPipe camera feed shows through).
    this.bridge.setEnvironment(this.envOn && !this.handOn);
  }

  private toggleEnvironment(): void {
    this.envOn = !this.envOn;
    // Effective env = envOn && !handOn. Hand mode always wins over the
    // splat backdrop because the AR canvas can't be both opaque (for
    // splat) and transparent (for camera feed) at the same time.
    this.bridge.setEnvironment(this.envOn && !this.handOn);
    // ALSO clear/restore the rocket model from the ArkTS side. The
    // JS-side detach in setEnvironment is the primary hide mechanism,
    // but using the older clearModel() / setParts() bridge calls here
    // gives us belt-and-suspenders — these are part of the long-stable
    // bridge surface, so even if the WebView is running a stale ar.html
    // (e.g. after a partial hot-reload that skipped rawfile assets),
    // the satellite still goes away when 空间站 turns on.
    if (this.envOn) {
      this.bridge.clearModel();
    } else {
      this.restoreCurrentBlueprint();
    }
    // 120 MB ISS splat parse can run several seconds on first toggle —
    // show the loading overlay until the WebView fires splat:ready (or
    // splat:failed). Subsequent toggles are instant because the splat
    // stays loaded; suppress the overlay if it's already in memory.
    if (this.envOn && !this.handOn && !this.splatReady) {
      this.splatLoading = true;
    } else {
      this.splatLoading = false;
    }
  }

  private restoreCurrentBlueprint(): void {
    const rocket: RocketBlueprint | null = Catalog.rocketById(this.rocketId);
    const sat: SatelliteBlueprint | null = rocket === null
      ? Catalog.satelliteById(this.rocketId) : null;
    const modelUrl: string = rocket !== null ? rocket.modelUrl
                           : sat !== null ? sat.modelUrl : '';
    if (modelUrl.length > 0) {
      this.bridge.loadBundledModel(modelUrl, 3.5);
      this.bridge.setExplode(this.explode);
    } else {
      this.sync();
    }
  }

  private toggleControlMode(): void {
    this.controlMode = this.controlMode === 'touch' ? 'motion' : 'touch';
    this.bridge.setControlMode(this.controlMode);
    NebulaSync.instance.updateControlMode(this.controlMode);
  }

  private recenter(): void {
    this.bridge.recenterCamera();
  }

  private orbit(dYaw: number, dPitch: number): void {
    this.bridge.orbitBy(dYaw, dPitch);
    if (this.controlMode !== 'touch') {
      this.controlMode = 'touch';
      this.bridge.setControlMode('touch');
    }
  }
  private onSelect(e: ArSelectEvent): void {
    this.selectedId = e.partId ?? '';
    if (this.selectedId.length > 0) this.sheetOpen = true;
    NebulaSync.instance.updateSelectedPart(this.selectedId);
  }
  private onAnchor(_e: ArAnchorEvent): void { this.anchored = true; }

  private onSplat(e: ArSplatEvent): void {
    if (e.status === 'ready') {
      this.splatReady = true;
      this.splatLoading = false;
    } else if (e.status === 'failed') {
      this.splatLoading = false;
    }
  }

  // ---- analog navigation handlers -----------------------------------
  // Left joystick: dy negative = up = walk forward.
  private onWalkStick(dx: number, dy: number): void {
    if (dx === 0 && dy === 0) return;
    this.bridge.walkMove(-dy, dx);
  }

  // Right joystick: dx = yaw (rotate horizontally), dy = pitch (rotate
  // vertically). dy negative (push up) means look up, hence the
  // negation when forwarding to lookBy.
  private onLookStick(dx: number, dy: number): void {
    if (dx === 0 && dy === 0) return;
    this.bridge.lookBy(dx, -dy);
  }

  private addPart(entry: PaletteEntry): void {
    const part = entry.factory();
    const next: PartSpec[] = [];
    for (const p of this.parts) next.push(p);
    next.push(part);
    this.parts = next;
    this.sync();
    this.pushBuild();
  }

  private removePart(id: string): void {
    const next: PartSpec[] = [];
    for (const p of this.parts) { if (p.id !== id) next.push(p); }
    this.parts = next;
    if (this.selectedId === id) { this.selectedId = ''; this.sheetOpen = false; }
    this.sync();
    this.pushBuild();
  }

  private clear(): void {
    this.parts = []; this.selectedId = ''; this.sheetOpen = false;
    this.sync(); this.pushBuild();
  }

  private onExplode(v: number): void {
    this.explode = v;
    this.bridge.setExplode(v);
    NebulaSync.instance.updateExplode(v);
  }

  private resetAnchor(): void {
    this.anchored = false; this.bridge.resetAnchor();
  }

  private selectedPart(): PartSpec | null {
    if (this.selectedId.length === 0) return null;
    for (const p of this.parts) { if (p.id === this.selectedId) return p; }
    return null;
  }

  private toneBg(tone: string): string {
    if (tone === 'brand') return NebulaColor.brandSoft;
    if (tone === 'violet') return NebulaColor.accentVioletSoft;
    if (tone === 'cyan') return NebulaColor.accentCyanSoft;
    if (tone === 'gold') return NebulaColor.accentGoldSoft;
    if (tone === 'mint') return NebulaColor.accentMintSoft;
    if (tone === 'coral') return NebulaColor.accentCoralSoft;
    return NebulaColor.bgSoft;
  }
  private toneFg(tone: string): string {
    if (tone === 'brand') return NebulaColor.brand;
    if (tone === 'violet') return NebulaColor.accentViolet;
    if (tone === 'cyan') return NebulaColor.accentCyan;
    if (tone === 'gold') return NebulaColor.accentGold;
    if (tone === 'mint') return NebulaColor.accentMint;
    if (tone === 'coral') return NebulaColor.accentCoral;
    return NebulaColor.textPrimary;
  }

  // ---------- builders ----------

  @Builder topBar() {
    // Mission-control toggle strip — 5 terminal-style chips + a sync
    // status badge inside a horizontal Scroll. Chips are inlined (not
    // extracted to a shared @Builder) because ArkTS's @Builder caches
    // renders by parameter identity for primitives, so passing a
    // boolean `active` flag can fail to re-render when state flips —
    // which is why selected chips previously stayed gray.
    Scroll() {
      Row({ space: 6 }) {
        // 手势 — camera-based hand gesture control
        Row({ space: 5 }) {
          NebulaIcon({
            name: IconName.Hand, glyphSize: 12, strokeW: 2,
            color: this.handOn ? '#0B0B0E' : NebulaColor.textSecondary
          })
          Text('手势')
            .fontSize('11vp')
            .fontFamily(NebulaFontFace.sans)
            .fontWeight(this.handOn ? FontWeight.Medium : FontWeight.Regular)
            .fontColor(this.handOn ? '#0B0B0E' : NebulaColor.textSecondary)
        }
        .padding({ top: 7, bottom: 7, left: 10, right: 12 })
        .backgroundColor(this.handOn ? NebulaColor.brand : '#0B0B0EE6')
        .border({ width: 1, color: this.handOn ? NebulaColor.brand : NebulaColor.border })
        .borderRadius(NebulaRadius.tight)
        .onClick((): void => { this.toggleHandGesture(); })

        // 体感 — accelerometer flicks
        Row({ space: 5 }) {
          NebulaIcon({
            name: IconName.Motion, glyphSize: 12, strokeW: 2,
            color: this.motionOn ? '#0B0B0E' : NebulaColor.textSecondary
          })
          Text('体感')
            .fontSize('11vp')
            .fontFamily(NebulaFontFace.sans)
            .fontWeight(this.motionOn ? FontWeight.Medium : FontWeight.Regular)
            .fontColor(this.motionOn ? '#0B0B0E' : NebulaColor.textSecondary)
        }
        .padding({ top: 7, bottom: 7, left: 10, right: 12 })
        .backgroundColor(this.motionOn ? NebulaColor.brand : '#0B0B0EE6')
        .border({ width: 1, color: this.motionOn ? NebulaColor.brand : NebulaColor.border })
        .borderRadius(NebulaRadius.tight)
        .onClick((): void => { this.toggleMotion(); })

        // 横屏 / 竖屏 — orientation lock
        Row({ space: 5 }) {
          NebulaIcon({
            name: IconName.Rotate, glyphSize: 12, strokeW: 2,
            color: this.landscape ? '#0B0B0E' : NebulaColor.textSecondary
          })
          Text(this.landscape ? '竖屏' : '横屏')
            .fontSize('11vp')
            .fontFamily(NebulaFontFace.sans)
            .fontWeight(this.landscape ? FontWeight.Medium : FontWeight.Regular)
            .fontColor(this.landscape ? '#0B0B0E' : NebulaColor.textSecondary)
        }
        .padding({ top: 7, bottom: 7, left: 10, right: 12 })
        .backgroundColor(this.landscape ? NebulaColor.brand : '#0B0B0EE6')
        .border({ width: 1, color: this.landscape ? NebulaColor.brand : NebulaColor.border })
        .borderRadius(NebulaRadius.tight)
        .onClick((): void => { this.toggleLandscape(); })

        // 空间站 — ISS splat backdrop
        Row({ space: 5 }) {
          NebulaIcon({
            name: IconName.Orbit, glyphSize: 12, strokeW: 2,
            color: this.envOn ? '#0B0B0E' : NebulaColor.textSecondary
          })
          Text('空间站')
            .fontSize('11vp')
            .fontFamily(NebulaFontFace.sans)
            .fontWeight(this.envOn ? FontWeight.Medium : FontWeight.Regular)
            .fontColor(this.envOn ? '#0B0B0E' : NebulaColor.textSecondary)
        }
        .padding({ top: 7, bottom: 7, left: 10, right: 12 })
        .backgroundColor(this.envOn ? NebulaColor.brand : '#0B0B0EE6')
        .border({ width: 1, color: this.envOn ? NebulaColor.brand : NebulaColor.border })
        .borderRadius(NebulaRadius.tight)
        .onClick((): void => { this.toggleEnvironment(); })

        // 跨端 — one-shot handoff trigger (not a toggle)
        Row({ space: 5 }) {
          NebulaIcon({
            name: IconName.Transfer, glyphSize: 12, strokeW: 2,
            color: NebulaColor.textSecondary
          })
          Text('跨端')
            .fontSize('11vp')
            .fontFamily(NebulaFontFace.sans)
            .fontColor(NebulaColor.textSecondary)
        }
        .padding({ top: 7, bottom: 7, left: 10, right: 12 })
        .backgroundColor('#0B0B0EE6')
        .border({ width: 1, color: NebulaColor.border })
        .borderRadius(NebulaRadius.tight)
        .onClick((): void => { AppBus.instance.openHandoffSheet(this.currentHandoffPayload()); })

        // Sync / controller status — icon-only in solo mode to claw
        // back horizontal space; expands to "MAIN · N" / "OBSERVE"
        // when actually driving a multi-device session.
        Row({ space: this.syncRole === 'solo' ? 0 : 6 }) {
          NebulaIcon({
            name: IconName.Signal, glyphSize: 12,
            color: this.syncRole === 'controller' ? NebulaColor.accentMint
                 : this.syncRole === 'observer'   ? NebulaColor.brand
                 :                                   NebulaColor.accentMint,
            strokeW: 2
          })
          if (this.syncRole !== 'solo') {
            Text(this.controllerLabel())
              .fontSize('10vp')
              .fontFamily(NebulaFontFace.mono)
              .fontColor(NebulaColor.textPrimary)
              .letterSpacing(1.8)
          }
        }
        .padding(this.syncRole === 'solo'
          ? { left: 9, right: 9, top: 9, bottom: 9 }
          : { left: 10, right: 12, top: 8, bottom: 8 })
        .backgroundColor('#0B0B0EE6')
        .border({ width: 1, color: NebulaColor.border })
        .borderRadius(NebulaRadius.tight)
        .backdropBlur(20)
        // Trailing breathing room so the last pill doesn't butt into
        // the screen edge when scrolled fully to the right.
        Column().width(4)
      }
    }
    .scrollable(ScrollDirection.Horizontal)
    .scrollBar(BarState.Off)
    .edgeEffect(EdgeEffect.Spring)
    .width('100%')
  }

  private controllerLabel(): string {
    // Returned labels land in the terminal-styled sync chip — mono +
    // uppercase. Solo falls back to the driver name ("IPHONE" etc).
    if (this.syncRole === 'controller') return 'MAIN · ' + this.peerCount;
    if (this.syncRole === 'observer') return 'OBSERVE';
    return this.controller.toUpperCase();
  }

  private currentHandoffPayload(): HandoffPayload {
    const p = new HandoffPayload();
    p.kind = 'workshop';
    p.rocketId = this.rocketId;
    p.explode = this.explode;
    p.selectedPart = this.selectedId;
    p.controlMode = this.controlMode;
    p.issuedAt = Date.now();
    return p;
  }

  @Builder actionRail() {
    Column({ space: 10 }) {
      this.railButton(IconName.Reset, NebulaColor.accentCoral,
        (): void => this.recenter())
      this.railButton(IconName.Share, NebulaColor.accentGold,
        (): void => { this.onShare(); })
    }
  }

  private toast(message: string): void {
    try {
      this.getUIContext().getPromptAction().showToast({ message, duration: 1800 });
    } catch (e) { /* ignore */ }
  }

  private hostContext(): common.UIAbilityContext | null {
    try {
      const host = this.getUIContext().getHostContext();
      return host !== undefined && host !== null ? host as common.UIAbilityContext : null;
    } catch (e) { return null; }
  }

  private buildShareCard(): CardContent {
    const rocket: RocketBlueprint | null = Catalog.rocketById(this.rocketId);
    const sat: SatelliteBlueprint | null = rocket === null
      ? Catalog.satelliteById(this.rocketId) : null;
    const display: string = rocket !== null ? rocket.display
                          : sat !== null ? sat.display
                          : (this.rocketId.length > 0 ? this.rocketId : '自定义飞行器');
    const card = new CardContent();
    card.title = display;
    card.kind = rocket !== null ? 'ROCKET' : (sat !== null ? 'SATELLITE' : 'CUSTOM');
    const partCount = this.parts.length;
    card.description = '在 AR 工坊中搭建的' + display + '，共 ' + partCount + ' 个部件。';
    const specs: CardSpec[] = [];
    const partsSpec = new CardSpec();
    partsSpec.key = '部件数'; partsSpec.value = String(partCount);
    specs.push(partsSpec);
    const modeSpec = new CardSpec();
    modeSpec.key = '操控模式'; modeSpec.value = this.controlMode === 'touch' ? '按钮' : '漂浮';
    specs.push(modeSpec);
    card.specs = specs;
    return card;
  }

  private onShare(): void {
    const ctx = this.hostContext();
    if (ctx === null) { this.toast('无法获取应用上下文'); return; }
    const card = this.buildShareCard();
    UserStats.instance.recordShare(card.title).then((): void => {});
    ShareCard.share(ctx, card).then((r: ShareResult): void => {
      if (r.channel === 'system-share') this.toast('已打开分享面板');
      else if (r.channel === 'clipboard') this.toast('卡片已复制到剪贴板');
      else this.toast('分享失败');
    });
  }

  @Builder modeSwitcher() {
    Row({ space: 0 }) {
      this.modePill('FLOAT', 'motion', IconName.Compass, true)
      this.modePill('TOUCH', 'touch', IconName.Orbit, false)
    }
    .backgroundColor('#0B0B0EE6')
    .border({ width: 1, color: NebulaColor.border })
    .borderRadius(NebulaRadius.tight)
    .backdropBlur(20)
  }

  @Builder modePill(label: string, mode: string, icon: string, first: boolean) {
    Row({ space: 6 }) {
      NebulaIcon({
        name: icon, glyphSize: 12,
        color: this.controlMode === mode ? '#0B0B0E' : NebulaColor.textTertiary,
        strokeW: 2
      })
      Text(label)
        .fontSize('10vp')
        .fontFamily(NebulaFontFace.mono)
        .fontWeight(this.controlMode === mode ? FontWeight.Medium : FontWeight.Regular)
        .fontColor(this.controlMode === mode ? '#0B0B0E' : NebulaColor.textTertiary)
        .letterSpacing(2.0)
    }
    .padding({ top: 9, bottom: 9, left: 14, right: 14 })
    .backgroundColor(this.controlMode === mode ? NebulaColor.brand : Color.Transparent)
    .border({
      width: { top: 0, bottom: 0, left: first ? 0 : 1, right: 0 },
      color: NebulaColor.border,
      style: BorderStyle.Solid
    })
    .onClick((): void => {
      if (this.controlMode !== mode) {
        this.controlMode = mode;
        this.bridge.setControlMode(mode);
        NebulaSync.instance.updateControlMode(mode);
      }
    })
  }

  @Builder railButton(icon: string, color: string, onTap: () => void) {
    Column() {
      NebulaIcon({ name: icon, glyphSize: 16, color, strokeW: 2 })
    }
    .width(42).height(42)
    .backgroundColor('#0B0B0EE6')
    .border({ width: 1, color: NebulaColor.border })
    .borderRadius(NebulaRadius.tight)
    .backdropBlur(20)
    .justifyContent(FlexAlign.Center)
    .alignItems(HorizontalAlign.Center)
    .onClick(onTap)
  }

  @Builder padButton(rotation: number, onTap: () => void) {
    Column() {
      NebulaIcon({ name: IconName.Arrow, glyphSize: 14, color: NebulaColor.textPrimary, strokeW: 2 })
        .rotate({ angle: rotation })
    }
    .width(38).height(38)
    .backgroundColor('#0B0B0EE6')
    .border({ width: 1, color: NebulaColor.border })
    .borderRadius(NebulaRadius.tight)
    .backdropBlur(20)
    .justifyContent(FlexAlign.Center)
    .alignItems(HorizontalAlign.Center)
    .onClick(onTap)
  }

  @Builder padGlyphButton(icon: string, onTap: () => void) {
    Column() {
      NebulaIcon({ name: icon, glyphSize: 14, color: NebulaColor.textPrimary, strokeW: 2 })
    }
    .width(38).height(38)
    .backgroundColor('#0B0B0EE6')
    .border({ width: 1, color: NebulaColor.border })
    .borderRadius(NebulaRadius.tight)
    .backdropBlur(20)
    .justifyContent(FlexAlign.Center)
    .alignItems(HorizontalAlign.Center)
    .onClick(onTap)
  }

  // Tight elevation button (up/down) — matches the Devices console
  // chrome. 4px corner + 1px hairline + mono arrow glyph.
  @Builder elevButton(icon: string, rotation: number, onTap: () => void) {
    Column() {
      NebulaIcon({ name: icon, glyphSize: 14, color: NebulaColor.textPrimary, strokeW: 2 })
        .rotate({ angle: rotation })
    }
    .width(46).height(40)
    .backgroundColor('#0B0B0EE6')
    .border({ width: 1, color: NebulaColor.border })
    .borderRadius(NebulaRadius.tight)
    .backdropBlur(20)
    .justifyContent(FlexAlign.Center)
    .alignItems(HorizontalAlign.Center)
    .onClick(onTap)
  }

  private zoom(factor: number): void {
    this.bridge.zoomBy(factor);
    if (this.controlMode !== 'touch') {
      this.controlMode = 'touch';
      this.bridge.setControlMode('touch');
    }
  }

  @Builder dPad() {
    Column({ space: 6 }) {
      // rotation pad — proper "+" cross. The middle row keeps an
      // empty 38vp slot between 左/右 so the d-pad reads as a true
      // four-way pad instead of two squashed-together buttons.
      Row() { this.padButton(-90, (): void => this.orbit(0, -12)) }
      Row({ space: 6 }) {
        this.padButton(180, (): void => this.orbit(-18, 0))
        Column().width(38).height(38)
        this.padButton(0,   (): void => this.orbit(18, 0))
      }
      Row() { this.padButton(90,  (): void => this.orbit(0, 12)) }
      // divider
      Column().width(28).height(1).backgroundColor('#52525B').margin({ top: 4, bottom: 4 })
      // zoom pad (远近)
      Row({ space: 6 }) {
        this.padGlyphButton(IconName.Plus,  (): void => this.zoom(0.82))
        this.padGlyphButton(IconName.Minus, (): void => this.zoom(1.22))
      }
    }
    .alignItems(HorizontalAlign.Center)
  }

  @Builder paletteTile(entry: PaletteEntry) {
    Column({ space: 8 }) {
      Stack() {
        Circle({ width: 44, height: 44 }).fill(this.toneBg(entry.tone))
        NebulaIcon({ name: entry.icon, glyphSize: 20, color: this.toneFg(entry.tone), strokeW: 2 })
      }
      .width(44).height(44)
      Text(entry.label)
        .fontSize(NebulaFont.caption)
        .fontColor(NebulaColor.textSecondary)
        .maxLines(1)
    }
    .width(74)
    .padding({ top: 8, bottom: 8 })
    .onClick((): void => this.addPart(entry))
  }

  @Builder drawerBody() {
    Column({ space: 14 }) {
      Row() {
        Text('AR 工坊')
          .fontSize(NebulaFont.titleLg)
          .fontWeight(FontWeight.Bold)
          .fontColor(NebulaColor.textPrimary)
        Blank()
        Row({ space: 6 }) {
          NebulaIcon({ name: IconName.Reset, glyphSize: 14, color: NebulaColor.textSecondary, strokeW: 2 })
          Text('清空')
            .fontSize(NebulaFont.small)
            .fontColor(NebulaColor.textSecondary)
        }
        .padding({ top: 6, bottom: 6, left: 12, right: 12 })
        .backgroundColor(NebulaColor.bgSoft)
        .borderRadius(999)
        .onClick((): void => this.clear())
      }
      .width('100%')

      // 模板
      Scroll() {
        Row({ space: NebulaSpace.sm }) {
          Text('模板')
            .fontSize(NebulaFont.small)
            .fontColor(NebulaColor.textTertiary)
            .margin({ right: 4 })
          ForEach(this.rocketOptions, (c: Chip): void => {
            Text(c.label)
              .fontSize(NebulaFont.small)
              .fontWeight(FontWeight.Medium)
              .fontColor(this.rocketId === c.id ? '#FFFFFF' : NebulaColor.textSecondary)
              .backgroundColor(this.rocketId === c.id ? NebulaColor.brand : NebulaColor.bgSoft)
              .padding({ top: 6, bottom: 6, left: 12, right: 12 })
              .borderRadius(999)
              .onClick((): void => this.loadBlueprint(c.id))
          }, (c: Chip): string => c.id)
        }
      }
      .scrollable(ScrollDirection.Horizontal)
      .scrollBar(BarState.Off)
      .width('100%')

      // 零件货架
      Column({ space: 6 }) {
        Text('零件货架')
          .fontSize(NebulaFont.caption)
          .fontColor(NebulaColor.textTertiary)
        Scroll() {
          Row({ space: 4 }) {
            ForEach(this.palette(), (entry: PaletteEntry): void => {
              this.paletteTile(entry)
            }, (entry: PaletteEntry): string => entry.id)
          }
        }
        .scrollable(ScrollDirection.Horizontal)
        .scrollBar(BarState.Off)
        .width('100%')
      }
      .alignItems(HorizontalAlign.Start)
      .width('100%')

      // explode slider
      Column({ space: 6 }) {
        Row() {
          Text('爆炸程度')
            .fontSize(NebulaFont.caption)
            .fontColor(NebulaColor.textTertiary)
          Blank()
          Text(Math.round(this.explode * 100) + '%')
            .fontSize(NebulaFont.caption)
            .fontColor(NebulaColor.textPrimary)
            .fontWeight(FontWeight.Medium)
        }
        .width('100%')
        Slider({
          value: this.explode * 100,
          min: 0, max: 100,
          style: SliderStyle.OutSet
        })
          .blockColor(NebulaColor.brand)
          .selectedColor(NebulaColor.brand)
          .trackColor(NebulaColor.bgSoft)
          .onChange((v: number): void => this.onExplode(v / 100))
      }
      .width('100%')

      // stack count
      Row() {
        Text('当前组装 · ' + this.parts.length + ' 件')
          .fontSize(NebulaFont.small)
          .fontColor(NebulaColor.textSecondary)
        Blank()
        // Tappable "collapse" chip — matches the drag handle at top so
        // users have a clickable exit wherever their thumb happens to be.
        Row({ space: 4 }) {
          NebulaIcon({ name: IconName.Close, glyphSize: 12, color: NebulaColor.textSecondary, strokeW: 2 })
          Text('下拉收起')
            .fontSize(NebulaFont.caption)
            .fontColor(NebulaColor.textSecondary)
        }
        .padding({ top: 6, bottom: 6, left: 10, right: 12 })
        .backgroundColor(NebulaColor.bgSoft)
        .borderRadius(999)
        .onClick((): void => { this.drawerOpen = false; })
      }
      .width('100%')
    }
    .alignItems(HorizontalAlign.Start)
  }

  @Builder drawer() {
    Column() {
      // Drag-handle strip. Tap anywhere on this whole strip to close,
      // or swipe down. The "下拉收起" hint lower in the body names the
      // same interaction — without either the drawer has no exit path
      // because the toggle button sits BEHIND this panel in Stack z-order.
      Column() {
        Column()
          .width(44).height(5)
          .backgroundColor(NebulaColor.textSecondary)
          .borderRadius(3)
      }
      .width('100%')
      .padding({ top: 10, bottom: 12 })
      .alignItems(HorizontalAlign.Center)
      .onClick((): void => { this.drawerOpen = false; })
      .gesture(
        // Down-only pan with a 24px activation distance. Using
        // PanDirection.Down + distance=24 means the gesture only fires
        // when the user genuinely swipes downward past the threshold,
        // so we can close on onActionStart without inspecting an event
        // object — which sidesteps the naming conflict between ArkUI's
        // built-in GestureEvent type and our custom one imported from
        // ../services/GesturePipeline.
        PanGesture({ direction: PanDirection.Down, distance: 24 })
          .onActionStart((): void => { this.drawerOpen = false; })
      )
      Column() {
        this.drawerBody()
      }
      .padding({ left: 20, right: 20, bottom: 88 })
    }
    .width('100%')
    .backgroundColor('#0A0A0CF4')
    .border({ width: { top: 1 }, color: '#27272AE6' })
    .borderRadius({ topLeft: 28, topRight: 28 })
    .backdropBlur(40)
    .shadow({ radius: 36, color: '#00000080', offsetX: 0, offsetY: -10 })
    .onClick((): void => {})
  }

  @Builder partSheet() {
    if (this.selectedPart() !== null) {
      Column() {
        Column()
          .width(36).height(4)
          .backgroundColor(NebulaColor.border)
          .borderRadius(2)
          .margin({ top: 8, bottom: 14 })

        Column({ space: 10 }) {
          Row() {
            NebulaPill({
              label: this.selectedPart()!.kind.toUpperCase(),
              tone: 'brand', solid: true, iconName: IconName.Spark
            })
            Blank()
            Row() {
              NebulaIcon({ name: IconName.Close, glyphSize: 14, color: NebulaColor.textSecondary, strokeW: 2 })
            }
            .padding(8)
            .backgroundColor(NebulaColor.bgSoft)
            .borderRadius(999)
            .onClick((): void => {
              this.sheetOpen = false;
              this.selectedId = '';
              this.bridge.selectPart(null);
            })
          }
          .width('100%')

          Text(this.selectedPart()!.display)
            .fontSize(NebulaFont.titleLg)
            .fontWeight(FontWeight.Bold)
            .fontColor(NebulaColor.textPrimary)

          Text(this.selectedPart()!.fullDesc)
            .fontSize(NebulaFont.body)
            .fontColor(NebulaColor.textSecondary)
            .lineHeight(22)

          Row({ space: 8 }) {
            ForEach(this.selectedPart()!.specs, (s: SpecLine): void => {
              Column({ space: 2 }) {
                Text(s.key).fontSize(NebulaFont.caption).fontColor(NebulaColor.textTertiary)
                Text(s.value).fontSize(NebulaFont.small).fontColor(NebulaColor.textPrimary).fontWeight(FontWeight.Medium)
              }
              .alignItems(HorizontalAlign.Start)
              .padding({ top: 8, bottom: 8, left: 10, right: 10 })
              .backgroundColor(NebulaColor.bgSoft)
              .borderRadius(10)
            }, (s: SpecLine): string => s.key)
          }
          .width('100%')

          Row({ space: 10 }) {
            Row({ space: 6 }) {
              NebulaIcon({ name: IconName.Sparkle, glyphSize: 14, color: '#FFFFFF', strokeW: 2 })
              Text('小艺讲解').fontSize(NebulaFont.small).fontColor('#FFFFFF').fontWeight(FontWeight.Medium)
            }
            .padding({ top: 9, bottom: 9, left: 14, right: 14 })
            .backgroundColor(NebulaColor.brand)
            .borderRadius(999)

            Row({ space: 6 }) {
              NebulaIcon({ name: IconName.Share, glyphSize: 14, color: NebulaColor.brand, strokeW: 2 })
              Text('生成卡片').fontSize(NebulaFont.small).fontColor(NebulaColor.brand).fontWeight(FontWeight.Medium)
            }
            .padding({ top: 9, bottom: 9, left: 14, right: 14 })
            .backgroundColor(NebulaColor.brandSoft)
            .borderRadius(999)

            Blank()

            Row({ space: 6 }) {
              NebulaIcon({ name: IconName.Close, glyphSize: 14, color: NebulaColor.danger, strokeW: 2 })
              Text('移除').fontSize(NebulaFont.small).fontColor(NebulaColor.danger).fontWeight(FontWeight.Medium)
            }
            .padding({ top: 9, bottom: 9, left: 14, right: 14 })
            .backgroundColor(NebulaColor.accentCoralSoft)
            .borderRadius(999)
            .onClick((): void => this.removePart(this.selectedId))
          }
          .width('100%')
        }
        .alignItems(HorizontalAlign.Start)
        .padding({ left: 20, right: 20, bottom: 100 })
      }
      .width('100%')
      .backgroundColor('#0A0A0CF4')
      .border({ width: { top: 1 }, color: '#27272AE6' })
      .borderRadius({ topLeft: 28, topRight: 28 })
      .backdropBlur(40)
      .shadow({ radius: 36, color: '#00000080', offsetX: 0, offsetY: -10 })
    }
  }

  @Builder drawerHandle() {
    Row({ space: 8 }) {
      NebulaIcon({
        name: this.drawerOpen ? IconName.Close : IconName.Plus,
        glyphSize: 12, color: NebulaColor.textPrimary, strokeW: 2
      })
      Text(this.drawerOpen
        ? 'CLOSE'
        : (this.parts.length === 0
          ? 'START BUILD'
          : 'ADD · ' + (this.parts.length < 10 ? '0' + this.parts.length : this.parts.length.toString())))
        .fontSize('10vp')
        .fontFamily(NebulaFontFace.mono)
        .fontColor(NebulaColor.textPrimary)
        .letterSpacing(2.0)
    }
    .padding({ top: 10, bottom: 10, left: 12, right: 14 })
    .backgroundColor('#0B0B0EE6')
    .border({ width: 1, color: NebulaColor.border })
    .borderRadius(NebulaRadius.tight)
    .backdropBlur(20)
    .onClick((): void => { this.drawerOpen = !this.drawerOpen; })
  }

  build() {
    Stack({ alignContent: Alignment.Top }) {
      // Layer 1: camera passthrough (or nothing at all in workshop mode).
      // When `envOn` is true the WebView paints its own opaque maker-lab
      // backdrop, so there's no reason to keep the native camera preview
      // running behind it — that just wastes power on a frame nobody
      // sees. Skip both the hand-tracking WebView and CameraPreview in
      // that case. Hand-gesture mode still takes priority over the
      // passthrough when env is off — MediaPipe needs getUserMedia.
      if (this.envOn && !this.handOn) {
        // Solid dark fill so that if the WebView loads slowly there's
        // no flash of phone UI beneath the AR canvas.
        Column()
          .width('100%').height('100%')
          .backgroundColor(NebulaColor.bg)
      } else if (this.handOn) {
        Web({
          src: $rawfile('hand.html'),
          controller: this.handWebController
        })
          .width('100%').height('100%')
          .backgroundColor('#0A0A0C')
          .javaScriptAccess(true)
          .domStorageAccess(true)
          .fileAccess(true)
          .imageAccess(true)
          .onlineImageAccess(true)
          .mediaPlayGestureAccess(false)
          .mixedMode(MixedMode.All)
          .horizontalScrollBarAccess(false)
          .verticalScrollBarAccess(false)
          .overScrollMode(OverScrollMode.NEVER)
          .zoomAccess(false)
          .javaScriptProxy({
            object: this.handBridge,
            name: 'handBridge',
            methodList: ['onHands', 'onHandGrab'],
            controller: this.handWebController
          })
          .onControllerAttached((): void => {
            // Belt-and-braces: the attribute-form javaScriptProxy sometimes
            // fails to register on remount, so we explicitly bind again
            // once the controller is live and refresh() to take effect
            // before onResults starts firing.
            try {
              this.handWebController.registerJavaScriptProxy(
                this.handBridge, 'handBridge', ['onHands', 'onHandGrab']
              );
              this.handWebController.refresh();
              hilog.info(0x1, 'WorkshopHandWeb', 'proxy re-registered + refreshed');
            } catch (err) {
              hilog.warn(0x1, 'WorkshopHandWeb',
                'proxy re-register failed: %{public}s', JSON.stringify(err));
            }
          })
          .onPermissionRequest((e): void => {
            // MediaPipe needs VIDEO_CAPTURE for getUserMedia. Grant
            // exactly what it asks for — anything else is rejected.
            try {
              const req = e.request;
              if (req !== undefined && req !== null) {
                const needs: string[] = req.getAccessibleResource();
                req.grant(needs);
              }
            } catch (err) { /* ignore */ }
          })
          .onConsole((e): boolean => {
            hilog.info(0x1, 'WorkshopHandWeb',
              'console %{public}s', e.message.getMessage());
            return false;
          })
          .onErrorReceive((e): void => {
            hilog.warn(0x1, 'WorkshopHandWeb',
              'error %{public}s', e.error.getErrorInfo());
          })
      } else {
        CameraPreview({ onReady: (ok: boolean): void => { this.cameraOk = ok; } })
      }

      // Layer 2: transparent AR scene
      ArViewer({
        bridge: this.bridge,
        onReady: (e: ArReadyEvent): void => this.onReady(e),
        onSelect: (e: ArSelectEvent): void => this.onSelect(e),
        onAnchor: (e: ArAnchorEvent): void => this.onAnchor(e),
        onSplat: (e: ArSplatEvent): void => this.onSplat(e)
      })

      // Layer 3: chrome. System bars are hidden in this page, but in
      // portrait the physical notch / camera cutout still sits at the
      // top, so we reserve ~52vp. In landscape the cutout is on the
      // side and we can tuck the pills much closer to the edge.
      Column() {
        this.topBar()
        Column().height(10)
        Row() { this.modeSwitcher() }.justifyContent(FlexAlign.Center).width('100%')
        Blank()
      }
      .width('100%').height('100%')
      .padding({
        top: this.landscape ? 18 : 52,
        left: NebulaSpace.md,
        right: NebulaSpace.md
      })
      .hitTestBehavior(HitTestMode.Transparent)

      // Reset + Share — top-right corner. In landscape we sit closer
      // to the right edge (24vp) since the Dock / punch-hole concern
      // doesn't apply in that orientation and the user wanted them
      // pushed out to the edge.
      if (!this.drawerOpen) {
        Column() {
          this.actionRail()
        }
        .position({
          right: this.landscape ? 24 : NebulaSpace.md,
          top: this.landscape ? 70 : 110
        })
        .hitTestBehavior(HitTestMode.Transparent)
      }

      // Left navigation joystick — XZ translation in the camera's local
      // frame: push up to walk forward, sideways to strafe. Reads as
      // "前后左右" to the user. Hidden in motion mode (sensors drive
      // camera) and while the parts drawer is open so the joystick
      // can't intercept drawer hits.
      if (this.controlMode === 'touch' && !this.drawerOpen) {
        Column() {
          JoystickPad({
            padSize: 96,
            onChange: (dx: number, dy: number): void => this.onWalkStick(dx, dy)
          })
        }
        .position({
          left: this.landscape ? 64 : NebulaSpace.md,
          bottom: this.landscape ? 24 : 160
        })
        .hitTestBehavior(HitTestMode.Transparent)
      }

      // Right navigation joystick — yaw/pitch look. Camera stays at its
      // current position; only its viewing direction changes. Reads as
      // "看上下左右前后" to the user.
      if (this.controlMode === 'touch' && !this.drawerOpen) {
        Column() {
          JoystickPad({
            padSize: 96,
            onChange: (dx: number, dy: number): void => this.onLookStick(dx, dy)
          })
        }
        .position({
          right: this.landscape ? 64 : NebulaSpace.md,
          bottom: this.landscape ? 24 : 160
        })
        .hitTestBehavior(HitTestMode.Transparent)
      }

      // Up/down — horizontal "↑ ↓" row tucked just inboard of the right
      // joystick. Anchored with right/bottom so it sits off-center: the
      // Index.ets Dock takes the bottom-center strip, and a centered
      // elev row at the same vertical level as the joysticks gets
      // buried by the Dock. Offsetting to the right side keeps the
      // elev at the joystick's vertical center (portrait) or just above
      // it (landscape) while staying clear of the Dock horizontally.
      if (this.controlMode === 'touch' && !this.drawerOpen) {
        Row({ space: 14 }) {
          // Up arrow (icon rotated -90°) moves the camera's look-up,
          // which means translating the orbit target DOWN (target
          // dropping = view shifts up). Down arrow mirrors. Previous
          // mapping had this inverted — pressing up dropped the view.
          this.elevButton(IconName.Arrow, -90, (): void => this.bridge.walkClimb(-1))
          this.elevButton(IconName.Arrow,  90, (): void => this.bridge.walkClimb(1))
        }
        .position({
          right: this.landscape ? 170 : 122,
          bottom: this.landscape ? 48 : 188
        })
        .hitTestBehavior(HitTestMode.Transparent)
      }

      // Drawer handle — centered above the dock in portrait, pulled
      // all the way down in landscape (where the Dock is hidden and
      // the joysticks sit at the bottom corners, leaving a clear
      // bottom-center strip for the + ADD · NN chip).
      Column() {
        this.drawerHandle()
      }
      .width('100%')
      .alignItems(HorizontalAlign.Center)
      .position({ bottom: this.landscape ? 30 : 120 })

      // Splat loading overlay — centered card shown while the 120 MB
      // ISS scene is parsing on first 空间站 toggle. Cleared by
      // onSplat({ status: 'ready' | 'failed' }) from the WebView side.
      if (this.splatLoading) {
        Column() {
          Column({ space: 10 }) {
            Row({ space: 8 }) {
              LoadingProgress()
                .width(20).height(20)
                .color(NebulaColor.brand)
              Text('空间站加载中…')
                .fontSize(NebulaFont.body)
                .fontColor(NebulaColor.textPrimary)
                .fontWeight(FontWeight.Medium)
            }
            Text('资源较大（约 120 MB），请稍候')
              .fontSize(NebulaFont.caption)
              .fontColor(NebulaColor.textTertiary)
          }
          .padding({ left: 22, right: 22, top: 18, bottom: 18 })
          .backgroundColor('#18181BEE')
          .border({ width: 1, color: '#27272AEE' })
          .borderRadius(18)
          .backdropBlur(24)
          .shadow({ radius: 24, color: '#00000099', offsetX: 0, offsetY: 10 })
          .alignItems(HorizontalAlign.Center)
        }
        .width('100%').height('100%')
        .justifyContent(FlexAlign.Center)
        .alignItems(HorizontalAlign.Center)
        .hitTestBehavior(HitTestMode.Transparent)
      }

      // Gesture status chip — when any gesture pipeline is armed, shows
      // the latest event label and a live state dot (motion: flick
      // magnitude; hand: stage + count of hands visible).
      if (this.motionOn || this.handOn) {
        Row({ space: 8 }) {
          Column()
            .width(6).height(6).borderRadius(3)
            .backgroundColor(this.handOn
              ? (this.handCount >= 1 ? NebulaColor.accentMint : NebulaColor.brand)
              : (this.gestureMag > 3.0 ? NebulaColor.accentMint : NebulaColor.brand))
          Text(this.lastGestureLabel.length > 0
            ? this.lastGestureLabel
            : (this.handOn
                ? (this.handCount === 0 ? '手势监听中 · 举一只手到摄像头前'
                   : this.handCount === 1 ? '单手识别 · 拇指食指张合=展开 · 握拳=拖动'
                   : '双手识别 · 双手张合=展开 · 合拢=复位')
                : '体感监听中 · 挥动强度 ' + this.gestureMag.toFixed(1)))
            .fontSize('11vp')
            .fontFamily(NebulaFontFace.mono)
            .fontColor(NebulaColor.textPrimary)
            .letterSpacing(0.6)
        }
        .padding({ left: 10, right: 12, top: 7, bottom: 7 })
        .backgroundColor('#0B0B0EE6')
        .border({ width: 1, color: NebulaColor.border })
        .borderRadius(NebulaRadius.tight)
        .backdropBlur(20)
        .position({ bottom: 152 })
      }

      // Sync role pill — shows who's driving the session across devices.
      if (this.syncRole !== 'solo') {
        Row({ space: 8 }) {
          Column()
            .width(6).height(6).borderRadius(3)
            .backgroundColor(this.syncRole === 'controller' ? NebulaColor.accentMint : NebulaColor.brand)
          Text(this.syncRole === 'controller'
            ? 'MAIN SYNC · ' + this.peerCount + ' UNITS'
            : 'FOLLOWING MAIN')
            .fontSize('10vp')
            .fontFamily(NebulaFontFace.mono)
            .fontColor(NebulaColor.textPrimary)
            .letterSpacing(1.8)
        }
        .padding({ left: 10, right: 12, top: 7, bottom: 7 })
        .backgroundColor('#0B0B0EE6')
        .border({ width: 1, color: NebulaColor.border })
        .borderRadius(NebulaRadius.tight)
        .backdropBlur(20)
        .position({ bottom: 100 })
      }

      // Pull-up drawer / part sheet
      if (this.sheetOpen && this.selectedPart() !== null) {
        Column() {
          this.partSheet()
        }
        .width('100%')
        .position({ bottom: 0 })
      } else if (this.drawerOpen) {
        Column() {
          this.drawer()
        }
        .width('100%')
        .position({ bottom: 0 })
      }

      // Gesture burst — sits above most chrome so flicks feel emphatic.
      // (The cross-device sheet is hoisted to Index.ets so it renders
      // above the bottom Dock.)
      GestureBurst({ trigger: this.gestureBurstTrigger })
    }
    .width('100%')
    .height('100%')
    // Mission-control dark fallback — visible for the brief moment
    // between Hand WebView unmounting and CameraPreview re-acquiring
    // the back camera. Never white-flash.
    .linearGradient({
      angle: 160,
      colors: [
        ['#0A0A0C', 0],
        ['#121214', 0.5],
        ['#09090B', 1]
      ]
    })
  }
}
