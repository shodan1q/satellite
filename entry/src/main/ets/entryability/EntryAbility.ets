import { AbilityConstant, ConfigurationConstant, UIAbility, Want } from '@kit.AbilityKit';
import { hilog } from '@kit.PerformanceAnalysisKit';
import { window } from '@kit.ArkUI';
import { NebulaSync } from '../services/NebulaSync';
import { PhoneInfo } from '../services/PhoneInfo';
import { UserStats } from '../services/UserStats';
import { ModelStore } from '../services/ModelStore';
import { Handoff, HandoffPayload } from '../services/Handoff';
import { AppBus } from '../services/AppBus';

const DOMAIN = 0x0000;

/**
 * EntryAbility is also the landing point for cross-device continuation:
 *   - onContinue  (sender side) — return AGREE so the system migrates us.
 *   - onNewWant   (receiver side, app already running) — restore state.
 *   - onCreate    (receiver side, cold start) — honours launchReason === CONTINUATION.
 *
 * In all cases the deserialised HandoffPayload is broadcast via AppBus so
 * Index / Workshop / Academy can navigate to the matching tab and rehydrate.
 */
export default class EntryAbility extends UIAbility {
  private initialPage: string = 'pages/Splash';

  onCreate(want: Want, launchParam: AbilityConstant.LaunchParam): void {
    try {
      // Mission-control console aesthetic is dark-only.
      this.context.getApplicationContext().setColorMode(ConfigurationConstant.ColorMode.COLOR_MODE_DARK);
    } catch (err) {
      hilog.error(DOMAIN, 'NebulaCore', 'color mode failed: %{public}s', JSON.stringify(err));
    }
    NebulaSync.instance.init(this.context).then((): void => {
      // If the user granted DISTRIBUTED_DATASYNC in a previous session,
      // auto-open the KVStore so multi-device sync works without them
      // having to open the handoff drawer first. First-time users still
      // need to open the drawer to trigger the permission prompt.
      Handoff.isPermissionGranted().then((granted: boolean): void => {
        if (granted) {
          NebulaSync.instance.bindSession(this.context).catch((err: Error): void => {
            hilog.warn(DOMAIN, 'NebulaCore', 'auto bindSession: %{public}s', err.message);
          });
        }
      });
    }).catch((err: Error): void => {
      hilog.warn(DOMAIN, 'NebulaCore', 'NebulaSync init: %{public}s', err.message);
    });
    UserStats.instance.init(this.context).catch((err: Error): void => {
      hilog.warn(DOMAIN, 'NebulaCore', 'UserStats init: %{public}s', err.message);
    });
    ModelStore.instance.init(this.context);
    PhoneInfo.instance.start();

    // Distributed startAbility delivers the want here regardless of
    // launchReason — if our payload is present, restore unconditionally.
    this.applyHandoff(want);
  }

  onNewWant(want: Want, _launchParam: AbilityConstant.LaunchParam): void {
    this.applyHandoff(want);
  }

  /**
   * Called on the sending device when the user taps "流转到其他设备".
   * We snapshot the current user-facing state into the wantParam bag; the
   * system hands that bag to the peer's onNewWant via the same key.
   */
  onContinue(wantParam: Record<string, Object>): AbilityConstant.OnContinueResult {
    try {
      const snap = NebulaSync.instance.state;
      const payload = new HandoffPayload();
      // Controller defaults to workshop — Workshop page patches this via
      // NebulaSync updates so we just read what's there. For academy entries
      // the snapshot also stores currentSatellite.
      payload.kind = 'workshop';
      payload.rocketId = snap.currentRocket;
      payload.satelliteId = snap.currentSatellite;
      payload.explode = 0;
      payload.selectedPart = '';
      payload.controlMode = 'touch';
      payload.issuedAt = Date.now();
      wantParam['nebula.handoff.v1'] = JSON.stringify(payload);
    } catch (err) {
      hilog.warn(DOMAIN, 'NebulaCore', 'onContinue serialise failed');
    }
    return AbilityConstant.OnContinueResult.AGREE;
  }

  private applyHandoff(want: Want): void {
    const payload = HandoffPayload.fromWant(want);
    if (payload === null) {
      hilog.info(DOMAIN, 'NebulaCore', 'applyHandoff: no payload');
      return;
    }
    // Continuation / deep-link arrivals skip the splash so Index can
    // consume the AppBus events immediately.
    this.initialPage = 'pages/Index';
    hilog.info(DOMAIN, 'NebulaCore',
      'applyHandoff tag=%{public}s kind=%{public}s rocket=%{public}s',
      payload.syncTag, payload.kind, payload.rocketId);
    if (payload.syncTag === 'sync') {
      // Silent live-sync tick — mirror state directly, no cinematic
      // or tab navigation. The Workshop subscriber picks up the snapshot
      // update and updates the 3D scene.
      NebulaSync.instance.applyRemotePayload(payload);
      return;
    }
    // One-shot handoff: play the cutscene, then deliver to AppBus so
    // Index.ets can navigate + auto-bind the session.
    AppBus.instance.startCinematic('incoming', '', payload.kind);
    AppBus.instance.deliverHandoff(payload);
  }

  onDestroy(): void {
    PhoneInfo.instance.stop();
  }

  onWindowStageCreate(windowStage: window.WindowStage): void {
    windowStage.loadContent(this.initialPage, (err) => {
      if (err.code) {
        hilog.error(DOMAIN, 'NebulaCore', 'loadContent failed: %{public}s', JSON.stringify(err));
        return;
      }
      try {
        const win = windowStage.getMainWindowSync();
        // Fill the whole screen — content extends under the status / nav bars.
        win.setWindowLayoutFullScreen(true);
        // Mission-control dark — status/nav bar content stays white
        // throughout the app (splash → index → every page).
        win.setWindowSystemBarProperties({
          statusBarContentColor: '#FFFFFF',
          navigationBarContentColor: '#FFFFFF'
        });
      } catch (e) {
        hilog.warn(DOMAIN, 'NebulaCore', 'fullscreen setup failed');
      }
    });
  }

  onWindowStageDestroy(): void {}
  onForeground(): void {}
  onBackground(): void {}
}
