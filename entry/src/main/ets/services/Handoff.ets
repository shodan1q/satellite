/**
 * Handoff — single-shot device-to-device continuation.
 *
 * Serialises the full user-facing session (which tab, which rocket or
 * satellite, explode amount, selected part, AR control mode) into a Want
 * parameter bag, then either:
 *   a) hands that bag to UIAbility.onContinue so the HarmonyOS system
 *      can migrate the session to a paired device ("流转"), or
 *   b) reconstructs the state on the receiving side via AppBus events
 *      + NebulaSync updates so the UI jumps to the same place.
 *
 * Use HandoffPayload.fromWant(want) on the receiver; Handoff.prepare(...)
 * on the sender.
 */

import { Want } from '@kit.AbilityKit';
import common from '@ohos.app.ability.common';
import distributedDeviceManager from '@ohos.distributedDeviceManager';
import abilityAccessCtrl, { Permissions, PermissionRequestResult } from '@ohos.abilityAccessCtrl';
import bundleManager from '@ohos.bundle.bundleManager';
import { BusinessError } from '@ohos.base';
import { hilog } from '@kit.PerformanceAnalysisKit';

const TAG = 'Handoff';
const BUNDLE = 'com.yu.satellite';
const MODULE = 'entry';
const ABILITY = 'EntryAbility';
const CONTINUE_KEY = 'nebula.handoff.v1';

export class HandoffDevice {
  networkId: string = '';
  name: string = '';
  type: string = '';
}

export class HandoffPayload {
  kind: string = 'workshop';           // 'workshop' | 'academy' | 'discover'
  rocketId: string = '';                // workshop
  satelliteId: string = '';             // academy (kind='satellite')
  entryKind: string = 'rocket';         // academy: 'rocket' | 'satellite'
  explode: number = 0;
  selectedPart: string = '';
  controlMode: string = 'touch';        // 'touch' | 'motion'
  focusedLaunchId: string = '';         // discover detail
  issuedAt: number = 0;
  // 'handoff' = one-shot continuation (plays cinematic, navigates tab).
  // 'sync'    = live collaborative tick (silent; observer mirrors state).
  syncTag: string = 'handoff';

  static empty(): HandoffPayload { return new HandoffPayload(); }

  static fromWant(want: Want | null | undefined): HandoffPayload | null {
    if (want === null || want === undefined) return null;
    const params = want.parameters as Record<string, Object> | undefined;
    if (params === null || params === undefined) return null;
    const raw = params[CONTINUE_KEY];
    if (raw === null || raw === undefined) return null;
    try {
      const parsed = JSON.parse(String(raw)) as HandoffPayload;
      // JSON.parse returns a plain object; rehydrate with defaults so
      // missing fields from older payload versions don't crash the UI.
      const p = new HandoffPayload();
      p.kind = parsed.kind ?? 'workshop';
      p.rocketId = parsed.rocketId ?? '';
      p.satelliteId = parsed.satelliteId ?? '';
      p.entryKind = parsed.entryKind ?? 'rocket';
      p.explode = parsed.explode ?? 0;
      p.selectedPart = parsed.selectedPart ?? '';
      p.controlMode = parsed.controlMode ?? 'touch';
      p.focusedLaunchId = parsed.focusedLaunchId ?? '';
      p.issuedAt = parsed.issuedAt ?? 0;
      p.syncTag = parsed.syncTag ?? 'handoff';
      return p;
    } catch (e) {
      hilog.warn(0x1, TAG, 'payload parse failed');
      return null;
    }
  }
}

export class Handoff {
  private static permissionAsked: boolean = false;

  /**
   * DISTRIBUTED_DATASYNC is a user_grant permission — without it the
   * DM service returns an empty device list and startAbility(deviceId)
   * throws 201. We request it the first time the user opens the sheet;
   * the system caches the answer so subsequent calls are a no-op.
   */
  /**
   * Silent check — returns true iff DISTRIBUTED_DATASYNC is already
   * granted for this app, WITHOUT prompting. Used at app launch to
   * auto-bind the KVStore when the user has already accepted before.
   */
  static async isPermissionGranted(): Promise<boolean> {
    try {
      const info = await bundleManager.getBundleInfoForSelf(
        bundleManager.BundleFlag.GET_BUNDLE_INFO_WITH_APPLICATION
      );
      const tokenId = info.appInfo.accessTokenId;
      const atm = abilityAccessCtrl.createAtManager();
      const status = await atm.checkAccessToken(tokenId, 'ohos.permission.DISTRIBUTED_DATASYNC');
      return status === abilityAccessCtrl.GrantStatus.PERMISSION_GRANTED;
    } catch (err) {
      return false;
    }
  }

  static async ensurePermission(ctx: common.UIAbilityContext): Promise<boolean> {
    try {
      const atm = abilityAccessCtrl.createAtManager();
      const perms: Permissions[] = ['ohos.permission.DISTRIBUTED_DATASYNC'];
      const res: PermissionRequestResult = await atm.requestPermissionsFromUser(ctx, perms);
      Handoff.permissionAsked = true;
      if (res.authResults.length > 0 && res.authResults[0] === 0) return true;
      hilog.warn(0x1, TAG, 'DISTRIBUTED_DATASYNC denied authResult=%{public}d',
        res.authResults.length > 0 ? res.authResults[0] : -1);
      return false;
    } catch (err) {
      const e = err as BusinessError;
      hilog.warn(0x1, TAG, 'permission request failed code=%{public}d', e.code ?? -1);
      return false;
    }
  }

  /**
   * Build a Want that, when handed to UIAbility.startAbility on a peer,
   * launches this app and restores the given payload via onNewWant /
   * onCreate (with launchReason=CONTINUATION).
   */
  static buildContinueWant(payload: HandoffPayload, deviceId: string): Want {
    const p = new HandoffPayload();
    p.kind = payload.kind;
    p.rocketId = payload.rocketId;
    p.satelliteId = payload.satelliteId;
    p.entryKind = payload.entryKind;
    p.explode = payload.explode;
    p.selectedPart = payload.selectedPart;
    p.controlMode = payload.controlMode;
    p.focusedLaunchId = payload.focusedLaunchId;
    p.issuedAt = Date.now();
    const params: Record<string, Object> = {};
    params[CONTINUE_KEY] = JSON.stringify(p);
    const want: Want = {
      bundleName: BUNDLE,
      moduleName: MODULE,
      abilityName: ABILITY,
      deviceId,
      parameters: params
    };
    return want;
  }

  /** Start the continuation on the selected device. */
  static async startContinuation(
    ctx: common.UIAbilityContext,
    payload: HandoffPayload,
    device: HandoffDevice
  ): Promise<boolean> {
    try {
      const want = Handoff.buildContinueWant(payload, device.networkId);
      hilog.info(0x1, TAG,
        'startAbility → %{public}s kind=%{public}s rocket=%{public}s',
        device.networkId, payload.kind, payload.rocketId);
      await ctx.startAbility(want);
      hilog.info(0x1, TAG, 'startAbility ok');
      return true;
    } catch (err) {
      const e = err as BusinessError;
      hilog.warn(0x1, TAG,
        'startAbility failed code=%{public}d msg=%{public}s',
        e.code ?? -1, e.message ?? '');
      return false;
    }
  }

  /**
   * List other trusted devices nearby. Empty array if the DM service is
   * unavailable (simulator, unpaired phone) — caller should fall back to
   * NebulaSync DDO sharing or a toast hint.
   */
  static listPeers(): HandoffDevice[] {
    const out: HandoffDevice[] = [];
    let dm: distributedDeviceManager.DeviceManager | null = null;
    try {
      dm = distributedDeviceManager.createDeviceManager(BUNDLE);
      const raw = dm.getAvailableDeviceListSync();
      for (const d of raw) {
        const hd = new HandoffDevice();
        hd.networkId = d.networkId ?? '';
        hd.name = d.deviceName ?? '';
        hd.type = (d.deviceType !== undefined && d.deviceType !== null) ? String(d.deviceType) : '';
        if (hd.networkId.length > 0) out.push(hd);
      }
    } catch (err) {
      const e = err as BusinessError;
      hilog.warn(0x1, TAG, 'listPeers failed code=%{public}d', e.code ?? -1);
    } finally {
      if (dm !== null) {
        try { distributedDeviceManager.releaseDeviceManager(dm); } catch (e) { /* ignore */ }
      }
    }
    return out;
  }
}
