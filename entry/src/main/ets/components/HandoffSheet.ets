/**
 * HandoffSheet — bottom sheet for cross-device continuation + collaboration.
 *
 * Workshop / Academy open this by flipping `open` to true. Inside:
 *   • Lists nearby trusted devices from Handoff.listPeers().
 *   • Tapping a device hands off the current HandoffPayload and dismisses.
 *   • A role toggle lets the user pin this device as controller / observer
 *     for the NebulaSync DDO session (persistent, survives the sheet).
 *
 * Purely a visual + event component — state mutations live in services.
 */

import { NebulaColor, NebulaFont, NebulaRadius, NebulaShadow, NebulaSpace } from '../theme/Theme';
import { NebulaIcon, IconName } from './NebulaIcon';
import common from '@ohos.app.ability.common';
import { Handoff, HandoffDevice, HandoffPayload } from '../services/Handoff';
import { NebulaSnapshot, NebulaSync, SyncDiagnostics } from '../services/NebulaSync';
import { AppBus } from '../services/AppBus';

@Component
export struct HandoffSheet {
  @Prop open: boolean = false;
  @Prop payload: HandoffPayload = new HandoffPayload();
  onDismiss: () => void = (): void => {};
  onHandedOff: (device: HandoffDevice) => void = (): void => {};

  @State private devices: HandoffDevice[] = [];
  @State private loading: boolean = false;
  @State private role: string = 'solo';
  @State private permissionDenied: boolean = false;
  @State private peerCount: number = 0;
  @State private diag: SyncDiagnostics = new SyncDiagnostics();
  private unsubSync: () => void = (): void => {};
  private diagTimer: number = -1;

  aboutToAppear(): void {
    this.unsubSync = NebulaSync.instance.subscribe((s: NebulaSnapshot): void => {
      this.role = s.role;
      this.peerCount = s.peerCount;
      this.diag = NebulaSync.instance.getDiagnostics();
    });
    this.refresh();
    // Refresh diag every 500ms while the sheet is open so the user can
    // watch lastPushAt / lastRecvAt tick as events happen.
    this.diagTimer = setInterval((): void => {
      this.diag = NebulaSync.instance.getDiagnostics();
    }, 500);
  }
  aboutToDisappear(): void {
    this.unsubSync();
    if (this.diagTimer >= 0) { clearInterval(this.diagTimer); this.diagTimer = -1; }
  }

  private hostCtx(): common.UIAbilityContext | null {
    try {
      const host = this.getUIContext().getHostContext();
      return host !== undefined && host !== null ? host as common.UIAbilityContext : null;
    } catch (e) { return null; }
  }

  private async refresh(): Promise<void> {
    this.loading = true;
    // Request the user-grant permission first — without DISTRIBUTED_DATASYNC
    // the DM list is empty AND the DDO session can't join, so we do both
    // here the moment the user opens the sheet.
    const ctx = this.hostCtx();
    if (ctx !== null) {
      const granted = await Handoff.ensurePermission(ctx);
      this.permissionDenied = !granted;
      if (granted) {
        // Late-join the distributed session now that we have permission.
        await NebulaSync.instance.bindSession(ctx);
      }
    }
    this.devices = Handoff.listPeers();
    this.loading = false;
  }

  private async handTo(device: HandoffDevice): Promise<void> {
    let ctx: common.UIAbilityContext | null = null;
    try {
      const host = this.getUIContext().getHostContext();
      ctx = host !== undefined && host !== null ? host as common.UIAbilityContext : null;
    } catch (e) { ctx = null; }
    if (ctx === null) return;
    // Dismiss the sheet first so the cutscene owns the whole screen,
    // then fire the cinematic and kick off the actual Want.
    this.onDismiss();
    AppBus.instance.startCinematic('outgoing', device.name, this.payload.kind);
    const ok = await Handoff.startContinuation(ctx, this.payload, device);
    if (ok) {
      this.onHandedOff(device);
    } else {
      // If startAbility fails, short-circuit the cutscene so the user
      // isn't staring at a half-played animation.
      AppBus.instance.endCinematic();
    }
  }

  private setRole(next: string): void {
    NebulaSync.instance.setRole(next);
    // When the user opts into shared mode, make sure the DDO session
    // is actually joined (with permission granted). bindSession is safe
    // to call repeatedly — it's a no-op if we're already bound.
    if (next === 'controller' || next === 'observer') {
      const ctx = this.hostCtx();
      if (ctx !== null) {
        Handoff.ensurePermission(ctx).then((granted: boolean): void => {
          if (granted) NebulaSync.instance.bindSession(ctx);
        });
      }
    }
  }

  private iconForType(type: string): string {
    // HarmonyOS device type codes: 4 = tablet/pad, 6 = watch, else treat as phone.
    const t = parseInt(type, 10);
    if (t === 4 || type.toLowerCase().indexOf('pad') >= 0) return IconName.Tablet;
    if (t === 6 || type.toLowerCase().indexOf('watch') >= 0) return IconName.Watch;
    return IconName.Phone;
  }

  @Builder deviceRow(d: HandoffDevice, idx: number) {
    Row({ space: 12 }) {
      Column() {
        NebulaIcon({
          name: this.iconForType(d.type), glyphSize: 20,
          color: NebulaColor.brand, strokeW: 2
        })
      }
      .width(44).height(44)
      .backgroundColor(NebulaColor.brandSoft)
      .border({ width: 1, color: NebulaColor.brand + '33' })
      .borderRadius(12)
      .justifyContent(FlexAlign.Center)
      .alignItems(HorizontalAlign.Center)

      Column({ space: 3 }) {
        Text(d.name.length > 0 ? d.name : '附近设备 ' + (idx + 1))
          .fontSize(NebulaFont.body)
          .fontColor(NebulaColor.textPrimary)
          .fontWeight(FontWeight.Bold)
        Text(d.networkId.length > 10 ? d.networkId.substring(0, 10) + '…' : d.networkId)
          .fontSize(NebulaFont.caption)
          .fontColor(NebulaColor.textTertiary)
      }
      .alignItems(HorizontalAlign.Start)
      .layoutWeight(1)

      Row({ space: 6 }) {
        NebulaIcon({ name: IconName.Transfer, glyphSize: 13, color: '#FFFFFF', strokeW: 2 })
        Text('流转')
          .fontSize(NebulaFont.small)
          .fontColor('#FFFFFF')
          .fontWeight(FontWeight.Medium)
      }
      .padding({ left: 12, right: 14, top: 8, bottom: 8 })
      .backgroundColor(NebulaColor.brand)
      .borderRadius(999)
      .shadow({ radius: 14, color: '#FF4D334D', offsetX: 0, offsetY: 4 })
      .onClick((): void => { this.handTo(d); })
    }
    .width('100%')
    .padding({ left: 14, right: 14, top: 12, bottom: 12 })
    .backgroundColor(NebulaColor.surface)
    .border({ width: 1, color: NebulaColor.border })
    .borderRadius(NebulaRadius.md)
  }

  @Builder diagRow(label: string, ok: boolean) {
    Row({ space: 6 }) {
      Column()
        .width(8).height(8).borderRadius(4)
        .backgroundColor(ok ? NebulaColor.accentMint : NebulaColor.border)
      Text(label)
        .fontSize(NebulaFont.caption)
        .fontColor(NebulaColor.textSecondary)
      Blank()
      Text(ok ? '✓' : '…')
        .fontSize(NebulaFont.caption)
        .fontColor(ok ? NebulaColor.accentMint : NebulaColor.textTertiary)
        .fontWeight(FontWeight.Bold)
    }
    .width('100%')
  }

  @Builder diagFooter() {
    Column({ space: 3 }) {
      if (this.diag.lastPushAt > 0) {
        Text('上次发送: ' + this.formatRelAge(this.diag.lastPushAt) + '前')
          .fontSize(10).fontColor(NebulaColor.textTertiary)
      }
      if (this.diag.lastRecvAt > 0) {
        Text('上次接收: ' + this.formatRelAge(this.diag.lastRecvAt) + '前')
          .fontSize(10).fontColor(NebulaColor.textTertiary)
      }
      if (this.diag.lastError.length > 0) {
        Text('错误: ' + this.diag.lastError)
          .fontSize(10).fontColor(NebulaColor.accentCoral)
          .maxLines(2)
          .textOverflow({ overflow: TextOverflow.Ellipsis })
      }
      if (this.peerCount === 0 && this.diag.sessionJoined) {
        Text('已入会但无对端 — 确认另一台设备也打开此应用 + 跨端抽屉')
          .fontSize(10).fontColor(NebulaColor.accentGold)
          .lineHeight(14)
      }
    }
    .alignItems(HorizontalAlign.Start)
    .width('100%')
    .padding({ top: 6 })
  }

  private formatRelAge(t: number): string {
    if (t === 0) return '-';
    const secs = Math.max(0, Math.floor((Date.now() - t) / 1000));
    if (secs < 60) return secs + 's';
    if (secs < 3600) return Math.floor(secs / 60) + 'm';
    return Math.floor(secs / 3600) + 'h';
  }

  @Builder roleButton(label: string, value: string, desc: string) {
    Column({ space: 4 }) {
      Text(label)
        .fontSize(NebulaFont.small)
        .fontWeight(FontWeight.Bold)
        .fontColor(this.role === value ? '#FFFFFF' : NebulaColor.textPrimary)
      Text(desc)
        .fontSize(NebulaFont.caption)
        .fontColor(this.role === value ? '#FFFFFFD0' : NebulaColor.textTertiary)
        .lineHeight(14)
    }
    .alignItems(HorizontalAlign.Start)
    .layoutWeight(1)
    .padding({ top: 10, bottom: 10, left: 12, right: 12 })
    .backgroundColor(this.role === value ? NebulaColor.brand : NebulaColor.surface)
    .border({ width: 1, color: this.role === value ? NebulaColor.brand : NebulaColor.border })
    .borderRadius(NebulaRadius.md)
    .onClick((): void => { this.setRole(value); })
  }

  build() {
    if (this.open) {
      Stack() {
        // Scrim — dark translucent so the sheet pops, not the old indigo-tint
        Column()
          .width('100%').height('100%')
          .backgroundColor('#000000B3')
          .onClick((): void => { this.onDismiss(); })

        // Sheet
        Column({ space: 16 }) {
          // Drag handle — brighter so it's visible on the dark surface
          Column()
            .width(44).height(5)
            .backgroundColor(NebulaColor.textSecondary)
            .borderRadius(3)
            .margin({ top: 12, bottom: 4 })

          // Header
          Row() {
            Column({ space: 3 }) {
              Text('跨端流转')
                .fontSize(NebulaFont.titleLg)
                .fontWeight(FontWeight.Bold)
                .fontColor(NebulaColor.textPrimary)
              Text('选择附近的鸿蒙设备继续操作')
                .fontSize(NebulaFont.small)
                .fontColor(NebulaColor.textSecondary)
            }
            .alignItems(HorizontalAlign.Start)
            Blank()
            Row() {
              NebulaIcon({ name: IconName.Close, glyphSize: 14, color: NebulaColor.textPrimary, strokeW: 2.2 })
            }
            .padding(9)
            .backgroundColor(NebulaColor.bgSoft)
            .border({ width: 1, color: NebulaColor.border })
            .borderRadius(999)
            .onClick((): void => { this.onDismiss(); })
          }
          .width('100%')

          // Role selector
          Column({ space: 8 }) {
            Row({ space: 6 }) {
              Text('协同模式').fontSize(NebulaFont.caption).fontColor(NebulaColor.textTertiary)
              Blank()
              Row({ space: 4 }) {
                Column()
                  .width(8).height(8).borderRadius(4)
                  .backgroundColor(this.peerCount > 0 ? NebulaColor.accentMint : NebulaColor.border)
                Text(this.peerCount > 0 ? '已连接 ' + this.peerCount + ' 台对端' : '尚未连接对端')
                  .fontSize(NebulaFont.caption)
                  .fontColor(this.peerCount > 0 ? NebulaColor.textPrimary : NebulaColor.textTertiary)
                  .fontWeight(FontWeight.Medium)
              }
            }
            .width('100%')
            Row({ space: 8 }) {
              this.roleButton('单机', 'solo', '关闭同步')
              this.roleButton('主控', 'controller', '驱动他人视角')
              this.roleButton('观察', 'observer', '跟随主控画面')
            }
            .width('100%')

            // Diagnostic strip — helps the user see at a glance which step
            // of the bring-up is failing without resorting to hdc.
            Column({ space: 4 }) {
              this.diagRow('KV 管理器', this.diag.ddoCreated)
              this.diagRow('KV 打开', this.diag.sessionJoined)
              this.diagRow('UDP 已绑定', this.diag.udpBound)
              this.diagRow('对端发现', this.peerCount > 0)
              Row({ space: 6 }) {
                Text('UDP 发/收').fontSize(NebulaFont.caption).fontColor(NebulaColor.textSecondary)
                Blank()
                Text(this.diag.udpSent + ' / ' + this.diag.udpReceived)
                  .fontSize(NebulaFont.caption)
                  .fontColor(this.diag.udpReceived > 0 ? NebulaColor.accentMint : NebulaColor.textTertiary)
                  .fontWeight(FontWeight.Bold)
              }
              .width('100%')
              this.diagFooter()
            }
            .width('100%')
            .padding({ top: 12, bottom: 12, left: 14, right: 14 })
            .backgroundColor(NebulaColor.bgSoft)
            .border({ width: 1, color: NebulaColor.border })
            .borderRadius(NebulaRadius.md)
            .alignItems(HorizontalAlign.Start)
          }
          .alignItems(HorizontalAlign.Start)
          .width('100%')

          // Device list
          Column({ space: 8 }) {
            Row({ space: 6 }) {
              Text('附近设备').fontSize(NebulaFont.caption).fontColor(NebulaColor.textTertiary)
              Blank()
              Row({ space: 4 }) {
                NebulaIcon({ name: IconName.Reset, glyphSize: 11, color: NebulaColor.brand, strokeW: 2 })
                Text('刷新').fontSize(NebulaFont.caption).fontColor(NebulaColor.brand)
              }
              .padding({ left: 8, right: 10, top: 4, bottom: 4 })
              .onClick((): void => { this.refresh(); })
            }
            .width('100%')

            if (this.loading) {
              Row({ space: 8 }) {
                LoadingProgress().color(NebulaColor.brand).width(18).height(18)
                Text('正在扫描…').fontSize(NebulaFont.small).fontColor(NebulaColor.textSecondary)
              }
              .padding({ top: 14, bottom: 14 })
            } else if (this.devices.length === 0) {
              Column({ space: 8 }) {
                Text(this.permissionDenied ? '权限未授予' : '没有发现其他设备')
                  .fontSize(NebulaFont.small)
                  .fontColor(NebulaColor.textSecondary)
                  .fontWeight(FontWeight.Medium)
                Text(this.permissionDenied
                  ? '多设备协同需要「分布式数据同步」权限，请到设置中授予后再刷新'
                  : '请确认：\n· 两台设备登录同一 华为账号\n· 两台设备都打开蓝牙 + WLAN + 多设备协同\n· 都安装并打开了本应用')
                  .fontSize(NebulaFont.caption)
                  .fontColor(NebulaColor.textTertiary)
                  .lineHeight(18)
                  .textAlign(TextAlign.Start)
              }
              .alignItems(HorizontalAlign.Start)
              .width('100%')
              .padding({ top: 12, bottom: 12, left: 6, right: 6 })
            } else {
              Column({ space: 8 }) {
                ForEach(this.devices, (d: HandoffDevice, i: number): void => {
                  this.deviceRow(d, i)
                }, (d: HandoffDevice): string => d.networkId)
              }
              .width('100%')
            }
          }
          .alignItems(HorizontalAlign.Start)
          .width('100%')
        }
        .width('100%')
        // Extra bottom inset so content clears the safe-area gesture bar
        // (the sheet itself already sits above the dock now that it's
        // hoisted into Index, but we still want the last device row to
        // be visible above the home-indicator).
        .padding({ left: NebulaSpace.md, right: NebulaSpace.md, top: 0, bottom: 44 })
        // Dark frosted panel matching the rest of the app's zinc-900
        // console look. White background was left over from the weixin
        // light-theme port and made the title / labels nearly invisible.
        .backgroundColor('#0A0A0CF4')
        .border({ width: { top: 1 }, color: '#27272AE6' })
        .borderRadius({ topLeft: 28, topRight: 28 })
        .backdropBlur(40)
        .shadow({ radius: 36, color: '#000000A6', offsetX: 0, offsetY: -10 })
        .alignItems(HorizontalAlign.Center)
        .offset({ y: 0 })
        .position({ bottom: 0 })
      }
      .width('100%').height('100%')
      .alignContent(Alignment.Bottom)
    }
  }
}
