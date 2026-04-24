/**
 * HandoffCinematic — full-screen transient overlay for cross-device flow.
 *
 * Covers status + nav bars via expandSafeArea so nothing on the page shows
 * through. Renders one of two mini-cutscenes:
 *   • outgoing — a rocket icon streaks from center toward the top-right,
 *     leaves a glowing tail; text reads "流转到 {device}".
 *   • incoming — a bright ring expands from top-right to center and a
 *     banner fades in at the top: "已从远端接续会话".
 *
 * Driven by AppBus.onCinematic — the sheet fires outgoing, EntryAbility's
 * handoff path fires incoming. Auto-dismisses after ~1.6s.
 */

import { NebulaColor, NebulaFont } from '../theme/Theme';
import { NebulaIcon, IconName } from './NebulaIcon';
import { AppBus, HandoffCinematicEvent } from '../services/AppBus';

@Component
export struct HandoffCinematic {
  @State private active: boolean = false;
  @State private kind: string = 'outgoing';
  @State private deviceName: string = '';
  @State private targetKind: string = 'workshop';

  // Animated values — driven by animateTo blocks so the system handles
  // interpolation / raf for us.
  @State private scrimAlpha: number = 0;
  @State private ringScale: number = 0.4;
  @State private ringAlpha: number = 0;
  @State private rocketOffsetX: number = 0;
  @State private rocketOffsetY: number = 0;
  @State private rocketAlpha: number = 0;
  @State private bannerAlpha: number = 0;
  @State private bannerOffsetY: number = -40;

  private unsub: () => void = (): void => {};
  private dismissTimer: number = -1;

  aboutToAppear(): void {
    this.unsub = AppBus.instance.onCinematic((e: HandoffCinematicEvent): void => {
      if (e.kind === 'end') { this.play(false); return; }
      this.kind = e.kind;
      this.deviceName = e.deviceName.length > 0 ? e.deviceName : '对端设备';
      this.targetKind = e.targetKind;
      this.play(true);
    });
  }
  aboutToDisappear(): void {
    this.unsub();
    if (this.dismissTimer >= 0) { clearTimeout(this.dismissTimer); this.dismissTimer = -1; }
  }

  private reset(): void {
    this.scrimAlpha = 0;
    this.ringScale = 0.4;
    this.ringAlpha = 0;
    this.rocketOffsetX = 0;
    this.rocketOffsetY = 0;
    this.rocketAlpha = 0;
    this.bannerAlpha = 0;
    this.bannerOffsetY = -40;
  }

  private play(start: boolean): void {
    if (this.dismissTimer >= 0) { clearTimeout(this.dismissTimer); this.dismissTimer = -1; }
    if (!start) {
      this.animateOut();
      return;
    }

    this.reset();
    this.active = true;

    // UIContext-scoped animateTo (the global one is deprecated).
    const ui = this.getUIContext();

    // Phase 1 — scrim fade-in.
    ui.animateTo({ duration: 220, curve: Curve.EaseOut }, (): void => {
      this.scrimAlpha = 1;
      this.bannerAlpha = 1;
      this.bannerOffsetY = 0;
    });

    if (this.kind === 'outgoing') {
      // Phase 2a — rocket rises out of the card, then streaks off to top-right.
      ui.animateTo({ duration: 260, curve: Curve.EaseOut, delay: 80 }, (): void => {
        this.rocketAlpha = 1;
      });
      ui.animateTo({ duration: 900, curve: Curve.FastOutSlowIn, delay: 280 }, (): void => {
        this.rocketOffsetX = 220;
        this.rocketOffsetY = -280;
        this.ringScale = 2.2;
        this.ringAlpha = 0.85;
      });
      ui.animateTo({ duration: 420, curve: Curve.EaseIn, delay: 1080 }, (): void => {
        this.rocketAlpha = 0;
        this.ringAlpha = 0;
      });
    } else {
      // Phase 2b — incoming ring pulses twice from the top-right inward.
      ui.animateTo({ duration: 260, curve: Curve.EaseOut, delay: 80 }, (): void => {
        this.rocketAlpha = 1;
        this.rocketOffsetX = 140;
        this.rocketOffsetY = -180;
      });
      ui.animateTo({ duration: 900, curve: Curve.FastOutSlowIn, delay: 260 }, (): void => {
        this.rocketOffsetX = 0;
        this.rocketOffsetY = 0;
        this.ringScale = 2.4;
        this.ringAlpha = 0.75;
      });
      ui.animateTo({ duration: 360, curve: Curve.EaseIn, delay: 1100 }, (): void => {
        this.ringAlpha = 0;
        this.rocketAlpha = 0.6;
      });
    }

    // Auto-dismiss.
    this.dismissTimer = setTimeout((): void => {
      this.animateOut();
    }, 1600);
  }

  private animateOut(): void {
    this.getUIContext().animateTo(
      { duration: 240, curve: Curve.EaseIn, onFinish: (): void => { this.active = false; this.reset(); } },
      (): void => {
        this.scrimAlpha = 0;
        this.bannerAlpha = 0;
        this.bannerOffsetY = -40;
        this.ringAlpha = 0;
        this.rocketAlpha = 0;
      }
    );
  }

  private heading(): string {
    if (this.kind === 'outgoing') return '流转到 ' + this.deviceName;
    return '已从远端接续会话';
  }
  private subtitle(): string {
    if (this.kind === 'outgoing') {
      return this.targetKind === 'academy' ? '科普详情同步中…' : 'AR 工坊状态同步中…';
    }
    return '数据已就位，画面继续';
  }

  build() {
    if (this.active) {
      Stack({ alignContent: Alignment.Center }) {
        // Scrim with subtle gradient — tinted toward brand navy.
        Column()
          .width('100%').height('100%')
          .backgroundColor('#0C1B33CC')
          .backdropBlur(10)
          .opacity(this.scrimAlpha)

        // Expanding glow ring.
        Column()
          .width(240).height(240)
          .borderRadius(999)
          .backgroundColor(this.kind === 'outgoing' ? '#2E6BF6' : NebulaColor.accentMint)
          .opacity(this.ringAlpha * 0.35)
          .scale({ x: this.ringScale, y: this.ringScale })

        // Secondary ring outline for depth.
        Column()
          .width(160).height(160)
          .borderRadius(999)
          .border({ width: 2, color: this.kind === 'outgoing' ? '#89B4FF' : '#7EE4C2' })
          .opacity(this.ringAlpha)
          .scale({ x: this.ringScale * 0.9, y: this.ringScale * 0.9 })

        // Rocket glyph — drifts according to kind.
        Column() {
          NebulaIcon({
            name: IconName.Rocket,
            glyphSize: 56,
            color: '#FFFFFF',
            strokeW: 2.4
          })
        }
        .width(96).height(96)
        .borderRadius(28)
        .backgroundColor('#2E6BF6E6')
        .border({ width: 1, color: '#FFFFFF55' })
        .shadow({ radius: 40, color: '#2E6BF6BB', offsetX: 0, offsetY: 10 })
        .justifyContent(FlexAlign.Center)
        .alignItems(HorizontalAlign.Center)
        .opacity(this.rocketAlpha)
        .translate({ x: this.rocketOffsetX, y: this.rocketOffsetY })

        // Banner at top.
        Column({ space: 4 }) {
          Text(this.heading())
            .fontSize(NebulaFont.titleLg)
            .fontWeight(FontWeight.Bold)
            .fontColor('#FFFFFF')
          Text(this.subtitle())
            .fontSize(NebulaFont.small)
            .fontColor('#27272AEE')
        }
        .alignItems(HorizontalAlign.Center)
        .padding({ left: 28, right: 28, top: 72, bottom: 14 })
        .opacity(this.bannerAlpha)
        .translate({ y: this.bannerOffsetY })
        .position({ top: 0 })
      }
      .width('100%').height('100%')
      .hitTestBehavior(HitTestMode.Transparent)
      .expandSafeArea([SafeAreaType.SYSTEM], [SafeAreaEdge.TOP, SafeAreaEdge.BOTTOM])
    }
  }
}
