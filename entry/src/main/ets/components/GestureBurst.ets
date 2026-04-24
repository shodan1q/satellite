/**
 * GestureBurst — full-screen flash + directional arrow when a gesture fires.
 *
 * Driven by a `trigger` prop: the parent sets it to a unique string
 * (e.g. "flickUp|1700000000") whenever a new gesture arrives. We @Watch
 * the prop and kick off the animation; same trigger twice does nothing.
 *
 * State field names are prefixed with `b` (bScale, bAlpha, …) so they
 * don't shadow ArkUI's built-in component attributes like `.scale()` /
 * `.opacity()` — ArkTS strict mode rejects the clash at compile time.
 */

import { NebulaFont } from '../theme/Theme';

@Component
export struct GestureBurst {
  @Prop @Watch('onTriggerChange') trigger: string = '';

  @State private active: boolean = false;
  @State private glyph: string = '↑';
  @State private label: string = '';
  @State private tint: string = '#2E6BF6';

  @State private bScale: number = 0.4;
  @State private bAlpha: number = 0;
  @State private bOffsetY: number = 0;
  @State private bOffsetX: number = 0;
  @State private bGlow: number = 0;

  private timer: number = -1;
  private lastHandled: string = '';

  onTriggerChange(): void {
    if (this.trigger.length === 0) return;
    if (this.trigger === this.lastHandled) return;
    this.lastHandled = this.trigger;
    const pipe = this.trigger.indexOf('|');
    const kind = pipe >= 0 ? this.trigger.substring(0, pipe) : this.trigger;
    this.play(kind);
  }

  private play(kind: string): void {
    if (kind === 'flickUp')        { this.glyph = '↑'; this.label = '向上挥'; this.tint = '#12B27A'; }
    else if (kind === 'flickDown') { this.glyph = '↓'; this.label = '向下拉'; this.tint = '#F08B3A'; }
    else if (kind === 'flickLeft') { this.glyph = '←'; this.label = '左挥'; this.tint = '#2E6BF6'; }
    else if (kind === 'flickRight'){ this.glyph = '→'; this.label = '右挥'; this.tint = '#2E6BF6'; }
    else if (kind === 'shake')     { this.glyph = '✺'; this.label = '晃动回中'; this.tint = '#7F5CFF'; }
    else return;

    this.bScale = 0.4;
    this.bAlpha = 0;
    this.bOffsetX = 0;
    this.bOffsetY = 0;
    this.bGlow = 0;
    this.active = true;

    // Use the UIContext-scoped animateTo (the global one is deprecated).
    const ui = this.getUIContext();
    ui.animateTo({ duration: 180, curve: Curve.EaseOut }, (): void => {
      this.bScale = 1.15;
      this.bAlpha = 1;
      this.bGlow = 0.85;
    });
    ui.animateTo({ duration: 260, curve: Curve.EaseInOut, delay: 180 }, (): void => {
      this.bScale = 1.0;
      this.bGlow = 0.4;
      if (kind === 'flickUp')    this.bOffsetY = -40;
      if (kind === 'flickDown')  this.bOffsetY =  40;
      if (kind === 'flickLeft')  this.bOffsetX = -60;
      if (kind === 'flickRight') this.bOffsetX =  60;
    });
    ui.animateTo({ duration: 360, curve: Curve.EaseIn, delay: 480 }, (): void => {
      this.bAlpha = 0;
      this.bGlow = 0;
      this.bScale = 0.9;
    });

    if (this.timer >= 0) clearTimeout(this.timer);
    this.timer = setTimeout((): void => {
      this.active = false;
      this.timer = -1;
    }, 900);
  }

  aboutToDisappear(): void {
    if (this.timer >= 0) { clearTimeout(this.timer); this.timer = -1; }
  }

  build() {
    if (this.active) {
      Stack({ alignContent: Alignment.Center }) {
        Column()
          .width(360).height(360)
          .borderRadius(999)
          .backgroundColor(this.tint)
          .opacity(this.bGlow * 0.35)
          .scale({ x: this.bScale * 1.3, y: this.bScale * 1.3 })
          .blur(40)

        Column()
          .width(220).height(220)
          .borderRadius(999)
          .border({ width: 3, color: this.tint })
          .opacity(this.bAlpha * 0.7)
          .scale({ x: this.bScale * 1.05, y: this.bScale * 1.05 })

        Column({ space: 10 }) {
          Text(this.glyph)
            .fontSize(140)
            .fontWeight(FontWeight.Bold)
            .fontColor(this.tint)
          Text(this.label)
            .fontSize(NebulaFont.titleLg)
            .fontWeight(FontWeight.Bold)
            .fontColor('#0C1B33')
            .padding({ left: 16, right: 16, top: 6, bottom: 6 })
            .backgroundColor('#FFFFFFE8')
            .borderRadius(999)
            .border({ width: 1, color: '#27272AEE' })
        }
        .alignItems(HorizontalAlign.Center)
        .opacity(this.bAlpha)
        .scale({ x: this.bScale, y: this.bScale })
        .translate({ x: this.bOffsetX, y: this.bOffsetY })
      }
      .width('100%').height('100%')
      .hitTestBehavior(HitTestMode.Transparent)
    }
  }
}
