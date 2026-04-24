/**
 * Profile — mission-control aesthetic, port of the instrument-panel
 * language used on the Devices WebView: serif display heading, mono
 * readouts with leading zeros, 1px hairline borders, 4px tight corners,
 * uppercase tracked section labels. All data still comes from real
 * services (UserStats, PhoneInfo) — only the presentation changed.
 */

import {
  NebulaColor, NebulaFont, NebulaFontFace, NebulaRadius, NebulaSpace
} from '../theme/Theme';
import { IconName, NebulaIcon } from '../components/NebulaIcon';
import { ActivityRecord, StatSnapshot, UserStats } from '../services/UserStats';
import { PhoneInfo, PhoneSnapshot } from '../services/PhoneInfo';

class AchievementDef {
  title: string = '';
  subtitle: string = '';
  tone: string = 'brand';
  icon: string = IconName.Rocket;
  goal: number = 1;
}

function mkAch(title: string, subtitle: string, tone: string, icon: string, goal: number): AchievementDef {
  const a = new AchievementDef();
  a.title = title; a.subtitle = subtitle; a.tone = tone; a.icon = icon; a.goal = goal;
  return a;
}

function pad(n: number, w: number = 3): string {
  const s = String(Math.max(0, Math.floor(n)));
  if (s.length >= w) return s;
  return '0'.repeat(w - s.length) + s;
}

function fmtAgo(ts: number): string {
  const delta = Date.now() - ts;
  const sec = Math.floor(delta / 1000);
  if (sec < 60) return sec + 's';
  const min = Math.floor(sec / 60);
  if (min < 60) return min + 'min';
  const h = Math.floor(min / 60);
  if (h < 24) return h + 'h';
  const d = Math.floor(h / 24);
  if (d < 30) return d + 'd';
  const m = Math.floor(d / 30);
  return m + 'mo';
}

function kindLabel(k: string): string {
  if (k === 'workshop') return 'WORKSHOP';
  if (k === 'academy') return 'ACADEMY';
  if (k === 'launch') return 'LAUNCH';
  if (k === 'share') return 'SHARE';
  if (k === 'bookmark') return 'BOOKMARK';
  return k.toUpperCase();
}

function kindIcon(k: string): string {
  if (k === 'workshop') return IconName.Workshop;
  if (k === 'academy') return IconName.Academy;
  if (k === 'launch') return IconName.Rocket;
  if (k === 'share') return IconName.Share;
  if (k === 'bookmark') return IconName.Bookmark;
  return IconName.Orbit;
}

function kindTone(k: string): string {
  if (k === 'workshop') return 'brand';
  if (k === 'academy') return 'violet';
  if (k === 'launch') return 'coral';
  if (k === 'share') return 'gold';
  if (k === 'bookmark') return 'cyan';
  return 'neutral';
}

@Component
export struct ProfilePage {
  @State private stats: StatSnapshot = UserStats.instance.snap();
  @State private phone: PhoneSnapshot = PhoneInfo.instance.snap();
  private unsub: () => void = (): void => {};
  private unsubPhone: () => void = (): void => {};

  aboutToAppear(): void {
    this.unsub = UserStats.instance.subscribe((s: StatSnapshot): void => { this.stats = s; });
    this.unsubPhone = PhoneInfo.instance.subscribe((p: PhoneSnapshot): void => { this.phone = p; });
  }
  aboutToDisappear(): void {
    this.unsub();
    this.unsubPhone();
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

  private achievements(): AchievementDef[] {
    return [
      mkAch('初次发射', 'FIRST AR LAUNCH', 'coral', IconName.Rocket, 1),
      mkAch('模型收藏家', 'BOOKMARK 10 PARTS', 'violet', IconName.Bookmark, 10),
      mkAch('科普达人', 'READ 20 ARTICLES', 'cyan', IconName.Academy, 20),
      mkAch('分享先锋', 'SHARE 5 CARDS', 'gold', IconName.Share, 5)
    ];
  }

  private achievementCount(a: AchievementDef): number {
    if (a.icon === IconName.Rocket) return this.stats.launches;
    if (a.icon === IconName.Bookmark) return this.stats.partsBookmarked;
    if (a.icon === IconName.Academy) return this.stats.academyOpens;
    if (a.icon === IconName.Share) return this.stats.shares;
    return 0;
  }

  private achievementProgress(a: AchievementDef): number {
    return Math.min(1, this.achievementCount(a) / a.goal);
  }

  private displayName(): string {
    if (this.phone.deviceName.length > 0) return this.phone.deviceName;
    if (this.phone.deviceModel.length > 0) return this.phone.deviceModel;
    return '本机用户';
  }

  private totalActivity(): number {
    return this.stats.workshopOpens + this.stats.academyOpens
      + this.stats.launches + this.stats.shares;
  }

  private level(): number {
    return 1 + Math.floor(this.totalActivity() / 10);
  }

  // ---------- shared ui atoms ----------------------------------------

  @Builder sectionLabel(label: string, index: string) {
    Row({ space: 10 }) {
      Column().width(24).height(1).backgroundColor(NebulaColor.border)
      Text(index + ' / ' + label)
        .fontSize(10)
        .fontFamily(NebulaFontFace.mono)
        .fontColor(NebulaColor.textTertiary)
        .letterSpacing(2.2)
      Column().layoutWeight(1).height(1).backgroundColor(NebulaColor.border)
    }
    .width('100%')
  }

  @Builder statusBar() {
    Row({ space: 8 }) {
      Row({ space: 6 }) {
        Column().width(6).height(6).borderRadius(3).backgroundColor(NebulaColor.accentMint)
          .shadow({ radius: 8, color: NebulaColor.accentMint, offsetX: 0, offsetY: 0 })
        Text('LINK STABLE')
          // 'vp' explicitly (not default fp) so this doesn't scale with
          // the system font-size accessibility slider, matching the HTML
          // WebView on the Devices tab where `font-size: 10px` is fixed.
          // Without this, users with larger system text see Profile's
          // status bar grow while Devices' stays put.
          .fontSize('10vp')
          .lineHeight('14vp')
          .fontFamily(NebulaFontFace.mono)
          .fontColor(NebulaColor.textTertiary)
          .letterSpacing(2.0)
      }
      Blank()
      Text('星核 · Nebula Core')
        .fontSize('13vp')
        .lineHeight('16vp')
        .fontFamily(NebulaFontFace.serif)
        .fontColor(NebulaColor.textPrimary)
      Blank()
      Text('PROFILE')
        .fontSize('10vp')
        .lineHeight('14vp')
        .fontFamily(NebulaFontFace.mono)
        .fontColor(NebulaColor.textTertiary)
        .letterSpacing(2.0)
    }
    .width('100%')
    // Top padding reserves the status-bar / notch area. The outer
    // Stack in Index.ets calls expandSafeArea(SYSTEM, [TOP, BOTTOM])
    // which extends content under the system status bar, so this
    // header has to push its own chrome down ~52vp or it'll sit
    // underneath the clock/battery row.
    .padding({ top: 52, bottom: 14, left: 18, right: 18 })
    .backgroundColor('#0B0B0EE6')
    .backdropBlur(20)
    .border({
      width: { top: 0, bottom: 1, left: 0, right: 0 },
      color: NebulaColor.border,
      style: BorderStyle.Solid
    })
  }

  // ---------- panels --------------------------------------------------

  @Builder heroPanel() {
    Column() {
      // rail ------------------------------------------------------------
      Row({ space: 14 }) {
        Text('05 · USR')
          .fontSize(10)
          .fontFamily(NebulaFontFace.mono)
          .letterSpacing(1.8)
          .fontColor(NebulaColor.brand)
          .padding({ top: 4, bottom: 4, left: 8, right: 8 })
          .backgroundColor(NebulaColor.brandSoft)
          .borderRadius(3)
          .border({ width: 1, color: NebulaColor.brand })
        Column({ space: 2 }) {
          Text(this.displayName())
            .fontSize(20)
            .fontFamily(NebulaFontFace.serif)
            .fontColor(NebulaColor.textPrimary)
            .maxLines(1)
            .textOverflow({ overflow: TextOverflow.Ellipsis })
          Text((this.phone.brand + ' · ' + this.phone.osFullName).toUpperCase())
            .fontSize(10)
            .fontFamily(NebulaFontFace.mono)
            .fontColor(NebulaColor.textTertiary)
            .letterSpacing(1.4)
            .maxLines(1)
        }
        .alignItems(HorizontalAlign.Start)
        .layoutWeight(1)
        // Level badge
        Column({ space: 2 }) {
          Text('LV')
            .fontSize(9)
            .fontFamily(NebulaFontFace.mono)
            .fontColor(NebulaColor.textTertiary)
            .letterSpacing(1.6)
          Text(pad(this.level(), 2))
            .fontSize(22)
            .fontFamily(NebulaFontFace.mono)
            .fontWeight(FontWeight.Medium)
            .fontColor(NebulaColor.brand)
        }
        .alignItems(HorizontalAlign.End)
      }
      .width('100%')
      .padding({ left: 16, right: 16, top: 14, bottom: 14 })
      .border({
        width: { top: 0, bottom: 1, left: 0, right: 0 },
        color: NebulaColor.border,
        style: BorderStyle.Solid
      })

      // big hero display -----------------------------------------------
      Column({ space: 10 }) {
        Text('CUMULATIVE RUNS')
          .fontSize(10)
          .fontFamily(NebulaFontFace.mono)
          .fontColor(NebulaColor.textTertiary)
          .letterSpacing(2.2)
        Row() {
          Text(pad(this.totalActivity(), 4))
            .fontSize(64)
            .fontFamily(NebulaFontFace.mono)
            .fontWeight(FontWeight.Medium)
            .fontColor(NebulaColor.textPrimary)
            .letterSpacing(-1.0)
          Column({ space: 2 }) {
            Text('TIMES')
              .fontSize(10)
              .fontFamily(NebulaFontFace.mono)
              .fontColor(NebulaColor.textTertiary)
              .letterSpacing(1.8)
            Text('LIFETIME')
              .fontSize(10)
              .fontFamily(NebulaFontFace.mono)
              .fontColor(NebulaColor.textTertiary)
              .letterSpacing(1.8)
          }
          .alignItems(HorizontalAlign.Start)
          .padding({ left: 10, bottom: 8 })
          .alignSelf(ItemAlign.End)
        }
        .alignItems(VerticalAlign.Bottom)
      }
      .width('100%')
      .alignItems(HorizontalAlign.Start)
      .padding({ left: 16, right: 16, top: 20, bottom: 22 })

      // telemetry strip ------------------------------------------------
      Row() {
        this.teleCell('LAUNCHES', pad(this.stats.launches, 3), NebulaColor.brand, true)
        this.teleCell('ACADEMY', pad(this.stats.academyOpens, 3), NebulaColor.accentViolet, false)
        this.teleCell('SHARES', pad(this.stats.shares, 3), NebulaColor.accentGold, false)
        this.teleCell('MARKS', pad(this.stats.partsBookmarked, 3), NebulaColor.accentCyan, false)
      }
      .width('100%')
      .border({
        width: { top: 1, bottom: 0, left: 0, right: 0 },
        color: NebulaColor.border,
        style: BorderStyle.Solid
      })

      // footer ---------------------------------------------------------
      Row() {
        Text('SN.' + (this.phone.deviceModel.length > 0
              ? this.phone.deviceModel.replace(' ', '').toUpperCase().substring(0, 10)
              : '0000000000'))
          .fontSize(10)
          .fontFamily(NebulaFontFace.mono)
          .fontColor(NebulaColor.textTertiary)
          .letterSpacing(1.4)
        Blank()
        Text('CHECKSUM OK')
          .fontSize(10)
          .fontFamily(NebulaFontFace.mono)
          .fontColor(NebulaColor.textTertiary)
          .letterSpacing(1.4)
      }
      .width('100%')
      .padding({ left: 16, right: 16, top: 10, bottom: 10 })
      .border({
        width: { top: 1, bottom: 0, left: 0, right: 0 },
        color: NebulaColor.border,
        style: BorderStyle.Solid
      })
    }
    .width('100%')
    .backgroundColor(NebulaColor.surface)
    .border({ width: 1, color: NebulaColor.border })
    .borderRadius(NebulaRadius.tight)
    .clip(true)
  }

  @Builder teleCell(k: string, v: string, color: string, first: boolean) {
    Column({ space: 3 }) {
      Text(k)
        .fontSize(9)
        .fontFamily(NebulaFontFace.mono)
        .fontColor(NebulaColor.textTertiary)
        .letterSpacing(1.8)
      Text(v)
        .fontSize(15)
        .fontFamily(NebulaFontFace.mono)
        .fontWeight(FontWeight.Medium)
        .fontColor(color)
    }
    .alignItems(HorizontalAlign.Start)
    .layoutWeight(1)
    .padding({ left: 12, right: 12, top: 10, bottom: 10 })
    .border({
      width: { top: 0, bottom: 0, left: first ? 0 : 1, right: 0 },
      color: NebulaColor.border,
      style: BorderStyle.Solid
    })
  }

  // ---------- achievements (2×2 grid) --------------------------------

  @Builder achievementTile(a: AchievementDef) {
    Column({ space: 10 }) {
      Row() {
        Column() {
          NebulaIcon({ name: a.icon, glyphSize: 16, color: this.toneFg(a.tone), strokeW: 1.8 })
        }
        .width(28).height(28)
        .border({ width: 1, color: this.toneFg(a.tone) })
        .borderRadius(3)
        .justifyContent(FlexAlign.Center)
        .alignItems(HorizontalAlign.Center)
        Blank()
        Text(pad(this.achievementCount(a), 2) + ' / ' + pad(a.goal, 2))
          .fontSize(10)
          .fontFamily(NebulaFontFace.mono)
          .fontColor(this.toneFg(a.tone))
          .letterSpacing(1.4)
      }.width('100%')

      Column({ space: 4 }) {
        Text(a.title)
          .fontSize(15)
          .fontFamily(NebulaFontFace.serif)
          .fontColor(NebulaColor.textPrimary)
          .maxLines(1)
        Text(a.subtitle)
          .fontSize(10)
          .fontFamily(NebulaFontFace.mono)
          .fontColor(NebulaColor.textTertiary)
          .letterSpacing(1.4)
          .maxLines(1)
      }.alignItems(HorizontalAlign.Start)

      // progress bar — single hairline + filled segment
      Stack({ alignContent: Alignment.Start }) {
        Column().width('100%').height(2).backgroundColor(NebulaColor.border)
        Column().width((this.achievementProgress(a) * 100) + '%').height(2)
          .backgroundColor(this.toneFg(a.tone))
      }.width('100%').height(2)
    }
    .alignItems(HorizontalAlign.Start)
    .padding(14)
    .width('100%')
    .height(142)
    .backgroundColor(NebulaColor.surface)
    .border({ width: 1, color: NebulaColor.border })
    .borderRadius(NebulaRadius.tight)
  }

  // ---------- activity log ------------------------------------------

  @Builder logRow(l: ActivityRecord, last: boolean) {
    Row({ space: 12 }) {
      // Sequence-style color stripe
      Column().width(2).height(22).backgroundColor(this.toneFg(kindTone(l.kind)))
      Column({ space: 3 }) {
        Text(l.title.length > 0 ? l.title : kindLabel(l.kind))
          .fontSize(13)
          .fontFamily(NebulaFontFace.sans)
          .fontWeight(FontWeight.Medium)
          .fontColor(NebulaColor.textPrimary)
          .maxLines(1)
          .textOverflow({ overflow: TextOverflow.Ellipsis })
        Text(kindLabel(l.kind))
          .fontSize(9)
          .fontFamily(NebulaFontFace.mono)
          .fontColor(this.toneFg(kindTone(l.kind)))
          .letterSpacing(1.6)
      }
      .alignItems(HorizontalAlign.Start)
      .layoutWeight(1)
      Text(fmtAgo(l.time))
        .fontSize(11)
        .fontFamily(NebulaFontFace.mono)
        .fontColor(NebulaColor.textTertiary)
        .letterSpacing(0.6)
    }
    .width('100%')
    .padding({ top: 12, bottom: 12, left: 14, right: 14 })
    .border({
      width: { top: 0, bottom: last ? 0 : 1, left: 0, right: 0 },
      color: NebulaColor.border,
      style: BorderStyle.Solid
    })
  }

  @Builder emptyLog() {
    Column({ space: 8 }) {
      Text('—— NO ACTIVITY ——')
        .fontSize(11)
        .fontFamily(NebulaFontFace.mono)
        .fontColor(NebulaColor.textTertiary)
        .letterSpacing(2.0)
      Text('在 AR 工坊或科普中心开始探索')
        .fontSize(12)
        .fontFamily(NebulaFontFace.sans)
        .fontColor(NebulaColor.textTertiary)
    }
    .alignItems(HorizontalAlign.Center)
    .padding({ top: 36, bottom: 36 })
    .width('100%')
  }

  // ---------- main ----------------------------------------------------

  build() {
    Column() {
      this.statusBar()
      Scroll() {
        Column({ space: 26 }) {
          // page header
          Column({ space: 14 }) {
            this.sectionLabel('USER', '05')
            Text('我的')
              .fontSize(44)
              .fontFamily(NebulaFontFace.serif)
              .fontColor(NebulaColor.textPrimary)
              .letterSpacing(-1.0)
              .lineHeight(48)
            Text('REAL ACTIVITY · STORED IN @OHOS.DATA.PREFERENCES')
              .fontSize(10)
              .fontFamily(NebulaFontFace.mono)
              .fontColor(NebulaColor.textTertiary)
              .letterSpacing(1.8)
          }
          .alignItems(HorizontalAlign.Start)
          .padding({ left: 18, right: 18, top: 20 })
          .width('100%')

          // hero panel
          Column() { this.heroPanel() }
            .padding({ left: 18, right: 18 })

          // achievements
          Column({ space: 14 }) {
            this.sectionLabel('ACHIEVEMENTS', '06')
            Grid() {
              ForEach(this.achievements(), (a: AchievementDef): void => {
                GridItem() { this.achievementTile(a) }
              }, (a: AchievementDef): string => a.title)
            }
            .columnsTemplate('1fr 1fr')
            .rowsGap(12)
            .columnsGap(12)
            .height(296)
          }
          .padding({ left: 18, right: 18 })

          // activity log
          Column({ space: 14 }) {
            Row({ space: 10 }) {
              Column().width(24).height(1).backgroundColor(NebulaColor.border)
              Text('07 / ACTIVITY LOG')
                .fontSize(10)
                .fontFamily(NebulaFontFace.mono)
                .fontColor(NebulaColor.textTertiary)
                .letterSpacing(2.2)
              Column().layoutWeight(1).height(1).backgroundColor(NebulaColor.border)
              Text('N=' + pad(this.stats.recent.length, 2))
                .fontSize(10)
                .fontFamily(NebulaFontFace.mono)
                .fontColor(NebulaColor.textTertiary)
                .letterSpacing(1.6)
            }
            .width('100%')

            Column() {
              if (this.stats.recent.length === 0) {
                this.emptyLog()
              } else {
                ForEach(this.stats.recent, (l: ActivityRecord, i: number): void => {
                  this.logRow(l, i === this.stats.recent.length - 1)
                }, (l: ActivityRecord): string => l.kind + '-' + l.time)
              }
            }
            .width('100%')
            .backgroundColor(NebulaColor.surface)
            .border({ width: 1, color: NebulaColor.border })
            .borderRadius(NebulaRadius.tight)
          }
          .padding({ left: 18, right: 18 })

          // clear-log button
          if (this.stats.recent.length > 0) {
            Row({ space: 8 }) {
              NebulaIcon({ name: IconName.Reset, glyphSize: 11, color: NebulaColor.danger, strokeW: 2 })
              Text('PURGE LOG')
                .fontSize(10)
                .fontFamily(NebulaFontFace.mono)
                .fontColor(NebulaColor.danger)
                .letterSpacing(2.2)
            }
            .padding({ top: 10, bottom: 10, left: 14, right: 14 })
            .border({ width: 1, color: NebulaColor.danger })
            .borderRadius(NebulaRadius.tight)
            .alignSelf(ItemAlign.Start)
            .margin({ left: 18 })
            .onClick((): void => {
              UserStats.instance.reset().then((): void => {});
            })
          }

          // Clear the Dock (~56vp strip + 34vp safe-area filler + air).
          Column().height(140)
        }
        .width('100%')
        .alignItems(HorizontalAlign.Start)
      }
      .edgeEffect(EdgeEffect.Spring)
      .scrollBar(BarState.Off)
      .layoutWeight(1)
      .width('100%')
      .backgroundColor(NebulaColor.bg)
    }
    .width('100%').height('100%')
    .backgroundColor(NebulaColor.bg)
  }
}
