/**
 * Discover — live-data surface, mission-control aesthetic.
 *
 * Data sources (all free, no auth):
 *   - wheretheiss.at             → live ISS lat/lon/altitude/velocity
 *   - ll.thespacedevs.com        → upcoming orbital launches
 *   - api.sunrise-sunset.org     → today's sunrise / sunset for a point
 *
 * Visual language ported from the Devices WebView console:
 *   statusBar + serif page hero + mono telemetry + hairline borders +
 *   tight 4px corners + uppercase tracked section labels.
 *
 * On failure, each section shows a "未能获取" placeholder instead of faking
 * values. AR missions at the bottom stay static — they're navigation targets
 * into Workshop/Academy, not data.
 */

import {
  NebulaColor, NebulaFont, NebulaFontFace, NebulaRadius, NebulaSpace
} from '../theme/Theme';
import { IconName, NebulaIcon } from '../components/NebulaIcon';
import { AppBus } from '../services/AppBus';
import { IssPosition, LaunchInfo, LaunchLink, SpaceApi, SunInfo } from '../services/SpaceApi';

class Mission {
  title: string = '';
  caption: string = '';
  tone: string = 'brand';
  icon: string = IconName.Rocket;
  minutes: string = '';
  rocketId: string = '';
  academyId: string = '';
}

function mkMission(t: string, c: string, tone: string, i: string, m: string, rocketId: string, academyId: string): Mission {
  const x = new Mission();
  x.title = t; x.caption = c; x.tone = tone; x.icon = i; x.minutes = m;
  x.rocketId = rocketId; x.academyId = academyId;
  return x;
}

function pad2(n: number): string { return n < 10 ? '0' + n : '' + n; }

function formatTimeHHMM(d: Date): string {
  return pad2(d.getHours()) + ':' + pad2(d.getMinutes());
}

function formatDateMD(d: Date): string {
  return pad2(d.getMonth() + 1) + '.' + pad2(d.getDate());
}

function formatCountdownShort(ms: number): string {
  if (ms <= 0) return 'T-00:00';
  const totalSec = Math.floor(ms / 1000);
  const d = Math.floor(totalSec / 86400);
  const h = Math.floor((totalSec % 86400) / 3600);
  const m = Math.floor((totalSec % 3600) / 60);
  const s = totalSec % 60;
  if (d > 0) return d + 'D ' + pad2(h) + 'H';
  if (h > 0) return pad2(h) + ':' + pad2(m) + ':' + pad2(s);
  return pad2(m) + ':' + pad2(s);
}

function formatCountdownLong(ms: number): string {
  if (ms <= 0) return '正在发生';
  const totalSec = Math.floor(ms / 1000);
  const d = Math.floor(totalSec / 86400);
  const h = Math.floor((totalSec % 86400) / 3600);
  const m = Math.floor((totalSec % 3600) / 60);
  const s = totalSec % 60;
  if (d > 0) return d + ' 天 ' + h + ' 小时';
  if (h > 0) return h + ' 小时 ' + m + ' 分';
  if (m > 0) return m + ' 分 ' + s + ' 秒';
  return s + ' 秒';
}

function formatDay(d: Date): string {
  const delta = Math.round((d.getTime() - Date.now()) / 86400000);
  if (delta <= 0) return 'TODAY';
  if (delta === 1) return 'T+1';
  if (delta === 2) return 'T+2';
  if (delta < 7) return 'T+' + delta;
  return 'W+' + Math.round(delta / 7);
}

/**
 * Best-effort mapping from the Launch Library's rocket name into a catalog
 * id we actually have in Workshop. Empty string means "no match".
 */
function matchCatalog(rocketName: string): string {
  const n = rocketName.toLowerCase();
  if (n.indexOf('falcon') >= 0) return 'falcon-9';
  if (n.indexOf('saturn') >= 0) return 'saturn-v';
  if (n.indexOf('long march 5') >= 0 || n.indexOf('长征五号') >= 0 || n.indexOf('cz-5') >= 0) return 'long-march-5';
  return '';
}

function statusTone(status: string): string {
  const s = status.toLowerCase();
  if (s.indexOf('go') >= 0 || s.indexOf('success') >= 0) return 'mint';
  if (s.indexOf('hold') >= 0 || s.indexOf('tbd') >= 0 || s.indexOf('tbc') >= 0) return 'gold';
  if (s.indexOf('failure') >= 0) return 'coral';
  return 'neutral';
}

@Component
export struct DiscoverPage {
  @State private now: number = Date.now();
  @State private iss: IssPosition | null = null;
  @State private issError: boolean = false;
  @State private sun: SunInfo | null = null;
  @State private sunError: boolean = false;
  @State private launches: LaunchInfo[] = [];
  @State private launchesError: boolean = false;
  @State private loadingIss: boolean = true;
  @State private loadingLaunches: boolean = true;
  @State private loadingSun: boolean = true;
  @State private openLaunchId: string = '';
  @State private openLaunch: LaunchInfo | null = null;
  @State private loadingDetail: boolean = false;
  @State private heroImageLoaded: boolean = false;
  @State private heroImageFailed: boolean = false;

  private clockTimer: number = -1;
  private issTimer: number = -1;

  private readonly defaultLat: number = 39.9042;
  private readonly defaultLng: number = 116.4074;

  aboutToAppear(): void {
    const cachedIss = SpaceApi.instance.issCached();
    if (cachedIss !== null) {
      this.iss = cachedIss;
      this.loadingIss = false;
      this.issError = false;
    }
    const cachedLaunches = SpaceApi.instance.launchesCached();
    if (cachedLaunches.length > 0) {
      this.launches = cachedLaunches;
      this.loadingLaunches = false;
      this.launchesError = false;
    }
    const cachedSun = SpaceApi.instance.sunCached(this.defaultLat, this.defaultLng);
    if (cachedSun !== null) {
      this.sun = cachedSun;
      this.loadingSun = false;
      this.sunError = false;
    }

    this.clockTimer = setInterval((): void => { this.now = Date.now(); }, 1000);
    this.refreshAll(false);
    this.issTimer = setInterval((): void => { this.refreshIss(false); }, 30_000);
  }

  aboutToDisappear(): void {
    if (this.clockTimer >= 0) clearInterval(this.clockTimer);
    if (this.issTimer >= 0) clearInterval(this.issTimer);
  }

  private refreshAll(force: boolean): void {
    this.refreshIss(force);
    this.refreshLaunches(force);
    this.refreshSun(force);
  }

  private refreshIss(force: boolean): void {
    SpaceApi.instance.getIss(force).then((p: IssPosition | null): void => {
      this.loadingIss = false;
      if (p === null && this.iss === null) { this.issError = true; return; }
      if (p !== null) { this.iss = p; this.issError = false; }
    });
  }

  private refreshLaunches(force: boolean): void {
    SpaceApi.instance.getUpcomingLaunches(8, force).then((list: LaunchInfo[]): void => {
      this.loadingLaunches = false;
      if (list.length === 0 && this.launches.length === 0) {
        this.launchesError = true;
        return;
      }
      if (list.length > 0) {
        this.launches = list;
        this.launchesError = false;
      }
    });
  }

  private refreshSun(force: boolean): void {
    SpaceApi.instance.getSun(this.defaultLat, this.defaultLng, force).then((s: SunInfo | null): void => {
      this.loadingSun = false;
      if (s === null && this.sun === null) { this.sunError = true; return; }
      if (s !== null) { this.sun = s; this.sunError = false; }
    });
  }

  private openLaunchDetail(l: LaunchInfo): void {
    this.openLaunchId = l.id;
    this.openLaunch = l;
    this.heroImageLoaded = false;
    this.heroImageFailed = false;
    this.loadingDetail = true;
    SpaceApi.instance.fetchLaunchDetail(l.id).then((full: LaunchInfo | null): void => {
      this.loadingDetail = false;
      if (full !== null && this.openLaunchId === full.id) this.openLaunch = full;
    });
  }

  private closeLaunchDetail(): void {
    this.openLaunchId = '';
    this.openLaunch = null;
    this.loadingDetail = false;
    this.heroImageLoaded = false;
    this.heroImageFailed = false;
  }

  private heroImageUrl(l: LaunchInfo): string {
    if (l.image.length > 0) return l.image;
    if (l.rocketImage.length > 0) return l.rocketImage;
    return '';
  }

  private missions(): Mission[] {
    return [
      mkMission('长征五号 · AR 发射', '深空任务推荐', 'coral', IconName.Rocket, '90s', 'long-march-5', 'long-march-5'),
      mkMission('Falcon 9 一级回收', '商业发射进阶', 'brand', IconName.Bolt, '2min', 'falcon-9', 'falcon-9'),
      mkMission('土星五号登月', '阿波罗经典重现', 'gold', IconName.Layers, '3min', 'saturn-v', 'saturn-v'),
      mkMission('哈勃 · 太阳翼展开', '天文卫星特辑', 'violet', IconName.Satellite, '1min', '', 'hubble')
    ];
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

  private issFreshness(): string {
    if (this.iss === null) return '';
    const ageSec = Math.max(0, Math.floor((this.now - this.iss.timestamp) / 1000));
    return 'T+' + ageSec + 's · WHERETHEISS.AT';
  }

  // ---------- shared UI atoms ------------------------------------------

  @Builder statusBar(pageLabel: string) {
    Row({ space: 8 }) {
      Row({ space: 6 }) {
        Column().width(6).height(6).borderRadius(3).backgroundColor(NebulaColor.accentMint)
          .shadow({ radius: 8, color: NebulaColor.accentMint, offsetX: 0, offsetY: 0 })
        Text('LINK STABLE')
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
      Text(pageLabel)
        .fontSize('10vp')
        .lineHeight('14vp')
        .fontFamily(NebulaFontFace.mono)
        .fontColor(NebulaColor.textTertiary)
        .letterSpacing(2.0)
    }
    .width('100%')
    .padding({ top: 52, bottom: 14, left: 18, right: 18 })
    .backgroundColor('#0B0B0EE6')
    .backdropBlur(20)
    .border({
      width: { top: 0, bottom: 1, left: 0, right: 0 },
      color: NebulaColor.border,
      style: BorderStyle.Solid
    })
  }

  @Builder sectionLabel(label: string, index: string, trailing: string) {
    Row({ space: 10 }) {
      Column().width(24).height(1).backgroundColor(NebulaColor.border)
      Text(index + ' / ' + label)
        .fontSize('10vp')
        .fontFamily(NebulaFontFace.mono)
        .fontColor(NebulaColor.textTertiary)
        .letterSpacing(2.2)
      Column().layoutWeight(1).height(1).backgroundColor(NebulaColor.border)
      if (trailing.length > 0) {
        Text(trailing)
          .fontSize('10vp')
          .fontFamily(NebulaFontFace.mono)
          .fontColor(NebulaColor.textTertiary)
          .letterSpacing(1.6)
      }
    }
    .width('100%')
  }

  @Builder pageHero(eyebrow: string, title: string, subtitle: string) {
    Column({ space: 14 }) {
      Row({ space: 10 }) {
        Column().width(24).height(1).backgroundColor(NebulaColor.border)
        Text(eyebrow)
          .fontSize('10vp')
          .fontFamily(NebulaFontFace.mono)
          .fontColor(NebulaColor.textTertiary)
          .letterSpacing(2.2)
        Column().layoutWeight(1).height(1).backgroundColor(NebulaColor.border)
      }
      Text(title)
        .fontSize('40vp')
        .lineHeight('44vp')
        .fontFamily(NebulaFontFace.serif)
        .fontColor(NebulaColor.textPrimary)
        .letterSpacing(-0.8)
      if (subtitle.length > 0) {
        Text(subtitle)
          .fontSize('10vp')
          .lineHeight('14vp')
          .fontFamily(NebulaFontFace.mono)
          .fontColor(NebulaColor.textTertiary)
          .letterSpacing(1.8)
      }
    }
    .alignItems(HorizontalAlign.Start)
    .width('100%')
  }

  // ---------- HERO: live ISS instrument panel --------------------------

  @Builder heroCard() {
    Column() {
      // rail
      Row({ space: 14 }) {
        Text(this.iss !== null ? '● LIVE' : (this.issError ? '× OFFLINE' : '~ CONNECTING'))
          .fontSize('10vp')
          .fontFamily(NebulaFontFace.mono)
          .fontColor(this.iss !== null ? NebulaColor.brand
                   : this.issError ? NebulaColor.accentCoral : NebulaColor.textTertiary)
          .letterSpacing(1.8)
          .padding({ top: 4, bottom: 4, left: 8, right: 8 })
          .backgroundColor(this.iss !== null ? NebulaColor.brandSoft : NebulaColor.bgSoft)
          .border({
            width: 1,
            color: this.iss !== null ? NebulaColor.brand
                 : this.issError ? NebulaColor.accentCoral : NebulaColor.border
          })
          .borderRadius(3)
        Column({ space: 2 }) {
          Text('国际空间站')
            .fontSize('15vp')
            .lineHeight('18vp')
            .fontFamily(NebulaFontFace.serif)
            .fontColor(NebulaColor.textPrimary)
          Text('ISS · ZARYA · 1998-067A')
            .fontSize('9vp')
            .fontFamily(NebulaFontFace.mono)
            .fontColor(NebulaColor.textTertiary)
            .letterSpacing(1.4)
        }
        .alignItems(HorizontalAlign.Start)
        .layoutWeight(1)
        Column({ space: 2 }) {
          Text(pad2(new Date(this.now).getHours()) + ':' + pad2(new Date(this.now).getMinutes()) + ':' + pad2(new Date(this.now).getSeconds()))
            .fontSize('13vp')
            .fontFamily(NebulaFontFace.mono)
            .fontWeight(FontWeight.Medium)
            .fontColor(NebulaColor.textPrimary)
          Text('LOCAL')
            .fontSize('9vp')
            .fontFamily(NebulaFontFace.mono)
            .fontColor(NebulaColor.textTertiary)
            .letterSpacing(1.8)
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

      // big velocity display
      Column({ space: 8 }) {
        Text('ORBITAL VELOCITY')
          .fontSize('10vp')
          .fontFamily(NebulaFontFace.mono)
          .fontColor(NebulaColor.textTertiary)
          .letterSpacing(2.2)
        Row() {
          Text(this.iss !== null ? this.iss.velocityKmS().toFixed(2) : '--.--')
            .fontSize('60vp')
            .lineHeight('60vp')
            .fontFamily(NebulaFontFace.mono)
            .fontWeight(FontWeight.Medium)
            .fontColor(NebulaColor.textPrimary)
            .letterSpacing(-1.0)
          Column({ space: 2 }) {
            Text('KM/S')
              .fontSize('10vp')
              .fontFamily(NebulaFontFace.mono)
              .fontColor(NebulaColor.textTertiary)
              .letterSpacing(1.8)
            Text(this.iss !== null ? this.iss.orbitalPeriodMin().toFixed(1) + 'min/orbit' : 'awaiting')
              .fontSize('10vp')
              .fontFamily(NebulaFontFace.mono)
              .fontColor(NebulaColor.textTertiary)
              .letterSpacing(1.2)
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

      // telemetry strip
      Row() {
        this.teleCell('LON', this.iss !== null ? this.iss.longitude.toFixed(2) + '°' : '—', true)
        this.teleCell('LAT', this.iss !== null ? this.iss.latitude.toFixed(2) + '°' : '—', false)
        this.teleCell('ALT', this.iss !== null ? this.iss.altitudeKm.toFixed(0) + 'km' : '—', false)
        this.teleCell('SIDE', this.iss !== null ? (this.iss.visibility === 'daylight' ? 'DAY' : 'NGT') : '—', false)
      }
      .width('100%')
      .border({
        width: { top: 1, bottom: 0, left: 0, right: 0 },
        color: NebulaColor.border,
        style: BorderStyle.Solid
      })

      // footer
      Row({ space: 12 }) {
        Row({ space: 6 }) {
          Column().width(5).height(5).borderRadius(3).backgroundColor(
            this.iss !== null ? NebulaColor.brand : NebulaColor.textTertiary)
          Text(this.issFreshness())
            .fontSize('9vp')
            .fontFamily(NebulaFontFace.mono)
            .fontColor(NebulaColor.textTertiary)
            .letterSpacing(1.4)
        }
        .layoutWeight(1)
        Row({ space: 6 }) {
          NebulaIcon({ name: IconName.Reset, glyphSize: 10, color: NebulaColor.textSecondary, strokeW: 2 })
          Text('REFRESH')
            .fontSize('9vp')
            .fontFamily(NebulaFontFace.mono)
            .fontColor(NebulaColor.textSecondary)
            .letterSpacing(1.6)
        }
        .padding({ top: 5, bottom: 5, left: 10, right: 10 })
        .border({ width: 1, color: NebulaColor.border })
        .borderRadius(3)
        .onClick((): void => this.refreshIss(true))
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

  @Builder teleCell(k: string, v: string, first: boolean) {
    Column({ space: 3 }) {
      Text(k)
        .fontSize('9vp')
        .fontFamily(NebulaFontFace.mono)
        .fontColor(NebulaColor.textTertiary)
        .letterSpacing(1.8)
      Text(v)
        .fontSize('14vp')
        .fontFamily(NebulaFontFace.mono)
        .fontWeight(FontWeight.Medium)
        .fontColor(NebulaColor.textPrimary)
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

  // ---------- missions grid --------------------------------------------

  @Builder missionTile(m: Mission) {
    Column({ space: 10 }) {
      Row() {
        Column() {
          NebulaIcon({ name: m.icon, glyphSize: 16, color: this.toneFg(m.tone), strokeW: 1.8 })
        }
        .width(28).height(28)
        .border({ width: 1, color: this.toneFg(m.tone) })
        .borderRadius(3)
        .justifyContent(FlexAlign.Center)
        .alignItems(HorizontalAlign.Center)
        Blank()
        Text(m.minutes)
          .fontSize('9vp')
          .fontFamily(NebulaFontFace.mono)
          .fontColor(NebulaColor.textTertiary)
          .letterSpacing(1.4)
      }
      .width('100%')

      Column({ space: 4 }) {
        Text(m.title)
          .fontSize('15vp')
          .lineHeight('19vp')
          .fontFamily(NebulaFontFace.serif)
          .fontColor(NebulaColor.textPrimary)
          .maxLines(2)
        Text(m.caption)
          .fontSize('10vp')
          .fontFamily(NebulaFontFace.mono)
          .fontColor(NebulaColor.textTertiary)
          .letterSpacing(1.4)
          .maxLines(1)
      }
      .alignItems(HorizontalAlign.Start)

      Blank()

      Row({ space: 4 }) {
        Text('ENTER AR')
          .fontSize('10vp')
          .fontFamily(NebulaFontFace.mono)
          .fontColor(this.toneFg(m.tone))
          .letterSpacing(2.0)
        NebulaIcon({ name: IconName.Arrow, glyphSize: 11, color: this.toneFg(m.tone), strokeW: 2.4 })
      }
    }
    .alignItems(HorizontalAlign.Start)
    .padding(14)
    .width('100%')
    .height(152)
    .backgroundColor(NebulaColor.surface)
    .border({ width: 1, color: NebulaColor.border })
    .borderRadius(NebulaRadius.tight)
    .onClick((): void => {
      if (m.rocketId.length > 0) AppBus.instance.openInWorkshop(m.rocketId);
      else if (m.academyId.length > 0) AppBus.instance.openInAcademy(m.academyId);
    })
  }

  // ---------- sun / iss footprint rows ---------------------------------

  @Builder skyRow(kind: string, label: string, iso: string, tone: string, last: boolean) {
    Row({ space: 12 }) {
      Column().width(2).height(30).backgroundColor(this.toneFg(tone))
      Column({ space: 3 }) {
        Text(label)
          .fontSize('14vp')
          .fontFamily(NebulaFontFace.sans)
          .fontWeight(FontWeight.Medium)
          .fontColor(NebulaColor.textPrimary)
        Text(kind)
          .fontSize('9vp')
          .fontFamily(NebulaFontFace.mono)
          .fontColor(this.toneFg(tone))
          .letterSpacing(1.8)
      }
      .alignItems(HorizontalAlign.Start)
      .layoutWeight(1)
      Column({ space: 2 }) {
        Text(iso.length > 0 ? formatTimeHHMM(new Date(iso)) : '--:--')
          .fontSize('16vp')
          .fontFamily(NebulaFontFace.mono)
          .fontWeight(FontWeight.Medium)
          .fontColor(NebulaColor.textPrimary)
        Text(iso.length > 0 && new Date(iso).getTime() > this.now
              ? 'T-' + formatCountdownShort(new Date(iso).getTime() - this.now)
              : 'PASSED')
          .fontSize('9vp')
          .fontFamily(NebulaFontFace.mono)
          .fontColor(NebulaColor.textTertiary)
          .letterSpacing(1.4)
      }
      .alignItems(HorizontalAlign.End)
    }
    .width('100%')
    .padding({ top: 12, bottom: 12, left: 12, right: 14 })
    .border({
      width: { top: 0, bottom: last ? 0 : 1, left: 0, right: 0 },
      color: NebulaColor.border,
      style: BorderStyle.Solid
    })
  }

  @Builder issFootprintRow(last: boolean) {
    Row({ space: 12 }) {
      Column().width(2).height(30).backgroundColor(NebulaColor.accentCyan)
      Column({ space: 3 }) {
        Text('ISS 当前足印')
          .fontSize('14vp')
          .fontFamily(NebulaFontFace.sans)
          .fontWeight(FontWeight.Medium)
          .fontColor(NebulaColor.textPrimary)
        Text(this.iss !== null
              ? pad2(Math.abs(Math.floor(this.iss.longitude))) + (this.iss.longitude >= 0 ? '°E' : '°W')
                + ' · ' + pad2(Math.abs(Math.floor(this.iss.latitude))) + (this.iss.latitude >= 0 ? '°N' : '°S')
              : 'AWAITING FIX')
          .fontSize('9vp')
          .fontFamily(NebulaFontFace.mono)
          .fontColor(NebulaColor.accentCyan)
          .letterSpacing(1.6)
      }
      .alignItems(HorizontalAlign.Start)
      .layoutWeight(1)
      Column({ space: 2 }) {
        Text(this.iss !== null ? this.iss.altitudeKm.toFixed(0) + 'km' : '--')
          .fontSize('16vp')
          .fontFamily(NebulaFontFace.mono)
          .fontWeight(FontWeight.Medium)
          .fontColor(NebulaColor.textPrimary)
        Text(this.iss !== null ? (this.iss.visibility === 'daylight' ? 'DAYSIDE' : 'NIGHTSIDE') : '')
          .fontSize('9vp')
          .fontFamily(NebulaFontFace.mono)
          .fontColor(NebulaColor.textTertiary)
          .letterSpacing(1.4)
      }
      .alignItems(HorizontalAlign.End)
    }
    .width('100%')
    .padding({ top: 12, bottom: 12, left: 12, right: 14 })
    .border({
      width: { top: 0, bottom: last ? 0 : 1, left: 0, right: 0 },
      color: NebulaColor.border,
      style: BorderStyle.Solid
    })
    .onClick((): void => AppBus.instance.openInAcademy('iss'))
  }

  // ---------- launch list row ------------------------------------------

  @Builder launchRow(l: LaunchInfo, last: boolean) {
    Row({ space: 12 }) {
      Column().width(2).height(54).backgroundColor(this.toneFg(statusTone(l.status)))
      Column({ space: 3 }) {
        Row({ space: 6 }) {
          Text(formatDay(l.net))
            .fontSize('9vp')
            .fontFamily(NebulaFontFace.mono)
            .fontColor(this.toneFg(statusTone(l.status)))
            .letterSpacing(1.8)
          Text(formatDateMD(l.net) + ' · ' + formatTimeHHMM(l.net))
            .fontSize('9vp')
            .fontFamily(NebulaFontFace.mono)
            .fontColor(NebulaColor.textTertiary)
            .letterSpacing(1.4)
        }
        Text(l.rocketName)
          .fontSize('15vp')
          .fontFamily(NebulaFontFace.serif)
          .fontColor(NebulaColor.textPrimary)
          .maxLines(1)
          .textOverflow({ overflow: TextOverflow.Ellipsis })
        Text(l.name)
          .fontSize('11vp')
          .fontFamily(NebulaFontFace.sans)
          .fontColor(NebulaColor.textSecondary)
          .maxLines(1)
          .textOverflow({ overflow: TextOverflow.Ellipsis })
      }
      .alignItems(HorizontalAlign.Start)
      .layoutWeight(1)
      Column({ space: 4 }) {
        Text((l.status.length > 0 ? l.status : 'PLANNED').toUpperCase())
          .fontSize('9vp')
          .fontFamily(NebulaFontFace.mono)
          .fontColor(this.toneFg(statusTone(l.status)))
          .letterSpacing(1.6)
          .maxLines(1)
        NebulaIcon({ name: IconName.Arrow, glyphSize: 12, color: NebulaColor.textTertiary, strokeW: 2 })
      }
      .alignItems(HorizontalAlign.End)
    }
    .width('100%')
    .padding({ top: 14, bottom: 14, left: 12, right: 14 })
    .border({
      width: { top: 0, bottom: last ? 0 : 1, left: 0, right: 0 },
      color: NebulaColor.border,
      style: BorderStyle.Solid
    })
    .onClick((): void => this.openLaunchDetail(l))
  }

  // ---------- LAUNCH DETAIL VIEW ---------------------------------------

  @Builder heroMedia(url: string) {
    Stack() {
      if (!this.heroImageLoaded && !this.heroImageFailed) {
        Column({ space: 10 }) {
          LoadingProgress()
            .color(NebulaColor.brand)
            .width(36).height(36)
          Text('LOADING · IMAGE')
            .fontSize('10vp')
            .fontFamily(NebulaFontFace.mono)
            .fontColor(NebulaColor.textTertiary)
            .letterSpacing(2.0)
        }
        .width('100%').height('100%')
        .justifyContent(FlexAlign.Center)
        .alignItems(HorizontalAlign.Center)
        .backgroundColor(NebulaColor.bgSoft)
      }
      if (this.heroImageFailed) {
        Column({ space: 8 }) {
          NebulaIcon({ name: IconName.Satellite, glyphSize: 24, color: NebulaColor.textTertiary, strokeW: 1.6 })
          Text('IMAGE UNAVAILABLE')
            .fontSize('10vp')
            .fontFamily(NebulaFontFace.mono)
            .fontColor(NebulaColor.textTertiary)
            .letterSpacing(2.0)
        }
        .width('100%').height('100%')
        .justifyContent(FlexAlign.Center)
        .alignItems(HorizontalAlign.Center)
        .backgroundColor(NebulaColor.bgSoft)
      }
      Image(url)
        .width('100%').height('100%')
        .objectFit(ImageFit.Cover)
        .opacity(this.heroImageLoaded ? 1 : 0)
        .animation({ duration: 260, curve: Curve.EaseOut })
        .onComplete((): void => { this.heroImageLoaded = true; })
        .onError((): void => { this.heroImageFailed = true; this.heroImageLoaded = true; })
    }
    .width('100%').height(200)
    .borderRadius(NebulaRadius.tight)
    .clip(true)
    .backgroundColor(NebulaColor.surface)
    .border({ width: 1, color: NebulaColor.border })
  }

  @Builder detailStatPair(label: string, value: string) {
    Column({ space: 4 }) {
      Text(label)
        .fontSize('9vp')
        .fontFamily(NebulaFontFace.mono)
        .fontColor(NebulaColor.textTertiary)
        .letterSpacing(1.8)
      Text(value)
        .fontSize('14vp')
        .fontFamily(NebulaFontFace.sans)
        .fontWeight(FontWeight.Medium)
        .fontColor(NebulaColor.textPrimary)
        .maxLines(2)
        .textOverflow({ overflow: TextOverflow.Ellipsis })
    }
    .alignItems(HorizontalAlign.Start)
    .layoutWeight(1)
  }

  @Builder linkRow(l: LaunchLink, tone: string, last: boolean) {
    Row({ space: 10 }) {
      Column().width(2).height(28).backgroundColor(this.toneFg(tone))
      Column({ space: 3 }) {
        Text(l.title.length > 0 ? l.title : l.url)
          .fontSize('13vp')
          .fontFamily(NebulaFontFace.sans)
          .fontWeight(FontWeight.Medium)
          .fontColor(NebulaColor.textPrimary)
          .maxLines(1)
          .textOverflow({ overflow: TextOverflow.Ellipsis })
        Text(l.url)
          .fontSize('9vp')
          .fontFamily(NebulaFontFace.mono)
          .fontColor(NebulaColor.textTertiary)
          .letterSpacing(0.6)
          .maxLines(1)
          .textOverflow({ overflow: TextOverflow.Ellipsis })
      }
      .alignItems(HorizontalAlign.Start)
      .layoutWeight(1)
      NebulaIcon({ name: IconName.Arrow, glyphSize: 12, color: NebulaColor.textTertiary, strokeW: 2 })
    }
    .width('100%')
    .padding({ top: 12, bottom: 12, left: 10, right: 14 })
    .border({
      width: { top: 0, bottom: last ? 0 : 1, left: 0, right: 0 },
      color: NebulaColor.border,
      style: BorderStyle.Solid
    })
  }

  @Builder launchDetailView(l: LaunchInfo) {
    Column() {
      this.statusBar('LAUNCH')
      Scroll() {
        Column({ space: 20 }) {
          // Back button
          Row({ space: 8 }) {
            NebulaIcon({ name: IconName.Arrow, glyphSize: 12, color: NebulaColor.textPrimary, strokeW: 2 })
              .rotate({ angle: 180 })
            Text('BACK · DISCOVER')
              .fontSize('10vp')
              .fontFamily(NebulaFontFace.mono)
              .fontColor(NebulaColor.textPrimary)
              .letterSpacing(2.0)
          }
          .padding({ top: 8, bottom: 8, left: 12, right: 14 })
          .border({ width: 1, color: NebulaColor.border })
          .borderRadius(NebulaRadius.tight)
          .alignSelf(ItemAlign.Start)
          .margin({ left: 18, top: 18 })
          .onClick((): void => this.closeLaunchDetail())

          // hero image
          if (this.heroImageUrl(l).length > 0) {
            Column() { this.heroMedia(this.heroImageUrl(l)) }
              .padding({ left: 18, right: 18 })
              .width('100%')
          }

          // header block
          Column({ space: 10 }) {
            Row({ space: 6 }) {
              Text((l.agencyAbbrev.length > 0 ? l.agencyAbbrev : (l.agency.length > 0 ? l.agency : 'N/A')).toUpperCase())
                .fontSize('9vp')
                .fontFamily(NebulaFontFace.mono)
                .fontColor(NebulaColor.brand)
                .letterSpacing(1.8)
                .padding({ top: 3, bottom: 3, left: 7, right: 7 })
                .border({ width: 1, color: NebulaColor.brand })
                .borderRadius(2)
              Text((l.status.length > 0 ? l.status : 'PLANNED').toUpperCase())
                .fontSize('9vp')
                .fontFamily(NebulaFontFace.mono)
                .fontColor(this.toneFg(statusTone(l.status)))
                .letterSpacing(1.8)
                .padding({ top: 3, bottom: 3, left: 7, right: 7 })
                .border({ width: 1, color: this.toneFg(statusTone(l.status)) })
                .borderRadius(2)
              if (l.missionType.length > 0) {
                Text(l.missionType.toUpperCase())
                  .fontSize('9vp')
                  .fontFamily(NebulaFontFace.mono)
                  .fontColor(NebulaColor.accentViolet)
                  .letterSpacing(1.8)
                  .padding({ top: 3, bottom: 3, left: 7, right: 7 })
                  .border({ width: 1, color: NebulaColor.accentViolet })
                  .borderRadius(2)
              }
            }
            Text(l.rocketName)
              .fontSize('32vp')
              .lineHeight('36vp')
              .fontFamily(NebulaFontFace.serif)
              .fontColor(NebulaColor.textPrimary)
              .letterSpacing(-0.5)
              .maxLines(2)
            Text(l.name)
              .fontSize('13vp')
              .lineHeight('20vp')
              .fontFamily(NebulaFontFace.sans)
              .fontColor(NebulaColor.textSecondary)
          }
          .alignItems(HorizontalAlign.Start)
          .padding({ left: 18, right: 18 })
          .width('100%')

          // countdown card
          Column({ space: 8 }) {
            Text(l.net.getTime() > this.now ? 'T-MINUS' : 'LAUNCHED')
              .fontSize('10vp')
              .fontFamily(NebulaFontFace.mono)
              .fontColor(NebulaColor.textTertiary)
              .letterSpacing(2.2)
            Text(l.net.getTime() > this.now
                  ? formatCountdownLong(l.net.getTime() - this.now)
                  : '任务已启动')
              .fontSize('36vp')
              .lineHeight('40vp')
              .fontFamily(NebulaFontFace.mono)
              .fontWeight(FontWeight.Medium)
              .fontColor(NebulaColor.textPrimary)
              .letterSpacing(-0.5)
            Text('SCHEDULED · ' + l.net.toLocaleString().toUpperCase())
              .fontSize('10vp')
              .fontFamily(NebulaFontFace.mono)
              .fontColor(NebulaColor.textTertiary)
              .letterSpacing(1.4)
          }
          .alignItems(HorizontalAlign.Start)
          .padding(16)
          .width('100%')
          .backgroundColor(NebulaColor.surface)
          .border({ width: 1, color: NebulaColor.border })
          .borderRadius(NebulaRadius.tight)

          // stats card
          Column({ space: 14 }) {
            Row({ space: 14 }) {
              this.detailStatPair('PAD', l.padName.length > 0 ? l.padName : '—')
              this.detailStatPair('LOCATION', l.locationName.length > 0 ? l.locationName : '—')
            }
            Column().width('100%').height(1).backgroundColor(NebulaColor.border)
            Row({ space: 14 }) {
              this.detailStatPair('AGENCY', l.agency.length > 0 ? l.agency : '—')
              this.detailStatPair('ORBIT', l.orbitName.length > 0 ? l.orbitName : '—')
            }
            if (l.probability >= 0) {
              Column().width('100%').height(1).backgroundColor(NebulaColor.border)
              Row({ space: 14 }) {
                this.detailStatPair('GO %', l.probability + '%')
                this.detailStatPair('STATUS', l.statusDescription.length > 0 ? l.statusDescription : l.status)
              }
            }
          }
          .padding(16)
          .width('100%')
          .backgroundColor(NebulaColor.surface)
          .border({ width: 1, color: NebulaColor.border })
          .borderRadius(NebulaRadius.tight)

          // mission description
          if (l.missionDescription.length > 0) {
            Column({ space: 10 }) {
              Text('MISSION BRIEF')
                .fontSize('10vp')
                .fontFamily(NebulaFontFace.mono)
                .fontColor(NebulaColor.textTertiary)
                .letterSpacing(2.2)
              Text(l.missionDescription)
                .fontSize('13vp')
                .lineHeight('20vp')
                .fontFamily(NebulaFontFace.sans)
                .fontColor(NebulaColor.textSecondary)
            }
            .alignItems(HorizontalAlign.Start)
            .padding(16)
            .width('100%')
            .backgroundColor(NebulaColor.surface)
            .border({ width: 1, color: NebulaColor.border })
            .borderRadius(NebulaRadius.tight)
          }

          // rocket description
          if (l.rocketDescription.length > 0) {
            Column({ space: 10 }) {
              Text('VEHICLE BRIEF')
                .fontSize('10vp')
                .fontFamily(NebulaFontFace.mono)
                .fontColor(NebulaColor.textTertiary)
                .letterSpacing(2.2)
              Text(l.rocketDescription)
                .fontSize('13vp')
                .lineHeight('20vp')
                .fontFamily(NebulaFontFace.sans)
                .fontColor(NebulaColor.textSecondary)
            }
            .alignItems(HorizontalAlign.Start)
            .padding(16)
            .width('100%')
            .backgroundColor(NebulaColor.surface)
            .border({ width: 1, color: NebulaColor.border })
            .borderRadius(NebulaRadius.tight)
          }

          // links
          if (l.vidURLs.length > 0 || l.infoURLs.length > 0) {
            Column() {
              Row({ space: 10 }) {
                Text('LIVE & RESOURCES')
                  .fontSize('10vp')
                  .fontFamily(NebulaFontFace.mono)
                  .fontColor(NebulaColor.textTertiary)
                  .letterSpacing(2.2)
              }
              .padding({ left: 14, right: 14, top: 12, bottom: 12 })
              .width('100%')
              .border({
                width: { top: 0, bottom: 1, left: 0, right: 0 },
                color: NebulaColor.border,
                style: BorderStyle.Solid
              })

              ForEach(l.vidURLs, (link: LaunchLink, i: number): void => {
                this.linkRow(link, 'coral', i === l.vidURLs.length - 1 && l.infoURLs.length === 0)
              }, (link: LaunchLink): string => 'v-' + link.url)
              ForEach(l.infoURLs, (link: LaunchLink, i: number): void => {
                this.linkRow(link, 'brand', i === l.infoURLs.length - 1)
              }, (link: LaunchLink): string => 'i-' + link.url)
            }
            .alignItems(HorizontalAlign.Start)
            .width('100%')
            .backgroundColor(NebulaColor.surface)
            .border({ width: 1, color: NebulaColor.border })
            .borderRadius(NebulaRadius.tight)
          }

          // catalog CTA
          if (matchCatalog(l.rocketName).length > 0) {
            Row({ space: 8 }) {
              NebulaIcon({ name: IconName.Workshop, glyphSize: 12, color: '#FAFAFA', strokeW: 2 })
              Text('OPEN IN AR WORKSHOP · ' + l.rocketName.toUpperCase())
                .fontSize('10vp')
                .fontFamily(NebulaFontFace.mono)
                .fontColor('#FAFAFA')
                .letterSpacing(2.0)
                .maxLines(1)
                .textOverflow({ overflow: TextOverflow.Ellipsis })
            }
            .padding({ top: 12, bottom: 12, left: 14, right: 14 })
            .backgroundColor(NebulaColor.brand)
            .borderRadius(NebulaRadius.tight)
            .onClick((): void => {
              AppBus.instance.openInWorkshop(matchCatalog(l.rocketName));
            })
          }

          Text('SOURCE · LAUNCH LIBRARY 2 · THESPACEDEVS.COM')
            .fontSize('9vp')
            .fontFamily(NebulaFontFace.mono)
            .fontColor(NebulaColor.textTertiary)
            .letterSpacing(1.4)
            .padding({ left: 18, right: 18 })

          Column().height(140)
        }
        .alignItems(HorizontalAlign.Start)
        .padding({ left: 18, right: 18 })
        .width('100%')
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

  // ---------- main list ------------------------------------------------

  @Builder errorStrip(msg: string) {
    Row({ space: 10 }) {
      NebulaIcon({ name: IconName.Info, glyphSize: 12, color: NebulaColor.accentCoral, strokeW: 2 })
      Text(msg)
        .fontSize('11vp')
        .fontFamily(NebulaFontFace.sans)
        .fontColor(NebulaColor.textSecondary)
        .layoutWeight(1)
      Row({ space: 6 }) {
        NebulaIcon({ name: IconName.Reset, glyphSize: 10, color: NebulaColor.textPrimary, strokeW: 2 })
        Text('RETRY')
          .fontSize('9vp')
          .fontFamily(NebulaFontFace.mono)
          .fontColor(NebulaColor.textPrimary)
          .letterSpacing(1.8)
      }
      .padding({ top: 5, bottom: 5, left: 10, right: 10 })
      .border({ width: 1, color: NebulaColor.border })
      .borderRadius(3)
      .onClick((): void => this.refreshAll(true))
    }
    .width('100%')
    .padding(14)
    .backgroundColor(NebulaColor.surface)
    .border({ width: 1, color: NebulaColor.border })
    .borderRadius(NebulaRadius.tight)
  }

  build() {
    Stack() {
      if (this.openLaunch !== null) {
        this.launchDetailView(this.openLaunch!)
      } else {
        this.discoverList()
      }
    }
    .width('100%').height('100%')
  }

  @Builder discoverList() {
    Column() {
      this.statusBar('DISCOVER')
      Scroll() {
        Column({ space: 26 }) {
          // page hero
          Column() {
            this.pageHero('03 / DISCOVER', '发现', '')
          }
          .padding({ left: 18, right: 18, top: 20 })
          .width('100%')

          // ISS hero card
          Column() { this.heroCard() }
            .padding({ left: 18, right: 18 })
            .width('100%')

          // AR missions grid
          Column({ space: 14 }) {
            this.sectionLabel('AR MISSIONS', '04', 'N=' + this.missions().length)
            Grid() {
              ForEach(this.missions(), (m: Mission): void => {
                GridItem() { this.missionTile(m) }
              }, (m: Mission): string => m.title)
            }
            .columnsTemplate('1fr 1fr')
            .rowsGap(12)
            .columnsGap(12)
            .height(316)
          }
          .padding({ left: 18, right: 18 })

          // Sky today
          Column({ space: 14 }) {
            this.sectionLabel('SKY TODAY', '05', formatDateMD(new Date(this.now)))
            if (this.sunError && !this.loadingSun) {
              this.errorStrip('没能获取日出日落时间')
            } else if (this.sun !== null) {
              Column() {
                this.skyRow('SUNRISE', '日出', this.sun.sunriseISO, 'gold', false)
                this.skyRow('SUNSET', '日落', this.sun.sunsetISO, 'coral', false)
                this.issFootprintRow(true)
              }
              .width('100%')
              .backgroundColor(NebulaColor.surface)
              .border({ width: 1, color: NebulaColor.border })
              .borderRadius(NebulaRadius.tight)
            } else {
              this.errorStrip('正在连接 sunrise-sunset.org…')
            }
          }
          .padding({ left: 18, right: 18 })

          // Upcoming launches
          Column({ space: 14 }) {
            this.sectionLabel('UPCOMING', '06',
              this.launches.length > 0 ? 'N=' + this.launches.length : (this.loadingLaunches ? '...' : 'OFF'))
            if (this.launchesError && !this.loadingLaunches) {
              this.errorStrip('没能获取发射时间表')
            } else if (this.launches.length === 0) {
              this.errorStrip('正在连接 ll.thespacedevs.com…')
            } else {
              Column() {
                ForEach(this.launches, (l: LaunchInfo, i: number): void => {
                  this.launchRow(l, i === this.launches.length - 1)
                }, (l: LaunchInfo): string => l.id)
              }
              .width('100%')
              .backgroundColor(NebulaColor.surface)
              .border({ width: 1, color: NebulaColor.border })
              .borderRadius(NebulaRadius.tight)
            }
            Text('SOURCE · LAUNCH LIBRARY 2 (THESPACEDEVS.COM)')
              .fontSize('9vp')
              .fontFamily(NebulaFontFace.mono)
              .fontColor(NebulaColor.textTertiary)
              .letterSpacing(1.4)
              .margin({ top: 4 })
          }
          .padding({ left: 18, right: 18 })

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
