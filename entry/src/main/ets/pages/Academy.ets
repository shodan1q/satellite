/**
 * Academy — catalog of rockets & satellites with an immersive AR detail view.
 * List is airy: big title, two-column tile grid. Detail opens into the same AR
 * workshop shell so the exploded model lives in real space.
 */

import {
  NebulaColor, NebulaFont, NebulaFontFace, NebulaRadius, NebulaSpace
} from '../theme/Theme';
import { NebulaIcon, IconName } from '../components/NebulaIcon';
import { CameraPreview } from '../components/CameraPreview';
import {
  ArBridge, ArViewer,
  ArReadyEvent, ArSelectEvent, ArAnchorEvent, ArModelEvent
} from '../components/ArViewer';
import { Catalog, PartSpec, RocketBlueprint, SatelliteBlueprint, ExplodeOffset, SpecLine } from '../data/Catalog';
import { NebulaSync } from '../services/NebulaSync';
import { DeviceMotion, MotionSample, ThrottledMotionListener } from '../services/DeviceMotion';
import { AppBus } from '../services/AppBus';
import { UserStats } from '../services/UserStats';
import { ModelStore } from '../services/ModelStore';
import { Narrator } from '../services/Narrator';
import { CardContent, CardSpec, ShareCard, ShareResult } from '../services/ShareCard';
import { HandoffPayload } from '../services/Handoff';
import { ViewerOffset, ViewerPart } from '../components/ViewerPart';
/* UI helpers are reached via this.getUIContext() (newer, non-deprecated API). */
import common from '@ohos.app.ability.common';
import image from '@ohos.multimedia.image';
import fs from '@ohos.file.fs';
import fileUri from '@ohos.file.fileuri';

class Entry {
  id: string = '';
  display: string = '';
  caption: string = '';
  summary: string = '';
  tags: string[] = [];
  tone: string = 'brand';
  kind: string = 'rocket';
  hero: string = '#2E6BF6';
  /** Rawfile-relative thumbnail name, e.g. 'thumbs/saturn-v.png'.  */
  thumb: string = '';
}

function mkModelFailEvent(url: string): ArModelEvent {
  const e = new ArModelEvent();
  e.ok = false; e.url = url; e.reason = 'bytes-unavailable';
  return e;
}

@Component
export struct AcademyPage {
  @State private tab: number = 0;
  @State private openId: string = '';
  @State private selectedPart: string = '';
  @State private explode: number = 0;
  @State private anchored: boolean = false;
  @State private modelLoading: boolean = false;
  @State private modelFailed: boolean = false;
  @State private modelCredit: string = '';
  @State private pendingCard: CardContent | null = null;
  @State private pendingCardAccent: string = '#2E6BF6';

  private bridge: ArBridge = new ArBridge();
  private unsubMotion: () => void = (): void => {};
  private unsubBus: () => void = (): void => {};

  aboutToAppear(): void {
    const throttled = new ThrottledMotionListener(
      (s: MotionSample): void => { this.bridge.setOrientation(s.yaw, s.pitch, s.roll); },
      33,
      0.3
    );
    this.unsubMotion = DeviceMotion.instance.subscribe(
      (s: MotionSample): void => throttled.handle(s)
    );
    this.unsubBus = AppBus.instance.onAcademyRequest((id: string): void => {
      for (const e of this.entries()) {
        if (e.id === id) { this.openEntry(e); return; }
      }
      // try the other tab
      this.tab = this.tab === 0 ? 1 : 0;
      for (const e of this.entries()) {
        if (e.id === id) { this.openEntry(e); return; }
      }
    });
  }
  aboutToDisappear(): void {
    this.unsubMotion();
    this.unsubBus();
  }

  /** Convert the .glb model name in the Catalog to the matching thumbnail
   *  file we downloaded from NASA 3D Resources. Falls back to empty so
   *  the card can render a dark gradient placeholder if no image exists. */
  private thumbFor(modelUrl: string): string {
    if (modelUrl.length === 0) return '';
    return 'thumbs/' + modelUrl.replace('.glb', '.png');
  }

  private rocketList(): Entry[] {
    const out: Entry[] = [];
    for (const r of Catalog.rockets()) {
      const e = new Entry();
      e.id = r.id; e.display = r.display; e.caption = r.agency;
      e.summary = r.summary; e.tags = r.tags;
      e.tone = 'brand'; e.kind = 'rocket'; e.hero = r.hero;
      e.thumb = this.thumbFor(r.modelUrl);
      out.push(e);
    }
    return out;
  }

  private satList(): Entry[] {
    const out: Entry[] = [];
    for (const s of Catalog.satellites()) {
      const e = new Entry();
      e.id = s.id; e.display = s.display; e.caption = s.orbit;
      e.summary = s.summary; e.tags = s.tags;
      e.tone = 'violet'; e.kind = 'satellite'; e.hero = '#7F5CFF';
      e.thumb = this.thumbFor(s.modelUrl);
      out.push(e);
    }
    return out;
  }

  private entries(): Entry[] { return this.tab === 0 ? this.rocketList() : this.satList(); }

  private partsForOpen(): PartSpec[] {
    if (this.openId.length === 0) return [];
    const r = Catalog.rocketById(this.openId);
    if (r !== null) return r.parts;
    const s = Catalog.satelliteById(this.openId);
    return s !== null ? s.parts : [];
  }

  private offsetsForOpen(): ExplodeOffset[] {
    if (this.openId.length === 0) return [];
    const r = Catalog.rocketById(this.openId);
    if (r !== null) return r.explodeOffsets;
    const s = Catalog.satelliteById(this.openId);
    return s !== null ? s.explodeOffsets : [];
  }

  private openedEntry(): Entry | null {
    if (this.openId.length === 0) return null;
    for (const e of this.entries()) { if (e.id === this.openId) return e; }
    return null;
  }

  private selectedPartSpec(): PartSpec | null {
    if (this.selectedPart.length === 0) return null;
    for (const p of this.partsForOpen()) { if (p.id === this.selectedPart) return p; }
    return null;
  }

  private openEntry(entry: Entry): void {
    this.openId = entry.id;
    this.selectedPart = '';
    this.explode = 0;
    this.anchored = false;

    const parts = this.partsForOpen();
    const offs = this.offsetsForOpen();
    const vps: ViewerPart[] = [];
    for (const p of parts) {
      const v = new ViewerPart();
      v.id = p.id; v.kind = p.kind; v.display = p.display;
      v.color = p.color; v.accent = p.accent;
      v.height = p.height; v.radiusTop = p.radiusTop; v.radiusBottom = p.radiusBottom;
      const o = new ViewerOffset();
      let found = false;
      for (const x of offs) {
        if (x.partId === p.id) { o.x = x.dx; o.y = x.dy; o.z = x.dz; found = true; break; }
      }
      if (!found) { o.x = 0; o.y = 0.2; o.z = 0; }
      v.explodeOffset = o;
      vps.push(v);
    }
    const rocket = entry.kind === 'rocket' ? Catalog.rocketById(entry.id) : null;
    const satellite = entry.kind === 'rocket' ? null : Catalog.satelliteById(entry.id);
    const modelFile: string = rocket !== null ? rocket.modelUrl
                            : satellite !== null ? satellite.modelUrl : '';
    this.modelCredit = rocket !== null ? rocket.modelCredit
                     : satellite !== null ? satellite.modelCredit : '';

    this.modelFailed = false;
    if (modelFile.length > 0) {
      this.modelLoading = true;
      // JS side calls nebula.getModelBytes(filename) via javaScriptProxy and
      // parses the returned base64 directly — no URL fetch, no cache issues.
      this.bridge.loadBundledModel(modelFile, 3.5);
    } else {
      this.modelLoading = false;
      this.bridge.setParts(vps);
    }
    this.bridge.setExplode(0);
    this.bridge.recenterCamera();

    if (entry.kind === 'rocket') NebulaSync.instance.updateRocket(entry.id);
    else NebulaSync.instance.updateSatellite(entry.id);

    UserStats.instance.recordOpenAcademy(entry.display).then((): void => {});
  }

  private onModelResult(e: ArModelEvent): void {
    this.modelLoading = false;
    if (e.ok) {
      this.modelFailed = false;
      return;
    }
    // GLB failed. We intentionally do NOT fall back to procedural
    // primitives any more — those primitives belong to the blueprint's
    // original schematic geometry, which is unrelated to the real mesh
    // the card's thumbnail promised. Rendering them used to produce
    // the "thumb shows Atlas, scene shows Falcon-9 rectangles" mismatch.
    // Instead we flip `modelFailed` so the overlay tells the user the
    // real reason (logged by ArViewer) and offers a retry.
    this.modelFailed = true;
    this.bridge.clearModel();
    this.bridge.recenterCamera();
    this.toast('3D 模型加载失败：' + (e.reason.length > 0 ? e.reason : '未知'));
  }

  private close(): void {
    this.openId = '';
    this.selectedPart = '';
    this.anchored = false;
    this.modelLoading = false;
    this.modelFailed = false;
    Narrator.instance.stop();
    // The detail view's Web is about to unmount. Swap in a fresh ArBridge
    // so the next open gets a live WebviewController + clean pending state.
    this.bridge = new ArBridge();
  }

  private onReady(_e: ArReadyEvent): void {
    if (this.openId.length > 0) {
      const o = this.openedEntry();
      if (o !== null) this.openEntry(o);
    } else {
      this.bridge.recenterCamera();
    }
  }

  private onSelect(e: ArSelectEvent): void {
    this.selectedPart = e.partId ?? '';
  }
  private onAnchor(_e: ArAnchorEvent): void { this.anchored = true; }

  private toast(message: string): void {
    try {
      this.getUIContext().getPromptAction().showToast({ message, duration: 1800 });
    } catch (e) { /* ignore */ }
  }

  private hostContext(): common.UIAbilityContext | null {
    try {
      const host = this.getUIContext().getHostContext();
      return host !== undefined && host !== null ? host as common.UIAbilityContext : null;
    } catch (e) {
      return null;
    }
  }

  private narrationText(): string {
    const ps = this.selectedPartSpec();
    if (ps === null) return '';
    const entry = this.openedEntry();
    const header = entry !== null ? `${entry.display}·${ps.display}。` : `${ps.display}。`;
    return header + ps.fullDesc;
  }

  private onNarrate(): void {
    const text = this.narrationText();
    if (text.length === 0) {
      this.toast('没有可朗读的内容');
      return;
    }
    this.toast('小艺正在朗读…');
    Narrator.instance.speak(text).then((ok: boolean): void => {
      if (!ok) {
        const why = Narrator.instance.lastFailure();
        this.toast(why.length > 0 ? '朗读失败 · ' + why : '本机语音引擎不可用');
      }
    });
  }

  private buildCard(): CardContent | null {
    const ps = this.selectedPartSpec();
    const entry = this.openedEntry();
    if (ps === null || entry === null) return null;
    const card = new CardContent();
    card.title = `${entry.display} · ${ps.display}`;
    card.kind = ps.kind.toUpperCase();
    card.description = ps.fullDesc;
    const specs: CardSpec[] = [];
    for (const s of ps.specs) {
      const c = new CardSpec();
      c.key = s.key; c.value = s.value;
      specs.push(c);
    }
    card.specs = specs;
    return card;
  }

  private onShare(): void {
    const card = this.buildCard();
    if (card === null) return;
    const entry = this.openedEntry();
    if (entry !== null) {
      if (entry.kind === 'rocket') NebulaSync.instance.updateRocket(entry.id);
      else NebulaSync.instance.updateSatellite(entry.id);
    }
    const ctx: common.UIAbilityContext | null = this.hostContext();
    if (ctx === null) { this.toast('无法获取应用上下文'); return; }

    // Kick the Builder off on next frame so @State writes have flushed,
    // then snapshot → save → share. A plain-text fallback covers the
    // case where componentSnapshot or image encoding fails.
    this.pendingCard = card;
    this.pendingCardAccent = entry !== null ? entry.hero : '#2E6BF6';
    this.toast('正在生成卡片…');
    const ctxFinal: common.UIAbilityContext = ctx;
    setTimeout((): void => {
      this.renderAndShareCard(ctxFinal, card);
    }, 80);
  }

  private async renderAndShareCard(ctx: common.UIAbilityContext, card: CardContent): Promise<void> {
    let imageUri: string = '';
    try {
      const snapshot = this.getUIContext().getComponentSnapshot();
      const pix = await snapshot.createFromBuilder(
        (): void => { this.cardBuilder(); },
        240,
        true,
        { scale: 2 }
      );
      imageUri = await this.savePixelMapAsJpg(ctx, pix);
      try { pix.release(); } catch (e) { /* ignore */ }
    } catch (e) {
      this.pendingCard = null;
      // Snapshot failed — fall back to text share so the user still
      // gets something useful in the share sheet.
      ShareCard.share(ctx, card).then((r: ShareResult): void => {
        if (r.channel === 'system-share') this.toast('已打开分享面板');
        else if (r.channel === 'clipboard') this.toast('卡片已复制到剪贴板');
        else this.toast('生成卡片失败');
      });
      return;
    }
    this.pendingCard = null;
    if (imageUri.length === 0) { this.toast('保存卡片图片失败'); return; }
    const r = await ShareCard.shareImage(ctx, card, imageUri);
    if (r.channel === 'system-share') this.toast('已打开隔空传送 / 分享面板');
    else this.toast('生成卡片失败，请重试');
  }

  private async savePixelMapAsJpg(
    ctx: common.UIAbilityContext,
    pix: image.PixelMap
  ): Promise<string> {
    const path = ctx.cacheDir + '/nebula-card.jpg';
    try { fs.unlinkSync(path); } catch (e) { /* not existing is fine */ }
    let file: fs.File | null = null;
    try {
      try {
        file = fs.openSync(path, fs.OpenMode.CREATE | fs.OpenMode.READ_WRITE);
      } catch (e) {
        return '';
      }
      const packer = image.createImagePacker();
      const opts: image.PackingOption = { format: 'image/jpeg', quality: 92 };
      try {
        await packer.packToFile(pix, file.fd, opts);
      } catch (e) { /* pack failed — caller falls back to text */ }
      try { await packer.release(); } catch (e) { /* ignore */ }
    } finally {
      if (file !== null) {
        try { fs.closeSync(file); } catch (e) { /* ignore */ }
      }
    }
    try {
      return fileUri.getUriFromPath(path);
    } catch (e) {
      return '';
    }
  }

  @Builder cardBuilder() {
    if (this.pendingCard !== null) {
      // Outer frame — gives the card breathing room inside the snapshot so
      // the 20-radius rounded corners render with their anti-aliasing and
      // the shadow doesn't get clipped.
      Column() {
        Column() {
          // ----- Hero -----
          Stack({ alignContent: Alignment.TopStart }) {
            // Gradient backdrop
            Column()
              .width('100%').height('100%')
              .linearGradient({
                angle: 150,
                colors: [
                  [this.pendingCardAccent, 0],
                  [this.pendingCardAccent + 'D9', 0.55],
                  ['#0C1B33', 1]
                ]
              })
            // Decorative orbit ring
            Column()
              .width(180).height(180)
              .borderRadius(999)
              .border({ width: 1.2, color: '#FFFFFF26' })
              .position({ x: 200, y: -60 })
            Column()
              .width(120).height(120)
              .borderRadius(999)
              .border({ width: 1, color: '#FFFFFF1F' })
              .position({ x: 230, y: -20 })
            // Constellation dots
            Column().width(4).height(4).borderRadius(2).backgroundColor('#27272AEE').position({ x: 28, y: 24 })
            Column().width(3).height(3).borderRadius(1.5).backgroundColor('#FFFFFF99').position({ x: 56, y: 40 })
            Column().width(2).height(2).borderRadius(1).backgroundColor('#52525B').position({ x: 80, y: 28 })
            Column().width(3).height(3).borderRadius(1.5).backgroundColor('#FFFFFFAA').position({ x: 42, y: 72 })

            // Content
            Column({ space: 14 }) {
              Row() {
                Text(this.pendingCard!.kind)
                  .fontSize(10)
                  .fontColor('#FFFFFF')
                  .fontWeight(FontWeight.Bold)
                  .letterSpacing(1.2)
                  .padding({ left: 10, right: 10, top: 4, bottom: 4 })
                  .backgroundColor('#FFFFFF2B')
                  .borderRadius(999)
                Blank()
                Text('N · CORE')
                  .fontSize(9)
                  .fontColor('#FFFFFFBB')
                  .fontWeight(FontWeight.Bold)
                  .letterSpacing(2)
              }
              .width('100%')

              Column({ space: 6 }) {
                Text(this.pendingCard!.title)
                  .fontSize(22)
                  .fontColor('#FFFFFF')
                  .fontWeight(FontWeight.Bold)
                  .lineHeight(30)
                  .maxLines(2)
                  .textOverflow({ overflow: TextOverflow.Ellipsis })
                Row({ space: 8 }) {
                  Column().width(14).height(2).backgroundColor('#18181BE6').borderRadius(2)
                  Text('AR 太空工坊 · 知识卡片')
                    .fontSize(10)
                    .fontColor('#27272AEE')
                    .fontWeight(FontWeight.Medium)
                    .letterSpacing(1)
                }
              }
              .alignItems(HorizontalAlign.Start)
              .width('100%')
            }
            .width('100%')
            .padding({ left: 20, right: 20, top: 22, bottom: 22 })
            .alignItems(HorizontalAlign.Start)
          }
          .width('100%')
          .height(150)
          .clip(true)

          // ----- Body -----
          Column({ space: 14 }) {
            Text(this.pendingCard!.description)
              .fontSize(12)
              .fontColor('#3E4D6B')
              .lineHeight(20)
              .maxLines(6)
              .textOverflow({ overflow: TextOverflow.Ellipsis })

            if (this.pendingCard!.specs.length > 0) {
              Column({ space: 6 }) {
                ForEach(this.pendingCard!.specs, (s: CardSpec): void => {
                  Row() {
                    Column()
                      .width(3).height(14)
                      .backgroundColor(this.pendingCardAccent)
                      .borderRadius(2)
                      .margin({ right: 10 })
                    Text(s.key)
                      .fontSize(11)
                      .fontColor('#6B7588')
                      .fontWeight(FontWeight.Regular)
                    Blank()
                    Text(s.value)
                      .fontSize(12)
                      .fontColor('#0C1B33')
                      .fontWeight(FontWeight.Bold)
                  }
                  .width('100%')
                  .padding({ left: 10, right: 12, top: 9, bottom: 9 })
                  .backgroundColor('#F5F8FD')
                  .borderRadius(8)
                }, (s: CardSpec): string => s.key)
              }
              .width('100%')
            }
          }
          .width('100%')
          .padding({ left: 20, right: 20, top: 18, bottom: 16 })
          .alignItems(HorizontalAlign.Start)

          // ----- Divider -----
          Column()
            .width('100%').height(1)
            .backgroundColor('#EEF1F7')

          // ----- Footer -----
          Row() {
            Column({ space: 2 }) {
              Row({ space: 6 }) {
                Column()
                  .width(8).height(8)
                  .borderRadius(2)
                  .backgroundColor(this.pendingCardAccent)
                Text('喂！星')
                  .fontSize(11)
                  .fontColor('#0C1B33')
                  .fontWeight(FontWeight.Bold)
                  .letterSpacing(0.5)
              }
              Text(this.pendingCard!.footer)
                .fontSize(9)
                .fontColor('#94A0B8')
            }
            .alignItems(HorizontalAlign.Start)
            Blank()
            Text(this.cardDateLabel())
              .fontSize(9)
              .fontColor('#94A0B8')
              .letterSpacing(0.6)
          }
          .width('100%')
          .padding({ left: 20, right: 20, top: 14, bottom: 16 })
        }
        .width(320)
        .backgroundColor('#FFFFFF')
        .borderRadius(16)
        .clip(true)
        .border({ width: 1, color: '#EEF1F7' })
      }
      .padding(12)
      .backgroundColor('#F0F3F9')
    }
  }

  private cardDateLabel(): string {
    const d = new Date();
    const pad = (n: number): string => n < 10 ? ('0' + n) : ('' + n);
    return `${d.getFullYear()}·${pad(d.getMonth() + 1)}·${pad(d.getDate())}`;
  }

  // ---------- list view ----------

  /**
   * Editorial card: full-bleed NASA render at the top with a hard
   * gradient fade to the dark card body, so the title reads over
   * the image. Type lives in the lower third — caption (kind + era)
   * tracked out, display name in large Inter Medium, primary tag
   * as a hairline chip. No pastel tints; the only colour comes
   * from the real mesh render itself.
   */
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
      Text('喂！星')
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

  @Builder catalogTile(entry: Entry) {
    Column() {
      // Hero image + thin bottom fade for legibility. Tight 4px corner
      // to match the Devices-console aesthetic — no more rounded iOS
      // tile feel. The colored type chip also becomes a 3px-corner
      // mono badge, not a pill.
      Stack({ alignContent: Alignment.TopStart }) {
        if (entry.thumb.length > 0) {
          Image($rawfile(entry.thumb))
            .width('100%').height(152)
            .objectFit(ImageFit.Cover)
        } else {
          Column()
            .width('100%').height(152)
            .linearGradient({
              angle: 135,
              colors: [[entry.hero + '40', 0], ['#121214', 1]]
            })
        }
        Column()
          .width('100%').height(152)
          .linearGradient({
            angle: 180,
            colors: [
              ['#0B0B0E00', 0.0],
              ['#0B0B0E00', 0.6],
              ['#0B0B0ECC', 1.0]
            ]
          })
        // Kind badge — mono tracked, terminal style
        Text(entry.kind === 'rocket' ? 'ROCKET' : 'SATELLITE')
          .fontSize('9vp')
          .fontFamily(NebulaFontFace.mono)
          .fontColor(entry.kind === 'rocket' ? NebulaColor.brand : NebulaColor.accentViolet)
          .letterSpacing(1.8)
          .padding({ top: 3, bottom: 3, left: 7, right: 7 })
          .border({
            width: 1,
            color: entry.kind === 'rocket' ? NebulaColor.brand : NebulaColor.accentViolet
          })
          .borderRadius(2)
          .backgroundColor('#0B0B0ECC')
          .position({ left: 10, top: 10 })
      }
      .width('100%').height(152)

      // Body with hairline separator above
      Column({ space: 6 }) {
        Text(entry.caption.toUpperCase())
          .fontSize('9vp')
          .fontFamily(NebulaFontFace.mono)
          .letterSpacing(1.8)
          .fontColor(NebulaColor.textTertiary)
          .maxLines(1)
        Text(entry.display)
          .fontSize('16vp')
          .lineHeight('20vp')
          .fontFamily(NebulaFontFace.serif)
          .fontColor(NebulaColor.textPrimary)
          .maxLines(1)
          .textOverflow({ overflow: TextOverflow.Ellipsis })
        if (entry.tags.length > 0) {
          Row({ space: 4 }) {
            Text(entry.tags[0])
              .fontSize('9vp')
              .fontFamily(NebulaFontFace.mono)
              .fontColor(NebulaColor.textSecondary)
              .letterSpacing(1.2)
              .padding({ top: 2, bottom: 2, left: 6, right: 6 })
              .border({ width: 1, color: NebulaColor.border })
              .borderRadius(2)
            if (entry.tags.length > 1) {
              Text('+' + (entry.tags.length - 1))
                .fontSize('9vp')
                .fontFamily(NebulaFontFace.mono)
                .fontColor(NebulaColor.textTertiary)
                .letterSpacing(1.2)
                .padding({ top: 2, bottom: 2, left: 5, right: 5 })
            }
          }
          .margin({ top: 2 })
        }
      }
      .alignItems(HorizontalAlign.Start)
      .width('100%')
      .padding({ left: 12, right: 12, top: 12, bottom: 14 })
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
    .onClick((): void => this.openEntry(entry))
  }

  @Builder segmentPill(label: string, idx: number) {
    Text(label)
      .fontSize('11vp')
      .fontFamily(NebulaFontFace.mono)
      .fontColor(this.tab === idx ? '#0B0B0E' : NebulaColor.textTertiary)
      .fontWeight(this.tab === idx ? FontWeight.Medium : FontWeight.Regular)
      .letterSpacing(2.0)
      .padding({ top: 10, bottom: 10, left: 16, right: 16 })
      .backgroundColor(this.tab === idx ? NebulaColor.accentViolet : Color.Transparent)
      .border({
        width: { left: idx === 0 ? 0 : 1 },
        color: NebulaColor.border,
        style: BorderStyle.Solid
      })
      .layoutWeight(1)
      .textAlign(TextAlign.Center)
      .onClick((): void => { this.tab = idx; this.close(); })
  }

  @Builder listView() {
    Column() {
      this.statusBar('ACADEMY')
      Scroll() {
        Column({ space: 26 }) {
          // page hero
          Column({ space: 14 }) {
            Row({ space: 10 }) {
              Column().width(24).height(1).backgroundColor(NebulaColor.border)
              Text('02 / ACADEMY')
                .fontSize('10vp')
                .fontFamily(NebulaFontFace.mono)
                .fontColor(NebulaColor.textTertiary)
                .letterSpacing(2.2)
              Column().layoutWeight(1).height(1).backgroundColor(NebulaColor.border)
            }
            Text('科普')
              .fontSize('40vp')
              .lineHeight('44vp')
              .fontFamily(NebulaFontFace.serif)
              .fontColor(NebulaColor.textPrimary)
              .letterSpacing(-0.8)
          }
          .alignItems(HorizontalAlign.Start)
          .padding({ left: 18, right: 18, top: 20 })
          .width('100%')

          // rockets/satellites terminal-style segmented control
          Column({ space: 10 }) {
            Row({ space: 10 }) {
              Column().width(24).height(1).backgroundColor(NebulaColor.border)
              Text('03 / CATALOG')
                .fontSize('10vp')
                .fontFamily(NebulaFontFace.mono)
                .fontColor(NebulaColor.textTertiary)
                .letterSpacing(2.2)
              Column().layoutWeight(1).height(1).backgroundColor(NebulaColor.border)
              Text('N=' + this.entries().length.toString())
                .fontSize('10vp')
                .fontFamily(NebulaFontFace.mono)
                .fontColor(NebulaColor.textTertiary)
                .letterSpacing(1.6)
            }
            .width('100%')
            Row() {
              this.segmentPill('ROCKET', 0)
              this.segmentPill('SATELLITE', 1)
            }
            .width('100%')
            .border({ width: 1, color: NebulaColor.border })
            .borderRadius(NebulaRadius.tight)
            .clip(true)
          }
          .padding({ left: 18, right: 18 })
          .width('100%')

          Grid() {
            ForEach(this.entries(), (entry: Entry): void => {
              GridItem() { this.catalogTile(entry) }
            }, (entry: Entry): string => entry.id)
          }
          .columnsTemplate('1fr 1fr')
          .rowsGap(12)
          .columnsGap(12)
          .padding({ left: 18, right: 18 })
          // Each tile ≈ 152 hero + 14 hairline + 12+14 padding + ~56 text
          // + ~22 tag row = ~270vp when tagged. Plus 12vp rowsGap between
          // rows. Use 272/row which is slightly generous so the last
          // tile's tag row never clips inside the Grid itself.
          .height(Math.ceil(this.entries().length / 2) * 272)

          // Trailing breathing room so the last grid row clears the
          // bottom Dock. Dock strip ≈ 60vp + 34vp safe-area filler +
          // buffer = ~160vp. Without this the bottom catalog tiles are
          // partially hidden behind the nav bar.
          Column().height(160)
        }
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

  // ---------- AR detail view ----------

  @Builder topBar(entry: Entry) {
    Row({ space: 10 }) {
      Row({ space: 8 }) {
        NebulaIcon({ name: IconName.Arrow, glyphSize: 12, color: NebulaColor.textPrimary, strokeW: 2 })
          .rotate({ angle: 180 })
        Text('BACK · CATALOG')
          .fontSize('10vp')
          .fontFamily(NebulaFontFace.mono)
          .fontColor(NebulaColor.textPrimary)
          .letterSpacing(2.0)
      }
      .padding({ top: 8, bottom: 8, left: 12, right: 14 })
      .backgroundColor('#0B0B0EE6')
      .border({ width: 1, color: NebulaColor.border })
      .borderRadius(NebulaRadius.tight)
      .backdropBlur(20)
      .onClick((): void => this.close())

      Blank()

      Column({ space: 2 }) {
        Text((entry.kind === 'rocket' ? 'ROCKET · ' : 'SATELLITE · ') + entry.caption.toUpperCase())
          .fontSize('9vp')
          .fontFamily(NebulaFontFace.mono)
          .fontColor(NebulaColor.textTertiary)
          .letterSpacing(1.6)
          .maxLines(1)
        Text(entry.display)
          .fontSize('14vp')
          .fontFamily(NebulaFontFace.serif)
          .fontColor(NebulaColor.textPrimary)
      }
      .alignItems(HorizontalAlign.End)
      .padding({ top: 7, bottom: 7, left: 14, right: 14 })
      .backgroundColor('#0B0B0EE6')
      .border({ width: 1, color: NebulaColor.border })
      .borderRadius(NebulaRadius.tight)
      .backdropBlur(20)
    }
    .width('100%')
    .padding({ left: NebulaSpace.md, right: NebulaSpace.md })
  }

  @Builder explodeBar() {
    Row({ space: 12 }) {
      Text('EXPLODE')
        .fontSize('9vp')
        .fontFamily(NebulaFontFace.mono)
        .fontColor(NebulaColor.textTertiary)
        .letterSpacing(2.0)
      Slider({
        value: this.explode * 100,
        min: 0, max: 100, style: SliderStyle.InSet
      })
        .blockColor(NebulaColor.textPrimary)
        .selectedColor(NebulaColor.brand)
        .trackColor(NebulaColor.border)
        .layoutWeight(1)
        .onChange((v: number): void => { this.explode = v / 100; this.bridge.setExplode(this.explode); })
      Text(Math.round(this.explode * 100).toString().padStart(3, '0') + '%')
        .fontSize('11vp')
        .fontFamily(NebulaFontFace.mono)
        .fontColor(NebulaColor.textPrimary)
        .fontWeight(FontWeight.Medium)
        .width(44)
        .textAlign(TextAlign.End)
    }
    .width('100%')
    .padding({ top: 11, bottom: 11, left: 14, right: 14 })
    .backgroundColor('#0B0B0EE6')
    .border({ width: 1, color: NebulaColor.border })
    .borderRadius(NebulaRadius.tight)
    .backdropBlur(20)
  }

  @Builder partStrip() {
    Scroll() {
      Row({ space: 6 }) {
        ForEach(this.partsForOpen(), (p: PartSpec): void => {
          Row({ space: 8 }) {
            Column()
              .width(2).height(16)
              .backgroundColor(p.accent.length > 0 ? p.accent : NebulaColor.brand)
            Text(p.display)
              .fontSize('11vp')
              .fontFamily(NebulaFontFace.mono)
              .letterSpacing(1.2)
              .fontColor(this.selectedPart === p.id ? '#0B0B0E' : NebulaColor.textSecondary)
              .fontWeight(this.selectedPart === p.id ? FontWeight.Medium : FontWeight.Regular)
          }
          .padding({ top: 7, bottom: 7, left: 10, right: 12 })
          .backgroundColor(this.selectedPart === p.id
            ? (p.accent.length > 0 ? p.accent : NebulaColor.brand)
            : '#0B0B0EE6')
          .border({ width: 1, color: this.selectedPart === p.id
            ? (p.accent.length > 0 ? p.accent : NebulaColor.brand)
            : NebulaColor.border })
          .borderRadius(NebulaRadius.tight)
          .backdropBlur(20)
          .onClick((): void => {
            this.selectedPart = p.id;
            this.bridge.selectPart(p.id);
          })
        }, (p: PartSpec): string => p.id)
      }
      .padding({ left: NebulaSpace.md, right: NebulaSpace.md })
    }
    .scrollable(ScrollDirection.Horizontal)
    .scrollBar(BarState.Off)
    .width('100%')
  }

  @Builder partSheet() {
    if (this.selectedPartSpec() !== null) {
      Column() {
        Column()
          .width(36).height(3)
          .backgroundColor(NebulaColor.border)
          .margin({ top: 10, bottom: 14 })
        Column({ space: 14 }) {
          // Rail row: kind tag + close
          Row() {
            Text((this.selectedPartSpec()!.kind || 'PART').toUpperCase())
              .fontSize('10vp')
              .fontFamily(NebulaFontFace.mono)
              .fontColor(NebulaColor.brand)
              .letterSpacing(2.0)
              .padding({ top: 4, bottom: 4, left: 8, right: 8 })
              .border({ width: 1, color: NebulaColor.brand })
              .borderRadius(2)
            Blank()
            Row() {
              NebulaIcon({ name: IconName.Close, glyphSize: 12, color: NebulaColor.textPrimary, strokeW: 2 })
            }
            .padding(8)
            .border({ width: 1, color: NebulaColor.border })
            .borderRadius(NebulaRadius.tight)
            .onClick((): void => {
              this.selectedPart = '';
              this.bridge.selectPart(null);
            })
          }
          .width('100%')

          Text(this.selectedPartSpec()!.display)
            .fontSize('26vp')
            .lineHeight('30vp')
            .fontFamily(NebulaFontFace.serif)
            .fontColor(NebulaColor.textPrimary)
            .letterSpacing(-0.5)

          Text(this.selectedPartSpec()!.fullDesc)
            .fontSize('13vp')
            .lineHeight('20vp')
            .fontFamily(NebulaFontFace.sans)
            .fontColor(NebulaColor.textSecondary)

          // Specs — terminal grid style
          if (this.selectedPartSpec()!.specs.length > 0) {
            Column() {
              ForEach(this.selectedPartSpec()!.specs, (s: SpecLine, i: number): void => {
                Row({ space: 12 }) {
                  Text(s.key.toUpperCase())
                    .fontSize('9vp')
                    .fontFamily(NebulaFontFace.mono)
                    .fontColor(NebulaColor.textTertiary)
                    .letterSpacing(1.6)
                    .width(90)
                  Text(s.value)
                    .fontSize('12vp')
                    .fontFamily(NebulaFontFace.mono)
                    .fontColor(NebulaColor.textPrimary)
                    .fontWeight(FontWeight.Medium)
                    .layoutWeight(1)
                }
                .width('100%')
                .padding({ top: 10, bottom: 10, left: 12, right: 12 })
                .border({
                  width: { top: 0, bottom: i === this.selectedPartSpec()!.specs.length - 1 ? 0 : 1, left: 0, right: 0 },
                  color: NebulaColor.border,
                  style: BorderStyle.Solid
                })
              }, (s: SpecLine): string => s.key)
            }
            .width('100%')
            .border({ width: 1, color: NebulaColor.border })
            .borderRadius(NebulaRadius.tight)
          }

          // Action buttons — terminal style
          Row({ space: 8 }) {
            Row({ space: 6 }) {
              NebulaIcon({ name: IconName.Sparkle, glyphSize: 12, color: '#FFFFFF', strokeW: 2 })
              Text('NARRATE')
                .fontSize('10vp')
                .fontFamily(NebulaFontFace.mono)
                .fontColor('#FFFFFF')
                .letterSpacing(2.0)
            }
            .padding({ top: 10, bottom: 10, left: 14, right: 14 })
            .backgroundColor(NebulaColor.brand)
            .borderRadius(NebulaRadius.tight)
            .onClick((): void => this.onNarrate())

            Row({ space: 6 }) {
              NebulaIcon({ name: IconName.Share, glyphSize: 12, color: NebulaColor.brand, strokeW: 2 })
              Text('SHARE CARD')
                .fontSize('10vp')
                .fontFamily(NebulaFontFace.mono)
                .fontColor(NebulaColor.brand)
                .letterSpacing(2.0)
            }
            .padding({ top: 10, bottom: 10, left: 14, right: 14 })
            .border({ width: 1, color: NebulaColor.brand })
            .borderRadius(NebulaRadius.tight)
            .onClick((): void => this.onShare())

            Row({ space: 6 }) {
              NebulaIcon({ name: IconName.Transfer, glyphSize: 12, color: NebulaColor.textPrimary, strokeW: 2 })
              Text('HANDOFF')
                .fontSize('10vp')
                .fontFamily(NebulaFontFace.mono)
                .fontColor(NebulaColor.textPrimary)
                .letterSpacing(2.0)
            }
            .padding({ top: 10, bottom: 10, left: 14, right: 14 })
            .border({ width: 1, color: NebulaColor.border })
            .borderRadius(NebulaRadius.tight)
            .onClick((): void => { AppBus.instance.openHandoffSheet(this.currentHandoffPayload()); })
          }
          .width('100%')
        }
        .alignItems(HorizontalAlign.Start)
        .padding({ left: 18, right: 18, bottom: 120 })
      }
      .width('100%')
      .backgroundColor('#0B0B0EF4')
      .border({
        width: { top: 1, bottom: 0, left: 0, right: 0 },
        color: NebulaColor.border,
        style: BorderStyle.Solid
      })
      .backdropBlur(40)
    }
  }

  @Builder detailView(entry: Entry) {
    Stack({ alignContent: Alignment.Top }) {
      CameraPreview()
      ArViewer({
        bridge: this.bridge,
        onReady: (e: ArReadyEvent): void => this.onReady(e),
        onSelect: (e: ArSelectEvent): void => this.onSelect(e),
        onAnchor: (e: ArAnchorEvent): void => this.onAnchor(e),
        onModel: (e: ArModelEvent): void => this.onModelResult(e)
      })

      if (this.modelLoading) {
        Column({ space: 12 }) {
          LoadingProgress()
            .color(NebulaColor.brand)
            .width(36).height(36)
          Text('LOADING 3D MODEL')
            .fontSize('10vp')
            .fontFamily(NebulaFontFace.mono)
            .fontColor(NebulaColor.textTertiary)
            .letterSpacing(2.2)
          Text(entry.display)
            .fontSize('16vp')
            .fontFamily(NebulaFontFace.serif)
            .fontColor(NebulaColor.textPrimary)
          Text('NASA 3D RESOURCES · ' + (entry.kind === 'rocket' ? 'ROCKET' : 'SATELLITE'))
            .fontSize('9vp')
            .fontFamily(NebulaFontFace.mono)
            .fontColor(NebulaColor.textTertiary)
            .letterSpacing(1.6)
        }
        .width('100%').height('100%')
        .justifyContent(FlexAlign.Center)
        .alignItems(HorizontalAlign.Center)
        .backgroundColor('#0B0B0EE6')
        .backdropBlur(20)
      }

      Column({ space: 14 }) {
        this.topBar(entry)
      }
      .width('100%')
      .padding({ top: 44 })

      // Bottom stack: explode slider + parts strip
      Column({ space: 10 }) {
        this.partStrip()
        Column() {
          this.explodeBar()
        }
        .padding({ left: NebulaSpace.md, right: NebulaSpace.md })
      }
      .width('100%')
      .position({ bottom: 112 })

      if (this.selectedPartSpec() !== null) {
        Column() { this.partSheet() }
          .width('100%')
          .position({ bottom: 0 })
      }
    }
    .width('100%')
    .height('100%')
    // Dark mission-control fallback — visible for the brief moment
    // between CameraPreview mount and the first camera frame, and
    // behind the translucent modelLoading overlay. Previously a pale
    // blue gradient which clashed with the dark instrument aesthetic.
    .backgroundColor(NebulaColor.bg)
  }

  private currentHandoffPayload(): HandoffPayload {
    const p = new HandoffPayload();
    p.kind = 'academy';
    const entry = this.openedEntry();
    if (entry !== null) {
      p.entryKind = entry.kind;
      if (entry.kind === 'rocket') p.rocketId = entry.id;
      else p.satelliteId = entry.id;
    }
    p.selectedPart = this.selectedPart;
    p.issuedAt = Date.now();
    return p;
  }

  build() {
    Stack() {
      if (this.openId.length > 0 && this.openedEntry() !== null) {
        this.detailView(this.openedEntry()!)
      } else {
        this.listView()
      }
      // HandoffSheet is hoisted to Index.ets so it renders above the
      // bottom Dock; Academy emits via AppBus.openHandoffSheet().
    }
    .width('100%')
    .height('100%')
  }
}
