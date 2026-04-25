import { router, window, font } from '@kit.ArkUI';
import { webview } from '@kit.ArkWeb';
import { hilog } from '@kit.PerformanceAnalysisKit';
import common from '@ohos.app.ability.common';

const SPLASH_TAG: string = 'NebulaSplash';
const SPLASH_DOMAIN: number = 0x0000;

/**
 * Two-stage editorial splash.
 *
 * Stage A (first ~2.2s) — weixin "PERSONAL SATELLITE / 个人卫星控制台"
 * console brand: the inverted Subtract mark + small uppercase eyebrow
 * + big Chinese title, left-aligned in the top-left corner exactly
 * like the console header in the reference screenshots.
 *
 * Stage B (cross-fade) — the stage-A chrome fades out and the
 * existing "喂！星" Instrument-Serif hero + hairline progress bar fade
 * in, running for another ~2s before routing to Index.
 *
 * The 3D splash.html scene stays mounted underneath both stages as a
 * shared full-bleed backdrop.
 */

@Entry
@Component
struct Splash {
  // Stage A (weixin brand) reveal flags
  @State private revealMark: boolean = false;
  @State private revealEyebrow: boolean = false;
  @State private revealChineseTitle: boolean = false;
  @State private revealStageAMeta: boolean = false;

  // Stage B (星·核) reveal flags — existing animation
  @State private stageA: boolean = true;          // true = weixin brand visible
  @State private revealTitle: boolean = false;
  @State private revealSub: boolean = false;
  @State private revealSlogan: boolean = false;
  @State private revealFooter: boolean = false;
  @State private progress: number = 0;
  @State private fadingOut: boolean = false;

  private webController: webview.WebviewController = new webview.WebviewController();
  private progressHandle: number = 0;
  private navigated: boolean = false;

  aboutToAppear(): void {
    hilog.info(SPLASH_DOMAIN, SPLASH_TAG, 'aboutToAppear start');
    this.registerFonts();

    // ---- Stage A: weixin-console brand reveal (0 – 2200ms) ----
    setTimeout((): void => { this.revealMark = true; }, 140);
    setTimeout((): void => { this.revealEyebrow = true; }, 380);
    setTimeout((): void => { this.revealChineseTitle = true; }, 640);
    setTimeout((): void => { this.revealStageAMeta = true; }, 1000);

    // ---- Cross-fade into Stage B (星·核) at 2200ms ----
    setTimeout((): void => { this.stageA = false; }, 2200);
    setTimeout((): void => { this.revealTitle = true; }, 2500);
    setTimeout((): void => { this.revealSub = true; }, 2850);
    setTimeout((): void => { this.revealSlogan = true; }, 3150);
    setTimeout((): void => { this.revealFooter = true; }, 3450);

    // Progress bar animates during stage B (~2000ms of it).
    const totalMs: number = 2000;
    const stepMs: number = 24;
    let elapsed: number = 0;
    setTimeout((): void => {
      this.progressHandle = setInterval((): void => {
        elapsed += stepMs;
        this.progress = Math.min(1, elapsed / totalMs);
        if (this.progress >= 1) {
          clearInterval(this.progressHandle);
          this.progressHandle = 0;
          this.scheduleNavigate();
        }
      }, stepMs);
    }, 2500);
  }

  aboutToDisappear(): void {
    if (this.progressHandle !== 0) {
      clearInterval(this.progressHandle);
      this.progressHandle = 0;
    }
  }

  private registerFonts(): void {
    try {
      font.registerFont({ familyName: 'Inter', familySrc: '/fonts/Inter-Variable.ttf' });
      font.registerFont({ familyName: 'Space Grotesk', familySrc: '/fonts/SpaceGrotesk.ttf' });
      font.registerFont({ familyName: 'Instrument Serif', familySrc: '/fonts/InstrumentSerif-Regular.ttf' });
      font.registerFont({ familyName: 'Instrument Italic', familySrc: '/fonts/InstrumentSerif-Italic.ttf' });
    } catch (e) { /* fonts may already be registered on warm start */ }
  }

  private scheduleNavigate(): void {
    if (this.navigated) return;
    setTimeout((): void => { this.fadingOut = true; }, 260);
    setTimeout((): void => {
      if (this.navigated) return;
      this.navigated = true;
      router.replaceUrl({ url: 'pages/Index' }).catch((): void => {});
    }, 940);
  }

  private setBarContent(color: string): void {
    try {
      const host = this.getUIContext().getHostContext();
      if (host === undefined || host === null) return;
      const ctx = host as common.UIAbilityContext;
      window.getLastWindow(ctx).then((win: window.Window): void => {
        win.setWindowSystemBarProperties({
          statusBarContentColor: color,
          navigationBarContentColor: color
        });
      }).catch((): void => {});
    } catch (e) { /* ignore */ }
  }

  private pctLabel(): string {
    const n: number = Math.floor(this.progress * 100);
    if (n < 10) return `00${n}`;
    if (n < 100) return `0${n}`;
    return `${n}`;
  }

  // ---- Stage A: weixin-console brand header ---------------------------

  @Builder stageABrand() {
    Column() {
      // 顶部 brand mark + 标题区已删除 —— 改由 splash.html 的视频自身
      // 来承载品牌视觉。Stage A 现在只剩底部的 meta strip。
      Blank()

      // Bottom-left meta strip — version + boot timestamp, small mono
      // row like a dashboard startup log.
      Row({ space: 18 }) {
        Row({ space: 6 }) {
          Column().width(5).height(5).borderRadius(3)
            .backgroundColor('#34D399')
            .shadow({ radius: 6, color: '#34D399', offsetX: 0, offsetY: 0 })
          Text('LINK STABLE')
            .fontSize(10).fontWeight(500)
            .letterSpacing(2.0).fontColor('#A1A1AA')
            .fontFamily('Space Grotesk')
        }
        Column().width(1).height(10).backgroundColor('#FFFFFF26')
        Text('HISILICON · V1.0 · 2026')
          .fontSize(10).fontWeight(500)
          .letterSpacing(2.0).fontColor('#A1A1AA')
          .fontFamily('Space Grotesk')
      }
      .opacity(this.revealStageAMeta ? 1 : 0)
      .translate({ y: this.revealStageAMeta ? 0 : 8 })
      .animation({ duration: 700, curve: Curve.FastOutSlowIn })
    }
    .alignItems(HorizontalAlign.Start)
    .width('100%').height('100%')
    .padding({ left: 28, right: 28, top: 64, bottom: 52 })
  }

  // ---- Stage B: 喂！星 hero + progress (existing) --------------------

  @Builder stageBHero() {
    Column() {
      // Brand mark 由 splash.html 的视频自身展现，ArkTS 这一层不再重复
      // 叠图标 / 大字。只保留下面的副标题 + slogan 作为文字浮层。

      Row() {
        Column() {}.width(28).height(1).backgroundColor('#FFFFFF66')
        Text('WEI · XING')
          .fontSize(12).fontWeight(500)
          .letterSpacing(6).fontColor('#FFFFFFD9')
          .fontFamily('Space Grotesk')
          .margin({ left: 14, right: 14 })
        Column() {}.width(28).height(1).backgroundColor('#FFFFFF66')
      }
      .margin({ top: 20 })
      .opacity(this.revealSub ? 1 : 0)
      .translate({ y: this.revealSub ? 0 : 10 })
      .animation({ duration: 950, curve: Curve.FastOutSlowIn })

      Text('A space between your hands')
        .fontSize(19).fontWeight(400)
        .fontColor('#FFFFFFE6')
        .fontFamily('Instrument Italic')
        .fontStyle(FontStyle.Italic)
        .margin({ top: 16 })
        .opacity(this.revealSlogan ? 1 : 0)
        .translate({ y: this.revealSlogan ? 0 : 14 })
        .animation({ duration: 1050, curve: Curve.FastOutSlowIn })
    }
    .width('100%').height('100%')
    .justifyContent(FlexAlign.End)
    .alignItems(HorizontalAlign.Center)
    .padding({ bottom: 170 })
    .hitTestBehavior(HitTestMode.Transparent)
  }

  @Builder stageBFooter() {
    Column() {
      Blank()
      Stack({ alignContent: Alignment.Start }) {
        Column() {}.width('100%').height(1).backgroundColor('#FFFFFF1F')
        Column() {}.width(`${this.progress * 100}%`).height(1).backgroundColor('#FFFFFFEE')
      }
      .width('100%')
      .margin({ bottom: 14 })

      Row() {
        Text('INITIALISING ORBIT')
          .fontSize(10).fontWeight(500)
          .letterSpacing(3).fontColor('#FFFFFFA6')
          .fontFamily('Space Grotesk')
        Blank()
        Text(this.pctLabel())
          .fontSize(10).fontWeight(700)
          .letterSpacing(2).fontColor('#FFFFFFE6')
          .fontFamily('Space Grotesk')
      }
      .width('100%')
    }
    .width('100%').height('100%')
    .padding({ left: 32, right: 32, bottom: 56 })
    .opacity(this.revealFooter ? 1 : 0)
    .translate({ y: this.revealFooter ? 0 : 10 })
    .animation({ duration: 700, curve: Curve.FastOutSlowIn })
    .hitTestBehavior(HitTestMode.Transparent)
  }

  build() {
    Stack() {
      // Shared backdrop: 3D satellite scene for both stages.
      Web({
        src: $rawfile('splash.html'),
        controller: this.webController
      })
        .width('100%').height('100%')
        .backgroundColor('#020510')
        .javaScriptAccess(true)
        .domStorageAccess(true)
        .fileAccess(true)
        .imageAccess(true)
        .onlineImageAccess(true)
        .mixedMode(MixedMode.All)
        .horizontalScrollBarAccess(false)
        .verticalScrollBarAccess(false)
        .overScrollMode(OverScrollMode.NEVER)
        .zoomAccess(false)
        .forceDarkAccess(false)
        .hitTestBehavior(HitTestMode.None)
        .onPageBegin((e): void => {
          hilog.info(SPLASH_DOMAIN, SPLASH_TAG, 'web pageBegin %{public}s', e.url);
        })
        .onPageEnd((e): void => {
          hilog.info(SPLASH_DOMAIN, SPLASH_TAG, 'web pageEnd %{public}s', e.url);
        })
        .onErrorReceive((e): void => {
          hilog.error(SPLASH_DOMAIN, SPLASH_TAG, 'web error %{public}s',
            e.error.getErrorInfo());
        })
        .onConsole((e): boolean => {
          hilog.info(SPLASH_DOMAIN, SPLASH_TAG,
            'web console %{public}s', e.message.getMessage());
          return false;
        })

      // Stage A — weixin console brand. Fades out at the cross-fade.
      Column() { this.stageABrand() }
        .width('100%').height('100%')
        .opacity(this.stageA ? 1 : 0)
        .animation({ duration: 700, curve: Curve.FastOutSlowIn })
        .hitTestBehavior(HitTestMode.Transparent)

      // Stage B — 星·核 hero + progress footer. Invisible until the
      // cross-fade; revealed per-element by the stagger timers above.
      Column() { this.stageBHero() }
        .width('100%').height('100%')
        .opacity(this.stageA ? 0 : 1)
        .animation({ duration: 700, curve: Curve.FastOutSlowIn })
        .hitTestBehavior(HitTestMode.Transparent)

      Column() { this.stageBFooter() }
        .width('100%').height('100%')
        .opacity(this.stageA ? 0 : 1)
        .animation({ duration: 700, curve: Curve.FastOutSlowIn })
        .hitTestBehavior(HitTestMode.Transparent)
    }
    .width('100%').height('100%')
    .backgroundColor('#020510')
    .opacity(this.fadingOut ? 0 : 1)
    .animation({ duration: 700, curve: Curve.FastOutSlowIn })
  }
}
