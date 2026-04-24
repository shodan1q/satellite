/**
 * Device — control panel for 星核 peripherals. Rendered via an ArkWeb
 * WebView pointing at rawfile/devices/index.html. A small JS bridge
 * (`nebula.readRawfileBase64`) hands the bundled TTFs over to the page
 * so it can register them via the FontFace API; @font-face with
 * resource:// URLs is unreliable on ArkWeb and falls through to system
 * fonts, which made the statusbar look different from the ArkTS pages
 * (those resolve Inter / Instrument Serif through font.registerFont).
 */

import { webview } from '@kit.ArkWeb';
import common from '@ohos.app.ability.common';
import util from '@ohos.util';

class DevicesBridge {
  context: common.UIAbilityContext | null = null;

  /** Returns the rawfile at `path` (e.g. "fonts/Inter-Variable.ttf")
   *  as a base64 string. Empty string on failure — caller treats that
   *  as "skip this font". */
  readRawfileBase64(path: string): string {
    if (this.context === null) return '';
    try {
      const bytes: Uint8Array =
        this.context.resourceManager.getRawFileContentSync(path);
      const helper = new util.Base64Helper();
      return helper.encodeToStringSync(bytes);
    } catch (e) {
      return '';
    }
  }
}

@Component
export struct DevicePage {
  private controller: webview.WebviewController = new webview.WebviewController();
  private bridgeObj: DevicesBridge = new DevicesBridge();

  aboutToAppear(): void {
    try {
      const host = this.getUIContext().getHostContext();
      this.bridgeObj.context = host !== undefined && host !== null
        ? host as common.UIAbilityContext : null;
    } catch (e) { /* context may not be ready; readRawfileBase64 returns '' */ }
  }

  build() {
    Column() {
      Web({
        src: $rawfile('devices/index.html'),
        controller: this.controller
      })
        .domStorageAccess(true)
        .fileAccess(true)
        .javaScriptAccess(true)
        .javaScriptProxy({
          object: this.bridgeObj,
          name: 'nebula',
          methodList: ['readRawfileBase64'],
          controller: this.controller
        })
        .width('100%')
        .height('100%')
        .backgroundColor('#0a0a0c')
    }
    .width('100%').height('100%')
    .backgroundColor('#0a0a0c')
  }
}
