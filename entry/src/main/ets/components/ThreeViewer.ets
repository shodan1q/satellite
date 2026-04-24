import { webview } from '@kit.ArkWeb';
import { hilog } from '@kit.PerformanceAnalysisKit';
import { BuildPart } from '../services/NebulaSync';
import { NebulaColor } from '../theme/Theme';

export class ViewerReadyEvent {
  ready: boolean = false;
}

export class ViewerSelectEvent {
  partId: string | null = null;
}

export class ViewerBuildEvent {
  parts: BuildPart[] = [];
}

class BridgePayload {
  partId: string = '';
  ready: boolean = false;
  parts: BuildPart[] = [];
}

class BridgeMessage {
  event: string = '';
  payload: BridgePayload = new BridgePayload();
}

class BridgeObject {
  onMsg: (msg: string) => void = (): void => {};
  postMessage(msg: string): void {
    this.onMsg(msg);
  }
}

export class ThreeBridge {
  controller: webview.WebviewController = new webview.WebviewController();
  ready: boolean = false;

  async setParts(parts: Object[]): Promise<void> {
    if (!this.ready) return;
    try {
      await this.controller.runJavaScript(
        `window.Nebula && window.Nebula.setParts(${JSON.stringify(parts)})`
      );
    } catch (e) { /* ignore */ }
  }

  async setExplode(ratio: number): Promise<void> {
    if (!this.ready) return;
    try {
      await this.controller.runJavaScript(`window.Nebula && window.Nebula.setExplode(${ratio})`);
    } catch (e) {}
  }

  async selectPart(id: string | null): Promise<void> {
    if (!this.ready) return;
    try {
      const payload = id === null ? 'null' : `'${id}'`;
      await this.controller.runJavaScript(`window.Nebula && window.Nebula.selectPart(${payload})`);
    } catch (e) {}
  }

  async setAutoSpin(on: boolean): Promise<void> {
    if (!this.ready) return;
    try {
      await this.controller.runJavaScript(`window.Nebula && window.Nebula.setAutoSpin(${on ? 'true' : 'false'})`);
    } catch (e) {}
  }
}

@Component
export struct ThreeViewer {
  bridge: ThreeBridge = new ThreeBridge();
  onReady: (e: ViewerReadyEvent) => void = (): void => {};
  onSelect: (e: ViewerSelectEvent) => void = (): void => {};
  onBuildChanged: (e: ViewerBuildEvent) => void = (): void => {};

  private bridgeObj: BridgeObject = new BridgeObject();

  aboutToAppear(): void {
    this.bridgeObj.onMsg = (msg: string): void => this.handleBridge(msg);
  }

  private handleBridge(msg: string): void {
    try {
      const parsed: BridgeMessage = JSON.parse(msg) as BridgeMessage;
      if (parsed.event === 'ready') {
        this.bridge.ready = true;
        const evt = new ViewerReadyEvent();
        evt.ready = true;
        this.onReady(evt);
      } else if (parsed.event === 'select') {
        const evt = new ViewerSelectEvent();
        if (parsed.payload !== null && parsed.payload !== undefined) {
          const pid = parsed.payload.partId;
          evt.partId = (pid !== undefined && pid !== null && pid.length > 0) ? pid : null;
        } else {
          evt.partId = null;
        }
        this.onSelect(evt);
      } else if (parsed.event === 'build') {
        const evt = new ViewerBuildEvent();
        evt.parts = parsed.payload !== null && parsed.payload !== undefined
          ? (parsed.payload.parts ?? [])
          : [];
        this.onBuildChanged(evt);
      }
    } catch (e) {
      hilog.warn(0x1, 'ThreeViewer', 'bridge msg parse failed');
    }
  }

  build() {
    Web({
      src: $rawfile('three/workshop.html'),
      controller: this.bridge.controller
    })
      .domStorageAccess(true)
      .fileAccess(true)
      .javaScriptAccess(true)
      .javaScriptProxy({
        object: this.bridgeObj,
        name: 'nebula',
        methodList: ['postMessage'],
        controller: this.bridge.controller
      })
      .onPageEnd((): void => {
        hilog.info(0x1, 'ThreeViewer', 'viewer loaded');
      })
      .backgroundColor(NebulaColor.bg)
      .width('100%')
      .height('100%')
  }
}
