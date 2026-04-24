/**
 * ModelStore — reads bundled .glb files from rawfile and hands them to the
 * WebView as base64 strings.
 *
 * We can't rely on the WebView fetching `rawfile://` or `file://` URLs
 * inside GLTFLoader (it silently hangs on HarmonyOS), so instead ArkTS
 * reads the binary once, base64-encodes it, and injects it into the page
 * via runJavaScript. The JS side decodes and calls GLTFLoader.parse() on
 * the in-memory ArrayBuffer — no network / filesystem round-trip.
 */

import common from '@ohos.app.ability.common';
import util from '@ohos.util';
import { hilog } from '@kit.PerformanceAnalysisKit';

const TAG = 'ModelStore';
const RAWFILE_PREFIX = 'three/models/';

export class ModelStore {
  private static _inst: ModelStore | null = null;
  private context: common.UIAbilityContext | null = null;
  private base64Cache: Map<string, string> = new Map();

  static get instance(): ModelStore {
    if (ModelStore._inst === null) ModelStore._inst = new ModelStore();
    return ModelStore._inst as ModelStore;
  }

  init(ctx: common.UIAbilityContext): void {
    this.context = ctx;
  }

  /**
   * Returns a base64 string of the model bytes (empty string on failure).
   * Cached after first read so subsequent opens are instant.
   */
  async readBase64(filename: string): Promise<string> {
    const cached = this.base64Cache.get(filename);
    if (cached !== undefined) return cached;

    const ctx = this.context;
    if (ctx === null || ctx === undefined) return '';

    try {
      const bytes: Uint8Array = await ctx.resourceManager.getRawFileContent(RAWFILE_PREFIX + filename);
      const helper = new util.Base64Helper();
      const b64: string = await helper.encodeToString(bytes);
      this.base64Cache.set(filename, b64);
      return b64;
    } catch (e) {
      hilog.warn(0x1, TAG, 'readBase64 %{public}s failed', filename);
      return '';
    }
  }
}
