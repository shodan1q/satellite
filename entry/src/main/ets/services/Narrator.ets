/**
 * Narrator — singleton wrapper around @kit.CoreSpeechKit TTS.
 *
 * Rewritten for observability: every stage (create / start / complete /
 * error) logs via hilog and is visible to callers via the speak()
 * Promise. The engine is re-used across calls; if it gets into a bad
 * state (e.g. after the app was backgrounded) we transparently tear
 * down and recreate on the next call.
 */

import { textToSpeech } from '@kit.CoreSpeechKit';
import { BusinessError } from '@ohos.base';
import { hilog } from '@kit.PerformanceAnalysisKit';

const TAG = 'Narrator';
const DOMAIN = 0x0001;

export class Narrator {
  private static _instance: Narrator | null = null;
  private engine: textToSpeech.TextToSpeechEngine | null = null;
  private creating: Promise<void> | null = null;
  private listenerInstalled: boolean = false;
  private lastRequestId: string = '';
  private lastError: string = '';

  static get instance(): Narrator {
    if (Narrator._instance === null) Narrator._instance = new Narrator();
    return Narrator._instance as Narrator;
  }

  lastFailure(): string { return this.lastError; }

  private async ensure(): Promise<boolean> {
    if (this.engine !== null) return true;
    if (this.creating !== null) {
      try { await this.creating; } catch (e) { /* ignore */ }
      return this.engine !== null;
    }
    this.creating = this.create();
    try { await this.creating; } catch (e) { /* ignore */ }
    this.creating = null;
    return this.engine !== null;
  }

  private async create(): Promise<void> {
    // Device support for createEngine params is wildly inconsistent —
    // some phones reject `person: 0`, others need `online: 0`, a few
    // only accept `{ language }`. We try the richest form first and
    // fall back through progressively smaller param sets, logging
    // what finally stuck.
    const attempts: textToSpeech.CreateEngineParams[] = [
      { language: 'zh-CN', person: 1, online: 0 },
      { language: 'zh-CN', person: 0, online: 0 },
      { language: 'zh-CN', person: 2, online: 0 },   // some phones expose 2
      { language: 'zh-CN', person: 0, online: 1 },   // cloud fallback
      { language: 'zh-CN', person: 1, online: 1 }
    ];
    hilog.info(DOMAIN, TAG, 'createEngine begin (will try %{public}d variants)',
      attempts.length);
    for (let i = 0; i < attempts.length; i++) {
      const params = attempts[i];
      try {
        this.engine = await textToSpeech.createEngine(params);
        this.installListener();
        hilog.info(DOMAIN, TAG, 'createEngine ok @ variant %{public}d (%{public}s)',
          i, JSON.stringify(params));
        return;
      } catch (err) {
        const e = err as BusinessError;
        this.lastError = `variant ${i} code=${e.code} msg=${e.message}`;
        hilog.warn(DOMAIN, TAG, 'variant %{public}d failed: %{public}s',
          i, this.lastError);
      }
    }
    hilog.error(DOMAIN, TAG,
      'createEngine exhausted all %{public}d variants — last: %{public}s',
      attempts.length, this.lastError);
    this.engine = null;
  }

  /**
   * Attach a SpeakListener so we can surface real error codes.
   * Without this, every speak() failure is invisible.
   */
  private installListener(): void {
    if (this.listenerInstalled || this.engine === null) return;
    const listener: textToSpeech.SpeakListener = {
      onStart: (rid: string, resp: textToSpeech.StartResponse): void => {
        hilog.info(DOMAIN, TAG, 'onStart rid=%{public}s', rid);
      },
      onComplete: (rid: string, resp: textToSpeech.CompleteResponse): void => {
        hilog.info(DOMAIN, TAG, 'onComplete rid=%{public}s', rid);
      },
      onStop: (rid: string, resp: textToSpeech.StopResponse): void => {
        hilog.info(DOMAIN, TAG, 'onStop rid=%{public}s', rid);
      },
      onError: (rid: string, code: number, msg: string): void => {
        this.lastError = `onError code=${code} msg=${msg}`;
        hilog.error(DOMAIN, TAG, 'onError rid=%{public}s code=%{public}d msg=%{public}s',
          rid, code, msg);
      },
      onData: (rid: string, audio: ArrayBuffer,
               resp: textToSpeech.SynthesisResponse): void => { /* unused */ }
    };
    try {
      this.engine.setListener(listener);
      this.listenerInstalled = true;
      hilog.info(DOMAIN, TAG, 'listener installed');
    } catch (err) {
      const e = err as BusinessError;
      hilog.warn(DOMAIN, TAG, 'setListener failed code=%{public}d msg=%{public}s',
        e.code, e.message);
    }
  }

  /**
   * Speak the given text. Returns true if the engine accepted the
   * call; actual audio can still fail asynchronously — check hilog
   * under tag "Narrator" for onError events, or call `lastFailure()`.
   */
  async speak(text: string): Promise<boolean> {
    this.lastError = '';
    const trimmed = text.trim();
    if (trimmed.length === 0) {
      this.lastError = 'empty text';
      return false;
    }
    // HarmonyOS TTS engine caps per-call text length; chunk if too big.
    const chunk = trimmed.length > 1800 ? trimmed.substring(0, 1800) : trimmed;

    const ok = await this.ensure();
    if (!ok || this.engine === null) {
      hilog.warn(DOMAIN, TAG, 'speak aborted — no engine (%{public}s)', this.lastError);
      return false;
    }
    // Stop any previous utterance so the new one starts immediately.
    try { this.engine.stop(); } catch (e) { /* not speaking */ }

    const rid = `nebula-${Date.now()}`;
    this.lastRequestId = rid;
    try {
      // extraParams: voice params map. HarmonyOS accepts a few common
      // keys; values are strings.
      const extra: Record<string, Object> = {
        'queueMode': '0',   // 0 = interrupt queue, 1 = append
        'speed': '1.0',     // 0.5 .. 2.0
        'volume': '1.0',    // 0 .. 2
        'pitch': '1.0',     // 0.5 .. 2.0
        'languageContext': 'zh-CN',
        'audioType': 'pcm',
        'soundChannel': '3',
        'playType': '1'     // 1 = play via system audio
      };
      const p: textToSpeech.SpeakParams = {
        requestId: rid,
        extraParams: extra
      };
      this.engine.speak(chunk, p);
      hilog.info(DOMAIN, TAG, 'speak dispatched rid=%{public}s len=%{public}d',
        rid, chunk.length);
      return true;
    } catch (err) {
      const e = err as BusinessError;
      this.lastError = `speak throw code=${e.code} msg=${e.message}`;
      hilog.error(DOMAIN, TAG, 'speak() threw: %{public}s', this.lastError);
      // Common mode of failure: engine entered a corrupt state. Kill
      // it so the next call will recreate cleanly.
      try { this.engine.shutdown(); } catch (_) {}
      this.engine = null;
      this.listenerInstalled = false;
      return false;
    }
  }

  stop(): void {
    if (this.engine === null) return;
    try { this.engine.stop(); } catch (e) { /* ignore */ }
  }
}
