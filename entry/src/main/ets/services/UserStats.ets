/**
 * UserStats — persistent activity store backed by @ohos.data.preferences.
 *
 * Keeps counters (workshopOpens, academyOpens, launches, shares) plus a
 * rolling activity log (type / title / timestamp). Profile reads these to
 * render real statistics — there are no hardcoded numbers anywhere.
 */

import dataPreferences from '@ohos.data.preferences';
import common from '@ohos.app.ability.common';
import { hilog } from '@kit.PerformanceAnalysisKit';

const TAG = 'UserStats';
const STORE = 'nebula-user-stats';
const LOG_LIMIT = 50;

export class ActivityRecord {
  kind: string = '';       // 'workshop' | 'academy' | 'launch' | 'share'
  title: string = '';
  time: number = 0;
}

class ActivityArray {
  items: ActivityRecord[] = [];
}

export class StatSnapshot {
  workshopOpens: number = 0;
  academyOpens: number = 0;
  launches: number = 0;
  shares: number = 0;
  partsBookmarked: number = 0;
  recent: ActivityRecord[] = [];
}

export type StatListener = (snap: StatSnapshot) => void;

export class UserStats {
  private static _inst: UserStats | null = null;

  private prefs: dataPreferences.Preferences | null = null;
  private listeners: StatListener[] = [];
  private cached: StatSnapshot = new StatSnapshot();
  private ready: boolean = false;

  static get instance(): UserStats {
    if (UserStats._inst === null) UserStats._inst = new UserStats();
    return UserStats._inst as UserStats;
  }

  async init(ctx: common.UIAbilityContext): Promise<void> {
    if (this.ready) return;
    try {
      this.prefs = await dataPreferences.getPreferences(ctx, STORE);
      await this.reload();
      this.ready = true;
    } catch (e) {
      hilog.warn(0x1, TAG, 'prefs init failed');
    }
  }

  snap(): StatSnapshot { return this.cached; }

  subscribe(cb: StatListener): () => void {
    this.listeners.push(cb);
    cb(this.cached);
    return (): void => {
      const i = this.listeners.indexOf(cb);
      if (i >= 0) this.listeners.splice(i, 1);
    };
  }

  private emit(): void {
    const s = this.cached;
    for (const l of this.listeners) { try { l(s); } catch (e) { /* swallow */ } }
  }

  private async reload(): Promise<void> {
    if (this.prefs === null) return;
    const p = this.prefs;
    const s = new StatSnapshot();
    try {
      s.workshopOpens = await p.get('workshopOpens', 0) as number;
      s.academyOpens = await p.get('academyOpens', 0) as number;
      s.launches = await p.get('launches', 0) as number;
      s.shares = await p.get('shares', 0) as number;
      s.partsBookmarked = await p.get('partsBookmarked', 0) as number;
      const logJson = await p.get('log', '{"items":[]}') as string;
      try {
        const arr = JSON.parse(logJson) as ActivityArray;
        s.recent = (arr.items ?? []).slice(0, LOG_LIMIT);
      } catch (e) {
        s.recent = [];
      }
    } catch (e) {
      hilog.warn(0x1, TAG, 'reload failed');
    }
    this.cached = s;
    this.emit();
  }

  async recordOpenWorkshop(title: string): Promise<void> {
    await this.bumpCounter('workshopOpens');
    await this.pushLog('workshop', title);
  }

  async recordOpenAcademy(title: string): Promise<void> {
    await this.bumpCounter('academyOpens');
    await this.pushLog('academy', title);
  }

  async recordLaunch(title: string): Promise<void> {
    await this.bumpCounter('launches');
    await this.pushLog('launch', title);
  }

  async recordShare(title: string): Promise<void> {
    await this.bumpCounter('shares');
    await this.pushLog('share', title);
  }

  async recordBookmark(title: string): Promise<void> {
    await this.bumpCounter('partsBookmarked');
    await this.pushLog('bookmark', title);
  }

  async reset(): Promise<void> {
    if (this.prefs === null) return;
    try {
      await this.prefs.clear();
      await this.prefs.flush();
    } catch (e) {
      hilog.warn(0x1, TAG, 'reset failed');
    }
    await this.reload();
  }

  private async bumpCounter(key: string): Promise<void> {
    if (this.prefs === null) return;
    try {
      const v = await this.prefs.get(key, 0) as number;
      await this.prefs.put(key, v + 1);
      await this.prefs.flush();
    } catch (e) {
      hilog.warn(0x1, TAG, 'bump ' + key + ' failed');
    }
  }

  private async pushLog(kind: string, title: string): Promise<void> {
    if (this.prefs === null) return;
    try {
      const existing = await this.prefs.get('log', '{"items":[]}') as string;
      let arr: ActivityArray;
      try { arr = JSON.parse(existing) as ActivityArray; }
      catch (e) { arr = new ActivityArray(); arr.items = []; }
      if (arr.items === null || arr.items === undefined) arr.items = [];
      const rec = new ActivityRecord();
      rec.kind = kind; rec.title = title; rec.time = Date.now();
      arr.items.unshift(rec);
      if (arr.items.length > LOG_LIMIT) arr.items = arr.items.slice(0, LOG_LIMIT);
      await this.prefs.put('log', JSON.stringify(arr));
      await this.prefs.flush();
      await this.reload();
    } catch (e) {
      hilog.warn(0x1, TAG, 'log push failed');
    }
  }
}
