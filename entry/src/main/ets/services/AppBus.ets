/**
 * AppBus — tiny cross-page event bus.
 *
 * Pages like Discover publish intents ("open Saturn V in AR Workshop") which
 * Index / Workshop / Academy listen for. Keeps the root @Entry component
 * thin while letting any page navigate without prop drilling.
 */

import { HandoffPayload } from './Handoff';

export type TabListener = (tab: number) => void;
export type RequestListener = (id: string) => void;
export type HandoffListener = (payload: HandoffPayload) => void;
export type HandoffSheetListener = (payload: HandoffPayload | null) => void;

export class HandoffCinematicEvent {
  // 'outgoing' = this device just fired off a handoff;
  // 'incoming' = this device just received one;
  // 'end'      = animation should dismiss.
  kind: string = 'end';
  deviceName: string = '';
  targetKind: string = 'workshop';   // 'workshop' | 'academy'
}
export type CinematicListener = (e: HandoffCinematicEvent) => void;

export const TAB_WORKSHOP: number = 0;
export const TAB_ACADEMY: number = 1;
export const TAB_DISCOVER: number = 2;
export const TAB_DEVICE: number = 3;
export const TAB_PROFILE: number = 4;

export class AppBus {
  private static _inst: AppBus | null = null;

  private tabListeners: TabListener[] = [];
  private rocketListeners: RequestListener[] = [];
  private academyListeners: RequestListener[] = [];
  private handoffListeners: HandoffListener[] = [];
  private pendingHandoff: HandoffPayload | null = null;
  private cinematicListeners: CinematicListener[] = [];
  private handoffSheetListeners: HandoffSheetListener[] = [];

  static get instance(): AppBus {
    if (AppBus._inst === null) {
      AppBus._inst = new AppBus();
    }
    return AppBus._inst as AppBus;
  }

  onTab(cb: TabListener): () => void {
    this.tabListeners.push(cb);
    return (): void => {
      const i = this.tabListeners.indexOf(cb);
      if (i >= 0) this.tabListeners.splice(i, 1);
    };
  }
  navigateTab(tab: number): void {
    for (const l of this.tabListeners) {
      try { l(tab); } catch (e) { /* swallow */ }
    }
  }

  onRocketRequest(cb: RequestListener): () => void {
    this.rocketListeners.push(cb);
    return (): void => {
      const i = this.rocketListeners.indexOf(cb);
      if (i >= 0) this.rocketListeners.splice(i, 1);
    };
  }
  requestRocket(id: string): void {
    for (const l of this.rocketListeners) {
      try { l(id); } catch (e) { /* swallow */ }
    }
  }

  onAcademyRequest(cb: RequestListener): () => void {
    this.academyListeners.push(cb);
    return (): void => {
      const i = this.academyListeners.indexOf(cb);
      if (i >= 0) this.academyListeners.splice(i, 1);
    };
  }
  requestAcademy(id: string): void {
    for (const l of this.academyListeners) {
      try { l(id); } catch (e) { /* swallow */ }
    }
  }

  openInWorkshop(rocketId: string): void {
    this.requestRocket(rocketId);
    this.navigateTab(TAB_WORKSHOP);
  }

  openInAcademy(entryId: string): void {
    this.requestAcademy(entryId);
    this.navigateTab(TAB_ACADEMY);
  }

  onHandoff(cb: HandoffListener): () => void {
    this.handoffListeners.push(cb);
    // If a payload arrived before anyone was listening (e.g. cold start via
    // continuation), replay it exactly once to the new subscriber.
    if (this.pendingHandoff !== null) {
      const p = this.pendingHandoff;
      try { cb(p); } catch (e) { /* swallow */ }
      this.pendingHandoff = null;
    }
    return (): void => {
      const i = this.handoffListeners.indexOf(cb);
      if (i >= 0) this.handoffListeners.splice(i, 1);
    };
  }
  deliverHandoff(payload: HandoffPayload): void {
    if (this.handoffListeners.length === 0) {
      this.pendingHandoff = payload;
      return;
    }
    for (const l of this.handoffListeners) {
      try { l(payload); } catch (e) { /* swallow */ }
    }
  }

  onCinematic(cb: CinematicListener): () => void {
    this.cinematicListeners.push(cb);
    return (): void => {
      const i = this.cinematicListeners.indexOf(cb);
      if (i >= 0) this.cinematicListeners.splice(i, 1);
    };
  }
  emitCinematic(e: HandoffCinematicEvent): void {
    for (const l of this.cinematicListeners) {
      try { l(e); } catch (err) { /* swallow */ }
    }
  }
  startCinematic(kind: string, deviceName: string, targetKind: string): void {
    const e = new HandoffCinematicEvent();
    e.kind = kind; e.deviceName = deviceName; e.targetKind = targetKind;
    this.emitCinematic(e);
  }
  endCinematic(): void {
    const e = new HandoffCinematicEvent();
    e.kind = 'end';
    this.emitCinematic(e);
  }

  onHandoffSheet(cb: HandoffSheetListener): () => void {
    this.handoffSheetListeners.push(cb);
    return (): void => {
      const i = this.handoffSheetListeners.indexOf(cb);
      if (i >= 0) this.handoffSheetListeners.splice(i, 1);
    };
  }
  openHandoffSheet(payload: HandoffPayload): void {
    for (const l of this.handoffSheetListeners) {
      try { l(payload); } catch (e) { /* swallow */ }
    }
  }
  closeHandoffSheet(): void {
    for (const l of this.handoffSheetListeners) {
      try { l(null); } catch (e) { /* swallow */ }
    }
  }
}
