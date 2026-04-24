import { NebulaColor } from '../theme/Theme';
import { IconName } from '../components/NebulaIcon';
import { Dock, DockItem, mkDockItem } from '../components/Dock';
import { HandoffCinematic } from '../components/HandoffCinematic';
import { HandoffSheet } from '../components/HandoffSheet';
import { AppBus } from '../services/AppBus';
import { Handoff, HandoffPayload } from '../services/Handoff';
import { NebulaSync } from '../services/NebulaSync';
import common from '@ohos.app.ability.common';
import mediaquery from '@ohos.mediaquery';
import { DiscoverPage } from './Discover';
import { WorkshopPage } from './Workshop';
import { AcademyPage } from './Academy';
import { DevicePage } from './Device';
import { ProfilePage } from './Profile';

@Entry
@Component
struct Index {
  @State private tab: number = 0;
  @State private sheetOpen: boolean = false;
  @State private sheetPayload: HandoffPayload = new HandoffPayload();
  @State private landscape: boolean = false;
  private unsubTab: () => void = (): void => {};
  private unsubHandoff: () => void = (): void => {};
  private unsubSheet: () => void = (): void => {};
  private landscapeQuery: mediaquery.MediaQueryListener | null = null;

  private items: DockItem[] = [
    mkDockItem(0, 'AR 工坊', IconName.Workshop),
    mkDockItem(1, '科普', IconName.Academy),
    mkDockItem(2, '发现', IconName.Discover),
    mkDockItem(3, '周边', IconName.Device),
    mkDockItem(4, '我的', IconName.Profile)
  ];

  // Visited-tabs bitmask. Bit N = page N has been entered at least once
  // and should stay mounted in the Stack with visibility toggled rather
  // than conditionally (re)built. Without this, switching to the Devices
  // tab reloads its WebView from scratch every time, which reads on
  // screen as a full-frame white flicker. Workshop / Academy etc. don't
  // flicker but they also pay a full rebuild cost on each tab return
  // (subscriptions re-fire, list scroll resets) — this fixes all of it
  // uniformly. Tab 0 is pre-visited because that's where we start.
  @State private visitedMask: number = 1;

  private gotoTab(t: number): void {
    this.visitedMask = this.visitedMask | (1 << t);
    this.tab = t;
  }

  aboutToAppear(): void {
    // Track landscape orientation so the bottom Dock can hide in
    // landscape (AR Workshop in particular needs the full screen for
    // the camera canvas — nav takes too much vertical real-estate at
    // that aspect). mediaquery covers both manual Workshop-triggered
    // rotations and system-level auto-rotate.
    try {
      this.landscapeQuery = mediaquery.matchMediaSync('(orientation: landscape)');
      this.landscape = this.landscapeQuery.matches;
      this.landscapeQuery.on('change', (r: mediaquery.MediaQueryResult): void => {
        this.landscape = r.matches;
      });
    } catch (e) { /* mediaquery may not be available; default to portrait */ }

    this.unsubTab = AppBus.instance.onTab((t: number): void => { this.gotoTab(t); });
    // Cross-device continuation arrives as a HandoffPayload via AppBus —
    // route to the right tab + trigger the page-level rocket / academy
    // request so Workshop.ets / Academy.ets rehydrate their state.
    this.unsubHandoff = AppBus.instance.onHandoff((p: HandoffPayload): void => {
      if (p.kind === 'academy') {
        const id = p.entryKind === 'satellite' ? p.satelliteId : p.rocketId;
        if (id.length > 0) AppBus.instance.openInAcademy(id);
      } else {
        const id = p.rocketId.length > 0 ? p.rocketId : p.satelliteId;
        if (id.length > 0) AppBus.instance.openInWorkshop(id);
        else this.gotoTab(0);
      }
      // Receiving side also auto-joins the distributed session
      // otherwise the observer's DDO isn't in the room and controller
      // mutations never propagate. We request permission first; the
      // prompt surfaces on the peer so it's clear why.
      try {
        const host = this.getUIContext().getHostContext();
        const ctx = host !== undefined && host !== null
          ? host as common.UIAbilityContext : null;
        if (ctx !== null) {
          Handoff.ensurePermission(ctx).then((granted: boolean): void => {
            if (granted) NebulaSync.instance.bindSession(ctx);
          });
        }
      } catch (e) { /* ignore */ }
    });
    this.unsubSheet = AppBus.instance.onHandoffSheet((p: HandoffPayload | null): void => {
      if (p !== null) { this.sheetPayload = p; this.sheetOpen = true; }
      else this.sheetOpen = false;
    });
  }
  aboutToDisappear(): void {
    this.unsubTab(); this.unsubHandoff(); this.unsubSheet();
    try {
      if (this.landscapeQuery !== null) this.landscapeQuery.off('change');
    } catch (e) { /* ignore */ }
  }

  @Builder page() {
    // Every tab that's been visited stays mounted inside this Stack.
    // Only the active tab's wrapper flips to Visibility.Visible; the
    // rest go to Visibility.None so they don't catch touches or paint.
    // The outer `if` on visitedMask controls *mount* (first visit lazy-
    // loads), the inner visibility controls *display* (subsequent
    // switches are instant, no remount, no flicker).
    Stack() {
      if ((this.visitedMask & 1) !== 0) {
        Column() { WorkshopPage() }
          .width('100%').height('100%')
          .visibility(this.tab === 0 ? Visibility.Visible : Visibility.None)
      }
      if ((this.visitedMask & 2) !== 0) {
        Column() { AcademyPage() }
          .width('100%').height('100%')
          .visibility(this.tab === 1 ? Visibility.Visible : Visibility.None)
      }
      if ((this.visitedMask & 4) !== 0) {
        Column() { DiscoverPage() }
          .width('100%').height('100%')
          .visibility(this.tab === 2 ? Visibility.Visible : Visibility.None)
      }
      if ((this.visitedMask & 8) !== 0) {
        Column() { DevicePage() }
          .width('100%').height('100%')
          .visibility(this.tab === 3 ? Visibility.Visible : Visibility.None)
      }
      if ((this.visitedMask & 16) !== 0) {
        Column() { ProfilePage() }
          .width('100%').height('100%')
          .visibility(this.tab === 4 ? Visibility.Visible : Visibility.None)
      }
    }
    .width('100%').height('100%')
  }

  build() {
    Stack({ alignContent: Alignment.Bottom }) {
      Stack() {
        this.page()
      }
      .width('100%').height('100%')

      if (!this.landscape) {
        Column() {
          Dock({
            items: this.items,
            active: this.tab,
            onSelect: (id: number): void => { this.gotoTab(id); }
          })
          // Safe-area filler — Index expandSafeArea extends content under
          // the home-indicator / gesture-nav strip, so we paint a solid
          // opaque slab below the Dock that tints out to the edge of the
          // screen. Height ~34vp matches the typical HarmonyOS bottom
          // inset; opaque background so the page content scrolling behind
          // doesn't bleed through (the old #0B0B0EF0 was translucent).
          Column().height(34).width('100%').backgroundColor(NebulaColor.bg)
        }
        .width('100%')
        .backgroundColor(NebulaColor.bg)
      }

      // Handoff sheet — mounted at the root (above pages + dock) so the
      // device list isn't clipped by the bottom navigation.
      HandoffSheet({
        open: this.sheetOpen,
        payload: this.sheetPayload,
        onDismiss: (): void => { this.sheetOpen = false; },
        onHandedOff: (): void => { this.sheetOpen = false; }
      })

      // Cross-device cutscene — mounted at the root so it always sits
      // above pages / dock / sheets when a handoff fires.
      HandoffCinematic()
    }
    .width('100%').height('100%')
    .backgroundColor(NebulaColor.bg)
    .expandSafeArea([SafeAreaType.SYSTEM], [SafeAreaEdge.TOP, SafeAreaEdge.BOTTOM])
  }
}
