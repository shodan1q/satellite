/**
 * JoystickPad — virtual analog stick for AR scene navigation.
 *
 * Renders a translucent ring with a draggable thumb. Reports the thumb's
 * normalized offset in [-1, 1] continuously to `onChange` while the user
 * is touching it, including when the finger is held still at a deflected
 * position (PanGesture only fires on movement, so we drive a tick timer
 * over the gesture lifecycle to keep the camera moving). On release the
 * thumb springs back to center and a final (0, 0) sample is emitted so
 * the consumer can stop integrating.
 *
 * Convention: dx is screen-X (positive = right), dy is screen-Y (positive
 * = down). Consumers usually want forward = -dy.
 */

import { NebulaColor } from '../theme/Theme';

@Component
export struct JoystickPad {
  /** Diameter of the outer ring in vp. Thumb size scales from this.
      Named `padSize` (not `size`) because ArkTS reserves `size` on
      CustomComponent — colliding with the CommonAttribute setter. */
  padSize: number = 84;
  /** Accent color on touch — defaults to cyan (sat-app brand). Caller
      can pass '#FF4D33' (brand-orange), '#27C7A3' (mint) etc. */
  accent: string = '#3FB7FF';
  /** Caller for live samples. Called at ~30 Hz while touched. */
  onChange: (dx: number, dy: number) => void = (): void => {};

  @State private knobX: number = 0;
  @State private knobY: number = 0;
  @State private active: boolean = false;
  private tickHandle: number = -1;

  // Knob can travel up to (size/2 - knobR - 4) from center before clamp.
  private maxOffset(): number {
    const knobR = this.padSize * 0.18;
    return this.padSize * 0.5 - knobR - 4;
  }

  private startTick(): void {
    if (this.tickHandle >= 0) return;
    const max = this.maxOffset();
    this.tickHandle = setInterval((): void => {
      if (this.knobX === 0 && this.knobY === 0) return;
      this.onChange(this.knobX / max, this.knobY / max);
    }, 33);
  }

  private stopTick(): void {
    if (this.tickHandle >= 0) {
      clearInterval(this.tickHandle);
      this.tickHandle = -1;
    }
  }

  aboutToDisappear(): void {
    this.stopTick();
  }

  build() {
    Stack({ alignContent: Alignment.Center }) {
      // Stacked translucent layers form a soft "depth-well" feel —
      // closer to the physical glass joysticks on Vision Pro / Apple
      // gaming controllers than the previous flat disc. Three concentric
      // circles with progressively darker fills give a vignette without
      // needing a real radial gradient (ArkTS Circle doesn't expose one).
      Circle()
        .width(this.padSize).height(this.padSize)
        .fill('#0F1115AA')
      Circle()
        .width(this.padSize * 0.86).height(this.padSize * 0.86)
        .fill('#0B0D1366')
      Circle()
        .width(this.padSize * 0.72).height(this.padSize * 0.72)
        .fill('#08090E55')
      // Active-only accent ring — sits *under* the knob, drifts with
      // it. Subtle, just enough to confirm engagement without the
      // garish yellow rings the previous design had.
      if (this.active) {
        Circle()
          .width(this.padSize * 0.62).height(this.padSize * 0.62)
          .fill(this.accent + '22')
          .translate({ x: this.knobX, y: this.knobY })
      }
      // Knob — a soft "off-white" puck with a subtle inner gradient
      // implied via two stacked circles (cooler highlight on top, warm
      // shadow at bottom). Reads premium without competing for color
      // attention with the chrome.
      Circle()
        .width(this.padSize * (this.active ? 0.42 : 0.36))
        .height(this.padSize * (this.active ? 0.42 : 0.36))
        .fill('#F5F6F8')
        .shadow({
          radius: this.active ? 16 : 10,
          color: '#000000B0',
          offsetX: 0, offsetY: this.active ? 6 : 4
        })
        .translate({ x: this.knobX, y: this.knobY })
      // Inner highlight on the knob — small lighter spot, makes it
      // feel 3D / spherical instead of pancake-flat.
      Circle()
        .width(this.padSize * (this.active ? 0.22 : 0.18))
        .height(this.padSize * (this.active ? 0.22 : 0.18))
        .fill('#FFFFFF')
        .translate({
          x: this.knobX - this.padSize * 0.05,
          y: this.knobY - this.padSize * 0.05
        })
        .opacity(0.55)
      // Tiny accent dot at the knob center when active — color signal
      // without rim noise.
      if (this.active) {
        Circle()
          .width(this.padSize * 0.08).height(this.padSize * 0.08)
          .fill(this.accent)
          .translate({ x: this.knobX, y: this.knobY })
      }
    }
    .width(this.padSize).height(this.padSize)
    .gesture(
      PanGesture({ direction: PanDirection.All, distance: 0 })
        .onActionStart((): void => {
          this.active = true;
          this.startTick();
        })
        .onActionUpdate((event: GestureEvent): void => {
          let dx = event.offsetX;
          let dy = event.offsetY;
          const max = this.maxOffset();
          const r = Math.sqrt(dx * dx + dy * dy);
          if (r > max) {
            const s = max / r;
            dx *= s; dy *= s;
          }
          this.knobX = dx;
          this.knobY = dy;
        })
        .onActionEnd((): void => {
          this.active = false;
          this.knobX = 0;
          this.knobY = 0;
          this.stopTick();
          this.onChange(0, 0);
        })
        .onActionCancel((): void => {
          this.active = false;
          this.knobX = 0;
          this.knobY = 0;
          this.stopTick();
          this.onChange(0, 0);
        })
    )
  }
}
