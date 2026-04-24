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
  padSize: number = 92;
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
      // Outer ring — glass disc.
      Circle()
        .width(this.padSize).height(this.padSize)
        .fill('#18181BCC')
        .stroke('#FFFFFF22')
        .strokeWidth(1)
      // Inner reference ring (slightly smaller, shows the knob's range).
      Circle()
        .width(this.padSize * 0.62).height(this.padSize * 0.62)
        .fill(Color.Transparent)
        .stroke('#FFFFFF14')
        .strokeWidth(1)
      // Draggable thumb.
      Circle()
        .width(this.padSize * 0.36).height(this.padSize * 0.36)
        .fill(this.active ? '#FFFFFF' : '#FFFFFFEE')
        .shadow({ radius: 10, color: '#00000080', offsetX: 0, offsetY: 4 })
        .translate({ x: this.knobX, y: this.knobY })
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
