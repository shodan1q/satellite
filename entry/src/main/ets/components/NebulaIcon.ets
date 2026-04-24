/**
 * NebulaIcon — Lucide icon renderer.
 *
 * Every icon is a 24×24 stroke-based SVG dropped into
 * `resources/rawfile/icons/<name>.svg`. ArkUI's Image component
 * respects `stroke="currentColor"` on SVGs when `.fillColor()` is set,
 * so we tint the mono glyph live without any Canvas path reconstruction.
 *
 * Using Lucide keeps the visual language consistent with the weixin
 * console reference design (same `viewBox="0 0 24 24" stroke-width=2
 * stroke-linecap=round stroke-linejoin=round` DNA).
 */

import { NebulaColor } from '../theme/Theme';

export class IconName {
  static readonly Workshop: string = 'workshop';
  static readonly Academy: string = 'academy';
  static readonly Discover: string = 'compass';
  static readonly Device: string = 'device';
  static readonly Profile: string = 'profile';
  static readonly Rocket: string = 'rocket';
  static readonly Satellite: string = 'satellite';
  static readonly Orbit: string = 'orbit';
  static readonly Spark: string = 'spark';
  static readonly Play: string = 'play';
  static readonly Pause: string = 'pause';
  static readonly Plus: string = 'plus';
  static readonly Minus: string = 'minus';
  static readonly Check: string = 'check';
  static readonly Close: string = 'close';
  static readonly Arrow: string = 'arrow';
  static readonly Share: string = 'share';
  static readonly Transfer: string = 'transfer';
  static readonly Watch: string = 'watch';
  static readonly Phone: string = 'phone';
  static readonly Tablet: string = 'tablet';
  static readonly Book: string = 'book';
  static readonly Compass: string = 'compass';
  static readonly Explode: string = 'explode';
  static readonly Reset: string = 'reset';
  static readonly Info: string = 'info';
  static readonly Sparkle: string = 'sparkle';
  static readonly Bolt: string = 'bolt';
  static readonly Layers: string = 'layers';
  static readonly Globe: string = 'globe';
  static readonly Flame: string = 'flame';
  static readonly Signal: string = 'signal';
  static readonly Bookmark: string = 'bookmark';
  static readonly Rotate: string = 'rotate';
  static readonly Hand: string = 'hand';
  static readonly Motion: string = 'motion';
}

@Component
export struct NebulaIcon {
  @Prop name: string = IconName.Rocket;
  @Prop glyphSize: number = 24;
  @Prop color: string = NebulaColor.textPrimary;
  /** Kept for API compatibility with the old canvas icon — Lucide SVGs
   *  bake their stroke-width into the path, so this is a no-op.  We keep
   *  the prop so every existing call-site compiles unchanged. */
  @Prop strokeW: number = 1.8;

  build() {
    // Template renderMode treats the SVG as an alpha mask and fillColor
    // reliably tints it across every HarmonyOS version. The bundled SVGs
    // are authored with `stroke="#FFFFFF"`, so the alpha shape is the
    // Lucide glyph silhouette; fillColor recolours it to match the theme.
    Image($rawfile('icons/' + this.name + '.svg'))
      .width(this.glyphSize)
      .height(this.glyphSize)
      .fillColor(this.color)
      .renderMode(ImageRenderMode.Template)
      .objectFit(ImageFit.Contain)
      .draggable(false)
      .focusable(false)
      .backgroundColor(Color.Transparent)
  }
}
