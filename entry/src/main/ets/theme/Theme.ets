/**
 * Nebula Core design tokens — mission-control dark theme, ported from
 * the weixin "喂！星" personal-satellite console. The language is
 * near-black surfaces, neutral zinc scale text, and a single SpaceX
 * orange-red (#ff4d33) accent.
 *
 * Typography: Inter for Latin/digits + HarmonyOS Sans for Chinese.
 * No emojis anywhere in the product. Icons are vector paths.
 */

export class NebulaColor {
  // Backgrounds — stacked three levels deep like a real dashboard chrome
  static readonly bg: string = '#0A0A0C';          // page background
  static readonly bgSoft: string = '#121214';      // inset panels
  static readonly surface: string = '#18181B';     // zinc-900 cards
  static readonly surfaceElev: string = '#1F1F23'; // elevated tiles / hover

  // Borders — all dark translucent zincs, never pure black
  static readonly border: string = '#27272A';      // zinc-800
  static readonly borderSoft: string = '#1F1F23';  // zinc-800/50 feel
  static readonly gridLine: string = '#27272A';

  // Text — mission-control grayscale, with near-white primary
  static readonly textPrimary: string = '#FAFAFA';   // zinc-50
  static readonly textSecondary: string = '#A1A1AA'; // zinc-400
  static readonly textTertiary: string = '#71717A';  // zinc-500
  static readonly textMuted: string = '#52525B';     // zinc-600

  // Single strong accent — the console's orange-red, used sparingly
  static readonly brand: string = '#FF4D33';
  static readonly brandSoft: string = '#FF4D3326';    // 15% alpha tint
  static readonly brandHover: string = '#FF6B4D';

  // Secondary accents kept for legacy callsites but re-tuned toward
  // the dark console: cool amber, muted sky, cool violet, soft mint,
  // danger-coral. Everything reads against the near-black body.
  static readonly accentCyan: string = '#38BDF8';    // sky-400
  static readonly accentCyanSoft: string = '#0EA5E91F';
  static readonly accentViolet: string = '#A78BFA';  // violet-400
  static readonly accentVioletSoft: string = '#8B5CF61F';
  static readonly accentGold: string = '#FBBF24';    // amber-400
  static readonly accentGoldSoft: string = '#F59E0B1F';
  static readonly accentMint: string = '#34D399';    // emerald-400
  static readonly accentMintSoft: string = '#10B9811F';
  static readonly accentCoral: string = '#FB7185';   // rose-400
  static readonly accentCoralSoft: string = '#F43F5E1F';

  // Status colours
  static readonly success: string = '#22C55E';       // green-500
  static readonly warning: string = '#F59E0B';       // amber-500
  static readonly danger: string = '#E53935';        // weixin danger red

  // Glass + shadows sit against the near-black bg, so lighter ring
  // highlights read where blur would be used on light themes.
  static readonly glassTop: string = '#FFFFFF0D';
  static readonly glassBottom: string = '#FFFFFF05';
  static readonly shadowLight: string = '#0000004D';
  static readonly shadowMid: string = '#00000080';
}

export class NebulaRadius {
  // `tight` (4-6px) is the instrument-panel corner — used on panels
  // styled after the Devices WebView console. Keeps cards feeling like
  // hardware chrome instead of soft iOS tiles. The bigger radii stay
  // around for legacy and for one-off pill/modal affordances.
  static readonly tight: number = 4;
  static readonly xs: number = 8;
  static readonly sm: number = 10;
  static readonly md: number = 14;
  static readonly lg: number = 18;
  static readonly xl: number = 24;
  static readonly pill: number = 999;
}

export class NebulaSpace {
  static readonly xs: number = 4;
  static readonly sm: number = 8;
  static readonly md: number = 12;
  static readonly lg: number = 16;
  static readonly xl: number = 20;
  static readonly xxl: number = 28;
  static readonly xxxl: number = 36;
}

/**
 * Font sizes — editorial but data-dense.
 * hero/display collapse a bit from the earlier light-theme values because
 * on a near-black canvas the negative space reads larger than it measures.
 */
export class NebulaFont {
  static readonly caption: number = 10;  // uppercase rail labels
  static readonly small: number = 12;    // meta
  static readonly body: number = 14;     // body copy
  static readonly bodyLg: number = 16;   // emphasised body
  static readonly title: number = 18;    // section titles
  static readonly titleLg: number = 22;  // page subsection
  static readonly display: number = 28;  // in-card displays
  static readonly hero: number = 32;     // page hero
}

/**
 * Font family tokens. Declare once here so every page can reference
 * the same names registered via font.registerFont in EntryAbility.
 *   - sans: Inter for Latin + Chinese falls back to HarmonyOS Sans
 *   - mono: system mono for tabular telemetry digits
 */
export class NebulaFontFace {
  // Body / mono / sans resolve to HarmonyOS Sans (the OS system font,
  // always present). We *used* to set `Inter` here via a registered
  // TTF, but ArkTS's text shaping renders Inter slightly differently
  // than the Devices WebView renders the same TTF — letter widths and
  // tracking ended up visibly off between the two. Dropping back to
  // the system font guarantees pixel-identical text across both
  // surfaces, at the cost of a slightly less editorial typeface.
  //
  // Serif stays registered via font.registerFont in Splash.ets — it's
  // only used on oversized page titles and the "星核 · Nebula Core"
  // wordmark where system substitution would be most obvious.
  static readonly sans: string = 'HarmonyOS Sans';
  static readonly display: string = 'HarmonyOS Sans';
  static readonly serif: string = 'Instrument Serif';
  static readonly serifItalic: string = 'Instrument Italic';
  static readonly mono: string = 'HarmonyOS Sans';
}

export class NebulaShadow {
  static readonly card: ShadowOptions = {
    radius: 20,
    color: '#00000080',
    offsetX: 0,
    offsetY: 8
  };
  static readonly lift: ShadowOptions = {
    radius: 32,
    color: '#000000A6',
    offsetX: 0,
    offsetY: 14
  };
  static readonly subtle: ShadowOptions = {
    radius: 10,
    color: '#00000033',
    offsetX: 0,
    offsetY: 4
  };
}
