import { NebulaColor, NebulaFont, NebulaRadius } from '../theme/Theme';
import { NebulaIcon } from './NebulaIcon';

@Component
export struct NebulaPill {
  @Prop label: string = '';
  @Prop iconName: string = '';
  @Prop tone: string = 'neutral';
  @Prop solid: boolean = false;

  private bg(): string {
    if (!this.solid) {
      if (this.tone === 'brand') return NebulaColor.brandSoft;
      if (this.tone === 'violet') return NebulaColor.accentVioletSoft;
      if (this.tone === 'cyan') return NebulaColor.accentCyanSoft;
      if (this.tone === 'gold') return NebulaColor.accentGoldSoft;
      if (this.tone === 'mint') return NebulaColor.accentMintSoft;
      if (this.tone === 'coral') return NebulaColor.accentCoralSoft;
      return NebulaColor.bgSoft;
    }
    if (this.tone === 'brand') return NebulaColor.brand;
    if (this.tone === 'violet') return NebulaColor.accentViolet;
    if (this.tone === 'cyan') return NebulaColor.accentCyan;
    if (this.tone === 'gold') return NebulaColor.accentGold;
    if (this.tone === 'mint') return NebulaColor.accentMint;
    if (this.tone === 'coral') return NebulaColor.accentCoral;
    return NebulaColor.textPrimary;
  }

  private fg(): string {
    if (this.solid) return '#FFFFFF';
    if (this.tone === 'brand') return NebulaColor.brand;
    if (this.tone === 'violet') return NebulaColor.accentViolet;
    if (this.tone === 'cyan') return NebulaColor.accentCyan;
    if (this.tone === 'gold') return NebulaColor.accentGold;
    if (this.tone === 'mint') return NebulaColor.accentMint;
    if (this.tone === 'coral') return NebulaColor.accentCoral;
    return NebulaColor.textSecondary;
  }

  build() {
    Row({ space: 6 }) {
      if (this.iconName.length > 0) {
        NebulaIcon({ name: this.iconName, glyphSize: 12, color: this.fg(), strokeW: 2 })
      }
      Text(this.label)
        .fontSize(NebulaFont.caption)
        .fontWeight(FontWeight.Medium)
        .fontColor(this.fg())
    }
    .padding({ top: 5, bottom: 5, left: 10, right: 10 })
    .backgroundColor(this.bg())
    .borderRadius(NebulaRadius.pill)
  }
}
