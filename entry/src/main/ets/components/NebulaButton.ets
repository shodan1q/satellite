import { NebulaColor, NebulaFont, NebulaRadius, NebulaShadow } from '../theme/Theme';
import { IconName, NebulaIcon } from './NebulaIcon';

@Component
export struct NebulaButton {
  @Prop label: string = '';
  @Prop iconName: string = '';
  @Prop variant: string = 'primary';
  @Prop btnSize: string = 'md';
  @Prop disabled: boolean = false;
  onTap: () => void = (): void => {};

  private bg(): string {
    if (this.disabled) return '#E6E9F1';
    if (this.variant === 'primary') return NebulaColor.brand;
    if (this.variant === 'ghost') return '#00000000';
    if (this.variant === 'outline') return NebulaColor.surface;
    if (this.variant === 'soft') return NebulaColor.brandSoft;
    if (this.variant === 'danger') return NebulaColor.danger;
    return NebulaColor.brand;
  }

  private fg(): string {
    if (this.disabled) return NebulaColor.textTertiary;
    if (this.variant === 'primary' || this.variant === 'danger') return '#FFFFFF';
    if (this.variant === 'ghost' || this.variant === 'outline') return NebulaColor.textPrimary;
    if (this.variant === 'soft') return NebulaColor.brand;
    return '#FFFFFF';
  }

  private padV(): number {
    if (this.btnSize === 'lg') return 14;
    if (this.btnSize === 'sm') return 6;
    return 10;
  }

  private padH(): number {
    if (this.btnSize === 'lg') return 22;
    if (this.btnSize === 'sm') return 12;
    return 16;
  }

  private fontSz(): number {
    if (this.btnSize === 'lg') return NebulaFont.bodyLg;
    if (this.btnSize === 'sm') return NebulaFont.small;
    return NebulaFont.body;
  }

  private iconSz(): number {
    if (this.btnSize === 'lg') return 20;
    if (this.btnSize === 'sm') return 14;
    return 16;
  }

  private borderW(): number {
    return this.variant === 'outline' ? 1 : 0;
  }

  build() {
    Row({ space: 8 }) {
      if (this.iconName.length > 0) {
        NebulaIcon({
          name: this.iconName,
          glyphSize: this.iconSz(),
          color: this.fg(),
          strokeW: 2
        })
      }
      if (this.label.length > 0) {
        Text(this.label)
          .fontSize(this.fontSz())
          .fontWeight(FontWeight.Medium)
          .fontColor(this.fg())
      }
    }
    .padding({ top: this.padV(), bottom: this.padV(), left: this.padH(), right: this.padH() })
    .backgroundColor(this.bg())
    .borderRadius(NebulaRadius.pill)
    .border({ width: this.borderW(), color: NebulaColor.border })
    .opacity(this.disabled ? 0.8 : 1)
    .onClick((): void => { if (!this.disabled) this.onTap(); })
  }
}
