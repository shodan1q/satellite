import { NebulaColor, NebulaRadius, NebulaShadow, NebulaSpace } from '../theme/Theme';

@Component
export struct NebulaCard {
  @Prop pad: number = NebulaSpace.lg;
  @Prop radius: number = NebulaRadius.lg;
  @Prop tone: string = 'surface';
  @Prop showBorder: boolean = true;
  @Prop lifted: boolean = false;
  @BuilderParam content: () => void = this.emptyBuilder;

  @Builder emptyBuilder() {}

  private bg(): string {
    if (this.tone === 'soft') return NebulaColor.bgSoft;
    if (this.tone === 'brand') return NebulaColor.brandSoft;
    if (this.tone === 'violet') return NebulaColor.accentVioletSoft;
    if (this.tone === 'cyan') return NebulaColor.accentCyanSoft;
    if (this.tone === 'gold') return NebulaColor.accentGoldSoft;
    if (this.tone === 'mint') return NebulaColor.accentMintSoft;
    if (this.tone === 'coral') return NebulaColor.accentCoralSoft;
    return NebulaColor.surface;
  }

  build() {
    Column() {
      this.content()
    }
    .padding(this.pad)
    .backgroundColor(this.bg())
    .borderRadius(this.radius)
    .border(this.showBorder ? { width: 1, color: NebulaColor.borderSoft } : { width: 0 })
    .shadow(this.lifted ? NebulaShadow.lift : NebulaShadow.card)
    .width('100%')
  }
}
