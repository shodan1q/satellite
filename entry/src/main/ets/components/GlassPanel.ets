import { NebulaRadius } from '../theme/Theme';

@Component
export struct GlassPanel {
  @Prop pad: number = 16;
  @Prop radius: number = NebulaRadius.lg;
  @Prop tint: string = 'light';
  @BuilderParam content: () => void = this.emptyBuilder;

  @Builder emptyBuilder() {}

  private bg(): string {
    // All tints are now dark-console glass. The legacy names are kept
    // for callsite compatibility but read as zinc-900 surfaces with
    // varying accent intensity.
    if (this.tint === 'dark') return '#09090BCC';      // zinc-950/80
    if (this.tint === 'tinted') return '#FF4D33B3';    // brand orange-red
    return '#18181BCC';                                 // zinc-900/80
  }
  private ring(): string {
    if (this.tint === 'dark') return '#27272A';        // zinc-800
    if (this.tint === 'tinted') return '#FF4D334D';    // brand @30%
    return '#27272AE6';                                 // zinc-800/90
  }

  build() {
    Column() {
      this.content()
    }
    .padding(this.pad)
    .backgroundColor(this.bg())
    .borderRadius(this.radius)
    .border({ width: 1, color: this.ring() })
    .backdropBlur(30)
    .shadow({ radius: 32, color: '#000000A6', offsetX: 0, offsetY: 14 })
  }
}
