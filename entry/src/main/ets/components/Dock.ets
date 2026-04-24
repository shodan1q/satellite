/**
 * Dock — bottom navigation bar. Restyled from the earlier rounded-pill
 * look into a terminal-strip to match the Devices page aesthetic:
 * narrow 4px corners, 1px hairline top border, monospaced uppercase
 * label under each icon, accent underline on the active tab.
 */

import { NebulaColor, NebulaFont, NebulaFontFace } from '../theme/Theme';
import { IconName, NebulaIcon } from './NebulaIcon';

export class DockItem {
  id: number = 0;
  label: string = '';
  icon: string = IconName.Workshop;
}

export function mkDockItem(id: number, label: string, icon: string): DockItem {
  const d = new DockItem();
  d.id = id; d.label = label; d.icon = icon;
  return d;
}

@Component
export struct Dock {
  @Prop items: DockItem[] = [];
  @Prop active: number = 0;
  onSelect: (id: number) => void = (): void => {};

  @Builder itemView(item: DockItem) {
    Column({ space: 4 }) {
      NebulaIcon({
        name: item.icon,
        glyphSize: 18,
        color: this.active === item.id ? NebulaColor.textPrimary : NebulaColor.textTertiary,
        strokeW: this.active === item.id ? 2.2 : 1.8
      })
      Text(item.label)
        .fontSize(NebulaFont.caption)
        .fontFamily(NebulaFontFace.mono)
        .fontWeight(this.active === item.id ? FontWeight.Medium : FontWeight.Regular)
        .fontColor(this.active === item.id ? NebulaColor.textPrimary : NebulaColor.textTertiary)
        .letterSpacing(1.4)
      // Active underline — sits flush below the label like an active tab
      // marker on an instrument console.
      Column()
        .width(22).height(2).borderRadius(1)
        .backgroundColor(this.active === item.id ? NebulaColor.brand : Color.Transparent)
    }
    .padding({ top: 10, bottom: 8, left: 10, right: 10 })
    .layoutWeight(1)
    .alignItems(HorizontalAlign.Center)
    .onClick((): void => this.onSelect(item.id))
  }

  build() {
    Column() {
      // Hairline top border — the single line that separates dock from
      // page content; replaces the old shadow.
      Column().width('100%').height(1).backgroundColor(NebulaColor.border)
      Row({ space: 0 }) {
        ForEach(this.items, (item: DockItem): void => {
          this.itemView(item)
        }, (item: DockItem): string => item.id.toString())
      }
      .width('100%')
      // Solid page-bg here (not translucent + blur) so the dock reads
      // as hardware chrome instead of a floating glass pill, and the
      // safe-area filler below it butts up seamlessly.
      .backgroundColor(NebulaColor.bg)
    }
    .width('100%')
  }
}
