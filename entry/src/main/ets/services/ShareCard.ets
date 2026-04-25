/**
 * ShareCard — builds a plain-text knowledge card and hands it off to the
 * system. Uses @ohos.pasteboard for a guaranteed local copy, then tries
 * @kit.ShareKit to open the system share sheet (隔空传送 / AirDrop lives
 * there). Either way the card content is mirrored into NebulaSync so a
 * paired HarmonyOS device picks up the same part highlight.
 */

import pasteboard from '@ohos.pasteboard';
import { systemShare } from '@kit.ShareKit';
import { uniformTypeDescriptor as utd } from '@kit.ArkData';
import common from '@ohos.app.ability.common';
import { hilog } from '@kit.PerformanceAnalysisKit';

const TAG = 'ShareCard';

export class CardSpec {
  key: string = '';
  value: string = '';
}

export class CardContent {
  title: string = '';
  kind: string = '';
  description: string = '';
  specs: CardSpec[] = [];
  footer: string = '生成自 喂！星 · HarmonyOS AR 太空工坊';
}

export class ShareResult {
  ok: boolean = false;
  channel: string = '';
}

function formatCard(card: CardContent): string {
  const lines: string[] = [];
  lines.push(`【${card.title}】`);
  if (card.kind.length > 0) lines.push(`类别：${card.kind}`);
  lines.push('');
  if (card.description.length > 0) {
    lines.push(card.description);
    lines.push('');
  }
  if (card.specs.length > 0) {
    for (const s of card.specs) lines.push(`• ${s.key}：${s.value}`);
    lines.push('');
  }
  lines.push('—— ' + card.footer);
  return lines.join('\n');
}

export class ShareCard {
  static async shareImage(
    ctx: common.UIAbilityContext,
    card: CardContent,
    imageUri: string
  ): Promise<ShareResult> {
    const result = new ShareResult();
    try {
      const shared = new systemShare.SharedData({
        utd: utd.UniformDataType.IMAGE,
        uri: imageUri,
        title: card.title,
        description: card.description
      });
      const controller = new systemShare.ShareController(shared);
      await controller.show(ctx, {
        selectionMode: systemShare.SelectionMode.SINGLE,
        previewMode: systemShare.SharePreviewMode.DETAIL
      });
      result.ok = true;
      result.channel = 'system-share';
    } catch (e) {
      hilog.warn(0x1, TAG, 'image share failed');
    }
    return result;
  }

  static async share(ctx: common.UIAbilityContext, card: CardContent): Promise<ShareResult> {
    const text = formatCard(card);
    const result = new ShareResult();

    // Step 1 — always copy locally. Guaranteed path so the user never
    // ends up with nothing on screen, regardless of ShareKit availability.
    try {
      const data = pasteboard.createData(pasteboard.MIMETYPE_TEXT_PLAIN, text);
      const sys = pasteboard.getSystemPasteboard();
      await sys.setData(data);
      result.ok = true;
      result.channel = 'clipboard';
    } catch (e) {
      hilog.warn(0x1, TAG, 'pasteboard failed');
    }

    // Step 2 — open the system share sheet (隔空传送 sits here). We pass
    // inline object literals so the types are inferred from the ShareKit
    // parameter signatures, avoiding hard dependency on specific type names.
    try {
      const shared = new systemShare.SharedData({
        utd: utd.UniformDataType.PLAIN_TEXT,
        content: text,
        title: card.title,
        description: card.description
      });
      const controller = new systemShare.ShareController(shared);
      await controller.show(ctx, {
        selectionMode: systemShare.SelectionMode.SINGLE,
        previewMode: systemShare.SharePreviewMode.DETAIL
      });
      result.ok = true;
      result.channel = 'system-share';
    } catch (e) {
      hilog.warn(0x1, TAG, 'system share unavailable');
    }

    return result;
  }
}
