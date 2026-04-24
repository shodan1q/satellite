/**
 * Rocket / satellite catalog. Content comes from the product knowledge base
 * described in app-dev-plan §4.4 and §6. Models are procedurally assembled in
 * the Three.js workshop so we ship zero binary blobs — every part is a spec
 * that maps to a primitive + metadata.
 */

export class SpecLine {
  key: string = '';
  value: string = '';
}

function sl(k: string, v: string): SpecLine {
  const s = new SpecLine();
  s.key = k; s.value = v;
  return s;
}

export class PartSpec {
  id: string = '';
  name: string = '';
  display: string = '';
  kind: string = 'stage';
  height: number = 1;
  radiusTop: number = 0.5;
  radiusBottom: number = 0.5;
  color: string = '#FFFFFF';
  accent: string = '#2E6BF6';
  shortDesc: string = '';
  fullDesc: string = '';
  specs: SpecLine[] = [];
}

function mk(id: string, name: string, display: string, kind: string,
            h: number, rT: number, rB: number, color: string, accent: string,
            shortDesc: string, fullDesc: string, specs: SpecLine[]): PartSpec {
  const p = new PartSpec();
  p.id = id; p.name = name; p.display = display; p.kind = kind;
  p.height = h; p.radiusTop = rT; p.radiusBottom = rB;
  p.color = color; p.accent = accent;
  p.shortDesc = shortDesc; p.fullDesc = fullDesc; p.specs = specs;
  return p;
}

export class ExplodeOffset {
  partId: string = '';
  dx: number = 0;
  dy: number = 0;
  dz: number = 0;
  delay: number = 0;
}

function off(id: string, dx: number, dy: number, dz: number, delay: number): ExplodeOffset {
  const o = new ExplodeOffset();
  o.partId = id; o.dx = dx; o.dy = dy; o.dz = dz; o.delay = delay;
  return o;
}

export class RocketBlueprint {
  id: string = '';
  name: string = '';
  display: string = '';
  agency: string = '';
  tags: string[] = [];
  summary: string = '';
  hero: string = '#2E6BF6';
  parts: PartSpec[] = [];
  explodeOffsets: ExplodeOffset[] = [];
  // Optional path to a real glb model (relative to ar.html at rawfile/three/).
  // When set, the viewer loads this instead of procedural primitives.
  modelUrl: string = '';
  modelCredit: string = '';
}

export class SatelliteBlueprint {
  id: string = '';
  name: string = '';
  display: string = '';
  orbit: string = '';
  tags: string[] = [];
  summary: string = '';
  parts: PartSpec[] = [];
  explodeOffsets: ExplodeOffset[] = [];
  modelUrl: string = '';
  modelCredit: string = '';
}

export class Catalog {
  static rockets(): RocketBlueprint[] {
    return [saturnV(), spaceShuttle(), ares1(), falcon9(), longMarch5()];
  }

  static satellites(): SatelliteBlueprint[] {
    // Custom-sat comes first — Workshop opens on it so users start with
    // a procedural stack they can actually add to / remove from.
    return [customSat(), hubble(), iss(), cassini(), chandra(), apolloLm(), beidou()];
  }

  static rocketById(id: string): RocketBlueprint | null {
    for (const r of Catalog.rockets()) {
      if (r.id === id) return r;
    }
    return null;
  }

  static satelliteById(id: string): SatelliteBlueprint | null {
    for (const s of Catalog.satellites()) {
      if (s.id === id) return s;
    }
    return null;
  }
}

function saturnV(): RocketBlueprint {
  const bp = new RocketBlueprint();
  bp.id = 'saturn-v';
  bp.name = 'Saturn V';
  bp.display = '土星五号';
  bp.agency = 'NASA · 1967';
  bp.tags = ['超重型', '阿波罗计划', '载人登月'];
  bp.summary = '人类迄今推力最强的成功运载火箭，送 12 人登月并返回。';
  bp.hero = '#E9A23B';
  bp.parts = [
    mk('cm', 'command-module', '指令舱', 'nose', 0.9, 0.05, 0.42, '#F2F5FA', '#B9C3D6',
      '宇航员居住舱，唯一返回地球的部分',
      '指令舱承载三名宇航员完成地月转移、月面降落前的集结与返回地球。',
      [sl('乘员', '3 人'), sl('质量', '5,557 kg'), sl('热盾', 'AVCOAT 烧蚀材料')]),
    mk('sm', 'service-module', '服务舱', 'payload', 1.2, 0.55, 0.55, '#DDE4F0', '#8FA1C2',
      '提供电力、氧气与主推进',
      '服务舱搭载 SPS 主推进、燃料电池以及生命维持消耗品。',
      [sl('推力', '91 kN'), sl('燃料', '偏二甲肼 / 四氧化二氮'), sl('电池', '3 枚燃料电池')]),
    mk('iu', 'instrument-unit', '仪器舱', 'interstage', 0.4, 0.75, 0.75, '#6F7A99', '#2E6BF6',
      '制导与控制大脑',
      'IBM 为土星五号设计的制导计算机，负责从点火到入轨的实时轨迹控制。',
      [sl('制导', 'IBM LVDC 计算机'), sl('高度', '0.91 m'), sl('直径', '6.6 m')]),
    mk('s4b', 'stage-3', '三级 S-IVB', 'stage', 1.6, 0.75, 0.75, '#F7FAFD', '#2E6BF6',
      'J-2 单引擎，负责地月转移',
      'S-IVB 在二级分离后点火，再重新点火完成地月转移喷射。',
      [sl('推力', '1,000 kN'), sl('燃料', 'LOX / LH2'), sl('发动机', 'Rocketdyne J-2 ×1')]),
    mk('s2', 'stage-2', '二级 S-II', 'stage', 2.4, 1.05, 1.05, '#E8EDF6', '#11B7D6',
      '5 台 J-2 液氢氧发动机',
      '二级使用液氢液氧产生近真空高比冲，把组合体推至近地轨道高度。',
      [sl('推力', '5,141 kN'), sl('直径', '10 m'), sl('发动机', 'J-2 ×5')]),
    mk('s1c', 'stage-1', '一级 S-IC', 'stage', 3.0, 1.05, 1.05, '#FBFCFE', '#E9A23B',
      '5 台 F-1，将巨兽送出大气',
      '一级使用世界最强单台液体发动机 F-1，燃烧煤油与液氧。',
      [sl('推力', '33,400 kN'), sl('燃料', 'RP-1 / LOX'), sl('发动机', 'F-1 ×5')]),
    mk('eng1', 'engines', '发动机阵列', 'engine', 0.45, 1.1, 1.25, '#B0B9CC', '#E5554A',
      'F-1 发动机裙摆',
      '五台 F-1 构成的发动机裙摆，中心一台固定、四周四台摆动。',
      [sl('比冲', '263 s（海平面）'), sl('质量流', '2,578 kg/s'), sl('工作时间', '150 s')])
  ];
  bp.explodeOffsets = [
    off('cm', 0, 2.4, 0, 0),
    off('sm', 0, 1.6, 0, 0.05),
    off('iu', 0, 1.0, 0, 0.1),
    off('s4b', 0, 0.5, 0, 0.15),
    off('s2', 0, -0.2, 0, 0.2),
    off('s1c', 0, -0.9, 0, 0.25),
    off('eng1', 0, -1.6, 0, 0.3)
  ];
  bp.modelUrl = 'saturn-v.glb';
  bp.modelCredit = 'NASA 3D Resources (public domain)';
  return bp;
}

function falcon9(): RocketBlueprint {
  const bp = new RocketBlueprint();
  bp.id = 'falcon-9';
  bp.name = 'Falcon 9';
  bp.display = '猎鹰 9';
  bp.agency = 'SpaceX · 2010';
  bp.tags = ['可复用', '中型', '商业发射'];
  bp.summary = '世界首款成功商用复用运载火箭，重写火箭成本曲线。';
  bp.hero = '#2E6BF6';
  bp.parts = [
    mk('fair', 'fairing', '整流罩', 'fairing', 1.0, 0.02, 0.42, '#F8FAFE', '#11B7D6',
      '可回收双瓣整流罩',
      'Falcon 9 的整流罩分两瓣打开、分离，之后回收。',
      [sl('高度', '13.1 m'), sl('直径', '5.2 m'), sl('回收', '海上网兜')]),
    mk('pay', 'payload', '载荷', 'payload', 0.7, 0.32, 0.32, '#DCE3F1', '#7F5CFF',
      '卫星或补给飞船',
      'Falcon 9 可把 22.8 吨送入 LEO。',
      [sl('LEO 运力', '22.8 t'), sl('GTO 运力', '8.3 t')]),
    mk('s2', 'stage-2', '二级', 'stage', 1.4, 0.55, 0.55, '#F7FAFD', '#2E6BF6',
      '单 Merlin Vacuum',
      '二级使用一台 MVac 发动机完成入轨与轨道机动。',
      [sl('推力', '981 kN（真空）'), sl('比冲', '348 s'), sl('重启', '是')]),
    mk('inter', 'interstage', '级间段', 'interstage', 0.4, 0.72, 0.72, '#44506E', '#E9A23B',
      '碳复合材料分离结构',
      '级间段采用碳复合材料，集成分离机构与栅格翼作动器连接点。',
      [sl('材料', 'Carbon Composite'), sl('分离', '气动冲压')]),
    mk('s1', 'stage-1', '一级（可回收）', 'stage', 3.2, 0.72, 0.72, '#FBFCFE', '#2E6BF6',
      '9 台 Merlin 可回收',
      '一级使用 9 台 Merlin 1D+，通过栅格翼与着陆支架回收。',
      [sl('发动机', 'Merlin 1D+ ×9'), sl('总推力', '7,607 kN'), sl('复用次数', '20+')]),
    mk('legs', 'landing-legs', '着陆支架 / 栅格翼', 'engine', 0.35, 0.8, 1.1, '#98A3BD', '#E5554A',
      '可折叠腿 + 气动控制面',
      '四条液压着陆腿、四片钛合金栅格翼。',
      [sl('着陆腿', '4 条可折叠'), sl('栅格翼', '钛合金')])
  ];
  bp.explodeOffsets = [
    off('fair', 0, 2.2, 0, 0),
    off('pay', 0, 1.4, 0, 0.05),
    off('s2', 0, 0.8, 0, 0.1),
    off('inter', 0, 0.3, 0, 0.15),
    off('s1', 0, -0.6, 0, 0.2),
    off('legs', 0, -1.6, 0, 0.25)
  ];
  // Until a public-domain Falcon 9 mesh exists, we render the real
  // Atlas LV-3B (Mercury "Friendship 7") from the NASA 3D Resources
  // repository — a genuine medium-class two-stage launcher.
  bp.modelUrl = 'atlas-6.glb';
  bp.modelCredit = 'NASA 3D Resources — Atlas LV-3B (Friendship 7)';
  return bp;
}

function longMarch5(): RocketBlueprint {
  const bp = new RocketBlueprint();
  bp.id = 'long-march-5';
  bp.name = 'Long March 5';
  bp.display = '长征五号';
  bp.agency = 'CASC · 2016';
  bp.tags = ['大型', '深空任务', '国产'];
  bp.summary = '中国最强运载火箭，托举嫦娥五号、天问一号、天和核心舱等重大任务。';
  bp.hero = '#E5554A';
  bp.parts = [
    mk('fair', 'fairing', '整流罩', 'fairing', 1.2, 0.04, 0.55, '#F2F5FA', '#E5554A',
      '5.2 米级冯卡门整流罩',
      '长征五号的整流罩直径 5.2 米，是中国目前最大级整流罩。',
      [sl('直径', '5.2 m'), sl('高度', '12.3 m')]),
    mk('pay', 'payload', '载荷（嫦娥 / 天问）', 'payload', 0.9, 0.4, 0.4, '#DDE4F0', '#7F5CFF',
      '深空探测器与空间站舱段',
      '承担过嫦娥五号、天问一号、天和核心舱等任务。',
      [sl('LEO 运力', '25 t'), sl('GTO 运力', '14 t')]),
    mk('s2', 'stage-2', '芯二级', 'stage', 1.6, 0.72, 0.72, '#F7FAFD', '#2E6BF6',
      'YF-75D 双机氢氧',
      '芯二级使用两台 YF-75D 膨胀循环氢氧发动机。',
      [sl('推力', '180 kN'), sl('比冲', '442 s')]),
    mk('s1', 'stage-1', '芯一级', 'stage', 3.0, 0.72, 0.72, '#FFFFFF', '#E9A23B',
      '两台 YF-77 氢氧',
      '芯一级直径 5 米，使用两台 YF-77 氢氧发动机。',
      [sl('直径', '5 m'), sl('发动机', 'YF-77 ×2')]),
    mk('boost-l', 'booster-left', '助推器 · 左', 'booster', 2.6, 0.35, 0.35, '#F8FAFE', '#11B7D6',
      'YF-100 煤油液氧',
      '每个助推器使用两台 YF-100 高压分级燃烧循环发动机。',
      [sl('数量', '4 枚'), sl('发动机', 'YF-100 ×2/枚')]),
    mk('boost-r', 'booster-right', '助推器 · 右', 'booster', 2.6, 0.35, 0.35, '#F8FAFE', '#11B7D6',
      'YF-100 煤油液氧',
      '对称安装的助推器。',
      [sl('数量', '4 枚'), sl('推力', '1,200 kN/枚')]),
    mk('eng', 'engine-cluster', '发动机裙摆', 'engine', 0.35, 0.75, 0.9, '#A3AEC6', '#E5554A',
      'YF-77 + YF-100 混合',
      '芯一级底部发动机裙摆整合姿态控制与摆动喷管。',
      [sl('控制', '双轴摆动喷管')])
  ];
  bp.explodeOffsets = [
    off('fair', 0, 2.2, 0, 0),
    off('pay', 0, 1.4, 0, 0.05),
    off('s2', 0, 0.6, 0, 0.1),
    off('s1', 0, -0.6, 0, 0.15),
    off('boost-l', -1.4, -0.3, 0, 0.2),
    off('boost-r', 1.4, -0.3, 0, 0.2),
    off('eng', 0, -1.6, 0, 0.25)
  ];
  // Stand-in until a public-domain Long March 5 mesh exists. We use
  // the real Jupiter-C (Juno I) rocket from NASA 3D Resources — an
  // authentic multi-stage launcher, distinct from Saturn V.
  bp.modelUrl = 'jupiter-c.glb';
  bp.modelCredit = 'NASA 3D Resources — Jupiter-C / Juno I';
  return bp;
}

function hubble(): SatelliteBlueprint {
  const bp = new SatelliteBlueprint();
  bp.id = 'hubble';
  bp.name = 'Hubble';
  bp.display = '哈勃空间望远镜';
  bp.orbit = 'LEO · 535 km';
  bp.tags = ['天文台', 'NASA / ESA', '可维修'];
  bp.summary = '人类最著名的空间望远镜，30 年来重塑了我们对宇宙的认知。';
  bp.parts = [
    mk('door', 'aperture-door', '光学舱门', 'nose', 0.3, 0.55, 0.55, '#E8EDF6', '#11B7D6',
      '阻挡杂散光的前端舱门',
      '进入太阳直射区时自动关闭，保护主镜。',
      [sl('工作', '地面遥控开合'), sl('材料', '复合蜂窝板')]),
    mk('tube', 'optical-tube', '光学筒（2.4m 主镜）', 'bus', 2.0, 0.55, 0.55, '#F8FAFE', '#7F5CFF',
      '2.4 米 Ritchey–Chrétien 望远镜',
      '主镜由超低热膨胀玻璃制成，镀铝表面。',
      [sl('口径', '2.4 m'), sl('光学', 'Ritchey–Chrétien'), sl('分辨率', '0.05 角秒')]),
    mk('bay', 'instrument-bay', '仪器舱', 'bus', 1.0, 0.55, 0.55, '#DDE4F0', '#2E6BF6',
      'WFC3 / COS / STIS / ACS 仪器',
      '仪器舱内有宽视场相机、紫外光谱仪等可更换设备。',
      [sl('可替换', '是'), sl('波段', '紫外 / 可见 / 近红外')]),
    mk('sa-l', 'solar-left', '太阳能板 · 左', 'solar', 2.6, 0.02, 0.9, '#1F3A8A', '#7F5CFF',
      '可卷曲砷化镓阵列',
      '第三次维修任务后更换为硬质太阳能板。',
      [sl('输出', '2,800 W'), sl('电池', 'Ni-H₂')]),
    mk('sa-r', 'solar-right', '太阳能板 · 右', 'solar', 2.6, 0.02, 0.9, '#1F3A8A', '#7F5CFF',
      '可卷曲砷化镓阵列',
      '与左翼对称，确保姿态稳定。',
      [sl('面积', '2 × 12 m²')]),
    mk('ha', 'hga', '高增益天线', 'antenna', 0.8, 0.02, 0.25, '#FBFCFE', '#E9A23B',
      '中继数据下行',
      '通过 TDRSS 中继卫星把科学数据下行至地面。',
      [sl('数据率', '1 Mbit/s'), sl('频段', 'S / Ku')])
  ];
  bp.explodeOffsets = [
    off('door', 0, 1.6, 0, 0),
    off('tube', 0, 0.7, 0, 0.05),
    off('bay', 0, -0.5, 0, 0.1),
    off('sa-l', -1.8, 0, 0, 0.15),
    off('sa-r', 1.8, 0, 0, 0.15),
    off('ha', 0, 0, 1.3, 0.2)
  ];
  bp.modelUrl = 'hubble.glb';
  bp.modelCredit = 'NASA 3D Resources (public domain)';
  return bp;
}

function iss(): SatelliteBlueprint {
  const bp = new SatelliteBlueprint();
  bp.id = 'iss';
  bp.name = 'ISS';
  bp.display = '国际空间站';
  bp.orbit = 'LEO · 400 km';
  bp.tags = ['多国合作', '载人', '轨道实验室'];
  bp.summary = '人类目前在轨最大航天器，5 国 15 方共建。';
  bp.parts = [
    mk('zarya', 'zarya', '曙光号功能货舱', 'bus', 1.2, 0.45, 0.45, '#F7FAFD', '#2E6BF6',
      'ISS 最早的模块（1998）',
      '由俄罗斯制造、美国出资。',
      [sl('发射', '1998-11-20'), sl('长度', '12.5 m')]),
    mk('unity', 'unity', '团结号节点舱', 'bus', 1.0, 0.45, 0.45, '#DDE4F0', '#11B7D6',
      '第一个美国舱段',
      '连接节点舱，是 ISS 早期的枢纽。',
      [sl('对接口', '6 个')]),
    mk('destiny', 'destiny', '命运号实验舱', 'bus', 1.4, 0.5, 0.5, '#FBFCFE', '#7F5CFF',
      '美国主实验舱',
      '承载多个微重力研究机架。',
      [sl('实验机架', '24 个')]),
    mk('truss', 'truss', '桁架结构', 'bus', 0.5, 1.6, 1.6, '#9AA6C0', '#E9A23B',
      '主桁架',
      '长 109 米，承载太阳能板、散热器和机械臂。',
      [sl('长度', '109 m')]),
    mk('solar-a', 'solar-array-a', '太阳能板阵列 · A', 'solar', 0.08, 2.4, 2.4, '#1F3A8A', '#7F5CFF',
      '8 对太阳能帆板',
      'ISS 共 8 对太阳能帆板。',
      [sl('总功率', '240 kW')]),
    mk('solar-b', 'solar-array-b', '太阳能板阵列 · B', 'solar', 0.08, 2.4, 2.4, '#1F3A8A', '#7F5CFF',
      '与 A 对称',
      '总面积 2,500 m²。',
      [sl('转速', 'SARJ 1 rpm')])
  ];
  bp.explodeOffsets = [
    off('zarya', 0, 1.6, 0, 0),
    off('unity', 0, 0.4, 0, 0.05),
    off('destiny', 0, -1.0, 0, 0.1),
    off('truss', 0, 0, 1.4, 0.15),
    off('solar-a', -2.6, 0, 0, 0.2),
    off('solar-b', 2.6, 0, 0, 0.2)
  ];
  bp.modelUrl = 'iss.glb';
  bp.modelCredit = 'NASA 3D Resources (public domain)';
  return bp;
}

/**
 * Blank-canvas satellite for the AR Workshop home. No GLB — everything
 * renders procedurally so the user can add / remove parts freely.
 * Starts with a bare bus + solar wings + a single antenna so the build
 * looks like a satellite from the first frame; the palette provides
 * the rest (extra antennas, thrusters, payloads) for user assembly.
 */
function customSat(): SatelliteBlueprint {
  const bp = new SatelliteBlueprint();
  bp.id = 'custom-sat';
  bp.name = 'Custom Satellite';
  bp.display = '自制卫星';
  bp.orbit = '自定义轨道';
  bp.tags = ['自制', '可搭建', '教学'];
  bp.summary = '从一个平台 + 太阳翼开始，按需添加天线 / 载荷 / 发动机，搭出属于自己的卫星。';
  bp.parts = [
    mk('bus', 'bus', '平台舱', 'bus', 2.2, 0.7, 0.7, '#F6F8FB', '#2E6BF6',
      '卫星主体平台',
      '承载电源、姿控、推进与载荷，是整星的中枢。',
      [sl('尺寸', '~1.5 m 立方'), sl('质量', '≈ 800 kg')]),
    mk('ant-top', 'antenna', '高增益天线', 'antenna', 1.2, 0.08, 0.4, '#F8FAFE', '#E9A23B',
      '指向地球的数据天线',
      '用于高码率对地下行。',
      [sl('频段', 'Ka')]),
    mk('sa-l', 'solar-left', '太阳翼 · 左', 'solar', 3.2, 0.04, 0.85, '#1F3A8A', '#7F5CFF',
      '三结砷化镓太阳能板',
      '展开对日取电，为整星供能。',
      [sl('功率', '~2 kW')]),
    mk('sa-r', 'solar-right', '太阳翼 · 右', 'solar', 3.2, 0.04, 0.85, '#1F3A8A', '#7F5CFF',
      '三结砷化镓太阳能板',
      '与左翼对称。',
      [sl('对日精度', '±1°')])
  ];
  bp.explodeOffsets = [
    off('bus', 0, 0, 0, 0),
    off('ant-top', 0, 1.0, 0, 0.1),
    off('sa-l', -1.8, 0, 0, 0.15),
    off('sa-r', 1.8, 0, 0, 0.15)
  ];
  // No modelUrl — Workshop falls through to the procedural stack so the
  // part palette / add / remove actions operate on what's rendered.
  bp.modelUrl = '';
  bp.modelCredit = '';
  return bp;
}

function beidou(): SatelliteBlueprint {
  const bp = new SatelliteBlueprint();
  bp.id = 'beidou-3';
  bp.name = 'BeiDou-3';
  bp.display = '北斗三号';
  bp.orbit = 'MEO · 21,528 km';
  bp.tags = ['导航', '国产', '短报文'];
  bp.summary = '中国自主的全球卫星导航系统，实现厘米级 RTK 服务。';
  bp.parts = [
    mk('bus', 'bus', '平台（DFH-3B）', 'bus', 1.0, 0.55, 0.55, '#FBFCFE', '#2E6BF6',
      '中国自主研发导航卫星平台',
      '北斗三号基于东方红三号 B 卫星平台，寿命 12 年。',
      [sl('平台', 'DFH-3B'), sl('质量', '1,060 kg')]),
    mk('l-ant', 'nav-antenna', '导航载荷天线', 'antenna', 0.4, 0.28, 0.32, '#F8FAFE', '#E9A23B',
      'B1/B2/B3 多频点导航信号',
      '播发 B1C/B1I/B2a/B3I 多频点导航信号。',
      [sl('频点', 'B1C/B1I/B2a/B3I')]),
    mk('s-ant', 'short-message', '短报文天线', 'antenna', 0.5, 0.02, 0.2, '#F7FAFD', '#11B7D6',
      '北斗独有短报文通讯',
      '在通信盲区仍可用。',
      [sl('容量', '1000 汉字/次')]),
    mk('sa-l', 'solar-left', '太阳能板 · 左', 'solar', 2.0, 0.02, 0.6, '#1F3A8A', '#7F5CFF',
      '三结砷化镓太阳能板',
      '常年对日定向。',
      [sl('功率', '2,300 W（末期）')]),
    mk('sa-r', 'solar-right', '太阳能板 · 右', 'solar', 2.0, 0.02, 0.6, '#1F3A8A', '#7F5CFF',
      '三结砷化镓太阳能板',
      '与左翼对称。',
      [sl('寿命', '12 年')])
  ];
  bp.explodeOffsets = [
    off('bus', 0, 0, 0, 0),
    off('l-ant', 0, -1.2, 0, 0.1),
    off('s-ant', 0, 1.2, 0, 0.1),
    off('sa-l', -2.0, 0, 0, 0.15),
    off('sa-r', 2.0, 0, 0, 0.15)
  ];
  // Stand-in until a public-domain BeiDou-3 mesh exists. We render
  // ESA's Galileo navigation satellite (NASA 3D Resources mirror) —
  // a genuine MEO GNSS bird in the same class and silhouette as
  // BeiDou-3 (cuboid bus + two rectangular solar wings + L-band
  // antenna array).
  bp.modelUrl = 'galileo.glb';
  bp.modelCredit = 'NASA 3D Resources — Galileo GNSS';
  return bp;
}

function spaceShuttle(): RocketBlueprint {
  const bp = new RocketBlueprint();
  bp.id = 'space-shuttle';
  bp.name = 'Space Shuttle';
  bp.display = '航天飞机';
  bp.agency = 'NASA · 1981-2011';
  bp.tags = ['部分可重复使用', '载人', 'LEO'];
  bp.summary = '世界首款部分可重复使用的载人运载器，完成 135 次飞行、修复哈勃与建造 ISS。';
  bp.hero = '#2E6BF6';
  bp.parts = [
    mk('orb', 'orbiter', '轨道器', 'bus', 1.6, 0.55, 0.55, '#F8FAFE', '#2E6BF6',
      '机组 2-8 人的主飞行体',
      '有效载荷舱长 18.3 m、直径 4.6 m，机翼为三角翼设计。',
      [sl('乘员', '2-8 人'), sl('载荷舱长度', '18.3 m'), sl('推进', '3 × SSME')]),
    mk('et', 'external-tank', '外挂燃料箱', 'stage', 3.2, 0.7, 0.7, '#FBFCFE', '#E9A23B',
      '液氢 / 液氧总燃料箱',
      '不可回收，入轨前数分钟抛弃。',
      [sl('推进剂', 'LH2 / LOX'), sl('高度', '47 m'), sl('直径', '8.4 m')]),
    mk('srb-l', 'srb-left', '固体助推器 · 左', 'booster', 2.4, 0.32, 0.32, '#F7FAFD', '#E5554A',
      'SRB 双固体', '起飞时提供 71% 的总推力，挂伞入海回收。',
      [sl('推力', '14.7 MN / 枚'), sl('工作时间', '124 s')]),
    mk('srb-r', 'srb-right', '固体助推器 · 右', 'booster', 2.4, 0.32, 0.32, '#F7FAFD', '#E5554A',
      'SRB 双固体', '与左侧对称。', [sl('数量', '2')]),
    mk('sse', 'ssme', 'SSME 主发动机', 'engine', 0.4, 0.8, 1.0, '#B0B9CC', '#2E6BF6',
      '三台主发动机',
      '可复用的高性能氢氧发动机。',
      [sl('数量', 'SSME × 3'), sl('比冲', '452 s（真空）')])
  ];
  bp.explodeOffsets = [
    off('orb', 0, 1.0, 0.9, 0),
    off('et', 0, -0.4, 0, 0.05),
    off('srb-l', -1.2, -0.5, 0, 0.1),
    off('srb-r', 1.2, -0.5, 0, 0.1),
    off('sse', 0, -1.4, 0, 0.15)
  ];
  bp.modelUrl = 'space-shuttle.glb';
  bp.modelCredit = 'NASA 3D Resources (public domain)';
  return bp;
}

function ares1(): RocketBlueprint {
  const bp = new RocketBlueprint();
  bp.id = 'ares-1';
  bp.name = 'Ares I';
  bp.display = 'Ares I';
  bp.agency = 'NASA 星座计划 · 2004-2010';
  bp.tags = ['载人', '一次性', '已取消'];
  bp.summary = '原计划取代航天飞机把猎户座飞船送入轨道的星座计划运载器（2010 年取消）。';
  bp.hero = '#7F5CFF';
  bp.parts = [
    mk('cm', 'orion', '猎户座飞船', 'payload', 1.0, 0.4, 0.45, '#F2F5FA', '#2E6BF6',
      '原计划载 4-6 名宇航员',
      '后续演化为 Artemis 计划使用的猎户座乘员舱。',
      [sl('乘员', '4 人'), sl('质量', '8,900 kg')]),
    mk('us', 'upper-stage', '第二级', 'stage', 1.4, 0.55, 0.55, '#F7FAFD', '#2E6BF6',
      'J-2X 氢氧引擎',
      '衍生自阿波罗的 J-2 单引擎上面级。',
      [sl('发动机', 'J-2X ×1'), sl('推进剂', 'LH2 / LOX')]),
    mk('srb', 'first-stage', '固体燃料一级', 'stage', 2.6, 0.55, 0.55, '#DDE4F0', '#E5554A',
      '五段式 SRB',
      '衍生自航天飞机 SRB 并加长一段。',
      [sl('段数', '5 段'), sl('推力', '16 MN（海平面）')])
  ];
  bp.explodeOffsets = [
    off('cm', 0, 1.6, 0, 0),
    off('us', 0, 0.6, 0, 0.1),
    off('srb', 0, -1.0, 0, 0.15)
  ];
  bp.modelUrl = 'ares-1.glb';
  bp.modelCredit = 'NASA 3D Resources (public domain)';
  return bp;
}

function cassini(): SatelliteBlueprint {
  const bp = new SatelliteBlueprint();
  bp.id = 'cassini';
  bp.name = 'Cassini-Huygens';
  bp.display = '卡西尼-惠更斯号';
  bp.orbit = '土星轨道 · 1997-2017';
  bp.tags = ['行星际', 'NASA/ESA/ASI', '已结束'];
  bp.summary = '20 年太阳系外行星任务，发现土卫六液态甲烷湖、土星 E 环源自土卫二间歇泉。';
  bp.parts = [
    mk('hga', 'hga', '高增益天线', 'antenna', 0.4, 0.05, 0.6, '#FBFCFE', '#E9A23B',
      '4 m 双频段碟形天线',
      '负责与地球深空网络通讯，也充当尘埃屏蔽。',
      [sl('直径', '4 m'), sl('频段', 'X / Ka')]),
    mk('bus', 'main-bus', '主舱体', 'bus', 1.4, 0.5, 0.5, '#F8FAFE', '#2E6BF6',
      '仪器平台 + 燃料',
      '装载 12 台科学仪器和 3 t 联胺 / 四氧化二氮推进剂。',
      [sl('仪器', '12 台'), sl('总质量', '5,712 kg')]),
    mk('rtg', 'rtg', 'RTG 核电池', 'payload', 0.7, 0.15, 0.18, '#6F7A99', '#E5554A',
      '钚-238 放射性同位素热电机',
      '因外行星光照不足而使用 3 颗 RTG 供电。',
      [sl('数量', 'RTG × 3'), sl('末期功率', '631 W')]),
    mk('huy', 'huygens', '惠更斯探测器', 'payload', 0.5, 0.45, 0.55, '#DDE4F0', '#7F5CFF',
      '由 ESA 研制的登陆器',
      '2005 年分离并成功登陆土卫六, 成为最远天体上的人造着陆器。',
      [sl('制造方', 'ESA'), sl('着陆日', '2005-01-14')])
  ];
  bp.explodeOffsets = [
    off('hga', 0, 1.4, 0, 0),
    off('bus', 0, 0.2, 0, 0.05),
    off('rtg', -0.9, 0, 0, 0.1),
    off('huy', 0.9, -0.3, 0, 0.15)
  ];
  bp.modelUrl = 'cassini.glb';
  bp.modelCredit = 'NASA 3D Resources (public domain)';
  return bp;
}

function chandra(): SatelliteBlueprint {
  const bp = new SatelliteBlueprint();
  bp.id = 'chandra';
  bp.name = 'Chandra X-ray Observatory';
  bp.display = '钱德拉 X 射线天文台';
  bp.orbit = 'HEO · 16,000-133,000 km';
  bp.tags = ['大观测台', 'X 射线', 'NASA'];
  bp.summary = 'NASA 四大观测台之一，通过斜视成像研究超新星残骸、黑洞与星系团。';
  bp.parts = [
    mk('hrma', 'hrma', '高分辨率镜面组件', 'bus', 1.2, 0.55, 0.55, '#F8FAFE', '#2E6BF6',
      '4 组嵌套斜视镜',
      '抛物面 + 双曲面组合，角分辨率 0.5 角秒。',
      [sl('角分辨率', '0.5 arcsec'), sl('镜面数', '8')]),
    mk('acis', 'acis', 'ACIS 成像仪', 'bus', 0.6, 0.35, 0.35, '#DDE4F0', '#7F5CFF',
      'CCD X 射线成像仪',
      '测量光子能量并成像。',
      [sl('阵列', 'ACIS-I / ACIS-S')]),
    mk('hrc', 'hrc', 'HRC 相机', 'bus', 0.6, 0.3, 0.3, '#E8EDF6', '#11B7D6',
      '微通道板探测器',
      '提供最佳角分辨率成像。',
      [sl('视野', '30 角分')]),
    mk('sa-l', 'solar-l', '太阳能板 · 左', 'solar', 2.2, 0.05, 0.7, '#1F3A8A', '#7F5CFF',
      '砷化镓三结电池',
      '寿命 5 年（实际已超 25 年）。',
      [sl('寿命预期', '5 y')]),
    mk('sa-r', 'solar-r', '太阳能板 · 右', 'solar', 2.2, 0.05, 0.7, '#1F3A8A', '#7F5CFF',
      '与左翼对称', '对称太阳帆。', [sl('总功率', '2,350 W')])
  ];
  bp.explodeOffsets = [
    off('hrma', 0, 1.0, 0, 0),
    off('acis', 0, 0, 1.2, 0.1),
    off('hrc', 0, 0, -1.2, 0.15),
    off('sa-l', -2.2, 0, 0, 0.2),
    off('sa-r', 2.2, 0, 0, 0.2)
  ];
  bp.modelUrl = 'chandra.glb';
  bp.modelCredit = 'NASA 3D Resources (public domain)';
  return bp;
}

function apolloLm(): SatelliteBlueprint {
  const bp = new SatelliteBlueprint();
  bp.id = 'apollo-lm';
  bp.name = 'Apollo Lunar Module';
  bp.display = '阿波罗登月舱';
  bp.orbit = '月球表面 · 1969-1972';
  bp.tags = ['登月', '载人', '阿波罗'];
  bp.summary = '双段式月球着陆器，将 12 名宇航员送上月表并全部成功返回月球轨道。';
  bp.parts = [
    mk('asc', 'ascent-stage', '上升段', 'bus', 1.0, 0.7, 0.8, '#F8FAFE', '#E9A23B',
      '宇航员舱 + 上升发动机',
      '任务结束后带人员点火返回月球轨道与指挥舱对接。',
      [sl('乘员', '2 人'), sl('上升推力', '15.6 kN')]),
    mk('dec', 'descent-stage', '下降段', 'bus', 1.0, 0.9, 1.1, '#DDE4F0', '#E5554A',
      '下降发动机 + 着陆四腿',
      '节流比可达 10:1，支持精确悬停下降。',
      [sl('节流范围', '10%-92%'), sl('推力上限', '45.04 kN')]),
    mk('legs', 'landing-legs', '着陆四腿', 'engine', 0.3, 1.1, 1.4, '#98A3BD', '#2E6BF6',
      '蜂窝结构缓冲',
      '可压缩铝蜂窝吸收着陆冲击能量。',
      [sl('触地探针长度', '1.73 m')]),
    mk('ant', 'hga', '高增益天线', 'antenna', 0.6, 0.04, 0.28, '#FBFCFE', '#E9A23B',
      '月地通讯',
      '工作于 S 波段，可将图像和语音直传地球。',
      [sl('频段', 'S')])
  ];
  bp.explodeOffsets = [
    off('asc', 0, 1.1, 0, 0),
    off('dec', 0, -0.2, 0, 0.1),
    off('legs', 0, -1.0, 0, 0.15),
    off('ant', 0, 0.8, 0.9, 0.2)
  ];
  bp.modelUrl = 'apollo-lm.glb';
  bp.modelCredit = 'NASA 3D Resources (public domain)';
  return bp;
}
