/**
 * PhoneInfo — live, *real* snapshot of the user's phone hardware.
 *
 * Replaces the "Nebula device" simulated telemetry. We show real values from
 * batteryInfo / deviceInfo / wifiManager / sensor so every number on the
 * Device page is genuinely this device, not a placeholder.
 */

import batteryInfo from '@ohos.batteryInfo';
import deviceInfo from '@ohos.deviceInfo';
import sensor from '@ohos.sensor';
import wifiManager from '@ohos.wifiManager';
import { hilog } from '@kit.PerformanceAnalysisKit';

const TAG = 'PhoneInfo';

export class PhoneSnapshot {
  deviceName: string = '';
  deviceModel: string = '';
  brand: string = '';
  osFullName: string = '';
  deviceType: string = '';

  batteryPercent: number = 0;
  batteryTempC: number = 0;      // real battery temperature
  chargingStatus: string = '';   // "充电中" / "放电中" / "满电" / "未知"
  healthStatus: string = '';     // e.g. "正常" / "过热"

  wifiSsid: string = '';
  wifiRssi: number = 0;
  wifiLinkSpeed: number = 0;
  wifiConnected: boolean = false;

  pressureHpa: number = 0;
  altitudeMeters: number = 0;    // barometric, if barometer supported
  ambientTempC: number = 0;      // if ambient-temperature sensor supported

  hasBarometer: boolean = false;
  hasAmbientTemp: boolean = false;
  sampledAt: number = 0;
}

export type PhoneListener = (snap: PhoneSnapshot) => void;

function chargingLabel(code: number): string {
  // batteryInfo.BatteryChargeState enum values (0=NONE, 1=ENABLE, 2=DISABLE, 3=FULL)
  if (code === 1) return '充电中';
  if (code === 2) return '放电中';
  if (code === 3) return '满电';
  return '未知';
}

function healthLabel(code: number): string {
  // batteryInfo.BatteryHealthState (0=UNKNOWN,1=GOOD,2=OVERHEAT,3=OVERVOLTAGE,4=COLD,5=DEAD)
  if (code === 1) return '正常';
  if (code === 2) return '过热';
  if (code === 3) return '过压';
  if (code === 4) return '偏冷';
  if (code === 5) return '故障';
  return '未知';
}

export class PhoneInfo {
  private static _inst: PhoneInfo | null = null;
  private snapshot: PhoneSnapshot = new PhoneSnapshot();
  private listeners: PhoneListener[] = [];
  private barometerOn: boolean = false;
  private ambientOn: boolean = false;
  private pollTimer: number = -1;

  static get instance(): PhoneInfo {
    if (PhoneInfo._inst === null) PhoneInfo._inst = new PhoneInfo();
    return PhoneInfo._inst as PhoneInfo;
  }

  start(): void {
    // Populate static fields once.
    this.snapshot.deviceName = (deviceInfo.marketName ?? deviceInfo.productModel ?? '').toString();
    this.snapshot.deviceModel = (deviceInfo.productModel ?? '').toString();
    this.snapshot.brand = (deviceInfo.brand ?? '').toString();
    this.snapshot.osFullName = (deviceInfo.osFullName ?? '').toString();
    this.snapshot.deviceType = (deviceInfo.deviceType ?? '').toString();

    this.readBattery();
    this.readWifi();

    // Try barometer
    try {
      sensor.on(sensor.SensorId.BAROMETER, (data: sensor.BarometerResponse): void => {
        this.snapshot.hasBarometer = true;
        this.snapshot.pressureHpa = data.pressure;
        // barometric altitude formula, standard atmosphere:
        // h = 44330 * (1 - (p / 1013.25)^(1/5.255))
        const p = data.pressure;
        if (p > 0) {
          this.snapshot.altitudeMeters = 44330 * (1 - Math.pow(p / 1013.25, 1 / 5.255));
        }
        this.snapshot.sampledAt = Date.now();
        this.emit();
      }, { interval: 500_000_000 });
      this.barometerOn = true;
    } catch (e) {
      hilog.info(0x1, TAG, 'barometer unavailable');
    }

    // Try ambient temperature
    try {
      sensor.on(sensor.SensorId.AMBIENT_TEMPERATURE, (data: sensor.AmbientTemperatureResponse): void => {
        this.snapshot.hasAmbientTemp = true;
        this.snapshot.ambientTempC = data.temperature;
        this.emit();
      }, { interval: 1_000_000_000 });
      this.ambientOn = true;
    } catch (e) {
      hilog.info(0x1, TAG, 'ambient temp unavailable');
    }

    // Poll battery + wifi every 5 s.
    this.pollTimer = setInterval((): void => {
      this.readBattery();
      this.readWifi();
    }, 5000) as number;
  }

  stop(): void {
    if (this.pollTimer >= 0) { clearInterval(this.pollTimer); this.pollTimer = -1; }
    try { if (this.barometerOn) sensor.off(sensor.SensorId.BAROMETER); } catch (e) {}
    try { if (this.ambientOn) sensor.off(sensor.SensorId.AMBIENT_TEMPERATURE); } catch (e) {}
    this.barometerOn = false;
    this.ambientOn = false;
  }

  subscribe(cb: PhoneListener): () => void {
    this.listeners.push(cb);
    cb(this.snapshot);
    return (): void => {
      const i = this.listeners.indexOf(cb);
      if (i >= 0) this.listeners.splice(i, 1);
    };
  }

  snap(): PhoneSnapshot { return this.snapshot; }

  private emit(): void {
    const s = this.snapshot;
    for (const l of this.listeners) { try { l(s); } catch (e) { /* swallow */ } }
  }

  private readBattery(): void {
    try {
      this.snapshot.batteryPercent = batteryInfo.batterySOC;
      this.snapshot.batteryTempC = batteryInfo.batteryTemperature / 10;
      this.snapshot.chargingStatus = chargingLabel(batteryInfo.chargingStatus);
      this.snapshot.healthStatus = healthLabel(batteryInfo.healthStatus);
      this.snapshot.sampledAt = Date.now();
      this.emit();
    } catch (e) {
      hilog.warn(0x1, TAG, 'battery read failed');
    }
  }

  private readWifi(): void {
    try {
      const connected = wifiManager.isConnected();
      this.snapshot.wifiConnected = connected;
      if (connected) {
        wifiManager.getLinkedInfo((err, info) => {
          if (err) return;
          this.snapshot.wifiSsid = info.ssid ?? '';
          this.snapshot.wifiRssi = info.rssi ?? 0;
          this.snapshot.wifiLinkSpeed = info.linkSpeed ?? 0;
          this.emit();
        });
      } else {
        this.snapshot.wifiSsid = '';
        this.snapshot.wifiRssi = 0;
        this.snapshot.wifiLinkSpeed = 0;
        this.emit();
      }
    } catch (e) {
      // Wi-Fi permission may not be granted; stay quiet.
    }
  }
}
