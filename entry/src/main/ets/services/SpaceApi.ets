/**
 * SpaceApi — live data feeds used by the Discover surface.
 *
 * Three free, no-auth endpoints:
 *   - wheretheiss.at                     → live ISS position/velocity/altitude
 *   - ll.thespacedevs.com (Launch Lib 2) → upcoming orbital launches
 *   - api.sunrise-sunset.org             → today's sunrise/sunset for a point
 *
 * All requests go through @ohos.net.http with sensible timeouts. Callers
 * receive typed classes or null/empty arrays on failure — nothing is faked.
 */

import http from '@ohos.net.http';
import { hilog } from '@kit.PerformanceAnalysisKit';

const TAG = 'SpaceApi';

export class IssPosition {
  latitude: number = 0;
  longitude: number = 0;
  altitudeKm: number = 0;
  velocityKmh: number = 0;
  visibility: string = '';
  timestamp: number = 0;

  velocityKmS(): number {
    return this.velocityKmh / 3600;
  }

  // Approximate orbital period (minutes) from altitude using the vis-viva
  // equation for a circular orbit: T = 2π·√(r³/μ).
  orbitalPeriodMin(): number {
    const rE = 6371;          // earth radius km
    const mu = 398600.4418;   // standard gravitational parameter km^3/s^2
    const r = rE + this.altitudeKm;
    const T = 2 * Math.PI * Math.sqrt((r * r * r) / mu); // seconds
    return T / 60;
  }
}

export class LaunchLink {
  title: string = '';
  url: string = '';
}

export class LaunchInfo {
  id: string = '';
  name: string = '';
  rocketName: string = '';
  rocketFamily: string = '';
  rocketDescription: string = '';
  rocketImage: string = '';
  net: Date = new Date();
  windowStart: Date = new Date();
  windowEnd: Date = new Date();
  padName: string = '';
  locationName: string = '';
  locationCountry: string = '';
  agency: string = '';
  agencyAbbrev: string = '';
  status: string = '';
  statusDescription: string = '';
  image: string = '';
  missionName: string = '';
  missionDescription: string = '';
  missionType: string = '';
  orbitName: string = '';
  vidURLs: LaunchLink[] = [];
  infoURLs: LaunchLink[] = [];
  webcastLive: boolean = false;
  probability: number = -1;
  url: string = '';

  isValid(): boolean { return this.id.length > 0 && this.rocketName.length > 0; }
}

export class SunInfo {
  sunriseISO: string = '';
  sunsetISO: string = '';
  solarNoonISO: string = '';
  dayLengthSec: number = 0;
}

function asString(v: ESObject): string {
  return (typeof v === 'string') ? (v as string) : '';
}
function asNumber(v: ESObject): number {
  return (typeof v === 'number') ? (v as number) : 0;
}

export class SpaceApi {
  private static _inst: SpaceApi | null = null;

  // Cache — cross-mount so Discover navigation doesn't re-hit the network.
  private issCache: IssPosition | null = null;
  private issAt: number = 0;
  private issInFlight: Promise<IssPosition | null> | null = null;

  private launchesCache: LaunchInfo[] = [];
  private launchesAt: number = 0;
  private launchesInFlight: Promise<LaunchInfo[]> | null = null;

  private sunCache: SunInfo | null = null;
  private sunCacheKey: string = '';
  private sunAt: number = 0;
  private sunInFlight: Promise<SunInfo | null> | null = null;

  static readonly ISS_TTL_MS: number = 30_000;            // 30 s
  static readonly LAUNCHES_TTL_MS: number = 10 * 60_000;  // 10 min
  // Sun times are only valid for the current local day; also expire at
  // local midnight (checked in sunFresh).
  static readonly SUN_TTL_MS: number = 12 * 60 * 60_000;

  static get instance(): SpaceApi {
    if (SpaceApi._inst === null) SpaceApi._inst = new SpaceApi();
    return SpaceApi._inst as SpaceApi;
  }

  // --- synchronous cache accessors used by pages for instant render ---

  issCached(): IssPosition | null { return this.issCache; }
  launchesCached(): LaunchInfo[] { return this.launchesCache; }
  sunCached(lat: number, lng: number): SunInfo | null {
    if (this.sunCache === null) return null;
    if (this.sunCacheKey !== this.sunKey(lat, lng)) return null;
    // Discard if the stored result is from a different local day.
    const cachedDay = new Date(this.sunAt).toDateString();
    if (cachedDay !== new Date().toDateString()) return null;
    return this.sunCache;
  }

  private sunKey(lat: number, lng: number): string {
    return lat.toFixed(3) + ',' + lng.toFixed(3);
  }

  private issFresh(): boolean {
    return this.issCache !== null && Date.now() - this.issAt < SpaceApi.ISS_TTL_MS;
  }
  private launchesFresh(): boolean {
    return this.launchesCache.length > 0 && Date.now() - this.launchesAt < SpaceApi.LAUNCHES_TTL_MS;
  }
  private sunFresh(lat: number, lng: number): boolean {
    return this.sunCached(lat, lng) !== null;
  }

  // --- high-level stale-while-revalidate getters ---

  async getIss(force: boolean): Promise<IssPosition | null> {
    if (!force && this.issFresh()) return this.issCache;
    if (this.issInFlight !== null) return this.issInFlight;
    this.issInFlight = this.fetchIssPosition().then((p: IssPosition | null): IssPosition | null => {
      if (p !== null) { this.issCache = p; this.issAt = Date.now(); }
      this.issInFlight = null;
      return p !== null ? p : this.issCache;
    });
    return this.issInFlight;
  }

  async getUpcomingLaunches(limit: number, force: boolean): Promise<LaunchInfo[]> {
    if (!force && this.launchesFresh()) return this.launchesCache;
    if (this.launchesInFlight !== null) return this.launchesInFlight;
    this.launchesInFlight = this.fetchUpcomingLaunches(limit).then((list: LaunchInfo[]): LaunchInfo[] => {
      if (list.length > 0) { this.launchesCache = list; this.launchesAt = Date.now(); }
      this.launchesInFlight = null;
      return list.length > 0 ? list : this.launchesCache;
    });
    return this.launchesInFlight;
  }

  async getSun(lat: number, lng: number, force: boolean): Promise<SunInfo | null> {
    if (!force && this.sunFresh(lat, lng)) return this.sunCache;
    if (this.sunInFlight !== null) return this.sunInFlight;
    this.sunInFlight = this.fetchSun(lat, lng).then((s: SunInfo | null): SunInfo | null => {
      if (s !== null) {
        this.sunCache = s;
        this.sunCacheKey = this.sunKey(lat, lng);
        this.sunAt = Date.now();
      }
      this.sunInFlight = null;
      return s !== null ? s : this.sunCache;
    });
    return this.sunInFlight;
  }

  async fetchIssPosition(): Promise<IssPosition | null> {
    const req = http.createHttp();
    try {
      const resp = await req.request(
        'https://api.wheretheiss.at/v1/satellites/25544',
        {
          method: http.RequestMethod.GET,
          header: { 'Accept': 'application/json' },
          connectTimeout: 8000,
          readTimeout: 8000
        }
      );
      if (resp.responseCode !== 200) return null;
      const raw = resp.result as string;
      const obj: ESObject = JSON.parse(raw);
      const p = new IssPosition();
      p.latitude = asNumber(obj.latitude);
      p.longitude = asNumber(obj.longitude);
      p.altitudeKm = asNumber(obj.altitude);
      p.velocityKmh = asNumber(obj.velocity);
      p.visibility = asString(obj.visibility);
      p.timestamp = asNumber(obj.timestamp) * 1000;
      return p;
    } catch (e) {
      hilog.warn(0x1, TAG, 'ISS fetch failed');
      return null;
    } finally {
      req.destroy();
    }
  }

  async fetchUpcomingLaunches(limit: number): Promise<LaunchInfo[]> {
    const req = http.createHttp();
    try {
      const resp = await req.request(
        'https://ll.thespacedevs.com/2.2.0/launch/upcoming/?limit=' + limit + '&mode=detailed&format=json',
        {
          method: http.RequestMethod.GET,
          header: { 'Accept': 'application/json' },
          connectTimeout: 15000,
          readTimeout: 15000
        }
      );
      if (resp.responseCode !== 200) return [];
      const raw = resp.result as string;
      const obj: ESObject = JSON.parse(raw);
      const arr: ESObject = obj.results;
      const list: LaunchInfo[] = [];
      if (arr === null || arr === undefined) return list;
      const len = asNumber(arr.length);
      for (let i = 0; i < len; i++) {
        const it: ESObject = arr[i];
        if (it === null || it === undefined) continue;
        const li = this.parseLaunch(it);
        if (li.isValid()) list.push(li);
      }
      return list;
    } catch (e) {
      hilog.warn(0x1, TAG, 'launches fetch failed');
      return [];
    } finally {
      req.destroy();
    }
  }

  async fetchLaunchDetail(id: string): Promise<LaunchInfo | null> {
    const req = http.createHttp();
    try {
      const resp = await req.request(
        'https://ll.thespacedevs.com/2.2.0/launch/' + id + '/?format=json',
        {
          method: http.RequestMethod.GET,
          header: { 'Accept': 'application/json' },
          connectTimeout: 12000,
          readTimeout: 12000
        }
      );
      if (resp.responseCode !== 200) return null;
      const obj: ESObject = JSON.parse(resp.result as string);
      const li = this.parseLaunch(obj);
      return li.isValid() ? li : null;
    } catch (e) {
      hilog.warn(0x1, TAG, 'launch detail fetch failed');
      return null;
    } finally {
      req.destroy();
    }
  }

  private parseLaunch(it: ESObject): LaunchInfo {
    const li = new LaunchInfo();
    li.id = asString(it.id);
    li.name = asString(it.name);
    li.url = asString(it.url);
    const netStr = asString(it.net);
    if (netStr.length > 0) li.net = new Date(netStr);
    const wStart = asString(it.window_start);
    if (wStart.length > 0) li.windowStart = new Date(wStart);
    const wEnd = asString(it.window_end);
    if (wEnd.length > 0) li.windowEnd = new Date(wEnd);

    const rocket: ESObject = it.rocket;
    if (rocket !== null && rocket !== undefined) {
      const config: ESObject = rocket.configuration;
      if (config !== null && config !== undefined) {
        li.rocketName = asString(config.name);
        li.rocketFamily = asString(config.family);
        li.rocketDescription = asString(config.description);
        li.rocketImage = asString(config.image_url);
      }
    }

    const pad: ESObject = it.pad;
    if (pad !== null && pad !== undefined) {
      li.padName = asString(pad.name);
      const loc: ESObject = pad.location;
      if (loc !== null && loc !== undefined) {
        li.locationName = asString(loc.name);
        li.locationCountry = asString(loc.country_code);
      }
    }

    const agency: ESObject = it.launch_service_provider;
    if (agency !== null && agency !== undefined) {
      li.agency = asString(agency.name);
      li.agencyAbbrev = asString(agency.abbrev);
    }

    const status: ESObject = it.status;
    if (status !== null && status !== undefined) {
      li.status = asString(status.name);
      li.statusDescription = asString(status.description);
    }

    li.image = asString(it.image);
    li.probability = asNumber(it.probability);
    li.webcastLive = asString(it.webcast_live) === 'true';

    const mission: ESObject = it.mission;
    if (mission !== null && mission !== undefined) {
      li.missionName = asString(mission.name);
      li.missionDescription = asString(mission.description);
      li.missionType = asString(mission.type);
      const orbit: ESObject = mission.orbit;
      if (orbit !== null && orbit !== undefined) li.orbitName = asString(orbit.name);
    }

    const vids: ESObject = it.vidURLs;
    if (vids !== null && vids !== undefined) {
      const vlen = asNumber(vids.length);
      for (let j = 0; j < vlen; j++) {
        const v: ESObject = vids[j];
        if (v === null || v === undefined) continue;
        const link = new LaunchLink();
        link.title = asString(v.title);
        link.url = asString(v.url);
        if (link.url.length > 0) li.vidURLs.push(link);
      }
    }
    const infos: ESObject = it.infoURLs;
    if (infos !== null && infos !== undefined) {
      const ilen = asNumber(infos.length);
      for (let j = 0; j < ilen; j++) {
        const v: ESObject = infos[j];
        if (v === null || v === undefined) continue;
        const link = new LaunchLink();
        link.title = asString(v.title);
        link.url = asString(v.url);
        if (link.url.length > 0) li.infoURLs.push(link);
      }
    }

    return li;
  }

  async fetchSun(latitude: number, longitude: number): Promise<SunInfo | null> {
    const req = http.createHttp();
    try {
      const url = 'https://api.sunrise-sunset.org/json?lat=' + latitude.toFixed(4)
        + '&lng=' + longitude.toFixed(4) + '&formatted=0';
      const resp = await req.request(url, {
        method: http.RequestMethod.GET,
        header: { 'Accept': 'application/json' },
        connectTimeout: 8000,
        readTimeout: 8000
      });
      if (resp.responseCode !== 200) return null;
      const obj: ESObject = JSON.parse(resp.result as string);
      if (asString(obj.status) !== 'OK') return null;
      const r: ESObject = obj.results;
      if (r === null || r === undefined) return null;
      const s = new SunInfo();
      s.sunriseISO = asString(r.sunrise);
      s.sunsetISO = asString(r.sunset);
      s.solarNoonISO = asString(r.solar_noon);
      s.dayLengthSec = asNumber(r.day_length);
      return s;
    } catch (e) {
      hilog.warn(0x1, TAG, 'sun fetch failed');
      return null;
    } finally {
      req.destroy();
    }
  }
}
