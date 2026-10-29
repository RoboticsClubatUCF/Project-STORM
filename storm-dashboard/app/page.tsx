"use client";

import { useState } from "react";
import {
  useRover,
  type BatteryStatus,
  type Rover,
  type ServiceResult,
  type SystemTelemetry,
  type TeleopProfile,
} from "@/lib/useRover";

const ROVER_WS = process.env.NEXT_PUBLIC_ROVER_WS ?? "ws://rover.local:8765";

const PROFILE_LABELS: Record<TeleopProfile, string> = { teleop: "Teleop", spaceu: "SpaceU (full turbo)" };

const BATTERY_LEVELS = ["Unknown", "OK", "Low", "Critical"];
const DIAG_LEVELS = ["OK", "Warn", "Error", "Stale"];

function fmt(value: number | undefined, digits = 1, unit = ""): string {
  if (value === undefined || Number.isNaN(value)) return "n/a";
  return `${value.toFixed(digits)}${unit}`;
}

function fmtUptime(s: number | undefined): string {
  if (s === undefined || Number.isNaN(s)) return "n/a";
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  return `${h}h ${m}m`;
}

function Row({ label, value }: { label: string; value: string }) {
  return (
    <div className="row">
      <span className="label">{label}</span>
      <span className="value">{value}</span>
    </div>
  );
}

function TeleopCard({ rover }: { rover: Rover }) {
  const [profile, setProfile] = useState<TeleopProfile>("teleop");
  const [useJoy, setUseJoy] = useState(false);
  const [busy, setBusy] = useState(false);
  const [reply, setReply] = useState("");

  const t = rover.teleop;
  const running = t?.running ?? false;
  const known = t !== undefined && rover.teleopServices;
  const status = !t ? "Unknown" : running ? "Running" : t.message.startsWith("exited") ? "Exited" : "Stopped";
  const statusClass = !t ? "" : running ? "diag-0" : status === "Exited" ? "diag-2" : "";

  const run = async (action: () => Promise<ServiceResult>) => {
    setBusy(true);
    try {
      const r = await action();
      setReply(r.message);
    } catch (e) {
      setReply(`Failed: ${(e as Error).message}`);
    } finally {
      setBusy(false);
    }
  };

  const onStop = () => {
    if (!window.confirm("Stop teleop? The rover stops immediately.")) return;
    run(rover.stopTeleop);
  };

  return (
    <section className="card">
      <h2>
        Teleop <span className={`pill ${statusClass}`}>{status}</span>
      </h2>
      {running && t && (
        <>
          <Row label="Profile" value={PROFILE_LABELS[t.profile as TeleopProfile] ?? t.profile} />
          <Row label="Controller" value={t.use_joy ? "On rover" : "Base station"} />
          <Row label="Uptime" value={fmtUptime(t.uptime_s)} />
        </>
      )}
      {!running && t && t.message && <Row label="Last" value={t.message} />}

      <div className="controls">
        <select
          value={profile}
          onChange={(e) => setProfile(e.target.value as TeleopProfile)}
          disabled={running || busy}
        >
          {(Object.keys(PROFILE_LABELS) as TeleopProfile[]).map((p) => (
            <option key={p} value={p}>
              {PROFILE_LABELS[p]}
            </option>
          ))}
        </select>
        <label className="check">
          <input
            type="checkbox"
            checked={useJoy}
            onChange={(e) => setUseJoy(e.target.checked)}
            disabled={running || busy}
          />
          Controller on rover (use_joy)
        </label>
      </div>
      <div className="controls">
        <button
          className="btn btn-start"
          onClick={() => run(() => rover.startTeleop(profile, useJoy))}
          disabled={!known || running || busy}
        >
          Start
        </button>
        <button className="btn btn-stop" onClick={onStop} disabled={!known || !running || busy}>
          Stop
        </button>
      </div>
      {!rover.teleopServices && rover.connection === "connected" && (
        <p className="muted">Teleop launcher not available on the rover</p>
      )}
      {reply && <p className="muted">{reply}</p>}
    </section>
  );
}

function BatteryCard({ b }: { b?: BatteryStatus }) {
  const level = b ? BATTERY_LEVELS[b.level] ?? "Unknown" : "Unknown";
  return (
    <section className="card">
      <h2>
        Battery <span className={`pill level-${b?.level ?? 0}`}>{level}</span>
      </h2>
      <Row label="Voltage" value={fmt(b?.voltage, 2, " V")} />
      <Row label="Min / max" value={`${fmt(b?.min_voltage, 2)} / ${fmt(b?.max_voltage, 2)} V`} />
      <Row label="SPARKs online" value={b ? `${b.sources_online} / ${b.sources_total}` : "n/a"} />
    </section>
  );
}

function SystemCard({ s }: { s?: SystemTelemetry }) {
  return (
    <section className="card">
      <h2>Computer</h2>
      <Row label="CPU" value={fmt(s?.cpu_percent, 0, "%")} />
      <Row label="Load (1m)" value={fmt(s?.load_avg_1m, 2)} />
      <Row label="Temp" value={fmt(s?.cpu_temp_c, 1, " °C")} />
      <Row label="Memory" value={fmt(s?.mem_used_percent, 0, "%")} />
      <Row label="Disk" value={fmt(s?.disk_used_percent, 0, "%")} />
      <Row label="Uptime" value={fmtUptime(s?.uptime_s)} />
      <h3>WiFi {s?.wifi_interface ? `(${s.wifi_interface})` : ""}</h3>
      <Row
        label="Link"
        value={s && !Number.isNaN(s.wifi_link_quality) ? `${Math.round(s.wifi_link_quality * 100)}%` : "n/a"}
      />
      <Row label="Signal" value={fmt(s?.wifi_signal_dbm, 0, " dBm")} />
    </section>
  );
}

export default function Page() {
  const rover = useRover(ROVER_WS);
  const diags = Object.values(rover.diagnostics).sort((a, b) => a.name.localeCompare(b.name));

  return (
    <main>
      <header>
        <h1>STORM</h1>
        <span className={`pill conn-${rover.connection}`}>{rover.connection}</span>
        <span className="url">{ROVER_WS}</span>
      </header>

      <div className="grid">
        <TeleopCard rover={rover} />
        <BatteryCard b={rover.battery} />
        <SystemCard s={rover.system} />
        <section className="card">
          <h2>Diagnostics</h2>
          {diags.length === 0 && <p className="muted">No diagnostics yet</p>}
          {diags.map((d) => (
            <div key={d.name} className="row">
              <span className="label">{d.name}</span>
              <span className={`pill diag-${d.level}`}>{DIAG_LEVELS[d.level] ?? d.level}</span>
              <span className="value">{d.message}</span>
            </div>
          ))}
        </section>
      </div>
    </main>
  );
}
