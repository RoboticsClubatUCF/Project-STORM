"use client";

import { FoxgloveClient, type Channel } from "@foxglove/ws-protocol";
import { parse } from "@foxglove/rosmsg";
import { MessageReader } from "@foxglove/rosmsg2-serialization";
import { useEffect, useState } from "react";

export type BatteryStatus = {
  voltage: number;
  min_voltage: number;
  max_voltage: number;
  sources_online: number;
  sources_total: number;
  level: number;
};

export type SystemTelemetry = {
  cpu_percent: number;
  load_avg_1m: number;
  cpu_temp_c: number;
  mem_used_percent: number;
  disk_used_percent: number;
  uptime_s: number;
  wifi_interface: string;
  wifi_link_quality: number;
  wifi_signal_dbm: number;
};

export type DiagnosticStatus = {
  level: number;
  name: string;
  message: string;
  hardware_id: string;
};

type DiagnosticArray = { status: DiagnosticStatus[] };

export type ConnectionState = "connecting" | "connected" | "disconnected";

export type RoverState = {
  connection: ConnectionState;
  battery?: BatteryStatus;
  system?: SystemTelemetry;
  diagnostics: Record<string, DiagnosticStatus>;
};

const TOPICS = ["/telemetry/battery", "/telemetry/system", "/diagnostics"];
const RECONNECT_MS = 2000;

export function useRover(url: string): RoverState {
  const [state, setState] = useState<RoverState>({
    connection: "connecting",
    diagnostics: {},
  });

  useEffect(() => {
    let client: FoxgloveClient | undefined;
    let retry: ReturnType<typeof setTimeout> | undefined;
    let stopped = false;

    const connect = () => {
      setState((s) => ({ ...s, connection: "connecting" }));
      client = new FoxgloveClient({
        ws: new WebSocket(url, [FoxgloveClient.SUPPORTED_SUBPROTOCOL]),
      });

      const subs = new Map<number, { topic: string; reader: MessageReader }>();

      client.on("open", () => setState((s) => ({ ...s, connection: "connected" })));

      client.on("advertise", (channels: Channel[]) => {
        for (const ch of channels) {
          if (!TOPICS.includes(ch.topic) || ch.encoding !== "cdr") continue;
          if (ch.schemaEncoding && ch.schemaEncoding !== "ros2msg") continue;
          const reader = new MessageReader(parse(ch.schema, { ros2: true }));
          const id = client!.subscribe(ch.id);
          subs.set(id, { topic: ch.topic, reader });
        }
      });

      client.on("message", ({ subscriptionId, data }) => {
        const sub = subs.get(subscriptionId);
        if (!sub) return;
        const msg = sub.reader.readMessage(data);
        setState((s) => {
          switch (sub.topic) {
            case "/telemetry/battery":
              return { ...s, battery: msg as BatteryStatus };
            case "/telemetry/system":
              return { ...s, system: msg as SystemTelemetry };
            default: {
              const diagnostics = { ...s.diagnostics };
              for (const d of (msg as DiagnosticArray).status) diagnostics[d.name] = d;
              return { ...s, diagnostics };
            }
          }
        });
      });

      client.on("close", () => {
        setState((s) => ({ ...s, connection: "disconnected" }));
        if (!stopped) retry = setTimeout(connect, RECONNECT_MS);
      });
    };

    connect();
    return () => {
      stopped = true;
      clearTimeout(retry);
      client?.close();
    };
  }, [url]);

  return state;
}
