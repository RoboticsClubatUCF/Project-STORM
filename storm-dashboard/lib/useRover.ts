"use client";

import { FoxgloveClient, type Channel, type Service } from "@foxglove/ws-protocol";
import { parse } from "@foxglove/rosmsg";
import { MessageReader, MessageWriter } from "@foxglove/rosmsg2-serialization";
import { useCallback, useEffect, useRef, useState } from "react";

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

export type TeleopStatus = {
  running: boolean;
  profile: string;
  use_joy: boolean;
  uptime_s: number;
  message: string;
};

export type TeleopProfile = "teleop" | "spaceu";

export type ServiceResult = { success: boolean; message: string };

type DiagnosticArray = { status: DiagnosticStatus[] };

export type ConnectionState = "connecting" | "connected" | "disconnected";

export type RoverState = {
  connection: ConnectionState;
  battery?: BatteryStatus;
  system?: SystemTelemetry;
  teleop?: TeleopStatus;
  diagnostics: Record<string, DiagnosticStatus>;
  teleopServices: boolean;
};

export type Rover = RoverState & {
  startTeleop: (profile: TeleopProfile, useJoy: boolean) => Promise<ServiceResult>;
  stopTeleop: () => Promise<ServiceResult>;
};

const TOPICS = ["/telemetry/battery", "/telemetry/system", "/diagnostics", "/storm/teleop/status"];
const RECONNECT_MS = 2000;
// foxglove_bridge 3.x (Foxglove SDK) speaks foxglove.sdk.v1, older bridges foxglove.websocket.v1
const SUBPROTOCOLS = ["foxglove.sdk.v1", FoxgloveClient.SUPPORTED_SUBPROTOCOL];

const START_SERVICE = "/storm/teleop/start";
const STOP_SERVICE = "/storm/teleop/stop";
// Stop waits up to 20 s on the rover for teleop to shut down
const CALL_TIMEOUT_MS = 25000;

// CDR header plus the one-byte placeholder ROS adds to empty structs
const EMPTY_REQUEST = Uint8Array.of(0, 1, 0, 0, 0);

type ServiceCodec = { id: number; write: (msg: unknown) => Uint8Array; reader: MessageReader };
type PendingCall = { resolve: (r: ServiceResult) => void; reject: (e: Error) => void };

function makeCodec(svc: Service): ServiceCodec {
  const reqSchema = svc.request?.schema ?? svc.requestSchema ?? "";
  const resSchema = svc.response?.schema ?? svc.responseSchema ?? "";
  const writer = reqSchema.trim() ? new MessageWriter(parse(reqSchema, { ros2: true })) : undefined;
  return {
    id: svc.id,
    write: (msg) => (writer ? writer.writeMessage(msg) : EMPTY_REQUEST),
    reader: new MessageReader(parse(resSchema, { ros2: true })),
  };
}

export function useRover(url: string): Rover {
  const [state, setState] = useState<RoverState>({
    connection: "connecting",
    diagnostics: {},
    teleopServices: false,
  });

  const clientRef = useRef<FoxgloveClient>();
  const servicesRef = useRef(new Map<string, ServiceCodec>());
  const pendingRef = useRef(new Map<number, PendingCall>());
  const nextCallId = useRef(1);

  useEffect(() => {
    let client: FoxgloveClient | undefined;
    let retry: ReturnType<typeof setTimeout> | undefined;
    let stopped = false;
    const services = servicesRef.current;
    const pending = pendingRef.current;

    const updateServices = () => {
      const teleopServices = services.has(START_SERVICE) && services.has(STOP_SERVICE);
      setState((s) => ({ ...s, teleopServices }));
    };

    const failPending = (reason: string) => {
      for (const p of pending.values()) p.reject(new Error(reason));
      pending.clear();
    };

    const connect = () => {
      setState((s) => ({ ...s, connection: "connecting" }));
      client = new FoxgloveClient({
        ws: new WebSocket(url, SUBPROTOCOLS),
      });
      clientRef.current = client;

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

      client.on("advertiseServices", (list: Service[]) => {
        for (const svc of list) {
          if (svc.name !== START_SERVICE && svc.name !== STOP_SERVICE) continue;
          services.set(svc.name, makeCodec(svc));
        }
        updateServices();
      });

      client.on("unadvertiseServices", (ids: number[]) => {
        for (const [name, codec] of services) if (ids.includes(codec.id)) services.delete(name);
        updateServices();
      });

      client.on("serviceCallResponse", (res) => {
        const call = pending.get(res.callId);
        if (!call) return;
        pending.delete(res.callId);
        const codec = [...services.values()].find((c) => c.id === res.serviceId);
        if (!codec) return call.reject(new Error("service went away"));
        const bytes = new Uint8Array(res.data.buffer, res.data.byteOffset, res.data.byteLength);
        call.resolve(codec.reader.readMessage(bytes) as ServiceResult);
      });

      client.on("serviceCallFailure", (f) => {
        const call = pending.get(f.callId);
        if (!call) return;
        pending.delete(f.callId);
        call.reject(new Error(f.message));
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
            case "/storm/teleop/status":
              return { ...s, teleop: msg as TeleopStatus };
            default: {
              const diagnostics = { ...s.diagnostics };
              for (const d of (msg as DiagnosticArray).status) diagnostics[d.name] = d;
              return { ...s, diagnostics };
            }
          }
        });
      });

      client.on("close", () => {
        services.clear();
        failPending("disconnected");
        setState((s) => ({ ...s, connection: "disconnected", teleop: undefined, teleopServices: false }));
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

  const call = useCallback((name: string, request: unknown): Promise<ServiceResult> => {
    const client = clientRef.current;
    const codec = servicesRef.current.get(name);
    if (!client || !codec) return Promise.reject(new Error(`${name} not available`));

    const callId = nextCallId.current++;
    return new Promise<ServiceResult>((resolve, reject) => {
      const timer = setTimeout(() => {
        pendingRef.current.delete(callId);
        reject(new Error("timed out"));
      }, CALL_TIMEOUT_MS);
      pendingRef.current.set(callId, {
        resolve: (r) => (clearTimeout(timer), resolve(r)),
        reject: (e) => (clearTimeout(timer), reject(e)),
      });
      client.sendServiceCallRequest({
        serviceId: codec.id,
        callId,
        encoding: "cdr",
        data: codec.write(request),
      });
    });
  }, []);

  const startTeleop = useCallback(
    (profile: TeleopProfile, useJoy: boolean) => call(START_SERVICE, { profile, use_joy: useJoy }),
    [call],
  );
  const stopTeleop = useCallback(() => call(STOP_SERVICE, {}), [call]);

  return { ...state, startTeleop, stopTeleop };
}
