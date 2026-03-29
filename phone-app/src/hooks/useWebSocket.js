import { useState, useCallback, useRef, useEffect } from 'react';

// Default laptop IP — user can override via the UI.
// Must be on the same WiFi network as the phone.
const DEFAULT_WS_URL = localStorage.getItem('leadme_ws_url') || 'ws://192.168.1.100:8765';

const RECONNECT_DELAY_MS = 3000;

/**
 * WebSocket connection to the laptop navigation server.
 *
 * Drop-in replacement for useBluetooth — same external interface:
 *   connected, connect(url?), disconnect, sendNav(navState, deviceHeading)
 *
 * The laptop expects:
 *   { type, target_bearing, device_heading, distance, lat, lng, speed }
 *
 * The laptop sends back:
 *   { type: "status", heading_error, pid_output, connected }
 */
export function useWebSocket() {
  const [connected, setConnected]     = useState(false);
  const [error, setError]             = useState(null);
  const [serverStatus, setServerStatus] = useState(null);  // latest status from laptop
  const [wsUrl, setWsUrlState]        = useState(DEFAULT_WS_URL);

  const wsRef        = useRef(null);
  const reconnectRef = useRef(null);
  const urlRef       = useRef(wsUrl);
  urlRef.current     = wsUrl;

  const _clearReconnect = () => {
    if (reconnectRef.current) {
      clearTimeout(reconnectRef.current);
      reconnectRef.current = null;
    }
  };

  const _open = useCallback((url) => {
    if (wsRef.current) {
      wsRef.current.onclose = null;
      wsRef.current.close();
    }

    const ws = new WebSocket(url);
    wsRef.current = ws;

    ws.onopen = () => {
      setConnected(true);
      setError(null);
    };

    ws.onclose = (ev) => {
      setConnected(false);
      if (!ev.wasClean) {
        setError(`Connection lost — retrying in ${RECONNECT_DELAY_MS / 1000}s`);
        reconnectRef.current = setTimeout(() => _open(urlRef.current), RECONNECT_DELAY_MS);
      }
    };

    ws.onerror = () => {
      setError(`Cannot reach laptop at ${urlRef.current}`);
    };

    ws.onmessage = (ev) => {
      try {
        const data = JSON.parse(ev.data);
        if (data.type === 'status') setServerStatus(data);
      } catch (_) {}
    };
  }, []);

  const connect = useCallback((url) => {
    const target = url || urlRef.current;
    if (url) {
      setWsUrlState(target);
      localStorage.setItem('leadme_ws_url', target);
    }
    _clearReconnect();
    _open(target);
  }, [_open]);

  const disconnect = useCallback(() => {
    _clearReconnect();
    if (wsRef.current) {
      wsRef.current.onclose = null;
      wsRef.current.close(1000, 'user disconnect');
      wsRef.current = null;
    }
    setConnected(false);
    setError(null);
  }, []);

  // Cleanup on unmount
  useEffect(() => () => {
    _clearReconnect();
    wsRef.current?.close();
  }, []);

  /**
   * Send navigation data to the laptop.
   * Call this every time navState or position updates.
   *
   * @param {object} navState   — { bearing, distance } from useNavigation
   * @param {object} position   — { lat, lng, heading } from useGPS
   */
  const sendNav = useCallback((navState, position) => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) return;
    if (!navState || !position) return;

    const packet = {
      type:           'nav',
      target_bearing: navState.bearing,
      device_heading: position.heading ?? null,   // null when stationary
      distance:       navState.distance,
      lat:            position.lat,
      lng:            position.lng,
      speed:          position.speed ?? null,
    };
    wsRef.current.send(JSON.stringify(packet));
  }, []);

  return {
    connected,
    error,
    wsUrl,
    serverStatus,
    connect,
    disconnect,
    sendNav,
  };
}
