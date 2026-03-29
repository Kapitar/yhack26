import { useState, useCallback, useRef } from 'react';

// BLE GATT identifiers — must match the cane firmware.
// TODO: define these once the cane BLE server is implemented.
const SERVICE_UUID = '00001234-0000-1000-8000-00805f9b34fb'; // placeholder
const BEARING_CHAR_UUID = '00001235-0000-1000-8000-00805f9b34fb'; // placeholder

/**
 * Stub for Web Bluetooth connection to the cane.
 * iOS requires the Bluefy browser; Android Chrome works natively.
 *
 * Returns:
 *   connected   — whether a GATT connection is active
 *   connect()   — request device + open GATT connection
 *   disconnect()
 *   sendBearing(degrees: number) — write target bearing to cane
 */
export function useBluetooth() {
  const [connected, setConnected] = useState(false);
  const [error, setError] = useState(null);
  const characteristicRef = useRef(null);
  const deviceRef = useRef(null);

  const connect = useCallback(async () => {
    if (!navigator.bluetooth) {
      setError('Web Bluetooth is not available. On iOS, open this app in Bluefy.');
      return;
    }
    try {
      setError(null);
      const device = await navigator.bluetooth.requestDevice({
        filters: [{ name: 'LeadMe-Cane' }],
        optionalServices: [SERVICE_UUID],
      });
      deviceRef.current = device;

      device.addEventListener('gattserverdisconnected', () => {
        setConnected(false);
        characteristicRef.current = null;
      });

      const server = await device.gatt.connect();
      const service = await server.getPrimaryService(SERVICE_UUID);
      characteristicRef.current = await service.getCharacteristic(BEARING_CHAR_UUID);

      setConnected(true);
    } catch (e) {
      setError(e.message);
    }
  }, []);

  const disconnect = useCallback(() => {
    deviceRef.current?.gatt?.disconnect();
    setConnected(false);
    characteristicRef.current = null;
  }, []);

  /**
   * Write bearing (0–360°) to the cane over BLE.
   * Encodes as a 2-byte little-endian uint16 (degrees × 10, e.g. 045.5° → 455).
   */
  const sendBearing = useCallback((degrees) => {
    if (!characteristicRef.current) return;
    const value = Math.round(degrees * 10) & 0xffff;
    const buf = new Uint8Array(2);
    buf[0] = value & 0xff;
    buf[1] = (value >> 8) & 0xff;
    characteristicRef.current.writeValueWithoutResponse(buf).catch(() => {});
  }, []);

  return { connected, error, connect, disconnect, sendBearing };
}