export default function BluetoothStatus({ connected, onConnect, onDisconnect }) {
  return (
    <button
      className={`ble-pill ${connected ? 'ble-connected' : 'ble-disconnected'}`}
      onClick={connected ? onDisconnect : onConnect}
      aria-label={connected ? 'Cane connected — tap to disconnect' : 'Cane not connected — tap to connect'}
    >
      <span className="ble-dot" />
      {connected ? 'Cane' : 'Connect Cane'}
    </button>
  );
}