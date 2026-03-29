import { useState } from 'react';

/**
 * Connection widget for the WebSocket link to the laptop.
 * Shows connection state and lets the user change the laptop IP/URL.
 */
export default function ConnectionStatus({
  connected,
  wsUrl,
  error,
  serverStatus,
  onConnect,
  onDisconnect,
}) {
  const [editing, setEditing]   = useState(false);
  const [inputUrl, setInputUrl] = useState(wsUrl);

  function handleConnect() {
    onConnect(inputUrl.trim() || wsUrl);
    setEditing(false);
  }

  return (
    <div className="connection-status">
      {editing ? (
        <div className="ws-url-editor">
          <input
            type="text"
            value={inputUrl}
            onChange={(e) => setInputUrl(e.target.value)}
            placeholder="ws://192.168.x.x:8765"
            autoFocus
          />
          <button onClick={handleConnect}>Connect</button>
          <button onClick={() => setEditing(false)}>Cancel</button>
        </div>
      ) : (
        <button
          className={`ble-pill ${connected ? 'ble-connected' : 'ble-disconnected'}`}
          onClick={connected ? onDisconnect : () => setEditing(true)}
          aria-label={connected ? 'Laptop connected — tap to disconnect' : 'Tap to connect to laptop'}
        >
          <span className="ble-dot" />
          {connected ? 'Laptop' : 'Connect Laptop'}
        </button>
      )}

      {error && !editing && (
        <div className="ws-error" role="alert">{error}</div>
      )}

      {connected && serverStatus && (
        <div className="server-status">
          err {serverStatus.heading_error > 0 ? '+' : ''}{serverStatus.heading_error}°
        </div>
      )}
    </div>
  );
}
