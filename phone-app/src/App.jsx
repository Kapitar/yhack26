import { useState } from 'react';
import Map from './components/Map.jsx';
import DestinationSearch from './components/DestinationSearch.jsx';
import NavPanel from './components/NavPanel.jsx';
import ConnectionStatus from './components/ConnectionStatus.jsx';
import { useGPS } from './hooks/useGPS.js';
import { useNavigation } from './hooks/useNavigation.js';
import { useWebSocket } from './hooks/useWebSocket.js';

export default function App() {
  const { position, error: gpsError } = useGPS();
  const {
    status,
    waypoints,
    navState,
    error: navError,
    startNavigation,
    stopNavigation,
  } = useNavigation(position);
  const {
    connected,
    error: wsError,
    wsUrl,
    serverStatus,
    connect,
    disconnect,
    sendNav,
  } = useWebSocket();

  const [destination, setDestination] = useState(null);

  const isNavigating = status === 'navigating' || status === 'arrived' || status === 'routing';

  // Stream nav data to laptop every time navState or position updates
  if (navState && connected) {
    sendNav(navState, position);
  }

  function handleDestinationSelect(result) {
    setDestination(result);
    startNavigation(result);
  }

  function handleStop() {
    stopNavigation();
    setDestination(null);
  }

  return (
    <div className="app">
      {/* Header */}
      <header className="header">
        <span className="logo">LeadMe</span>
        <ConnectionStatus
          connected={connected}
          wsUrl={wsUrl}
          error={wsError}
          serverStatus={serverStatus}
          onConnect={connect}
          onDisconnect={disconnect}
        />
      </header>

      {/* Status bars */}
      {(gpsError || navError) && (
        <div className="status-bar error" role="alert">
          {gpsError || navError}
        </div>
      )}
      {!position && !gpsError && (
        <div className="status-bar" role="status">
          Acquiring GPS…
        </div>
      )}
      {position && (
        <div style={{ fontFamily: 'monospace', fontSize: 11, padding: '2px 8px', background: '#111', color: '#0f0' }}>
          lat: {position.lat.toFixed(6)} lng: {position.lng.toFixed(6)} acc: {position.accuracy?.toFixed(0)}m
          {position.heading != null ? `  hdg: ${position.heading.toFixed(0)}°` : '  hdg: —'}
        </div>
      )}

      {/* Map — always visible */}
      <div className={`map-wrapper ${isNavigating ? 'map-navigating' : ''}`}>
        <Map
          position={position}
          waypoints={waypoints}
          destination={destination}
        />
      </div>

      {/* Bottom panel: search or nav */}
      <div className="bottom-panel">
        {!isNavigating ? (
          <DestinationSearch
            onSelect={handleDestinationSelect}
            disabled={!position}
          />
        ) : (
          <NavPanel
            navState={navState}
            deviceHeading={position?.heading}
            status={status}
            onStop={handleStop}
          />
        )}
      </div>
    </div>
  );
}