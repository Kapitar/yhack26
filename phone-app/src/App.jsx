import { useState } from 'react';
import Map from './components/Map.jsx';
import DestinationSearch from './components/DestinationSearch.jsx';
import NavPanel from './components/NavPanel.jsx';
import BluetoothStatus from './components/BluetoothStatus.jsx';
import { useGPS } from './hooks/useGPS.js';
import { useNavigation } from './hooks/useNavigation.js';
import { useBluetooth } from './hooks/useBluetooth.js';

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
  const { connected, connect, disconnect, sendBearing } = useBluetooth();

  const [destination, setDestination] = useState(null);

  const isNavigating = status === 'navigating' || status === 'arrived' || status === 'routing';

  // Forward bearing to cane whenever navState updates
  if (navState && connected) {
    sendBearing(navState.bearing);
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
        <BluetoothStatus
          connected={connected}
          onConnect={connect}
          onDisconnect={disconnect}
        />
      </header>

      {/* GPS status bar */}
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