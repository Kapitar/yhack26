import { useEffect, useRef } from 'react';
import { MapContainer, TileLayer, Marker, Polyline, useMap } from 'react-leaflet';
import L from 'leaflet';

// Fix default icon paths broken by bundlers
delete L.Icon.Default.prototype._getIconUrl;
L.Icon.Default.mergeOptions({
  iconRetinaUrl: 'https://unpkg.com/leaflet@1.9.4/dist/images/marker-icon-2x.png',
  iconUrl: 'https://unpkg.com/leaflet@1.9.4/dist/images/marker-icon.png',
  shadowUrl: 'https://unpkg.com/leaflet@1.9.4/dist/images/marker-shadow.png',
});

const userIcon = L.divIcon({
  className: '',
  html: `<div style="
    width:18px;height:18px;border-radius:50%;
    background:#3b82f6;border:3px solid #fff;
    box-shadow:0 0 0 4px rgba(59,130,246,0.35);
  "></div>`,
  iconSize: [18, 18],
  iconAnchor: [9, 9],
});

const destIcon = L.divIcon({
  className: '',
  html: `<div style="
    width:22px;height:22px;border-radius:50%;
    background:#ef4444;border:3px solid #fff;
    box-shadow:0 2px 8px rgba(0,0,0,0.4);
  "></div>`,
  iconSize: [22, 22],
  iconAnchor: [11, 11],
});

/** Keeps the map centered on the user position. */
function AutoCenter({ position }) {
  const map = useMap();
  const initialRef = useRef(false);

  useEffect(() => {
    if (!position) return;
    if (!initialRef.current) {
      map.setView([position.lat, position.lng], 17);
      initialRef.current = true;
    } else {
      map.panTo([position.lat, position.lng], { animate: true });
    }
  }, [position, map]);

  return null;
}

export default function Map({ position, waypoints, destination }) {
  const defaultCenter = [40.7128, -74.006]; // NYC fallback

  return (
    <MapContainer
      center={position ? [position.lat, position.lng] : defaultCenter}
      zoom={17}
      style={{ width: '100%', height: '100%' }}
      zoomControl={false}
    >
      <TileLayer
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>'
      />

      <AutoCenter position={position} />

      {position && (
        <Marker position={[position.lat, position.lng]} icon={userIcon} />
      )}

      {destination && (
        <Marker position={[destination.lat, destination.lng]} icon={destIcon} />
      )}

      {waypoints.length > 1 && (
        <Polyline
          positions={waypoints.map((w) => [w.lat, w.lng])}
          pathOptions={{ color: '#3b82f6', weight: 5, opacity: 0.8 }}
        />
      )}
    </MapContainer>
  );
}