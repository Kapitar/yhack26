// Self-hosted Valhalla for pedestrian routing.
// Change VALHALLA_BASE if your instance runs on a different host/port.
// Proxied through Vite dev server to avoid CORS — see vite.config.js
const VALHALLA_BASE = '/valhalla';
const NOMINATIM_BASE = 'https://nominatim.openstreetmap.org';

/**
 * Decode a Valhalla encoded polyline6 string into [lat, lng] pairs.
 * Valhalla uses precision=6 (unlike Google Maps which uses precision=5).
 */
function decodePolyline6(encoded) {
  const points = [];
  let index = 0, lat = 0, lng = 0;

  while (index < encoded.length) {
    let shift = 0, result = 0, b;
    do {
      b = encoded.charCodeAt(index++) - 63;
      result |= (b & 0x1f) << shift;
      shift += 5;
    } while (b >= 0x20);
    lat += result & 1 ? ~(result >> 1) : result >> 1;

    shift = 0; result = 0;
    do {
      b = encoded.charCodeAt(index++) - 63;
      result |= (b & 0x1f) << shift;
      shift += 5;
    } while (b >= 0x20);
    lng += result & 1 ? ~(result >> 1) : result >> 1;

    points.push({ lat: lat / 1e6, lng: lng / 1e6 });
  }
  return points;
}

/**
 * Fetch a pedestrian route via Valhalla.
 * Returns an array of { lat, lng } waypoints along the full route geometry.
 */
export async function fetchRoute(from, to) {
  const body = {
    locations: [
      { lon: from.lng, lat: from.lat },
      { lon: to.lng,   lat: to.lat  },
    ],
    costing: 'pedestrian',
  };

  const res = await fetch(`${VALHALLA_BASE}/route`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  if (!res.ok) throw new Error('Routing request failed');
  const data = await res.json();
  if (data.trip?.status !== 0) throw new Error(data.trip?.status_message ?? 'No route found');

  return decodePolyline6(data.trip.legs[0].shape);
}

/**
 * Geocode a text query. Returns array of { lat, lng, display_name }.
 */
export async function geocode(query) {
  const url =
    `${NOMINATIM_BASE}/search?format=json&q=${encodeURIComponent(query)}&limit=5`;
  const res = await fetch(url, {
    headers: { 'Accept-Language': 'en', 'User-Agent': 'LeadMe/0.1' },
  });
  if (!res.ok) throw new Error('Geocoding failed');
  const results = await res.json();
  return results.map((r) => ({
    lat: parseFloat(r.lat),
    lng: parseFloat(r.lon),
    display_name: r.display_name,
  }));
}