import { relativeBearing } from '../utils/geo';

/** Cardinal direction label from absolute bearing. */
function cardinalLabel(deg) {
  const dirs = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];
  return dirs[Math.round(deg / 45) % 8];
}

function formatDistance(meters) {
  if (meters >= 1000) return `${(meters / 1000).toFixed(1)} km`;
  return `${Math.round(meters)} m`;
}

export default function NavPanel({ navState, deviceHeading, status, onStop }) {
  if (status === 'arrived') {
    return (
      <div className="nav-panel arrived" role="status" aria-live="assertive">
        <p className="arrived-text">You have arrived!</p>
        <button className="btn-stop" onClick={onStop} aria-label="End navigation">
          Done
        </button>
      </div>
    );
  }

  if (status === 'routing') {
    return (
      <div className="nav-panel" role="status" aria-live="polite">
        <p className="nav-hint">Calculating route…</p>
      </div>
    );
  }

  if (!navState) return null;

  const { bearing, distance } = navState;

  // If device heading is available, show relative arrow; otherwise show absolute bearing
  const arrowAngle =
    deviceHeading != null
      ? relativeBearing(deviceHeading, bearing)
      : bearing;

  const progress = Math.round((navState.waypointIndex / navState.total) * 100);

  return (
    <div className="nav-panel" role="navigation" aria-label="Navigation guidance">
      <div className="nav-top">
        {/* Direction arrow */}
        <div
          className="nav-arrow"
          style={{ transform: `rotate(${arrowAngle}deg)` }}
          aria-label={`Direction: ${cardinalLabel(bearing)}`}
          role="img"
        >
          ↑
        </div>

        <div className="nav-info">
          <span className="nav-distance" aria-label={`Distance: ${formatDistance(distance)}`}>
            {formatDistance(distance)}
          </span>
          <span className="nav-bearing">
            {Math.round(bearing)}° {cardinalLabel(bearing)}
          </span>
        </div>
      </div>

      {/* Progress bar */}
      <div className="nav-progress-track" aria-label={`Route progress: ${progress}%`}>
        <div className="nav-progress-fill" style={{ width: `${progress}%` }} />
      </div>

      <button
        className="btn-stop"
        onClick={onStop}
        aria-label="Stop navigation"
      >
        Stop
      </button>
    </div>
  );
}