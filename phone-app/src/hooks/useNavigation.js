import { useState, useEffect, useCallback } from 'react';
import { computeBearing, computeDistance } from '../utils/geo';
import { fetchRoute } from '../utils/routing';

const WAYPOINT_ADVANCE_RADIUS_M = 8;

/**
 * Navigation state machine.
 *   idle → routing → navigating → arrived
 *
 * `navState` contains { bearing, distance, waypointIndex, total } when navigating.
 */
export function useNavigation(position) {
  const [status, setStatus] = useState('idle'); // idle | routing | navigating | arrived
  const [waypoints, setWaypoints] = useState([]);
  const [waypointIndex, setWaypointIndex] = useState(0);
  const [navState, setNavState] = useState(null);
  const [error, setError] = useState(null);

  const startNavigation = useCallback(
    async (destination) => {
      if (!position) {
        setError('GPS not available yet. Wait for a fix and try again.');
        return;
      }
      setStatus('routing');
      setError(null);
      try {
        const wpts = await fetchRoute(position, destination);
        setWaypoints(wpts);
        setWaypointIndex(0);
        setStatus('navigating');
      } catch (e) {
        setError(e.message);
        setStatus('idle');
      }
    },
    [position]
  );

  const stopNavigation = useCallback(() => {
    setStatus('idle');
    setWaypoints([]);
    setWaypointIndex(0);
    setNavState(null);
    setError(null);
  }, []);

  // Recompute bearing / distance every GPS update
  useEffect(() => {
    if (status !== 'navigating' || !position || waypoints.length === 0) return;

    const target = waypoints[waypointIndex];
    const distance = computeDistance(position, target);
    const bearing = computeBearing(position, target);

    setNavState({ bearing, distance, waypointIndex, total: waypoints.length });

    if (distance < WAYPOINT_ADVANCE_RADIUS_M) {
      if (waypointIndex >= waypoints.length - 1) {
        setStatus('arrived');
      } else {
        setWaypointIndex((i) => i + 1);
      }
    }
  }, [position, waypoints, waypointIndex, status]);

  return {
    status,
    waypoints,
    waypointIndex,
    navState,
    error,
    startNavigation,
    stopNavigation,
  };
}