'use strict';

import MWNP from './mwnp.js';

const ROUTE_ACTIONS = new Set([
    MWNP.WPTYPE.WAYPOINT,
    MWNP.WPTYPE.POSHOLD_UNLIM,
    MWNP.WPTYPE.POSHOLD_TIME,
    MWNP.WPTYPE.LAND
]);

function hasValidHomePosition(home) {
    if (!home?.getLat || !home?.getLon) return false;

    const lat = Number(home.getLat());
    const lon = Number(home.getLon());
    return Number.isFinite(lat) && Number.isFinite(lon) && !(lat === 0 && lon === 0);
}

function isRouteWaypoint(waypoint) {
    return !waypoint.isAttached() && ROUTE_ACTIONS.has(waypoint.getAction());
}

function isJumpWaypoint(waypoint) {
    return waypoint.isAttached() && waypoint.getAction() === MWNP.WPTYPE.JUMP;
}

// RTH and an unattached LAND end the flown route wherever they occur — same firmware semantics
// js/mission_sim.js's getSimulationRoute() stops at — not just when the multi-mission end marker
// happens to be set on that slot.
function terminatesMission3DRoute(waypoint) {
    const action = waypoint.getAction();
    return action === MWNP.WPTYPE.RTH
        || (action === MWNP.WPTYPE.LAND && !waypoint.isAttached())
        || waypoint.getEndMission() === 0xA5;
}

function buildMission3DPoint(waypoint) {
    const layerNumber = waypoint.getLayerNumber();
    const lat = Number(waypoint.getLatMap());
    const lon = Number(waypoint.getLonMap());
    const altitude = Number(waypoint.getAlt()) / 100;
    if (!Number.isFinite(lat) || !Number.isFinite(lon) || !Number.isFinite(altitude)) return null;

    const action = waypoint.getAction();
    return {
        number: layerNumber === 'undefined' ? waypoint.getNumber() : layerNumber,
        waypointNumber: waypoint.getNumber(),
        lat,
        lon,
        altitude,
        absoluteAltitude: (waypoint.getP3() & (1 << MWNP.P3.ALT_TYPE)) !== 0,
        action,
        isHome: false,
        isRoutePoint: ROUTE_ACTIONS.has(action)
    };
}

function buildMission3DHomePoint(home) {
    return {
        number: 'H',
        waypointNumber: null,
        lat: home.getLatMap(),
        lon: home.getLonMap(),
        altitude: Number(home.getAlt()) || 0,
        absoluteAltitude: true,
        action: 0,
        isHome: true,
        isRoutePoint: false
    };
}

// The markers of the mission: every positional waypoint plus HOME when it is set. Attached
// actions have no position of their own; JUMP shapes the route through getMission3DFlightLegs().
export function getMission3DPoints(waypoints, home) {
    const points = waypoints
        .filter((waypoint) => !waypoint.isAttached())
        .map(buildMission3DPoint)
        .filter(Boolean);

    if (hasValidHomePosition(home)) {
        points.unshift(buildMission3DHomePoint(home));
    }

    return points;
}

// Walks the mission the way the firmware flies it and returns every leg in the order it is first
// flown, as pairs of waypoint storage numbers. A JUMP is taken as often as its repeat count says
// (an infinite one once, which already covers all of its ground), so waypoints a forward jump
// skips get no legs and a jump with zero repeats adds none. The leg a jump adds from the point it
// is attached to into its target carries the jump, so the caller can draw it apart. RTH, LAND and
// the sub-mission end marker close the current chain; points after them start a new one, like
// the 2D editor draws them. `maximumSteps` bounds the walk against malformed jump loops.
export function getMission3DFlightLegs(waypoints, maximumSteps = waypoints.length * 64) {
    const indexByNumber = new Map(waypoints.map((waypoint, index) => [waypoint.getNumber(), index]));
    const remainingJumps = new Map();
    const seen = new Set();
    const legs = [];
    let current = null;
    let pendingJump = null;
    let missionStartNumber = 0;

    for (let index = 0, steps = 0; index < waypoints.length && steps < maximumSteps; index++, steps++) {
        const waypoint = waypoints[index];

        if (isRouteWaypoint(waypoint)) {
            const number = waypoint.getNumber();
            const key = `${current}->${number}`;
            if (current !== null && current !== number && !seen.has(key)) {
                seen.add(key);
                legs.push({from: current, to: number, jump: pendingJump});
            }
            pendingJump = null;
            current = number;
        } else if (isJumpWaypoint(waypoint) && current !== null) {
            const targetNumber = missionStartNumber + Number(waypoint.getP1());
            const targetIndex = indexByNumber.get(targetNumber);
            const target = waypoints[targetIndex];
            const repeat = Number(waypoint.getP2());
            if (!remainingJumps.has(index)) remainingJumps.set(index, repeat === -1 ? 1 : Math.max(0, repeat));
            const remaining = remainingJumps.get(index);

            if (remaining > 0 && target && isRouteWaypoint(target) && targetNumber !== current) {
                remainingJumps.set(index, remaining - 1);
                pendingJump = {repeat};
                index = targetIndex - 1;
                continue;
            }
        }

        if (terminatesMission3DRoute(waypoint)) {
            current = null;
            pendingJump = null;
        }
        if (waypoint.getEndMission() === 0xA5) missionStartNumber = waypoint.getNumber() + 1;
    }

    return legs;
}

export function getMission3DPlannedHeight(point, groundHeight, homeGroundHeight) {
    if (point.isHome) return groundHeight;
    if (point.absoluteAltitude) return point.altitude;
    if (!Number.isFinite(homeGroundHeight)) return groundHeight + point.altitude;
    return homeGroundHeight + point.altitude;
}

// Turns the flown legs into polyline segments over the given points (which may be rendered
// copies, so they are matched by waypoint number). Consecutive legs chain into one segment; a
// jump leg is a segment of its own so it can be drawn in the jump colour with its repeat label.
export function getMission3DFlightSegments(points, legs) {
    const pointsByWaypointNumber = new Map(points.map((point) => [point.waypointNumber, point]));
    const segments = [];
    let segment = null;

    legs.forEach((leg) => {
        const start = pointsByWaypointNumber.get(leg.from);
        const end = pointsByWaypointNumber.get(leg.to);
        if (!start || !end) {
            segment = null;
            return;
        }
        if (leg.jump) {
            segments.push({points: [start, end], jump: leg.jump});
            segment = null;
            return;
        }
        if (segment && segment.points.at(-1) === start) {
            segment.points.push(end);
            return;
        }
        segment = {points: [start, end], jump: null};
        segments.push(segment);
    });

    return segments;
}

export function getMission3DJumpLabel(repeat) {
    return 'Repeat x' + (repeat === -1 ? ' infinite' : String(repeat));
}

export function getMission3DSamplingSpacing(edgeDistances, minimumSpacing = 30, maximumSamples = 4096) {
    const distances = edgeDistances.filter((distance) => Number.isFinite(distance) && distance > 0);
    if (!distances.length) return minimumSpacing;

    const totalDistance = distances.reduce((sum, distance) => sum + distance, 0);
    const availableSteps = Math.max(1, maximumSamples - distances.length);
    return Math.max(minimumSpacing, totalDistance / availableSteps);
}

export function getMission3DRouteRuns(samples) {
    const runs = [];

    for (let index = 1; index < samples.length; index++) {
        const previousSample = samples[index - 1];
        const sample = samples[index];
        const terrainClearanceAvailable = previousSample.terrainClearanceAvailable !== false
            && sample.terrainClearanceAvailable !== false;
        const collidesWithTerrain = terrainClearanceAvailable
            && (previousSample.clearance <= 0 || sample.clearance <= 0);
        const currentRun = runs.at(-1);

        if (currentRun?.collidesWithTerrain !== collidesWithTerrain) {
            runs.push({
                collidesWithTerrain,
                samples: [previousSample, sample]
            });
        } else {
            currentRun.samples.push(sample);
        }
    }

    return runs;
}

export function getMission3DPointLabel(point) {
    if (point.isHome) return 'H';

    const number = Number(point.number);
    return String(Number.isFinite(number) ? number + 1 : point.number);
}
