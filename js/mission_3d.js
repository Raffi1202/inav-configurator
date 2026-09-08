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
        isRoutePoint: ROUTE_ACTIONS.has(action),
        endsMission: false,
        jumps: []
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
        isRoutePoint: false,
        endsMission: false,
        jumps: []
    };
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

function isMission3DJump(waypoint) {
    return waypoint.isAttached() && waypoint.getAction() === MWNP.WPTYPE.JUMP;
}

// Folds one waypoint into the route being built. `missionStartNumber` is the storage number of the
// first entry of the current sub-mission: a JUMP's P1 counts from there, the same way
// repaintLine4Waypoints() resolves it for the 2D editor.
function addMission3DWaypoint(route, waypoint) {
    const point = waypoint.isAttached() ? null : buildMission3DPoint(waypoint);

    if (point) {
        route.points.push(point);
        if (point.isRoutePoint) route.lastRoutePoint = point;
    } else if (isMission3DJump(waypoint) && route.lastRoutePoint) {
        // The jump is flown from the route point it is attached to back (or forward) to its
        // target. The target is resolved once every point is known, since it may come later.
        route.jumps.push({
            source: route.lastRoutePoint,
            targetWaypointNumber: route.missionStartNumber + Number(waypoint.getP1()),
            repeat: Number(waypoint.getP2())
        });
    }

    if (terminatesMission3DRoute(waypoint)) {
        const lastPoint = route.points.at(-1);
        if (lastPoint) lastPoint.endsMission = true;
        route.lastRoutePoint = null;
    }
    if (waypoint.getEndMission() === 0xA5) route.missionStartNumber = waypoint.getNumber() + 1;
}

function attachMission3DJumps(points, jumps) {
    const pointsByWaypointNumber = new Map(points.map((point) => [point.waypointNumber, point]));

    jumps.forEach((jump) => {
        const target = pointsByWaypointNumber.get(jump.targetWaypointNumber);
        if (!target?.isRoutePoint || target === jump.source) return;
        jump.source.jumps.push({targetWaypointNumber: target.waypointNumber, repeat: jump.repeat});
    });
}

export function getMission3DPoints(waypoints, home) {
    const route = {points: [], jumps: [], lastRoutePoint: null, missionStartNumber: 0};

    waypoints.forEach((waypoint) => addMission3DWaypoint(route, waypoint));
    attachMission3DJumps(route.points, route.jumps);

    if (hasValidHomePosition(home)) {
        route.points.unshift(buildMission3DHomePoint(home));
    }

    return route.points;
}

export function getMission3DPlannedHeight(point, groundHeight, homeGroundHeight) {
    if (point.isHome) return groundHeight;
    if (point.absoluteAltitude) return point.altitude;
    if (!Number.isFinite(homeGroundHeight)) return groundHeight + point.altitude;
    return homeGroundHeight + point.altitude;
}

export function getMission3DRouteSegments(points) {
    const segments = [];
    let segment = [];

    points.forEach((point) => {
        if (point.isRoutePoint) segment.push(point);
        if (point.endsMission && segment.length) {
            segments.push(segment);
            segment = [];
        }
    });

    if (segment.length) segments.push(segment);
    return segments;
}

// The legs a JUMP adds to the flown route: from the point carrying the jump to its target.
// Repeated legs after the target reuse edges the route already samples, so the return leg is the
// only new ground to check. The leg is drawn whether or not the repeat count is zero, matching the
// "Repeat x" line of the 2D editor.
export function getMission3DJumpSegments(points) {
    const pointsByWaypointNumber = new Map(points.map((point) => [point.waypointNumber, point]));
    const segments = [];

    points.forEach((point) => {
        (point.jumps || []).forEach((jump) => {
            const target = pointsByWaypointNumber.get(jump.targetWaypointNumber);
            if (!target || target === point) return;
            segments.push({start: point, end: target, repeat: jump.repeat});
        });
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
