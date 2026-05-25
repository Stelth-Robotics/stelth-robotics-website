/** Planar 4-DOF arm kinematics (SVG: +x right, +y down, angles in degrees). */

/** Slightly longer forearm + wrist links for far pile reach (visual matches SVG). */
export const LINK_LENGTHS = [60, 78.1, 57.8, 40.2] as const;
export const HOME_ANGLES: [number, number, number, number] = [-90, 50.2, -5.2, 5.2];
export const JOINT_LIMITS: [number, number][] = [
	[-175, 175],
	[-175, 175],
	[-175, 175],
	[-175, 175],
];

/** Max joint speed (deg/s): base → shoulder → elbow → wrist. */
export const MAX_JOINT_SPEED_DEG_S = [30, 44, 64, 96] as const;

const DEG = Math.PI / 180;

export type Pose2D = { x: number; y: number; phi: number };

export function fkDeg(q: readonly number[]): Pose2D {
	let x = 0;
	let y = 0;
	let theta = 0;
	for (let i = 0; i < 4; i++) {
		theta += q[i] * DEG;
		x += LINK_LENGTHS[i] * Math.cos(theta);
		y += LINK_LENGTHS[i] * Math.sin(theta);
	}
	return { x, y, phi: theta / DEG };
}

function clampAngles(q: number[]): number[] {
	return q.map((v, i) => {
		const a = normAngleDeg(v);
		const [lo, hi] = JOINT_LIMITS[i];
		return Math.max(lo, Math.min(hi, a));
	});
}

type Point = { x: number; y: number };

/** Joint positions [base, j1, j2, j3, tip]. */
function jointPositions(q: readonly number[]): Point[] {
	const pts: Point[] = [{ x: 0, y: 0 }];
	let theta = 0;
	for (let i = 0; i < 4; i++) {
		theta += q[i] * DEG;
		const p = pts[pts.length - 1];
		pts.push({
			x: p.x + LINK_LENGTHS[i] * Math.cos(theta),
			y: p.y + LINK_LENGTHS[i] * Math.sin(theta),
		});
	}
	return pts;
}

export function normAngleDeg(deg: number): number {
	let a = deg;
	while (a > 180) a -= 360;
	while (a < -180) a += 360;
	return a;
}

/** CCD IK — stable for pick-and-place paths. */
function solveIk(target: Pose2D, seed: readonly number[], iterations: number): number[] {
	const q = [...seed];
	const tx = target.x;
	const ty = target.y;

	for (let pass = 0; pass < iterations; pass++) {
		let pts = jointPositions(q);

		// Orient end link toward target.phi
		let dPhi = normAngleDeg(target.phi - fkDeg(q).phi);
		q[3] += dPhi * 0.55;
		clampAngles(q);
		pts = jointPositions(q);

		// Position: pull tip toward target from each joint
		for (let i = 3; i >= 0; i--) {
			const j = pts[i];
			const toTip = Math.atan2(pts[4].y - j.y, pts[4].x - j.x);
			const toGoal = Math.atan2(ty - j.y, tx - j.x);
			let delta = normAngleDeg((toGoal - toTip) / DEG);
			q[i] += delta * 0.48;
			clampAngles(q);
			pts = jointPositions(q);
		}

		const tip = pts[4];
		const err = Math.hypot(tip.x - tx, tip.y - ty);
		if (err < 0.8 && Math.abs(normAngleDeg(target.phi - fkDeg(q).phi)) < 5) break;
	}

	return q;
}

/** CCD IK with fallback seed when the chain gets stuck on joint limits. */
export function ikDeg(target: Pose2D, seed: readonly number[], iterations = 28): number[] {
	const q = solveIk(target, seed, iterations);
	const tip = fkDeg(q);
	const err = Math.hypot(tip.x - target.x, tip.y - target.y);
	if (err > 2.5) {
		const alt = solveIk(target, HOME_ANGLES, Math.max(iterations, 36));
		const altTip = fkDeg(alt);
		if (Math.hypot(altTip.x - target.x, altTip.y - target.y) < err) return alt;
	}
	return q;
}

export function lerpPose(a: Pose2D, b: Pose2D, t: number): Pose2D {
	const u = Math.max(0, Math.min(1, t));
	let dphi = normAngleDeg(b.phi - a.phi);
	return {
		x: a.x + (b.x - a.x) * u,
		y: a.y + (b.y - a.y) * u,
		phi: a.phi + dphi * u,
	};
}

export function smoothstep(t: number): number {
	const x = Math.max(0, Math.min(1, t));
	return x * x * (3 - 2 * x);
}

/** Ease-out only — for gentle deceleration into place (no end snap). */
export function easeOutCubic(t: number): number {
	const x = Math.max(0, Math.min(1, t));
	return 1 - (1 - x) ** 3;
}

/** Ease-in — for gentle departure from rest (first move off home). */
export function easeInCubic(t: number): number {
	const x = Math.max(0, Math.min(1, t));
	return x * x * x;
}

/** Smooth acceleration/deceleration along a segment. */
export function easeInOutCubic(t: number): number {
	const x = Math.max(0, Math.min(1, t));
	return x < 0.5 ? 4 * x * x * x : 1 - (-2 * x + 2) ** 3 / 2;
}

/** Low-pass joint tracking — reduces jerky IK steps between frames. */
export function smoothFollowJoints(
	prev: readonly number[],
	target: readonly number[],
	dtMs: number,
	tauMs = 100,
): number[] {
	const alpha = 1 - Math.exp(-dtMs / Math.max(20, tauMs));
	return prev.map((p, i) => {
		const d = normAngleDeg(target[i] - p);
		return p + d * alpha;
	});
}

/** Limit per-joint step using max RPM (smoothest when dt is frame delta). */
export function rateLimitJoints(
	prev: readonly number[],
	target: readonly number[],
	dtMs: number,
	speedScale = 1,
): number[] {
	if (dtMs <= 0) return [...target];
	const dtSec = dtMs / 1000;
	const scale = Math.max(0.35, Math.min(2.2, speedScale));
	return target.map((goal, i) => {
		let delta = normAngleDeg(goal - prev[i]);
		const cap = MAX_JOINT_SPEED_DEG_S[i] * dtSec * scale;
		if (Math.abs(delta) > cap) return prev[i] + Math.sign(delta) * cap;
		return goal;
	});
}

function catmullScalar(p0: number, p1: number, p2: number, p3: number, t: number): number {
	const t2 = t * t;
	const t3 = t2 * t;
	return (
		0.5 *
		(2 * p1 +
			(-p0 + p2) * t +
			(2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 +
			(-p0 + 3 * p1 - 3 * p2 + p3) * t3)
	);
}

export type PathWaypoint = {
	pose: Pose2D;
	ms: number;
	/** Use straight interpolation (avoids Catmull–Rom overshoot on vertical moves). */
	linear?: boolean;
	/** Constant speed along segment (transit — no ease-in-out pulsing). */
	cruise?: boolean;
	/** Ease-in on linear segment (departure from rest / home). */
	easeIn?: boolean;
	/** Ease-out on linear segment (for final approach to pick/place). */
	easeOut?: boolean;
	/** Fired when this segment passes the given progress (works even if IK lags). */
	tag?: 'grasp' | 'release';
	tagAt?: number;
};

export type PathSegmentInfo = { index: number; localT: number; elapsedMs: number };

/** Which path segment is active and normalized progress along it. */
export function pathSegmentAt(waypoints: PathWaypoint[], elapsedMs: number): PathSegmentInfo {
	if (waypoints.length === 0) return { index: 0, localT: 0, elapsedMs: 0 };

	let acc = 0;
	for (let i = 0; i < waypoints.length; i++) {
		const dur = waypoints[i].ms;
		if (elapsedMs <= acc + dur || i === waypoints.length - 1) {
			const localT = dur > 0 ? Math.min(1, Math.max(0, (elapsedMs - acc) / dur)) : 1;
			return { index: i, localT, elapsedMs };
		}
		acc += dur;
	}

	const last = waypoints.length - 1;
	return { index: last, localT: 1, elapsedMs };
}

/** Sample pose along waypoints with Catmull–Rom on position (continuous motion, no segment stops). */
export function samplePath(waypoints: PathWaypoint[], elapsedMs: number): Pose2D {
	if (waypoints.length === 0) return { x: 0, y: 0, phi: 0 };
	if (waypoints.length === 1) return waypoints[0].pose;

	const total = waypoints.reduce((s, w) => s + w.ms, 0);
	if (elapsedMs <= 0) return waypoints[0].pose;
	if (elapsedMs >= total) return waypoints[waypoints.length - 1].pose;

	const poses = waypoints.map((w) => w.pose);
	let acc = 0;

	for (let i = 0; i < waypoints.length; i++) {
		const dur = waypoints[i].ms;
		if (elapsedMs <= acc + dur || i === waypoints.length - 1) {
			const localT = Math.min(1, Math.max(0, (elapsedMs - acc) / dur));
			const p1 = poses[i];
			const p2 = poses[Math.min(poses.length - 1, i + 1)];

			if (waypoints[i].linear) {
				let u = localT;
				if (waypoints[i].easeOut) u = easeOutCubic(localT);
				else if (waypoints[i].easeIn) u = easeInCubic(localT);
				else if (!waypoints[i].cruise) u = easeInOutCubic(localT);
				return lerpPose(p1, p2, u);
			}

			const p0 = poses[Math.max(0, i - 1)];
			const p3 = poses[Math.min(poses.length - 1, i + 2)];
			const u = localT;
			const dphi = normAngleDeg(p2.phi - p1.phi);
			return {
				x: catmullScalar(p0.x, p1.x, p2.x, p3.x, u),
				y: catmullScalar(p0.y, p1.y, p2.y, p3.y, u),
				phi: p1.phi + dphi * u,
			};
		}
		acc += dur;
	}

	return poses[poses.length - 1];
}

export function pathDuration(waypoints: PathWaypoint[]): number {
	return waypoints.reduce((s, w) => s + w.ms, 0);
}

