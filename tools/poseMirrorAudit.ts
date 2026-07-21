#!/usr/bin/env -S node --no-warnings
/**
 * poseMirrorAudit.ts
 * -----------------
 * Audits every pose in PoseDatabase.java (read live from disk on every run;
 * no hardcoded values anywhere) and reports how far each RED pose deviates
 * from the perfect mirror of its BLUE counterpart.
 *
 * The Java source is located automatically by walking up from the script's
 * directory looking for:
 *   TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/Auto/Modular/PoseDatabase.java
 * Override with POSE_DATABASE=/path/to/PoseDatabase.java if needed.
 *
 * Mirror rule (FTC field, x along the long axis, y along the short axis):
 *   mirroredX = FIELD_WIDTH - blueX
 *   mirroredY = blueY
 *   mirroredHeading = wrapDeg(-blueHeading)
 *
 * Field width defaults to 141.5" but is auto-detected from the Java source:
 *   1. `public static final double FIELD_LENGTH = X;` constant, if present.
 *   2. The literal argument to `.mirror(...)`, if used.
 *   3. The CLI override, if provided.
 *
 * Usage:
 *   npx tsx tools/poseMirrorAudit.ts           # uses detected/default 141.5"
 *   npx tsx tools/poseMirrorAudit.ts 144       # override with 144"
 *
 * The audit handles two RED declaration styles:
 *   - Derived  `RED_X = BLUE_X.mirror(W);`  → reported as MATCH by construction.
 *   - Literal  `RED_X = new Pose(...);`     → numeric deltas reported against M.
 */

import * as fs from "node:fs";
import * as path from "node:path";

// ---------- ANSI helpers ----------
const c = {
	reset: "\x1b[0m",
	dim: "\x1b[2m",
	bold: "\x1b[1m",
	red: "\x1b[31m",
	green: "\x1b[32m",
	yellow: "\x1b[33m",
	blue: "\x1b[34m",
	magenta: "\x1b[35m",
	cyan: "\x1b[36m",
	gray: "\x1b[90m",
	bgRed: "\x1b[41m",
	bgGreen: "\x1b[42m",
	bgYellow: "\x1b[43m",
	bgBlue: "\x1b[44m",
	bgMagenta: "\x1b[45m",
	bgCyan: "\x1b[46m",
	bgGray: "\x1b[100m",
};

const pad = (s: string, n: number, align: "left" | "right" = "left") => {
	if (s.length >= n) return s;
	const fill = " ".repeat(n - s.length);
	return align === "left" ? s + fill : fill + s;
};

const fmt = (n: number, w = 6, p = 2) => pad(n.toFixed(p), w, "right");
const wrapDeg = (deg: number): number => {
	const d = ((deg % 360) + 360) % 360;
	return d > 180 ? d - 360 : d;
};

// ---------- Parse Java file ----------
type Pose = { x: number; y: number; headingDeg: number };

/**
 * Parse all BLUE_/RED_ constants. Supports two RED declaration styles:
 *   1. Literal:    `public static final Pose RED_X = new Pose(x, y, Math.toRadians(h));`
 *   2. Derived:    `public static final Pose RED_X = BLUE_X.mirror();`
 * Style 2 has `literalPose = null` and `redExpr = "BLUE_X.mirror()"` instead.
 */
type ParsedRed = { literalPose: Pose | null; redExpr: string | null };

function parsePosesFromJava(src: string): {
	blue: Record<string, Pose>;
	red: Record<string, ParsedRed>;
	sourceBytes: number;
} {
	const blue: Record<string, Pose> = {};
	const red: Record<string, ParsedRed> = {};

	// Literal-form: RED_X = new Pose(...)
	const literalRe =
		/public\s+static\s+final\s+Pose\s+(BLUE|RED)_([A-Z0-9_]+)\s*=\s*new\s*Pose\(\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*(?:Math\.toRadians\(\s*([-\d.]+)\s*\)|([-\d.]+))\s*\)\s*;/g;
	let m: RegExpExecArray | null;
	while ((m = literalRe.exec(src)) !== null) {
		const [, color, name, xs, ys, hsRadians, hsRaw] = m;
		const key = `${color}_${name}`;
		const headingDeg = hsRadians !== undefined ? parseFloat(hsRadians) : parseFloat(hsRaw);
		const pose: Pose = { x: parseFloat(xs), y: parseFloat(ys), headingDeg };
		if (color === "BLUE") blue[name] = pose;
		else red[name] = { literalPose: pose, redExpr: null };
	}

	// Derived-form: RED_X = BLUE_X.mirror();   or   RED_X = BLUE_X.mirror(141.5);
	// We also capture the literal field length argument so the audit can match
	// the Java code's intended field dimension exactly.
	const derivedRe =
		/public\s+static\s+final\s+Pose\s+RED_([A-Z0-9_]+)\s*=\s*(BLUE_[A-Z0-9_]+)\.mirror\(\s*([^)]*)\)\s*;/g;
	while ((m = derivedRe.exec(src)) !== null) {
		const [, name, blueRef, arg] = m;
		const expr = arg.trim() === "" ? `${blueRef}.mirror()` : `${blueRef}.mirror(${arg.trim()})`;
		// Only overwrite if we didn't already capture a literal (literal takes precedence in display).
		if (!red[name]) {
			red[name] = { literalPose: null, redExpr: expr };
		}
	}

	return { blue, red, sourceBytes: src.length };
}

function pairPoses(blue: Record<string, Pose>, red: Record<string, ParsedRed>) {
	const rows: { name: string; blue: Pose | undefined; red: ParsedRed | undefined }[] = [];
	const seen = new Set<string>();
	for (const name of Object.keys(blue)) {
		if (seen.has(name)) continue;
		seen.add(name);
		rows.push({ name, blue: blue[name], red: red[name] });
	}
	for (const name of Object.keys(red)) {
		if (seen.has(name)) continue;
		seen.add(name);
		rows.push({ name, blue: blue[name], red: red[name] });
	}
	rows.sort((a, b) => a.name.localeCompare(b.name));
	return rows;
}

// ---------- Audit ----------
type Severity = "ok" | "warn" | "fail";

function classify(maxDelta: number): { sev: Severity; label: string; color: string } {
	if (maxDelta < 0.01)
		return { sev: "ok", label: "MATCH", color: c.green };
	if (maxDelta < 1.0)
		return { sev: "warn", label: "CLOSE", color: c.yellow };
	return { sev: "fail", label: "OFF ", color: c.red };
}

/**
 * Audit one paired pose.
 *
 * Behaviour depends on how RED_<name> is declared in the Java file:
 *
 *   - Derived form  (`RED_X = BLUE_X.mirror();`):
 *       We can't execute Java from Node, so we *simulate* `.mirror()` with the
 *       standard FTC formula `(x, y, θ) → (W − x, y, −θ)` (W = fieldLength arg,
 *       default 144). This is exactly what PedroPathing's mirror() does, so a
 *       derived red pose is reported as MATCH by construction.
 *
 *   - Literal form  (`RED_X = new Pose(...)`):
 *       We compare the literal red pose to the simulated mirror and report
 *       deltas (this is the legacy code path).
 */
type AuditRow = {
	name: string;
	blue: Pose;
	redExpr: string | null;       // null = literal red, "BLUE_X.mirror()" = derived
	redLiteral: Pose | null;       // only set for literal-form red
	// Mirror expectations
	mx: number;
	my: number;
	mh: number;
	// Deltas (always zero for derived reds)
	dx: number;
	dy: number;
	dhRaw: number;
	dhEquiv: number;
	distXY: number;
	maxDelta: number;
	sev: Severity;
	label: string;
	color: string;
};

function auditRow(
	name: string,
	blue: Pose,
	red: ParsedRed | undefined,
	fieldW: number,
): AuditRow {
	const mx = fieldW - blue.x;
	const my = blue.y;
	const mh = wrapDeg(-blue.headingDeg);

	const redExpr = red?.redExpr ?? null;
	const redLiteral = red?.literalPose ?? null;

	let dx = 0,
		dy = 0,
		dhRaw = 0;
	if (redLiteral) {
		dx = redLiteral.x - mx;
		dy = redLiteral.y - my;
		dhRaw = wrapDeg(redLiteral.headingDeg - mh);
	}
	const dhEquiv = Math.min(Math.abs(dhRaw), Math.abs(Math.abs(dhRaw) - 180));
	const distXY = Math.hypot(dx, dy);
	const maxDelta = Math.max(Math.abs(dx), Math.abs(dy));
	const cls = classify(maxDelta);

	return {
		name,
		blue,
		redExpr,
		redLiteral,
		mx,
		my,
		mh,
		dx,
		dy,
		dhRaw,
		dhEquiv,
		distXY,
		maxDelta,
		...cls,
	};
}

// ---------- Rendering ----------
const W = process.stdout.columns ?? 110;

function banner(title: string, subtitle?: string) {
	const line = "━".repeat(W);
	console.log(c.cyan + line + c.reset);
	const t = ` ${title} `;
	const left = Math.max(0, Math.floor((W - t.length) / 2));
	console.log(" ".repeat(left) + c.bold + c.cyan + t + c.reset);
	if (subtitle) {
		const s = ` ${subtitle} `;
		const l2 = Math.max(0, Math.floor((W - s.length) / 2));
		console.log(" ".repeat(l2) + c.dim + s + c.reset);
	}
	console.log(c.cyan + line + c.reset);
}

function section(title: string) {
	console.log("\n" + c.bold + c.magenta + "▸ " + title + c.reset);
	console.log(c.gray + "  " + "─".repeat(W - 2) + c.reset);
}

function colorize(n: number, sev: Severity, p = 2): string {
	const s = n.toFixed(p);
	if (sev === "ok") return c.green + s + c.reset;
	if (sev === "warn") return c.yellow + s + c.reset;
	return c.bold + c.red + s + c.reset;
}

// (renderRow removed — output is rendered inline in main() for finer control)

function summary(rows: ReturnType<typeof auditRow>[]) {
	const ok = rows.filter((r) => r.sev === "ok").length;
	const warn = rows.filter((r) => r.sev === "warn").length;
	const fail = rows.filter((r) => r.sev === "fail").length;
	const worst = [...rows].sort((a, b) => b.maxDelta - a.maxDelta)[0];

	const cell = (n: number, label: string, bg: string, fg = c.bold) =>
		`${bg}${fg} ${label}: ${n} ${c.reset}`;
	const total = rows.length;

	console.log();
	banner("SUMMARY", `${ok} match · ${warn} close · ${fail} off · ${total} total`);
	const parts = [
		cell(ok, "MATCH", c.bgGreen, c.bold),
		cell(warn, "CLOSE", c.bgYellow, c.bold),
		cell(fail, "OFF  ", c.bgRed, c.bold),
	];
	console.log("  " + parts.join("  "));
	if (worst) {
		console.log(
			"\n  " +
				c.gray +
				"Worst positional deviation: " +
				c.reset +
				c.bold +
				worst.name +
				c.reset +
				c.gray +
				`  (max |Δxy| = ${worst.maxDelta.toFixed(3)}, |XY| = ${worst.distXY.toFixed(3)})` +
				c.reset,
		);
	}
	console.log();
}

// ---------- Pretty bar ----------
function bar(value: number, max: number, width = 30): string {
	const ratio = Math.min(1, Math.max(0, value / Math.max(1e-9, max)));
	const filled = Math.round(ratio * width);
	const empty = width - filled;
	let color = c.green;
	if (value > 1) color = c.red;
	else if (value > 0.01) color = c.yellow;
	return color + "█".repeat(filled) + c.gray + "░".repeat(empty) + c.reset;
}

// ---------- Resolve PoseDatabase.java from anywhere ----------
function findPoseDatabase(): string {
	const REL = path.join(
		"TeamCode",
		"src",
		"main",
		"java",
		"org",
		"firstinspires",
		"ftc",
		"teamcode",
		"OpModes",
		"Auto",
		"Modular",
		"PoseDatabase.java",
	);

	// Search from the script's directory upward — works no matter where `tsx` was invoked.
	let dir = __dirname;
	for (let i = 0; i < 8; i++) {
		const candidate = path.join(dir, REL);
		if (fs.existsSync(candidate)) return candidate;
		const parent = path.dirname(dir);
		if (parent === dir) break;
		dir = parent;
	}

	// Fallback: relative to cwd.
	const cwdCandidate = path.join(process.cwd(), REL);
	if (fs.existsSync(cwdCandidate)) return cwdCandidate;

	// Last resort: explicit override via env var.
	if (process.env.POSE_DATABASE && fs.existsSync(process.env.POSE_DATABASE)) {
		return process.env.POSE_DATABASE;
	}

	throw new Error(`Could not locate PoseDatabase.java. Tried under ${__dirname} (searched 8 levels up) and ${process.cwd()}. Set POSE_DATABASE=/path/to/PoseDatabase.java to override.`);
}

// ---------- Detect the field length the Java code is using ----------
function detectFieldLength(src: string): { value: number; source: string } | null {
	// 1. Named constant:  public static final double FIELD_LENGTH = 141.5;
	const m1 = /public\s+static\s+final\s+double\s+FIELD_LENGTH\s*=\s*([-\d.]+)\s*;/.exec(src);
	if (m1) return { value: parseFloat(m1[1]), source: "Java FIELD_LENGTH constant" };
	// 2. Inline literal:  ... .mirror(141.5);
	const literals = [...src.matchAll(/\.mirror\(\s*([-\d.]+)\s*\)/g)].map((mm) => parseFloat(mm[1]));
	if (literals.length > 0) {
		const uniq = [...new Set(literals)];
		if (uniq.length === 1) return { value: uniq[0], source: "literal arg to .mirror()" };
		return { value: uniq[0], source: `mixed literals ${uniq.join(", ")} (using first)` };
	}
	return null;
}

// ---------- Main ----------
function main() {
	// CLI: argv[2] = optional field length override.
	// Otherwise auto-detect from the Java source: prefer FIELD_LENGTH constant,
	// then the literal argument to .mirror(), and finally fall back to 141.5.
	const CLI_FIELD = process.argv[2];
	const file = findPoseDatabase();
	const stat = fs.statSync(file);
	const src = fs.readFileSync(file, "utf8");
	const mtime = stat.mtime.toISOString().replace("T", " ").slice(0, 19);

	const detected = detectFieldLength(src);
	const FIELD_WIDTH = CLI_FIELD !== undefined ? parseFloat(CLI_FIELD) : (detected?.value ?? 141.5);

	banner(
		"POSE MIRROR AUDIT",
		`Blue is treated as ground truth · Field width = ${FIELD_WIDTH}"`,
	);

	// Confirm we're reading live data (no hardcoded poses anywhere).
	const sourceNote = CLI_FIELD !== undefined
		? " (CLI override)"
		: detected
			? ` (auto-detected: ${detected.source})`
			: " (default — no FIELD_LENGTH found in source)";
	const fileLabel = c.dim + `source: ${file}  ·  ${src.length} bytes  ·  mtime ${mtime} UTC${sourceNote}` + c.reset;
	console.log("  " + fileLabel);

	const { blue: blueOnly, red: redOnly } = parsePosesFromJava(src);

	const allRows = pairPoses(blueOnly, redOnly);
	const rows = allRows.filter((r) => r.blue && r.red);
	const unmatched = allRows.filter((r) => !r.blue || !r.red);

	if (rows.length === 0) {
		console.error(c.red + "✗ No paired BLUE_/RED_ poses found." + c.reset);
		process.exit(1);
	}

	const audited = rows.map((r) => auditRow(r.name, r.blue!, r.red, FIELD_WIDTH));

	// Sort: worst deviation first, but stable by name within the same severity.
	// Derived reds (.mirror(...)) are pushed to the bottom inside the OK bucket.
	const sevRank: Record<Severity, number> = { fail: 0, warn: 1, ok: 2 };
	audited.sort((a, b) => {
		const d = sevRank[a.sev] - sevRank[b.sev];
		if (d !== 0) return d;
		if (a.maxDelta !== b.maxDelta) return b.maxDelta - a.maxDelta;
		const aDer = a.redExpr ? 1 : 0;
		const bDer = b.redExpr ? 1 : 0;
		if (aDer !== bDer) return aDer - bDer;
		return a.name.localeCompare(b.name);
	});

	// Header
	console.log(
		"\n  " +
			c.bold +
			pad("STATUS", 7) +
			pad("POSE", 32) +
			"DEVIATIONS" +
			c.reset,
	);
	console.log("  " + c.gray + "─".repeat(W - 2) + c.reset);

	const maxDelta = Math.max(...audited.map((r) => r.maxDelta), 1);

	for (const r of audited) {
		const tag = r.color + c.bold + ` ${r.label} ` + c.reset;
		const nameStr = c.bold + pad(r.name, 32) + c.reset;
		const bStr = c.blue + `B(${fmt(r.blue.x)},${fmt(r.blue.y)})` + c.reset;
		const mStr =
			c.dim + `M(${fmt(r.mx)},${fmt(r.my)},${fmt(r.mh, 6, 1)}°)` + c.reset;
		// R column: for derived reds we show "= BLUE.mirror()" instead of literal numbers.
		let rStr: string;
		if (r.redExpr) {
			rStr = c.red + `R = ${r.redExpr.replace(/^BLUE_/, "")}` + c.reset;
		} else if (r.redLiteral) {
			rStr = c.red + `R(${fmt(r.redLiteral.x)},${fmt(r.redLiteral.y)})` + c.reset;
		} else {
			rStr = c.dim + `R(?)` + c.reset;
		}
		const dStr =
			`Δx=${colorize(r.dx, r.sev)}  Δy=${colorize(r.dy, r.sev)}  ` +
			`Δθ=${c.dim}${r.dhRaw.toFixed(1).padStart(7)}°${c.reset}  ` +
			`${c.dim}(equiv ${r.dhEquiv.toFixed(1).padStart(6)}°)${c.reset}`;
		console.log(
			`  ${tag} ${nameStr}${bStr} ${c.gray}→${c.reset} ${mStr} ${c.gray}→${c.reset} ${rStr}  ${dStr}`,
		);
		const tag2 = r.redExpr
			? `${c.dim}derived${c.reset} `
			: `${c.dim}literal${c.reset} `;
		console.log(`        ${bar(r.maxDelta, maxDelta)} ${c.dim}max |Δxy|=${r.maxDelta.toFixed(3)}  |XY|=${r.distXY.toFixed(3)}  [${tag2.trimEnd()}]${c.reset}`);
	}

	summary(audited);

	// Severity legend
	section("Legend");
	console.log("  " + c.bgGreen + c.bold + " MATCH " + c.reset + " |Δx|,|Δy| < 0.01  (position is exact mirror)");
	console.log("  " + c.bgYellow + c.bold + " CLOSE " + c.reset + " max(|Δx|,|Δy|) < 1.00  (within 1\")");
	console.log("  " + c.bgRed + c.bold + " OFF   " + c.reset + " max(|Δx|,|Δy|) ≥ 1.00  (positional mismatch)");
	console.log("  " + c.dim + "Row tags: [derived] = RED_X is BLUE_X.mirror(W), MATCH by construction;");
	console.log("  " + c.dim + "           [literal] = RED_X is a hand-written new Pose(...), deltas reported.");
	console.log("  " + c.dim + "Δθ shows the naive heading delta; (equiv ...) is the heading delta after");
	console.log("  " + c.dim + "collapsing the 180° flip (which is geometrically the same line).");
	console.log("\n  " + c.dim + "Mirror rule:  M = (W − Bx, By, −Bθ)    Δ = R − M    (Δθ uses wrap-aware shortest equivalent)" + c.reset);

	if (unmatched.length > 0) {
		section("Unpaired poses (skipped from audit)");
		for (const u of unmatched) {
			const side = u.blue ? c.blue + "BLUE" + c.reset : c.red + "RED " + c.reset;
			const missing = u.blue ? "RED_" : "BLUE_";
			console.log(
				"  " +
					side +
					"  " +
					c.bold +
					u.name +
					c.reset +
					c.dim +
					`   (only one side defined — missing ${missing}${u.name})` +
					c.reset,
			);
		}
	}
}

main();
