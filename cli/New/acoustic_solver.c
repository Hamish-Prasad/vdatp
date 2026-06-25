#include "acoustic_solver.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct Cx {
	double re;
	double im;
} Cx;

typedef struct ControlPoint {
	NewVec3 p;
	Cx target;
	double weight;
	int null_point;
} ControlPoint;

static Cx cx(double re, double im) { Cx z; z.re = re; z.im = im; return z; }
static Cx cadd(Cx a, Cx b) { return cx(a.re + b.re, a.im + b.im); }
static Cx csub(Cx a, Cx b) { return cx(a.re - b.re, a.im - b.im); }
static Cx cmul(Cx a, Cx b) { return cx(a.re*b.re - a.im*b.im, a.re*b.im + a.im*b.re); }
static Cx cscale(Cx a, double s) { return cx(a.re*s, a.im*s); }
static Cx cconj(Cx a) { return cx(a.re, -a.im); }
static double cabs2(Cx a) { return a.re*a.re + a.im*a.im; }
static Cx cexp_i(double a) { return cx(cos(a), sin(a)); }

static const int16_t x_cols[5] = {450, 350, 250, 150, 50};
static const int16_t z_rows[10] = {-450, -350, -250, -150, -50, 50, 150, 250, 350, 450};

static NewVec3 transducer_pos(int idx, double board_distance_mm)
{
	int board = idx / NEW_CHANNELS_PER_BOARD;
	int ch = idx % NEW_CHANNELS_PER_BOARD;
	int col = ch / 10;
	double x0 = (double)x_cols[col] * 0.1;
	double z0 = (double)z_rows[ch % 10] * 0.1;
	NewVec3 p;
	p.x = (board == 0 || board == 2) ? x0 : -x0;
	p.y = (board == 2 || board == 3) ? -0.5 * board_distance_mm : 0.5 * board_distance_mm;
	p.z = z0;
	return p;
}

static double transducer_normal_sign(int idx)
{
	int board = idx / NEW_CHANNELS_PER_BOARD;
	return (board == 2 || board == 3) ? 1.0 : -1.0;
}

static Cx propagator(const NewSolverConfig *cfg, int idx, NewVec3 p)
{
	NewVec3 t = transducer_pos(idx, cfg->board_distance_mm);
	double dx = p.x - t.x, dy = p.y - t.y, dz = p.z - t.z;
	double r_mm = sqrt(dx*dx + dy*dy + dz*dz);
	if(r_mm < 1e-6) r_mm = 1e-6;
	double k = 2.0 * M_PI * cfg->frequency_hz / cfg->sound_speed_m_s / 1000.0;
	double cos_theta = transducer_normal_sign(idx) * dy / r_mm;
	if(cos_theta < 0.0) cos_theta = 0.0;
	double directivity = cos_theta;
	double arg = k * cfg->transducer_radius_mm * sqrt(fmax(0.0, 1.0 - cos_theta*cos_theta));
	if(fabs(arg) > 1e-6) {
		double j1_approx = 0.5*arg - arg*arg*arg/16.0 + arg*arg*arg*arg*arg/384.0;
		if(fabs(arg) < 1.2) directivity *= 2.0 * j1_approx / arg;
	}
	return cscale(cexp_i(k * r_mm), directivity / r_mm);
}

void new_solver_default_config(NewSolverConfig *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
	cfg->board_distance_mm = 135.0;
	cfg->frequency_hz = 40000.0;
	cfg->sound_speed_m_s = 343.0;
	cfg->transducer_radius_mm = 5.0;
	cfg->lobe_offset_mm = 2.3;
	cfg->cage_radius_mm = 2.6;
	cfg->regularization = 0.04;
	cfg->iterations = 32;
	cfg->method = NEW_TRAP_CURVATURE_BOOST;
}

const char *new_solver_method_name(NewTrapMethod method)
{
	if(method == NEW_TRAP_CURVATURE_BOOST) return "curvature-boost";
	return method == NEW_TRAP_SHADOW_NULLSPACE ? "shadow-nullspace" : "replicate-wgs";
}

static int add_replicate_controls(const NewSolverConfig *cfg, const NewVec3 *traps, int n, ControlPoint *cp)
{
	int m = 0;
	for(int i = 0; i < n; i++) {
		double d = cfg->lobe_offset_mm;
		cp[m++] = (ControlPoint){ {traps[i].x - d, traps[i].y, traps[i].z}, cx(1, 0), 1.0, 0 };
		cp[m++] = (ControlPoint){ {traps[i].x + d, traps[i].y, traps[i].z}, cx(-1, 0), 1.0, 0 };
		cp[m++] = (ControlPoint){ traps[i], cx(0, 0), 0.45, 1 };
	}
	return m;
}

static int add_shadow_controls(const NewSolverConfig *cfg, const NewVec3 *traps, int n, ControlPoint *cp)
{
	static const NewVec3 dirs[6] = {
		{1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}
	};
	int m = 0;
	for(int i = 0; i < n; i++) {
		cp[m++] = (ControlPoint){ traps[i], cx(0, 0), 2.2, 1 };
		for(int d = 0; d < 6; d++) {
			double a = 2.0 * M_PI * (double)d / 6.0 + 0.73 * (double)i;
			NewVec3 p = {
				traps[i].x + dirs[d].x * cfg->cage_radius_mm,
				traps[i].y + dirs[d].y * cfg->cage_radius_mm,
				traps[i].z + dirs[d].z * cfg->cage_radius_mm
			};
			cp[m++] = (ControlPoint){ p, cexp_i(a), 0.85, 0 };
		}
	}
	return m;
}

static int add_boost_controls(const NewSolverConfig *cfg, const NewVec3 *traps, int n, ControlPoint *cp)
{
	static const NewVec3 dirs[10] = {
		{1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1},
		{1,0,1}, {-1,0,1}, {1,0,-1}, {-1,0,-1}
	};
	int m = 0;
	(void)cfg;
	double radius = 1.6;
	for(int i = 0; i < n; i++) {
		cp[m++] = (ControlPoint){ traps[i], cx(0, 0), 3.0, 1 };
		for(int d = 0; d < 10; d++) {
			double norm = sqrt(dirs[d].x*dirs[d].x + dirs[d].y*dirs[d].y + dirs[d].z*dirs[d].z);
			double scale = radius / (norm > 0.0 ? norm : 1.0);
			double a = M_PI * (double)d / 3.0 + 0.73 * (double)i;
			double w = dirs[d].y != 0.0 ? 1.4 : 1.0;
			NewVec3 p = {
				traps[i].x + dirs[d].x * scale,
				traps[i].y + dirs[d].y * scale,
				traps[i].z + dirs[d].z * scale
			};
			cp[m++] = (ControlPoint){ p, cexp_i(a), w, 0 };
		}
	}
	return m;
}

static void phase_only(Cx *s)
{
	for(int j = 0; j < NEW_TRANSDUCERS; j++) {
		double a = atan2(s[j].im, s[j].re);
		s[j] = cexp_i(a);
	}
}

static void project_null_pass(const NewSolverConfig *cfg, const ControlPoint *cp, int m, Cx *s)
{
	for(int q = 0; q < m; q++) {
		if(!cp[q].null_point) continue;
		Cx p = cx(0, 0);
		double norm = cfg->regularization;
		for(int j = 0; j < NEW_TRANSDUCERS; j++) {
			Cx a = propagator(cfg, j, cp[q].p);
			p = cadd(p, cmul(a, s[j]));
			norm += cabs2(a);
		}
		Cx corr = cscale(p, 1.0 / norm);
		for(int j = 0; j < NEW_TRANSDUCERS; j++) {
			Cx a = propagator(cfg, j, cp[q].p);
			s[j] = csub(s[j], cscale(cmul(cconj(a), corr), cp[q].weight));
		}
	}
}

void new_solver_make_phases(const NewSolverConfig *cfg,
	const NewVec3 *traps, int trap_count, uint16_t phases[NEW_TRANSDUCERS])
{
	ControlPoint cp[NEW_MAX_TRAPS * 16];
	Cx s[NEW_TRANSDUCERS];
	int n = trap_count;
	if(n < 1) n = 1;
	if(n > NEW_MAX_TRAPS) n = NEW_MAX_TRAPS;
	int m = cfg->method == NEW_TRAP_CURVATURE_BOOST ? add_boost_controls(cfg, traps, n, cp) :
		(cfg->method == NEW_TRAP_SHADOW_NULLSPACE ? add_shadow_controls(cfg, traps, n, cp) :
		add_replicate_controls(cfg, traps, n, cp));

	for(int j = 0; j < NEW_TRANSDUCERS; j++) s[j] = cx(1, 0);
	int iters = cfg->method == NEW_TRAP_REPLICATE_WGS ? cfg->iterations : 1;
	if(iters < 1) iters = 1;
	for(int it = 0; it < iters; it++) {
		for(int j = 0; j < NEW_TRANSDUCERS; j++) s[j] = cx(0, 0);
		for(int q = 0; q < m; q++) {
			if(cp[q].null_point) continue;
			for(int j = 0; j < NEW_TRANSDUCERS; j++) {
				Cx a = propagator(cfg, j, cp[q].p);
				s[j] = cadd(s[j], cscale(cmul(cconj(a), cp[q].target), cp[q].weight));
			}
		}
		if(cfg->method == NEW_TRAP_SHADOW_NULLSPACE || cfg->method == NEW_TRAP_CURVATURE_BOOST) {
			int passes = cfg->method == NEW_TRAP_CURVATURE_BOOST ? 6 : 3;
			NewSolverConfig local = *cfg;
			if(cfg->method == NEW_TRAP_CURVATURE_BOOST) {
				local.regularization = 0.02;
			}
			for(int pass = 0; pass < passes; pass++) project_null_pass(&local, cp, m, s);
		}
		phase_only(s);
	}

	for(int j = 0; j < NEW_TRANSDUCERS; j++) {
		double phase = atan2(s[j].im, s[j].re);
		double ticks = -phase * (double)NEW_PHASE_MAX / (2.0 * M_PI);
		int v = (int)llround(ticks) % NEW_PHASE_MAX;
		if(v < 0) v += NEW_PHASE_MAX;
		phases[j] = (uint16_t)v;
	}
}

static Cx field_pressure(const NewSolverConfig *cfg, const uint16_t phases[NEW_TRANSDUCERS], NewVec3 p)
{
	Cx out = cx(0, 0);
	for(int j = 0; j < NEW_TRANSDUCERS; j++) {
		double phase = -2.0 * M_PI * (double)phases[j] / (double)NEW_PHASE_MAX;
		out = cadd(out, cmul(propagator(cfg, j, p), cexp_i(phase)));
	}
	return out;
}

void new_solver_eval_field(const NewSolverConfig *cfg,
	const uint16_t phases[NEW_TRANSDUCERS], NewVec3 p, NewFieldSample *sample)
{
	const double h = 0.35;
	Cx p0 = field_pressure(cfg, phases, p);
	NewVec3 pxp = {p.x+h,p.y,p.z}, pxm = {p.x-h,p.y,p.z};
	NewVec3 pyp = {p.x,p.y+h,p.z}, pym = {p.x,p.y-h,p.z};
	NewVec3 pzp = {p.x,p.y,p.z+h}, pzm = {p.x,p.y,p.z-h};
	Cx gx = cscale(csub(field_pressure(cfg, phases, pxp), field_pressure(cfg, phases, pxm)), 1.0/(2.0*h));
	Cx gy = cscale(csub(field_pressure(cfg, phases, pyp), field_pressure(cfg, phases, pym)), 1.0/(2.0*h));
	Cx gz = cscale(csub(field_pressure(cfg, phases, pzp), field_pressure(cfg, phases, pzm)), 1.0/(2.0*h));
	sample->pressure2 = cabs2(p0);
	sample->grad2 = cabs2(gx) + cabs2(gy) + cabs2(gz);
	sample->gorkov_like = sample->pressure2 - 0.58 * sample->grad2;
}

double new_solver_trap_score(const NewSolverConfig *cfg,
	const uint16_t phases[NEW_TRANSDUCERS], NewVec3 trap, double step_mm)
{
	NewFieldSample c, xp, xm, yp, ym, zp, zm;
	NewVec3 a = trap;
	new_solver_eval_field(cfg, phases, a, &c);
	a.x = trap.x + step_mm; new_solver_eval_field(cfg, phases, a, &xp);
	a.x = trap.x - step_mm; new_solver_eval_field(cfg, phases, a, &xm);
	a = trap; a.y = trap.y + step_mm; new_solver_eval_field(cfg, phases, a, &yp);
	a.y = trap.y - step_mm; new_solver_eval_field(cfg, phases, a, &ym);
	a = trap; a.z = trap.z + step_mm; new_solver_eval_field(cfg, phases, a, &zp);
	a.z = trap.z - step_mm; new_solver_eval_field(cfg, phases, a, &zm);
	return (xp.gorkov_like + xm.gorkov_like + yp.gorkov_like + ym.gorkov_like +
		zp.gorkov_like + zm.gorkov_like - 6.0 * c.gorkov_like) / (step_mm * step_mm);
}
