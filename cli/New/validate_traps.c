#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "acoustic_solver.h"

static void write_slice(const char *path, const NewSolverConfig *cfg,
	const uint16_t phases[NEW_TRANSDUCERS], double y_mm)
{
	FILE *f = fopen(path, "w");
	if(!f) return;
	int w = 181, h = 181;
	fprintf(f, "P3\n%d %d\n255\n", w, h);
	double vals[181][181], mn = 1e99, mx = -1e99;
	for(int iy = 0; iy < h; iy++) {
		for(int ix = 0; ix < w; ix++) {
			NewVec3 p = {-45.0 + 90.0 * ix / (w - 1), y_mm, -45.0 + 90.0 * iy / (h - 1)};
			NewFieldSample s;
			new_solver_eval_field(cfg, phases, p, &s);
			vals[iy][ix] = s.gorkov_like;
			if(vals[iy][ix] < mn) mn = vals[iy][ix];
			if(vals[iy][ix] > mx) mx = vals[iy][ix];
		}
	}
	for(int iy = 0; iy < h; iy++) {
		for(int ix = 0; ix < w; ix++) {
			double t = (vals[iy][ix] - mn) / (mx - mn + 1e-30);
			int r = (int)(255.0 * t);
			int b = (int)(255.0 * (1.0 - t));
			int g = (int)(120.0 * (1.0 - fabs(2.0*t - 1.0)));
			fprintf(f, "%d %d %d ", r, g, b);
		}
		fprintf(f, "\n");
	}
	fclose(f);
}

static void write_csv(const char *path, const NewSolverConfig *cfg,
	const uint16_t phases[NEW_TRANSDUCERS], const NewVec3 *traps, int trap_count)
{
	FILE *f = fopen(path, "w");
	if(!f) return;
	fprintf(f, "method,trap,x_mm,y_mm,z_mm,pressure2,grad2,gorkov_like,curvature_score\n");
	for(int i = 0; i < trap_count; i++) {
		NewFieldSample s;
		new_solver_eval_field(cfg, phases, traps[i], &s);
		fprintf(f, "%s,%d,%.3f,%.3f,%.3f,%.12g,%.12g,%.12g,%.12g\n",
			new_solver_method_name(cfg->method), i, traps[i].x, traps[i].y, traps[i].z,
			s.pressure2, s.grad2, s.gorkov_like,
			new_solver_trap_score(cfg, phases, traps[i], 0.8));
	}
	fclose(f);
}

static void run_case(NewTrapMethod method, const char *tag)
{
	NewSolverConfig cfg;
	new_solver_default_config(&cfg);
	cfg.method = method;
	NewVec3 traps[3] = {{-12,0,0}, {0,0,0}, {12,0,0}};
	uint16_t phases[NEW_TRANSDUCERS];
	new_solver_make_phases(&cfg, traps, 3, phases);
	char csv[128], ppm[128];
	snprintf(csv, sizeof(csv), "%s_metrics.csv", tag);
	snprintf(ppm, sizeof(ppm), "%s_gorkov_xz.ppm", tag);
	write_csv(csv, &cfg, phases, traps, 3);
	write_slice(ppm, &cfg, phases, 0.0);
	printf("%s written: %s %s\n", tag, csv, ppm);
	for(int i = 0; i < 3; i++) {
		printf("  trap %d curvature %.6g\n", i, new_solver_trap_score(&cfg, phases, traps[i], 0.8));
	}
}

int main(void)
{
	run_case(NEW_TRAP_REPLICATE_WGS, "replicate_wgs");
	run_case(NEW_TRAP_SHADOW_NULLSPACE, "shadow_nullspace");
	run_case(NEW_TRAP_CURVATURE_BOOST, "curvature_boost");
	return 0;
}
