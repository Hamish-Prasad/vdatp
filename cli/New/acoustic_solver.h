#ifndef NEW_ACOUSTIC_SOLVER_H
#define NEW_ACOUSTIC_SOLVER_H

#include <stdint.h>

#define NEW_NUM_BOARDS 4
#define NEW_CHANNELS_PER_BOARD 50
#define NEW_TRANSDUCERS 200
#define NEW_MAX_TRAPS 8
#define NEW_PHASE_MAX 512

typedef struct NewVec3 {
	double x;
	double y;
	double z;
} NewVec3;

typedef enum NewTrapMethod {
	NEW_TRAP_REPLICATE_WGS = 0,
	NEW_TRAP_SHADOW_NULLSPACE = 1,
	NEW_TRAP_CURVATURE_BOOST = 2
} NewTrapMethod;

typedef struct NewSolverConfig {
	double board_distance_mm;
	double frequency_hz;
	double sound_speed_m_s;
	double transducer_radius_mm;
	double lobe_offset_mm;
	double cage_radius_mm;
	double regularization;
	int iterations;
	NewTrapMethod method;
} NewSolverConfig;

typedef struct NewFieldSample {
	double pressure2;
	double grad2;
	double gorkov_like;
} NewFieldSample;

void new_solver_default_config(NewSolverConfig *cfg);
void new_solver_make_phases(const NewSolverConfig *cfg,
	const NewVec3 *traps, int trap_count, uint16_t phases[NEW_TRANSDUCERS]);
void new_solver_eval_field(const NewSolverConfig *cfg,
	const uint16_t phases[NEW_TRANSDUCERS], NewVec3 p, NewFieldSample *sample);
double new_solver_trap_score(const NewSolverConfig *cfg,
	const uint16_t phases[NEW_TRANSDUCERS], NewVec3 trap, double step_mm);
const char *new_solver_method_name(NewTrapMethod method);

#endif
