// Stanley Bak
// 4-2014
// Geometry header for real-time reach

#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <stdbool.h>
#include "main.h"
#include "dynamics_bicycle.h"

#define NUM_FACES (2 * NUM_DIMS)

typedef struct Interval
{
	REAL min;
	REAL max;
} Interval;

// for a HyperPoint use a REAL[]
typedef struct HyperPoint
{
	REAL dims[NUM_DIMS];
} HyperPoint;

// for a HyperRectangle use an Interval[]
typedef struct HyperRectangle
{
	Interval dims[NUM_DIMS];
} HyperRectangle;

REAL interval_width(Interval* i);

bool hyperrectangle_contains(HyperRectangle* outside, HyperRectangle* inside, bool printErrors);
void hyperrectangle_grow_to_convex_hull(HyperRectangle* grower, HyperRectangle* contained);
REAL hyperrectange_max_width(HyperRectangle* rect);
void hyperrectangle_bloat(HyperRectangle* out, REAL from[NUM_DIMS], REAL width);

void print(HyperRectangle* r);
void println(HyperRectangle* r);

#endif
