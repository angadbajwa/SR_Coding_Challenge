#pragma once

//custom libs
#include "atc/atc.h"

float get_dist(Sector s1, Sector s2) {
	return sqrtf(powf(s1.x - s2.x, 2) + powf(s1.y - s2.y, 2));
}
