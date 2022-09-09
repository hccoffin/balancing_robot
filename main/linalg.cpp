#include <Arduino.h>
#include "linalg.h"

vec prod(mat m, vec v) {
	vec val;
	val.x1 = m.x11*v.x1 + m.x12*v.x2;
	val.x2 = m.x21*v.x1 + m.x22*v.x2;
	return val;
}
vec prod(float s, vec v) {
	vec val;
	val.x1 = v.x1*s;
	val.x2 = v.x2*s;
	return val;
}
mat prod(mat m1, mat m2) {
	mat val;
	val.x11 = m1.x11*m2.x11 + m1.x12*m2.x21;
	val.x21 = m1.x21*m2.x11 + m1.x22*m2.x21;
	val.x12 = m1.x11*m2.x12 + m1.x12*m2.x22;
	val.x22 = m1.x21*m2.x12 + m1.x22*m2.x22;
	return val;
}
mat prod(float s, mat m) {
	mat val;
	val.x11 = m.x11*s;
	val.x12 = m.x12*s;
	val.x21 = m.x21*s;
	val.x22 = m.x22*s;
	return val;
}

mat sum(mat m1, mat m2) {
	mat val;
	val.x11 = m1.x11 + m2.x11;
	val.x12 = m1.x12 + m2.x12;
	val.x21 = m1.x21 + m2.x21;
	val.x22 = m1.x22 + m2.x22;
	return val;
}
vec sum(vec v1, vec v2) {
	vec val;
	val.x1 = v1.x1 + v2.x1;
	val.x1 = v2.x2 + v2.x2;
	return val;
}

mat trans(mat m) {
	mat val;
	val.x11 = m.x11;
	val.x21 = m.x12;
	val.x12 = m.x21;
	val.x22 = m.x22;
	return val;
}
mat inv(mat m) {
	float det = m.x11*m.x22 - m.x12*m.x21;
	if (abs(det) < 1e-5) {
		mat val = { };
		return val;
	} else {
		mat val;
		float one_over = 1 / det;
		val.x11 = one_over*m.x22;
		val.x12 = -one_over*m.x12;
		val.x21 = -one_over*m.x21;
		val.x22 = one_over*m.x11;
		return val;
	}
}
