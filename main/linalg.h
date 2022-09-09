#include <Arduino.h>

struct mat {
	float x11;
	float x12;
	float x21;
	float x22;
};

struct vec {
	float x1;
	float x2;
};

void print(mat m);
void print(vec v);

vec prod(mat m, vec v);
vec prod(float s, vec v);
mat prod(mat m1, mat m2);
mat prod(float s, mat m);

mat sum(mat m1, mat m2);
vec sum(vec v1, vec v2);

mat trans(mat m);
mat inv(mat m);
