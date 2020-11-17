/*
 * Copyright 2016 Blender Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* This code implements a matrix optimization based on simulated annealing
 * that minimizes the energy function described in the paper "Blue-noise Dithered Sampling".
 *
 * Dimensionality and Size of the matrix are hardcoded as #defines below.
 * It includes both a easily readable scalar implementation as well as a SSE4.1-optimized code path (which only supports DIM=2 currently).
 *
 * For full speed, compile with: g++ -o cycles_dithering cycles_dithering.cpp -O3 -march=native --std=c++11
 */

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <random>

std::mt19937 rng(time(0));

float *mat;
int size;
float starting_temp, num_iter;

/* Note: SSE code paths are hardcoded for DIM = 2. */
#define DIM 2

/* Note: The random swapping code is only designed for SIZE=2048 or less. */
#define SIZE 128
#define SIZEMASK 0x7f
#define SIZEBITS 7

#define MAT(x, y, d) mat[(((y)<<SIZEBITS)+(x)) + ((d)<<(SIZEBITS*2))]

#if (DIM == 2) && __SSE4_1__
#define USE_SSE_CODE
#endif

/* To speed up the process, only nearby pixels are considered.
 * For pixels that are further away, the weight is nearly zero anyways. */
#define WINDOW 10


#define WRAP(x) ((x) & SIZEMASK)

/* Fairly rough approximation of the exponential function. */
inline float approx_exp(float x)
{
	x = (1.0f + x * (1.0f / 256.0f));
	x *= x;
	x *= x;
	x *= x;
	x *= x;
	x *= x;
	x *= x;
	x *= x;
	x *= x;
	return x;
}

/* Decent approximation of the square root function. */
inline float approx_sqrt(float x)
{
#ifdef __SSE__
	return _mm_cvtss_f32(_mm_mul_ss(_mm_rsqrt_ss(_mm_set_ss(x)), _mm_set_ss(x)));
#else
	return sqrtf(x);
#endif
}


inline float energy_pq(int px, int py, int qx, int qy)
{
	if(px == qx && py == qy) return 0.0f;
	int dx = px-qx;
	int dy = py-qy;
	qx = WRAP(qx);
	qy = WRAP(qy);
	float dist = -(dx*dx+dy*dy)*(1.0f/4.41f);
	float d_p = 0.0f;
	for(int d = 0; d < DIM; d++) {
		float d_c = MAT(px, py, d) - MAT(qx, qy, d);
		d_p += d_c*d_c;
	}
	if(DIM == 2)
		d_p = approx_sqrt(d_p);
	else
		d_p = powf(d_p, DIM * 0.25f);
	return approx_exp(dist - d_p);
}

/* Returns all the energy in that specific pixel, by looping over the window around the pixel. */
float pixel_energy_scalar(int x, int y)
{
	float energy = 0.0f;
	for(int dy = -WINDOW; dy <= WINDOW; dy++) {
		for(int dx = -WINDOW; dx <= WINDOW; dx++) {
			energy += energy_pq(x, y, x+dx, y+dy);
		}
	}
	return energy;
}

#ifdef USE_SSE_CODE
inline __m128 approx_exp_sse(__m128 x)
{
	x = _mm_add_ps(_mm_set1_ps(1.0f), _mm_mul_ps(x, _mm_set1_ps(1.0f / 256.0f)));
	x = _mm_mul_ps(x, x);
	x = _mm_mul_ps(x, x);
	x = _mm_mul_ps(x, x);
	x = _mm_mul_ps(x, x);
	x = _mm_mul_ps(x, x);
	x = _mm_mul_ps(x, x);
	x = _mm_mul_ps(x, x);
	x = _mm_mul_ps(x, x);
	return x;
}

inline __m128 approx_sqrt_sse(__m128 x)
{
	return _mm_mul_ps(_mm_rsqrt_ps(x), x);
}

inline __m128 energy_pq_sse(__m128 *center, float *row, __m128 dx4, __m128 dy4)
{
	__m128 dist = _mm_mul_ps(_mm_add_ps(_mm_mul_ps(dx4, dx4), _mm_mul_ps(dy4, dy4)), _mm_set1_ps(-1.0f / 4.41f));
	__m128 d1 = _mm_sub_ps(center[0], _mm_loadu_ps(row)), d2 = _mm_sub_ps(center[1], _mm_loadu_ps(row + SIZE*SIZE));
	__m128 dist2 = _mm_add_ps(_mm_mul_ps(d1, d1), _mm_mul_ps(d2, d2));
	return approx_exp_sse(_mm_sub_ps(dist, approx_sqrt_sse(dist2)));
}

/* Same as above, but processes 4 pixels at a time. */
float pixel_energy_sse(int x, int y)
{
	__m128 energy = _mm_setzero_ps();
	__m128 center_pixel[2] = {_mm_set1_ps(MAT(x, y, 0)), _mm_set1_ps(MAT(x, y, 1))};
	for(int dy = -WINDOW; dy <= WINDOW; dy++) {
		int wy = WRAP(y+dy);
		__m128 dy4 = _mm_set1_ps(dy);
		for(int dx = -WINDOW; dx <= WINDOW; dx += 4) {
			__m128 dx4 = _mm_add_ps(_mm_set1_ps(dx), _mm_set_ps(3.0f, 2.0f, 1.0f, 0.0f));
			__m128 active = _mm_cmple_ps(dx4, _mm_set1_ps(WINDOW));
			if(dy == 0) active = _mm_and_ps(active, _mm_cmpneq_ps(dx4, _mm_setzero_ps()));
			energy = _mm_add_ps(energy, _mm_blendv_ps(_mm_setzero_ps(), energy_pq_sse(center_pixel, &MAT(x+dx, wy, 0), dx4, dy4), active));
		}
	}
	energy = _mm_hadd_ps(energy, energy);
	return _mm_cvtss_f32(_mm_hadd_ps(energy, energy));
}

inline float pixel_energy(int x, int y)
{
	/* The SSE code doesn't account for wrapping around the x axis. */
	if(x < WINDOW || (x + WINDOW) >= SIZE)
		return pixel_energy_scalar(x, y);
	else
		return pixel_energy_sse(x, y);
}

#else
#define pixel_energy(x, y) pixel_energy_scalar(x, y)
#endif

int main(int argc, char** argv)
{
	if(argc < 4) {
		fprintf(stderr, "Usage: %s <starting_temp> <num_iterations> <output_name> [initial_matrix.dat]\n", argv[0]);
		fprintf(stderr, "<starting_temp>: Temperature parameter for the simulated annealing process. Values around 0.01 seem to give the best results.\n");
		fprintf(stderr, "                 Note that this implementation uses the following temperature profile: <starting_temp> * (1 - progress)^2\n");
		fprintf(stderr, "<num_iterations>: Number of iterations to run.\n");
		fprintf(stderr, "<output_name>: Name without file ending of the output files - the tool will save <output_name>.dat and <output_name>.c\n");
		fprintf(stderr, "[initial_matrix.mat]: Optional existing matrix that will be used to initialize the process - if not given, a random matrix will be used.\n");
		return -1;
	}
	starting_temp = atof(argv[1]);
	num_iter = atof(argv[2]);

	mat = new float[SIZE*SIZE*DIM];

	bool matrix_initialized = false;
	if(argc == 5) {
		FILE *f = fopen(argv[4], "rb");
		if(f) {
			int floats = fread(mat, sizeof(float), SIZE*SIZE*DIM, f);
			if(floats == SIZE*SIZE*DIM)
				matrix_initialized = true;
			fclose(f);
		}
	}
	if(!matrix_initialized) {
		/* Initialize matrix with random values. */
		for(int i = 0; i < SIZE*SIZE*DIM; i++)
			mat[i] = rng() / ((float) 0xffffffff);
	}

	float total_energy = 0.0f;
	for(int y = 0; y < SIZE; y++) {
		for(int x = 0; x < SIZE; x++) {
			total_energy += pixel_energy(x, y);
		}
	}
	printf("Total Energy: %f\n", total_energy);
	fflush(stdout);

	float energy = 0.0;
	for(int i = 0; i < num_iter; i++) {
		int32_t randval = rng();
 		/* Pick pixel pair to swap. */
		int sx = randval & SIZEMASK;
		int sy = (randval >> 12) & SIZEMASK;
		int ox = WRAP(sx + ((randval >> 24) % 5) - 2);
		int oy = WRAP(sy + ((randval >> 28) % 5) - 2);

		/* Subtract energy contributed by the two pixels to be swapped. */
		float d_energy = -pixel_energy(sx, sy) - pixel_energy(ox, oy);
		/* Swap the pixels. */
		for(int d = 0; d < DIM; d++) {
			float temp = MAT(sx, sy, d);
			MAT(sx, sy, d) = MAT(ox, oy, d);
			MAT(ox, oy, d) = temp;
		}
		/* Add energy contributed by the two pixels that were swapped. */
		d_energy += pixel_energy(sx, sy) + pixel_energy(ox, oy);

		/* Accept the swap or not? */
		float i_f = (1.0f - i/((float) num_iter));
		float temperature = i_f*i_f*starting_temp;
		if(d_energy >= 0.0 && (rng() / ((float) 0xffffffff)) >= expf(-d_energy / temperature)) {
			/* Not accepted => Undo swap. */
			for(int d = 0; d < DIM; d++) {
				float temp = MAT(sx, sy, d);
				MAT(sx, sy, d) = MAT(ox, oy, d);
				MAT(ox, oy, d) = temp;
			}
		}
		else energy += 2.0*d_energy;

		if((i % 1000) == 0) {
			printf("%f %d\n", energy, i);
			fflush(stdout);
		}
	}

	/* Calculate final energy */
	total_energy = 0.0f;
	for(int y = 0; y < SIZE; y++) {
		for(int x = 0; x < SIZE; x++) {
			total_energy += pixel_energy(x, y);
		}
	}
	printf("Total Energy: %f\n", total_energy);
	fflush(stdout);

	/* Debug output.
	 * These files can be visualized in Octave/Matlab with "imshow(fftshift(abs(fft2(dlmread("<file>.dat")))) .* 0.01)". */
#if 0
	for(int d = 0; d < DIM; d++) {
		char filename[1024];
		sprintf(filename, "%s_%d**2_dim%d.dat", argv[3], SIZE, d);
		FILE* f = fopen(filename, "w");
		for(int y = 0; y < SIZE; y++) {
			for(int x = 0; x < SIZE; x++) {
				fprintf(f, "%f ", MAT(x, y, d));
			}
			fprintf(f, "\n");
		}
		fclose(f);
	}
#endif
	char filename[1024];
	sprintf(filename, "%s.mat", argv[3]);
	FILE* f = fopen(filename, "wb");
	fwrite(mat, sizeof(float), DIM*SIZE*SIZE, f);
	fclose(f);

	sprintf(filename, "%s.c", argv[3]);
	f = fopen(filename, "w");
	int num = DIM*SIZE*SIZE;
	fprintf(f, "float dither_matrix[%d] = {\n", num );
	int i = 0;
	while(i < num) {
		for(int j = 0; j < 8 && i < num; i++, j++) {
			/* Reorder elements from dimension-major to y-major. */
			int d = i & 0x1;
			int x = (i >> 1) & SIZEMASK;
			int y = (i >> (SIZEBITS+1)) & SIZEMASK;
			fprintf(f, "%ff", MAT(x, y, d));
			if(i+1 < num) fprintf(f, ",");
			if(j+1 < 8) fprintf(f, " ");
		}
		fprintf(f, "\n");
	}
	fprintf(f, "};\n");
	fclose(f);
}
