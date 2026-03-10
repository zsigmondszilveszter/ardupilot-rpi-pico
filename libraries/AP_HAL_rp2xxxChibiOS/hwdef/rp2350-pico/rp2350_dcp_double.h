#pragma once

/*
 * Minimal header replacing the pico-sdk dependency for the RP2350 DCP-backed
 * double wrappers vendored into this board directory.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#define WRAPPER_FUNC(x) __wrap_ ## x
#define REAL_FUNC(x) __real_ ## x

#define GCC_Like_Pragma _Pragma
#define GCC_Pragma _Pragma

#define PICO_DOUBLE_PROPAGATE_NANS 0
#define HAS_DOUBLE_COPROCESSOR 1

double int2double(int32_t i);
double uint2double(uint32_t i);
double int642double(int64_t i);
double uint642double(uint64_t i);
double fix2double(int32_t m, int e);
double ufix2double(uint32_t m, int e);
double fix642double(int64_t m, int e);
double ufix642double(uint64_t m, int e);

int32_t double2int_z(double f);
int64_t double2int64_z(double f);
int32_t double2uint_z(double f);
int64_t double2uint64_z(double f);
int32_t double2fix_z(double f, int e);
uint32_t double2ufix_z(double f, int e);
int64_t double2fix64_z(double f, int e);
uint64_t double2ufix64_z(double f, int e);

int32_t double2int(double f);
uint32_t double2uint(double f);
int64_t double2int64(double f);
uint64_t double2uint64(double f);
int32_t double2fix(double f, int e);
uint32_t double2ufix(double f, int e);
int64_t double2fix64(double f, int e);
uint64_t double2ufix64(double f, int e);

double exp10(double x);
void sincos(double x, double *sinx, double *cosx);
double powint(double x, int y);
double ddiv_fast(double n, double d);
double sqrt_fast(double f);
double fma_fast(double x, double y, double z);
double mla(double x, double y, double z);
