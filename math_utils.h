//
// Created by jcc on 23-10-20.
//

#ifndef DRONE_MP_MATH_UTILS_H
#define DRONE_MP_MATH_UTILS_H
#include "common.h"

int get_short_cp_len(float sample_rate);
int get_long_cp_len(float sample_rate);
int get_fft_size(float fs);
void ifft(CF32* in, CF32* out, int fft_size);


int cross_correlation_to_find_peak_by_fft(CF32 *input, int input_len, CF32 *xn_fft, int fft_bin, float threshold,double *show);
int cross_correlation_to_find_peak(CF32 *input,int input_len,CF32 *xn,int fft_bin,double* show);


void cfo_correct(CF32* in, int in_len, double cfo, float fs);
void fft_and_shift(CF32* signal, CF32* result, int len_signal);
void smooth_complex_signal(CF32* in, int len, int windows_size);
void apply_fir(CF32* in, const float* taps, int samples_len, int taps_len);
void downsamplerate_data(CF32* in, int len, CF32* out, int down_scale);
float caluate_fft_abs_spectrum(CF32* fft_spectrum, float* abs_spectrum, uint32_t samples, double gain);



double getRad(double d);
double get_distance_by_position(double lata, double lona, double latb, double lonb);
double get_angle_by_position(double lata, double lona, double latb, double lonb);
std::string convertGPSTimeToDateTime(uint64_t gps_time);



void fft_just(CF32* signal, CF32* result, int len_signal);
void fft_just_no(CF32* signal, CF32* result, int len_signal);
int find_video_by_fft(CF32* input, CF32* filter_signal_freq, int intput_len);
void smooth_signal(float* in, int len, int windows_size);
float find_signal_center(float* db_spectrum, float average_signal, int samples, float fs);

int find_evo_video_fft(CF32* input,CF32* evo_cof,int evo_cof_length,int input_len);
#endif //DRONE_MP_MATH_UTILS_H
