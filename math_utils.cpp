//
// Created by jcc on 23-10-20.
//
#include "math_utils.h"
#include "fftw3.h"
#include "chrono"
#define _USE_MATH_DEFINES
#include <cmath>
#include "mutex"

std::mutex mtx;


int get_short_cp_len(float sample_rate){
    return (int)(0.0000046875f * sample_rate);
}

int get_long_cp_len(float sample_rate){
    return (int)(1.0f / 192000.0f * sample_rate);
}

int get_fft_size(float sample_rate){
    return (int)(sample_rate / 15e3);
}

void ifft(CF32 *in,CF32 *out,int fft_size){
    fftwf_complex* in_fft = new fftwf_complex[fft_size];
    fftwf_complex* out_fft = new fftwf_complex[fft_size];

    for(int i=0;i<fft_size;i++){
        in_fft[i][0] = in[i].real();
        in_fft[i][1] = in[i].imag();
    }

    fftwf_plan p = fftwf_plan_dft_1d((int)fft_size,in_fft,out_fft,FFTW_BACKWARD,FFTW_ESTIMATE);
    fftwf_execute(p);
    fftwf_destroy_plan(p);
    for(int i=0;i<fft_size;i++){
        out[i].real(out_fft[i][0] / (float)fft_size);
        out[i].imag(out_fft[i][1] / (float)fft_size);
    }

    fftwf_free(in_fft);
    fftwf_free(out_fft);
}

int cross_correlation_to_find_peak_by_fft(CF32 *input, int input_len, CF32 *xn_fft, int fft_bin, float threshold,double *show) {
    auto startTime = std::chrono::high_resolution_clock::now();
    CF32 *fft_result = new CF32[16];
    CF32* filtered_signal_freq_result = new CF32[16];
    CF32* re = new CF32[16];
    auto *data_show = new float[16];

    for(int i=0;i<input_len - fft_bin;i = i+16){
        fft_just(input + i,fft_result,16);
        for (int j = 0; j < 16; j++) {
            filtered_signal_freq_result[j] = fft_result[j] * std::conj(xn_fft[j]);
        }
        fft_just(filtered_signal_freq_result,re,16);
        float total = 0.0f;
        for(int j=0;j<16;j++){
            total += std::abs(re[j]);
        }
        total /= 16;
        for(int j=0;j<16;j++){
            data_show[j] = std::abs(re[j]) / total;
            show[i+j] = data_show[j];
        }
//        for(int j=0;j<16;j++){
//            int index_max = j;
//            if(data_show[j] > threshold){
//                float max_value = data_show[j];
//                for(int true_index = j+1;true_index<16;true_index++){
//                    if(data_show[true_index] > max_value){
//                        max_value = data_show[true_index];
//                        index_max = true_index;
//                    }
//                }
//                delete[] fft_result;
//                delete[] filtered_signal_freq_result;
//                delete[] re;
//                delete[] data_show;
//                auto endTime = std::chrono::high_resolution_clock::now();
//                auto duration = (float)std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
////                printf("This function  %s spend %lf us\n",__FUNCTION__,duration);
//                return i + index_max;
//            }
//        }
    }
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = (float)std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
//    printf("This function  %s spend %lf us\n",__FUNCTION__,duration);

    delete[] fft_result;
    delete[] filtered_signal_freq_result;
    delete[] re;
    delete[] data_show;

    return -1;
}

void cfo_correct(CF32 *in,int in_len,double cfo,float fs){
    for(int i=0;i<in_len;i++){
        float phase = -2.0f * M_PI * cfo * (float)i * (1.0f / fs);
        CF32 exp_phase = CF32 (std::cos(phase),std::sin(phase));
        in[i] = in[i] * exp_phase;
    }
}

void fft_just(CF32* signal, CF32* result, int len_signal) {
    fftwf_complex* in_fft = (fftwf_complex*)fftwf_malloc(len_signal * sizeof(fftwf_complex));
    fftwf_complex* out_fft = (fftwf_complex*)fftwf_malloc(len_signal * sizeof(fftwf_complex));
    for (int i = 0; i < len_signal; i++) {
        in_fft[i][0] = signal[i].real();
        in_fft[i][1] = signal[i].imag();
    }
    fftwf_plan p = fftwf_plan_dft_1d((int)len_signal, in_fft, out_fft, FFTW_FORWARD, FFTW_ESTIMATE);
    fftwf_execute(p);
    fftwf_destroy_plan(p);
    for (int i = 0; i < len_signal; i++) {
        result[i].real(out_fft[i][0]);
        result[i].imag(out_fft[i][1]);
        result[i] = std::conj(result[i]);
    }
    fftwf_free(in_fft);
    fftwf_free(out_fft);
}

void fft_and_shift(CF32* signal,CF32* result,int len_signal){
    mtx.lock();
    fftwf_complex* in_fft = (fftwf_complex*)fftwf_malloc(len_signal*sizeof(fftwf_complex));
    fftwf_complex* out_fft = (fftwf_complex*)fftwf_malloc(len_signal*sizeof(fftwf_complex));
    fftwf_plan p = fftwf_plan_dft_1d(len_signal, in_fft, out_fft, FFTW_FORWARD, FFTW_ESTIMATE);

    for(int i=0;i<len_signal;i++){
        in_fft[i][0] = signal[i].real() ;
        in_fft[i][1] = signal[i].imag() ;
    }
    fftwf_execute(p);
    fftwf_destroy_plan(p);
    for(int i=0;i<len_signal / 2;i++){

        result[i].real(out_fft[len_signal / 2 + i][0]);
        result[i].imag(out_fft[len_signal / 2 + i][1]);
    }
    int j=0;
    for(int i=len_signal / 2;i < len_signal;i++,j++){
        result[i].real(out_fft[j][0]);
        result[i].imag(out_fft[j][1]);
    }
    fftwf_free(in_fft);
    fftwf_free(out_fft);
    mtx.unlock();
}

void fft_just_no(CF32* signal, CF32* result, int len_signal) {
    mtx.lock();
    fftwf_complex* in_fft = (fftwf_complex*)fftwf_malloc(len_signal * sizeof(fftwf_complex));
    fftwf_complex* out_fft = (fftwf_complex*)fftwf_malloc(len_signal * sizeof(fftwf_complex));
    for (int i = 0; i < len_signal; i++) {
        in_fft[i][0] = signal[i].real();
        in_fft[i][1] = signal[i].imag();
    }
    fftwf_plan p = fftwf_plan_dft_1d((int)len_signal, in_fft, out_fft, FFTW_FORWARD, FFTW_ESTIMATE);
    fftwf_execute(p);
    fftwf_destroy_plan(p);
    for (int i = 0; i < len_signal; i++) {
        result[i].real(out_fft[i][0]);
        result[i].imag(out_fft[i][1]);
    }
    fftwf_free(in_fft);
    fftwf_free(out_fft);
    mtx.unlock();
}

int find_video_by_fft(CF32* input, CF32* filter_signal_freq, int intput_len) {
    CF32* fft_result = new CF32[4096];
    CF32* filtered_signal_freq_result = new CF32[4096];
    CF32* re = new CF32[4096];

    for (int i = 0; i < intput_len - 4096; i = i + 4096) {
        fft_just_no(input + i, fft_result, 4096);
        for (int j = 0; j < 4096; j++) {
            filtered_signal_freq_result[j] = fft_result[j] * filter_signal_freq[j];
        }
        fft_just_no(filtered_signal_freq_result, re, 4096);
        float total = 0.0f;
        for (int i = 0; i < 4096; i++) {
            total += std::abs(re[i]);
        }
        total /= 4096;

        for (int index = 0; index < 4096; index++) {
            if (std::abs(re[index]) / total > 10) {
                delete[] fft_result;
                delete[] filtered_signal_freq_result;
                delete[] re;
                return i;
            }
        }
    }

    delete[] fft_result;
    delete[] filtered_signal_freq_result;
    delete[] re;
    return -1;
}

void smooth_signal(float* in, int len, int windows_size) {
    if (windows_size < 1) {
        windows_size = 1;
    }

    for (int i = 0; i < len - windows_size; i++) {
        float sum = 0.0f;
        for (int j = i; j < i + windows_size; j++) {
            sum += in[j];
        }
        sum /= windows_size;
        in[i] = sum;
    }
}

void smooth_complex_signal(CF32 *in, int len,int windows_size){
    if(windows_size < 1){
        windows_size = 1;
    }

    for(int i=0;i<len - windows_size;i++){
        CF32 sum = {0,0};
        for(int j=i;j<i+windows_size;j++){
            sum += in[j];
        }
        sum /= windows_size;
        in[i] = sum;
    }

}

float find_signal_center(float* db_spectrum, float average_signal, int samples, float fs) {
    float start_index = 0, end_index = 0;
    int lock = 0;
    float fs_M = fs / 1e6;
    float start_M = -fs_M / 2;
    float pices = fs_M / samples;

    float center = 0.0f;
    float span;

    for (int i = 0; i < samples; i++) {
        if (db_spectrum[i] > average_signal && lock == 0) {
            start_index = start_M + pices * i;
            lock = 1;
        }
        if (lock == 1 && db_spectrum[i] < average_signal) {
            lock = 0;
            end_index = start_M + pices * i;
        }

        if (lock == 0) {
            span = end_index - start_index;
            if (span > 16 && span < 22) {
                center = (start_index + end_index) / 2;
                return center;
            }
        }
    }
    return -fs_M;
}

void apply_fir(CF32 *in, const float *taps, int samples_len, int taps_len) {
    CF32 *output_signal = new CF32[samples_len];
    for (int n = 0; n < samples_len; n++) {
        output_signal[n].real(0.0);
        output_signal[n].imag(0.0);
        for (int k = 0; k < taps_len; k++) {
            if (n >= k) {
                output_signal[n].real(output_signal[n].real() + in[n - k].real() * taps[k]);
                output_signal[n].imag(output_signal[n].imag() + in[n - k].imag() * taps[k]);
            }
        }
    }
    for (int n = 0; n < samples_len; n++) {
        in[n] = output_signal[n];
    }

    delete[] output_signal;
}

void downsamplerate_data(CF32 *in, int len, CF32 *out, int down_scale) {
    for(int i=0;i<len / down_scale;i++){
        out[i] = in[i*down_scale];
    }
}

double getRad(double d) {
    return d * M_PI / 180.0;
}

double get_distance_by_position(double lata, double lona, double latb, double lonb) {
    if(lata == 0 || lona == 0 || latb == 0 || lonb == 0)
        return 0.0;
    double f = getRad((lata + latb) / 2);
    double g = getRad((lata - latb) / 2);
    double l = getRad((lona - lonb) / 2);
    double sg = sin(g);
    double sl = sin(l);
    double sf = sin(f);
    double s, c, w, r, d, h1, h2;
    double a = 6378137.0; // 地球半径，单位：米
    double fl = 1 / 298.257;

    sg = sg * sg;
    sl = sl * sl;
    sf = sf * sf;
    s = sg * (1 - sl) + (1 - sf) * sl;
    c = (1 - sg) * (1 - sl) + sf * sl;
    w = atan(sqrt(s / c));
    r = sqrt(s * c) / w;
    d = 2 * w * a;
    h1 = (3 * r - 1) / (2 * c);
    h2 = (3 * r + 1) / (2 * s);
    s = d * (1 + fl * (h1 * sf * (1 - sg) - h2 * (1 - sf) * sg));
    return s;
}

double get_angle_by_position(double lata, double lona, double latb, double lonb) {
    if(lata == 0 || lona == 0 || latb == 0 || lonb == 0)
        return 300;

    double dpi = 0.017453292519943295;
    double yy = sin((lonb - lona) * dpi) * cos(latb * dpi);
    double xx = cos(lata * dpi) * sin(latb * dpi) - sin(lata * dpi) * cos(latb * dpi) * cos((lonb - lona) * dpi);

    double angle = atan2(yy, xx) / dpi;
    return angle;
}

std::string convertGPSTimeToDateTime(uint64_t gps_time) {
    auto tick = (time_t)(gps_time / 1000);
    struct tm tm{};
    char s[40];
    tm = *gmtime(&tick); // 转换 Convert time_t to tm as UTC time
    tm.tm_hour = tm.tm_hour+8; // 转换为北京时间 Beijing (China)
    strftime(s, sizeof(s), "%Y-%m-%d %H:%M:%S", &tm);
    std::string str(s);
    return str;
}

float caluate_fft_abs_spectrum(CF32* fft_spectrum, float* abs_spectrum, uint32_t samples,double gain) {
    float average = 0.0f;
    for (int i = 0; i < samples; i++) {
        float amplitude = (std::abs(fft_spectrum[i]) / samples);
        abs_spectrum[i] = 10.0f * log10(amplitude * amplitude) - 57 -gain;
        average += abs_spectrum[i];
    }
    average /= samples;
    return average;
}


int find_evo_video_fft(CF32* input,CF32* evo_cof,int evo_cof_length,int input_len){
//    FILE *wr = fopen("./evo.txt","wb");
    for(int sample_index = 0;sample_index < input_len - 4096;sample_index += 8192){
        CF32 fft_data[4096];
        fft_and_shift(input+sample_index,fft_data,4096);
        float resultcos[4096 - 108];
        for(int i=0;i<4096-evo_cof_length;i++){
            float cos_d = 0.0f;
            for(int j=0;j<108;j++){
                cos_d += std::abs(evo_cof[j] * fft_data[j]);
            }
            float seq_norm = 0.0f;
            float fft_norm = 0.0f;
            for(int index_cof = 0;index_cof < evo_cof_length;index_cof++){
                seq_norm += std::norm(evo_cof[index_cof]);
                fft_norm += std::norm(fft_data[index_cof+i]);
            }
            seq_norm = std::sqrt(seq_norm);
            fft_norm = std::sqrt(fft_norm);

            cos_d = std::abs(cos_d) / seq_norm / fft_norm;
            if(cos_d > 0.7){
                printf("get evo\n");
            }
            resultcos[i] = cos_d;
        }
//        fwrite(resultcos,sizeof(float ) * (4096-108),1,wr);
//        fclose(wr);
//        return 0;
    }
}


int cross_correlation_to_find_peak(CF32 *input,int input_len,CF32 *xn,int fft_bin,double* show){
    CF32* signal = input;
    CF32* zc_taps_600 = xn;
    CF32* cross_acf_600 = new CF32[input_len];

    CF32 sum[input_len];
    for(int i=0;i<input_len;i++){
        for(int j=0;j<fft_bin;j++){
            sum[i] += zc_taps_600[j] * std::conj(signal[i+j]);
        }
        show[i] = (std::abs(sum[i]));
    }
    return 1;
}