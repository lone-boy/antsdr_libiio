//
// Created by jcc on 23-10-20.
//
#include "zcProcess.h"
#include "math_utils.h"

zcProcess::zcProcess(int root, float fs, int len) {
    fs_ = fs;
    root_ = root;
    fft_bins_ = get_fft_size(fs_);
    len_ = len;
    seq_ = new CF32[len_];
    xn_ = new CF32[fft_bins_];
    create_zc(root_, len_);
}

void zcProcess::create_zc(int root, int seq_length) {
    for (uint32_t i = 0; i < seq_length; i++) {
        float real_part = std::cos(-M_PI * (float)root * (float)i * (float)(i + 1) / (float)seq_length);
        float imag_part = std::sin(-M_PI * (float)root * (float)i * (float)(i + 1) / (float)seq_length);
        seq_[i] = CF32(real_part, imag_part);
    }
    seq_[len_ / 2] = 0;
    ofdm_modulation();
}

void zcProcess::ofdm_modulation() {
    CF32* xk = (CF32*)malloc(sizeof(CF32) * fft_bins_);
    memset(xk, 0, sizeof(CF32) * fft_bins_);
    int off = (fft_bins_ - len_) / 2;
    int j = 0;
    for (int i = off; i < off + NCARRIERS; i++, j++) {
        xk[i] = seq_[j];
    }

    /* fftshift */
    CF32* temp = (CF32*)malloc(sizeof(CF32) * fft_bins_ / 2);
    for (int i = 0; i < fft_bins_ / 2; i++) {
        temp[i] = xk[i];
        xk[i] = xk[fft_bins_ / 2 + i];
        xk[fft_bins_ / 2 + i] = temp[i];
    }
    /* ifft */
    ifft(xk, xn_, (int)fft_bins_);
}

zcProcess::~zcProcess() {

}



