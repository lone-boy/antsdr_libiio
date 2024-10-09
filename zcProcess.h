//
// Created by jcc on 23-10-20.
//

#ifndef DRONE_MP_ZCPROCESS_H
#define DRONE_MP_ZCPROCESS_H

#include "common.h"

class zcProcess {
public:
    zcProcess(int root, float fs, int len);
    ~zcProcess();


    CF32* get_zc_xn() {
        return xn_;
    };

    CF32* get_zc_seq() {
        return seq_;
    }

    int get_len() {
        return fft_bins_;
    }
private:
    float fs_;
    int root_;
    CF32* seq_;
    CF32* xn_;
    int fft_bins_;
    int len_;

private:
    void create_zc(int root, int seq_length);
    void ofdm_modulation();
};

#endif //DRONE_MP_ZCPROCESS_H
