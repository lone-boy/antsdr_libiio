#include <iostream>
#include "antsdrDevice.h"
#include "unistd.h"
#include "vector"
#include "zcProcess.h"
#include "common.h"
#include "math_utils.h"
#include <algorithm>

#define SAMPLES 1024

std::vector<double> x_data,recv1_i,recv1_q;
std::vector<double> recv2_i,recv2_q;
pthread_mutex_t lock;

CF32 iq_data[SAMPLES];

void get_rx_data(sdr_transfer *trans){
    pthread_mutex_lock(&lock);
    recv1_i.clear();
    recv1_q.clear();
    if(trans->channels == 2){
        recv2_i.clear();
        recv2_q.clear();
    }
    for(int i=0;i<trans->length / trans->channels;i++){
        recv1_i.push_back(trans->data[(2*trans->channels)*i]);
        recv1_q.push_back(trans->data[(2*trans->channels)*i+1]);
        iq_data[i].real(recv1_i[i]);
        iq_data[i].imag(recv1_q[i]);
        if(trans->channels == 2){
            recv2_i.push_back(trans->data[(2*trans->channels)*i+2]);
            recv2_q.push_back(trans->data[(2*trans->channels)*i+3]);
        }

    }
    pthread_mutex_unlock(&lock);
}

//#include "matplotlibcpp.h"

//namespace plt = matplotlibcpp;
int main() {
    antsdrDevice device;
    double gain = 10;
    if(device.open(true) < 0){
        return -1;
    }

    int start;
    std::cout << "Input 1 start test: ";
    std::cin >> start;

    if(start != 1)
        return -1;

//    int cnt_pll = 0;
//    bool is_lock = false;
//    while(cnt_pll <= 30){
//        std::cout << "get 10M value" <<std::endl;
//        if(device.get_10M_is_lock()){
//            std::cout << "10M clock is lock" << std::endl;
//            is_lock = true;
//            break;
//        }
//        cnt_pll++;
//        sleep(1);
//    }
//    if(not is_lock){
//        std::cerr << "PLL no lock" << std::endl;
//        return -2;
//    }

    sleep(2);
    std::cout << "begian test rf\n";
    zcProcess* zc1_63_192 = new zcProcess(1,1.92e6,63);
    CF32 *zc1_data = zc1_63_192->get_zc_xn();
    CF32 zc1_data_fft[128];
    fft_just(zc1_data,zc1_data_fft,128);


    device.set_tx_samprate(1.92e6);
    device.set_tx_freq(1e9);
    device.set_tx_attenuation(0);
    device.start_tx(1 << 0);


    device.set_rx_samprate(1.92e6);
    device.set_rx_freq(1e9);
    device.set_rx_gain(gain, 1 << 0);
    device.start_rx(get_rx_data,1<<0,NULL,SAMPLES);

    std::vector<double> cross_value(SAMPLES-128);
    for(int i=0;i<SAMPLES-128;i++)
        x_data.push_back(i);
    int cnt = 0;

//    while(cnt < 8){
//        pthread_mutex_lock(&lock);
//        if(x_data.size() == recv1_i.size()-128){
//            cross_correlation_to_find_peak(iq_data,SAMPLES-128, zc1_data, 128,cross_value.data());
//            plt::figure(1);
//            plt::clf();
//            plt::plot(x_data,cross_value,"b");
////            plt::plot(x_data,recv1_q,"g");
//            if(recv2_i.size() !=0 ){
//                plt::plot(x_data,recv2_i,"r");
//                plt::plot(x_data,recv2_q,"y");
//            }
//
//            plt::draw();
//            plt::pause(0.01);
//
//        }
//        gain += 10;
//        if(gain > 73)
//            gain = 73;
//        fprintf(stdout,"rx gain %f\n ",gain);
//        device.set_rx_gain(gain,1 << 0);
//        cnt++;
//        pthread_mutex_unlock(&lock);
//        sleep(1);
//    }
    while(cnt < 8){
        pthread_mutex_lock(&lock);
        cross_correlation_to_find_peak(iq_data,SAMPLES-128, zc1_data, 128,cross_value.data());
        auto max_it = std::max_element(cross_value.begin(), cross_value.end());
        int max_index = std::distance(cross_value.begin(), max_it);
        gain += 10;
        if(gain > 73)
            gain = 73;
        fprintf(stdout,"max value %f rx gain %f\n ",*max_it,gain);
        device.set_rx_gain(gain,1 << 0);
        cnt++;
        pthread_mutex_unlock(&lock);
        sleep(1);
    }
    device.stop_rx();
    fprintf(stdout,"Exit Program.\n");
    return 0;
}
