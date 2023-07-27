//
// Created by jcc on 23-7-26.
//

#ifndef ANTSDR_LIBIIO_ANTSDRDEVICE_HPP
#define ANTSDR_LIBIIO_ANTSDRDEVICE_HPP

#include "iio.h"
#include "thread"

typedef struct
{
    void *user;
    int16_t *data;
    int channels;
    int length;
} sdr_transfer;

typedef void(*RXdataCallback)(sdr_transfer *transfer);

class antsdrDevice{
public:
    antsdrDevice();
    ~antsdrDevice();

    int open();
    bool set_rx_freq(double freq);
    double get_rx_freq();
    bool set_rx_samprate(double fs);
    bool set_rx_gain(double gain);

    void stop_rx();


    /**
     * ad9361A
     * rx1 rx2
     * 1    1
     * 1    0
     * 0    1
     * */
    bool start_rx(RXdataCallback handler,int channels,void *user,int buff_size);
    void RXSyncThread(int channels);

private:
    struct iio_context *antsdr_ctx_;
    struct iio_device  *antsdr_rx_;
    struct iio_device  *phy_dev_;
    struct iio_buffer  *rx_buf_;
    struct iio_buffer  *tx_buf_;
    struct iio_channel *phy_rx_chn0_;


    struct iio_channel *rxone0_i_,*rxone0_q_;
    struct iio_channel *rxone1_i_,*rxone1_q_;




    bool is_Inited_;
    bool rx_running_;
    void *rx_user_;
    int64_t buffer_size_;
    RXdataCallback rx_handler_;
    std::thread *rx_thread_;
    char tmpstr_[64];


private:

    bool config_stream_device(iio_channel **channel,int chid,bool tx,struct iio_device *dev);
    void disable_chan(iio_channel ** chan);
    const char* get_chan_name(const char *type,int id);
};



#endif //ANTSDR_LIBIIO_ANTSDRDEVICE_HPP
