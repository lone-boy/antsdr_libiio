//
// Created by jcc on 23-7-26.
//

#include "antsdrDevice.hpp"
#include "functional"

antsdrDevice::antsdrDevice()
:is_Inited_(false),rx_user_(NULL),rx_buf_(NULL),rx_thread_(NULL)
,rx_handler_(NULL), rx_running_(false)
{
    rxone0_i_ = NULL;
    rxone0_q_ = NULL;
    rxone1_i_ = NULL;
    rxone1_q_ = NULL;
}

antsdrDevice::~antsdrDevice() {

}

const char *antsdrDevice::get_chan_name(const char *type, int id) {
    snprintf(tmpstr_,sizeof(tmpstr_),"%s%d",type,id);
    return tmpstr_;
}

bool antsdrDevice::config_stream_device(iio_channel **channel, int chid, bool tx, struct iio_device *dev) {
    *channel = iio_device_find_channel(dev, get_chan_name("voltage",chid),tx);
    fprintf(stderr,"find device %d iq channel.\n",chid);
    if(*channel != NULL)
        iio_channel_enable(*channel);
    return *channel != NULL;
}

void antsdrDevice::disable_chan(iio_channel **chan) {
    if(*chan != NULL)
        iio_channel_disable(*chan);
    *chan = NULL;
}

int antsdrDevice::open() {
    if(is_Inited_)
        return 0;

    antsdr_ctx_ = iio_create_context_from_uri("ip:192.168.1.10");
    if(antsdr_ctx_ == NULL){
        fprintf(stderr,"cannot find device.\n");
        return -1;
    }

    phy_dev_ = iio_context_find_device(antsdr_ctx_,"ad9361-phy");
    if(phy_dev_ == NULL){
        fprintf(stderr,"cannot find ad9361 device.\n");
    }

    phy_rx_chn0_ = iio_device_find_channel(phy_dev_,"voltage0", false);
    if(phy_rx_chn0_ == NULL){
        fprintf(stderr,"cannot find ad9361 one rx ch0.\n");
        return -1;
    }


    iio_channel_attr_write(phy_rx_chn0_, "gain_control_mode", "manual");

    antsdr_rx_ = iio_context_find_device(antsdr_ctx_,"cf-ad9361-lpc");
    if(antsdr_rx_ == NULL){
        fprintf(stderr,"cannot find ad9361 rx iq.\n");
        return -1;
    }
    fprintf(stdout,"Find ad9361 device.\nWelcom to ANTSDR E310.\n");
    is_Inited_ = true;
    return 0;
}

bool antsdrDevice::set_rx_freq(double freq) {
    if(not is_Inited_ or not phy_dev_)
        return false;
    struct iio_channel *phy = iio_device_find_channel(phy_dev_,"altvoltage0",true);
    int r = iio_channel_attr_write_longlong(phy,"frequency",(long long)freq);
    return r==0;
}

double antsdrDevice::get_rx_freq() {
    long long int frequency;
    if(not is_Inited_ or not phy_dev_)
        return false;
    struct iio_channel *phy = iio_device_find_channel(phy_dev_,"altvoltage0",true);
    int r = iio_channel_attr_read_longlong(phy,"frequency",&frequency);
    return r == 0 ? (double)frequency : 0;
}

bool antsdrDevice::set_rx_samprate(double fs) {
    if(not is_Inited_ or not phy_dev_)
        return false;
    int r = iio_channel_attr_write_longlong(phy_rx_chn0_,"sampling_frequency",(long long)fs);
    iio_channel_attr_write_longlong(phy_rx_chn0_, "rf_bandwidth", fs);
    return r==0;
}

bool antsdrDevice::set_rx_gain(double gain) {
    if(!is_Inited_ or phy_rx_chn0_){
        return false;
    }
    int r = iio_channel_attr_write_double(phy_rx_chn0_, "hardwaregain", gain);
    return r == 0;
}

void antsdrDevice::stop_rx() {
    rx_running_ = false;
    if(rx_thread_ && rx_thread_->joinable()) {
        rx_thread_->join();
        delete (rx_thread_);
        rx_thread_ = NULL;
    }
    disable_chan(&rxone0_i_);
    disable_chan(&rxone0_q_);
    if(rx_buf_){
        iio_buffer_destroy(rx_buf_);
        rx_buf_ = NULL;
    }
}

bool antsdrDevice::start_rx(RXdataCallback handler, int channels, void *user, int buff_size) {
    rx_handler_ = handler;
    rx_user_ = user;
    buffer_size_ = buff_size;
    if(not is_Inited_)
        return false;
    printf("channels=%d\n",channels);
    /* iio_channel_enable */
    if(channels & 0x1){
        config_stream_device(&rxone0_i_,0, false,antsdr_rx_);
        config_stream_device(&rxone0_q_,1, false,antsdr_rx_);

    }
    if(channels  & 0x2){
        config_stream_device(&rxone1_i_,2, false,antsdr_rx_);
        config_stream_device(&rxone1_q_,3, false,antsdr_rx_);
    }
    iio_device_set_kernel_buffers_count(antsdr_rx_,2);
    if(! rx_running_ && !rx_thread_){
        rx_running_ = true;
        rx_thread_ = new std::thread(std::bind(&antsdrDevice::RXSyncThread,this,channels));
    }
    return rx_running_;
}

void antsdrDevice::RXSyncThread(int channels) {
    sdr_transfer trans;

    rx_buf_ = iio_device_create_buffer(antsdr_rx_,buffer_size_, false);
    while(rx_running_){
        int n_read = iio_buffer_refill(rx_buf_);
        if(n_read > 0){
            trans.data = (int16_t*) iio_buffer_start(rx_buf_);
            trans.user = rx_user_;
            trans.length = n_read / 4;
            trans.channels = channels;
            rx_handler_(&trans);
        }
        else{
            break;
        }

    }
    rx_running_ = false;
}









