//
// Created by jcc on 23-7-26.
//

#include "antsdrDevice.h"
#include "functional"
#include "math.h"
#include "cstring"

// Function to generate a cosine signal
std::vector<int16_t> generateCosineSignal(double frequency, double sampleRate, double durationInSeconds,double phaseoffset) {
    std::vector<int16_t> signal;
    double angularFrequency = 2 * M_PI * frequency;
    double maxAmplitude = 32767.0; // Maximum positive value for int16_t

    // Calculate the number of samples needed for the desired duration
    int numSamples = static_cast<int>(durationInSeconds * sampleRate);

    for (int i = 0; i < numSamples; i++) {
        double time = static_cast<double>(i) / sampleRate;
        double sample = cos(angularFrequency * time + phaseoffset);
        int16_t scaledSample = static_cast<int16_t>(round(sample * maxAmplitude));
        signal.push_back(scaledSample);
    }

    return signal;
}


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
    if(*channel == NULL)
        fprintf(stderr,"find device %d iq channel failed.\n",chid);
    else
        fprintf(stdout,"find device %d iq channel.\n",chid);
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
    phy_rx_chn1_ = iio_device_find_channel(phy_dev_,"voltage1", false);
    if(phy_rx_chn1_ == NULL){
        fprintf(stderr,"cannot find ad9361 one rx ch1.\n");
        return -1;
    }
    phy_tx_chn0_ = iio_device_find_channel(phy_dev_,"voltage0", true);
    if(phy_tx_chn0_ == NULL){
        fprintf(stderr,"cannot find ad9361 one tx ch0.\n");
        return -1;
    }
    iio_channel_attr_write(phy_rx_chn0_, "gain_control_mode", "manual");
    iio_channel_attr_write(phy_rx_chn1_, "gain_control_mode", "manual");

    antsdr_rx_ = iio_context_find_device(antsdr_ctx_,"cf-ad9361-lpc");
    if(antsdr_rx_ == NULL){
        fprintf(stderr,"cannot find ad9361 rx iq.\n");
        return -1;
    }
    antsdr_tx_ = iio_context_find_device(antsdr_ctx_,"cf-ad9361-dds-core-lpc");
    if(antsdr_tx_ == NULL){
        fprintf(stderr,"cannot find ad9361 tx iq.\n");
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

bool antsdrDevice::set_rx_gain(double gain, int chanels) {
    if(!is_Inited_){
        return false;
    }
    int r;
    if(chanels & 0x1){
        if(phy_rx_chn0_ == NULL)
            return false;
        r = iio_channel_attr_write_double(phy_rx_chn0_, "hardwaregain", gain);
    }
    if(chanels & 0x2){
        if(phy_rx_chn1_ == NULL)
            return false;
        r = iio_channel_attr_write_double(phy_rx_chn1_, "hardwaregain", gain);
    }
    return r == 0;
}

void antsdrDevice::stop_rx() {
    rx_running_ = false;
    if(rx_thread_ && rx_thread_->joinable()) {
        rx_thread_->join();
        delete (rx_thread_);
        rx_thread_ = NULL;
    }
    if(rx_buf_){
        iio_buffer_destroy(rx_buf_);
        rx_buf_ = NULL;
    }
    disable_chan(&rxone0_i_);
    disable_chan(&rxone0_q_);
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
        fprintf(stdout,"Enable channels 1\n");
    }
    if(channels  & 0x2){
        if(config_stream_device(&rxone1_i_,2, false,antsdr_rx_) & config_stream_device(&rxone1_q_,3, false,antsdr_rx_)){
            fprintf(stdout,"Enable channels 2\n");
        }else{
            channels = 1;
        }
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
    if(channels & 0x1 && channels & 0x2)
        channels = 2;
    else
        channels = 1;
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

bool antsdrDevice::set_tx_freq(double freq) {
    if(not is_Inited_ or not phy_dev_)
        return false;
    struct iio_channel *phy = iio_device_find_channel(phy_dev_,"altvoltage1",true);
    int r = iio_channel_attr_write_longlong(phy,"frequency",(long long)freq);
    return r==0;
}

bool antsdrDevice::set_tx_samprate(double fs) {
    if(not is_Inited_ or not phy_tx_chn0_)
        return false;
    int r = iio_channel_attr_write_longlong(phy_tx_chn0_,"sampling_frequency",fs);;
    iio_channel_attr_write_longlong(phy_tx_chn0_, "rf_bandwidth", fs);
    return r==0;
}

bool antsdrDevice::set_tx_attenuation(double attenuation) {
    if(not is_Inited_ or not phy_tx_chn0_)
        return false;

    if(attenuation < 0 || attenuation > 89.75){
        fprintf(stderr,"ERROR INPUT 0-89.75");
        return false;
    }
    int r = iio_channel_attr_write_double(phy_tx_chn0_, "hardwaregain", -attenuation);
    return r==0;
}

bool antsdrDevice::start_tx(int channels) {
    int channelsCnt = 0;
    if(channels & 0x1){
        config_stream_device(&txone0_i_,0, true,antsdr_tx_);
        config_stream_device(&txone0_q_,1, true,antsdr_tx_);
        channelsCnt++;
    }
    if(channels  & 0x2){
        config_stream_device(&txone1_i_,2, true,antsdr_tx_);
        config_stream_device(&txone1_q_,3, true,antsdr_tx_);
        channelsCnt++;
    }
    iio_device_set_kernel_buffers_count(antsdr_tx_,1);

    double frequency = 20e3;
    double samprate = 3e6;
    double duration = 1.0 / frequency * 50;
    std::vector<int16_t> wave_signal_i = generateCosineSignal(frequency,samprate,duration,0);
    std::vector<int16_t> wave_signal_q = generateCosineSignal(frequency,samprate,duration,- M_PI / 2);

    tx_buf_ = iio_device_create_buffer(antsdr_tx_,wave_signal_i.size()*channelsCnt, true);
    auto *tx_buffer = (int16_t*) iio_buffer_start(tx_buf_);
    memset(tx_buffer,0,sizeof(int16_t )*wave_signal_i.size()*channelsCnt);
    for(int i=0;i<wave_signal_i.size()*channelsCnt;i++){
        tx_buffer[2*i] = wave_signal_i[i];
        tx_buffer[2*i+1] = wave_signal_q[i];
    }
    ssize_t n_write = iio_buffer_push(tx_buf_);
    if(n_write < 0){
        fprintf(stderr,"tx send failed\n");
        return false;
    }else{
        fprintf(stdout,"tx send %d samples.\n",n_write / 4);
    }
    return true;
}

void antsdrDevice::stop_tx() {
    disable_chan(&txone0_i_);
    disable_chan(&txone0_q_);
    disable_chan(&txone1_i_);
    disable_chan(&txone1_i_);
    if(tx_buf_){
        iio_buffer_destroy(tx_buf_);
        tx_buf_ = NULL;
    }
}









