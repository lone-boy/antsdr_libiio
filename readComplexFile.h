//
// Created by jcc on 23-8-2.
//

#ifndef ANTSDR_DRONE_READCOMPLEXFILE_H
#define ANTSDR_DRONE_READCOMPLEXFILE_H
#include "unistd.h"
#include "common.h"
#include "log.h"
#include <cstdio>


class ReadComplexFile{
public:
    ReadComplexFile(const std::string& file_name,uint16_t samples_bytes){
        fp_ = fopen(file_name.c_str(),"rb");
        if(fp_ == nullptr){
            log_error("Faile open file");
        }
        fseek(fp_,0L,SEEK_END);
        int file_size = ftell(fp_);
        fseek(fp_,0L,SEEK_SET);
        log_info("Read file size %d bytes.",file_size);
        samples_ = file_size / 2 /  samples_bytes;
        log_info("Read %d samples from file",samples_);

        buffer_ = (CI16*) malloc(sizeof(CI16)*samples_);
        log_info("Malloc %d size",sizeof(CI16)*samples_);
        if(buffer_ != nullptr){
            size_t read_samples = fread(buffer_,sizeof(CI16),samples_,fp_);
            if(read_samples == samples_)
                log_info("Read %d samples success.",read_samples);
        }
        fclose(fp_);
        fp_ = nullptr;
    }
    ~ReadComplexFile(){
        if(fp_)
            fclose(fp_);
    }


public:
    CI16 *get_buffer(){
        return buffer_;
    }

    uint32_t get_samples(){
        return samples_;
    }

private:
    FILE    *fp_;
    CI16 *buffer_;
    uint32_t samples_;
};


#endif //ANTSDR_DRONE_READCOMPLEXFILE_H
