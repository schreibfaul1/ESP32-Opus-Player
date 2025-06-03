// OPUS player demo
// plays Opus files from SD via I2S


#include <Arduino.h>
#include "SD_MMC.h"
#include "FS.h"
#include "driver/i2s.h"
#include "opusfile.h"


// Digital I/O used
#ifdef CONFIG_IDF_TARGET_ESP32S3
    #define SD_MMC_D0     11
    #define SD_MMC_CLK    13
    #define SD_MMC_CMD    14
    #define I2S_DOUT       9
    #define I2S_BCLK       3
    #define I2S_LRC        1
#endif

#ifdef CONFIG_IDF_TARGET_ESP32
    #define SD_MMC_D0      2
    #define SD_MMC_CLK    14
    #define SD_MMC_CMD    15
    #define I2S_DOUT      25
    #define I2S_BCLK      27
    #define I2S_LRC       26
#endif

uint8_t             m_i2s_num = I2S_NUM_0;          // I2S_NUM_0 or I2S_NUM_1
i2s_config_t        m_i2s_config;                   // stores values for I2S driver
i2s_pin_config_t    m_pin_config;
uint32_t            m_sampleRate=16000;
uint8_t             m_bitsPerSample = 16;           // bitsPerSample
uint8_t             m_vol=64;                       // volume
size_t              m_i2s_bytesWritten = 0;         // set in i2s_write() but not used
uint8_t             m_channels=2;
int16_t             m_outBuff[2048*2];              // Interleaved L/R
int16_t             m_validSamples = 0;
int16_t             m_curSample = 0;
boolean             m_f_forceMono = false;

typedef enum { LEFTCHANNEL=0, RIGHTCHANNEL=1 } SampleIndex;

const uint8_t volumetable[22]={   0,  1,  2,  3,  4 , 6 , 8, 10, 12, 14, 17,
                                 20, 23, 27, 30 ,34, 38, 43 ,48, 52, 58, 64}; //22 elements

TaskHandle_t opus_task;
File file;

// prototypes
bool playSample(int16_t sample[2]);

//---------------------------------------------------------------------------------------------------------------------
//        I 2 S   S t u f f
//---------------------------------------------------------------------------------------------------------------------
void setupI2S(){
    m_i2s_num = I2S_NUM_0; // i2s port number
    m_i2s_config.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    m_i2s_config.sample_rate          = 16000;
    m_i2s_config.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    m_i2s_config.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
    m_i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S);
    m_i2s_config.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1; // high interrupt priority
    m_i2s_config.dma_buf_count        = 8;      // max buffers
    m_i2s_config.dma_buf_len          = 1024;   // max value
    m_i2s_config.tx_desc_auto_clear   = true;   // new in V1.0.1
    m_i2s_config.fixed_mclk           = I2S_PIN_NO_CHANGE;
    i2s_driver_install((i2s_port_t)m_i2s_num, &m_i2s_config, 0, NULL);
}
//---------------------------------------------------------------------------------------------------------------------
esp_err_t I2Sstart(uint8_t i2s_num) {
    // It is not necessary to call this function after i2s_driver_install() (it is started automatically),
    // however it is necessary to call it after i2s_stop()
    return i2s_start((i2s_port_t) i2s_num);
}
//---------------------------------------------------------------------------------------------------------------------
esp_err_t I2Sstop(uint8_t i2s_num) {
    return i2s_stop((i2s_port_t) i2s_num);
}
//---------------------------------------------------------------------------------------------------------------------
bool setPinout(uint8_t BCLK, uint8_t LRC, uint8_t DOUT, int8_t DIN) {

    m_pin_config.bck_io_num   = BCLK;
    m_pin_config.ws_io_num    = LRC; //  wclk
    m_pin_config.data_out_num = DOUT;
    m_pin_config.data_in_num  = DIN;
    const esp_err_t result = i2s_set_pin((i2s_port_t) m_i2s_num, &m_pin_config);
    return (result == ESP_OK);
}
//---------------------------------------------------------------------------------------------------------------------
bool setSampleRate(uint32_t sampRate) {
    i2s_set_sample_rates((i2s_port_t)m_i2s_num, sampRate);
    m_sampleRate = sampRate;
    return true;
}
//---------------------------------------------------------------------------------------------------------------------
bool setBitsPerSample(int bits) {
    if((bits != 16) && (bits != 8)) return false;
    m_bitsPerSample = bits;
    return true;
}
uint8_t getBitsPerSample(){
    return m_bitsPerSample;
}
//---------------------------------------------------------------------------------------------------------------------
bool setChannels(int ch) {
    if((ch < 1) || (ch > 2)) return false;
    m_channels = ch;
    return true;
}
uint8_t getChannels(){
    return m_channels;
}
//---------------------------------------------------------------------------------------------------------------------
int32_t Gain(int16_t s[2]) {
    int32_t v[2];
    float step = (float)m_vol /64;
    uint8_t l = 0, r = 0;

    v[LEFTCHANNEL] = (s[LEFTCHANNEL]  * (m_vol - l)) >> 6;
    v[RIGHTCHANNEL]= (s[RIGHTCHANNEL] * (m_vol - r)) >> 6;

    return (v[RIGHTCHANNEL] << 16) | (v[LEFTCHANNEL] & 0xffff);
}
//---------------------------------------------------------------------------------------------------------------------
bool playChunk() {
    // If we've got data, try and pump it out..
    int16_t sample[2];
    if(getBitsPerSample() == 8) {
        if(m_channels == 1) {
            while(m_validSamples) {
                uint8_t x =  m_outBuff[m_curSample] & 0x00FF;
                uint8_t y = (m_outBuff[m_curSample] & 0xFF00) >> 8;
                sample[LEFTCHANNEL]  = x;
                sample[RIGHTCHANNEL] = x;
                while(1) {
                    if(playSample(sample)) break;
                } // Can't send?
                sample[LEFTCHANNEL]  = y;
                sample[RIGHTCHANNEL] = y;
                while(1) {
                    if(playSample(sample)) break;
                } // Can't send?
                m_validSamples--;
                m_curSample++;
            }
        }
        if(m_channels == 2) {
            while(m_validSamples) {
                uint8_t x =  m_outBuff[m_curSample] & 0x00FF;
                uint8_t y = (m_outBuff[m_curSample] & 0xFF00) >> 8;
                if(!m_f_forceMono) { // stereo mode
                    sample[LEFTCHANNEL]  = x;
                    sample[RIGHTCHANNEL] = y;
                }
                else { // force mono
                    uint8_t xy = (x + y) / 2;
                    sample[LEFTCHANNEL]  = xy;
                    sample[RIGHTCHANNEL] = xy;
                }

                while(1) {
                    if(playSample(sample)) break;
                } // Can't send?
                m_validSamples--;
                m_curSample++;
            }
        }
        m_curSample = 0;
        return true;
    }
    if(getBitsPerSample() == 16) {
        if(m_channels == 1) {
            while(m_validSamples) {
                sample[LEFTCHANNEL]  = m_outBuff[m_curSample];
                sample[RIGHTCHANNEL] = m_outBuff[m_curSample];
                if(!playSample(sample)) {
                    return false;
                } // Can't send
                m_validSamples--;
                m_curSample++;
            }
        }
        if(m_channels == 2) {
            while(m_validSamples) {
                if(!m_f_forceMono) { // stereo mode
                    sample[LEFTCHANNEL]  = m_outBuff[m_curSample * 2];
                    sample[RIGHTCHANNEL] = m_outBuff[m_curSample * 2 + 1];
                }
                else { // mono mode, #100
                    int16_t xy = (m_outBuff[m_curSample * 2] + m_outBuff[m_curSample * 2 + 1]) / 2;
                    sample[LEFTCHANNEL] = xy;
                    sample[RIGHTCHANNEL] = xy;
                }
                if(!playSample(sample)) {
                    return false;
                } // Can't send
                m_validSamples--;
                m_curSample++;
            }
        }
        m_curSample = 0;
        return true;
    }
    log_e("BitsPer Sample must be 8 or 16!");
    return false;
}
//---------------------------------------------------------------------------------------------------------------------
bool playSample(int16_t sample[2]) {

    int16_t sample1[2]; int16_t* s1;
    int16_t sample2[2]; int16_t* s2 = sample2;
    int16_t sample3[2]; int16_t* s3 = sample3;

    if (getBitsPerSample() == 8) { // Upsample from unsigned 8 bits to signed 16 bits
        sample[LEFTCHANNEL]  = ((sample[LEFTCHANNEL]  & 0xff) -128) << 8;
        sample[RIGHTCHANNEL] = ((sample[RIGHTCHANNEL] & 0xff) -128) << 8;
    }

    sample[LEFTCHANNEL]  = sample[LEFTCHANNEL]  >> 1; // half Vin so we can boost up to 6dB in filters
    sample[RIGHTCHANNEL] = sample[RIGHTCHANNEL] >> 1;

    uint32_t s32 = Gain(sample); // vosample2lume;

    esp_err_t err = i2s_write((i2s_port_t) m_i2s_num, (const char*) &s32, sizeof(uint32_t), &m_i2s_bytesWritten, 1000);
    if(err != ESP_OK) {
        log_e("ESP32 Errorcode %i", err);
        return false;
    }
    if(m_i2s_bytesWritten < 4) {
        log_e("Can't stuff any more in I2S..."); // increase waitingtime or outputbuffer
        return false;
    }
    return true;
}

//---------------------------------------------------------------------------------------------------------------------
//   O P U S   S t u f f
//---------------------------------------------------------------------------------------------------------------------
int SD_read(unsigned char* buff, int nbytes){
    if (nbytes == 0) return 0;
    nbytes = file.read(buff, nbytes);
    if (nbytes == 0) return -1;
    return nbytes;
}
void opusTask(void *parameter) {
    int ret;
    do {
        ret = op_read_stereo(m_outBuff, 2048);
        if(ret > 0){
            m_validSamples = ret;
            playChunk();
        }
        vTaskDelay(5);
        // log_e("%u", uxTaskGetStackHighWaterMark(NULL));
    } while(ret > 0);
    vTaskDelete(opus_task);
}
//---------------------------------------------------------------------------------------------------------------------
void setup() {
    setupI2S();
    setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT, -1);
    setBitsPerSample(16);
    setChannels(2);
    setSampleRate(48000);
    I2Sstart(m_i2s_num);
    Serial.begin(115200);
    delay(1000);
    pinMode(SD_MMC_D0, INPUT_PULLUP);
    #ifdef CONFIG_IDF_TARGET_ESP32S3
        SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
    #endif
    if(!SD_MMC.begin("/sdcard", true)){
        log_i("SD card not found,");
        while(true){;}
    }

 //   file = SD_MMC.open("/opus/Symphony No.6 (1st movement).opus");
 //   file = SD_MMC.open("/opus/hybrid.opus");
 //   file = SD_MMC.open("/opus/celt_8000Hz.opus");
 //   file = SD_MMC.open("/opus/silk_8000Hz.opus");
      file = SD_MMC.open("/opus/dlf24.opus");
    log_i("free heap before %d", ESP.getFreeHeap());
    opus_init_decoder();
    log_i("free heap after %d", ESP.getFreeHeap());
    xTaskCreatePinnedToCore(
            opusTask, /* Function to implement the task */
            "OPUS", /* Name of the task */
            4096 * 6,  /* Stack size in words */
            NULL,  /* Task input parameter */
            2,  /* Priority of the task */
            &opus_task,  /* Task handle. */
            1 /* Core where the task should run */
    );
}

void loop() {
    ;
}
//---------------------------------------------------------------------------------------------------------------------