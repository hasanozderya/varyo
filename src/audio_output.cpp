#include "audio_output.h"
#include "audio_pcm.h"
#include "config.h"
#include "debug_log.h"
#include <driver/i2s_std.h>
#include <freertos/task.h>

namespace {
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    AudioToneRequest request;
    constexpr size_t FRAMES = 128; // 8 ms per block, four DMA blocks

    AudioToneRequest snapshot() {
        portENTER_CRITICAL(&mux);
        const auto result = request;
        portEXIT_CRITICAL(&mux);
        return result;
    }

    esp_err_t openChannel(i2s_chan_handle_t& tx) {
        i2s_chan_config_t channel = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
        channel.dma_desc_num = 4;
        channel.dma_frame_num = FRAMES;
        // If the audio task stalls, DMA sends zeros instead of replaying a tone.
        channel.auto_clear = true;
        esp_err_t error = i2s_new_channel(&channel, &tx, nullptr);
        if (error != ESP_OK) return error;
        i2s_std_config_t config{};
        config.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AudioPcm::SAMPLE_RATE);
        config.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
        config.gpio_cfg.mclk = I2S_GPIO_UNUSED;
        config.gpio_cfg.bclk = (gpio_num_t)Pins::AUDIO_BCLK;
        config.gpio_cfg.ws = (gpio_num_t)Pins::AUDIO_WS;
        config.gpio_cfg.dout = (gpio_num_t)Pins::AUDIO_DATA;
        config.gpio_cfg.din = I2S_GPIO_UNUSED;
        error = i2s_channel_init_std_mode(tx, &config);
        if (error == ESP_OK) error = i2s_channel_enable(tx);
        if (error != ESP_OK) { i2s_del_channel(tx); tx = nullptr; }
        return error;
    }

    void task(void*) {
        int16_t samples[FRAMES*2];
        esp_err_t previousError = ESP_OK;
        for (;;) {
            i2s_chan_handle_t tx = nullptr;
            esp_err_t error = openChannel(tx);
            if (error == ESP_OK) {
                DebugLog::log("Audio: MAX98357A I2S ready, DATA=4 BCLK=5 WS=6, 16kHz\n");
                previousError = ESP_OK;
                AudioPcm pcm;
                for (;;) {
                    const auto tone = snapshot();
                    pcm.render(samples, FRAMES, tone.effectiveFrequency(millis()), tone.volume);
                    size_t written = 0;
                    error = i2s_channel_write(tx, samples, sizeof(samples), &written, 100);
                    if (error != ESP_OK || written != sizeof(samples)) {
                        if (error == ESP_OK) error = ESP_FAIL;
                        break;
                    }
                }
                i2s_channel_disable(tx);
                i2s_del_channel(tx);
            }
            if (error != previousError) {
                DebugLog::logf("Audio: I2S error %s; retry in 2 seconds\n", esp_err_to_name(error));
                previousError = error;
            }
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
}

bool AudioOutput::begin() {
    return xTaskCreatePinnedToCore(task, "AudioI2S", 4096, nullptr, 3, nullptr, 1) == pdPASS;
}

void AudioOutput::setTone(uint32_t frequency, int volume) {
    const AudioToneRequest next{frequency, constrain(volume, 0, 100), millis()};
    portENTER_CRITICAL(&mux);
    request = next;
    portEXIT_CRITICAL(&mux);
}
