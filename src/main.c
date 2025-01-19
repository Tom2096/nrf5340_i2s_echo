/*
	Author: Siyuan He
	Email: sy4he@uwaterloo.ca

    This is a simple example of how to use the I2S callback to achieve a simple audio out from PDM -> Headphones.
*/

#include <zephyr/kernel.h>
#include <nrfx_clock.h>
#include <audio_i2s.h>
#include <audio_i2s_macros.h>
#include <zephyr/sys/ring_buffer.h>
#include <pcm_stream_channel_modifier.h>
#include <macros_common.h>
#include <hw_codec.h>
#include <tone.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);

#define CONFIG_LOGGING

static uint8_t *tx_buf;
static uint8_t *rx_buf;

/* Ring buffer for PDM Mic input */
RING_BUF_DECLARE(ringbuf_audio_in, RING_BUF_SIZE);

/* Ring buffer for intermediate processing */
RING_BUF_DECLARE(ringbuf_audio_tmp, RING_BUF_SIZE);

/* Ring buffer for headphone out */
RING_BUF_DECLARE(ringbuf_audio_out, RING_BUF_SIZE);

/* Alternate-buffers used when there is no active audio stream.
 * Used interchangeably by I2S.
 */
static struct {
	uint8_t __aligned(WB_UP(1)) buf_0[BLK_STEREO_SIZE_OCTETS];
	uint8_t __aligned(WB_UP(1)) buf_1[BLK_STEREO_SIZE_OCTETS];
	bool buf_0_in_use;
	bool buf_1_in_use;
} alt;

/**
 * @brief	Get first available alternative-buffer.
 *
 * @param	p_buffer	Double pointer to populate with buffer.
 *
 * @retval	0 if success.
 * @retval	-ENOMEM No available buffers.
 */
static int alt_buffer_get(void **p_buffer)
{
	if (!alt.buf_0_in_use) {
		alt.buf_0_in_use = true;
		*p_buffer = alt.buf_0;
	} else if (!alt.buf_1_in_use) {
		alt.buf_1_in_use = true;
		*p_buffer = alt.buf_1;
	} else {
		return -ENOMEM;
	}

	return 0;
}

/**
 * @brief	Frees both alternative buffers.
 */
static void alt_buffer_free_both(void)
{
	alt.buf_0_in_use = false;
	alt.buf_1_in_use = false;
}

/**
 * @brief	Function to be called when I2S transfer is complete (every ~1ms).
 * 			
 * @param	frame_start_ts_us 	I2S frame start timestamp
 * @param	rx_buf_released 	Pointer to the released buffer containing PDM received data
 * @param	tx_buf_released 	Pointer to the released buffer that was used to send data
 */
static void i2s_callback(uint32_t frame_start_ts_us, uint32_t *rx_buf_released,
					    uint32_t const *tx_buf_released)
{

	/* There should be 2 main parts to this callback:
	 *
	 * 1st. RX: The rx_buf_released is the buffer that was just filled with PDM data, 
	 *		we should now call ring_buf_put_finish to complete the data write operation.
	 *
	 * 2nd. TX: The tx_buf_released is the buffer that was just sent to the I2S peripheral,
	 *		we should now call ring_buf_get_claim to claim the next buffer to be sent.
	 */

	int ret;
	size_t transfer_size = I2S_TRANSFER_SIZE;

	/********** I2S RX **********/
	if (rx_buf_released != NULL) {
		ret = ring_buf_put_finish(&ringbuf_audio_in, transfer_size);
		if (ret) {
			LOG_WRN("Ring buf put finish err: %d", ret);
		}

		/* Attempt to make space in the ring buffer for rx_buf */
		if (ring_buf_space_get(&ringbuf_audio_in) < transfer_size) {
			LOG_ERR("Overflow in ring buffer");
			return;
		}
		transfer_size = ring_buf_put_claim(&ringbuf_audio_in, &rx_buf, transfer_size);
		if (transfer_size != I2S_TRANSFER_SIZE) {
			LOG_ERR("Write size (%d) not equal requested size (%d)", transfer_size, I2S_TRANSFER_SIZE);
			return;
		}
	} else {
		LOG_ERR("rx_buf_released is null!");
		return;
	}

	/********** I2S TX **********/
	if (tx_buf_released != NULL) {

		/* Finish reading the previous tx_buf */
		ret = ring_buf_get_finish(&ringbuf_audio_out, transfer_size);
		if (ret) {
			LOG_WRN("Ring buf get finish err: %d", ret);
		}

		transfer_size = ring_buf_get_claim(&ringbuf_audio_out, &tx_buf, transfer_size);
		if (transfer_size != I2S_TRANSFER_SIZE) {
			LOG_WRN("Read size (%d) not equal requested size (%d)", transfer_size, I2S_TRANSFER_SIZE);
		}
	} else {
		LOG_ERR("tx_buf_released is null!");
		return;
	}

	/*** Data exchange ***/
	audio_i2s_set_next_buf(tx_buf, (uint32_t *)rx_buf);
}

static int hfclock_config_and_start(void)
{
	int ret;

	/* Use this to turn on 128 MHz clock for cpu_app */
	ret = nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);

	ret -= NRFX_ERROR_BASE_NUM;
	if (ret) {
		return ret;
	}

	nrfx_clock_hfclk_start();
	while (!nrfx_clock_hfclk_is_running()) {
	}

	return 0;
}

int main(void) 
{
    int ret;

#ifdef CONFIG_LOGGING
    printk("\nLogging I2S Info ...\n");
    printk("\nCONFIG_I2S_LRCK_FREQ_HZ:     %d\n", CONFIG_I2S_LRCK_FREQ_HZ);
    printk("\nCONFIG_I2S_CH_NUM:           %d\n", CONFIG_I2S_CH_NUM);
    printk("\nCONFIG_AUDIO_BIT_DEPTH_BITS: %d\n", CONFIG_AUDIO_BIT_DEPTH_BITS);
	printk("\nI2S_SAMPLES_NUM: 		   	   %d\n", I2S_SAMPLES_NUM);
	printk("\nBLK_STEREO_SIZE_OCTETS: 	   %d\n", BLK_STEREO_SIZE_OCTETS);
#endif
 
	ret = hfclock_config_and_start();
	ERR_CHK(ret);

	audio_i2s_blk_comp_cb_register(i2s_callback);
	audio_i2s_init();

	ret = hw_codec_init();
	if (ret) {
		LOG_ERR("Failed to initialize HW codec: %d", ret);
		return ret;
	}
	hw_codec_volume_set(200);

	printk("\nStarting I2S!\n");

	/* Starting I2S */

	/********** I2S TX **********/
	ret = alt_buffer_get((void **)&tx_buf);
	ERR_CHK(ret);

	/********** I2S RX **********/
	ret = alt_buffer_get((void **)&rx_buf);
	ERR_CHK(ret);

	memset(tx_buf, 0, BLK_STEREO_SIZE_OCTETS);

	ret = hw_codec_default_conf_enable();
	ERR_CHK(ret);
	audio_i2s_start(tx_buf, (uint32_t *)rx_buf);
	// Need to run this manually once to trigger the callback
	audio_i2s_set_next_buf(tx_buf, (uint32_t *)rx_buf);
	
    /**
     * 
     *  We will run the I2S for 5 seconds and then stop it.
     * 
     *  The I2S callback will be called every ~1ms, and we will, in a busy loop, repeatly read from ringbuf_audio_in and write to ringbuf_audio_tmp.
     *  This is to simulate some processing that would be done on the audio data. We will then read from ringbuf_audio_tmp and write to ringbuf_audio_out.
     * 
     */
    
	uint8_t *data_buf;
	size_t transfer_size;
	int64_t start_time = k_uptime_get();

	while (true) {
		int64_t current_time = k_uptime_get();
		int64_t elapsed_time_ms = current_time - start_time;
		
        if (elapsed_time_ms >= 5000) { // Check if 5000 ms has passed
            break; // Exit the loop after 5 seconds
        }

		transfer_size = I2S_TRANSFER_SIZE;
		/* Attempt to read from ringbuf_audio_in */
		if (ring_buf_size_get(&ringbuf_audio_in) > 0) {
			/* Attempt to make space in ringbuf_audio_tmp */
			if (ring_buf_space_get(&ringbuf_audio_tmp) < transfer_size) {
				LOG_ERR("Overflow in ringbuf_audio_tmp");
				break;
			}
			transfer_size = ring_buf_get_claim(&ringbuf_audio_in, &data_buf, transfer_size);
			if (transfer_size != I2S_TRANSFER_SIZE) {
				LOG_ERR("Read size of ringbuf_audio_in (%d) not equal available_data (%d)", transfer_size, I2S_TRANSFER_SIZE);
				break;
			}
			transfer_size = ring_buf_put(&ringbuf_audio_tmp, data_buf, transfer_size);
			if (transfer_size != I2S_TRANSFER_SIZE) {
				LOG_ERR("Write size of ringbuf_audio_tmp (%d) not equal available_data (%d)", transfer_size, I2S_TRANSFER_SIZE);
				break;
			}
			ret = ring_buf_get_finish(&ringbuf_audio_in, transfer_size);
			if (ret) {
				LOG_ERR("ringbuf_audio_in get finish err: %d", ret);
				break;
			}
		}

		transfer_size = RING_BUF_BATCH_SIZE;
		/* Attempt to write from ringbuf_audio_tmp to ringbuf_audio_out */
		if (ring_buf_size_get(&ringbuf_audio_tmp) >= transfer_size) {
			/* Attempt to make space in ringbuf_audio_out */
			if (ring_buf_space_get(&ringbuf_audio_out) < transfer_size) {
				LOG_ERR("Overflow in ringbuf_audio_out");
				break;
			}
			transfer_size = ring_buf_get_claim(&ringbuf_audio_tmp, &data_buf, transfer_size);
			if (transfer_size != RING_BUF_BATCH_SIZE) {
				LOG_ERR("Read size of ringbuf_audio_tmp (%d) not equal available_data (%d)", transfer_size, RING_BUF_BATCH_SIZE);
				break;
			}
			transfer_size = ring_buf_put(&ringbuf_audio_out, data_buf, transfer_size);
			if (transfer_size != RING_BUF_BATCH_SIZE) {
				LOG_ERR("Write size of ringbuf_audio_out (%d) not equal available_data (%d)", transfer_size, RING_BUF_BATCH_SIZE);
				break;
			}
			ret = ring_buf_get_finish(&ringbuf_audio_tmp, transfer_size);
			if (ret) {
				LOG_ERR("ringbuf_audio_tmp get finish err: %d", ret);
				break;
			}
		}
	}

	ret = hw_codec_soft_reset();
	ERR_CHK(ret);
	audio_i2s_stop();
	
	alt_buffer_free_both();

	printk("\nI2S Stopped!\n");

    return 0;
}