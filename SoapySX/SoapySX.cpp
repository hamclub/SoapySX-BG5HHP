// SPDX-License-Identifier: MIT

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Time.hpp>

#include <string.h>
#include <cassert>
#include <algorithm>
#include <climits>
#include <chrono>
#include <complex>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <thread>
#include <mutex>
#include <sstream>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/spi/spidev.h>

#include <alsa/asoundlib.h>
#include <alsa/control.h>

extern const char *SoapySX_tag;
extern const char *SoapySX_commit;

// Streaming mode, affecting how starting, stopping, overruns and underruns
// are handled.
enum stream_mode {
    // Behave like most SDRs: RX overrun or TX underrun
    // may cause samples to be dropped, but streams will keep running.
    // Application can use timestamps to maintain correct timing.
    STREAM_MODE_NORMAL,
    // Behave like linked ALSA PCMs with default software parameters.
    // Overrun or underrun causes both RX and TX streams to stop.
    // TX buffer must be always kept filled to avoid underrun.
    // First write to TX stream starts both streams.
    // This was a stop-gap solution before implementation of timestamps
    // and is primarily kept here for compatibility with applications
    // that have not been modified to use timestamps yet.
    STREAM_MODE_LINK,
};

// Clamp, offset, scale and quantize a value based on a SoapySDR::Range
// and convert it to an integer.
// The value is offset so that the minimum value becomes 0
// and scaled so that step becomes 1.
static int32_t scale_from_range(SoapySDR::Range range, double value)
{
    return (int)std::round(
        (std::min(std::max(value, range.minimum()), range.maximum())
         - range.minimum()) / range.step());
}

// Inverse of scale_from_range.
static double scale_to_range(SoapySDR::Range range, int32_t value)
{
    return std::min(std::max(
        range.minimum() + range.step() * (double)value,
        range.minimum()), range.maximum());
}

// Read a number from a file in format 0x1234.
static uint16_t read_hex_file(const char *filename)
{
    std::ifstream file(filename);
    std::string str;
    std::getline(file, str);
    return std::stoul(str, NULL, 16);
}

static std::string format_double(double value, int precision = 3)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision) << value;
    return ss.str();
}

static unsigned parse_unsigned_setting(const std::string &value, unsigned min_value, unsigned max_value)
{
    unsigned parsed = std::stoul(value, NULL, 0);
    if (parsed < min_value || parsed > max_value) {
        throw std::runtime_error("Setting value out of range");
    }
    return parsed;
}

static float parse_correction_setting(const std::string &value)
{
    double parsed = std::stod(value);
    if (!std::isfinite(parsed) || std::abs(parsed) > 1.0) {
        throw std::runtime_error("Correction value must be finite and within +/-1.0");
    }
    return (float)parsed;
}

struct hat_info {
    uint16_t product_id;
    uint16_t product_ver;
    bool read_success;
};

static struct hat_info read_hat_info(void)
{
    const uint16_t product_id_expected = 0x1255;
    // Default assumption if ID cannot be read
    struct hat_info info = { product_id_expected, 0x0101, false };
    try {
        info.product_id  = read_hex_file("/proc/device-tree/hat/product_id");
        info.product_ver = read_hex_file("/proc/device-tree/hat/product_ver");
        info.read_success = true;
        SoapySDR_logf(SOAPY_SDR_INFO, "Hardware version %d.%d",
            info.product_ver >> 8, info.product_ver & 0xFF);
    } catch(std::exception &e) {
        SoapySDR_logf(SOAPY_SDR_WARNING, "Could not read HAT ID. Assuming hardware version %d.%d",
            info.product_ver >> 8, info.product_ver & 0xFF);
    }
    if (info.product_id != product_id_expected) {
        SoapySDR_logf(SOAPY_SDR_WARNING, "Unexpected product ID 0x%04x. Are you sure the correct HAT is connected?", info.product_id);
    }
    return info;
}

// Convert raw received samples to CF32.
// TODO: Support other formats and add format as a parameter.
static inline void convert_rx_buffer(
    const void *src,
    size_t src_offset,
    void *dest,
    size_t dest_offset,
    size_t length,
    float dc_i,
    float dc_q,
    std::complex<float> iq_coeff
)
{
    const int32_t *src_ = (const int32_t*)src + src_offset*2;
    float *dest_ = (float*)dest + dest_offset*2;
    const float scaling = 1.0f / 0x80000000L;
    const float coeff_i = iq_coeff.real();
    const float coeff_q = iq_coeff.imag();

    for (size_t i = 0; i < length*2; i += 2)
    {
        const float in_i = scaling * (float)src_[i] - dc_i;
        const float in_q = scaling * (float)src_[i + 1] - dc_q;

        // Correct image leakage as y = x + c * conj(x).
        dest_[i] = in_i + coeff_i * in_i + coeff_q * in_q;
        dest_[i + 1] = in_q + coeff_q * in_i - coeff_i * in_q;
    }
}

// Convert CF32 to raw transmit samples.
// TODO: Support other formats and add format as a parameter.
static inline void convert_tx_buffer(const void *src, size_t src_offset, void *dest, size_t dest_offset, size_t length, float tx_threshold2)
{
    const float *src_ = (const float*)src + src_offset*2;
    int32_t *dest_ = (int32_t*)dest + dest_offset*2;
    const float scaling = (float)0x7FFFFFFFL;
    for (size_t i = 0; i < length*2; i+=2)
    {
        float fi = src_[i], fq = src_[i+1];
        int32_t vi = scaling * std::max(std::min(fi, 1.0f), -1.0f);
        int32_t vq = scaling * std::max(std::min(fq, 1.0f), -1.0f);
        // Second lowest bit of each "I" sample controls RX/TX switching.
        // Set the lowest bit to the same value just in case.
        // Let's also reserve the 2 lowest bits of "Q" samples
        // for future extensions and keep them as 0.
        vi &= 0xFFFFFFFCL;
        vq &= 0xFFFFFFFCL;
        if (fi*fi + fq*fq >= tx_threshold2)
            vi |= 0b11L;
        dest_[i  ] = vi;
        dest_[i+1] = vq;
    }
}

#define MAX_REGS 0x80

#define SX1255_REG_TXFE2 0x0A
#define SX1255_REG_TXFE3 0x0B
#define SX1255_REG_RXFE2 0x0D
#define SX1255_REG_RXFE3 0x0E
#define SX1255_TEMP_DEFAULT_REF_C 25.0
#define SX1255_TEMP_DEFAULT_REF_RAW 150.0
#define SX1255_TEMP_DEFAULT_SAMPLES 1024
#define SX1255_TEMP_DEFAULT_SHIFT 23
#define SX1255_TEMP_DISCARD_SAMPLES 256
#define SX1255_TEMP_SETTLE_MS 20
#define SX1255_TEMP_MAX_SATURATED_FRACTION 0.05
#define SX1255_TEMP_MAX_ATTEMPTS 6
#define SX1255_TEMP_MIN_COHERENT_ATTEMPTS 2
#define SX1255_TEMP_MAX_COHERENT_SPAN_RAW 8.0
#define SX1255_TEMP_MIN_VALID_RAW 8.0
#define SX1255_TEMP_MAX_VALID_RAW 255.0
#define SX1255_TEMP_CACHE_MS 250

// Number of initial register values for SX1255.
// Only write the documented registers from 0x00 to 0x13.
#define N_INIT_REGISTERS 0x14
// Initial register values for SX1255.
static const uint8_t init_registers[N_INIT_REGISTERS] = {
    // 0x00: default value from datasheet (enable oscillator)
    0b00000001,
    // 0x01-0x06: RX and TX frequencies: 433.92 MHz
    0xD8, 0xF5, 0xC3, 0xD8, 0xF5, 0xC3,
    // 0x07: read-only register, written value does not matter
    0x11,
    // 0x08: TX gains, default value from datasheet
    0b00101110,
    // 0x09: default value from datasheet
    0b00100100,
    // 0x0A: default value from datasheet
    0b00110000,
    // 0x0B: default value from datasheet
    0b00000010,
    // 0x0C: RX gains, default value from datasheet
    0b00111111,
    // 0x0D: RX filters, make them narrow
    // ADCTRIM value of 7 minimized ADC spurs
    // but maybe 6 worked better with 38.4 MHz clock in some cases.
    0b00111011,
    // 0x0E: default value from datasheet
    0b00000110,
    // 0x0F: IO_MAP, default value from datasheet
    0b00000000,
    // 0x10, CK_SEL, default value from datasheet
    0b00000010,
    // 0x11: read-only register(?), written value does not matter
    0b00000000,
    // 0x12-0x13: I2S at 125 kHz: CLKOUT divider 4, decimate by 256
    0b00100010, 0b00101100,
};


// Register values for a given sample rate
struct sampleRateRegs {
    // Ratio of reference clock to sample rate
    uint16_t div;
    // iism_clk_div value (register 0x12 bits 3-0)
    uint8_t clkout : 4;
    // int_dec_mantisse value (register 0x13 bit 7)
    uint8_t mant : 1;
    // int_dec_m_parameter value (register 0x13 bit 6)
    uint8_t m : 1;
    // int_dec_n_parameter value (register 0x13 bits 5-3)
    uint8_t n : 3;
};

#define N_SAMPLE_RATES 6

// Register values for different sample rates
static const struct sampleRateRegs sample_rates[N_SAMPLE_RATES] = {
    {1536, 0b0110, 0, 1, 6 },
    { 768, 0b0100, 0, 1, 5 },
    { 512, 0b0011, 0, 0, 6 },
    //{ 384, 0b0011, 0, 1, 4 }, // 24 bit samples (did not work correctly)
    { 256, 0b0010, 0, 0, 5 },
    //{ 192, 0b0010, 0, 1, 3 }, // 24 bit samples (did not work correctly)
    { 128, 0b0001, 0, 0, 4 },
    //{  96, 0b0001, 0, 1, 2 }, // 24 bit samples (did not work correctly)
    {  64, 0b0000, 0, 0, 3 },
    //{  48, 0b0000, 0, 1, 1 }, // 24 bit samples (did not work correctly)
    //{  32, 0b0000, 0, 0, 2 }, // 16 bit samples (did not work)
};


// Class to use SPI through the SPI userspace API (SPIDEV) in Linux.
// Putting it in a separate class helps use RAII to ensure
// the file descriptor gets closed in all situations.
class Spi {
private:
    // SPIDEV file descriptor
    int fd;

public:
    Spi(const char *spidev_path)
    {
        assert(spidev_path != NULL);
        fd = open(spidev_path, O_RDWR);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "SPIDEV opened: %d", fd);
        if (fd < 0) {
            // TODO: add more detailed error messages from strerror or something
            throw std::runtime_error("Failed to open SPI");
        }
    }

    ~Spi()
    {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Closing SPIDEV fd %d", fd);
        close(fd);
    }

    int transfer(const uint8_t *tx_data, uint8_t *rx_data, const size_t len) const
    {
        struct spi_ioc_transfer transfer;
        memset(&transfer, 0, sizeof(transfer));

        transfer.tx_buf = (__u64)tx_data;
        transfer.rx_buf = (__u64)rx_data;
        transfer.len = (__u32)len;
        transfer.speed_hz = 10000000;

        int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "SPIDEV ioctl: %d", ret);

        if ((size_t)ret != len)
            throw std::runtime_error("SPI transfer failed");

        return ret;
    }

    int transfer(const std::vector<uint8_t> &tx_data, std::vector<uint8_t> &rx_data) const
    {
        // Make sure data fits in both buffers
        const size_t transfer_len = std::min(tx_data.size(), rx_data.size());
        return transfer(tx_data.data(), rx_data.data(), transfer_len);
    }
};



// Similar to class Spi, but for GPIO pins.
class GpioChip {
public:
    int chip_fd;

    GpioChip(const char *gpiochip_path)
    {
        assert(gpiochip_path != NULL);
        chip_fd = open(gpiochip_path, O_RDWR);
        if (chip_fd < 0) {
            throw std::runtime_error("Failed to open GPIO");
        }
    }

    ~GpioChip()
    {
        close(chip_fd);
    }
};

class GpioLine {
public:
    int line_fd;

    GpioLine(class GpioChip &chip, __u32 pin_number, const char *name, __u64 flags, bool initial_value)
    {
        /*
        struct gpio_v2_line_request req = {
            .offsets = { pin_number },
            .consumer = "",
            .config = (struct gpio_v2_line_config) {
                .flags = flags,
                .num_attrs = 0,
                .padding = { },
                .attrs = { },
            },
            .num_lines = 1,
            .event_buffer_size = 0,
            .padding = { },
            .fd = 0,
        };
        */
        struct gpio_v2_line_request req;
        ::memset(&req, 0, sizeof(req));
        req.offsets[0]   = pin_number;
        req.config.flags = flags;
        req.num_lines    = 1;

        strncpy(req.consumer, name, GPIO_MAX_NAME_SIZE-1);

        int ret = ioctl(chip.chip_fd, GPIO_V2_GET_LINE_IOCTL, &req);
        if (ret < 0) {
            throw std::runtime_error("Failed to request GPIO line");
        }

        line_fd = req.fd;

        set_value(initial_value);
    }

    ~GpioLine()
    {
        close(line_fd);
    }

    void set_value(bool value)
    {
        struct gpio_v2_line_values val = {
            .bits = value,
            .mask = 1,
        };
        int ret = ioctl(line_fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &val);
        if (ret < 0) {
            throw std::runtime_error("Failed to write GPIO line");
        }
    }
};



// Convert an ALSA recording error to a corresponding SoapySDR readStream error.
static int alsa_error_to_soapy_rx(int alsa_error_code)
{
    if (alsa_error_code == -EPIPE) {
        // Overrun
        return SOAPY_SDR_OVERFLOW;
    } else {
        // Some other error
        return SOAPY_SDR_STREAM_ERROR;
    }
}

// Convert an ALSA playback error to a corresponding SoapySDR writeStream error.
static int alsa_error_to_soapy_tx(int alsa_error_code)
{
    if (alsa_error_code == -EPIPE) {
        // Underrun
        return SOAPY_SDR_UNDERFLOW;
    } else {
        // Some other error
        return SOAPY_SDR_STREAM_ERROR;
    }
}


#define ALSACHECK(a) do {\
int retcheck = (a); \
if (retcheck < 0) { \
SoapySDR_logf(SOAPY_SDR_ERROR, "\nALSA error in %s: %s\n", #a, snd_strerror(retcheck)); \
goto alsa_error; } } while(0)

class AlsaPcm {
public:
    const char *name;
    snd_pcm_t *pcm;
    mutable std::mutex mutex;
    snd_pcm_stream_t dir;
    enum stream_mode stream_mode;
    bool setup_done;
    bool activated;
    int64_t position;
    snd_pcm_uframes_t hwp_period_size;
    snd_pcm_uframes_t hwp_buffer_size;

    AlsaPcm(const char *name, snd_pcm_stream_t dir):
        name(name),
        pcm(NULL),
        dir(dir),
        stream_mode(STREAM_MODE_NORMAL),
        setup_done(0),
        activated(0),
        position(0),
        hwp_period_size(0),
        hwp_buffer_size(0)
    {
    }

    void open(void)
    {
        ALSACHECK(snd_pcm_open(&pcm, name, dir, 0));
        return;

        alsa_error:
        if (pcm != NULL)
            snd_pcm_close(pcm);

        // TODO more detailed error messages?
        throw std::runtime_error("Error opening ALSA device");
    }

    ~AlsaPcm()
    {
        if (pcm != NULL)
            snd_pcm_close(pcm);
    }

    bool is_tx(void)
    {
        return dir == SND_PCM_STREAM_PLAYBACK;
    }

    int reset()
    {
        int ret = 0;
        // Make sure stream is stopped. If it was stopped already,
        // drop will fail but that is fine, so do not check the return value.
        snd_pcm_drop(pcm);

        ret = snd_pcm_prepare(pcm);
        if (ret < 0)
            return ret;
        position = 0;
        ret = snd_pcm_reset(pcm);
        return ret;
    }

    void configure(snd_pcm_uframes_t period)
    {
        if (pcm == NULL)
            return;
        snd_pcm_hw_params_t *hwp = NULL;
        snd_pcm_sw_params_t *swp = NULL;

        unsigned int hwp_periods = 0;

        // Period size basically determines the I2S DMA interrupt rate.
        // A higher period size somewhat reduces CPU use
        // but increases the minimum latency.
        // If an application reads samples in fixed size blocks,
        // it is usually best to use a period a size equal to the block size
        // to minimize latency while avoiding unnecessary CPU use.
        // Period size can configured using the "period" stream argument.
        // Use default value if 0.
        hwp_period_size = period > 0 ? period : 256;

        // Based on some tests requesting various period and buffer sizes,
        // it seems like, on a Raspberry Pi:
        // * Maximum buffer size is 65536.
        // * Buffer size should be a multiple of period size.
        //   Otherwise ALSA forces period size to be a submultiple of buffer size.
        // * There should be at least 2 periods per buffer, but we do not really need
        //   to check for this. ALSA handles it nicely.
        // * Minimum period size is 2 but this does not really need
        //   a separate check either.
        // Given these constraints, find the maximum possible buffer size
        // to achieve the period requested.
        const snd_pcm_uframes_t max_buffer_size = 65536;
        hwp_period_size = std::min(hwp_period_size, max_buffer_size);
        hwp_buffer_size = max_buffer_size / hwp_period_size * hwp_period_size;


        snd_pcm_uframes_t swp_boundary = 0;

        ALSACHECK(snd_pcm_hw_params_malloc(&hwp));
        ALSACHECK(snd_pcm_hw_params_any(pcm, hwp));
        ALSACHECK(snd_pcm_hw_params_set_access(pcm, hwp, SND_PCM_ACCESS_RW_INTERLEAVED));
        ALSACHECK(snd_pcm_hw_params_set_format(pcm, hwp, SND_PCM_FORMAT_S32_LE));
        // Sample rate given to ALSA does not affect the actual sample rate,
        // so just use a fixed "dummy" value.
        ALSACHECK(snd_pcm_hw_params_set_rate(pcm, hwp, 192000, 0));
        ALSACHECK(snd_pcm_hw_params_set_channels(pcm, hwp, 2));
        ALSACHECK(snd_pcm_hw_params_set_buffer_size_near(pcm, hwp, &hwp_buffer_size));
        ALSACHECK(snd_pcm_hw_params_set_period_size_near(pcm, hwp, &hwp_period_size, 0));
        ALSACHECK(snd_pcm_hw_params_get_periods(hwp, &hwp_periods, 0));

        ALSACHECK(snd_pcm_hw_params(pcm, hwp));
        snd_pcm_hw_params_free(hwp);

        SoapySDR_logf(SOAPY_SDR_DEBUG, "ALSA parameters: buffer_size=%d, period_size=%d, periods=%d", hwp_buffer_size, hwp_period_size, hwp_periods);

        ALSACHECK(snd_pcm_sw_params_malloc(&swp));
        ALSACHECK(snd_pcm_sw_params_current(pcm, swp));
        ALSACHECK(snd_pcm_sw_params_get_boundary(swp, &swp_boundary));
        SoapySDR_logf(SOAPY_SDR_DEBUG, "ALSA SW parameters: boundary=%lu", swp_boundary);
        if (stream_mode == STREAM_MODE_NORMAL) {
            ALSACHECK(snd_pcm_sw_params_set_stop_threshold(pcm, swp, swp_boundary));
            // https://stackoverflow.com/a/20515251
            ALSACHECK(snd_pcm_sw_params_set_silence_threshold(pcm, swp, 0));
            ALSACHECK(snd_pcm_sw_params_set_silence_size(pcm, swp, swp_boundary));
        } else {
            ALSACHECK(snd_pcm_sw_params_set_stop_threshold(pcm, swp, hwp_buffer_size));
            ALSACHECK(snd_pcm_sw_params_set_silence_threshold(pcm, swp, 0));
            ALSACHECK(snd_pcm_sw_params_set_silence_size(pcm, swp, 0));
        }

        ALSACHECK(snd_pcm_sw_params(pcm, swp));
        snd_pcm_sw_params_free(swp);

        ALSACHECK(reset());

        return;

    alsa_error:
        if (hwp != NULL)
            snd_pcm_hw_params_free(hwp);
        if (swp != NULL)
            snd_pcm_sw_params_free(swp);
        // TODO more detailed error messages?
        throw std::runtime_error("Error configuring ALSA device");
    }
};


/***********************************************************************
 * Device interface
 **********************************************************************/
class SoapySX : public SoapySDR::Device
{
private:
    struct temperature_measurement {
        double iout_mean;
        double qout_mean;
        double i_raw_code;
        double q_raw_code;
        double raw_code;
        char channel;
        size_t samples;
        size_t i_samples;
        size_t q_samples;
        size_t i_saturated;
        size_t q_saturated;
        size_t accepted_attempt;
        std::chrono::steady_clock::time_point measured_at;
    };

    double masterClock;
    double sampleRate;

    // Mutex for anything involving SX1255 registers
    // to avoid problems if an application calls methods from multiple threads.
    mutable std::recursive_mutex reg_mutex;

    Spi spi;
    GpioChip gpio;
    GpioLine gpio_reset, gpio_rx, gpio_tx;
    AlsaPcm alsa_rx;
    AlsaPcm alsa_tx;

    // Transmitter is turned on when squared magnitude of a TX sample
    // exceeds this threshold.
    float tx_threshold2;
    // If true, RX and TX streams have been linked using snd_pcm_link
    bool linked;
    // Temperature conversion calibration:
    // datasheet output is approximately -1 LSB/C, with 150d typical at ambient.
    double temperature_ref_c;
    double temperature_ref_raw;
    unsigned temperature_samples;
    unsigned temperature_shift;
    mutable std::mutex temperature_mutex;
    mutable bool temperature_cache_valid;
    mutable temperature_measurement temperature_cache;
    std::string pa_mode;
    mutable std::mutex rx_correction_mutex;
    float rx_dc_i;
    float rx_dc_q;
    std::complex<float> rx_iq_balance;

    // Values of registers (to be) written to the chip.
    // Storing them here makes it easier and faster to change
    // specific bits since they do not need to be read
    // from the chip every time.
    uint8_t regs[MAX_REGS];

    // Buffer for RX samples before type conversion.
    // Data is not actually uint64_t but it simplifies code a bit
    // by making one vector element correspond to one 32+32 bit complex sample.
    std::vector<uint64_t> buffer_rx;
    // Buffer for TX samples after type conversion.
    std::vector<uint64_t> buffer_tx;

    struct hat_info hat_info;

    // Convert a SoapySDR nanosecond timestamp to a sample counter.
    int64_t timestamp_to_samples(long long timestamp) const
    {
        return SoapySDR::timeNsToTicks(timestamp, sampleRate);
    }

    // Convert a sample counter to a SoapySDR nanosecond timestamp.
    long long samples_to_timestamp(int64_t samples) const
    {
        return SoapySDR::ticksToTimeNs(samples, sampleRate);
    }

    // Set given bits of a register.
    // The registers are not actually written to the chip.
    void set_register_bits(size_t address, unsigned lowestbit, unsigned nbits, unsigned value)
    {
        if (address >= MAX_REGS)
            throw std::runtime_error("Invalid register address");
        unsigned mask = ((1 << nbits) - 1) << lowestbit;
        regs[address] = (regs[address] & (~mask)) | ((value << lowestbit) & mask);
    }

    // Get given bits of a cached register.
    // The registers are not actually read from the chip.
    unsigned get_cached_register_bits(size_t address, unsigned lowestbit, unsigned nbits) const
    {
        if (address >= MAX_REGS)
            throw std::runtime_error("Invalid register address");
        unsigned mask = ((1 << nbits) - 1) << lowestbit;
        return (regs[address] & mask) >> lowestbit;
    }

    // Write a range of registers to the chip.
    void write_registers_to_chip(size_t firstreg, size_t nregs)
    {
        if ((firstreg >= MAX_REGS) || (nregs > MAX_REGS) || (firstreg > MAX_REGS - nregs))
            throw std::runtime_error("Invalid register address");

        size_t transfer_len = nregs + 1;
        // Buffer for SPI transfer
        std::vector<uint8_t> buf(transfer_len, 0);

        buf[0] = firstreg | 0x80;
        for (size_t i = 1; i < transfer_len; i++)
            buf[i] = regs[firstreg + i - 1];

        spi.transfer(buf, buf);
    }

    void reset_chip(void)
    {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Resetting chip");
        // Timing from datasheet Figure 6-2: Manual Reset Timing Diagram
        gpio_reset.set_value(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        gpio_reset.set_value(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    void init_chip(void)
    {
        for (size_t i = 0; i < N_INIT_REGISTERS; i++)
            set_register_bits(i, 0, 8, init_registers[i]);
        // Enable RX and TX, just for initial testing. This should be done somewhere else.
        set_register_bits(0, 1, 3, 0b111);
        write_registers_to_chip(0, N_INIT_REGISTERS);
    }

    bool does_synth_tune(double frequency)
    {
        setFrequency(SOAPY_SDR_RX, 0, frequency, {});
        setFrequency(SOAPY_SDR_TX, 0, frequency, {});
        // Give some time for PLL to lock
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        auto status_register = readRegister("", 0x11);
        return (status_register & 3) == 3;
    }

    void detect_clock(void)
    {
        // Try to detect whether SX1255 clock is 32 MHz or 38.4 MHz
        // by tuning near edges of tuning range.
        // First assume it is 32 MHz and try tuning accordingly.
        masterClock = 32.0e6;
        // If clock is 38.4 MHz instead, 510 MHz ends up at 612 MHz
        // where synthesizers (hopefully) do not lock anymore.
        bool tunes_high = does_synth_tune(510e6);
        // Synthesizers do not (hopefully) go down to 330 MHz, but
        // if clock is 38.4 MHz instead, they end up at 396 MHz.
        bool tunes_low  = does_synth_tune(330e6);
        if (tunes_low && (!tunes_high)) {
            SoapySDR_logf(SOAPY_SDR_INFO, "Detected clock as 38.4 MHz");
            masterClock = 38.4e6;
        } else if (tunes_high && (!tunes_low)) {
            SoapySDR_logf(SOAPY_SDR_INFO, "Detected clock as 32.0 MHz");
        } else {
            SoapySDR_logf(SOAPY_SDR_INFO, "Clock detection failed, assuming 38.4 MHz");
            masterClock = 38.4e6;
        }

        // Update default values for new masterClock value
        sampleRate = masterClock / 256.0;
        setFrequency(SOAPY_SDR_RX, 0, 433.92e6, {});
        setFrequency(SOAPY_SDR_TX, 0, 433.92e6, {});
    }

    double raw_temperature_to_c(double raw_code) const
    {
        // SX1255 reports approximately -1 LSB/C. The default reference uses
        // the datasheet typical ambient value, but operators can override both
        // sides with TEMP_REF_C and TEMP_REF_RAW after a hardware measurement.
        return temperature_ref_c + (temperature_ref_raw - raw_code);
    }

    temperature_measurement measure_temperature_adc(void) const
    {
        auto *self = const_cast<SoapySX *>(this);
        std::scoped_lock lock(self->alsa_rx.mutex, self->alsa_tx.mutex, self->reg_mutex);

        if (self->alsa_rx.activated || self->alsa_tx.activated) {
            throw std::runtime_error("SX1255 temperature sensor requires inactive RX/TX streams");
        }
        if (self->alsa_rx.pcm == NULL) {
            throw std::runtime_error("RX ALSA PCM is not open");
        }

        if (!self->alsa_rx.setup_done) {
            self->alsa_rx.configure(temperature_samples);
        }

        const uint8_t original_opmode = self->regs[0x00];
        const uint8_t original_rxfe3 = self->regs[SX1255_REG_RXFE3];

        // Temperature uses the RX ADC path. Keep RX/I2S alive, but force TX
        // and PA off so the internal measurement is not polluted by TX leakage.
        self->set_register_bits(0x00, 1, 3, 0b001);
        self->write_registers_to_chip(0x00, 1);
        self->set_register_bits(SX1255_REG_RXFE3, 0, 1, 1);
        self->write_registers_to_chip(SX1255_REG_RXFE3, 1);

        std::this_thread::sleep_for(std::chrono::milliseconds(SX1255_TEMP_SETTLE_MS));

        const size_t discard_samples = std::min((size_t)SX1255_TEMP_DISCARD_SAMPLES, (size_t)temperature_samples);
        const size_t total_samples = (size_t)temperature_samples + discard_samples;
        std::vector<int32_t> temp_buffer(total_samples * 2, 0);
        std::vector<temperature_measurement> attempts;

        try {
            std::string last_error = "No temperature measurement attempted";
            for (size_t attempt = 1; attempt <= SX1255_TEMP_MAX_ATTEMPTS; attempt++) {
                std::fill(temp_buffer.begin(), temp_buffer.end(), 0);
                size_t read_offset = 0;

                if (self->alsa_rx.reset() < 0)
                    throw std::runtime_error("Failed to reset RX PCM for temperature measurement");
                int ret = snd_pcm_start(self->alsa_rx.pcm);
                if (ret < 0)
                    throw std::runtime_error("Failed to start RX PCM for temperature measurement");

                while (read_offset < total_samples) {
                    ret = snd_pcm_wait(self->alsa_rx.pcm, 1000);
                    if (ret <= 0) {
                        last_error = "Timed out waiting for temperature samples";
                        break;
                    }

                    snd_pcm_sframes_t samples_read = snd_pcm_readi(
                        self->alsa_rx.pcm,
                        temp_buffer.data() + read_offset * 2,
                        total_samples - read_offset);
                    if (samples_read < 0) {
                        last_error = "Failed to read RX PCM temperature samples";
                        break;
                    }
                    if (samples_read == 0) {
                        last_error = "RX PCM returned no temperature samples";
                        break;
                    }
                    read_offset += (size_t)samples_read;
                }

                if (read_offset < total_samples) {
                    self->alsa_rx.reset();
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }

                double i_sum = 0.0;
                double q_sum = 0.0;
                size_t i_count = 0;
                size_t q_count = 0;
                size_t i_saturated = 0;
                size_t q_saturated = 0;

                for (size_t i = discard_samples; i < read_offset; i++) {
                    const int32_t iv = temp_buffer[i * 2];
                    const int32_t q = temp_buffer[i * 2 + 1];
                    if (iv == INT32_MIN || iv == INT32_MAX) {
                        i_saturated++;
                    } else {
                        i_sum += (double)iv;
                        i_count++;
                    }
                    if (q == INT32_MIN || q == INT32_MAX) {
                        q_saturated++;
                    } else {
                        q_sum += (double)q;
                        q_count++;
                    }
                }

                const double iout_mean = i_count > 0 ? i_sum / (double)i_count : NAN;
                const double qout_mean = q_count > 0 ? q_sum / (double)q_count : NAN;
                const double i_raw_candidate = std::abs(iout_mean) / (double)(1ULL << temperature_shift);
                const double q_raw_candidate = std::abs(qout_mean) / (double)(1ULL << temperature_shift);
                const size_t measurement_samples = read_offset - discard_samples;
                const double i_saturated_fraction = (double)i_saturated / (double)measurement_samples;
                const double q_saturated_fraction = (double)q_saturated / (double)measurement_samples;
                const bool i_valid =
                    i_count > 0 &&
                    i_saturated_fraction <= SX1255_TEMP_MAX_SATURATED_FRACTION &&
                    i_raw_candidate >= SX1255_TEMP_MIN_VALID_RAW &&
                    i_raw_candidate <= SX1255_TEMP_MAX_VALID_RAW;
                const bool q_valid =
                    q_count > 0 &&
                    q_saturated_fraction <= SX1255_TEMP_MAX_SATURATED_FRACTION &&
                    q_raw_candidate >= SX1255_TEMP_MIN_VALID_RAW &&
                    q_raw_candidate <= SX1255_TEMP_MAX_VALID_RAW;
                const double i_raw_code = i_valid ? i_raw_candidate : NAN;
                const double q_raw_code = q_valid ? q_raw_candidate : NAN;

                if (!i_valid && !q_valid) {
                    last_error = "No coherent non-saturated temperature channel available";
                    self->alsa_rx.reset();
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }

                SoapySDR_logf(
                    SOAPY_SDR_INFO,
                    "SX1255 temperature attempt %zu: I mean=%.3f raw=%.3f sat=%zu/%zu Q mean=%.3f raw=%.3f sat=%zu/%zu",
                    attempt,
                    iout_mean,
                    i_raw_code,
                    i_saturated,
                    measurement_samples,
                    qout_mean,
                    q_raw_code,
                    q_saturated,
                    measurement_samples);

                const bool prefer_q = q_valid && (!i_valid || q_raw_code >= i_raw_code);
                attempts.push_back({
                    iout_mean,
                    qout_mean,
                    i_raw_code,
                    q_raw_code,
                    prefer_q ? q_raw_code : i_raw_code,
                    prefer_q ? 'Q' : 'I',
                    prefer_q ? q_count : i_count,
                    i_count,
                    q_count,
                    i_saturated,
                    q_saturated,
                    attempt,
                    std::chrono::steady_clock::now()
                });
            }

            if (attempts.empty())
                throw std::runtime_error(last_error);

            if (self->alsa_rx.reset() < 0)
                throw std::runtime_error("Failed to reset RX PCM after temperature measurement");
        } catch (...) {
            self->set_register_bits(0x00, 0, 8, original_opmode);
            self->write_registers_to_chip(0x00, 1);
            self->set_register_bits(SX1255_REG_RXFE3, 0, 8, original_rxfe3);
            self->write_registers_to_chip(SX1255_REG_RXFE3, 1);
            self->alsa_rx.reset();
            throw;
        }

        self->set_register_bits(0x00, 0, 8, original_opmode);
        self->write_registers_to_chip(0x00, 1);
        self->set_register_bits(SX1255_REG_RXFE3, 0, 8, original_rxfe3);
        self->write_registers_to_chip(SX1255_REG_RXFE3, 1);

        auto coherent = [](std::vector<double> raw_codes) {
            if (raw_codes.size() < SX1255_TEMP_MIN_COHERENT_ATTEMPTS)
                return std::pair<bool, double>(false, NAN);
            std::sort(raw_codes.begin(), raw_codes.end());
            const double span = raw_codes.back() - raw_codes.front();
            const double median = raw_codes[raw_codes.size() / 2];
            return std::pair<bool, double>(span <= SX1255_TEMP_MAX_COHERENT_SPAN_RAW, median);
        };

        std::vector<double> i_raw_codes;
        std::vector<double> q_raw_codes;
        for (const auto &attempt : attempts) {
            if (!std::isnan(attempt.i_raw_code))
                i_raw_codes.push_back(attempt.i_raw_code);
            if (!std::isnan(attempt.q_raw_code))
                q_raw_codes.push_back(attempt.q_raw_code);
        }
        const auto i_coherent = coherent(i_raw_codes);
        const auto q_coherent = coherent(q_raw_codes);
        if (!i_coherent.first && !q_coherent.first)
            throw std::runtime_error("Temperature attempts were not coherent");

        const bool use_q = q_coherent.first && (!i_coherent.first || q_coherent.second >= i_coherent.second);
        const double selected_raw = use_q ? q_coherent.second : i_coherent.second;
        temperature_measurement selected = attempts.back();
        for (const auto &attempt : attempts) {
            const double attempt_raw = use_q ? attempt.q_raw_code : attempt.i_raw_code;
            if (!std::isnan(attempt_raw) && std::abs(attempt_raw - selected_raw) <= SX1255_TEMP_MAX_COHERENT_SPAN_RAW) {
                selected = attempt;
                break;
            }
        }
        selected.raw_code = selected_raw;
        selected.channel = use_q ? 'Q' : 'I';
        selected.samples = use_q ? selected.q_samples : selected.i_samples;
        selected.measured_at = std::chrono::steady_clock::now();

        SoapySDR_logf(
            SOAPY_SDR_INFO,
            "SX1255 temperature selected: channel=%c raw=%.3f temp=%.3f C attempts=%zu I_coherent=%d Q_coherent=%d samples=%zu discarded=%zu I_sat=%zu Q_sat=%zu source_attempt=%zu",
            selected.channel,
            selected.raw_code,
            raw_temperature_to_c(selected.raw_code),
            attempts.size(),
            i_coherent.first ? 1 : 0,
            q_coherent.first ? 1 : 0,
            selected.samples,
            discard_samples,
            selected.i_saturated,
            selected.q_saturated,
            selected.accepted_attempt);

        return selected;
    }

    void set_filter_profile(const std::string &profile)
    {
        if (profile == "TETRA_CLEAN") {
            // Keep hardware filters much wider than a 25 kHz TETRA channel.
            // FlowStation's DSP defines the occupied channel; these settings
            // reduce wideband TX DAC/image energy without live gain changes.
            set_register_bits(SX1255_REG_TXFE2, 0, 5, 0);
            set_register_bits(SX1255_REG_TXFE3, 0, 3, 5);
            set_register_bits(SX1255_REG_RXFE2, 5, 3, 1);
            set_register_bits(SX1255_REG_RXFE2, 2, 3, 6);
            set_register_bits(SX1255_REG_RXFE2, 0, 2, 3);
            write_registers_to_chip(SX1255_REG_TXFE2, 2);
            write_registers_to_chip(SX1255_REG_RXFE2, 1);
        } else if (profile == "LEGACY") {
            set_register_bits(SX1255_REG_TXFE2, 0, 5, 16);
            set_register_bits(SX1255_REG_TXFE3, 0, 3, 2);
            set_register_bits(SX1255_REG_RXFE2, 5, 3, 1);
            set_register_bits(SX1255_REG_RXFE2, 2, 3, 6);
            set_register_bits(SX1255_REG_RXFE2, 0, 2, 3);
            write_registers_to_chip(SX1255_REG_TXFE2, 2);
            write_registers_to_chip(SX1255_REG_RXFE2, 1);
        } else {
            throw std::runtime_error("Unsupported RF profile");
        }
    }

    std::string get_filter_profile(void) const
    {
        const unsigned tx_filter_bw = get_cached_register_bits(SX1255_REG_TXFE2, 0, 5);
        const unsigned tx_dac_bw = get_cached_register_bits(SX1255_REG_TXFE3, 0, 3);
        const unsigned rx_adc_bw = get_cached_register_bits(SX1255_REG_RXFE2, 5, 3);
        const unsigned rx_adc_trim = get_cached_register_bits(SX1255_REG_RXFE2, 2, 3);
        const unsigned rx_pga_bw = get_cached_register_bits(SX1255_REG_RXFE2, 0, 2);

        if (tx_filter_bw == 0 && tx_dac_bw == 5 && rx_adc_bw == 1 && rx_adc_trim == 6 && rx_pga_bw == 3)
            return "TETRA_CLEAN";
        if (tx_filter_bw == 16 && tx_dac_bw == 2 && rx_adc_bw == 1 && rx_adc_trim == 6 && rx_pga_bw == 3)
            return "LEGACY";
        return "CUSTOM";
    }

    void validate_rx_correction_channel(const int direction, const size_t channel) const
    {
        if (direction != SOAPY_SDR_RX) {
            throw std::runtime_error("RX correction is only supported for RX direction");
        }
        if (channel != 0) {
            throw std::runtime_error("Invalid RX correction channel");
        }
    }

/***********************************************************************
 * Initialization and destruction
 **********************************************************************/

public:
    SoapySX(const SoapySDR::Kwargs &args, struct hat_info hat_info):
        masterClock(32.0e6),
        sampleRate(125.0e3),

        spi("/dev/spidev0.0"),

        gpio("/dev/gpiochip0"),
        gpio_reset(gpio,
            5,
            "SX reset",
            GPIO_V2_LINE_FLAG_OUTPUT | GPIO_V2_LINE_FLAG_OPEN_SOURCE,
            0
        ),
        gpio_rx(gpio,
            hat_info.product_ver == 0x0100 ? 13 : 23,
            "SX RX",
            GPIO_V2_LINE_FLAG_OUTPUT,
            1
        ),
        gpio_tx(gpio,
            hat_info.product_ver == 0x0100 ? 12 : 22,
            "SX TX",
            GPIO_V2_LINE_FLAG_OUTPUT,
            1
        ),

        alsa_rx(AlsaPcm("hw:CARD=SX1255,DEV=1", SND_PCM_STREAM_CAPTURE)),
        alsa_tx(AlsaPcm("hw:CARD=SX1255,DEV=0", SND_PCM_STREAM_PLAYBACK)),
        tx_threshold2(0.0f),
        linked(false),
        temperature_ref_c(SX1255_TEMP_DEFAULT_REF_C),
        temperature_ref_raw(SX1255_TEMP_DEFAULT_REF_RAW),
        temperature_samples(SX1255_TEMP_DEFAULT_SAMPLES),
        temperature_shift(SX1255_TEMP_DEFAULT_SHIFT),
        temperature_cache_valid(false),
        temperature_cache{},
        pa_mode("AUTO"),
        rx_dc_i(0.0f),
        rx_dc_q(0.0f),
        rx_iq_balance(0.0f, 0.0f),
        regs{0},

        // Allocate reasonably large buffers by default.
        // Their size will be increased if needed.
        buffer_rx(8192),
        buffer_tx(8192),

        hat_info(hat_info)
    {
        (void)args;

        SoapySDR_logf(SOAPY_SDR_INFO, "Initializing SoapySX");

        reset_chip();
        init_chip();
        detect_clock();
        // Open ALSA devices now when I2S clocks are already running.
        // I am not sure if this makes any difference but just in case.
        alsa_rx.open();
        alsa_tx.open();
    }

    ~SoapySX(void)
    {
        SoapySDR_logf(SOAPY_SDR_INFO, "Uninitializing SoapySX");

        // Put SX1255 to sleep
        set_register_bits(0, 0, 4, 0);
        write_registers_to_chip(0, 1);

        // Make sure PA is turned off
        writeSetting("PA", "OFF");
    }

/***********************************************************************
 * Sample streams
 **********************************************************************/

    SoapySDR::Stream *setupStream(
        const int direction,
        const std::string & format,
        const std::vector<size_t> & channels,
        const SoapySDR::Kwargs & args
    )
    {
        (void)channels; // Only one channel
        (void)args; // Unused for now

        std::scoped_lock lock(alsa_rx.mutex, alsa_tx.mutex);

        if (format != "CF32")
            throw std::runtime_error("Only CF32 format is currently supported");
        if (
            (snd_pcm_state(alsa_rx.pcm) == SND_PCM_STATE_RUNNING) ||
            (snd_pcm_state(alsa_tx.pcm) == SND_PCM_STATE_RUNNING)
        )
            throw std::runtime_error("Streams can be setup only if none of the streams are running");

        auto *stream = direction == SOAPY_SDR_RX ? &alsa_rx : &alsa_tx;

        // Allow setting up only one stream per direction.
        if (stream->setup_done)
            throw std::runtime_error("Stream has been setup already");

        if (stream->is_tx()) {
            const float tx_threshold_default = 1.0e-3;
            // Enable TX when magnitude of a sample exceeds a given threshold.
            float tx_threshold =
                (args.count("threshold") > 0)
                ? std::stof(args.at("threshold"))
                : tx_threshold_default;
            tx_threshold2 = tx_threshold * tx_threshold;
        }

        bool arg_link = (args.count("link") > 0 && args.at("link") == "1");
        stream->stream_mode = arg_link ? STREAM_MODE_LINK : STREAM_MODE_NORMAL;

        stream->configure(args.count("period") > 0 ? std::stoul(args.at("period")) : 0);

        stream->setup_done = 1;

        // Always link RX and TX PCMs if both have been setup.
        if ((!linked) && alsa_rx.setup_done && alsa_tx.setup_done) {
            SoapySDR_logf(SOAPY_SDR_DEBUG, "Linking streams");
            ALSACHECK(snd_pcm_link(alsa_rx.pcm, alsa_tx.pcm));
            linked = 1;
        }

        return reinterpret_cast<SoapySDR::Stream *>(stream);
    alsa_error:
        // TODO more detailed error messages?
        throw std::runtime_error("ALSA error");
    }

    void closeStream(SoapySDR::Stream * handle)
    {
        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        std::scoped_lock lock(stream->mutex);
        stream->setup_done = 0;
    }

    int activateStream(
        SoapySDR::Stream * handle,
        const int flags,
        const long long timeNs,
        const size_t numElems
    )
    {
        (void)flags; (void)timeNs; (void)numElems;

        std::scoped_lock rx_lock(alsa_rx.mutex, alsa_tx.mutex);

        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        if (stream->activated) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "Stream was already activated");
            return SOAPY_SDR_STREAM_ERROR;
        }
        stream->activated = 1;

        if (stream->stream_mode == STREAM_MODE_NORMAL) {
            if (snd_pcm_state(stream->pcm) == SND_PCM_STATE_PREPARED) {
                ALSACHECK(snd_pcm_start(stream->pcm));
            }
        }

        return 0;
    alsa_error:
        return SOAPY_SDR_STREAM_ERROR;
    }

    int deactivateStream(
        SoapySDR::Stream * handle,
        const int flags,
        const long long timeNs
    )
    {
        (void)flags; (void)timeNs;

        std::scoped_lock rx_lock(alsa_rx.mutex, alsa_tx.mutex);

        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        if (!stream->activated) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "Stream was already deactivated");
            return SOAPY_SDR_STREAM_ERROR;
        }
        stream->activated = 0;

        // If both streams have been deactivated, stop them.
        if ((!alsa_rx.activated) && (!alsa_tx.activated)) {
            SoapySDR_logf(SOAPY_SDR_INFO, "Stopping and resetting streams");
            ALSACHECK(alsa_rx.reset());
            ALSACHECK(alsa_tx.reset());
        }

        return 0;
    alsa_error:
        return SOAPY_SDR_STREAM_ERROR;
    }

    size_t getStreamMTU(SoapySDR::Stream * handle) const
    {
        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        std::scoped_lock lock(stream->mutex);
        return stream->hwp_period_size;
    }

    int readStream(
        SoapySDR::Stream * handle,
        void *const * buffs,
        const size_t numElems,
        int & flags,
        long long & timeNs,
        const long timeoutUs
    )
    {
        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        std::scoped_lock lock(stream->mutex);

        flags = 0;

        if (stream->is_tx())
            throw std::runtime_error("Wrong direction");

        snd_pcm_t *pcm = stream->pcm;

        // If readStream is called before ALSA stream has been started,
        // it will get stuck forever in snd_pcm_wait.
        // Looks like this was the reason SoapyRemote did not work before.
        // Proper implementation of timeout might fix this too but that
        // might need replacing snd_pcm_wait with something more complicated.
        // For now, handle it by returning if the stream is not active.
        if (!stream->activated)
            return 0;


        snd_pcm_sframes_t pcm_avail = 0, pcm_delay = 0;
        int avail_ret = snd_pcm_avail_delay(pcm, &pcm_avail, &pcm_delay);
        if (avail_ret < 0) {
            // TODO: check if it can fail and think what to actually do here
            SoapySDR_logf(SOAPY_SDR_ERROR, "rx snd_pcm_avail_delay: %d", avail_ret);
            return alsa_error_to_soapy_rx(avail_ret);
        }
        SoapySDR_logf(SOAPY_SDR_DEBUG, "rx snd_pcm_avail_delay: %d %ld %ld", avail_ret, pcm_avail, pcm_delay);


        // If number of available samples is more than the ALSA buffer size,
        // it means the buffer has overrun and old samples have been overwritten.
        // Skip these samples.
        if (pcm_avail > (snd_pcm_sframes_t)stream->hwp_buffer_size) {
            snd_pcm_uframes_t overwritten = (snd_pcm_uframes_t)pcm_avail - stream->hwp_buffer_size;
            // Round the number of samples to skip to a multiple of period size,
            // so applications that use reads aligned to periods will stay aligned.
            // Add 1-2 extra periods for some margin.
            snd_pcm_uframes_t samples_to_skip = (overwritten / stream->hwp_period_size + 2) * stream->hwp_period_size;
            snd_pcm_sframes_t forwarded = snd_pcm_forward(pcm, samples_to_skip);
            if (forwarded >= 0) {
                stream->position += forwarded;
                pcm_avail -= forwarded;

                SoapySDR_logf(SOAPY_SDR_WARNING, "RX buffer overrun. Skipped %u samples", forwarded);
            } else {
                // TODO: check if it can fail and think what to actually do here
                SoapySDR_logf(SOAPY_SDR_ERROR, "rx snd_pcm_forward: %d", forwarded);
                return alsa_error_to_soapy_rx((int)forwarded);
            }
        }


        // Number of samples to read.
        // snd_pcm_uframes_t is unsigned long. Clamp to its maximum value just in case.
        snd_pcm_uframes_t length = (snd_pcm_uframes_t)std::min(numElems, (size_t)ULONG_MAX);

        if (timeoutUs <= 0) {
            // Hack to make non-blocking read work:
            // limit number of samples to what is available right now.
            if (pcm_avail <= 0) {
                length = 0;
            } else if ((snd_pcm_uframes_t)pcm_avail < length) {
                length = (snd_pcm_uframes_t)pcm_avail;
            }
        }

        if (length > buffer_rx.size())
            buffer_rx.resize(length);

        if (length > 0) {
            snd_pcm_sframes_t samples_read = snd_pcm_readi(pcm, buffer_rx.data(), length);
            if (samples_read >= 0) {
                timeNs = samples_to_timestamp(stream->position);
                flags |= SOAPY_SDR_HAS_TIME;

                stream->position += samples_read;

                assert((size_t)samples_read <= buffer_rx.size());
                assert((size_t)samples_read <= numElems);
                float dc_i;
                float dc_q;
                std::complex<float> iq_balance;
                {
                    std::scoped_lock correction_lock(rx_correction_mutex);
                    dc_i = rx_dc_i;
                    dc_q = rx_dc_q;
                    iq_balance = rx_iq_balance;
                }
                convert_rx_buffer(buffer_rx.data(), 0, buffs[0], 0, samples_read, dc_i, dc_q, iq_balance);

                return (int)samples_read;
            } else {
                return alsa_error_to_soapy_rx((int)samples_read);
            }
        } else {
            // Nothing to read
            return 0;
        }
    }

    int writeStream(
        SoapySDR::Stream * handle,
        const void *const * buffs,
        const size_t numElems,
        int & flags,
        long long timeNs,
        const long timeoutUs
    )
    {
        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        std::scoped_lock lock(stream->mutex);

        if (!stream->is_tx())
            throw std::runtime_error("Wrong direction");

        if (!stream->activated)
            return 0;

        snd_pcm_t *pcm = stream->pcm;

        snd_pcm_sframes_t pcm_avail = 0, pcm_delay = 0;
        int avail_ret = snd_pcm_avail_delay(pcm, &pcm_avail, &pcm_delay);
        if (avail_ret < 0) {
            // TODO: check if it can fail and think what to actually do here
            SoapySDR_logf(SOAPY_SDR_ERROR, "tx snd_pcm_avail_delay: %d", avail_ret);
            return alsa_error_to_soapy_tx(avail_ret);
        }
        SoapySDR_logf(SOAPY_SDR_DEBUG, "tx snd_pcm_avail_delay: %d %ld %ld", avail_ret, pcm_avail, pcm_delay);

        // Current position where stream is being "played",
        // similar to getHardwareTime:
        int64_t playback_position = stream->position - (int64_t)pcm_delay;

        // Position where samples are going to be written
        int64_t write_position;

        // Number of samples to write.
        // snd_pcm_uframes_t is unsigned long. Clamp to its maximum value just in case.
        snd_pcm_uframes_t length = (snd_pcm_uframes_t)std::min(numElems, (size_t)ULONG_MAX);

        if (flags & SOAPY_SDR_HAS_TIME) {
            // Timestamp provided.
            // Write to the position corresponding to the timestamp.
            write_position = timestamp_to_samples(timeNs);
            // If timestamp is in the past, quietly discard the samples.
            // It might be more "correct" to return SOAPY_SDR_TIME_ERROR instead,
            // but seems like discarding is a common behavior with other SDRs, so
            // I am worried some applications might not properly handle the error code.
            int64_t diff = playback_position - write_position;
            if (diff > 0) {
                // Timestamp is in the past.
                // Do nothing but pretend all samples were written.
                SoapySDR_logf(SOAPY_SDR_WARNING, "Discarding TX %d samples in the past", diff);
                return (int)length;
            }
        } else {
            // No timestamp provided.
            // Continue writing from where last write ended.
            write_position = stream->position;
            // If TX buffer has underrun, skip to a future position.
            // Round the number of samples to skip to a multiple of period size,
            // so applications that use writes aligned to periods will stay aligned.
            // Add 1-2 extra periods for some margin.
            int64_t diff = playback_position - write_position;
            if (diff > 0) {
                diff = (diff / (int64_t)stream->hwp_period_size + 2) * (int64_t)stream->hwp_period_size;
                write_position += diff;
                SoapySDR_logf(SOAPY_SDR_WARNING, "TX buffer underrun. Forwarding TX stream by %d samples", diff);
            }
        }


        // How much the stream should be forwarded
        // to get samples written to write_position.
        int64_t posdiff = write_position - stream->position;

        while (posdiff > 0) {
            // snd_pcm_sframes_t is long. Clamp to its maximum value just in case.
            snd_pcm_sframes_t samples_to_forward = (snd_pcm_sframes_t)std::min(posdiff, (int64_t)LONG_MAX);

            snd_pcm_sframes_t forwardable = snd_pcm_forwardable(pcm);
            if (forwardable < 0) {
                // TODO: check if it can fail and think what to actually do here
                SoapySDR_logf(SOAPY_SDR_ERROR, "tx snd_pcm_forwardable: %d", forwardable);
                return alsa_error_to_soapy_tx((int)forwardable);
            }

            snd_pcm_sframes_t forwarded;
            if (samples_to_forward < forwardable) {
                forwarded = snd_pcm_forward(pcm, samples_to_forward);
            } else {
                // Forward as much as possible, then wait for more space
                forwarded = snd_pcm_forward(pcm, forwardable);
                snd_pcm_wait(pcm, -10001);
            }

            if (forwarded < 0) {
                // TODO: check if it can fail and think what to actually do here
                SoapySDR_logf(SOAPY_SDR_ERROR, "tx snd_pcm_forward: %d", forwarded);
                return alsa_error_to_soapy_tx((int)forwarded);
            }
            stream->position += forwarded;
            posdiff -= forwarded;
            pcm_avail -= forwarded;
        }


        if (timeoutUs <= 0) {
            // Hack to make non-blocking write work:
            // limit number of samples to the amount of space in the buffer now.

            if (pcm_avail <= 0) {
                length = 0;
            } else if ((snd_pcm_uframes_t)pcm_avail < length) {
                length = (snd_pcm_uframes_t)pcm_avail;
            }
        }

        if (length > buffer_tx.size())
            buffer_tx.resize(length);

        convert_tx_buffer(buffs[0], 0, buffer_tx.data(), 0, length, tx_threshold2);

        if (length > 0) {
            snd_pcm_sframes_t samples_written = snd_pcm_writei(pcm, buffer_tx.data(), length);
            if (samples_written >= 0) {
                // Successful write
                stream->position += samples_written;
                return (int)samples_written;
            } else {
                return alsa_error_to_soapy_tx((int)samples_written);
            }
        } else {
            // Nothing to write
            return 0;
        }
    }

    long long getHardwareTime(const std::string &what) const
    {
        if (what == "") {
            // RX and TX streams should give similar results here.
            //
            // Use TX stream to avoid locking the RX stream mutex.
            // Reasoning is that getHardwareTime seems most useful
            // for determining timing of producing TX signals,
            // and some applications might run RX and TX in different threads,
            // we want to avoid locks between them, and I guess
            // getHardwareTime is most likely to be called from a TX thread.
            //
            // The mutex is needed to avoid race conditions between updating
            // our stream->position and the ALSA internal stream position,
            // in case getHardwareTime gets called from some other thread
            // while writeStream is running.

            auto *stream = &alsa_tx;
            std::scoped_lock lock(stream->mutex);
            snd_pcm_t *pcm = stream->pcm;

            snd_pcm_sframes_t pcm_avail = 0, pcm_delay = 0;
            int ret = snd_pcm_avail_delay(pcm, &pcm_avail, &pcm_delay);
            SoapySDR_logf(SOAPY_SDR_DEBUG, "hwt snd_pcm_avail_delay: %d %ld %ld  pos: %ld", ret, pcm_avail, pcm_delay, stream->position);
            if (ret < 0) {
                throw std::runtime_error("ALSA error");
            } else {
                return samples_to_timestamp(stream->position - (int64_t)pcm_delay);
            }
        } else {
            throw std::runtime_error("Unsupported time");
        }
    }

/***********************************************************************
 * Sensors
 **********************************************************************/

    std::vector<std::string> listSensors(void) const
    {
        return {
            "temperature",
            "temperature_raw",
            "temperature_channel",
            "temperature_i_raw",
            "temperature_iout",
            "temperature_q_raw",
            "temperature_qout",
        };
    }

    SoapySDR::ArgInfo getSensorInfo(const std::string &key) const
    {
        SoapySDR::ArgInfo info;
        info.key = key;
        info.type = SoapySDR::ArgInfo::FLOAT;

        if (key == "temperature") {
            info.name = "SX1255 temperature";
            info.units = "C";
            info.description =
                "SX1255 internal temperature estimate. Reading requires inactive streams because the sensor uses the RX ADC path.";
        } else if (key == "temperature_raw") {
            info.name = "SX1255 raw temperature ADC";
            info.units = "LSB";
            info.description = "Selected coherent raw SX1255 temperature ADC code.";
        } else if (key == "temperature_channel") {
            info.name = "SX1255 temperature channel";
            info.type = SoapySDR::ArgInfo::STRING;
            info.description = "ADC lane selected for the coherent SX1255 temperature reading.";
        } else if (key == "temperature_i_raw") {
            info.name = "SX1255 I raw temperature ADC";
            info.units = "LSB";
            info.description = "Raw SX1255 temperature ADC code derived from I_OUT.";
        } else if (key == "temperature_iout") {
            info.name = "SX1255 temperature I_OUT mean";
            info.units = "S32";
            info.description = "Mean raw signed 32-bit I_OUT sample measured while rx_adc_temp is enabled.";
        } else if (key == "temperature_q_raw") {
            info.name = "SX1255 Q raw temperature ADC";
            info.units = "LSB";
            info.description = "Raw SX1255 temperature ADC code derived from Q_OUT.";
        } else if (key == "temperature_qout") {
            info.name = "SX1255 temperature Q_OUT mean";
            info.units = "S32";
            info.description = "Mean raw signed 32-bit Q_OUT sample measured while rx_adc_temp is enabled.";
        } else {
            throw std::runtime_error("Unsupported sensor");
        }

        return info;
    }

    std::string readSensor(const std::string &key) const
    {
        if (
            key != "temperature" &&
            key != "temperature_raw" &&
            key != "temperature_channel" &&
            key != "temperature_i_raw" &&
            key != "temperature_iout" &&
            key != "temperature_q_raw" &&
            key != "temperature_qout"
        ) {
            throw std::runtime_error("Unsupported sensor");
        }

        temperature_measurement measurement;
        try {
            std::scoped_lock cache_lock(temperature_mutex);
            const auto now = std::chrono::steady_clock::now();
            if (
                !temperature_cache_valid ||
                std::chrono::duration_cast<std::chrono::milliseconds>(now - temperature_cache.measured_at).count() > SX1255_TEMP_CACHE_MS
            ) {
                temperature_cache = measure_temperature_adc();
                temperature_cache_valid = true;
            }
            measurement = temperature_cache;
        } catch (const std::exception &e) {
            SoapySDR_logf(SOAPY_SDR_WARNING, "SX1255 temperature read failed: %s", e.what());
            return "nan";
        }

        if (key == "temperature") {
            return format_double(raw_temperature_to_c(measurement.raw_code), 3);
        } else if (key == "temperature_raw") {
            return format_double(measurement.raw_code, 3);
        } else if (key == "temperature_channel") {
            return std::string(1, measurement.channel);
        } else if (key == "temperature_i_raw") {
            return format_double(measurement.i_raw_code, 3);
        } else if (key == "temperature_iout") {
            return format_double(measurement.iout_mean, 3);
        } else if (key == "temperature_q_raw") {
            return format_double(measurement.q_raw_code, 3);
        } else if (key == "temperature_qout") {
            return format_double(measurement.qout_mean, 3);
        }

        return "nan";
    }

/***********************************************************************
 * RX correction
 **********************************************************************/

    bool hasDCOffset(const int direction, const size_t channel) const override
    {
        (void)channel;
        return direction == SOAPY_SDR_RX && channel == 0;
    }

    void setDCOffset(const int direction, const size_t channel, const std::complex<double> &offset) override
    {
        validate_rx_correction_channel(direction, channel);
        if (!std::isfinite(offset.real()) || !std::isfinite(offset.imag()) ||
            std::abs(offset.real()) > 1.0 || std::abs(offset.imag()) > 1.0) {
            throw std::runtime_error("RX DC offset correction must be finite and within +/-1.0");
        }

        std::scoped_lock lock(rx_correction_mutex);
        rx_dc_i = (float)offset.real();
        rx_dc_q = (float)offset.imag();
    }

    std::complex<double> getDCOffset(const int direction, const size_t channel) const override
    {
        validate_rx_correction_channel(direction, channel);
        std::scoped_lock lock(rx_correction_mutex);
        return std::complex<double>(rx_dc_i, rx_dc_q);
    }

    bool hasIQBalance(const int direction, const size_t channel) const override
    {
        (void)channel;
        return direction == SOAPY_SDR_RX && channel == 0;
    }

    void setIQBalance(const int direction, const size_t channel, const std::complex<double> &balance) override
    {
        validate_rx_correction_channel(direction, channel);
        if (!std::isfinite(balance.real()) || !std::isfinite(balance.imag()) ||
            std::abs(balance.real()) > 1.0 || std::abs(balance.imag()) > 1.0) {
            throw std::runtime_error("RX IQ balance correction must be finite and within +/-1.0");
        }

        std::scoped_lock lock(rx_correction_mutex);
        rx_iq_balance = std::complex<float>((float)balance.real(), (float)balance.imag());
    }

    std::complex<double> getIQBalance(const int direction, const size_t channel) const override
    {
        validate_rx_correction_channel(direction, channel);
        std::scoped_lock lock(rx_correction_mutex);
        return std::complex<double>(rx_iq_balance.real(), rx_iq_balance.imag());
    }

/***********************************************************************
 * Sample rates
 **********************************************************************/

    std::vector<double> listSampleRates(const int direction, const size_t channel) const
    {
        (void)direction; (void)channel;
        std::vector<double> sampleRates;
        for (size_t i = 0; i < N_SAMPLE_RATES; i++) {
            sampleRates.push_back(masterClock / (double)sample_rates[i].div);
        }
        return sampleRates;
    }

    // Wrapper for listSampleRates to support both methods
    SoapySDR::RangeList getSampleRateRange(const int direction, const size_t channel) const
    {
        const auto sampleRates = listSampleRates(direction, channel);
        SoapySDR::RangeList ranges;
        for (const auto sampleRate: sampleRates) {
            ranges.push_back({sampleRate, sampleRate, 0});
        }
        return ranges;
    }

    void setSampleRate(
        const int direction,
        const size_t channel,
        const double rate
    )
    {
        (void)direction; (void)channel;

        std::scoped_lock lock(reg_mutex);

        if (rate != rate || rate <= 0)
            throw std::runtime_error("Sample rate must be positive");

        double divider = round(masterClock / rate);
        struct sampleRateRegs r;
        bool found = false;
        for (size_t i = 0; i < N_SAMPLE_RATES; i++) {
            r = sample_rates[i];
            if ((double)r.div == divider) {
                found = true;
                break;
            }
        }
        if (!found) {
            throw std::runtime_error("Unsupported sample rate");
        }
        // Disable RX and TX before changing sample rate.
        // Changing sample rate while RX/TX was running sometimes seemed
        // to break ordering of data over I2S, causing things like swapped
        // left and right channels or otherwise corrupted data.
        // This seems to fix the problem.
        set_register_bits(0x00, 1, 2, 0);
        write_registers_to_chip(0x00, 1);
        // Change the sample rate
        set_register_bits(0x12, 0, 4, r.clkout);
        set_register_bits(0x13, 7, 1, r.mant);
        set_register_bits(0x13, 6, 1, r.m);
        set_register_bits(0x13, 3, 3, r.n);
        write_registers_to_chip(0x12, 2);
        sampleRate = masterClock / divider;
        // Enable RX and TX again
        set_register_bits(0x00, 1, 2, 3);
        write_registers_to_chip(0x00, 1);
    }

    double getSampleRate(
        const int direction,
        const size_t channel
    ) const
    {
        (void)direction; (void)channel;
        std::scoped_lock lock(reg_mutex);
        return sampleRate;
    }

/***********************************************************************
 * Center frequency
 **********************************************************************/

    void setFrequency(
        const int direction,
        const size_t channel,
        const double frequency,
        const SoapySDR::Kwargs & args
    )
    {
        (void)channel; (void)args;

        std::scoped_lock lock(reg_mutex);

        const double step = masterClock * (1.0 / (double)(1L<<20));
        const uint32_t quantized = (uint32_t)scale_from_range(
            SoapySDR::Range(0, step * (double)((1L<<24)-1), step),
            frequency);
        if (direction == SOAPY_SDR_RX) {
            set_register_bits(0x01, 0, 8, quantized >> 16);
            set_register_bits(0x02, 0, 8, (quantized >> 8) & 0xFF);
            set_register_bits(0x03, 0, 8, quantized & 0xFF);
            write_registers_to_chip(0x01, 3);
        } else {
            set_register_bits(0x04, 0, 8, quantized >> 16);
            set_register_bits(0x05, 0, 8, (quantized >> 8) & 0xFF);
            set_register_bits(0x06, 0, 8, quantized & 0xFF);
            write_registers_to_chip(0x04, 3);
        }
    }

    double getFrequency(
        const int direction,
        const size_t channel
    ) const
    {
        (void)channel;

        std::scoped_lock lock(reg_mutex);

        const double step = masterClock * (1.0 / (double)(1L<<20));
        if (direction == SOAPY_SDR_RX)
            return step * (
                (((uint32_t)regs[1]) << 16) |
                (((uint32_t)regs[2]) << 8) |
                 ((uint32_t)regs[3]));
        else
            return step * (
                (((uint32_t)regs[4]) << 16) |
                (((uint32_t)regs[5]) << 8) |
                 ((uint32_t)regs[6]));
    }

/***********************************************************************
 * Gains
 **********************************************************************/

    std::vector<std::string> listGains(
        const int direction,
        const size_t channel
    ) const
    {
        (void)channel;
        if (direction == SOAPY_SDR_RX)
            return std::vector<std::string>{"LNA", "PGA"};
        else
            return std::vector<std::string>{"DAC", "MIXER"};
    }

    SoapySDR::Range getGainRange(
        const int direction,
        const size_t channel,
        const std::string & name
    ) const
    {
        (void)channel;
        if (direction == SOAPY_SDR_RX) {
            if (name == "LNA")   return {  0.0 , 48.0 , 6.0 };
            if (name == "PGA")   return {  0.0 , 30.0 , 2.0 };
        } else {
            if (name == "DAC")   return {  0.0 ,  9.0 , 3.0 };
            if (name == "MIXER") return {  0.0 , 30.0 , 2.0 };
        }
        return {0, 0, 0};
    }

    void setGain(
        const int direction,
        const size_t channel,
        const std::string & name,
        const double value
    )
    {
        std::scoped_lock lock(reg_mutex);

        int32_t quantized = scale_from_range(getGainRange(direction, channel, name), value);
        if (direction == SOAPY_SDR_RX) {
            if (name == "LNA") {
                // LNA gain does not have a constant step,
                // so some extra logic is needed.
                if (quantized <= 6) // -48 to -12 dB
                    set_register_bits(0x0C, 5, 3, 6-quantized/2);
                else if (quantized == 7) // -6 dB
                    set_register_bits(0x0C, 5, 3, 2);
                else // 0 dB
                    set_register_bits(0x0C, 5, 3, 1);
            } else if (name == "PGA") {
                set_register_bits(0x0C, 1, 4, quantized);
            }
            SoapySDR_logf(SOAPY_SDR_DEBUG, "RXFE1=0x%02x", regs[0x0C]);
            write_registers_to_chip(0x0C, 1);
        } else {
            if (name == "DAC") {
                set_register_bits(0x08, 4, 3, quantized);
            } else if (name == "MIXER") {
                set_register_bits(0x08, 0, 4, quantized);
            }
            SoapySDR_logf(SOAPY_SDR_DEBUG, "TXFE1=0x%02x", regs[0x08]);
            write_registers_to_chip(0x08, 1);
        }
    }

    double getGain(
        const int direction,
        const size_t channel,
        const std::string & name
    ) const
    {
        std::scoped_lock lock(reg_mutex);

        int32_t quantized = 0;
        if (direction == SOAPY_SDR_RX) {
            if (name == "LNA") {
                const int32_t map[8] = {0, 8, 7, 6, 4, 2, 0, 0};
                quantized = map[get_cached_register_bits(0x0C, 5, 3)];
            } else if (name == "PGA") {
                quantized = get_cached_register_bits(0x0C, 1, 4);
            }
        } else {
            if (name == "DAC") {
                quantized = get_cached_register_bits(0x08, 4, 3);
            } else if (name == "MIXER") {
                quantized = get_cached_register_bits(0x08, 0, 4);
            }
        }
        return scale_to_range(getGainRange(direction, channel, name), quantized);
    }

    void setGain(
        const int direction,
        const size_t channel,
        const double value
    )
    {
        std::scoped_lock lock(reg_mutex);

        if (direction == SOAPY_SDR_RX) {
            // Keep PGA gain around pga_gain_target over most of the
            // gain range while adjusting LNA gain over a wide range.
            // PGA gain has a smaller step, so use it to fine tune gain.
            const double pga_gain_target = 12.0;
            setGain(direction, channel, "LNA", value - pga_gain_target);
            double lna_gain = getGain(direction, channel, "LNA");
            setGain(direction, channel, "PGA", value - lna_gain);
        } else {
            // Not sure about best TX gain distribution yet.
            // Use similar logic as RX gains for now.
            const double mixer_gain_target = 26.0;
            setGain(direction, channel, "DAC", value - mixer_gain_target);
            double dac_gain = getGain(direction, channel, "DAC");
            setGain(direction, channel, "MIXER", value - dac_gain);
        }
    }

/***********************************************************************
 * Antennas
 **********************************************************************/

    std::vector<std::string> listAntennas(const int direction, const size_t channel) const
    {
        (void)channel; // Only one channel
        std::vector<std::string> antennas;
        if (direction == SOAPY_SDR_RX) {
            antennas.push_back("RX");
            antennas.push_back("LB");
            // Digital loopback did not seem to work, so do not list it
            //antennas.push_back("DLB");
        } else {
            antennas.push_back("TX");
            antennas.push_back("NONE");
        }
        return antennas;
    }

    void setAntenna(const int direction, const size_t channel, const std::string &name)
    {
        (void)channel;
        std::scoped_lock lock(reg_mutex);

        if (direction == SOAPY_SDR_RX) {
            if (name == "RX") {
                // Disable loopback
                set_register_bits(0x10, 2, 2, 0);
            } else if (name == "LB") {
                // RF loopback
                set_register_bits(0x10, 2, 2, 1);
            } else if (name == "DLB") {
                // Digital loopback
                set_register_bits(0x10, 2, 2, 3);
            }
            write_registers_to_chip(0x10, 1);
        } else {
            if (name == "TX") {
                // Enable PA
                set_register_bits(0x00, 3, 1, 1);
            } else if (name == "NONE") {
                // Disable PA
                set_register_bits(0x00, 3, 1, 0);
            }
            write_registers_to_chip(0x00, 1);
        }
    }

    std::string getAntenna(const int direction, const size_t channel) const
    {
        (void)channel;
        std::scoped_lock lock(reg_mutex);

        if (direction == SOAPY_SDR_RX) {
            unsigned lb = get_cached_register_bits(0x10, 2, 2);
            if (lb & 2)
                return "DLB";
            if (lb & 1)
                return "LB";
            return "RX";
        } else {
            if (get_cached_register_bits(0x00, 3, 1)) {
                // PA is enabled
                return "TX";
            } else {
                // PA is disabled
                return "NONE";
            }
        }
    }

/***********************************************************************
 * Other settings
 **********************************************************************/

    std::vector<std::string> listSettings(void) const
    {
        return {
            "PA",
            "TEMP_REF_C",
            "TEMP_REF_RAW",
            "TEMP_SAMPLES",
            "TEMP_SHIFT",
            "RF_PROFILE",
            "TX_FILTER_BW",
            "TX_DAC_BW",
            "RX_ADC_BW",
            "RX_ADC_TRIM",
            "RX_PGA_BW",
            "RX_DC_I",
            "RX_DC_Q",
            "RX_IQ_I",
            "RX_IQ_Q",
        };
    }

    SoapySDR::ArgInfoList getSettingInfo(void) const
    {
        SoapySDR::ArgInfoList infos;

        SoapySDR::ArgInfo pa;
        pa.key = "PA";
        pa.name = "PA control";
        pa.description = "Power amplifier control: ON, OFF, or AUTO";
        pa.type = SoapySDR::ArgInfo::STRING;
        pa.options = {"ON", "OFF", "AUTO"};
        infos.push_back(pa);

        SoapySDR::ArgInfo ref_c;
        ref_c.key = "TEMP_REF_C";
        ref_c.name = "Temperature reference";
        ref_c.description = "Known board temperature in Celsius for SX1255 temperature conversion calibration";
        ref_c.type = SoapySDR::ArgInfo::FLOAT;
        ref_c.units = "C";
        infos.push_back(ref_c);

        SoapySDR::ArgInfo ref_raw;
        ref_raw.key = "TEMP_REF_RAW";
        ref_raw.name = "Raw temperature reference";
        ref_raw.description = "Raw SX1255 temperature ADC code measured at TEMP_REF_C";
        ref_raw.type = SoapySDR::ArgInfo::FLOAT;
        ref_raw.units = "LSB";
        infos.push_back(ref_raw);

        SoapySDR::ArgInfo samples;
        samples.key = "TEMP_SAMPLES";
        samples.name = "Temperature sample count";
        samples.description = "Number of RX Q_OUT samples averaged for each temperature measurement";
        samples.type = SoapySDR::ArgInfo::INT;
        samples.units = "samples";
        samples.range = SoapySDR::Range(16, 4096, 1);
        infos.push_back(samples);

        SoapySDR::ArgInfo shift;
        shift.key = "TEMP_SHIFT";
        shift.name = "Temperature Q_OUT shift";
        shift.description = "Right shift used to convert mean signed 32-bit Q_OUT samples to raw ADC code";
        shift.type = SoapySDR::ArgInfo::INT;
        shift.units = "bits";
        shift.range = SoapySDR::Range(0, 31, 1);
        infos.push_back(shift);

        SoapySDR::ArgInfo rf_profile;
        rf_profile.key = "RF_PROFILE";
        rf_profile.name = "RF filter profile";
        rf_profile.description = "Preset SX1255 TX/RX filter settings. TETRA_CLEAN keeps RF filters wider than the TETRA channel while reducing TX DAC/image noise.";
        rf_profile.type = SoapySDR::ArgInfo::STRING;
        rf_profile.options = {"TETRA_CLEAN", "LEGACY"};
        infos.push_back(rf_profile);

        SoapySDR::ArgInfo tx_filter_bw;
        tx_filter_bw.key = "TX_FILTER_BW";
        tx_filter_bw.name = "TX analog filter code";
        tx_filter_bw.description = "SX1255 TX analog filter bandwidth code from TXFE2 bits 4:0";
        tx_filter_bw.type = SoapySDR::ArgInfo::INT;
        tx_filter_bw.range = SoapySDR::Range(0, 31, 1);
        infos.push_back(tx_filter_bw);

        SoapySDR::ArgInfo tx_dac_bw;
        tx_dac_bw.key = "TX_DAC_BW";
        tx_dac_bw.name = "TX DAC FIR bandwidth code";
        tx_dac_bw.description = "SX1255 TX DAC FIR bandwidth code from TXFE3 bits 2:0";
        tx_dac_bw.type = SoapySDR::ArgInfo::INT;
        tx_dac_bw.range = SoapySDR::Range(0, 5, 1);
        infos.push_back(tx_dac_bw);

        SoapySDR::ArgInfo rx_adc_bw;
        rx_adc_bw.key = "RX_ADC_BW";
        rx_adc_bw.name = "RX ADC bandwidth code";
        rx_adc_bw.description = "SX1255 RX ADC bandwidth code from RXFE2 bits 7:5";
        rx_adc_bw.type = SoapySDR::ArgInfo::INT;
        rx_adc_bw.range = SoapySDR::Range(0, 7, 1);
        infos.push_back(rx_adc_bw);

        SoapySDR::ArgInfo rx_adc_trim;
        rx_adc_trim.key = "RX_ADC_TRIM";
        rx_adc_trim.name = "RX ADC trim code";
        rx_adc_trim.description = "SX1255 RX ADC trim code from RXFE2 bits 4:2";
        rx_adc_trim.type = SoapySDR::ArgInfo::INT;
        rx_adc_trim.range = SoapySDR::Range(0, 7, 1);
        infos.push_back(rx_adc_trim);

        SoapySDR::ArgInfo rx_pga_bw;
        rx_pga_bw.key = "RX_PGA_BW";
        rx_pga_bw.name = "RX PGA bandwidth code";
        rx_pga_bw.description = "SX1255 RX PGA bandwidth code from RXFE2 bits 1:0";
        rx_pga_bw.type = SoapySDR::ArgInfo::INT;
        rx_pga_bw.range = SoapySDR::Range(0, 3, 1);
        infos.push_back(rx_pga_bw);

        SoapySDR::ArgInfo rx_dc_i;
        rx_dc_i.key = "RX_DC_I";
        rx_dc_i.name = "RX DC I correction";
        rx_dc_i.description = "Manual RX I DC correction applied in the SoapySX sample converter";
        rx_dc_i.type = SoapySDR::ArgInfo::FLOAT;
        rx_dc_i.range = SoapySDR::Range(-1.0, 1.0, 0.0);
        infos.push_back(rx_dc_i);

        SoapySDR::ArgInfo rx_dc_q;
        rx_dc_q.key = "RX_DC_Q";
        rx_dc_q.name = "RX DC Q correction";
        rx_dc_q.description = "Manual RX Q DC correction applied in the SoapySX sample converter";
        rx_dc_q.type = SoapySDR::ArgInfo::FLOAT;
        rx_dc_q.range = SoapySDR::Range(-1.0, 1.0, 0.0);
        infos.push_back(rx_dc_q);

        SoapySDR::ArgInfo rx_iq_i;
        rx_iq_i.key = "RX_IQ_I";
        rx_iq_i.name = "RX IQ correction real";
        rx_iq_i.description = "Real part of image correction coefficient applied as y = x + c * conj(x)";
        rx_iq_i.type = SoapySDR::ArgInfo::FLOAT;
        rx_iq_i.range = SoapySDR::Range(-1.0, 1.0, 0.0);
        infos.push_back(rx_iq_i);

        SoapySDR::ArgInfo rx_iq_q;
        rx_iq_q.key = "RX_IQ_Q";
        rx_iq_q.name = "RX IQ correction imaginary";
        rx_iq_q.description = "Imaginary part of image correction coefficient applied as y = x + c * conj(x)";
        rx_iq_q.type = SoapySDR::ArgInfo::FLOAT;
        rx_iq_q.range = SoapySDR::Range(-1.0, 1.0, 0.0);
        infos.push_back(rx_iq_q);

        return infos;
    }

    void writeSetting(
        const std::string & key,
        const std::string & value
    )
    {
        std::scoped_lock lock(reg_mutex);

        // PA control modes
        if (key == "PA") {
            if (value == "ON") {
                // PA always on
                gpio_tx.set_value(1);
                gpio_rx.set_value(0);
                pa_mode = value;
            } else if (value == "OFF") {
                // PA always off
                gpio_tx.set_value(0);
                gpio_rx.set_value(1);
                pa_mode = value;
            } else if (value == "AUTO") {
                // PA on/off controlled by TX stream (default)
                gpio_tx.set_value(1);
                gpio_rx.set_value(1);
                pa_mode = value;
            } else {
                throw std::runtime_error("Unsupported PA setting");
            }
        } else if (key == "TEMP_REF_C") {
            temperature_ref_c = std::stod(value);
        } else if (key == "TEMP_REF_RAW") {
            temperature_ref_raw = std::stod(value);
        } else if (key == "TEMP_SAMPLES") {
            temperature_samples = parse_unsigned_setting(value, 16, 4096);
        } else if (key == "TEMP_SHIFT") {
            temperature_shift = parse_unsigned_setting(value, 0, 31);
        } else if (key == "RF_PROFILE") {
            set_filter_profile(value);
        } else if (key == "TX_FILTER_BW") {
            set_register_bits(SX1255_REG_TXFE2, 0, 5, parse_unsigned_setting(value, 0, 31));
            write_registers_to_chip(SX1255_REG_TXFE2, 1);
        } else if (key == "TX_DAC_BW") {
            set_register_bits(SX1255_REG_TXFE3, 0, 3, parse_unsigned_setting(value, 0, 5));
            write_registers_to_chip(SX1255_REG_TXFE3, 1);
        } else if (key == "RX_ADC_BW") {
            set_register_bits(SX1255_REG_RXFE2, 5, 3, parse_unsigned_setting(value, 0, 7));
            write_registers_to_chip(SX1255_REG_RXFE2, 1);
        } else if (key == "RX_ADC_TRIM") {
            set_register_bits(SX1255_REG_RXFE2, 2, 3, parse_unsigned_setting(value, 0, 7));
            write_registers_to_chip(SX1255_REG_RXFE2, 1);
        } else if (key == "RX_PGA_BW") {
            set_register_bits(SX1255_REG_RXFE2, 0, 2, parse_unsigned_setting(value, 0, 3));
            write_registers_to_chip(SX1255_REG_RXFE2, 1);
        } else if (key == "RX_DC_I") {
            std::scoped_lock correction_lock(rx_correction_mutex);
            rx_dc_i = parse_correction_setting(value);
        } else if (key == "RX_DC_Q") {
            std::scoped_lock correction_lock(rx_correction_mutex);
            rx_dc_q = parse_correction_setting(value);
        } else if (key == "RX_IQ_I") {
            std::scoped_lock correction_lock(rx_correction_mutex);
            rx_iq_balance = std::complex<float>(parse_correction_setting(value), rx_iq_balance.imag());
        } else if (key == "RX_IQ_Q") {
            std::scoped_lock correction_lock(rx_correction_mutex);
            rx_iq_balance = std::complex<float>(rx_iq_balance.real(), parse_correction_setting(value));
        } else {
            throw std::runtime_error("Unsupported setting");
        }
    }

    std::string readSetting(const std::string & key) const
    {
        std::scoped_lock lock(reg_mutex);

        if (key == "PA") {
            return pa_mode;
        } else if (key == "TEMP_REF_C") {
            return format_double(temperature_ref_c, 6);
        } else if (key == "TEMP_REF_RAW") {
            return format_double(temperature_ref_raw, 6);
        } else if (key == "TEMP_SAMPLES") {
            return std::to_string(temperature_samples);
        } else if (key == "TEMP_SHIFT") {
            return std::to_string(temperature_shift);
        } else if (key == "RF_PROFILE") {
            return get_filter_profile();
        } else if (key == "TX_FILTER_BW") {
            return std::to_string(get_cached_register_bits(SX1255_REG_TXFE2, 0, 5));
        } else if (key == "TX_DAC_BW") {
            return std::to_string(get_cached_register_bits(SX1255_REG_TXFE3, 0, 3));
        } else if (key == "RX_ADC_BW") {
            return std::to_string(get_cached_register_bits(SX1255_REG_RXFE2, 5, 3));
        } else if (key == "RX_ADC_TRIM") {
            return std::to_string(get_cached_register_bits(SX1255_REG_RXFE2, 2, 3));
        } else if (key == "RX_PGA_BW") {
            return std::to_string(get_cached_register_bits(SX1255_REG_RXFE2, 0, 2));
        } else if (key == "RX_DC_I") {
            std::scoped_lock correction_lock(rx_correction_mutex);
            return format_double(rx_dc_i, 9);
        } else if (key == "RX_DC_Q") {
            std::scoped_lock correction_lock(rx_correction_mutex);
            return format_double(rx_dc_q, 9);
        } else if (key == "RX_IQ_I") {
            std::scoped_lock correction_lock(rx_correction_mutex);
            return format_double(rx_iq_balance.real(), 9);
        } else if (key == "RX_IQ_Q") {
            std::scoped_lock correction_lock(rx_correction_mutex);
            return format_double(rx_iq_balance.imag(), 9);
        }

        throw std::runtime_error("Unsupported setting");
    }

/***********************************************************************
 * Low level interfaces
 **********************************************************************/

    std::vector<unsigned> readRegisters(
        const std::string & name,
        const unsigned addr,
        const size_t length
    ) const
    {
        (void)name; // Ignore name since there's only one register bank
        std::scoped_lock lock(reg_mutex);

        size_t transfer_len = length + 1;
        // Buffer for SPI transfer
        std::vector<uint8_t> buf(transfer_len, 0);

        buf[0] = addr;
        for (size_t i = 1; i < transfer_len; i++)
            buf[i] = 0;

        spi.transfer(buf, buf);

        std::vector<unsigned> result(length, 0);
        for (size_t i = 0; i < length; i++)
            result[i] = buf[i+1];

        return result;
    }

    unsigned readRegister(
        const std::string & name,
        const unsigned addr
    ) const
    {
        auto r = readRegisters(name, addr, 1);
        return r.at(0);
    }

    void writeRegisters(
        const std::string & name,
        const unsigned addr,
        const std::vector<unsigned> & value
    )
    {
        (void)name; // Ignore name since there's only one register bank
        std::scoped_lock lock(reg_mutex);

        for (size_t i = 0; i < value.size(); i++)
            set_register_bits(addr + i, 0, 8, value[i]);

        write_registers_to_chip(addr, value.size());
    }

    void writeRegister(
        const std::string & name,
        const unsigned addr,
        const unsigned value
    )
    {
        (void)name;
        std::scoped_lock lock(reg_mutex);
        set_register_bits(addr, 0, 8, value);
        write_registers_to_chip(addr, 1);
    }

/***********************************************************************
 * Other hardware and driver information
 **********************************************************************/

    std::string getDriverKey(void) const
    {
        return "sx";
    }

    std::string getHardwareKey(void) const
    {
        return "sx";
    }

    SoapySDR::Kwargs getHardwareInfo(void) const
    {
        SoapySDR::Kwargs args;
        args["soapysx_tag"] = SoapySX_tag;
        args["soapysx_commit"] = SoapySX_commit;
        if (hat_info.read_success) {
            args["hardware_version"] = std::to_string(hat_info.product_ver >> 8)
                               + "." + std::to_string(hat_info.product_ver & 0xFF);
        } else {
            args["hardware_version"] = "unknown";
        }
        return args;
    }

    size_t getNumChannels(const int direction) const
    {
        (void)direction; // Same for both directions
        return 1;
    }

    std::string getNativeStreamFormat(
        const int direction,
        const size_t channel,
        double &fullScale
    ) const
    {
        (void)direction; (void)channel;
        fullScale = 1.0;
        // This is not really the "native" format
        // but the only one currently supported.
        return "CF32";
    }

    std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const
    {
       (void)direction; (void)channel;
        std::vector<std::string> streamFormats;
        streamFormats.push_back("CF32");
        return streamFormats;
    }

    bool hasHardwareTime(const std::string &what) const
    {
        if (what == "")
            return true;
        return false;
    }
};

/***********************************************************************
 * Find available devices
 **********************************************************************/
static SoapySDR::KwargsList findDevice(const SoapySDR::Kwargs &args)
{
    (void)args;
    SoapySDR::KwargsList devices;

    // TODO: check whether a device is actually found

    SoapySDR::Kwargs device;
    device["label"] = "sx";
    device["driver"] = "sx";
    devices.push_back(device);

    return devices;
}

/***********************************************************************
 * Make device instance
 **********************************************************************/
static SoapySDR::Device *makeDevice(const SoapySDR::Kwargs &args)
{
    SoapySDR::logf(SOAPY_SDR_INFO, "SoapySX version %s %s", SoapySX_tag, SoapySX_commit);
    return new SoapySX(args, read_hat_info());
}

/***********************************************************************
 * Registration
 **********************************************************************/
static SoapySDR::Registry registerDevice("sx", &findDevice, &makeDevice, SOAPY_SDR_ABI_VERSION);
