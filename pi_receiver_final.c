#define _GNU_SOURCE

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

//nRF24L01+ commands & registers 
#define NRF_CMD_R_REGISTER     0x00
#define NRF_CMD_W_REGISTER     0x20
#define NRF_CMD_R_RX_PAYLOAD   0x61
#define NRF_CMD_W_TX_PAYLOAD   0xA0
#define NRF_CMD_FLUSH_TX       0xE1
#define NRF_CMD_FLUSH_RX       0xE2
#define NRF_CMD_NOP            0xFF

#define NRF_REG_CONFIG         0x00
#define NRF_REG_EN_AA          0x01
#define NRF_REG_EN_RXADDR      0x02
#define NRF_REG_SETUP_AW       0x03
#define NRF_REG_SETUP_RETR     0x04
#define NRF_REG_RF_CH          0x05
#define NRF_REG_RF_SETUP       0x06
#define NRF_REG_STATUS         0x07
#define NRF_REG_RX_ADDR_P0     0x0A
#define NRF_REG_TX_ADDR        0x10
#define NRF_REG_RX_PW_P0       0x11
#define NRF_REG_FIFO_STATUS    0x17
#define NRF_REG_DYNPD          0x1C

//CONFIG bits
#define NRF_MASK_RX_DR         (1 << 6)
#define NRF_MASK_TX_DS         (1 << 5)
#define NRF_MASK_MAX_RT        (1 << 4)
#define NRF_EN_CRC             (1 << 3)
#define NRF_CRCO               (1 << 2)
#define NRF_PWR_UP             (1 << 1)
#define NRF_PRIM_RX            (1 << 0)

//STATUS bits
#define NRF_RX_DR              (1 << 6)
#define NRF_TX_DS              (1 << 5)
#define NRF_MAX_RT             (1 << 4)

//RF params
#define RF_CHANNEL             76 
#define RF_SETUP_VAL           0x06    // 1 Mbps, 0 dBm
static const uint8_t RF_ADDR[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

//UDP 
#define DEFAULT_DEST_IP   "<IP address>"  // IP here
#define DEFAULT_DEST_PORT 5005

//Packet Layout

typedef struct __attribute__((packed)){
    uint8_t  magic;      //0xAA
    uint8_t  version;
    uint16_t seq;

    int16_t  yaw;
    int16_t  pitch;
    int16_t  roll;

    uint8_t  flex[5];
    uint8_t  contacts;

    uint16_t batt_mv;    //battery (unused)

    uint8_t  reserved[12];  //future use

    uint16_t checksum;   //16-bit sum of first 30 bytes
} GlovePacket;

//CE Pin
#define CE_GPIO_NUM 17


static struct gpiod_chip *ce_chip = NULL;
static struct gpiod_line *ce_line = NULL;

static int gpio_init_ce(void)
{
    ce_chip = gpiod_chip_open_by_name("gpiochip0");
    if (!ce_chip){
        perror("gpiod_chip_open_by_name");
        return -1;
    }

    ce_line = gpiod_chip_get_line(ce_chip, CE_GPIO_NUM);
    if (!ce_line){
        perror("gpiod_chip_get_line");
        return -1;
    }

    if (gpiod_line_request_output(ce_line, "nrf24-ce", 0) < 0){
        perror("gpiod_line_request_output");
        return -1;
    }

    return 0;
}

static void gpio_set_ce(int level)
{
    if (!ce_line){
        return;
    }
    if (gpiod_line_set_value(ce_line, level) < 0){
        perror("gpiod_line_set_value");
    }
}

//SPI
static int spi_fd = -1;

static int spi_init(const char *device)
{
    uint8_t mode = 0;
    uint8_t bits = 8;
    uint32_t speed = 1000000; //1 MHz

    spi_fd = open(device, O_RDWR);
    if (spi_fd < 0){
        perror("open(spidev)");
        return -1;
    }

    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0){
        perror("SPI_IOC_WR_MODE");
        return -1;
    }
    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0){
        perror("SPI_IOC_WR_BITS_PER_WORD");
        return -1;
    }
    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0){
        perror("SPI_IOC_WR_MAX_SPEED_HZ");
        return -1;
    }

    return 0;
}

static uint8_t spi_xfer_byte(uint8_t data)
{
    uint8_t rx = 0;
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)&data,
        .rx_buf = (unsigned long)&rx,
        .len = 1,
        .speed_hz = 1000000,
        .delay_usecs = 0,
        .bits_per_word = 8,
    };

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0){
        perror("SPI_IOC_MESSAGE");
        return 0xFF;
    }
    return rx;
}

//nRF24L01+
static uint8_t nrf_read_reg(uint8_t reg)
{
    uint8_t tx[2] = { NRF_CMD_R_REGISTER | (reg & 0x1F), 0xFF };
    uint8_t rx[2] = { 0 };

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 2,
        .speed_hz = 1000000,
        .delay_usecs = 0,
        .bits_per_word = 8,
    };

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0){
        perror("SPI_IOC_MESSAGE");
        return 0xFF;
    }

    return rx[1];
}

static void nrf_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { NRF_CMD_W_REGISTER | (reg & 0x1F), val };
    uint8_t rx[2] = { 0 };

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 2,
        .speed_hz = 1000000,
        .delay_usecs = 0,
        .bits_per_word = 8,
    };

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0){
        perror("SPI_IOC_MESSAGE");
    }
}

static void nrf_write_buf(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    uint8_t tx[1 + 32] = {0};
    uint8_t rx[1 + 32] = {0};

    if (len > 32) len = 32;

    tx[0] = NRF_CMD_W_REGISTER | (reg & 0x1F);
    memcpy(&tx[1], buf, len);

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len    = len + 1,
        .speed_hz = 1000000,
        .delay_usecs = 0,
        .bits_per_word = 8,
    };

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0){
        perror("SPI_IOC_MESSAGE");
    }
}

static void nrf_read_rx_payload(uint8_t *buf, uint8_t len)
{
    uint8_t tx[1 + 32] = {0};
    uint8_t rx[1 + 32] = {0};

    if (len > 32) len = 32;

    tx[0] = NRF_CMD_R_RX_PAYLOAD;
    memset(&tx[1], NRF_CMD_NOP, len);

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len    = len + 1,
        .speed_hz = 1000000,
        .delay_usecs = 0,
        .bits_per_word = 8,
    };

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0){
        perror("SPI_IOC_MESSAGE");
        return;
    }

    memcpy(buf, &rx[1], len);
}

static void nrf_flush_rx(void)
{
    (void)spi_xfer_byte(NRF_CMD_FLUSH_RX);
}

static void nrf_clear_irq(void)
{
    nrf_write_reg(NRF_REG_STATUS, NRF_RX_DR | NRF_TX_DS | NRF_MAX_RT);
}


static void nrf_init_rx(void)
{  
    gpio_set_ce(0); //Bring CE low while configuring

    usleep(5000); //Power-up delay

    nrf_write_reg(NRF_REG_EN_AA, 0x00);  //Disable auto-ack

    nrf_write_reg(NRF_REG_EN_RXADDR, 0x01);     //Enable data pipe 0

    nrf_write_reg(NRF_REG_SETUP_AW, 0x03);    //Address width = 5 bytes

    nrf_write_reg(NRF_REG_SETUP_RETR, 0x00);    //No auto-retransmit

    //RF channel and setup
    nrf_write_reg(NRF_REG_RF_CH, RF_CHANNEL);
    nrf_write_reg(NRF_REG_RF_SETUP, RF_SETUP_VAL);

    //Set RX and TX address to same value
    nrf_write_buf(NRF_REG_RX_ADDR_P0, RF_ADDR, 5);
    nrf_write_buf(NRF_REG_TX_ADDR,    RF_ADDR, 5);


    nrf_write_reg(NRF_REG_RX_PW_P0, 32);     //Fixed payload width 32 bytes

    nrf_write_reg(NRF_REG_DYNPD, 0x00);     //Disable dynamic payloads

    //Clear FIFOs & IRQ flags
    nrf_flush_rx();
    nrf_clear_irq();

    //CONFIG: PWR_UP=1, PRIM_RX=1, EN_CRC=1
    uint8_t config = NRF_EN_CRC | NRF_PWR_UP | NRF_PRIM_RX;
    nrf_write_reg(NRF_REG_CONFIG, config);

    usleep(5000);

    //Enter RX mode
    gpio_set_ce(1);
}

//Packet decode
static uint16_t glove_checksum(const GlovePacket *pkt)
{
    const uint8_t *bytes = (const uint8_t *)pkt;
    uint16_t sum = 0;
    for(int i = 0; i < 30; i++){
        sum += bytes[i];
    }
    return sum;
}

//Verify
static int decode_glove_packet(const uint8_t *raw, GlovePacket *out)
{
    memcpy(out, raw, sizeof(GlovePacket));

    if (out->magic != 0xAA){
        fprintf(stderr, "bad magic: 0x%02X\n", out->magic);
        return -1;
    }
    if (out->version != 1){
        fprintf(stderr, "unsupported version: %u\n", out->version);
        return -1;
    }

    uint16_t sum = glove_checksum(out);
    if (sum != out->checksum){
        fprintf(stderr, "Checksum mismatch: calc=%u pkt=%u\n", sum, out->checksum);
        return -1;
    }

    return 0;
}




int main(int argc, char **argv)
{
    if (gpio_init_ce() < 0){
        fprintf(stderr, "Failed to init CE GPIO\n");
        return 1;
    }

    if (spi_init("/dev/spidev0.0") < 0){
        fprintf(stderr, "Failed to init SPI\n");
        return 1;
    }

    nrf_init_rx();
    printf("nRF24 RX ready. Listening for 32-byte payloads...\n");

    //UDP
    const char *dest_ip   = DEFAULT_DEST_IP;
    int         dest_port = DEFAULT_DEST_PORT;

    if (argc >= 2){
        dest_ip = argv[1];
    }
    if (argc >= 3){
        dest_port = atoi(argv[2]);
    }

    int udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_fd < 0){
        perror("socket");
        return 1;
    }

    struct sockaddr_in dest_addr;
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port   = htons(dest_port);
    if (inet_pton(AF_INET, dest_ip, &dest_addr.sin_addr) <= 0){
        perror("inet_pton");
        return 1;
    }

    printf("UDP target: %s:%d\n", dest_ip, dest_port);



    while (1){

        uint8_t status = spi_xfer_byte(NRF_CMD_NOP);

        if (status & NRF_RX_DR){
            uint8_t raw[32];
            memset(raw, 0, sizeof(raw));

            nrf_read_rx_payload(raw, 32);
            nrf_clear_irq();

            GlovePacket pkt;
            if (decode_glove_packet(raw, &pkt) != 0){
                continue;
            }

            float yaw   = pkt.yaw   / 100.0f;
            float pitch = pkt.pitch / 100.0f;
            float roll  = pkt.roll  / 100.0f;
            uint8_t flex0 = pkt.flex[0];
            uint8_t flex1 = pkt.flex[1];
            uint8_t flex2 = pkt.flex[2];
            uint8_t flex3 = pkt.flex[3];
            uint8_t fsr  = (pkt.contacts & 0x01) ? 1 : 0;

            printf("RX seq=%u yaw=%.2f pitch=%.2f roll=%.2f "
                "flex=[%u,%u,%u,%u] fsr=%u\n",
                pkt.seq, yaw, pitch, roll,
                flex0, flex1, flex2, flex3, fsr);
            fflush(stdout);

            char buf[128];
            int len = snprintf(buf, sizeof(buf),"%u,%.2f,%.2f,%.2f,%u,%u,%u,%u,%u\n", pkt.seq, yaw, pitch, roll, flex0, flex1, flex2, flex3, fsr);
            if (len > 0){
                ssize_t sent = sendto(udp_fd, buf, (size_t)len, 0,
                                      (struct sockaddr *)&dest_addr,
                                      sizeof(dest_addr));
                if (sent < 0){
                    perror("sendto");
                }
            }
        }

        usleep(10000); //10 ms polling
    }

    close(spi_fd);
    close(udp_fd);

    if (ce_chip){
        gpiod_chip_close(ce_chip);
        ce_chip = NULL;
        ce_line = NULL;
    }

    return 0;
}
