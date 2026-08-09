/*
 * dw1000_regs.h - DW1000 register map, bit defines and SPI command constants.
 *
 * Pure C, no Arduino / no C++. Values are taken from the Decawave DW1000
 * User Manual and from the vendored arduino-dw1000 DW1000Constants.h (which
 * itself mirrors the User Manual). Added to / edited as the pure-C port
 * progresses step by step.
 */
#ifndef DW1000_REGS_H
#define DW1000_REGS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================ SPI command header ============================
 * First byte of every SPI transaction:
 *   bit 7 : 0 = read, 1 = write
 *   bit 6 : 1 = sub-addressing used
 *   bits 5-0 : register address
 * Then 1 or 2 sub-address bytes (only with sub-addressing), then data. */

#define DW1000_WRITE       0x80  /* write, no sub-address */
#define DW1000_WRITE_SUB   0xC0  /* write with sub-address */
#define DW1000_READ        0x00  /* read, no sub-address */
#define DW1000_READ_SUB    0x40  /* read with sub-address */
#define DW1000_RW_SUB_EXT  0x80  /* extended (2-byte) sub-address follows */

#define DW1000_NO_SUB      0xFF  /* magic value: no sub-address */
#define DW1000_JUNK        0x00  /* dummy byte for read transactions */

/* ============================ clocks (PMSC_CTRL0 bits 1..0) =============== */

#define DW1000_AUTO_CLOCK  0x00  /* clock derived from PLL */
#define DW1000_XTI_CLOCK   0x01  /* external 38.4 MHz crystal clock */
#define DW1000_PLL_CLOCK   0x02  /* PLL clock */

/* ============================ register map ============================ */

#define DW1000_DEV_ID        0x00   /* device identifier            */
#define DW1000_LEN_DEV_ID    4
#define DW1000_EUI           0x01   /* extended unique identifier   */
#define DW1000_LEN_EUI       8
#define DW1000_PANADR        0x03   /* PAN id + short address      */
#define DW1000_LEN_PANADR    4
#define DW1000_SYS_CFG       0x04   /* system configuration        */
#define DW1000_LEN_SYS_CFG   4
#define DW1000_SYS_TIME      0x06   /* system time counter         */
#define DW1000_LEN_SYS_TIME  5
#define DW1000_TX_FCTRL      0x08   /* TX frame control            */
#define DW1000_LEN_TX_FCTRL  5
#define DW1000_TX_BUFFER     0x09   /* TX data buffer              */
#define DW1000_DX_TIME       0x0A   /* delayed TX/RX time          */
#define DW1000_LEN_DX_TIME   5
#define DW1000_SYS_CTRL      0x0D   /* system control              */
#define DW1000_LEN_SYS_CTRL  4
#define DW1000_SYS_MASK      0x0E   /* system event mask           */
#define DW1000_LEN_SYS_MASK  4
#define DW1000_SYS_STATUS    0x0F   /* system event status         */
#define DW1000_LEN_SYS_STATUS 5
#define DW1000_RX_FINFO      0x10   /* RX frame info              */
#define DW1000_LEN_RX_FINFO  4
#define DW1000_RX_BUFFER     0x11   /* RX data buffer              */
#define DW1000_RX_FQUAL      0x12   /* RX frame quality           */
#define DW1000_RX_TIME       0x15   /* RX timestamp + power        */
#define DW1000_TX_TIME       0x17   /* TX timestamp                */
#define DW1000_TX_ANTD       0x18   /* TX antenna delay            */
#define DW1000_CHAN_CTRL     0x1F   /* channel control             */
#define DW1000_LEN_CHAN_CTRL 4
#define DW1000_USR_SFD       0x21   /* user-defined SFD            */
#define DW1000_AGC_TUNE      0x23   /* AGC tuning                  */
#define DW1000_DRX_TUNE      0x27   /* DRX tuning                  */
#define DW1000_RF_CONF       0x28   /* RF config                   */
#define DW1000_TX_CAL        0x2A   /* TX calibration              */
#define DW1000_FS_CTRL       0x2B   /* frequency synthesizer ctrl  */
#define DW1000_AON           0x2C   /* always-on                   */
#define DW1000_OTP_IF        0x2D   /* OTP interface               */
#define DW1000_LDE_IF        0x2E   /* LDE interface               */
#define DW1000_PMSC          0x36   /* power management / clock    */

/* ============================ sub-addresses ============================ */

/* PMSC */
#define DW1000_PMSC_CTRL0_SUB 0x00
#define DW1000_PMSC_CTRL1_SUB 0x04
#define DW1000_PMSC_LEDC_SUB  0x28

/* OTP interface */
#define DW1000_OTP_ADDR_SUB 0x04
#define DW1000_OTP_CTRL_SUB 0x06
#define DW1000_OTP_RDAT_SUB 0x0A

/* LDE interface */
#define DW1000_LDE_CFG1_SUB   0x0806
#define DW1000_LDE_RXANTD_SUB 0x1804
#define DW1000_LDE_CFG2_SUB   0x1806
#define DW1000_LDE_REPC_SUB   0x2804

/* ============================ device modes (SYS_CTRL) ============================ */

#define DW1000_MODE_IDLE 0x00
#define DW1000_MODE_RX   0x01
#define DW1000_MODE_TX   0x02

/* ============================ SYS_CTRL bits ============================ */

#define DW1000_SFCST_BIT    0  /* suppress frame check on TX/RX */
#define DW1000_TXSTRT_BIT   1  /* start TX */
#define DW1000_TXDLYS_BIT   2  /* delayed TX */
#define DW1000_TRXOFF_BIT   6  /* transceiver off (idle) */
#define DW1000_WAIT4RESP_BIT 7 /* wait for response */
#define DW1000_RXENAB_BIT   8  /* receiver enable */
#define DW1000_RXDLYS_BIT   9  /* delayed RX */

/* ============================ data rate / PRF / preamble / channel encodings ============================ */

#define DW1000_RATE_110KBPS   0x00
#define DW1000_RATE_850KBPS   0x01
#define DW1000_RATE_6800KBPS  0x02

#define DW1000_PRF_16MHZ      0x01
#define DW1000_PRF_64MHZ      0x02

#define DW1000_PREAMBLE_LEN_64    0x01
#define DW1000_PREAMBLE_LEN_128   0x05
#define DW1000_PREAMBLE_LEN_256   0x09
#define DW1000_PREAMBLE_LEN_512   0x0D
#define DW1000_PREAMBLE_LEN_1024  0x02
#define DW1000_PREAMBLE_LEN_1536  0x06
#define DW1000_PREAMBLE_LEN_2048  0x0A
#define DW1000_PREAMBLE_LEN_4096  0x03

#define DW1000_CHANNEL_1  1
#define DW1000_CHANNEL_2  2
#define DW1000_CHANNEL_3  3
#define DW1000_CHANNEL_4  4
#define DW1000_CHANNEL_5  5
#define DW1000_CHANNEL_7  7

/* ============================ SYS_CFG bits ============================ */

#define DW1000_FFEN_BIT      0   /* frame filtering enable */
#define DW1000_FFBC_BIT      1   /* frame filter: behave as coordinator */
#define DW1000_FFAB_BIT      2   /* frame filter: allow beacon */
#define DW1000_FFAD_BIT      3   /* frame filter: allow data */
#define DW1000_FFAA_BIT      4   /* frame filter: allow acknowledgement */
#define DW1000_FFAM_BIT      5   /* frame filter: allow MAC command */
#define DW1000_FFAR_BIT      6   /* frame filter: allow reserved (blink) */
#define DW1000_HIRQ_POL_BIT  9   /* interrupt polarity (active high) */
#define DW1000_DIS_DRXB_BIT  12  /* disable double receive buffer */
#define DW1000_DIS_STXP_BIT  18  /* disable smart TX power */
#define DW1000_RXM110K_BIT   22  /* RX 110 kb/s mode flag */
#define DW1000_RXAUTR_BIT    29  /* receiver auto-reenable */

/* ============================ SYS_STATUS / SYS_MASK bits ============================ */

#define DW1000_AAT_BIT    3   /* auto acknowledge trigger */
#define DW1000_TXFRB_BIT  4   /* TX frame began */
#define DW1000_TXPRS_BIT  5   /* TX preamble sent */
#define DW1000_TXPHS_BIT  6   /* TX header sent */
#define DW1000_TXFRS_BIT  7   /* TX frame sent */
#define DW1000_LDEDONE_BIT 10 /* LDE processing done */
#define DW1000_RXPHE_BIT  12  /* RX preamble header error */
#define DW1000_RXDFR_BIT  13  /* RX data frame ready */
#define DW1000_RXFCG_BIT  14  /* RX frame check good (CRC ok) */
#define DW1000_RXFCE_BIT  15  /* RX frame check error (CRC fail) */
#define DW1000_RXRFSL_BIT 16  /* RX frame sync loss */
#define DW1000_RXRFTO_BIT 17  /* RX frame reception timeout */
#define DW1000_LDEERR_BIT 18  /* LDE error */
#define DW1000_RXPTO_BIT  21  /* RX preamble timeout */
#define DW1000_RFPLL_LL_BIT 24 /* RF PLL loss of lock */
#define DW1000_CLKPLL_LL_BIT 25 /* clock PLL loss of lock */
#define DW1000_RXSFDTO_BIT 26 /* RX SFD timeout */

/* ============================ CHAN_CTRL bits ============================ */

/* RX_CHAN field: bits 2..0, TX_CHAN field: bits 7..5 */
#define DW1000_CHAN_CTRL_RX_CHAN_MASK 0x07
#define DW1000_DWSFD_BIT   17  /* data-rate dependent SFD */
#define DW1000_TNSSFD_BIT  20  /* TX non-standard SFD */
#define DW1000_RNSSFD_BIT  21  /* RX non-standard SFD */

/* ============================ USR_SFD ============================ */

#define DW1000_SFD_LENGTH_SUB 0x00
#define DW1000_LEN_SFD_LENGTH 1

/* ============================ tuning registers ============================ */

#define DW1000_TX_POWER       0x1E
#define DW1000_LEN_TX_POWER   4

#define DW1000_AGC_TUNE1_SUB 0x04
#define DW1000_AGC_TUNE2_SUB 0x0C
#define DW1000_AGC_TUNE3_SUB 0x12
#define DW1000_LEN_AGC_TUNE1 2
#define DW1000_LEN_AGC_TUNE2 4
#define DW1000_LEN_AGC_TUNE3 2

#define DW1000_DRX_TUNE0b_SUB 0x02
#define DW1000_DRX_TUNE1a_SUB 0x04
#define DW1000_DRX_TUNE1b_SUB 0x06
#define DW1000_DRX_TUNE2_SUB  0x08
#define DW1000_DRX_TUNE4H_SUB 0x26
#define DW1000_LEN_DRX_TUNE0b 2
#define DW1000_LEN_DRX_TUNE1a 2
#define DW1000_LEN_DRX_TUNE1b 2
#define DW1000_LEN_DRX_TUNE2  4
#define DW1000_LEN_DRX_TUNE4H 2

#define DW1000_LEN_LDE_CFG1   1
#define DW1000_LEN_LDE_CFG2   2
#define DW1000_LEN_LDE_REPC   2
#define DW1000_LEN_LDE_RXANTD 2

#define DW1000_RF_RXCTRLH_SUB 0x0B
#define DW1000_RF_TXCTRL_SUB  0x0C
#define DW1000_LEN_RF_RXCTRLH 1
#define DW1000_LEN_RF_TXCTRL  4

#define DW1000_TC_PGDELAY_SUB 0x0B
#define DW1000_LEN_TC_PGDELAY 1

#define DW1000_FS_PLLCFG_SUB  0x07
#define DW1000_FS_PLLTUNE_SUB 0x0B
#define DW1000_FS_XTALT_SUB   0x0E
#define DW1000_LEN_FS_PLLCFG  4
#define DW1000_LEN_FS_PLLTUNE 1
#define DW1000_LEN_FS_XTALT   1

#define DW1000_TX_ANTD        0x18
#define DW1000_LEN_TX_ANTD    5

/* OTP / PMSC lengths (used by the LDE microcode load) */
#define DW1000_LEN_OTP_CTRL   2
#define DW1000_LEN_PMSC_CTRL0 4

/* frame length limits */
#define DW1000_LEN_UWB_FRAMES     127
#define DW1000_LEN_EXT_UWB_FRAMES 1023

/* ============================ timestamps ============================ */

#define DW1000_LEN_STAMP     5
#define DW1000_TX_STAMP_SUB  0x00
#define DW1000_RX_STAMP_SUB  0x00

/* ============================ RX quality (RX_TIME / RX_FQUAL) ============================ */

#define DW1000_FP_AMPL1_SUB  0x07   /* RX_TIME: first path amplitude 1 */
#define DW1000_LEN_FP_AMPL1  2
#define DW1000_STD_NOISE_SUB 0x00   /* RX_FQUAL: standard noise       */
#define DW1000_FP_AMPL2_SUB  0x02   /* RX_FQUAL: first path amplitude 2 */
#define DW1000_FP_AMPL3_SUB  0x04   /* RX_FQUAL: first path amplitude 3 */
#define DW1000_CIR_PWR_SUB   0x06   /* RX_FQUAL: CIR power             */
#define DW1000_LEN_STD_NOISE 2
#define DW1000_LEN_FP_AMPL2  2
#define DW1000_LEN_FP_AMPL3  2
#define DW1000_LEN_CIR_PWR   2

/* ============================ preamble codes ============================ */

#define DW1000_PREAMBLE_CODE_16MHZ_1  1
#define DW1000_PREAMBLE_CODE_16MHZ_2  2
#define DW1000_PREAMBLE_CODE_16MHZ_3  3
#define DW1000_PREAMBLE_CODE_16MHZ_4  4
#define DW1000_PREAMBLE_CODE_16MHZ_5  5
#define DW1000_PREAMBLE_CODE_16MHZ_6  6
#define DW1000_PREAMBLE_CODE_16MHZ_7  7
#define DW1000_PREAMBLE_CODE_16MHZ_8  8
#define DW1000_PREAMBLE_CODE_64MHZ_9  9
#define DW1000_PREAMBLE_CODE_64MHZ_10 10
#define DW1000_PREAMBLE_CODE_64MHZ_11 11
#define DW1000_PREAMBLE_CODE_64MHZ_12 12
#define DW1000_PREAMBLE_CODE_64MHZ_17 17
#define DW1000_PREAMBLE_CODE_64MHZ_18 18
#define DW1000_PREAMBLE_CODE_64MHZ_19 19
#define DW1000_PREAMBLE_CODE_64MHZ_20 20

/* ============================ PAC size / frame length ============================ */

#define DW1000_PAC_SIZE_8    8
#define DW1000_PAC_SIZE_16   16
#define DW1000_PAC_SIZE_32   32
#define DW1000_PAC_SIZE_64   64

#define DW1000_FRAME_LENGTH_NORMAL   0x00
#define DW1000_FRAME_LENGTH_EXTENDED 0x03

#ifdef __cplusplus
}
#endif

#endif /* DW1000_REGS_H */
