#include <asm/types.h>
#include <config.h>

#ifdef CONFIG_DRIVER_W5500

#define SR_NUM		8
#define SR0			0
#define	SR1			1
#define	SR2			2
#define	SR3			3
#define	SR4			4
#define	SR5			5
#define	SR6			6
#define	SR7			7

#define MR          0x0000
#define GAR0        0x0001
#define GAR1        0x0002
#define GAR2        0x0003
#define GAR3        0x0004
#define SUBR0       0x0005
#define SUBR1       0x0006
#define SUBR2       0x0007
#define SUBR3       0x0008
#define SHAR0       0x0009
#define SHAR1       0x000A
#define SHAR2       0x000B
#define SHAR3       0x000C
#define SHAR4       0x000D
#define SHAR5       0x000E
#define SIPR0       0x000F
#define SIPR1       0x0010
#define SIPR2       0x0011
#define SIPR3       0x0012
#define INTLEVEL0   0x0013
#define INTLEVEL1   0x0014
#define IR          0x0015
#define IMR         0x0016
#define SIR         0x0017
#define SIMR        0x0018
#define RTR0        0x0019
#define RTR1        0x001A
#define RCR         0x001B
#define PTIMER      0x001C
#define PMAGIC      0x001D
#define PHAR0       0x001E
#define PHAR1       0x001F
#define PHAR2       0x0020
#define PHAR3       0x0021
#define PHAR4       0x0022
#define PHAR5       0x0023
#define PSID0       0x0024
#define PSID1       0x0025
#define PMRU0       0x0026
#define PMEU1       0x0027
#define UIPR0       0x0028
#define UIPR1       0x0029
#define UIPR2       0x002A
#define UIPR3       0x002B
#define UPORT0      0x002C
#define UPORT1      0x002D
#define PHYCFGR     0x002E
#define VERSIONR    0x0039

#define S0_BASE     0x0000
#define S1_BASE     0x0000
#define S2_BASE     0x0000
#define S3_BASE     0x0000

#define Sn_MR       0x0000
#define Sn_CR       0x0001
#define Sn_IR       0x0002
#define Sn_SR       0x0003
#define Sn_PORT0    0x0004
#define Sn_PORT1    0x0005
#define Sn_DHAR0    0x0006
#define Sn_DHAR1    0x0007
#define Sn_DHAR2    0x0008
#define Sn_DHAR3    0x0009
#define Sn_DHAR4    0x000A
#define Sn_DHAR5    0x000B
#define Sn_DIPR0    0x000C
#define Sn_DIPR1    0x000D
#define Sn_DIPR2    0x000E
#define Sn_DIPR3    0x000F
#define Sn_DPORT0   0x0010
#define Sn_DPORT1   0x0011
#define Sn_MSSR0    0x0012
#define Sn_MSSR1    0x0013
#define Sn_PROTO    0x0014
#define Sn_TOS      0x0015
#define Sn_TTL      0x0016
#define Sn_RXBUF_SIZE   0x001E
#define Sn_TXBUF_SIZE   0x001F
#define Sn_TX_FSR0  0x0020
#define Sn_TX_FSR1  0x0021
#define Sn_TX_RD0   0x0022
#define Sn_TX_RD1   0x0023
#define Sn_TX_WR0   0x0024
#define Sn_TX_WR1   0x0025
#define Sn_RX_RSR0  0x0026
#define Sn_RX_RSR1  0x0027
#define Sn_RX_RD0   0x0028
#define Sn_RX_RD1   0x0029
#define Sn_RX_WR0   0x002A
#define Sn_RX_WR1   0x002B
#define Sn_IMR      0x002C
#define Sn_FRAG0    0x002D
#define Sn_FRAG1    0x002E
#define Sn_KPALVTR  0x002F

#define MR_RST              (1<<7)
#define MR_PB               (1<<4)
#define MR_PPPOE            (1<<3)
#define MR_AI               (1<<1)
#define MR_IND              (1<<0)

#define IR_CONFLICT         (1<<7)
#define IR_UNREACH          (1<<6)
#define IR_PPPoE            (1<<5)
#define IR_SOCK3            (1<<3)
#define IR_SOCK2            (1<<2)
#define IR_SOCK1            (1<<1)
#define IR_SOCK0            (1<<0)

#define Sn_MR_CLOSE         0x00
#define Sn_MR_TCP           0x01
#define Sn_MR_UDP           0x02
#define Sn_MR_IPRAW         0x03
#define Sn_MR_MACRAW        0x04
#define Sn_MR_PPPoE         0x05

#define Sn_MR_ND            (1<<5)
#define Sn_MR_MC            (1<<5)
#define Sn_MR_MULTI         (1<<7)

#define Sn_CR_OPEN          0x01
#define Sn_CR_LISTEN        0x02
#define Sn_CR_CONNECT       0x04
#define Sn_CR_DISCON        0x08
#define Sn_CR_CLOSE         0x10
#define Sn_CR_SEND          0x20
#define Sn_CR_SEND_MAC      0x21
#define Sn_CR_SEND_KEEP     0x22
#define Sn_CR_RECV          0x40

#define Sn_IR_SEND_OK       (1<<4)
#define Sn_IR_TIMEOUT       (1<<3)
#define Sn_IR_RECV          (1<<2)
#define Sn_IR_DISCON        (1<<1)
#define Sn_IR_CON           (1<<0)

#define Sn_SR_CLOSED        0x00
#define Sn_SR_INIT          0x13
#define Sn_SR_LISTEN        0x14
#define Sn_SR_ESTABLISHED   0x17
#define Sn_SR_CLOSE_WAIT    0x1C
#define Sn_SR_UDP           0x22
//#define Sn_SR_IPRAW         0x32
#define Sn_SR_MACRAW        0x02
//#define Sn_SR_PPPoE         0x5F

#define Sn_SR_SYNSENT       0x15
#define Sn_SR_SYNRECV       0x16
#define Sn_SR_FIN_WAIT      0x18
#define Sn_SR_CLOSING       0x1A
#define Sn_SR_TIME_WAIT     0x1B
#define Sn_SR_LAST_ACK      0x1D
//#define Sn_SR_ARP           0x11
//#define Sn_SR_ARP2          0x21
//#define Sn_SR_ARP3          0x31

#define Sn_TOS_DEFAULT              0x00
#define Sn_TOS_MIN_MONETARY_COST    0x01
#define Sn_TOS_MAX_RELIABILITY      0x02
#define Sn_TOS_MAX_THROUGHPUT       0x04
#define Sn_TOS_MIN_DELAY            0x08
#define Sn_TOS_MAX_SECURITY         0x0f

#define PHYCFGR_RST                  ~(1<<7)  //< For PHY reset, must operate AND mask.
#define PHYCFGR_OPMD                 (1<<6)   // Configre PHY with OPMDC value
#define PHYCFGR_OPMDC_ALLA           (7<<3)
#define PHYCFGR_OPMDC_PDOWN          (6<<3)
#define PHYCFGR_OPMDC_NA             (5<<3)
#define PHYCFGR_OPMDC_100FA          (4<<3)
#define PHYCFGR_OPMDC_100F           (3<<3)
#define PHYCFGR_OPMDC_100H           (2<<3)
#define PHYCFGR_OPMDC_10F            (1<<3)
#define PHYCFGR_OPMDC_10H            (0<<3)
#define PHYCFGR_DPX_FULL             (1<<2)
#define PHYCFGR_DPX_HALF             (0<<2)
#define PHYCFGR_SPD_100              (1<<1)
#define PHYCFGR_SPD_10               (0<<1)
#define PHYCFGR_LNK_ON               (1<<0)
#define PHYCFGR_LNK_OFF              (0<<0)

#endif /* CONFIG_DRIVER_W5500 */

