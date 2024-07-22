/*
 * @Author       : Eacher
 * @Date         : 2024-07-10 10:02:04
 * @LastEditTime : 2024-07-19 14:24:12
 * @LastEditors  : Eacher
 * --------------------------------------------------------------------------------<
 * @Description  : 
 * --------------------------------------------------------------------------------<
 * @FilePath     : /zephyrproject/veryark/application/include/app/isotp/isotp.h
 */

#ifndef __VERYARK_ISOTP_H_
#define __VERYARK_ISOTP_H_
#include <zephyr/kernel.h>
#include <zephyr/net/buf.h>

/**
 * @name CAN frame flags
 * @anchor CAN_FRAME_FLAGS
 *
 * @{
 */

/** Frame uses extended (29-bit) CAN ID */
#define ISOTP_PKG_IDE BIT(0)

/** Frame is a Remote Transmission Request (RTR) */
#define ISOTP_PKG_RTR BIT(1)

/** Frame uses CAN FD format (FDF) */
#define ISOTP_PKG_FDF BIT(2)

/** Frame uses CAN FD Baud Rate Switch (BRS). Only valid in combination with ``ISOTP_PKG_FDF``. */
#define ISOTP_PKG_BRS BIT(3)

/** CAN FD Error State Indicator (ESI). Indicates that the transmitting node is in error-passive
 * state. Only valid in combination with ``ISOTP_PKG_FDF``.
 */
#define ISOTP_PKG_ESI BIT(4)

/** Message uses ISO-TP extended addressing (first payload byte of CAN frame) */
#define ISOTP_PKG_EXT_ADDR BIT(5)

/** @} */


/** Completed successfully */
#define ISOTP_N_OK              0

/** Ar/As has timed out */
#define ISOTP_N_TIMEOUT_A      -1

/** Reception of next FC has timed out */
#define ISOTP_N_TIMEOUT_BS     -2

/** Cr has timed out */
#define ISOTP_N_TIMEOUT_CR     -3

/** Unexpected sequence number */
#define ISOTP_N_WRONG_SN       -4

/** Invalid flow status received*/
#define ISOTP_N_INVALID_FS     -5

/** Unexpected PDU received */
#define ISOTP_N_UNEXP_PDU      -6

/** Maximum number of WAIT flowStatus PDUs exceeded */
#define ISOTP_N_WFT_OVRN       -7

/** FlowStatus OVFLW PDU was received */
#define ISOTP_N_BUFFER_OVERFLW -8

/** General error */
#define ISOTP_N_ERROR          -9

/** Implementation specific errors */

/** Can't bind or send because the CAN device has no filter left*/
#define ISOTP_NO_FREE_FILTER    -10

/** No net buffer left to allocate */
#define ISOTP_NO_NET_BUF_LEFT   -11

/** Not sufficient space in the buffer left for the data */
#define ISOTP_NO_BUF_DATA_LEFT  -12

/** No context buffer left to allocate */
#define ISOTP_NO_CTX_LEFT       -13

/** Timeout for recv */
#define ISOTP_RECV_TIMEOUT      -14





typedef void (*isotp_callback)(int error_nr, void *arg);

struct isotp_pkg_id {
	/**
	 * CAN identifier
	 *
	 * If ISO-TP fixed addressing is used, isotp_bind ignores SA and
	 * priority sections and modifies TA section in flow control frames.
	 */
	union {
		uint32_t std_id  : 11;
		uint32_t ext_id  : 29;
	};
	/** ISO-TP extended address (if used) */
	uint8_t ext_addr;
	/** Flags. @see @ref ISOTP_PKG_FLAGS. */
	uint8_t flags;
	/**
	 * ISO-TP frame data length (TX_DL for TX address or RX_DL for RX address).
	 *
	 * Valid values are 8 for classical CAN or 8, 12, 16, 20, 24, 32, 48 and 64 for CAN-FD.
	 *
	 * 0 will be interpreted as 8 or 64 (if ISOTP_MSG_FDF is set).
	 *
	 * The value for incoming transmissions (RX_DL) is determined automatically based on the
	 * received first frame and does not need to be set during initialization.
	 */
	uint8_t dlc;
};

struct isotp_fc_opts {
	uint8_t bs;    /**< Block size. Number of CF PDUs before next CF is sent */
	uint8_t stmin; /**< Minimum separation time. Min time between frames */
};

struct tpconn {
	struct can_frame *focus;
	struct k_timer timer;
	uint32_t idx;
	uint32_t len;
	uint32_t state;
	uint8_t bs;
	uint8_t sn;
	struct net_buf *next;
	struct net_buf *buf;
	struct isotp_pkg_id addr;
	struct isotp_fc_opts fc;
};

struct linux_isotp_sock {
	const struct device *can_dev;
	int close;
	struct k_mutex mutex;
	struct k_work_delayable work;
	k_timeout_t tx_gap;
	int filter_id;
	struct tpconn rx, tx;
	struct k_fifo fifo;
	isotp_callback callback;
	void *args;
};

struct linux_isotp_sock *isotp_init(const struct device *can_dev, struct isotp_pkg_id rxaddr, struct isotp_pkg_id txaddr, struct isotp_fc_opts rxfc);
void isotp_close(struct linux_isotp_sock *sk);
int64_t isotp_recv(struct linux_isotp_sock *sk, struct net_buf **buffer, k_timeout_t timeout);
int isotp_send(struct linux_isotp_sock *sk, uint8_t *data, size_t len, isotp_callback complete_cb, void *cb_arg);

#endif
