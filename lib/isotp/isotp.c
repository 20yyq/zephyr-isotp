/*
 * @Author       : Eacher
 * @Date         : 2024-07-10 10:00:22
 * @LastEditTime : 2024-07-19 15:19:27
 * @LastEditors  : Eacher
 * --------------------------------------------------------------------------------<
 * @Description  : 
 * --------------------------------------------------------------------------------<
 * @FilePath     : /zephyrproject/veryark/application/lib/isotp/isotp.c
 */
#include <stdbool.h>
#include <string.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include <app/isotp/isotp.h>

/* N_PCI type values in bits 7-4 of N_PCI bytes */
#define N_PCI_SF 0x00	/* single frame */
#define N_PCI_FF 0x10	/* first frame */
#define N_PCI_CF 0x20	/* consecutive frame */
#define N_PCI_FC 0x30	/* flow control */

#define N_PCI_SZ 1		/* size of the PCI byte #1 */
#define SF_PCI_SZ4 1	/* size of SingleFrame PCI including 4 bit SF_DL */
#define SF_PCI_SZ8 2	/* size of SingleFrame PCI including 8 bit SF_DL */
#define FF_PCI_SZ12 2	/* size of FirstFrame PCI including 12 bit FF_DL */
#define FF_PCI_SZ32 6	/* size of FirstFrame PCI including 32 bit FF_DL */
#define FC_CONTENT_SZ 3	/* flow control content size in byte (FS/BS/STmin) */
#define MAX_FF_DL12 0xFFF /* max 12 bit data length FF_DL */

// #define ISOTP_CHECK_PADDING (CAN_ISOTP_CHK_PAD_LEN | CAN_ISOTP_CHK_PAD_DATA)

/* Flow Status given in FC frame */
#define ISOTP_FC_CTS 0		/* clear to send */
#define ISOTP_FC_WT 1		/* wait */
#define ISOTP_FC_OVFLW 2	/* overflow */

#define ISOTP_STMIN_MS_MAX   0x7F

enum {
	ISOTP_IDLE = 0,
	ISOTP_WAIT_FIRST_FC,
	ISOTP_WAIT_FC,
	ISOTP_WAIT_FC_START,
	ISOTP_WAIT_DATA,
	ISOTP_WAIT_DATA_END,
	ISOTP_SENDING,
	ISOTP_SEND_END,
	ISOTP_RX_STATE_SEND_FC_ERR,
	ISOTP_RX_POOL_ALLOC_BUFFER_ERR,
	ISOTP_TX_STATE_SEND_ERR,
	VISOTP_RX_STATE_WAIT_FF_SF
};

struct isotp_frame
{
	struct can_frame frame;
	uint64_t send_num;
	struct linux_isotp_sock *sk;
};

struct callback_timer {
	isotp_callback callback;
	int error;
	void *args;
	struct k_timer timer;
};

K_MUTEX_DEFINE(sk_list_mutex);
static struct linux_isotp_sock *sk_list[CONFIG_CAN_MAX_FILTER] = {NULL};

LOG_MODULE_REGISTER(isotp, CONFIG_VISOTP_LOG_LEVEL);

NET_BUF_POOL_DEFINE(isotp_receive_pool, CONFIG_VISOTP_RX_BUF_COUNT, CONFIG_VISOTP_BUF_SIZE, sizeof(uint32_t), NULL);
NET_BUF_POOL_DEFINE(isotp_send_pool, CONFIG_VISOTP_TX_BUF_COUNT, CONFIG_VISOTP_BUF_SIZE, sizeof(uint32_t), NULL);

K_MEM_SLAB_DEFINE(isotp_frame_slab, sizeof(struct isotp_frame), CONFIG_CAN_MAX_FILTER * 2, sizeof(uint32_t));
K_MEM_SLAB_DEFINE(isotp_sock_slab, sizeof(struct linux_isotp_sock), CONFIG_CAN_MAX_FILTER, sizeof(uint32_t));
K_MEM_SLAB_DEFINE(isotp_callback_timer_slab, sizeof(struct callback_timer), CONFIG_CAN_MAX_FILTER, sizeof(uint32_t));

static inline struct net_buf *from_pool_add_net_buf_data(struct net_buf_pool *pool, struct net_buf *buf, uint8_t *data, size_t len)
{
	uint16_t max, idx, item;
	item = 0;
	do
	{
		max = net_buf_tailroom(buf);
		for (idx = 0; idx < max; idx++)
		{
			net_buf_add_u8(buf, data[item]);
			item++;
			len--;
			if (!len)
				break;
		}
		if (idx == max)
		{
			buf->frags = net_buf_alloc_fixed(pool, K_NO_WAIT);
			buf = buf->frags;
			if (!buf)
			{
				LOG_ERR("No free alloc to append data len [%d] item [%d] idx [%d]", len, item, idx);
				break;
			}
		}
	} while (len);
	return buf;
}

static void callback_timer_handler(struct k_timer *arg)
{
	struct callback_timer *timer = CONTAINER_OF(arg, struct callback_timer, timer);
	timer->callback(timer->error, timer->args);
	k_mem_slab_free(&isotp_callback_timer_slab, timer);
}

static inline void inline_callback_timer(isotp_callback complete_cb, int error_nr, void *cb_arg)
{
	struct callback_timer *timer;
	if (!complete_cb || k_mem_slab_alloc(&isotp_callback_timer_slab, (void **)&timer, K_NO_WAIT))
	{
		if (complete_cb)
			LOG_ERR("isotp_callback_timer_slab k_mem_slab_alloc: NULL");
		return;
	}
	timer->callback = complete_cb;
	timer->args = cb_arg;
	timer->error = error_nr;
	k_timer_init(&timer->timer, callback_timer_handler, NULL);
	k_timer_start(&timer->timer, K_NO_WAIT, K_NO_WAIT);
}

static void receive_timeout_handler(struct k_timer *timer)
{
	struct linux_isotp_sock *sk = CONTAINER_OF(CONTAINER_OF(timer, struct tpconn, timer), struct linux_isotp_sock, rx);

	switch (sk->rx.state)
	{
	case ISOTP_RX_POOL_ALLOC_BUFFER_ERR:
		LOG_ERR("ISOTP_RX_POOL_ALLOC_BUFFER_ERR");
		sk->rx.state = VISOTP_RX_STATE_WAIT_FF_SF;
		net_buf_unref(sk->rx.buf);
		break;

	case ISOTP_RX_STATE_SEND_FC_ERR:
		LOG_ERR("ISOTP_RX_STATE_SEND_FC_ERR");
		sk->rx.state = VISOTP_RX_STATE_WAIT_FF_SF;
		net_buf_unref(sk->rx.buf);
		break;

	case ISOTP_WAIT_DATA:
		LOG_ERR("ISOTP_WAIT_DATA timeout");
		sk->rx.state = VISOTP_RX_STATE_WAIT_FF_SF;
		net_buf_unref(sk->rx.buf);
		break;
	case ISOTP_WAIT_DATA_END:
		sk->rx.state = VISOTP_RX_STATE_WAIT_FF_SF;
		net_buf_put(&sk->fifo, sk->rx.buf);
		break;
	}
	k_mutex_lock(&sk->mutex, K_FOREVER);
	if (sk->close)
	{
		can_remove_rx_filter(sk->can_dev, sk->filter_id);
		sk->filter_id = -1;
	}
	k_mutex_unlock(&sk->mutex);
	return;
}

static inline int isotp_send_cf(struct linux_isotp_sock *sk);

static void send_timeout_handler(struct k_timer *timer)
{
	struct linux_isotp_sock *sk = CONTAINER_OF(CONTAINER_OF(timer, struct tpconn, timer), struct linux_isotp_sock, tx);

	switch (sk->tx.state)
	{
	case ISOTP_TX_STATE_SEND_ERR:
		inline_callback_timer(sk->callback, -ISOTP_TX_STATE_SEND_ERR, sk->args);
		sk->callback = NULL;
		sk->tx.state = ISOTP_IDLE;
		net_buf_unref(sk->tx.buf);
		break;

	case ISOTP_WAIT_FIRST_FC:
		LOG_ERR("ISOTP_WAIT_FIRST_FC timeout");
		inline_callback_timer(sk->callback, -ISOTP_WAIT_FIRST_FC, sk->args);
		sk->callback = NULL;
		sk->tx.state = ISOTP_IDLE;
		net_buf_unref(sk->tx.buf);
		break;

	case ISOTP_WAIT_FC:
		LOG_ERR("ISOTP_WAIT_FC timeout");
		inline_callback_timer(sk->callback, -ISOTP_WAIT_FC, sk->args);
		sk->callback = NULL;
		sk->tx.state = ISOTP_IDLE;
		net_buf_unref(sk->tx.buf);
		break;
	case ISOTP_SENDING:
		if (isotp_send_cf(sk))
		{
			inline_callback_timer(sk->callback, -ISOTP_TX_STATE_SEND_ERR, sk->args);
			sk->callback = NULL;
			sk->tx.state = ISOTP_IDLE;
			net_buf_unref(sk->tx.buf);
		}
		break;
	}
}

static void isotp_send_fc_can_cb(const struct device *dev, int error, void *arg)
{
	struct isotp_frame *frame = (struct isotp_frame *)arg;
	if (!frame->send_num)
	{
		LOG_ERR("isotp_send_fc_can_cb send_num is zero");
		return;
	}
	frame->send_num--;
	if (error)
	{
		if (!frame->send_num)
		{
			k_timer_stop(&frame->sk->rx.timer);
			frame->sk->rx.state = ISOTP_RX_STATE_SEND_FC_ERR;
			k_timer_start(&frame->sk->rx.timer, K_NO_WAIT, K_NO_WAIT);
		}
		LOG_ERR("send fc error: [%d] send_num [%lld]", error, frame->send_num);
	}
}

static inline int isotp_send_frame(can_tx_callback_t callback, struct isotp_frame *frame) 
{
	frame->frame.id = frame->sk->tx.addr.ext_id;
	frame->frame.flags = (frame->sk->tx.addr.flags & ISOTP_PKG_IDE ? CAN_FRAME_IDE : 0) |
		       (frame->sk->tx.addr.flags & ISOTP_PKG_FDF ? CAN_FRAME_FDF : 0) |
		       (frame->sk->tx.addr.flags & ISOTP_PKG_BRS ? CAN_FRAME_BRS : 0);
	frame->send_num++;
	int ret = can_send(frame->sk->can_dev, &frame->frame, K_MSEC(CONFIG_VISOTP_A_TIMEOUT), callback, frame);
	if (ret)
	{
		frame->send_num--;
		LOG_ERR("Can't send Frame, (%d) send_num [%lld]", ret, frame->send_num);
	}
	return ret;
}

static int isotp_send_fc(struct isotp_frame *frame, uint8_t flowstatus)
{
	int item = frame->sk->tx.addr.flags & ISOTP_PKG_EXT_ADDR ? 1 : 0;
	if (item)
		frame->frame.data[item - 1] = frame->sk->tx.addr.ext_addr;
	frame->frame.data[item++] = N_PCI_FC | flowstatus;
	frame->frame.data[item++] = frame->sk->rx.fc.bs;
	frame->frame.data[item++] = frame->sk->rx.fc.stmin;
	frame->frame.dlc = item;
	return isotp_send_frame(isotp_send_fc_can_cb, frame);
}

static inline void isotp_rcv_fc(struct linux_isotp_sock *sk, struct can_frame *cf, int item)
{
	uint32_t state = sk->tx.state;
	if (sk->tx.state != ISOTP_WAIT_FC && sk->tx.state != ISOTP_WAIT_FIRST_FC)
	{
		LOG_ERR("sk->tx.state != ISOTP_WAIT_FIRST_FC state: [%d] len [%d] idx [%d]", sk->tx.state, sk->tx.len, sk->tx.idx);
		return;
	}
	k_timer_stop(&sk->tx.timer);
	sk->tx.state = ISOTP_SENDING;
	sk->tx.bs = 1;
	/* get communication parameters only from the first FC frame */
	if (state == ISOTP_WAIT_FIRST_FC)
	{
		sk->tx.fc.bs = cf->data[item + 1];
		sk->tx.fc.stmin = cf->data[item + 2];

		/* fix wrong STmin values according spec */
		if (sk->tx.fc.stmin > 0xF9 || (sk->tx.fc.stmin > ISOTP_STMIN_MS_MAX && sk->tx.fc.stmin < 0xF1))
		{
			sk->tx.fc.stmin = ISOTP_STMIN_MS_MAX;
			sk->tx_gap = K_MSEC(ISOTP_STMIN_MS_MAX);
		}
		else if (sk->tx.fc.stmin < 0x80)
			sk->tx_gap = K_MSEC(sk->tx.fc.stmin);
		else
			sk->tx_gap = K_USEC((sk->tx.fc.stmin - 0xF0) * 100U);
	}

	k_timeout_t tx_gap = K_NO_WAIT;
	if (cf->dlc < item + FC_CONTENT_SZ)
	{
		LOG_ERR("cf->dlc < item + FC_CONTENT_SZ dlc: [%d] 1=[%d],2=[%d],3=[%d],4=[%d]", cf->dlc, cf->data[0], cf->data[1], cf->data[2], cf->data[3]);
		sk->tx.state = ISOTP_TX_STATE_SEND_ERR;
		k_timer_start(&sk->tx.timer, tx_gap, K_NO_WAIT);
		return;
	}

	switch (cf->data[item] & 0x0F)
	{
	case ISOTP_FC_CTS:
		LOG_DBG("starting txtimer for sending");
		/* start cyclic timer for sending CF frame */
		break;

	case ISOTP_FC_WT:
		sk->tx.state = state;
		tx_gap = K_MSEC(CONFIG_VISOTP_BS_TIMEOUT);
		LOG_DBG("starting waiting for next FC");
		/* start timer to wait for next FC frame */
		break;
	case ISOTP_FC_OVFLW:
		LOG_ERR("overflow in receiver side");
	default:
		/* stop this tx job */
		sk->tx.state = ISOTP_TX_STATE_SEND_ERR;
	}
	k_timer_start(&sk->tx.timer, tx_gap, K_NO_WAIT);
}

static inline void isotp_rcv_sf(struct linux_isotp_sock *sk, struct can_frame *cf, int item)
{
	k_mutex_lock(&sk->mutex, K_FOREVER);
	bool close = sk->close;
	k_mutex_unlock(&sk->mutex);
	if (close)
	{
		can_remove_rx_filter(sk->can_dev, sk->filter_id);
		sk->filter_id = -1;
		return;
	}
	if (sk->rx.state != VISOTP_RX_STATE_WAIT_FF_SF)
	{
		LOG_ERR("sk->rx.state != VISOTP_RX_STATE_WAIT_SF state: [%d]", sk->rx.state);
		return;
	}
	sk->rx.state = ISOTP_IDLE;
	sk->rx.len = cf->data[item] & 0x0F;
	int sf_dl = 1;
	if (can_dlc_to_bytes(cf->dlc) == CAN_MAX_DLEN)
	{
		sf_dl = 2;
		sk->rx.len = cf->data[SF_PCI_SZ4 + item];
	}
	if (!sk->rx.len || sk->rx.len > cf->dlc)
	{
		LOG_ERR("!sf_dl || sf_dl > cf->dlc sf_dl:[%d] dlc: [%d]", sk->rx.len, cf->dlc);
		sk->rx.state = VISOTP_RX_STATE_WAIT_FF_SF;
		return;
	}
	sk->rx.buf = net_buf_alloc_fixed(&isotp_receive_pool, K_NO_WAIT);
	if (!sk->rx.buf)
	{
		LOG_ERR("pool_alloc_buffer net buf == NULL");
		sk->rx.state = VISOTP_RX_STATE_WAIT_FF_SF;
		return;
	}
	net_buf_add_le32(sk->rx.buf, sk->rx.len);
	from_pool_add_net_buf_data(&isotp_receive_pool, sk->rx.buf, &cf->data[item + sf_dl], sk->rx.len);
	net_buf_put(&sk->fifo, sk->rx.buf);
	sk->rx.state = VISOTP_RX_STATE_WAIT_FF_SF;
}

static inline void isotp_rcv_ff(struct linux_isotp_sock *sk, struct can_frame *cf, int item)
{
	k_mutex_lock(&sk->mutex, K_FOREVER);
	bool close = sk->close;
	k_mutex_unlock(&sk->mutex);
	if (close)
	{
		can_remove_rx_filter(sk->can_dev, sk->filter_id);
		sk->filter_id = -1;
		return;
	}
	if (sk->rx.state != VISOTP_RX_STATE_WAIT_FF_SF)
	{
		LOG_ERR("sk->rx.state != VISOTP_RX_STATE_WAIT_FF state: [%d] len [%d] idx [%d]", sk->rx.state, sk->rx.len, sk->rx.idx);
		return;
	}
	sk->rx.state = ISOTP_WAIT_DATA;

	/* get the used sender LL_DL from the (first) CAN frame data length */
	sk->rx.addr.dlc = can_dlc_to_bytes(cf->dlc);

	/* the first frame has to use the entire frame up to LL_DL length */
	if (cf->dlc != sk->rx.addr.dlc)
	{
		LOG_ERR("cf->dlc != sk->rx.addr.dlc dlc: [%d] ll_dl: [%d]", cf->dlc, sk->rx.addr.dlc);
		sk->rx.state = VISOTP_RX_STATE_WAIT_FF_SF;		
		return;
	}
	/* get the FF_DL */
	sk->rx.len = (cf->data[item++] & 0x0F) << 8;
	sk->rx.len += cf->data[item++];
	/* Check for FF_DL escape sequence supporting 32 bit PDU length */
	if (!sk->rx.len)
	{
		/* FF_DL = 0 => get real length from next 4 bytes */
		sk->rx.len = cf->data[item++] << 24;
		sk->rx.len += cf->data[item++] << 16;
		sk->rx.len += cf->data[item++] << 8;
		sk->rx.len += cf->data[item++];
	}
	/* max 32 bit data length */
	if (sk->rx.len > (CONFIG_VISOTP_RX_BUF_COUNT * CONFIG_VISOTP_BUF_SIZE))
	{
		/* send FC frame with overflow status */
		isotp_send_fc((struct isotp_frame *)sk->rx.focus, ISOTP_FC_OVFLW);
		sk->rx.state = VISOTP_RX_STATE_WAIT_FF_SF;
		LOG_WRN("ISOTP_FC_OVFLW len: [%d]", sk->rx.len);
		return;
	}

	/* copy the first received data bytes */
	sk->rx.idx = sk->rx.addr.dlc - item;
	sk->rx.bs = 0;
	sk->rx.buf = net_buf_alloc_fixed(&isotp_receive_pool, K_NO_WAIT);
	if (!sk->rx.buf)
	{
		LOG_ERR("pool_alloc_buffer net buf == NULL");
		sk->rx.state = VISOTP_RX_STATE_WAIT_FF_SF;
		return;
	}
	net_buf_add_le32(sk->rx.buf, sk->rx.len);
	sk->rx.next = from_pool_add_net_buf_data(&isotp_receive_pool, sk->rx.buf, &cf->data[item], sk->rx.idx);
	sk->rx.sn = 1;
	/* send our first FC frame */
	isotp_send_fc((struct isotp_frame *)sk->rx.focus, ISOTP_FC_CTS);
	k_timer_start(&sk->rx.timer, K_MSEC(CONFIG_VISOTP_A_TIMEOUT), K_NO_WAIT);
}

static inline void isotp_rcv_cf(struct linux_isotp_sock *sk, struct can_frame *cf, int ae)
{
	if (sk->rx.state != ISOTP_WAIT_DATA)
	{
		LOG_ERR("sk->rx.state != ISOTP_WAIT_DATA state: [%d]", sk->rx.state);
		return;
	}
	/* CFs are never longer than the FF */
	if (cf->dlc > sk->rx.addr.dlc)
	{
		LOG_WRN("cf->dlc > sk->rx.addr.dlc len: [%d]", cf->dlc);
		return;
	}

	/* CFs have usually the LL_DL length */
	if (cf->dlc < sk->rx.addr.dlc)
	{
		/* this is only allowed for the last CF */
		if (sk->rx.len - sk->rx.idx > cf->dlc - ae - N_PCI_SZ)
		{
			LOG_WRN("sk->rx.len - sk->rx.idx len: [%d] [%d]", sk->rx.len - sk->rx.idx, cf->dlc - ae - N_PCI_SZ);
			return;
		}
	}
	if ((cf->data[ae] & 0x0F) != sk->rx.sn)
	{
		LOG_DBG("wrong sn %d. expected %d.", cf->data[ae] & 0x0F, sk->rx.sn);
		return;
	}
	k_timer_stop(&sk->rx.timer);
	sk->rx.sn++;
	sk->rx.sn %= 16;
	sk->rx.next = from_pool_add_net_buf_data(&isotp_receive_pool, sk->rx.next, &cf->data[ae + N_PCI_SZ], cf->dlc - ae - N_PCI_SZ);
	if (!sk->rx.next)
	{
		sk->rx.state = ISOTP_RX_POOL_ALLOC_BUFFER_ERR;
		k_timer_start(&sk->rx.timer, K_NO_WAIT, K_NO_WAIT);
		return;
	}
	sk->rx.idx += cf->dlc - ae - N_PCI_SZ;

	if (sk->rx.idx >= sk->rx.len)
	{
		/* we are done */
		sk->rx.state = ISOTP_WAIT_DATA_END;
		k_timer_start(&sk->rx.timer, K_NO_WAIT, K_NO_WAIT);
		return;
	}

	/* perform blocksize handling, if enabled */
	if (sk->rx.fc.bs && ++sk->rx.bs == sk->rx.fc.bs)
	{
		/* we reached the specified blocksize so->rxfc.bs */
		sk->rx.bs = 0;
		isotp_send_fc((struct isotp_frame *)sk->rx.focus, ISOTP_FC_CTS);
	}
	k_timer_start(&sk->rx.timer, K_MSEC(CONFIG_VISOTP_CR_TIMEOUT), K_NO_WAIT);
}

static void isotp_rcv(const struct device *dev, struct can_frame *frame, void *arg)
{
	struct linux_isotp_sock *sk = (struct linux_isotp_sock *)arg;
	struct can_frame *cf = frame;
	int ae = sk->rx.addr.flags & ISOTP_PKG_EXT_ADDR ? 1: 0;
	if (!ae || cf->data[0] == sk->rx.addr.ext_addr)
	{
		uint8_t n_pci_type;
		n_pci_type = cf->data[ae] & 0xF0;
		switch (n_pci_type)
		{
		case N_PCI_FC:
			/* tx path: flow control frame containing the FC parameters */
			isotp_rcv_fc(sk, cf, ae);
			return;

		case N_PCI_SF:
			/* rx path: single frame
			*
			* As we do not have a rx.ll_dl configuration, we can only test
			* if the CAN frames payload length matches the LL_DL == 8
			* requirements - no matter if it's CAN 2.0 or CAN FD
			*/

			/* get the SF_DL from the N_PCI byte */
			isotp_rcv_sf(sk, cf, ae);
			return;

		case N_PCI_FF:
			/* rx path: first frame */
			isotp_rcv_ff(sk, cf, ae);
			return;

		case N_PCI_CF:
			/* rx path: consecutive frame */
			isotp_rcv_cf(sk, cf, ae);
			return;
		}
	}
	LOG_ERR("cf->data[0] != sk->rx.addr.ext_addr src: [%d] dst [%d]", cf->data[0], sk->rx.addr.ext_addr);
}

int64_t isotp_recv(struct linux_isotp_sock *sk, struct net_buf **buffer, k_timeout_t timeout)
{
	__ASSERT(sk, "sock is NULL");

	k_mutex_lock(&sk->mutex, K_FOREVER);
	bool close = sk->close;
	int64_t len = -1;
	k_mutex_unlock(&sk->mutex);
	if (!close)
	{
		len = 0;
		struct net_buf *buf = net_buf_get(&sk->fifo, timeout);
		if (buf)
		{
			*buffer = buf;
			len = net_buf_pull_le32(buf);
		}
	}
	return len;
}

static void isotp_sk_close_work_handler(struct k_work *work)
{
	struct linux_isotp_sock *sk = CONTAINER_OF(work, struct linux_isotp_sock, work);
	if (((struct isotp_frame *)sk->rx.focus)->send_num || ((struct isotp_frame *)sk->tx.focus)->send_num)
		k_work_submit(work);
	if (sk->rx.state != VISOTP_RX_STATE_WAIT_FF_SF || sk->tx.state != ISOTP_IDLE)
	{
		if (sk->filter_id != -1 || sk->tx.state != ISOTP_IDLE)
			k_work_submit(work);
	}
	can_remove_rx_filter(sk->can_dev, sk->filter_id);
	sk->filter_id = -1;

	k_fifo_cancel_wait(&sk->fifo);
	net_buf_unref(sk->rx.buf);
	net_buf_unref(sk->tx.buf);
	k_mem_slab_free(&isotp_frame_slab, sk->rx.focus);
	k_mem_slab_free(&isotp_frame_slab, sk->tx.focus);
	
	k_mutex_lock(&sk_list_mutex, K_FOREVER);
	for (int idx = 0; idx < CONFIG_CAN_MAX_FILTER; idx++)
	{
		if (sk_list[idx] == sk)
		{
			k_mem_slab_free(&isotp_sock_slab, sk);
			sk_list[idx] = NULL;
			break;
		}
	}
	k_mutex_unlock(&sk_list_mutex);
}

struct linux_isotp_sock *isotp_init(const struct device *can_dev, struct isotp_pkg_id rxaddr, struct isotp_pkg_id txaddr, struct isotp_fc_opts rxfc)
{
	if (rxaddr.flags & ISOTP_PKG_FDF || txaddr.flags & ISOTP_PKG_FDF)
	{
		can_mode_t cap;
		if (can_get_capabilities(can_dev, &cap) || !(cap & CAN_MODE_FD))
		{
			LOG_ERR("CAN controller does not support FD mode");
			return NULL;
		}
	}
	struct linux_isotp_sock tmp = {
		.can_dev = can_dev, .close = false, .tx_gap = {},
		.filter_id = 0, .fifo = {}, .callback = NULL, .args = NULL,
		.rx = { .addr = rxaddr, .fc = rxfc, .state = VISOTP_RX_STATE_WAIT_FF_SF },
		.tx = { .addr = txaddr, .state = ISOTP_IDLE }
	};
	struct isotp_frame *frame;
	if (k_mem_slab_alloc(&isotp_frame_slab, (void **)&frame, K_NO_WAIT))
	{
		LOG_ERR("isotp_frame_slab k_mem_slab_alloc: NULL");
		return NULL;
	}
	frame->send_num = 0;
	tmp.rx.focus = &frame->frame;
	if (k_mem_slab_alloc(&isotp_frame_slab, (void **)&frame, K_NO_WAIT))
	{
		LOG_ERR("isotp_frame_slab k_mem_slab_alloc: NULL");
		k_mem_slab_free(&isotp_frame_slab, tmp.rx.focus);
		return NULL;
	}
	frame->send_num = 0;
	tmp.tx.focus = &frame->frame;
	struct linux_isotp_sock *sk = NULL;

	int idx = 0;
	k_mutex_lock(&sk_list_mutex, K_FOREVER);
	for (; idx < CONFIG_CAN_MAX_FILTER; idx++)
	{
		if (!sk_list[idx])
		{
			if (k_mem_slab_alloc(&isotp_sock_slab, (void **)&sk, K_NO_WAIT))
				break;
			sk_list[idx] = sk;
			break;
		}
	}
	k_mutex_unlock(&sk_list_mutex);
	if (!sk)
	{
		LOG_ERR("not free sock list");
		k_mem_slab_free(&isotp_frame_slab, tmp.rx.focus);
		k_mem_slab_free(&isotp_frame_slab, tmp.tx.focus);
		return NULL;
	}
	*sk = tmp;
	((struct isotp_frame *)sk->rx.focus)->sk = sk;
	((struct isotp_frame *)sk->tx.focus)->sk = sk;
	struct can_filter filter;
	filter.id = sk->rx.addr.ext_id;
	filter.mask = CAN_EXT_ID_MASK;
	filter.flags = CAN_FILTER_IDE;
	sk->filter_id = can_add_rx_filter(sk->can_dev, isotp_rcv, sk, &filter);
	LOG_ERR("Error adding FC filter [%d] %d %d %d", sk->filter_id, filter.id, filter.mask, filter.flags);
	if (sk->filter_id < 0)
	{
		LOG_ERR("Error adding FC filter [%d]", sk->filter_id);
		sk_list[idx] = NULL;
		k_mem_slab_free(&isotp_frame_slab, tmp.rx.focus);
		k_mem_slab_free(&isotp_frame_slab, tmp.tx.focus);
		k_mem_slab_free(&isotp_sock_slab, sk);
		return NULL;
	}

	k_fifo_init(&sk->fifo);
	k_timer_init(&sk->tx.timer, send_timeout_handler, NULL);
	k_timer_init(&sk->rx.timer, receive_timeout_handler, NULL);
	k_mutex_init(&sk->mutex);
	k_work_init(&sk->work, isotp_sk_close_work_handler);
	return sk;
}

void isotp_close(struct linux_isotp_sock *sk)
{
	__ASSERT(sk, "sock is NULL");

	k_mutex_lock(&sk->mutex, K_FOREVER);
	sk->close = true;
	k_mutex_unlock(&sk->mutex);
	k_work_submit(&sk->work);
}

static void isotp_send_flow_frame_cb(const struct device *dev, int error, void *arg)
{
	struct isotp_frame *frame = (struct isotp_frame *)arg;
	if (!frame->send_num)
	{
		LOG_ERR("isotp_send_flow_frame_cb send_num is zero");
		return;
	}
	frame->send_num--;
	if (!frame->send_num)
	{
		if (error)
		{
			k_timer_stop(&frame->sk->tx.timer);
			frame->sk->tx.state = ISOTP_TX_STATE_SEND_ERR;
			k_timer_start(&frame->sk->tx.timer, K_NO_WAIT, K_NO_WAIT);
			LOG_ERR("send flow frame error: [%d]", error);
			return;
		}
		switch (frame->sk->tx.state)
		{
		case ISOTP_SEND_END:
			k_timer_stop(&frame->sk->tx.timer);
			inline_callback_timer(frame->sk->callback, ISOTP_N_OK, frame->sk->args);
			frame->sk->callback = NULL;
			frame->sk->tx.state = ISOTP_IDLE;
			net_buf_unref(frame->sk->tx.buf);
			break;
		case ISOTP_WAIT_FC_START:
			frame->sk->tx.state = ISOTP_WAIT_FC;
			k_timer_start(&frame->sk->tx.timer, K_MSEC(CONFIG_VISOTP_CR_TIMEOUT), K_NO_WAIT);
			break;
		case ISOTP_SENDING:
			if (frame->sk->tx.fc.stmin)
			{
				k_timer_start(&frame->sk->tx.timer, frame->sk->tx_gap, K_NO_WAIT);
				break;
			}
			if (isotp_send_cf(frame->sk))
			{
				k_timer_stop(&frame->sk->tx.timer);
				inline_callback_timer(frame->sk->callback, -ISOTP_TX_STATE_SEND_ERR, frame->sk->args);
				frame->sk->callback = NULL;
				frame->sk->tx.state = ISOTP_IDLE;
				net_buf_unref(frame->sk->tx.buf);
				return;
			}
		}
	}
}

static int isotp_send_sf(struct linux_isotp_sock *sk) {
	/** The message size generally fits into a SingleFrame - good.
	 *
	 * SF_DL ESC offset optimization:
	 *
	 * When TX_DL is greater 8 but the message would still fit
	 * into a 8 byte CAN frame, we can omit the offset.
	 * This prevents a protocol caused length extension from
	 * CAN_DL = 8 to CAN_DL = 12 due to the SF_SL ESC handling.
	 */
	struct isotp_frame *frame = (struct isotp_frame *)sk->tx.focus;
	frame->frame.dlc = sk->tx.addr.dlc;
	int8_t item = 0;
	if (sk->tx.addr.flags & ISOTP_PKG_EXT_ADDR)
		frame->frame.data[item++] = sk->tx.addr.ext_addr;

	/* place single frame N_PCI w/o length in appropriate index */
	frame->frame.data[item++] = N_PCI_SF | sk->tx.len;

	/* take care of a potential SF_DL ESC offset for TX_DL > 8 */
	if (sk->tx.len > (CAN_MAX_DLC - SF_PCI_SZ4 - item))
	{
		frame->frame.data[item - 1] = N_PCI_SF;
		frame->frame.data[item] = sk->tx.len;
	}
	for (int i = item; i < sk->tx.addr.dlc; i++)
		frame->frame.data[i] = net_buf_pull_u8(sk->tx.next);

	/* we are done */
	LOG_DBG("single are done");
	sk->tx.state = ISOTP_SEND_END;
	k_timer_start(&sk->tx.timer, K_MSEC(CONFIG_VISOTP_A_TIMEOUT), K_NO_WAIT);
	return isotp_send_frame(isotp_send_flow_frame_cb, frame);
}

static int isotp_send_ff(struct linux_isotp_sock *sk) {
	struct isotp_frame *frame = (struct isotp_frame *)sk->tx.focus;
	frame->frame.dlc = sk->tx.addr.dlc;
	int8_t item = 0;
	if (sk->tx.addr.flags & ISOTP_PKG_EXT_ADDR)
		frame->frame.data[item++] = sk->tx.addr.ext_addr;
	/* use 12 bit FF_DL notation */
	frame->frame.data[item++] = (uint8_t)(sk->tx.len >> 8) | N_PCI_FF;
	frame->frame.data[item++] = (uint8_t)sk->tx.len & 0xFFU;
	/* create N_PCI bytes with 12/32 bit FF_DL data length */
	if (sk->tx.len > MAX_FF_DL12)
	{
		/* use 32 bit FF_DL notation */
		frame->frame.data[item - 2] = N_PCI_FF;
		frame->frame.data[item - 1] = 0;
		frame->frame.data[item++] = (uint8_t)(sk->tx.len >> 24) & 0xFFU;
		frame->frame.data[item++] = (uint8_t)(sk->tx.len >> 16) & 0xFFU;
		frame->frame.data[item++] = (uint8_t)(sk->tx.len >> 8) & 0xFFU;
		frame->frame.data[item++] = (uint8_t)sk->tx.len & 0xFFU;
	}
	/* add first data bytes depending on item */
	for (int i = item; i < sk->tx.addr.dlc; i++)
	{
		sk->tx.idx++;
		if (net_buf_tailroom(sk->tx.next))
			sk->tx.next = sk->tx.next->frags;
		frame->frame.data[i] = net_buf_pull_u8(sk->tx.next);
	}
	sk->tx.sn = 1;
	k_timer_start(&sk->tx.timer, K_MSEC(CONFIG_VISOTP_A_TIMEOUT), K_NO_WAIT);
	return isotp_send_frame(isotp_send_flow_frame_cb, frame);
}

static inline int isotp_send_cf(struct linux_isotp_sock *sk)
{
	struct isotp_frame *frame = (struct isotp_frame *)sk->tx.focus;
	int8_t item = 0;
	if (sk->tx.addr.flags & ISOTP_PKG_EXT_ADDR)
		frame->frame.data[item++] = sk->tx.addr.ext_addr;

	int pcilen = N_PCI_SZ + item;
	int space = sk->tx.addr.dlc - pcilen;
	frame->frame.dlc = MIN(sk->tx.len - sk->tx.idx, space) + pcilen;
	/* place consecutive frame N_PCI in appropriate index */
	frame->frame.data[item++] = N_PCI_CF | sk->tx.sn++;
	sk->tx.sn %= 16;

	/* add first data bytes depending on item */
	for (int i = item; i < sk->tx.addr.dlc; i++)
	{
		if (net_buf_tailroom(sk->tx.next))
			sk->tx.next = sk->tx.next->frags;
		sk->tx.idx++;
		frame->frame.data[i] = net_buf_pull_u8(sk->tx.next);
	}
	if (sk->tx.fc.bs && sk->tx.bs++ == sk->tx.fc.bs)
	{
		/* stop and wait for FC */
		LOG_DBG("BS stop and wait for FC");
		sk->tx.state = ISOTP_WAIT_FC_START;
	}

	if (sk->tx.idx >= sk->tx.len)
	{
		/* we are done */
		LOG_DBG("we are done");
		sk->tx.state = ISOTP_SEND_END;
	}
	return isotp_send_frame(isotp_send_flow_frame_cb, frame);
}

int isotp_send(struct linux_isotp_sock *sk, uint8_t *data, size_t len, isotp_callback complete_cb, void *cb_arg)
{
	__ASSERT(sk, "sock is NULL");

	k_mutex_lock(&sk->mutex, K_FOREVER);
	bool close = sk->close;
	k_mutex_unlock(&sk->mutex);
	if (close)
		return 1;
	typedef int (*send_frame)(struct linux_isotp_sock *sk);
	if (sk->tx.state != ISOTP_IDLE)
	{
		LOG_ERR("sk->tx.state != ISOTP_IDLE state: [%d] len [%d] idx [%d]", sk->tx.state, sk->tx.len, sk->tx.idx);
		return 1;
	}
	sk->tx.state = ISOTP_WAIT_FIRST_FC;
	sk->tx.buf = net_buf_alloc_fixed(&isotp_send_pool, K_NO_WAIT);
	if (!sk->tx.buf)
	{
		LOG_ERR("pool_alloc_buffer net buf == NULL");
		sk->tx.state = ISOTP_IDLE;
		return 1;
	}
	send_frame func = isotp_send_sf;
	// max sf_dl
	int sf_dl = sk->tx.addr.dlc - SF_PCI_SZ8 - 1;
	// not ext addr and one byte
	if (!(sk->tx.addr.flags & ISOTP_PKG_EXT_ADDR))
		sf_dl++;
	// not can fd and one byte
	if (!(sk->tx.addr.flags & ISOTP_PKG_FDF))
		sf_dl++;
	if (len > sf_dl)
		func = isotp_send_ff;
	sk->tx.idx = 0;
	sk->tx.len = len;
	sk->tx.next = sk->tx.buf;
	if (!from_pool_add_net_buf_data(&isotp_send_pool, sk->tx.buf, (uint8_t *)data, sk->tx.len))
	{
		net_buf_unref(sk->tx.next);
		LOG_ERR("pool_alloc_buffer net buf == NULL");
		sk->tx.state = ISOTP_IDLE;
		return 1;
	}
	
	int ret = func(sk);
	if (ret)
	{
		k_timer_stop(&sk->tx.timer);
		sk->tx.state = ISOTP_TX_STATE_SEND_ERR;
		k_timer_start(&sk->tx.timer, K_NO_WAIT, K_NO_WAIT);
		LOG_ERR("send first or single frame error: [%d]", ret);
	}
	else if (complete_cb)
	{
		sk->callback = complete_cb;
		sk->args = cb_arg;
	}
	return ret;
}
