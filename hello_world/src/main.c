/*
 * @Author       : Eacher
 * @Date         : 2024-04-18 15:25:02
 * @LastEditTime : 2024-07-19 10:56:43
 * @LastEditors  : Eacher
 * --------------------------------------------------------------------------------<
 * @Description  : 
 * --------------------------------------------------------------------------------<
 * @FilePath     : /zephyrproject/veryark/application/fs_sample/src/main.c
 */
#include <sys/_stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <app/isotp/isotp_conn.h>

LOG_MODULE_REGISTER(main, 3);

#define BLINK_PERIOD_MS_STEP 100U
#define BLINK_PERIOD_MS_MAX  1000U

#define MAIN main1

void isotp_conn_open(struct linux_isotp_sock *conn);

uint8_t data[4095] = {0};

void ___read(struct linux_isotp_sock *conn)
{
	struct net_buf *buffer = NULL;
	uint16_t item = 0;
	while (1) {
		LOG_INF("Could not get sample STATE: %d %p", conn->rx.state, conn);
		// k_sleep(K_MSEC(10000));
		int64_t n = isotp_recv(conn, &buffer, K_SECONDS(10));
		while (buffer) {
			if (item + buffer->len <= 4095) {
				memcpy(&data[item], buffer->data, buffer->len);
				item += buffer->len;
			}
			buffer = net_buf_frag_del(NULL, buffer);
		}
		data[item] = 0;
		if (item > 21) {
			LOG_INF("n: %lld len: %d data: %s", n, item, &data[item-21]);
		}
		item = 0;
	}
}

void complete_cb(int error_nr, void *arg)
{

}

void ___write(struct linux_isotp_sock *conn)
{
	int ret;
	while (1) {
		// memset(data, 0, 4095);
		ret = isotp_send(conn, data, 3072, complete_cb, NULL);
		k_sleep(K_MSEC(1000));
	}
}

uint32_t rxid = 2 << 11 | 101;
uint32_t txid = 101 << 11 | 2;
// uint32_t txid = 2 << 11 | 101;
// uint32_t rxid = 101 << 11 | 2;

// uint32_t rxid = 102 << 11 | 101;
// uint32_t txid = 101 << 11 | 102;
// uint32_t txid = 102 << 11 | 101;
// uint32_t rxid = 101 << 11 | 102;

int main1(void)
{
	int ret = __init_can();
	if (ret) {
		LOG_ERR("__init_can error (%d)", ret);
	}
	struct linux_isotp_sock conn;
	struct linux_isotp_sock *sk;
	// conn.rx.fc.bs = 0xF;
	conn.rx.fc.bs = 0x00;
	conn.rx.fc.stmin = 0;
	conn.rx.addr.ext_id = rxid;
	conn.rx.addr.flags = 0;
	if (conn.rx.addr.ext_id > 2047) {
		conn.rx.addr.flags |= ISOTP_PKG_IDE;	
	}
	conn.tx.addr.ext_id = txid;
	conn.tx.addr.flags = 0;
	conn.tx.addr.dlc = 8;
	if (conn.tx.addr.ext_id > 2047) {
		conn.tx.addr.flags |= ISOTP_PKG_IDE;	
	}
	sk = isotp_init(get_can_dev(), conn.rx.addr, conn.tx.addr, conn.rx.fc);
	if (sk) {
		___read(sk);
		// ___write(sk);
	}
	LOG_ERR("isotp_init error (%d)", ret);
	return 0;
}

int main2(void)
{
	int ret = __init_can();
	if (ret) {
		LOG_ERR("__init_can error (%d)", ret);
	}
    k_msleep(1000);
    // ark_d_register_api_method("file_transport", sys_shutdown, NULL, NULL, NULL);
    uint8_t data[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    while(1){
        ret = can_frame_send(data, 8);
		LOG_INF("ret: %d ", ret);
        k_msleep(10000);
    }
	return 0;
}

int main(void)
{
	return MAIN();
}
