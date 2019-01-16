/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <stdio.h>
#include <uart.h>
#include <string.h>
#include <at_host.h>

#include <net/mqtt.h>
#include <net/socket.h>
#include <lte_lc.h>

#include <pwm.h>

#define PWM_DRIVER CONFIG_PWM_0_NAME
#define PWM_CHANNEL 10
#define PERIOD (USEC_PER_SEC / 50)

#define MINPULSEWIDTH 700
#define MAXPULSEWIDTH 2300

/* Buffers for MQTT client. */
static u8_t rx_buffer[CONFIG_MQTT_MESSAGE_BUFFER_SIZE];
static u8_t tx_buffer[CONFIG_MQTT_MESSAGE_BUFFER_SIZE];
static u8_t payload_buf[CONFIG_MQTT_PAYLOAD_BUFFER_SIZE];

/* The mqtt client struct */
static struct mqtt_client client;

/* MQTT Broker details. */
static struct sockaddr_storage broker;

static bool connected;

/* PWM device */
struct device *pwm_dev;

/* Function for setting servo postition (0-100) */
static int servo_set(int pos)
{
	printk("servo_set: %d\n", pos);
	return pwm_pin_set_usec(pwm_dev, PWM_CHANNEL, PERIOD,
	(MINPULSEWIDTH + (pos * (MAXPULSEWIDTH - MINPULSEWIDTH)/100)) );
}

static int servo_init(void)
{
  pwm_dev = device_get_binding(PWM_DRIVER);
  if (!pwm_dev) {
          printk("Cannot find PWM device!\n");
          return -ENOTSUP;
  }
  return 0;
}

/**@brief Function to print strings without null-termination
 */
static void data_print(u8_t *prefix, u8_t *data, size_t len)
{
	char buf[len + 1];

	memcpy(buf, data, len);
	buf[len] = 0;
	printk("%s%s\n", prefix, buf);
}

/**@brief Function to publish data on the configured topic
 */
static int data_publish(struct mqtt_client *c, enum mqtt_qos qos,
	u8_t *data, size_t len)
{
	struct mqtt_publish_param param;

	param.message.topic.qos = qos;
	param.message.topic.topic.utf8 = CONFIG_MQTT_PUB_TOPIC;
	param.message.topic.topic.size = strlen(CONFIG_MQTT_PUB_TOPIC);
	param.message.payload.data = data;
	param.message.payload.len = len;
	param.message_id = sys_rand32_get();
	param.dup_flag = 0;
	param.retain_flag = 0;

	data_print("Publish: ", data, len);
	printk("to topic: %s len: %d\n",
		CONFIG_MQTT_PUB_TOPIC,
		strlen(CONFIG_MQTT_PUB_TOPIC));

	return mqtt_publish(c, &param);
}

/**@brief Function to subscribe to the configured topic
 */
static int subscribe(void)
{
	struct mqtt_topic subscribe_topic = {
		.topic = {
			.utf8 = CONFIG_MQTT_SUB_TOPIC,
			.size = strlen(CONFIG_MQTT_SUB_TOPIC)
		},
		.qos = MQTT_QOS_1_AT_LEAST_ONCE
	};

	const struct mqtt_subscription_list subscription_list = {
		.list = &subscribe_topic,
		.list_count = 1,
		.message_id = 1234
	};

	printk("Subscribing to: %s len %d\n", CONFIG_MQTT_SUB_TOPIC,
		strlen(CONFIG_MQTT_SUB_TOPIC));

	return mqtt_subscribe(&client, &subscription_list);
}

/**@brief Function to read the published payload.
 */
static int publish_get_payload(struct mqtt_client *c, size_t length)
{
	u8_t *buf = payload_buf;
	u8_t *end = buf + length;
	struct pollfd fds;

	if (length > sizeof(payload_buf)) {
		return -EMSGSIZE;
	}

	if (c->transport.type == MQTT_TRANSPORT_NON_SECURE) {
		fds.fd = c->transport.tcp.sock;
	} else {
#if defined(CONFIG_MQTT_LIB_TLS)
		fds.fd = c->transport.tls.sock;
#else
		return -ENOTSUP;
#endif
	}

	fds.events = POLLIN;

	while (buf < end) {
		int ret = mqtt_read_publish_payload(c, buf, end - buf);

		if (ret < 0) {
			if (ret == -EAGAIN) {
				printk("mqtt_read_publish_payload: EAGAIN");
				poll(&fds, 1, K_FOREVER);
				continue;
			}

			return ret;
		}

		if (ret == 0) {
			return -EIO;
		}

		buf += ret;
	}

	return 0;
}

/**@brief MQTT client event handler
 */
void mqtt_evt_handler(struct mqtt_client *const c,
		      const struct mqtt_evt *evt)
{
	int err;

	switch (evt->type) {
	case MQTT_EVT_CONNACK:
		if (evt->result != 0) {
			printk("MQTT connect failed %d\n", evt->result);
			break;
		}

		connected = true;
		printk("[%s:%d] MQTT client connected!\n", __func__, __LINE__);
		subscribe();
		break;

	case MQTT_EVT_DISCONNECT:
		printk("[%s:%d] MQTT client disconnected %d\n", __func__,
		       __LINE__, evt->result);

		connected = false;
		break;

	case MQTT_EVT_PUBLISH: {
		const struct mqtt_publish_param *p = &evt->param.publish;
		size_t len = p->message.payload.len;

		printk("[%s:%d] MQTT PUBLISH result=%d len=%d\n", __func__,
		       __LINE__, evt->result, len);
		err = publish_get_payload(c, len);
		if (err >= 0) {
			data_print("Received: ", payload_buf, len);
			if(len > 4) {
				if(!strncmp("PWM:", payload_buf, 4)) {
					char str[len - 3];
					int value;

					/* Create null terminated string */
					memcpy(str, &payload_buf[4], len - 4);
					str[len - 2] = 0;

					value = atoi(str);
					printk("str=%s value=%d\n", str, value);
					if ((value >= 0) && (value <= 100)) {
						servo_set(value);
					} 
				}
			}
			/* Echo back received data */
			data_publish(&client, MQTT_QOS_1_AT_LEAST_ONCE,
				payload_buf, len);
		} else {
			printk("mqtt_read_publish_payload: Failed! %d\n", err);
		}
	} break;

	case MQTT_EVT_PUBACK:
		if (evt->result != 0) {
			printk("MQTT PUBACK error %d\n", evt->result);
			break;
		}

		printk("[%s:%d] PUBACK packet id: %u\n", __func__, __LINE__,
				evt->param.puback.message_id);
		break;

	case MQTT_EVT_SUBACK:
		if (evt->result != 0) {
			printk("MQTT SUBACK error %d\n", evt->result);
			break;
		}

		printk("[%s:%d] SUBACK packet id: %u\n", __func__, __LINE__,
				evt->param.suback.message_id);
		break;

	default:
		printk("[%s:%d] default: %d\n", __func__, __LINE__,
				evt->type);
		break;
	}
}

/**@brief Resolves the configured hostname and 
 * initializes the MQTT broker structure
 */
static void broker_init(void)
{
	int err;
	struct addrinfo *result;
	struct addrinfo *addr;
	struct addrinfo hints;

	hints.ai_flags = 0;
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = 0;

	err = getaddrinfo(CONFIG_MQTT_BROKER_HOSTNAME, NULL, &hints, &result);
	if (err) {
		printk("ERROR: getaddrinfo failed %d\n", err);

		return;
	}

	addr = result;
	err = -ENOENT;

	/* Look for address of the broker. */
	while (addr != NULL) {
		/* IPv4 Address. */
		if (addr->ai_addrlen == sizeof(struct sockaddr_in)) {
			struct sockaddr_in *broker4 =
				((struct sockaddr_in *)&broker);

			broker4->sin_addr.s_addr =
				((struct sockaddr_in *)addr->ai_addr)
				->sin_addr.s_addr;
			broker4->sin_family = AF_INET;
			broker4->sin_port = htons(CONFIG_MQTT_BROKER_PORT);
			printk("IPv4 Address found 0x%08x\n",
				broker4->sin_addr.s_addr);
			break;
		} else {
			printk("ai_addrlen = %d should be %d or %d\n",
				addr->ai_addrlen,
				sizeof(struct sockaddr_in),
				sizeof(struct sockaddr_in6));
		}

		addr = addr->ai_next;
		break;
	}

	/* Free the address. */
	freeaddrinfo(result);
}

/**@brief Initialize the MQTT client structure
 */
static void client_init(struct mqtt_client *client)
{
	mqtt_client_init(client);

	broker_init();

	/* MQTT client configuration */
	client->broker = &broker;
	client->evt_cb = mqtt_evt_handler;
	client->client_id.utf8 = (u8_t *)CONFIG_MQTT_CLIENT_ID;
	client->client_id.size = strlen(CONFIG_MQTT_CLIENT_ID);
	client->password = NULL;
	client->user_name = NULL;
	client->protocol_version = MQTT_VERSION_3_1_1;

	/* MQTT buffers configuration */
	client->rx_buf = rx_buffer;
	client->rx_buf_size = sizeof(rx_buffer);
	client->tx_buf = tx_buffer;
	client->tx_buf_size = sizeof(tx_buffer);

	/* MQTT transport configuration */
	client->transport.type = MQTT_TRANSPORT_NON_SECURE;
}

/**@brief Configures modem to provide LTE link. Blocks until link is
 * successfully established.
 */
static void modem_configure(void)
{
	if (IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT)) {
		/* Do nothing, modem is already turned on
		 * and connected.
		 */
	} else {
		int err;

		printk("LTE Link Connecting ...\n");
		err = lte_lc_init_and_connect();
		__ASSERT(err == 0, "LTE link could not be established.");
		printk("LTE Link Connected!\n");
	}
}

void main(void)
{
	int err;

	printk("The MQTT servo sample started\n");

	err = servo_init();
	if (err) {
		return;
	}

	servo_set(50);

	modem_configure();

	if (IS_ENABLED(CONFIG_AT_HOST_LIBRARY)) {
		err = at_host_init(CONFIG_AT_HOST_UART,
			CONFIG_AT_HOST_TERMINATION);
		if (err != 0) {
			printk("ERROR: AT Host not initialized\n");
			return;
		}
	}
	if (IS_ENABLED(CONFIG_MQTT_LIB)) {
		client_init(&client);

		err = mqtt_connect(&client);
		if (err != 0) {
			printk("ERROR: mqtt_connect. %d\n", err);
			return;
		}
	}

	while (1) {
		if (IS_ENABLED(CONFIG_AT_HOST_LIBRARY)) {
			at_host_process();
		}
		if (IS_ENABLED(CONFIG_MQTT_LIB)) {
			(void)mqtt_input(&client);
			(void)mqtt_live(&client);
		}
		if (IS_ENABLED(CONFIG_LOG)) {
			/* if logging is enabled, sleep */
			k_sleep(K_MSEC(10));
		} else {
			/* other, put CPU to idle to save power */
			k_cpu_idle();
		}
	}
}
