/****************************************************************************
 *
 * Copyright 2025 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
/****************************************************************************
 * drivers/sensors/ais25ba.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/sensors/ais25ba.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static ssize_t ais25ba_read(FAR struct sensor_upperhalf_s *dev, FAR void *buffer);
static void ais25ba_set_mclk(struct sensor_upperhalf_s *priv, int mclk);
static void ais25ba_set_bclk(struct sensor_upperhalf_s *priv, int bclk);
static void ais25ba_start(struct sensor_upperhalf_s *priv);
static void ais25ba_stop(struct sensor_upperhalf_s *priv);
static void ais25ba_setchannel_count(struct sensor_upperhalf_s *priv, int channel_count);
static void ais25ba_setbit_perchannel(struct sensor_upperhalf_s *priv, int bit_per_channel);
static void ais25ba_set_samprate(struct sensor_upperhalf_s *priv, int samp_rate);
static int ais25ba_verify_sensor(struct sensor_upperhalf_s *upper, struct i2c_dev_s *i2c, struct i2c_config_s config);
static void ais25ba_alivecheck_work(struct ais25ba_dev_s *dev);
static void ais25ba_set_config_i2c(struct i2c_dev_s *i2c, struct i2c_config_s config);
static void ais25ba_timer_handler(int argc, uint32_t arg1);
static int ais25ba_register_mq(struct sensor_upperhalf_s *priv, mqd_t g_mems_mq);
static void ais25ba_prepare(struct sensor_upperhalf_s *priv);
static void ais25ba_get_bufsize(struct sensor_upperhalf_s *upper, int* buf_size);
static void ais25ba_get_bufnum(struct sensor_upperhalf_s *upper, int* buf_num);
static void ais25ba_send_buffer(struct sensor_upperhalf_s *upper, unsigned long buffer);
static int ais25ba_read_data(struct ais25ba_dev_s *priv);
static int ais25ba_send_result(FAR struct ais25ba_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/
struct sensor_ops_s g_ais25ba_ops = {
    .sensor_read = ais25ba_read,
	.sensor_set_mclk = ais25ba_set_mclk,
	.sensor_set_bclk = ais25ba_set_bclk,
	.sensor_start = ais25ba_start,
	.sensor_stop = ais25ba_stop,
	.sensor_setchannel_count = ais25ba_setchannel_count,
	.sensor_setbit_perchannel = ais25ba_setbit_perchannel,
	.sensor_set_samprate = ais25ba_set_samprate,
	.sensor_verify = ais25ba_verify_sensor,
	.sensor_prepare = ais25ba_prepare,
	.sensor_register_mq = ais25ba_register_mq,
	.sensor_get_bufsize = ais25ba_get_bufsize,
	.sensor_get_bufnum = ais25ba_get_bufnum,
	.sensor_send_buffer = ais25ba_send_buffer,
};

//struct ap_buffer_s *g_apb;

static struct ais25ba_dev_s g_ais25ba_priv;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static float ais25ba_raw_to_mg(int16_t lsb)
{
	return ((float)lsb) * 0.122f;
}

static void ais25ba_i2s_callback(FAR struct i2s_dev_s *dev, FAR struct ap_buffer_s *apb, FAR void *arg, int result)
{
	struct ais25ba_ctrl_s *ctrl = (struct ais25ba_ctrl_s *)arg;
	sem_post(&ctrl->callback_wait_sem);
	sem_post(&ctrl->read_sem);
}

static void ais25ba_enqueue_data(FAR struct ais25ba_dev_s *priv, struct ais25ba_buf_s *buf)
{
	lldbg("buf: %x\n", buf);
	sq_addlast((sq_entry_t *)&buf->entry, &priv->pendq);
}

static void ais25ba_set_mclk(struct sensor_upperhalf_s *priv, int mclk)
{
	return;
}

static void ais25ba_set_bclk(struct sensor_upperhalf_s *priv, int bclk)
{
	return;
}

static void ais25ba_start(struct sensor_upperhalf_s *upper)
{
	dbg("sensor start\n");
	struct ais25ba_dev_s *priv = upper->priv;
	struct i2s_dev_s *i2s = priv->i2s;

	
	if (priv->mq == NULL) {
		dbg("ERROR: sensor mq is NULL\n");
		return -ENOTTY;
	}
	dbg("\n\n\n\n\n\n**************priv_addr: %p, line: %d\n", priv, __LINE__);
	priv->sensor_run_on = true;
	dbg("sensor_run_on: val: %d, addr: %d, line: %d\n", priv->sensor_run_on, &(priv->sensor_run_on), __LINE__);
	dbg("%d\n", __LINE__);
	//I2S_RESUME(i2s, I2S_RX);		--> Creating kernel crash. Board reboot

	/*
	// For testing queue
	ais25ba_read_data(priv);
	ais25ba_send_result(priv);
	*/

	dbg("%d\n", __LINE__);
}

static void ais25ba_stop(struct sensor_upperhalf_s *upper)
{
	lldbg("sensor stop\n");
	lldbg("%d\n", __LINE__);
	struct ais25ba_dev_s *priv = upper->priv;
	I2S_STOP(priv->i2s, I2S_RX);
	priv->sensor_run_on = false;
}

static void ais25ba_setchannel_count(struct sensor_upperhalf_s *priv, int channel_count)
{
	return;
}

static void ais25ba_setbit_perchannel(struct sensor_upperhalf_s *upper, int bit_per_channel)
{
	struct ais25ba_dev_s *priv = upper->priv;
	lldbg("bit per channel %d\n", bit_per_channel);
	I2S_RXDATAWIDTH(priv->i2s, bit_per_channel);
}

static void ais25ba_set_samprate(struct sensor_upperhalf_s *upper, int samp_rate)
{
	FAR struct ais25ba_dev_s *priv = upper->priv;
	lldbg("i2s rx sample rate %d\n", samp_rate);
	I2S_RXSAMPLERATE(priv->i2s, samp_rate);
}

static void ais25ba_get_bufsize(struct sensor_upperhalf_s *upper, int* buf_size)
{
	*buf_size = AIS25BA_BUFSIZE;}

static void ais25ba_get_bufnum(struct sensor_upperhalf_s *upper, int* buf_num)
{
	*buf_num = AIS25BA_BUFNUM;
}

static void ais25ba_send_buffer(struct sensor_upperhalf_s *upper, unsigned long buffer)
{
	FAR struct ais25ba_dev_s *priv = upper->priv;
	struct ais25ba_buf_s *buf = (FAR struct ais25ba_buf_s *)buffer;
	ais25ba_enqueue_data(priv, buf);
}

static int ais25ba_register_mq(struct sensor_upperhalf_s *upper, mqd_t g_mems_mq)
{
	lldbg("%d\n", __LINE__);
	FAR struct ais25ba_dev_s *priv = upper->priv;
	lldbg("%d\n", __LINE__);
	priv->mq = (mqd_t)g_mems_mq;
	lldbg("%d\n", __LINE__);
	return OK;
}

static void ais25ba_prepare(struct sensor_upperhalf_s *priv)
{
	return;
}

static int ais25ba_verify_sensor(struct sensor_upperhalf_s *upper, struct i2c_dev_s *i2c, struct i2c_config_s config)
{
	int reg[2];
	uint8_t data[2];
	reg[0] = AIS25BA_WHOAMI_REGISTER;                                  //WHO_AM_I

#ifdef CONFIG_I2C_WRITEREAD
	if (i2c_writeread(i2c, &config, (uint8_t *)reg, 1, data, 1) != 1){
		return ERROR;
	}
#else
	if (i2c_write(i2c, &config, (uint8_t *)reg, 1) == 1) {
		i2c_read(i2c, &config, (uint8_t *)data, 1);
	} else {
		return ERROR;
	}
#endif

	if (data[0] == AIS25BA_WHOAMI_VALUE) {
		lldbg("MEMS: Alive check Sensor verify success, whoamIvalue matched!\n");
		return OK;
	}
	return ERROR;
}

static void ais25ba_i2c_read_data(struct i2c_dev_s *i2c, struct i2c_config_s config)
{
	int ret = 0;
	uint8_t reg[2];
	uint8_t data[2];
#ifdef CONFIG_I2C_WRITEREAD

	reg[0] = AIS25BA_TEST_REG;
	ret = i2c_writeread(i2c, &config, (uint8_t *)reg, 1, data, 1);

	reg[0] = AIS25BA_CTRL_REG_1;
	ret = i2c_writeread(i2c, &config, (uint8_t *)reg, 1, data, 1);
	printf("ret 26 data : %8x\n",  data[0]);

	reg[0] = AIS25BA_CTRL_REG_2;
	ret = i2c_writeread(i2c, &config, (uint8_t *)reg, 1, data, 1);
	printf("ret 2F data : %8x\n",  data[0]);

	reg[0] = AIS25BA_CTRL_REG_FS;
	ret = i2c_writeread(i2c, &config, (uint8_t *)reg, 1, data, 1);
	printf("ret 30 data : %8x\n", data[0]);

	reg[0] = AIS25BA_TDM_CTRL_REG;
	ret = i2c_writeread(i2c, &config, (uint8_t *)reg, 1, data, 1);
	printf("ret 2E data : %8x\n", data[0]);

#else
	reg[0] = AIS25BA_TEST_REG;
	ret = i2c_write(i2c, &config, (uint8_t *)reg, 1);
	if (ret == 1) {
		i2c_read(i2c, &config, (uint8_t *)data, 1);
		printf("data read 0B : %8x\n", data[0]); // this should be 0x20
	}

	reg[0] = AIS25BA_CTRL_REG_1;
	ret = i2c_write(i2c, &config, (uint8_t *)reg, 1);
	if (ret == 1) {
		i2c_read(i2c, &config, (uint8_t *)data, 1);
		printf("data read 26 : %8x\n", data[0]); // this should be 0x20
	}

	reg[0] = AIS25BA_CTRL_REG_2;
	ret = i2c_write(i2c, &config, (uint8_t *)reg, 1);
	if (ret == 1) {
		i2c_read(i2c, &config, (uint8_t *)data, 1);
		printf("data read 2F : %8x\n", data[0]); // this should be 0x20
	}

	reg[0] = AIS25BA_CTRL_REG_FS;
	ret = i2c_write(i2c, &config, (uint8_t *)reg, 1);
	if (ret == 1) {
		i2c_read(i2c, &config, (uint8_t *)data, 1);
		printf("data read 30 : %8x\n", data[0]); // this should be 0x20
	}

	reg[0] = AIS25BA_TDM_CTRL_REG;
	ret = i2c_write(i2c, &config, (uint8_t *)reg, 1);
	if (ret == 1) {
		i2c_read(i2c, &config, (uint8_t *)data, 1);
		printf("data read 2E : %8x\n", data[0]); // this should be 0x20
	}
#endif
}

static void ais25ba_i2c_write_data(struct i2c_dev_s *i2c, struct i2c_config_s config)
{
	int ret = 0;
	uint8_t reg[2];
	uint8_t data[2];
#ifdef CONFIG_I2C_WRITEREAD
	reg[0] = 0x26;
	reg[1] = 0x00;
	ret = i2c_writeread(i2c, &config, (uint8_t *)reg, 2, data, 0);
	//printf("\nret 26 data : %8x\n",  data[0]);
	DelayMs(100);
	reg[0] = 0x2E;
	reg[1] = 0x62;
	ret = i2c_writeread(i2c, &config, (uint8_t *)reg, 2, data, 0);
	//printf("\nret 2E data : %8x\n",  data[0]);
#else
	reg[0] = 0x26;
	reg[1] = 0x00;
	ret = i2c_write(i2c, &config, (uint8_t *)reg, 2);

	reg[0] = 0x2E;
	reg[1] = 0x62;
	ret = i2c_write(i2c, &config, (uint8_t *)reg, 2);
#endif
}

static void ais25ba_set_config_i2c(struct i2c_dev_s *i2c, struct i2c_config_s config)
{
	ais25ba_i2c_read_data(i2c, config);
	DelayMs(2000);
	ais25ba_i2c_write_data(i2c, config);
	DelayMs(2000);
	ais25ba_i2c_read_data(i2c, config);
	DelayMs(2000);
}

static int ais25ba_read_i2s(struct i2s_dev_s *i2s, struct ais25ba_ctrl_s *ctrl, FAR void *buffer)
{
	dbg("%d\n", __LINE__);
	struct audio_buf_desc_s desc;
	int sem_cnt;
	int prev_sem_cnt;
	struct ap_buffer_s *g_apb;
	desc.numbytes = 512;
	desc.u.ppBuffer = &g_apb;

	dbg("%d\n", __LINE__);
	int ret = apb_alloc(&desc);
	if (ret < 0) {
			printf("ERROR: apb_alloc: apb buffer allocation failed\n");
			return;
	}

	dbg("%d\n", __LINE__);
	//sem_timedwait(&ctrl->read_sem, &ctrl->sem_timeout);				/* To prevent deadlock in I2S_RECEIVE */
	sem_wait(&ctrl->read_sem);

	dbg("%d\n", __LINE__);
    ret = I2S_RECEIVE(i2s, g_apb, ais25ba_i2s_callback, ctrl, 100);	/* 100 ms timeout for read data */
	if (ret != OK) {
		lldbg("ERROR: I2S_RECEIVE FAILED\n");
	}

	dbg("%d\n", __LINE__);
	//sem_timedwait(&ctrl->callback_wait_sem, &ctrl->sem_timeout);	/* To prevent deadlock in I2S_RECEIVE */
	sem_wait(&ctrl->callback_wait_sem);

	dbg("%d\n", __LINE__);
	sensor_data_s *data = (sensor_data_s *)buffer;
	int16_t *samp_data = (int16_t *)&g_apb->samp[0];

	for (int i = 0, j = 0; i < g_apb->nbytes; i+=16, j++) {
		data[j].x = ais25ba_raw_to_mg(*samp_data);
		samp_data++;
		data[j].y = ais25ba_raw_to_mg(*samp_data);
		samp_data++;
		data[j].z = ais25ba_raw_to_mg(*samp_data);
		samp_data += 6;		/* Vendor specific skip bits */
	}
	dbg("%d\n", __LINE__);
	apb_free(g_apb);
	dbg("%d\n", __LINE__);
	return ret;
}

static ssize_t ais25ba_read(FAR struct sensor_upperhalf_s *dev, FAR void *buffer)
{
	dbg("%d\n", __LINE__);
	FAR struct ais25ba_dev_s *priv = dev->priv;
	int ret;

	dbg("%d\n", __LINE__);
	struct i2c_dev_s *i2c = priv->i2c;
	struct i2s_dev_s *i2s = priv->i2s;
	struct i2c_config_s config = priv->i2c_config;

	dbg("%d\n", __LINE__);
	ret = ais25ba_read_i2s(i2s, &priv->ctrl, buffer);
	DelayMs(5000);

	dbg("%d\n", __LINE__);
	return ret;
}
/*
void print_sensor_data(sensor_data_s *data)
{
	dbg("%d\n", __LINE__);
	for (int i = 0; i < AIS25BA_BUFLENGTH; i++) {
                lldbg("x: %f, y: %f, z: %f\n", data[i].x, data[i].y, data[i].z);
	}
}
*/
static int ais25ba_read_data(struct ais25ba_dev_s *priv)
{
	dbg("%d\n", __LINE__);
	int ret = OK;
	struct ais25ba_buf_s *buf = (struct ais25ba_buf_s *)sq_remfirst(&priv->pendq);

	dbg("%d\n", __LINE__);
	ret = ais25ba_read(priv->upper, buf);
	// add ret check

	//dbg("Printing the result data from driver\n");	//--> for testing
	print_sensor_data(buf->data);						//--> for testing
	dbg("%d\n", __LINE__);
	sq_addlast((sq_entry_t *)&buf->entry, &priv->pendq);
	dbg("%d\n", __LINE__);
}

static void ais25ba_alivecheck_work(struct ais25ba_dev_s *dev)
{
	int sensor_status;
	int retry_count = AIS25BA_ALIVECHECK_RETRY_COUNT;
	sensor_status = ais25ba_verify_sensor(dev->upper, dev->i2c, dev->i2c_config);

	if (sensor_status != OK) {
		// Retry Sensor Verification
		lldbg("Sensor verification failed, applying retry and recover\n");
		goto retry_sensor_verification;
	}
	(void)wd_start(dev->wdog, MSEC2TICK(AIS25BA_ALIVECHECK_TIME), (wdentry_t)ais25ba_timer_handler, 1, (uint32_t)dev);
	return;

retry_sensor_verification:
	sensor_status = OK;
	while (retry_count > 0 && sensor_status == OK) {
		sensor_status = ais25ba_verify_sensor(dev->upper, dev->i2c, dev->i2c_config);
		retry_count--;
		DelayMs(1000);
	}

	if (sensor_status != OK) {
		// Initialize sensor again;
		struct ais25ba_ctrl_s *ctrl = &(dev->ctrl);
		sem_wait(&ctrl->read_sem);
		ais25ba_set_config_i2c(dev->i2c, dev->i2c_config);
		lldbg("Sensor reinitialized");
		(void)wd_start(dev->wdog, MSEC2TICK(AIS25BA_ALIVECHECK_TIME), (wdentry_t)ais25ba_timer_handler, 1, (uint32_t)dev);
		sem_post(&ctrl->read_sem);
	}
}

static void ais25ba_timer_handler(int argc, uint32_t arg1)
{
	struct ais25ba_dev_s *priv = (struct ais25ba_dev_s *)arg1;
	work_queue(HPWORK, &priv->work, (worker_t)ais25ba_alivecheck_work, priv, 0);
}

static int ais25ba_send_result(FAR struct ais25ba_dev_s *priv)		/* Pop buffer from doneq and push to mq */
{
	struct ais25ba_msg_s msg;
	struct ais25ba_buf_s *buf;
	int ret = OK;
	lldbg("\n%d\n", __LINE__);
	while (sq_peek(&priv->doneq) != NULL) {
		buf = (struct ais25ba_buf_s *)sq_remfirst(&priv->doneq);
		msg.msgId = buf->msgId;
		msg.pData = (FAR void *)buf;
		lldbg("\n%d\n", __LINE__);
		print_sensor_data(buf->data);
		lldbg("\n%d\n", __LINE__);
		ret = mq_send(priv->mq, (FAR const char *)&msg, sizeof(msg), CONFIG_AIS25BA_SG_DEQUEUE_PRIO);
		if (ret != OK) {
			lldbg("mq_send error, errno : %d\n", errno);
		}
	}
	return ret;
}

static int ais25ba_mq_thread(int argc, char **argv)
{
	struct ais25ba_dev_s *priv;
	int ret = OK;

	//DEBUGASSERT(argc == 2);
	priv = (struct ist415_dev_s *)strtoul(argv[1], NULL, 16);

	dbg("\n\n\n\n\n\n**************sensor_run_on: val: %d, addr: %d, priv_addr: %p, line: %d\n\n", priv->sensor_run_on, &(priv->sensor_run_on), priv, __LINE__);

	while (1) {
		
		if (priv->sensor_run_on == false) {
			DelayMs(3000);
			continue;
		}
		
		ret = ais25ba_read_data(priv);
		dbg("%d\n", __LINE__);
		if (work_available(&priv->work)) {
			dbg("%d\n", __LINE__);
			ret = work_queue(HPWORK, &priv->work, ais25ba_send_result, priv, 0);
		}
	}
	dbg("********* ERROR ERROR ERROR!!!!!!, mq_thread is exited\n");
	return OK;
}

int ais25ba_initialize(const char *devpath, struct ais25ba_dev_s *priv)
{
	/* Setup device structure. */
	char *parm[2];
	char parm_buf[9];
	int sensor_verify_status;
	pid_t pid;

	struct sensor_upperhalf_s *upper = (struct sensor_upperhalf_s *)kmm_zalloc(sizeof(struct sensor_upperhalf_s));
	if (!upper) {
		lldbg("ERROR: upperhalf memory allocation failed\n");
	}
	upper->ops = &g_ais25ba_ops;
	upper->priv = priv;
	priv->upper = upper;

	/* Sensor Connection Verification */
	struct i2c_dev_s *i2c = priv->i2c;
	struct i2c_config_s config = priv->i2c_config;
	sem_init(&priv->ctrl.read_sem, 0, 1);
	sem_init(&priv->ctrl.callback_wait_sem, 0, 0);
	priv->ctrl.sem_timeout.tv_sec = 10;		// Seconds
	priv->ctrl.sem_timeout.tv_nsec = 100000000;	// nanoseconds
	sq_init(&priv->pendq);
	sq_init(&priv->doneq);

	itoa((int)priv, parm_buf, 16);
	parm[0] = parm_buf;
	parm[1] = NULL;

	if (ais25ba_verify_sensor(upper, i2c, config) == OK) {
		lldbg("Sensor connection verification success\n");
	} else{
		lldbg("ERROR: Sensor verification failed, sensor not found/not responding\n");
	}

	//I2C config set. Read data is to check if write register is successful or not
	ais25ba_set_config_i2c(i2c, config);

	priv->wdog = wd_create();
	if (wd_start(priv->wdog, MSEC2TICK(AIS25BA_ALIVECHECK_TIME), (wdentry_t)ais25ba_timer_handler, 1, (uint32_t)priv) != OK) {
		lldbg("Fail to start AIS25BA alive-check wdog, errno : %d\n", get_errno());
	}

	pid = kernel_thread(AIS25BA_KERNEL_MQ_THREAD, 200, 18000, (main_t)ais25ba_mq_thread, (FAR char *const *)parm);
	if (pid < 0) {
		lldbg("ais25ba_mq_thread thread creation failed\n");
		return ERROR;
	}

	return sensor_register(devpath, upper);
}
