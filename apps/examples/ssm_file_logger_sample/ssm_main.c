/****************************************************************************
 *
 * Copyright 2026 Samsung Electronics All Rights Reserved.
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
 * SSM File Logger Sample
 *
 * Commands:
 *   ssm write <code> <message>   — add a log entry and write all to file
 *   ssm read                     — read back and print stored logs
 *   ssm erase                    — delete the log file
 *
 * Uses: pthread_create, sem_init/sem_timedwait/sem_post/sem_destroy,
 *       pthread_mutex_init/lock/unlock/destroy, fopen/fwrite/fclose,
 *       stat, unlink, rename
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdbool.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <tinyara/arch.h>

/* ioctl command for assert callback registration */
#define IOCTL_ASSERT_CALLBACK_REGISTER 0x9001

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SSM_LOG_FILE             "/tmp/ssm_logs"
#define SSM_LOG_TEMP_FILE        "/tmp/ssm_logs_temp"
#define MAX_LOG_ENTRIES          32
#define LOG_CODE_MAX_SIZE        8
#define LOG_MSG_MAX_SIZE         128
#define LOG_LINE_MAX_SIZE        256
#define WORKER_STACK_SIZE        (1024 * 6)
#define STORE_TIMEOUT_MS         5000

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct {
	int time;
	char code[LOG_CODE_MAX_SIZE];
	char message[LOG_MSG_MAX_SIZE];
} ssm_log_entry;

typedef struct {
	ssm_log_entry entries[MAX_LOG_ENTRIES];
	int count;
	pthread_mutex_t lock;
} ssm_log_buffer;

typedef struct {
	int result;
	sem_t completion_sem;
	bool completed;
} file_write_context;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static ssm_log_buffer g_log_buffer;
static file_write_context g_write_ctx;
static bool g_store_api_done = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ssm_log_buffer_init
 ****************************************************************************/

static void ssm_log_buffer_init(void)
{
	memset(&g_log_buffer, 0, sizeof(g_log_buffer));
	pthread_mutex_init(&g_log_buffer.lock, NULL);
}

/****************************************************************************
 * Name: ssm_log_add
 *
 * Description:
 *   Add a log entry to the in-memory buffer. Thread-safe via mutex.
 *
 ****************************************************************************/

int ssm_log_add(const char *code, const char *message)
{
	int ret = 0;

	pthread_mutex_lock(&g_log_buffer.lock);

	if (g_log_buffer.count >= MAX_LOG_ENTRIES) {
		printf("ssm: log buffer is full (%d entries)\n", MAX_LOG_ENTRIES);
		ret = -1;
		goto out;
	}

	g_log_buffer.entries[g_log_buffer.count].time = (int)time(NULL);
	snprintf(g_log_buffer.entries[g_log_buffer.count].code,
			 LOG_CODE_MAX_SIZE, "%s", code ? code : "");
	snprintf(g_log_buffer.entries[g_log_buffer.count].message,
			 LOG_MSG_MAX_SIZE, "%s", message ? message : "");
	g_log_buffer.count++;

	printf("ssm: added log [%d] code=%s msg=%s\n",
		   g_log_buffer.count, code ? code : "", message ? message : "");

out:
	pthread_mutex_unlock(&g_log_buffer.lock);
	return ret;
}

/****************************************************************************
 * Name: ssm_file_is_exist
 ****************************************************************************/

static int ssm_file_is_exist(const char *path)
{
	struct stat st;

	if (path == NULL) {
		return -1;
	}

	if (stat(path, &st) != 0) {
		return -1;
	}

	return 0;
}

/****************************************************************************
 * Name: ssm_write_logs_to_file
 *
 * Description:
 *   Worker thread function. Opens a temp file, writes all log entries from
 *   the in-memory buffer, closes the file, then:
 *     1. Checks if the old log file exists
 *     2. Deletes the old file if it exists (commit 6688bf7 logic)
 *     3. Renames the temp file to the final file
 *   Signals completion via semaphore.
 *
 ****************************************************************************/

static void *ssm_write_logs_to_file(void *arg)
{
	FILE *fp = NULL;
	int i;

	/* Open temp file for writing */
	fp = fopen(SSM_LOG_TEMP_FILE, "w");
	if (fp == NULL) {
		printf("ssm: failed to open %s, errno=%d\n",
			   SSM_LOG_TEMP_FILE, errno);
		g_write_ctx.completed = true;
		g_write_ctx.result = -1;
		sem_post(&g_write_ctx.completion_sem);
		return NULL;
	}

	/* Lock the log buffer and write all entries to file */
	pthread_mutex_lock(&g_log_buffer.lock);

	for (i = 0; i < g_log_buffer.count; i++) {
		char line[LOG_LINE_MAX_SIZE];
		int len;

		len = snprintf(line, sizeof(line), "<0#%d#%s#%s>\n",
					   g_log_buffer.entries[i].time,
					   g_log_buffer.entries[i].code,
					   g_log_buffer.entries[i].message);

		if ((int)fwrite(line, 1, len, fp) != len) {
			printf("ssm: fwrite failed, errno=%d\n", errno);
			pthread_mutex_unlock(&g_log_buffer.lock);
			fclose(fp);
			g_write_ctx.completed = true;
			g_write_ctx.result = -1;
			sem_post(&g_write_ctx.completion_sem);
			return NULL;
		}
	}

	pthread_mutex_unlock(&g_log_buffer.lock);

	/* Close the file */
	fclose(fp);

	/*
	 * Logic from commit 6688bf7:
	 * Before renaming temp -> final, check if old log file exists.
	 * If it does, delete it first to ensure clean atomic rename.
	 */
	if (ssm_file_is_exist(SSM_LOG_FILE) == 0) {
		if (unlink(SSM_LOG_FILE) != 0) {
			printf("ssm: unable to delete old log file, errno=%d\n", errno);
			g_write_ctx.completed = true;
			g_write_ctx.result = -1;
			sem_post(&g_write_ctx.completion_sem);
			return NULL;
		}
		printf("ssm: deleted old log file\n");
	}

	/* Rename temp file to final file */
	if (rename(SSM_LOG_TEMP_FILE, SSM_LOG_FILE) != 0) {
		printf("ssm: rename failed, errno=%d\n", errno);
		g_write_ctx.completed = true;
		g_write_ctx.result = -1;
		sem_post(&g_write_ctx.completion_sem);
		return NULL;
	}

	printf("ssm: wrote %d log entries to %s\n",
		   g_log_buffer.count, SSM_LOG_FILE);

	g_write_ctx.completed = true;
	g_write_ctx.result = 0;
	sem_post(&g_write_ctx.completion_sem);
	return NULL;
}

/****************************************************************************
 * Name: ssm_store_logs_to_file
 *
 * Description:
 *   Creates a semaphore, spawns a worker thread, waits for completion
 *   with a timeout. Mirrors SSM_store_logs_to_file_impl().
 *
 ****************************************************************************/

int ssm_store_logs_to_file(unsigned int timeout_ms)
{
	pthread_t worker;
	pthread_attr_t attr;
	struct timespec abstime;
	int ret;

	if (g_store_api_done) {
		printf("ssm: store already called, reset buffer first\n");
		return -1;
	}

	if (timeout_ms < 1000) {
		printf("ssm: invalid timeout %u (min 1000)\n", timeout_ms);
		return -1;
	}

	g_write_ctx.result = -1;
	g_write_ctx.completed = false;

	/* Initialize completion semaphore with count 0 */
	if (sem_init(&g_write_ctx.completion_sem, 0, 0) != 0) {
		printf("ssm: sem_init failed, errno=%d\n", errno);
		return -1;
	}

	/* Create worker thread */
	pthread_attr_init(&attr);
	pthread_attr_setstacksize(&attr, WORKER_STACK_SIZE);

	ret = pthread_create(&worker, &attr, ssm_write_logs_to_file, NULL);
	pthread_attr_destroy(&attr);

	if (ret != 0) {
		printf("ssm: pthread_create failed, ret=%d\n", ret);
		sem_destroy(&g_write_ctx.completion_sem);
		return -1;
	}

	/* Calculate absolute timeout time */
	clock_gettime(CLOCK_REALTIME, &abstime);
	abstime.tv_sec += timeout_ms / 1000;
	abstime.tv_nsec += (timeout_ms % 1000) * 1000000;
	if (abstime.tv_nsec >= 1000000000) {
		abstime.tv_sec++;
		abstime.tv_nsec -= 1000000000;
	}

	/* Wait for worker to complete or timeout */
	ret = sem_timedwait(&g_write_ctx.completion_sem, &abstime);

	if (ret == -1 && errno == ETIMEDOUT) {
		printf("ssm: timed out after %u ms\n", timeout_ms);
		g_store_api_done = true;
		sem_destroy(&g_write_ctx.completion_sem);
		pthread_detach(worker);
		return -2;
	}

	/* Worker completed — get result */
	int result = g_write_ctx.result;
	sem_destroy(&g_write_ctx.completion_sem);
	pthread_join(worker, NULL);

	g_store_api_done = true;
	return result;
}

/****************************************************************************
 * Name: ssm_read_logs_from_file
 *
 * Description:
 *   Reads back the stored log file and prints its contents.
 *
 ****************************************************************************/

int ssm_read_logs_from_file(void)
{
	FILE *fp;
	char line[LOG_LINE_MAX_SIZE];
	int count = 0;

	if (ssm_file_is_exist(SSM_LOG_FILE) != 0) {
		printf("ssm: %s does not exist\n", SSM_LOG_FILE);
		return -1;
	}

	fp = fopen(SSM_LOG_FILE, "r");
	if (fp == NULL) {
		printf("ssm: failed to open %s, errno=%d\n", SSM_LOG_FILE, errno);
		return -1;
	}

	printf("=== Reading stored logs from %s ===\n", SSM_LOG_FILE);

	while (fgets(line, sizeof(line), fp) != NULL) {
		printf("  [%d] %s", count + 1, line);
		count++;
	}

	fclose(fp);

	printf("=== Total %d log entries read ===\n", count);

	if (count == 0) {
		printf("ssm: no logs found in file\n");
		return -1;
	}

	return 0;
}

/****************************************************************************
 * Name: ssm_erase_log_file
 *
 * Description:
 *   Deletes the stored log file.
 *
 ****************************************************************************/

static int ssm_erase_log_file(void)
{
	if (ssm_file_is_exist(SSM_LOG_FILE) != 0) {
		printf("ssm: %s does not exist, nothing to erase\n", SSM_LOG_FILE);
		return -1;
	}

	if (unlink(SSM_LOG_FILE) != 0) {
		printf("ssm: failed to delete %s, errno=%d\n", SSM_LOG_FILE, errno);
		return -1;
	}

	printf("ssm: erased %s\n", SSM_LOG_FILE);
	return 0;
}

/****************************************************************************
 * Name: ssm_print_usage
 ****************************************************************************/

static void ssm_print_usage(void)
{
	printf("Usage:\n");
	printf("  ssm write <code> <message>   Add a log entry and write to file\n");
	printf("  ssm read                      Read back stored logs\n");
	printf("  ssm erase                     Delete the log file\n");
	printf("  ssm register                  Register assert callback\n");
	printf("  ssm assert_test               Test: write -> read -> ASSERT(0)\n");
}

/****************************************************************************
 * Name: ssm_assert_callback
 *
 * Description:
 *   Callback function called by kernel on assert.
 *   Writes a test message to the log file using low-level syscalls.
 ****************************************************************************/

static void ssm_assert_callback(const char *msg)
{
	int fd;
	int len;
	int written;
	char log_msg[256];

	printf("ssm: [USER] assert callback ENTERED with msg: %s", msg);

	/* Prepare log message */
	len = snprintf(log_msg, sizeof(log_msg), "<ASSERT_CALLBACK#%s>\n", msg);

	/* Open log file using low-level syscall */
	fd = open(SSM_LOG_FILE, O_WRONLY | O_CREAT | O_APPEND, 0666);
	if (fd < 0) {
		printf("ssm: [USER] open failed, errno=%d\n", errno);
		return;
	}
	printf("ssm: [USER] file opened, fd=%d\n", fd);

	/* Write directly using syscall */
	written = write(fd, log_msg, len);
	if (written < 0) {
		printf("ssm: [USER] write failed, errno=%d\n", errno);
		close(fd);
		return;
	}
	printf("ssm: [USER] wrote %d bytes\n", written);

	/* Sync to disk */
	fsync(fd);
	printf("ssm: [USER] fsync done\n");

	close(fd);
	printf("ssm: [USER] file closed, callback wrote to %s\n", SSM_LOG_FILE);
}

/****************************************************************************
 * Name: ssm_register_callback
 *
 * Description:
 *   Register the assert callback with the kernel via ioctl on /dev/null.
 ****************************************************************************/

static void ssm_register_callback(void)
{
	int fd;

	fd = open("/dev/null", O_RDWR);
	if (fd < 0) {
		printf("ssm: failed to open /dev/null, errno=%d\n", errno);
		return;
	}

	if (ioctl(fd, IOCTL_ASSERT_CALLBACK_REGISTER, (unsigned long)ssm_assert_callback) < 0) {
		printf("ssm: ioctl failed, errno=%d\n", errno);
		close(fd);
		return;
	}

	close(fd);
	printf("ssm: assert callback registered successfully via ioctl\n");
}

/****************************************************************************
 * Name: ssm_assert_test
 *
 * Description:
 *   Test case that performs ssm write, then ssm read, then triggers ASSERT(0).
 ****************************************************************************/

static void ssm_assert_test(void)
{
	int ret;

	ret = ssm_log_add("ASSERT_TEST", "Test message before assert");
	if (ret != 0) {
		printf("ssm_log_add failed\n");
		return;
	}

	ret = ssm_store_logs_to_file(STORE_TIMEOUT_MS);
	if (ret != 0) {
		printf("ssm_store_logs_to_file failed (ret=%d)\n", ret);
		return;
	}

	ret = ssm_read_logs_from_file();
	if (ret != 0) {
		printf("ssm_read_logs_from_file failed\n");
	}
	printf("SSM read completed\n");

	/* Step 3: Trigger ASSERT(0) */
	printf("calling ASSERT(0)\n");
	ASSERT(0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ssm_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int ssm_main(int argc, char *argv[])
#endif
{
	int ret;

	if (argc < 2) {
		ssm_print_usage();
		return -1;
	}

	/* Initialize the log buffer and mutex (idempotent) */
	ssm_log_buffer_init();

	if (strcmp(argv[1], "write") == 0) {
		if (argc < 4) {
			printf("ssm: write requires <code> and <message>\n");
			ssm_print_usage();
			return -1;
		}

		/* Add the log entry from command-line args */
		ret = ssm_log_add(argv[2], argv[3]);
		if (ret != 0) {
			return -1;
		}

		/* Write all buffered entries to file */
		ret = ssm_store_logs_to_file(STORE_TIMEOUT_MS);
		if (ret != 0) {
			printf("ssm: failed to write logs to file (ret=%d)\n", ret);
			return -1;
		}

		printf("ssm: write success\n");

	} else if (strcmp(argv[1], "read") == 0) {
		ret = ssm_read_logs_from_file();
		if (ret != 0) {
			return -1;
		}

	} else if (strcmp(argv[1], "erase") == 0) {
		ret = ssm_erase_log_file();
		if (ret != 0) {
			return -1;
		}

	} else if (strcmp(argv[1], "register") == 0) {
		ssm_register_callback();

	} else if (strcmp(argv[1], "assert_test") == 0) {
		ssm_assert_test();

	} else {
		printf("ssm: unknown command '%s'\n", argv[1]);
		ssm_print_usage();
		return -1;
	}

	return 0;
}
