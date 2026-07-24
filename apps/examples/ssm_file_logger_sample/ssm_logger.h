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

#ifndef __SSM_LOGGER_H
#define __SSM_LOGGER_H

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/*
 * ssm_log_add - Add a log entry to the in-memory buffer
 * @code: log code string (e.g., "ERROR", "INFO")
 * @message: log message string
 * Returns: 0 on success, -1 on failure (buffer full)
 */
EXTERN int ssm_log_add(const char *code, const char *message);

/*
 * ssm_store_logs_to_file - Write buffered logs to file
 * @timeout_ms: timeout in milliseconds (min 1000)
 * Returns: 0 on success, -1 on failure, -2 on timeout
 */
EXTERN int ssm_store_logs_to_file(unsigned int timeout_ms);

/*
 * ssm_read_logs_from_file - Read and print logs from file
 * Returns: 0 on success, -1 on failure (file not found or empty)
 */
EXTERN int ssm_read_logs_from_file(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __SSM_LOGGER_H */
