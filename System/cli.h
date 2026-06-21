#ifndef CLI_H
#define CLI_H

#include "stm32f10x.h"

#define CLI_CMD_BUF_SIZE 128
#define CLI_MAX_ARGS    8
#define CLI_PROMPT      "STM32> "

void CLI_Task(void *pvParameters);

#endif
