#include "cli.h"
#include "usart.h"
#include "fly_ctrl.h"
#include "pid.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static char cmd_buf[CLI_CMD_BUF_SIZE];
static uint16_t cmd_len = 0;

static void CLI_Send(const char *str)
{
	while (*str)
	{
		fputc((int)*str, NULL);
		str++;
	}
}

static void CLI_SendLine(const char *str)
{
	CLI_Send(str);
	CLI_Send("\r\n");
}

static uint8_t CLI_ParseArgs(char *cmd, char *args[], uint8_t max_args)
{
	uint8_t argc = 0;
	char *p = cmd;
	while (*p == ' ') p++;
	if (*p == '\0') return 0;
	while (argc < max_args && *p != '\0')
	{
		args[argc++] = p;
		while (*p != ' ' && *p != '\0') p++;
		if (*p == ' ')
		{
			*p++ = '\0';
			while (*p == ' ') p++;
		}
	}
	return argc;
}

static void CLI_Help(void)
{
	CLI_SendLine("Available commands:");
	CLI_SendLine("  help                    - Show this help");
	CLI_SendLine("  pid list                - List all PID gains");
	CLI_SendLine("  pid get <name>          - Get PID gains");
	CLI_SendLine("  pid set <name> <kp> <ki> <kd> - Set PID gains");
	CLI_SendLine("  pid reset <name>        - Reset PID integral");
	CLI_SendLine("  status                  - Show flight status");
	CLI_SendLine("PID names: roll_angle, pitch_angle, roll_rate, pitch_rate, yaw_rate");
}

static PID_t* CLI_FindPID(const char *name)
{
	if (strcmp(name, "roll_angle")  == 0) return Fly_GetRollAnglePID();
	if (strcmp(name, "pitch_angle") == 0) return Fly_GetPitchAnglePID();
	if (strcmp(name, "roll_rate")   == 0) return Fly_GetRollRatePID();
	if (strcmp(name, "pitch_rate")  == 0) return Fly_GetPitchRatePID();
	if (strcmp(name, "yaw_rate")    == 0) return Fly_GetYawRatePID();
	return NULL;
}

static void CLI_PIDList(void)
{
	static const char *names[] = {
		"roll_angle", "pitch_angle", "roll_rate", "pitch_rate", "yaw_rate"
	};
	PID_t *pid;
	char line[80];
	uint8_t i;
	or (i = 0; i < 5; i++)
	{
		pid = CLI_FindPID(names[i]);
		if (pid)
		{
			snprintf(line, sizeof(line), "  %s: Kp=%.3f  Ki=%.3f  Kd=%.3f  i_limit=%.1f  out_limit=%.1f",
				names[i], PID_GetKp(pid), PID_GetKi(pid), PID_GetKd(pid),
				pid->integral_limit, pid->output_limit);
			CLI_SendLine(line);
		}
	}
}

static void CLI_PIDGet(const char *name)
{
	PID_t *pid = CLI_FindPID(name);
	char line[80];
	if (!pid)
	{
		CLI_SendLine("Error: unknown PID name");
		return;
	}
	snprintf(line, sizeof(line), "%s: Kp=%.3f  Ki=%.3f  Kd=%.3f",
		name, PID_GetKp(pid), PID_GetKi(pid), PID_GetKd(pid));
	CLI_SendLine(line);
}

static void CLI_PIDSet(const char *name, float kp, float ki, float kd)
{
	PID_t *pid = CLI_FindPID(name);
	char line[80];
	if (!pid)
	{
		CLI_SendLine("Error: unknown PID name");
		return;
	}
	PID_SetGains(pid, kp, ki, kd);
	snprintf(line, sizeof(line), "%s set: Kp=%.3f  Ki=%.3f  Kd=%.3f", name, kp, ki, kd);
	CLI_SendLine(line);
}

static void CLI_PIDReset(const char *name)
{
	PID_t *pid = CLI_FindPID(name);
	if (!pid)
	{
		CLI_SendLine("Error: unknown PID name");
		return;
	}
	PID_Reset(pid);
	CLI_SendLine("PID integral reset");
}

static void CLI_Status(void)
{
	Motor_Out motor;
	Fly_Control_GetMotorOut(&motor);
	{
		char line[80];
		snprintf(line, sizeof(line), "M1=%.0f  M2=%.0f  M3=%.0f  M4=%.0f",
			(double)motor.m1, (double)motor.m2, (double)motor.m3, (double)motor.m4);
		CLI_SendLine(line);
	}
}

static void CLI_ProcessLine(char *line)
{
	char *args[CLI_MAX_ARGS];
	uint8_t argc = CLI_ParseArgs(line, args, CLI_MAX_ARGS);
	if (argc == 0) return;

	if (strcmp(args[0], "help") == 0)
	{
		CLI_Help();
	}
	else if (strcmp(args[0], "pid") == 0)
	{
		if (argc < 2) { CLI_SendLine("Usage: pid list|get|set|reset"); return; }
		if (strcmp(args[1], "list") == 0)
		{
			CLI_PIDList();
		}
		else if (strcmp(args[1], "get") == 0)
		{
			if (argc < 3) { CLI_SendLine("Usage: pid get <name>"); return; }
			CLI_PIDGet(args[2]);
		}
		else if (strcmp(args[1], "set") == 0)
		{
			if (argc < 5) { CLI_SendLine("Usage: pid set <name> <kp> <ki> <kd>"); return; }
			CLI_PIDSet(args[2], atof(args[3]), atof(args[4]), atof(args[5]));
		}
		else if (strcmp(args[1], "reset") == 0)
		{
			if (argc < 3) { CLI_SendLine("Usage: pid reset <name>"); return; }
			CLI_PIDReset(args[2]);
		}
		else
		{
			CLI_SendLine("Unknown pid subcommand");
		}
	}
	else if (strcmp(args[0], "status") == 0)
	{
		CLI_Status();
	}
	else
	{
		CLI_Send("Unknown command: ");
		CLI_SendLine(args[0]);
		CLI_SendLine("Type 'help' for available commands");
	}
}

void CLI_Task(void *pvParameters)
{
	(void)pvParameters;

	UART_EnableRX();
	CLI_SendLine("\r\n--- STM32 Flight Controller CLI ---");
	CLI_Send(CLI_PROMPT);

	while (1)
	{
		uint8_t ch;
		if (UART_GetChar(&ch))
		{
			if (ch == '\r' || ch == '\n')
			{
				CLI_Send("\r\n");
				cmd_buf[cmd_len] = '\0';
				if (cmd_len > 0)
				{
					CLI_ProcessLine(cmd_buf);
				}
				cmd_len = 0;
				CLI_Send(CLI_PROMPT);
			}
			else if (ch == '\b' || ch == 0x7F)
			{
				if (cmd_len > 0)
				{
					cmd_len--;
					CLI_Send("\b \b");
				}
			}
			else if (cmd_len < CLI_CMD_BUF_SIZE - 1)
			{
				if (ch >= 0x20 && ch <= 0x7E)
				{
					cmd_buf[cmd_len++] = ch;
					fputc(ch, NULL);
				}
			}
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}
