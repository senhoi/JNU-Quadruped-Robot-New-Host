#include "main.h"
#ifdef __RPI
#include <wiringPi.h>
#endif

#define CALC_CYCLE_US 10000

extern int usleep(__useconds_t __useconds);

static uint64_t sys_timer_us()
{
	struct timeval tv;

	gettimeofday(&tv, NULL);

	return tv.tv_sec * 1000 * 1000 + tv.tv_usec;
}

static void handle_IRQ(void)
{
	InterruptTask();
}

Sys_Setting_t setting;

void sys_exit(int signo)
{
	printf("\nProgram aborted.\n");
	serialClose(setting.fd_uart);
	if (setting.log.enable == ON)
	{
		if (!fclose(setting.log.fd))
			printf("Log file saved. Filename is %s.\n", setting.log.filename);
		else
			printf("Failed to closed log file. \n");
	}

	_exit(0);
}

void sys_init(Sys_Interface_t interface, Sys_SynSig_t syn_sig, Sys_EnLog_t log_enable)
{
	setting.interface = interface;
	setting.syn_sig = syn_sig;
	setting.log.enable = log_enable;

	if (setting.syn_sig == GPIO)
	{
#ifdef __RPI
		wiringPiSetup();
		wiringPiISR(1, INT_EDGE_FALLING, &handle_IRQ);
#endif
	}

	if (setting.log.enable == ON)
	{
		char str_data[64], str_cwd[64];
		time_t timer = time(NULL);

		strftime(str_data, sizeof(str_data), "%Y-%m-%d[%H:%M:%S]", localtime(&timer));
		strcat(setting.log.filename, str_data);
		strcat(setting.log.filename, ".txt");
		setting.log.fd = fopen(setting.log.filename, "w");

		if (setting.log.fd == NULL)
			printf("Failed to create log file.\n");
		else
			LOG(setting.log.fd, "%s", setting.log.filename);
	}
	else
	{
		setting.log.fd = NULL;
	}

	if (setting.interface == GPIO)
	{
		setting.fd_uart = serialOpen("/dev/ttyAMA0", 115200);
		PRINTF_TIPS("Open /dev/ttyAMA0");
	}
	else if (setting.interface == USB)
	{
		setting.fd_uart = serialOpen("/dev/ttyUSB0", 115200);
		PRINTF_TIPS("Open /dev/ttyUSB0");
	}
	if (setting.fd_uart < 0)
	{
		PRINTF_WARNING("SerialPort failed to initialize. -err:%d", setting.fd_uart);
		PRINTF_TIPS("Please check whether there is a serial device plugged in.");
		PRINTF_TIPS("Press ENTER to continue without connection.");
		getchar();
	}
	else
	{
		PRINTF_TIPS("SerialPort initialized successfully. fd:%d", setting.fd_uart);
	}

	signal(SIGINT, sys_exit);
	if (setProgPri(100) != 0)
	{
		PRINTF_WARNING("Set priority unsuccessfully.");
		PRINTF_TIPS("Please retry with [sudo].");
		PRINTF_TIPS("Press ENTER to continue without change priority.");
		getchar();
	}

	PRINTF_TIPS("Sys has been initialized. Press ENTER to continue.\n");
	getchar();

	InitTask();

	//serialTest(setting.fd_uart, 0x55AA);
}

void sys_loop()
{
	uint64_t time_begin_us, time_end_us;
	int64_t time_diff_us;

	while (1)
	{
		if (setting.syn_sig == TIMER)
		{
			time_begin_us = sys_timer_us();
			handle_IRQ();
			LowPriorityTask();
			time_end_us = sys_timer_us();

			time_diff_us = time_end_us - time_begin_us;

			if (time_diff_us < CALC_CYCLE_US)
				usleep(CALC_CYCLE_US - time_diff_us);
			else
				PRINTF_WARNING("Execute Overtime, -beyond_us:\t%ld", time_diff_us);
		}
		else if (setting.syn_sig == INTERRUPT)
		{
			LowPriorityTask();
		}
	}
}

int main()
{
	sys_init(USB, TIMER, OFF);
	sys_loop();
}
