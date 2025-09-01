#include"stm32l4xx.h"
#include"stdio.h"
#include"RTC.h"
#include"LCD.h"

#define SYSTICK_TIM_CLK   16000000UL   // SysTick clock = 16 MHz
#define PRINT_LCD          // Uncomment to use LCD instead of printf

// Function prototypes
void init_systick_timer(uint32_t tick_hz);
char* get_day_of_week(uint8_t i);
void number_to_string(uint8_t num , char* buf);
char* time_to_string(RTC_time_t *rtc_time);
char* date_to_string(RTC_date_t *rtc_date);

int main(void)
{
	RTC_time_t current_time;
	RTC_date_t current_date;

#ifndef PRINT_LCD
	// Use printf output
	printf("RTC test\n");
#else
	// Initialize LCD and display test message
	lcd_init();
	lcd_print_string("RTC Test...");
	mdelay(2000);
	lcd_display_clear();
	lcd_display_return_home();
#endif

	// Initialize DS1307 RTC
	if(ds1307_init()){
#ifndef PRINT_LCD
		printf("RTC init has failed\n");
#else
		lcd_print_string("Init FAIL!");
#endif
		while(1); // Stop if RTC init fails
	}

	// Start SysTick timer at 1Hz (1 interrupt per second)
	init_systick_timer(1);

	// ✅ Set today's date (Monday, 01/09/25)
	current_date.day   = MONDAY;   // Day of the week
	current_date.date  = 1;        // Day number
	current_date.month = 9;        // September
	current_date.year  = 25;       // Year = 2025 (last two digits)

	// ✅ Set current time (10:30:00 AM)
	current_time.hours       = 10;
	current_time.minutes     = 30;
	current_time.seconds     = 0;
	current_time.time_format = TIME_FORMAT_12HRS_AM;

	// Write initial time & date to RTC
	ds1307_set_current_date(&current_date);
	ds1307_set_current_time(&current_time);

	// Read back current time & date from RTC
	ds1307_get_current_time(&current_time);
	ds1307_get_current_date(&current_date);

	// Display time
	char *am_pm;
	if(current_time.time_format != TIME_FORMAT_24HRS)
	{
		am_pm = (current_time.time_format) ? "PM" : "AM";
#ifndef PRINT_LCD
		printf("Current time = %s %s\n", time_to_string(&current_time), am_pm);
#else
		lcd_print_string(time_to_string(&current_time));
		lcd_print_string(am_pm);
#endif
	}
	else
	{
#ifndef PRINT_LCD
		printf("Current time = %s\n", time_to_string(&current_time));
#else
		lcd_print_string(time_to_string(&current_time));
#endif
	}

	// Display date
#ifndef PRINT_LCD
	printf("Current date = %s <%s>\n", date_to_string(&current_date), get_day_of_week(current_date.day));
#else
	lcd_set_cursor(2, 1);
	lcd_print_string(date_to_string(&current_date));
	lcd_print_char('<');
	lcd_print_string(get_day_of_week(current_date.day));
	lcd_print_char('>');
#endif

	while(1); // Main loop runs forever

	return 0;
}

// Convert day number to string (1=Sunday, 7=Saturday)
char* get_day_of_week(uint8_t i)
{
	char* days[] = { "Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday"};
	return days[i-1];
}

// Convert number (0-99) into 2-character string
void number_to_string(uint8_t num , char* buf)
{
	if(num < 10){
		buf[0] = '0';
		buf[1] = num + 48; // ASCII conversion
	}else if(num >= 10 && num < 99){
		buf[0] = (num/10) + 48;
		buf[1] = (num % 10) + 48;
	}
}

// Format time as "HH:MM:SS"
char* time_to_string(RTC_time_t *rtc_time)
{
	static char buf[9];

	buf[2]= ':';
	buf[5]= ':';

	number_to_string(rtc_time->hours, buf);
	number_to_string(rtc_time->minutes, &buf[3]);
	number_to_string(rtc_time->seconds, &buf[6]);

	buf[8] = '\0'; // Null terminator
	return buf;
}

// Format date as "DD/MM/YY"
char* date_to_string(RTC_date_t *rtc_date)
{
	static char buf[9];

	buf[2]= '/';
	buf[5]= '/';

	number_to_string(rtc_date->date, buf);
	number_to_string(rtc_date->month, &buf[3]);
	number_to_string(rtc_date->year, &buf[6]);

	buf[8]= '\0'; // Null terminator
	return buf;
}

// Configure SysTick timer to generate periodic interrupts
void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014; // Reload register
	uint32_t *pSCSR = (uint32_t*)0xE000E010; // Control register
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;

    *pSRVR &= ~(0x00FFFFFFFF); // Clear reload value
    *pSRVR |= count_value;     // Set reload value
    *pSCSR |= (1 << 1);        // Enable SysTick exception
    *pSCSR |= (1 << 2);        // Select processor clock
    *pSCSR |= (1 << 0);        // Enable SysTick
}

// SysTick interrupt handler: updates LCD/printf every second
void SysTick_Handler(void)
{
	RTC_time_t current_time;
	RTC_date_t current_date;

	// Read time from RTC
	ds1307_get_current_time(&current_time);

	// Display time
	char *am_pm;
	if(current_time.time_format != TIME_FORMAT_24HRS)
	{
		am_pm = (current_time.time_format) ? "PM" : "AM";
#ifndef PRINT_LCD
		printf("Current time = %s %s\n", time_to_string(&current_time), am_pm);
#else
		lcd_set_cursor(1, 1);
		lcd_print_string(time_to_string(&current_time));
		lcd_print_string(am_pm);
#endif
	}
	else
	{
#ifndef PRINT_LCD
		printf("Current time = %s\n", time_to_string(&current_time));
#else
		lcd_set_cursor(1, 1);
		lcd_print_string(time_to_string(&current_time));
#endif
	}

	// Read date from RTC
	ds1307_get_current_date(&current_date);

	// Display date
#ifndef PRINT_LCD
	printf("Current date = %s <%s>\n", date_to_string(&current_date), get_day_of_week(current_date.day));
#else
	lcd_set_cursor(2, 1);
	lcd_print_string(date_to_string(&current_date));
	lcd_print_char('<');
	lcd_print_string(get_day_of_week(current_date.day));
	lcd_print_char('>');
#endif
}
