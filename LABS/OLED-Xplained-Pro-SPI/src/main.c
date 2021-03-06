#include <asf.h>
#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

typedef struct
{
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t seccond;
} calendar;

#define LED_PIO PIOC
#define LED_PIO_ID 12
#define LED_PIO_IDX 8
#define LED_PIO_IDX_MASK (1 << LED_PIO_IDX)

#define LED1_PIO PIOA
#define LED1_PIO_ID ID_PIOA
#define LED1_PIO_IDX 0
#define LED1_PIO_IDX_MASK (1 << LED1_PIO_IDX)

#define LED2_PIO PIOC
#define LED2_PIO_ID ID_PIOC
#define LED2_PIO_IDX 30
#define LED2_PIO_IDX_MASK (1 << LED2_PIO_IDX)

#define LED3_PIO PIOB
#define LED3_PIO_ID ID_PIOB
#define LED3_PIO_IDX 2
#define LED3_PIO_IDX_MASK (1 << LED3_PIO_IDX)

volatile char flag_rtc = 0;
volatile char flag_rtc1 = 0;

volatile Bool f_rtt_alarme = false;
volatile char flag_tc1 = 0;
volatile char flag_tc0 = 0;

void LED_init(int estado, int estado2);
void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq);

void pisca_led(int n, int t, int a);

void pin_toggle(Pio *pio, uint32_t mask);
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);

void TC1_Handler(void)
{
	volatile uint32_t ul_dummy;
	ul_dummy = tc_get_status(TC0, 1);
	UNUSED(ul_dummy);
	flag_tc1 = 1;
}
void TC0_Handler(void)
{
	volatile uint32_t ul_dummy;
	ul_dummy = tc_get_status(TC0, 0);
	UNUSED(ul_dummy);
	flag_tc0 = 1;
}
void RTT_Handler(void)
{
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC)
	{
	}
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS)
	{
		pin_toggle(LED2_PIO, LED2_PIO_IDX_MASK);
		f_rtt_alarme = true;
	}
}
void pin_toggle(Pio *pio, uint32_t mask)
{
	if (pio_get_output_data_status(pio, mask))
		pio_clear(pio, mask);
	else
		pio_set(pio, mask);
}
void LED_init(int estado, int estado2)
{
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, estado, 0, 0);
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, estado2, 0, 0);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_PIO_IDX_MASK, PIO_DEFAULT);
}

static float get_time_rtt()
{
	uint ul_previous_time = rtt_read_timer_value(RTT);
}
void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq)
{
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
	pmc_enable_periph_clk(ID_TC);
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);
	NVIC_EnableIRQ((IRQn_Type)ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
	tc_start(TC, TC_CHANNEL);
}
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT))
		;
	rtt_write_alarm_time(RTT, IrqNPulses + ul_previous_time);
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
}
void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC)
	{
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
		flag_rtc1 = 1;
	}
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM)
	{
		rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
		flag_rtc = 1;
	}
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type)
{
	pmc_enable_periph_clk(ID_RTC);
	rtc_set_hour_mode(rtc, 0);
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.seccond);
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 0);
	NVIC_EnableIRQ(id_rtc);
	rtc_enable_interrupt(rtc, irq_type);
}
void pisca_led(int n, int t, int led)
{
	for (int i = 0; i < n; i++)
	{
		if (led == 1)
		{
			pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
			delay_ms(t);
			pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
			delay_ms(t);
		}
		if (led == 3)
		{
			pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
			delay_ms(t);
			pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
			delay_ms(t);
		}
		if (led == 0)
		{
			pio_clear(LED_PIO, LED_PIO_IDX_MASK);
			delay_ms(t);
			pio_set(LED_PIO, LED_PIO_IDX_MASK);
			delay_ms(t);
		}
	}
}
int main(void)
{
	board_init();
	sysclk_init();
	delay_init();
	WDT->WDT_MR = WDT_MR_WDDIS;
	LED_init(1, 0);
	f_rtt_alarme = true;
	gfx_mono_ssd1306_init();
	calendar rtc_initial = {2018, 3, 19, 12, 15, 45, 1};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN | RTC_IER_SECEN);
	rtc_set_date_alarm(RTC, 1, rtc_initial.month, 1, rtc_initial.day);
	rtc_set_time_alarm(RTC, 1, rtc_initial.hour, 1, rtc_initial.minute, 1, rtc_initial.seccond + 20);
	TC_init(TC0, ID_TC1, 1, 4);
	TC_init(TC0, ID_TC0, 0, 5);

	while (1)
	{
		if (flag_rtc)
		{
			pisca_led(5, 200, 1);
			flag_rtc = 0;
		}

		if (flag_rtc1)
		{
			rtc_get_time(RTC, &rtc_initial.hour, &rtc_initial.minute, &rtc_initial.seccond);
			char b[200];
			sprintf(b, "%2d : %2d : %2d", rtc_initial.hour, rtc_initial.minute, rtc_initial.seccond);
			gfx_mono_draw_string(b, 1, 13, &sysfont);
			flag_rtc1 = 0;
		}
		if (f_rtt_alarme)
		{
			uint16_t pllPreScale = (int)(((float)32768) / 4.0);
			uint32_t irqRTTvalue = 8;
			RTT_init(pllPreScale, irqRTTvalue);
			f_rtt_alarme = false;
		}
		if (flag_tc1)
		{
			pisca_led(1, 10, 3);
			flag_tc1 = 0;
		}
		if (flag_tc0)
		{
			pisca_led(1, 10, 0);
			flag_tc0 = 0;
		}
	}
}