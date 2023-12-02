#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define BUT_PLACA_PIO     PIOA
#define BUT_PLACA_PIO_ID  ID_PIOA
#define BUT_PLACA_PIO_PIN 11
#define BUT_PLACA_PIO_PIN_MASK (1 << BUT_PLACA_PIO_PIN)

// # placa
#define LED_PIO       PIOC
#define LED_PIO_ID    ID_PIOC
#define LED_IDX       8u
#define LED_IDX_MASK  (1 << LED_IDX)

// # (1)
#define LED1_PIO			PIOA
#define LED1_PIO_ID			ID_PIOA
#define LED1_PIO_IDX		0
#define LED1_PIO_IDX_MASK (1 << LED1_PIO_IDX)

// # (2)
#define LED2_PIO           PIOC                 
#define LED2_PIO_ID        ID_PIOC              
#define LED2_PIO_IDX       30                   
#define LED2_PIO_IDX_MASK  (1 << LED2_PIO_IDX)

// # (3)
#define LED3_PIO			PIOB
#define LED3_PIO_ID			ID_PIOB
#define LED3_PIO_IDX		2
#define LED3_PIO_IDX_MASK	(1 << LED3_PIO_IDX)


xSemaphoreHandle xSemaphore;
xSemaphoreHandle xSemaphoreClick;
xSemaphoreHandle xSemaphoreRTC2;
xSemaphoreHandle xSemaphoreRTCA;

typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
} calendar;

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_PRINTCONSOLE_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_PRINTCONSOLE_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);


/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/
volatile int flag_rtt = 0;
volatile char flag_rtc_alarm = 0;

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/
void pin_toggle(Pio *pio, uint32_t mask);
void pisca_led(int n, int t);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void but_callback(void);
static void BUT_init(void);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* RTT                		                                            */
/************************************************************************/

void RTT_Handler(void) {
  uint32_t ul_status;
  ul_status = rtt_get_status(RTT);

  /* IRQ due to Alarm */
  if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
   }  
}

static float get_time_rtt(){
  uint ul_previous_time = rtt_read_timer_value(RTT); 
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

  uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
  rtt_sel_source(RTT, false);
  rtt_init(RTT, pllPreScale);
  
  if (rttIRQSource & RTT_MR_ALMIEN) {
	uint32_t ul_previous_time;
  	ul_previous_time = rtt_read_timer_value(RTT);
  	while (ul_previous_time == rtt_read_timer_value(RTT));
  	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
  }

  /* config NVIC */
  NVIC_DisableIRQ(RTT_IRQn);
  NVIC_ClearPendingIRQ(RTT_IRQn);
  NVIC_SetPriority(RTT_IRQn, 4);
  NVIC_EnableIRQ(RTT_IRQn);

  /* Enable RTT interrupt */
  if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
  else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
		  
}

// RTT_init(4, 1, RTT_MR_ALMIEN);  // inicializa rtt com alarme

/************************************************************************/
/* RTC                		                                            */
/************************************************************************/

void RTC_Handler(void) {
    uint32_t ul_status = rtc_get_status(RTC);
	printf("RTC_Handler\n");
	
    /* Second Counter */
    if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {	
		BaseType_t xHigherPriorityTaskWoken = pdTRUE;
		xSemaphoreGiveFromISR(xSemaphoreRTC2, &xHigherPriorityTaskWoken);
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
		printf("Second!\n");
    }
	
    /* Time or date alarm */
    if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
		BaseType_t xHigherPriorityTaskWoken = pdTRUE;
		xSemaphoreGiveFromISR(xSemaphoreRTCA, &xHigherPriorityTaskWoken);
		printf("Alarm!\n");

    }

	rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
    //rtc_clear_status(RTC, RTC_SCCR_SECCLR);
    rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
    rtc_clear_status(RTC, RTC_SCCR_CALCLR);
    rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type) {
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}

///* Configura RTC */                                                                            
// calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};                                            
// RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN);                                              
																								
// /* Leitura do valor atual do RTC */           
// uint32_t current_hour, current_min, current_sec;
// rtc_get_time(RTC, &current_hour, &current_min, &current_sec);

// /* configura alarme do RTC para daqui 20 segundos */                             
// rtc_set_time_alarm(RTC, 1, current_hour, 1, current_min, 1, current_sec + 20);

/************************************************************************/
/* TC                 		                                            */
/************************************************************************/
void TC1_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC0, 1);

	/** Muda o estado do LED (pisca) **/
	pin_toggle(LED1_PIO, LED1_PIO_IDX_MASK);  
}

void TC4_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC1, 1);

	/** Muda o estado do LED (pisca) **/
	pin_toggle(LED_PIO, LED_IDX_MASK);  
}

void TC7_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC2, 1);

	/** Muda o estado do LED (pisca) **/
	pin_toggle(LED3_PIO, LED3_PIO_IDX_MASK);  
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	
	/** ATIVA PMC PCK6 TIMER_CLOCK1  */
	if(ul_tcclks == 0 )
	    pmc_enable_pck(PMC_PCK_6);
	
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
  	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

// TC_init(TC0, ID_TC1, 1, 4);
// tc_start(TC0, 1);


/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreClick, &xHigherPriorityTaskWoken);
	printf("but_callback\n");
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();
	BUT_init();
	
	calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};
	uint32_t current_hour, current_min, current_sec;
	uint32_t current_year, current_month, current_day, current_week;
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN | RTC_IER_SECEN);


	/* Inicia o RTT primario */
	//Pisca cada 10 segundos
	RTT_init(10, 1, RTT_MR_ALMIEN);

	/* Pisca Tc em 10Hz */
	TC_init(TC0, ID_TC1, 1, 9);
	tc_start(TC0, 1);

	/* Pisca Tc em 2Hz*/
	TC_init(TC1, ID_TC4, 1, 2);
	tc_start(TC1, 1);

	char lista[10];
	for (;;)
	{	
		if(xSemaphoreTake(xSemaphore, 10)){
			//LED2 LIGADO COM ALARME RTT a cada 4segundos (0.25hz)
			pin_toggle(LED2_PIO, LED2_PIO_IDX_MASK);
			RTT_init(4, 1, RTT_MR_ALMIEN);
		}
		if(xSemaphoreTake(xSemaphoreClick, 10)){    
			//Click botão seta alarme RTC para daqui 20 segundos
			rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
			rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
    		rtc_set_date_alarm(RTC, 1, current_month, 1, current_day);                              
			rtc_set_time_alarm(RTC, 1, current_hour, 1, current_min, 1, current_sec + 20);
		}
		if(xSemaphoreTake(xSemaphoreRTC2, 10)){
			//Troca hora RTC a cada segundo.
			rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
			rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
			gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
			sprintf(lista,"%d:%d:%02d",current_hour, current_min, current_sec);
			gfx_mono_draw_string(lista, 10, 10, &sysfont);
		}
		if(xSemaphoreTake(xSemaphoreRTCA, 10)){
			//Pós ALARME 20 sec liga led3
			TC_init(TC2, ID_TC7, 1, 1);
			tc_start(TC2, 1);
		}

	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void pin_toggle(Pio *pio, uint32_t mask){
  if(pio_get_output_data_status(pio, mask))
    pio_clear(pio, mask);
  else
    pio_set(pio,mask);
}


static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void BUT_init(void) {

	/* conf botão como entrada */
	pio_configure(BUT_PLACA_PIO, PIO_INPUT, BUT_PLACA_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PLACA_PIO, BUT_PLACA_PIO_PIN_MASK, 60);
	
	pio_handler_set(BUT_PLACA_PIO,
					BUT_PLACA_PIO_ID,
					BUT_PLACA_PIO_PIN_MASK,
					PIO_IT_FALL_EDGE,
					but_callback);
					
	pio_enable_interrupt(BUT_PLACA_PIO, BUT_PLACA_PIO_PIN_MASK);
	pio_get_interrupt_status(BUT_PLACA_PIO);
	
	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PLACA_PIO_ID);
	NVIC_SetPriority(BUT_PLACA_PIO_ID, 4);

	/* Led Placa */
	pmc_enable_periph_clk(LED_PIO_ID);
 	pio_configure(LED_PIO, PIO_OUTPUT_1, LED_IDX_MASK, PIO_DEFAULT);

	pmc_enable_periph_clk(LED1_PIO_ID);
 	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_PIO_IDX_MASK, PIO_DEFAULT);
	pio_set(LED1_PIO, LED1_PIO_IDX_MASK);

	pmc_enable_periph_clk(LED2_PIO_ID);
 	pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_PIO_IDX_MASK, PIO_DEFAULT);
	pio_set(LED2_PIO, LED2_PIO_IDX_MASK);

	pmc_enable_periph_clk(LED3_PIO_ID);
 	pio_configure(LED3_PIO, PIO_OUTPUT_0, LED3_PIO_IDX_MASK, PIO_DEFAULT);
	pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
	
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}

	xSemaphore = xSemaphoreCreateBinary();
	if (xSemaphore == NULL){
		printf("falha em criar o semaforo \n");
	}

	xSemaphoreClick = xSemaphoreCreateBinary();
	if (xSemaphoreClick == NULL){
		printf("falha em criar o semaforo \n");
	}

	xSemaphoreRTC2 = xSemaphoreCreateBinary();
	if (xSemaphoreRTC2 == NULL){
		printf("falha em criar o semaforo \n");
	}

	xSemaphoreRTCA = xSemaphoreCreateBinary();
	if (xSemaphoreRTCA == NULL){
		printf("falha em criar o semaforo \n");
	}
	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){
		//entrar em sleep mode;
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
