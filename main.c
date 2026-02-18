/******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  ******************************************************************************/

#include "stdio.h"
#include "main.h"
#include "string.h"
#include "math.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_nfctag.h"


/* I2C1 used to talk to both the OLED and the LED matrix. */
#include "stm32l4xx_hal_i2c.h"

#define double_click_threshold 350     // Max ms between clicks to count as a double-click
#define BUZZER_Pin GPIO_PIN_4          // Buzzer connected to PB4 (push-pull output)
#define BUZZER_GPIO_Port GPIOB

/* ---------------- Function prototypes (private) ---------------- */
static void UART1_Init(void);          // Configure UART1
static void MX_GPIO_Init(void);        // Configure button, LED, buzzer GPIO

static void MX_I2C1_Init(void);        // Configure I2C1 on PB8/PB9

/* ---------------- HT16K33 8x8 LED matrix ---------------- */
static void HT16K33_Init(uint8_t addr7);
static void HT16K33_SetBrightness(uint8_t level);   // Level 0..15
static void HT16K33_Clear(void);
static void HT16K33_ShowBitmap(const uint8_t rows[8]);

/* ======== OLED (SSD1308) DRIVER ======== */
#define OLED_ADDR        0x3C          // 7-bit I2C address of the OLED module
#define OLED_W           128           // Width in pixels
#define OLED_H            64           // Height in pixels
#define OLED_CMD         0x00          // Control byte for "command"
#define OLED_DATA        0x40          // Control byte for "data"
static void OLED_SendCmd(uint8_t c);                 // Send 1 command byte
static void OLED_SendCmd2(uint8_t c1, uint8_t c2);   // Send command with one data byte
static void OLED_SendData(uint8_t *d, uint16_t n);   // Send raw pixel bytes
static void OLED_Init(void);                         // Run the SSD1308 init sequence
static void OLED_Clear(void);                        // Fill screen with zeros (black)
static void OLED_SetCursor(uint8_t col, uint8_t page); // Set write position (x = col, y = 8px page)
static void OLED_Print(const char *s);               // Print a string at current cursor
static void OLED_PrintAt(uint8_t col, uint8_t page, const char *s); // Move + print

static const uint8_t font5x7[96][5] = {
{0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},{0x14,0x7F,0x14,0x7F,0x14},{0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},{0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},{0x00,0x1C,0x22,0x41,0x00},{0x00,0x41,0x22,0x1C,0x00},{0x14,0x08,0x3E,0x08,0x14},{0x08,0x08,0x3E,0x08,0x08},{0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},{0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},{0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},{0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},{0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},{0x32,0x49,0x79,0x41,0x3E},{0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},{0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},{0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},{0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},{0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},{0x7F,0x20,0x18,0x20,0x7F},{0x63,0x14,0x08,0x14,0x63},{0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},{0x00,0x7F,0x41,0x41,0x00},{0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7F,0x00},{0x04,0x02,0x01,0x02,0x04},{0x40,0x40,0x40,0x40,0x40},{0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},{0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},{0x38,0x44,0x44,0x48,0x7F},{0x38,0x54,0x54,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02},{0x0C,0x52,0x52,0x52,0x3E},{0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},{0x7F,0x10,0x28,0x44,0x00},{0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},{0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},{0x7C,0x14,0x14,0x14,0x08},{0x08,0x14,0x14,0x18,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},{0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},{0x1C,0x20,0x40,0x20,0x1C},{0x3C,0x40,0x30,0x40,0x3C},{0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},{0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},{0x00,0x00,0x7F,0x00,0x00},{0x00,0x41,0x36,0x08,0x00},{0x10,0x08,0x08,0x10,0x08},{0x00,0x00,0x00,0x00,0x00}
};

static void OLED_ClearLine(uint8_t page)
{
    OLED_SetCursor(0, page);
    // Write 128 zero pixels for this page (8 × 16-byte chunks)
    uint8_t zeros[16] = {0};
    for (int i = 0; i < 8; i++) {
    	OLED_SendData(zeros, sizeof(zeros));
    }
}

/* ---------------- Global peripheral handles ---------------- */
UART_HandleTypeDef huart1;   // For UART1 transmit/receive

I2C_HandleTypeDef hi2c1;     // For I2C1 (OLED + HT16K33)

/* 7-bit I2C address */
static uint8_t ht16_addr = 0x70;

static const uint8_t ICON_OK[8] = {
    0b00000000,
    0b00000001,
    0b00000011,
    0b00000110,
    0b10001100,
    0b11011000,
    0b01110000,
    0b00100000
};
static const uint8_t ICON_X[8] = {
    0b10000001,
    0b01000010,
    0b00100100,
    0b00011000,
    0b00011000,
    0b00100100,
    0b01000010,
    0b10000001

};

static void uart_send(const char *message_print)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF); 	//Sending in normal mode
}

/* -------------- Game state-------------- */
volatile uint8_t game_switched = 0; // Game 1 = 0, Game 2 = 1
volatile uint8_t click_count = 0;         // global press counter
volatile uint32_t first_click_time = 0;   //  record first click time

//------------------Game 1---------------------------------//
/* Two phases: move (Green) vs don't-move (Red) */
typedef enum { Green_Phase = 0, Red_Phase = 1} phase_type;

/* Game 1 variables */
typedef struct{
	phase_type current_phase; 		// (Game Phase: Green or Red)
	uint32_t phase_start_time; 		// (Timestamp of when the phase started)
	uint32_t phase_last_message; 	// (Timestamp of last message sent)
	uint32_t phase_last_blink; 		// (Timestamp of last LED blink)
	uint8_t green_annouce; 			// (Flag: Was Green Light announced?)
	uint8_t red_annouce; 			// (Flag: Was Red Light announced?)
	uint8_t game_over; 				// (Flag: Is the game over?)
} game_1;


game_1 game_state = { Green_Phase, 0, 0, 0, 0, 0, 0 };  // Initialize Game 1 state

//------------------Game 2---------------------------------//

typedef struct{
	uint32_t phase_start_time; 		// (Timestamp of when the phase started)
	uint32_t phase_last_message; 		// (Timestamp of last message sent)
	uint32_t phase_last_blink; 		// (Timestamp of last LED blink)
	uint8_t game_over; 			// (Flag: Is the game over?)
} game_2;


/* ------------------Switching between games-------------- */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin != BUTTON_EXTI13_Pin)
		return ;

	uint32_t now = HAL_GetTick();

	if(click_count == 0){
		click_count = 1;
		first_click_time = now;
		return;
	}

	if(now - first_click_time <= double_click_threshold) {
		game_switched = !game_switched;
		uart_send("Game Switched!\r\n");
	}
	click_count = 0; // reset click sequence either way
}

/* ---------------- NFC gate ---------------- */
static uint8_t  nfc_authorized    = 0;
static uint32_t nfc_last_poll_ms  = 0;

static void NFC_Init(void)
{
    if (BSP_NFCTAG_Init(0) != NFCTAG_OK) {
        uart_send("NFC init failed\r\n");
    } else {
        uart_send("NFC ready. Present card to start.\r\n");
    }
}

static int NFC_GateAndSelectGame(void)
{
    if (nfc_authorized)
        return 1;

    ST25DV_FIELD_STATUS rf;
    uint32_t current_time = HAL_GetTick();

    if (BSP_NFCTAG_GetRFField_Dyn(0, &rf) == NFCTAG_OK && rf == ST25DV_FIELD_ON)
    {
        nfc_authorized = 1;
        game_switched = 0;  // Ensure we start in Game 1
        uart_send("NFC card detected! Starting Game 1...\r\n");
        uart_send("Entering Red Light, Green Light Game as Player\r\n");
        game_state.phase_start_time   = current_time;
        game_state.phase_last_message = current_time;
        game_state.phase_last_blink   = current_time;
        OLED_Clear();
        OLED_PrintAt(0,0,"Game 1: RLGL");
        OLED_PrintAt(0,2,"Phase: GREEN");
        return 1;
    }
    else
    {
        if (current_time - nfc_last_poll_ms > 5000U)
        {
            uart_send("Please tap any NFC card to start the game.\r\n");
            nfc_last_poll_ms = current_time;
        }
    }
    return 0;
}

/* ========================= MAIN ========================= */
int main(void)
{
	// Reset of all peripherals, Initializes the Flash interface and the Systick.
	HAL_Init();

	// UART initialization
	UART1_Init();

	// NFC
	NFC_Init();

	// Peripheral initializations using BSP functions
	BSP_TSENSOR_Init();
	BSP_HSENSOR_Init();
	BSP_PSENSOR_Init();
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
	BSP_MAGNETO_Init();

	// GPIO initialization
	MX_GPIO_Init();

    /* I2C + LED matrix init (OLED and HT16K33) */
    MX_I2C1_Init();
    HT16K33_Init(0x70);
    HT16K33_SetBrightness(8);       // Medium brightness
    HT16K33_ShowBitmap(ICON_OK);

    OLED_Init();
    OLED_PrintAt(0,0,"Pay $$ First");

	game_2 game2_state = {0, 0, 0, 0};
	uint8_t prev_game_switched = game_switched; // Track changes to switch screens cleanly

	while (1)
	{
		uint32_t current_time = HAL_GetTick();

		if (!NFC_GateAndSelectGame()) {
		    continue;   // wait in place
		}

		if(game_switched != prev_game_switched){
			prev_game_switched = game_switched;

			//Switch Back to Game 1
			if(game_switched == 0){
				uart_send("Entering Red Light, Green Light Game as Player\r\n");
				/* Reset Game 1 state */
				game_state.current_phase      = Green_Phase;
				game_state.phase_start_time   = current_time;
				game_state.phase_last_message = current_time;
				game_state.phase_last_blink   = current_time;
				game_state.green_annouce      = 0;
				game_state.red_annouce        = 0;
				game_state.game_over          = 0;
				HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET); // buzzer OFF

                HT16K33_ShowBitmap(ICON_OK);
                OLED_Clear();
                OLED_PrintAt(0,0,"Game 1: RLGL");
                OLED_PrintAt(0,2,"Phase: GREEN");
			}
			else{
				/* Enter Game 2 (Catch & Run) */
				uart_send("Entering Catch & Run as Player\r\n");
				game2_state.phase_start_time = HAL_GetTick();
				HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
				HT16K33_Clear();
                OLED_Clear();
                OLED_PrintAt(0,0,"Game 2: C&R");
                OLED_PrintAt(0,2,"Find/escape!");

			}
		}

		//--------------------------- Game 2 loop ---------------------------
		if (game_switched != 0){
		    /* Update once per second: read sensors, update UI, handle escape window */
		    if (current_time - game2_state.phase_last_message > 1000){
		        game2_state.phase_last_message = current_time;

		        /* Read magnetometer (enforcer proximity proxy). Convert int16 → float. */
		        float magneto_data[3];
		        int16_t magneto_data_i16[3] = {0};
		        BSP_MAGNETO_GetXYZ(magneto_data_i16); //read magneto
		        magneto_data[0] = (float)magneto_data_i16[0];
		        magneto_data[1] = (float)magneto_data_i16[1];
		        magneto_data[2] = (float)magneto_data_i16[2];

		        //Sensor Monitoring: temperature/pressure/humidity thresholds
		        float temp_data = BSP_TSENSOR_ReadTemp();
		        float p_data = BSP_PSENSOR_ReadPressure();
		        float h_data = BSP_HSENSOR_ReadHumidity();

		        float temp_thr = 30.5f;
		        float h_thr   = 90.0f;
		        float p_thr   = 2000.0f;

		        if(temp_data > temp_thr ){
		        	char env_data[128];
		        	sprintf(env_data, "\rHigh Temperature detected! Temp = %.2f deg C Dangerous environment!\r\n ", temp_data);
		        	uart_send(env_data);
                    OLED_PrintAt(0,4,"High Temp!        ");
		        }
		        else{
		        	 OLED_ClearLine(4);
		        }
		        if(p_data > p_thr){
		        	char env_data[128];
		        	sprintf(env_data, "\rHigh Pressure detected! Pressure = %.2f hPa Dangerous environment!\r\n ", p_data);
		        	uart_send(env_data);
                    OLED_PrintAt(0,5,"High Pressure!    ");
		        }
		        else{
		        	OLED_ClearLine(5);
		        }
		        if(h_data > h_thr){
		        	char env_data[128];
		        	sprintf(env_data, "\rHigh humidity detected! Humidity = %.2f %% Dangerous environment!\r\n ", h_data);
		        	uart_send(env_data);
                    OLED_PrintAt(0,6,"High Humidity!    ");
		        }
		        else{
		        	 OLED_ClearLine(6);
		        }


		        if(temp_data > temp_thr || p_data > p_thr || h_data > h_thr){
		        	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);   // buzzer ON
		        } else {
		        	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET); // buzzer OFF
		        }

		        char proximity_data[128];
		        sprintf(proximity_data, "\rMag X = %.2f Mag Y = %.2f Mag Z = %.2f \r\n",magneto_data[0], magneto_data[1], magneto_data[2]);
		        uart_send(proximity_data);

		        float mag_threshold_1 = 4000.0f;
		        float mag_threshold_2 = 8000.0f;
		        float mag_threshold_3 = 12000.0f;

		        float mag_magnitude = fabs(magneto_data[0]) + fabs(magneto_data[1]) + fabs(magneto_data[2]);

		         /* If enforcer is close, player has 3 seconds to press the button once. */
		        static uint32_t detect_time = 0;    // when enforcer first detected
		        static uint8_t  escape_active = 0;  // 1 while the 3s window is active
		        static uint32_t escaped_msg_time = 0;   // when “Escaped!” was shown

		        if (mag_magnitude > mag_threshold_3) {
		            uart_send("Enforcer very near!\r\n");
		            /* Faster blink when very near */
		            if (current_time - game2_state.phase_last_blink > 500) {
		            	HAL_GPIO_TogglePin(GPIOB, LED2_Pin);
		            	game2_state.phase_last_blink = current_time;
		            }
                    OLED_PrintAt(0,3,"Enforcer: VERY NEAR");
		            if (!escape_active){
		            	detect_time = current_time;
		            	escape_active = 1;
		            	uart_send("Press button within 3 seconds to escape!\r\n");
		            }
		        }
		        else if (mag_magnitude > mag_threshold_2) {
		        	uart_send("Enforcer near.\r\n");
		        	if (current_time - game2_state.phase_last_blink > 1000) {
		          		HAL_GPIO_TogglePin(GPIOB, LED2_Pin);
		            	game2_state.phase_last_blink = current_time;
		            }
                    OLED_PrintAt(0,3,"Enforcer: NEAR     ");
		        	if (!escape_active){
		          		detect_time = current_time;
		          		escape_active = 1;
		          		uart_send("Press button within 3 seconds to escape!\r\n");
		          	}
		        }
		        else if (mag_magnitude > mag_threshold_1) {
		        	uart_send("Enforcer approaching.\r\n");
		        	if (current_time - game2_state.phase_last_blink > 2000) {
		        		HAL_GPIO_TogglePin(GPIOB, LED2_Pin);
		        		game2_state.phase_last_blink = current_time;
		        	}
                    OLED_PrintAt(0,3,"Enforcer: APPROACH ");
		        	if (!escape_active){
		        		detect_time = current_time;
		        		escape_active = 1;
		        		uart_send("Press button within 3 seconds to escape!\r\n");
		        	}
		        }
		        else{
		        	/* Far: cancel escape mode, show FAR on OLED */
		        	escape_active = 0;
                    OLED_PrintAt(0,3,"Enforcer: FAR      ");
		        }

		        /* If user pressed button (single click) during escape window, success message */
		        if (escape_active == 1 && click_count == 1){
		        	uart_send("You escaped! Good Job!\r\n");
                    OLED_PrintAt(0,7,"Escaped!           ");
                    escaped_msg_time = current_time;
		        	escape_active = 0;
		        	click_count = 0;
		        }
		        /* Clear "Escaped!" message after 1s  */
		        if (escaped_msg_time && (current_time - escaped_msg_time > 1000)){
		        	OLED_ClearLine(7);
		        	escaped_msg_time = 0;
		        	}
		        /* If 3s passes without a click → game over */
		        if (escape_active == 1 && (current_time - detect_time > 3000)){
		        	uart_send("Game Over!! Good Job!! :)\r\n");
		        	HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
		        	game2_state.game_over = 1;
                    OLED_PrintAt(0,6,"Game Over          ");
		        	return 0;
		        }
		   }
		    continue;
		}

		//--------------------------- Game 1 loop ---------------------------
		if(game_state.game_over){
			HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
            OLED_PrintAt(0,6,"Game Over          ");
			continue;
		}

		if (game_state.current_phase == Green_Phase){
			if (!game_state.green_annouce){
				uart_send("Green Light!\r\n");
			    game_state.green_annouce = 1;
			    game_state.red_annouce = 0;
			    HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET);  // LED ON in GREEN
                HT16K33_ShowBitmap(ICON_OK); // show OK in Green phase
                OLED_PrintAt(0,2,"Phase: GREEN   ");
			}
			/* Every 2s print environmental readings to UART + OLED */
			if (current_time - game_state.phase_last_message > 2000){
				game_state.phase_last_message = current_time;
				float temp_data = BSP_TSENSOR_ReadTemp(); // read temperature sensor
				float p_data = BSP_PSENSOR_ReadPressure(); // read pressure sensor
				float h_data = BSP_HSENSOR_ReadHumidity(); // read humidity sensor
				char current_data[128];
				sprintf(current_data, "\rTemp = %.2f deg C Pressure = %.2f hPa Humidity = %.2f %%\r\n ", temp_data, p_data, h_data);
				uart_send(current_data);
                char line[22];
                snprintf(line,sizeof(line),"T%.2f P%.2f H%.2f", temp_data, p_data, h_data);
                OLED_PrintAt(0,4,"                ");
                OLED_PrintAt(0,4,line);
			}
			/* After 10s, switch to Red phase */
			if (current_time - game_state.phase_start_time > 10000){
				game_state.current_phase = Red_Phase;
				game_state.phase_start_time = current_time; //Reset phase start time
				game_state.phase_last_message = current_time;
				game_state.phase_last_blink = current_time;
				game_state.green_annouce = 0;
				game_state.red_annouce = 0;
				uart_send("\rSwitch to Red Light Phase \r\n");
			}
		}

		//Red Phase
		else {
			if(!game_state.red_annouce){
				uart_send("Red Light!\r\n");
				game_state.red_annouce = 1;
				game_state.green_annouce = 0;
                HT16K33_ShowBitmap(ICON_X);   // show X in Red phase
                OLED_PrintAt(0,2,"Phase: RED     ");
                OLED_ClearLine(4);
                OLED_ClearLine(5);
                OLED_ClearLine(6);

			}
			/* Every 2s check player movement using accel + gyro */
			if (current_time - game_state.phase_last_message > 2000) {
				game_state.phase_last_message = current_time;
				float accel_data[3];
				int16_t accel_data_i16[3] = {0};
				BSP_ACCELERO_AccGetXYZ(accel_data_i16);		// read accelerometer
				/* Convert raw milli-g values to m/s^2 */
				accel_data[0] = (float)accel_data_i16[0] * (9.8/1000.0f);
				accel_data[1] = (float)accel_data_i16[1] * (9.8/1000.0f);
				accel_data[2] = (float)accel_data_i16[2] * (9.8/1000.0f);

				float gyro_data[3] = {0};
				BSP_GYRO_GetXYZ(gyro_data);		// read gyro meter

				/* If too large, player moved → game over */
				float acc_threshold = 25000.0f;
				float gyro_threshold = 25000.0f;
				float acc_magnitude = fabs(accel_data[0]) + fabs(accel_data[1]) + fabs(accel_data[2]);
				float gyro_magnitude = fabs(gyro_data[0]) + fabs(gyro_data[1]) + fabs(gyro_data[2]);

				if (acc_magnitude > acc_threshold || gyro_magnitude > gyro_threshold) {
				    uart_send("Game Over\r\n");
				    HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
				    game_state.game_over = 1;
                    OLED_PrintAt(0,6,"Game Over          ");
				    return 0;
				}

				char move_data[128];
				sprintf(move_data, "\rAcc X = %.2f Acc Y = %.2f Acc Z = %.2f Gyro X = %.2f Gyro Y = %.2f Gyro Z = %.2f \r\n ",
                        accel_data[0], accel_data[1], accel_data[2], gyro_data[0],gyro_data[1], gyro_data[2]);
				uart_send(move_data);

				/* Blink LED every 500 ms during Red  */
				if (current_time - game_state.phase_last_blink > 500){
					HAL_GPIO_TogglePin (GPIOB, LED2_Pin);
					game_state.phase_last_blink = current_time;
				}
			    /* After 10s, back to Green */
			    if (current_time - game_state.phase_start_time > 10000){
					game_state.current_phase = Green_Phase;
					game_state.phase_start_time = current_time; //Reset phase start time
					game_state.phase_last_message = current_time;
					game_state.phase_last_blink = current_time;
					game_state.red_annouce = 0;
					game_state.green_annouce = 0;
					uart_send("Switch to Green Light Phase \r\n");
                    OLED_PrintAt(0,2,"Phase: GREEN   ");
				}
			}
		}
	}
}

/* ================= UART ==================
   Configure UART1 on PB6 (TX) / PB7 (RX), AF7, 115200-8-N-1. */
static void UART1_Init(void)
{
    /* Pin configuration for UART. BSP_COM_Init() */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Configuring UART1 core parameters */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
      while(1);
    }
}

/* ================= GPIO ==================
   Configure button PC13 (interrupt), LED2 (output), buzzer PB4 (output). */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOC_CLK_ENABLE();	// Enable AHB2 Bus for GPIOC (button)
	__HAL_RCC_GPIOB_CLK_ENABLE();   // Enable for GPIOB (LED, buzzer)

	/* Start with LED2 off */
	HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);

	// BUTTON_EXTI13_Pin (PC13) as external interrupt on falling edge (button press)
	GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* LED2 (PBx) as push-pull output, no pull, low speed is fine */
	GPIO_InitStruct.Pin = LED2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// Enable NVIC EXTI line 13 interrupt handling
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	// Buzzer on PB4 as push-pull output
	GPIO_InitStruct.Pin = BUZZER_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);
}

/* =============== I2C1 INIT (PB8/PB9) ===============
   Configure I2C1 pins and timing so we can talk to OLED/HT16K33. */
static void MX_I2C1_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;   // PB8=SCL, PB9=SDA
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;           // I2C requires open-drain
    GPIO_InitStruct.Pull      = GPIO_PULLUP;               // Pull-ups needed on SCL/SDA
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;             // Alt-function mapping for I2C1
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Core I2C settings; Timing picked for ~100kHz on STM32L4. */
    hi2c1.Instance             = I2C1;
    hi2c1.Init.Timing          = 0x00707CBB;               // ~100kHz typical on L4
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.OwnAddress2Masks= I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) while(1);          // Stop if I2C init fails

    /* Filters help signal integrity; analog ON, digital filter 0 (off). */
    HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);
}

/* =========== HT16K33 DRIVER =========== */
/* Send a single-byte command to HT16K33 (address is set in ht16_addr). */
static void HT16K33_WriteCmd(uint8_t cmd)
{
    HAL_I2C_Master_Transmit(&hi2c1, (ht16_addr << 1), &cmd, 1, 100);
}

/* Basic init from datasheet:
   - turn on oscillator
   - display on (no blink)
   - set brightness
   - clear the matrix */
static void HT16K33_Init(uint8_t addr7)
{
    ht16_addr = addr7;
    HT16K33_WriteCmd(0x21);          // oscillator on
    HT16K33_WriteCmd(0x81);          // display on, blink off
    HT16K33_SetBrightness(8);
    HT16K33_Clear();
}

/* Brightness 0..15. Value is ORed into 0xE0 per datasheet. */
static void HT16K33_SetBrightness(uint8_t level)
{
    if (level > 15) level = 15;
    HT16K33_WriteCmd(0xE0 | level);  // 0xE0..0xEF
}

/* Write 8 rows to the display RAM.
   HT16K33 expects pairs of bytes per row; we send row byte + 0 padding. */
static void HT16K33_ShowBitmap(const uint8_t rows[8])
{
    uint8_t buf[17];
    buf[0] = 0x00; // RAM address pointer
    for (int r = 0; r < 8; r++) {
        buf[1 + 2*r]     = rows[r];
        buf[1 + 2*r + 1] = 0x00;
    }
    HAL_I2C_Master_Transmit(&hi2c1, (ht16_addr << 1), buf, sizeof(buf), 100);
}

/* Fill display with zeros (all LEDs off) */
static void HT16K33_Clear(void)
{
    uint8_t blank[8] = {0,0,0,0,0,0,0,0};
    HT16K33_ShowBitmap(blank);
}

/* =========== OLED low-level helpers =========== */
/* Send one command byte (control prefix 0x00 + command) */
static void OLED_SendCmd(uint8_t c){
    uint8_t buf[2] = {OLED_CMD, c};
    HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR<<1, buf, 2, 100);
}
/* Send a command that needs one data byte (two-byte sequence) */
static void OLED_SendCmd2(uint8_t c1, uint8_t c2){
    uint8_t buf[3] = {OLED_CMD, c1, c2};
    HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR<<1, buf, 3, 100);
}
/* Send raw pixel data. We prefix each I2C burst with 0x40 (data mode).
   To keep transfers small, send at most 16 bytes per burst. */
static void OLED_SendData(uint8_t *d, uint16_t n){
    uint8_t buf[17];
    buf[0] = OLED_DATA;
    while(n){
        uint8_t chunk = (n>16)?16:n;
        for(uint8_t i=0;i<chunk;i++) buf[1+i]=*d++;
        HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR<<1, buf, chunk+1, 100);
        n -= chunk;
    }
}
/* Move the "cursor" to (col,page) in page addressing mode.
   page is 0..7 (each page = 8 pixel tall band). col is 0..127. */
static void OLED_SetCursor(uint8_t col, uint8_t page){
    OLED_SendCmd(0xB0 | (page & 0x07));              // Set page address (row band)
    OLED_SendCmd(0x00 | (col & 0x0F));               // Set lower column nibble
    OLED_SendCmd(0x10 | ((col>>4) & 0x0F));          // Set upper column nibble
}
/* Minimal, standard init sequence for SSD1308 */
static void OLED_Init(void){
    OLED_SendCmd(0xAE);  // Display OFF
    OLED_SendCmd2(0xD5,0x80); // Set display clock divide ratio/oscillator frequency
                              //A[7:4] = 1000b, A[3:0]=0000b
    OLED_SendCmd2(0xA8,0x3F); // Set multiplex ratio = 63 --> 64 rows total (128x64 display)
    OLED_SendCmd2(0xD3,0x00); // Set display offset = 0 (no vertical shift)
    OLED_SendCmd(0x40); // Set display start line = 0 (top line)
    OLED_SendCmd(0x8D); OLED_SendCmd(0x14); // Charge pump enabled
    OLED_SendCmd(0x20); OLED_SendCmd(0x02); // Memory addressing mode: Set to page mode
    OLED_SendCmd(0xA1); // Flip screen horizontally (so text appears correctly from left to right)
    OLED_SendCmd(0xC8); //Flip screen vertically (so the first line is at the top)
    OLED_SendCmd2(0xDA,0x12); // Set up how the rows and columns are connected inside the display
    OLED_SendCmd2(0x81,0x7F); // Set the screen brightness
    OLED_SendCmd2(0xD9,0xF1); // Control how long the pixels are pre-charged before turning on
    OLED_SendCmd2(0xDB,0x40); // Set the voltage level used for the screen’s “off” pixels
    OLED_SendCmd(0xA4);  // Display what’s in memory (don’t light up all pixels)
    OLED_SendCmd(0xA6);  // Normal display mode (white pixels = ON, black = OFF)
    OLED_Clear(); //Clear display
    OLED_SendCmd(0xAF); //Display ON
}
// Clear display
static void OLED_Clear(void){
	// Screen is divided into 8 pages (each page = 8 pixels tall)
    for(uint8_t p=0; p<8; p++){
        OLED_SetCursor(0,p); //move to col 0 of current page)
        for(uint8_t i=0;i<128;i++){
            uint8_t z=0x00; // 0x00 = all pixels OFF for this column
            OLED_SendData(&z,1); // Send one column of blank pixels
        }
    }
}
/* Draw a null-terminated string using 5x7 font, with 1-pixel spacing column. */
static void OLED_Print(const char *s){
    while(*s){
        char c = *s++;
        if(c < 32 || c > 127){ c = '?'; }            // Replace non-printable chars
        const uint8_t *g = font5x7[c-32];            // Lookup glyph columns
        uint8_t col6[6];
        for(int i=0;i<5;i++) col6[i]=g[i];           // Copy 5 columns
        col6[5]=0x00;                                 // 1 column spacing
        OLED_SendData(col6,6);                        // Send 6 columns to OLED
    }
}
static void OLED_PrintAt(uint8_t col, uint8_t page, const char *s){
    OLED_SetCursor(col,page);
    OLED_Print(s);
}
