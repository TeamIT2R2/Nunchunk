#include "Driver_I2C.h"                 // ::CMSIS Driver:I2C
#include "Board_GLCD.h"                 // ::Board Support:Graphic LCD
#include "GLCD_Config.h"                // Keil.MCB1700::Board Support:Graphic LCD
#include "stdio.h"
#include "LPC17xx.h"
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "Driver_USART.h"               // ::CMSIS Driver:USART

#define SLAVE_I2C_ADDR_WRITE       0x52			// Adresse esclave sur 7 bits
#define	SLAVE_I2C_ADDR_READ 			 0x52

//extern definition
extern ARM_DRIVER_I2C Driver_I2C0;
extern GLCD_FONT GLCD_Font_6x8;
extern GLCD_FONT GLCD_Font_16x24;
extern ARM_DRIVER_USART Driver_USART1;

//Thread declaration
void Thread_Nunchunk (void const *argument);                       // thread function
osThreadId tid_Thread_Nunchunk;                                    // thread id
osThreadDef (Thread_Nunchunk, osPriorityNormal, 1, 0);             // thread object

void Thread_Affichage (void const *argument);                       // thread function
osThreadId tid_Thread_Affichage;                                    // thread id
osThreadDef (Thread_Affichage, osPriorityNormal, 1, 0);             // thread object

void Thread_Envoie_BT (void const *argument);
osThreadId tid_Thread_Envoie_BT;                                    // thread id
osThreadDef (Thread_Envoie_BT, osPriorityNormal, 1, 0);             // thread object
// Mutex declaration;
osMutexId mid_Thread_Mutex;                                     // mutex id
osMutexDef (SampleMutex);                                       // mutex name definition

// function declarations;
void read6byte(unsigned char composant1,unsigned char *tab);
void sendzero(unsigned char composant,unsigned char registre);
void write2byte(unsigned char composant, unsigned char registre, unsigned char valeur);
void Init_I2C(void);
void NunChuck_phase1_init(void);
void NunChuck_phase2_read(void);
void NunChuck_translate_data(void);
void NunChuck_print_data(void);
void Init_UART(void);
void Fonction_BT(void);

// variable declarations;
unsigned char tab_read[8];
char etat=0;

/*
 * main: initialize and start the system
 */
int main (void) {
  osKernelInitialize ();                    // initialize CMSIS-RTOS

  // initialize peripherals here
	// (1) Initialisation du GLCD
	GLCD_Initialize();
  GLCD_ClearScreen();
  GLCD_SetFont(&GLCD_Font_16x24);
	
	 // (2) Init I2C0 device;
	Init_I2C(); 
	Init_UART();
	
	// create Mutex that start executing
	mid_Thread_Mutex = osMutexCreate (osMutex (SampleMutex));
	
  // create 'thread' functions that start executing,
	tid_Thread_Nunchunk = osThreadCreate (osThread(Thread_Nunchunk), NULL);
	tid_Thread_Affichage = osThreadCreate (osThread(Thread_Affichage), NULL);
	tid_Thread_Envoie_BT = osThreadCreate (osThread(Thread_Envoie_BT), NULL);
	
	// (3) NunChuck phase 1 
	NunChuck_phase1_init();
  osKernelStart ();                         // start thread execution 
	
	osDelay(osWaitForever);
	

}


void Thread_Nunchunk (void const *argument)                       // thread function Nunchunk
	{
		int i;
		
		while(1)
		{
		// (a) reset stuff
		for(i=0;i>=6;i++)
		{
			tab_read[i] = 0x00;
		}
		
	  // (b) NunChuck phase 2 
		NunChuck_phase2_read();
		NunChuck_translate_data();
   //	NunChuck_print_data();
			
		}
	}
	
	
void Thread_Affichage (void const *argument)                       // thread function affichage
	{
			while(1)
				{
					NunChuck_print_data();
				}	
	}
	
	
void Thread_Envoie_BT (void const *argument)                       // thread function affichage
	{
		
			while(1)
			{
				Fonction_BT();
			}	
	}	
	
	
void Init_I2C(void)
	{
		Driver_I2C0.Initialize(NULL);
		Driver_I2C0.PowerControl(ARM_POWER_FULL);
		Driver_I2C0.Control	(ARM_I2C_BUS_SPEED,				// 2nd argument = débit
											 ARM_I2C_BUS_SPEED_STANDARD  );	// 100 kHz
		Driver_I2C0.Control	(ARM_I2C_BUS_CLEAR,
							0 );
	}

void Init_UART(void)
	{
		Driver_USART1.Initialize(NULL);								// Debut d'initialisation
		Driver_USART1.PowerControl(ARM_POWER_FULL);		// Alimentation 
		Driver_USART1.Control(	ARM_USART_MODE_ASYNCHRONOUS |						
								ARM_USART_DATA_BITS_8		|
								ARM_USART_STOP_BITS_1		|
								ARM_USART_PARITY_NONE		|
								ARM_USART_FLOW_CONTROL_NONE,
								115200);														// Definition du mode de fonctionement
		Driver_USART1.Control(ARM_USART_CONTROL_TX,1);	//Validation de la reception
		Driver_USART1.Control(ARM_USART_CONTROL_RX,1);	// Validation de la transmition
	}	

void NunChuck_phase1_init(void)
	{
		//Configuration nunchuk
		write2byte(SLAVE_I2C_ADDR_WRITE, 0xF0, 0x55); //Config CTRL1
		osDelay(5);
		
		write2byte(SLAVE_I2C_ADDR_WRITE, 0xFB, 0x00); //Config CTRL2
		osDelay(5);


	}

void NunChuck_phase2_read(void)
	{
		sendzero(SLAVE_I2C_ADDR_WRITE,0x00);
		read6byte(SLAVE_I2C_ADDR_READ, tab_read);
	}	

	
	
void NunChuck_translate_data(void)
{
	char byte5;
	volatile int joy_x_axis;
	volatile int joy_y_axis;
	volatile int accel_x_axis;  
	volatile int accel_y_axis; 
	volatile int accel_z_axis; 
	volatile int z_button = 0;
	volatile int c_button = 0;
		
	byte5 = tab_read[5];
	joy_x_axis = tab_read[0];
	joy_y_axis = tab_read[1];
	accel_x_axis = (tab_read[2]<< 2);
	accel_y_axis = (tab_read[3]<< 2);
	accel_z_axis = (tab_read[4]<<2);
	z_button = 1;
  c_button = 1;
	
	 if ((byte5 >> 0) & 1) 
    z_button = 0;
  if ((byte5 >> 1) & 1)
    c_button = 0;
	
}	
	
	


void NunChuck_print_data(void)
{
	char affichage[20];
	
	sprintf(affichage,"joystick x :%03d",joy_x_axis);	// du blabla
	GLCD_DrawString(1 ,0,affichage);	// colonne puis ligne en pixel

	
	sprintf(affichage,"joystick y :%03d",joy_y_axis);	// du blabla
	GLCD_DrawString(1 ,30,affichage);	// colonne puis ligne en pixel

	
	sprintf(affichage,"accel x :%04d",accel_x_axis);	// du blabla
	GLCD_DrawString(1 ,60,affichage);	// colonne puis ligne en pixel

	
	sprintf(affichage,"accel y :%04d",accel_y_axis);	// du blabla
	GLCD_DrawString(1 ,90,affichage);	// colonne puis ligne en pixel

	
	sprintf(affichage,"accel z :%04d",accel_z_axis);	// du blabla
	GLCD_DrawString(1 ,120,affichage);	// colonne puis ligne en pixel
	
	sprintf(affichage,"z buton :%01d",z_button);	// du blabla
	GLCD_DrawString(1 ,150,affichage);	// colonne puis ligne en pixel
	
		sprintf(affichage,"c buton :%01d",c_button);	// du blabla
	GLCD_DrawString(1 ,180,affichage);	// colonne puis ligne en pixel

	
}
void write2byte(unsigned char composant, unsigned char registre, unsigned char valeur)
{
	unsigned char tab[2];
	tab[0] = registre;
	tab[1] = valeur;
	
	// Ecriture vers registre esclave : START + ADDR(W) + 1W_DATA + 1W_DATA + STOP
	Driver_I2C0.MasterTransmit (composant, tab, 2	, false);
	while (Driver_I2C0.GetStatus().busy == 1);	// attente fin transmission
}


void read6byte(unsigned char composant1,unsigned char *tab)
{
		// Lecture de data esclave : START + ADDR(R) + 6R_DATA + STOP
		Driver_I2C0.MasterReceive (composant1, tab_read, 6, false);		// false = avec stop
		while (Driver_I2C0.GetStatus().busy == 1);	// attente fin transmission
	
		osDelay(5);
}

void sendzero(unsigned char composant,unsigned char valeur)
	{
			unsigned char tab[1];
			tab[0] = valeur;
		
		// Ecriture vers registre esclave : START + ADDR(W) + 1W_DATA + RESTART
			Driver_I2C0.MasterTransmit (composant, tab, 1, false);		// false = avec stop
			while (Driver_I2C0.GetStatus().busy == 1);
	}	
	
		
void Fonction_BT(void)
{
		char affichage[20];
	switch(etat)
		{
			case 0:
					
					sprintf(affichage,"joystick x :%03d",joy_x_axis);	// du blabla
					GLCD_DrawString(1 ,210,affichage);	// colonne puis ligne en pixel
					if ( ((joy_y_axis <= 255) && (joy_y_axis>=200)) )	//joystick a fond en avant
						{
							etat=1;
						}
						
					if ( ((joy_y_axis <=200 ) && (joy_y_axis>=137)) ) //joystick en avant
						{
							etat=2;
						}
							
					if ( ((joy_y_axis <137 ) && (joy_y_axis>133)) && ((joy_x_axis<137)&&(joy_x_axis>133)) ) //joystick centré 
						{
							etat=3;
						}
						
					if ( ((joy_y_axis <= 132) && (joy_y_axis>=60)) ) // joystick en arrière
						{
							etat=4;
						}
					
					if ( ((joy_y_axis <= 60) && (joy_y_axis>=0)) ) // joystick a fond en arrière
						{
							etat=5;
						}
						
					if ( ((joy_x_axis <= 255) && (joy_x_axis>=200)) )	//joystick a fond à droite
						{
							etat=6;
						}
						
					if ( ((joy_x_axis <=200 ) && (joy_x_axis>=137)) ) //joystick à droite
						{
							etat=7;
						}
							
					if ( ((joy_x_axis <= 132) && (joy_x_axis>=60)) ) // joystick à gauche
						{
							etat=8;
						}
					
					if ( ((joy_x_axis <= 60) && (joy_x_axis>=0)) ) // joystick a fond à gauche
						{
							etat=9;
						}
						break;
					
			case 1:	
					 
					while(Driver_USART1.GetStatus().tx_busy == 1); // attente buffer TX vide
					Driver_USART1.Send("V1",2);  // Envoie de la carte vers l'hyperterminal sur L'uart
			
						if ( (joy_y_axis<200))
							{
								etat=0;
							}
						break;
			
			case 2:
					
					while(Driver_USART1.GetStatus().tx_busy == 1); // attente buffer TX vide
					Driver_USART1.Send("V2",2);  // Envoie de la carte vers l'hyperterminal sur L'uart
						
						if ( ((joy_y_axis>200) && ((joy_y_axis <137) )))
						 {
							 etat=0;
						 }
						 break;
					
			case 3:
					
					while(Driver_USART1.GetStatus().tx_busy == 1); // attente buffer TX vide
					Driver_USART1.Send("V3",2);  // Envoie de la carte vers l'hyperterminal sur L'uart
						
						if ( ((joy_y_axis<133) && (joy_y_axis >137))&& ((joy_x_axis>137)&&(joy_x_axis<133)))
						 {
							 etat=0;
						 }
						break;
				
			case 4:
					
					while(Driver_USART1.GetStatus().tx_busy == 1); // attente buffer TX vide
					Driver_USART1.Send("V4",2);  // Envoie de la carte vers l'hyperterminal sur L'uart
						
						if ( ((joy_y_axis > 132) && (joy_y_axis<60)))
						 {
							 etat=0;
						 }
						break;
					
			case 5:
					
					while(Driver_USART1.GetStatus().tx_busy == 1); // attente buffer TX vide
					Driver_USART1.Send("V5",2);  // Envoie de la carte vers l'hyperterminal sur L'uart
						
						if ( (joy_y_axis > 60))
						 {
							 etat=0;
						 }
					  break;
					
			case 6:
	
					while(Driver_USART1.GetStatus().tx_busy == 1); // attente buffer TX vide
					Driver_USART1.Send("V6",2);  // Envoie de la carte vers l'hyperterminal sur L'uart
		
						if ( (joy_x_axis <200))
						 {
							 etat=0;
						 }
						break;
					
			case 7:
					
					while(Driver_USART1.GetStatus().tx_busy == 1); // attente buffer TX vide
					Driver_USART1.Send("V7",2);  // Envoie de la carte vers l'hyperterminal sur L'uart
						
						if ( ((joy_x_axis >200 ) && (joy_x_axis<137)))
						 {
							 etat=0;
						 }
						break;
			
			case 8:
					
					while(Driver_USART1.GetStatus().tx_busy == 1); // attente buffer TX vide
					Driver_USART1.Send("V8",2);  // Envoie de la carte vers l'hyperterminal sur L'uart
						
						if ( (joy_x_axis > 132) && (joy_x_axis<60))
						 {
							 etat=0;
						 }
						break;
									
			case 9:
					
					while(Driver_USART1.GetStatus().tx_busy == 1); // attente buffer TX vide
					Driver_USART1.Send("V9",2);  // Envoie de la carte vers l'hyperterminal sur L'uart
						
						if ( (joy_x_axis > 60))
						 {
							 etat=0;
						 }
						break;
		}
}	
			