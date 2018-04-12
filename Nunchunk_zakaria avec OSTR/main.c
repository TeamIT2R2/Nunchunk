#include "Driver_I2C.h"                 // ::CMSIS Driver:I2C
#include "Board_GLCD.h"                 // ::Board Support:Graphic LCD
#include "GLCD_Config.h"                // Keil.MCB1700::Board Support:Graphic LCD
#include "stdio.h"
#include "LPC17xx.h"
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "Driver_USART.h"               // ::CMSIS Driver:USART



#define SLAVE_I2C_ADDR_WRITE       0x52			// Adresse esclave sur 7 bits
#define	SLAVE_I2C_ADDR_READ 			 0x52
#define MAILQUEUE_OBJECTS      16                               // number of Message Queue Objects

typedef struct {                                                // Déclaration de la structure
  volatile int Buf[32];
} Info_Nunchunk;


//extern definition
extern ARM_DRIVER_I2C Driver_I2C0;
extern GLCD_FONT GLCD_Font_6x8;
extern GLCD_FONT GLCD_Font_16x24;
extern ARM_DRIVER_USART Driver_USART1;

//Thread declaration
void Thread_Nunchunk (void const *argument);                       // thread function
osThreadId tid_Thread_Nunchunk;                                    // thread id
osThreadDef (Thread_Nunchunk, osPriorityNormal, 1, 0);             // thread object

void Thread_Envoie_BT (void const *argument);
osThreadId tid_Thread_Envoie_BT;                                    // thread id
osThreadDef (Thread_Envoie_BT, osPriorityNormal, 1, 0);             // thread object

//MailBox declaration
osMailQId qid_MailQueue;                                        // mail queue id
osMailQDef (MailQueue, MAILQUEUE_OBJECTS, Info_Nunchunk);     // mail queue object

// Mutex declaration;
osMutexId mid_Thread_Mutex;                                     // mutex id
osMutexDef (SampleMutex);                                       // mutex name definition


// function declarations;
void read6byte(unsigned char composant1,unsigned char *tab);
void sendzero(unsigned char composant,unsigned char registre);
void write2byte(unsigned char composant, unsigned char registre, unsigned char valeur);
void Init_I2C(void);
void NunChuck_phase1_init(void);
void NunChuck_translate_data(void);
void Init_UART(void);

// variable declarations;
unsigned char tab_read[8];

/*
 * main: initialize and start the system
 */
int main (void) {
	char affichage[20];
	
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
	
	// create MailBox that start executing
	qid_MailQueue = osMailCreate (osMailQ(MailQueue), NULL); 
	
  // create 'thread' functions that start executing,
	tid_Thread_Nunchunk = osThreadCreate (osThread(Thread_Nunchunk), NULL);
	tid_Thread_Envoie_BT = osThreadCreate (osThread(Thread_Envoie_BT), NULL);
	
	// (3) NunChuck phase 1 
	NunChuck_phase1_init();
	
	sprintf(affichage,"joystick x : ");	// affichage du texte qui ne changera jamais
	GLCD_DrawString(1 ,0,affichage);	// cmd d'affichage

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
				
			// (b) NunChuck Lecture/Rangement 
			sendzero(SLAVE_I2C_ADDR_WRITE,0x00);					// Sequence obligatoire pour la reception
			read6byte(SLAVE_I2C_ADDR_READ, tab_read);			// Lecture de l'état du nunchunk
				
			NunChuck_translate_data();										// Fonction de trie des variables / envoie par MailBox
			}
	}
	
	

	
void Thread_Envoie_BT (void const *argument)                       // thread function d'envoie BT
	{
		static char etat=0;
		char affichage[20];
		Info_Nunchunk	*pMail = 0;
		osEvent           evt;
		
	while(1)
		{
			evt = osMailGet (qid_MailQueue, osWaitForever);             // wait for mail
			
			if (evt.status == osEventMail) {
				pMail = evt.value.p;																			// Reception des valeurs
					if (pMail) {
								sprintf(affichage,"%03d",pMail->Buf[0]);	// du blabla							//NB : On affichera qu'une valeur car l'affichage prend 23ms et monopolise les tâches
								GLCD_DrawString(200 ,0,affichage);	// cmd d'affichage							// 		 On supposera donc en affichant 1 valeur que les autres sont pris en compte (validé)
				
									
							if ( ((pMail->Buf[1] <= 255) && (pMail->Buf[1]>=200)) )		//joystick a fond en avant
										{
												while(Driver_USART1.GetStatus().tx_busy == 1); // attente buffer TX vide
												Driver_USART1.Send("V8",2);  // Envoie de la carte vers l'hyperterminal sur L'uart
										}
														
							if ( ((pMail->Buf[1] <=200 ) && (pMail->Buf[1]>=130)) ) //joystick en avant
										{
												while(Driver_USART1.GetStatus().tx_busy == 1); // attente buffer TX vide
												Driver_USART1.Send("V1",2);  // Envoie de la carte vers l'hyperterminal sur L'uart
										}
																
							if ( ((pMail->Buf[1] <130 ) && (pMail->Buf[1]>122)) && ((pMail->Buf[0]<135)&&(pMail->Buf[0]>127)) ) //joystick centré 
										{
												while(Driver_USART1.GetStatus().tx_busy == 1); // attente buffer TX vide
												Driver_USART1.Send("V0",2);  // Envoie de la carte vers l'hyperterminal sur L'uart
										}
								
							if ( ((pMail->Buf[1] <= 121) && (pMail->Buf[1]>=70)) ) // joystick en arrière
										{
												while(Driver_USART1.GetStatus().tx_busy == 1); // attente buffer TX vide
												Driver_USART1.Send("V2",2);  // Envoie de la carte vers l'hyperterminal sur L'uart
										}
													
							if ( ((pMail->Buf[1] < 70) && (pMail->Buf[1]>=0)) ) // joystick a fond en arrière
										{
												while(Driver_USART1.GetStatus().tx_busy == 1); // attente buffer TX vide
												Driver_USART1.Send("V9",2);  // Envoie de la carte vers l'hyperterminal sur L'uart
										}
														
							if ( ((pMail->Buf[0] <= 255) && (pMail->Buf[0]>=180)) )	//joystick a fond à droite
										{
												while(Driver_USART1.GetStatus().tx_busy == 1); // attente buffer TX vide
												Driver_USART1.Send("V5",2);  // Envoie de la carte vers l'hyperterminal sur L'uart
										}
								
							if ( ((pMail->Buf[0] <180 ) && (pMail->Buf[0]>=135)) ) //joystick à droite
										{
												while(Driver_USART1.GetStatus().tx_busy == 1); // attente buffer TX vide
												Driver_USART1.Send("V4",2);  // Envoie de la carte vers l'hyperterminal sur L'uart
										}
															
							if ( ((pMail->Buf[0] <= 127) && (pMail->Buf[0]>=70)) ) // joystick à gauche
										{
												while(Driver_USART1.GetStatus().tx_busy == 1); // attente buffer TX vide
												Driver_USART1.Send("V3",2);  // Envoie de la carte vers l'hyperterminal sur L'uart
										}
													
							if ( ((pMail->Buf[0] < 70) && (pMail->Buf[0]>=0)) ) // joystick a fond à gauche
										{
												while(Driver_USART1.GetStatus().tx_busy == 1); // attente buffer TX vide
												Driver_USART1.Send("V7",2);  // Envoie de la carte vers l'hyperterminal sur L'uart
										}
							
						osMailFree (qid_MailQueue, pMail);                      // free memory allocated for mail
						}
					}	
				}
			}		
	
	
	
/***********************************************************************************/	
												//Fonctions d'initialisation 
/***********************************************************************************/		
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

/***********************************************************************************/	
												// Fonctions pour le nunchunk 
/***********************************************************************************/	
void NunChuck_phase1_init(void)		// Fonction d'initialisation
	{
		//Configuration nunchuk
		write2byte(SLAVE_I2C_ADDR_WRITE, 0xF0, 0x55); //Config CTRL1
		osDelay(5);
		
		write2byte(SLAVE_I2C_ADDR_WRITE, 0xFB, 0x00); //Config CTRL2
		osDelay(5);
	}
	
	
void NunChuck_translate_data(void)	// Fonction de rangement des variables et d'envoie par MailBox
{
	Info_Nunchunk *pMail = 0;					// Déclaration des pointeur et des variables
	char byte5;	
	volatile int joy_x_axis;
	volatile int joy_y_axis;
	volatile int accel_x_axis;  
	volatile int accel_y_axis; 
	volatile int accel_z_axis; 
	volatile int z_button = 0;
	volatile int c_button = 0;
	
	byte5 = tab_read[5];							// Rangement des valeurs du nunchunk dans des variables
	joy_x_axis = tab_read[0];
	joy_y_axis = tab_read[1];
	accel_x_axis = (tab_read[2]<< 2);
	accel_y_axis = (tab_read[3]<< 2);
	accel_z_axis = (tab_read[4]<<2);
	z_button = 1;
  c_button = 1;
	
	 if ((byte5 >> 0) & 1) 						// Prise de l'information de l'état de c et z
    z_button = 0;
	 if ((byte5 >> 1) & 1)
    c_button = 0;
	
	  pMail = osMailAlloc (qid_MailQueue, osWaitForever);         // Allocate memory
    if (pMail) {
      pMail->Buf[0] = joy_x_axis;                                    // Set the mail content
			pMail->Buf[1] = joy_y_axis;                                    // Set the mail content
			pMail->Buf[2] = accel_x_axis;                                  // Set the mail content
			pMail->Buf[3] = accel_y_axis;                                  // Set the mail content
			pMail->Buf[4] = accel_z_axis;                                  // Set the mail content
			pMail->Buf[5] = z_button;                                      // Set the mail content
			pMail->Buf[6] = c_button;                                      // Set the mail content

      osMailPut (qid_MailQueue, pMail);                         // Send Mail
	
		}
}		

/*******************************************************************************/
									//Fonctions pour la communication I2C
/*******************************************************************************/
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