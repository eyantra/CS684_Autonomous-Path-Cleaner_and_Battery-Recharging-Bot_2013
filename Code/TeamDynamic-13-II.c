

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function

#include "lcd.c"
#include "location.c"


#define F 0x11
#define B 0x12
#define R 0x13
#define L 0x14
#define M 0x19

#define N 0x15
#define S 0x16
#define E 0x17
#define W 0x18

void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;

/*unsigned int path[]={F,F,F,F,F,F,R,
					R,F,F,F,F,F,L,
					L,F,F,F,F,F,R,
					R,F,F,F,F,F,L,
					L,F,F,F,F,F,R,
					R,F,F,F,F,F,L,
					L,F,F,F,F,F,M};
					*/


unsigned int path[]={F,R,F,M};

unsigned int pathindex = 0;
unsigned int dirn = N;
unsigned char sensor[3];
volatile long int ShaftCountLeft = 0; //to keep track of left position encoder 
volatile long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning
unsigned int turnL = 0,turnR =0;
float BATT_Voltage, BATT_V;


/**************** Location.h **********************/

// This will store the length of the Array storing the shortest path to return to Origin
unsigned int return_path_counter = 0;

//--------------
// Bot-Direction
//--------------
// Shows the direction in 2D
// like Four Quadrants in 2D-plane
// So 	East 	--> 0
//	North	--> 1
//	West	--> 2
//	South	--> 3

//---------------
// Grid-Direction
//---------------
// All possible directions at any point
// Forward  	FR = 0 
// Left			LT = 1
// Right 		RT = 2
// Stop			ST = 3


void init_location(void)
{
	set_direction(north);
}

/***************** Location.h - Closed ***************/

//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}



//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}


//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}



void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}




//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x10; //Enable internal pullup for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x20; //Enable internal pullup for PORTE 4 pin
}



void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
 EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
 sei();   // Enables the global interrupt 
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
 EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
 sei();   // Enables the global interrupt 
}


// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}


// This Function calculates the actual distance in millimeters(mm) from the input
// analog value of Sharp Sensor. 
// Applicableo Battery Only
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}


//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}

void forward (void) 
{
  motion_set (0x06);
}

void stop (void)
{
  motion_set (0x00);
}

void back (void) //both wheels backward
{
  motion_set(0x09);
}
void right(void)
{
  motion_set(0x0A);
}
void left(void)
{
  motion_set(0x05);
}
void soft_left (void) //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
 motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}

//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 ShaftCountRight = 0; 
 ShaftCountLeft = 0; 

 while (1)
 {
  if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
  break;
 }
 //stop(); //Stop action
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
 ShaftCountRight = 0;
 while(1)
 {
  if(ShaftCountRight > ReqdShaftCountInt)
  {
  	break;
  }
 } 
 stop(); //Stop action
}

void forward_mm(unsigned int DistanceInMM)
{
 forward();
 linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
 back();
 linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees) 
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 left(); //Turn left
 angle_rotate(Degrees);
}



void right_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 right(); //Turn right
 angle_rotate(Degrees);
}


void soft_left_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_left(); //Turn soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_right();  //Turn soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_left_2(); //Turn reverse soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_right_2();  //Turn reverse soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer5_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();   //Enables the global interrupts

	// To initialize the direction of the bot to north
	init_location();
}


void show_battery_status()
{
	BATT_V = ADC_Conversion(0);
	BATT_Voltage = ((ADC_Conversion(0)*100)*0.07902) + 0.7;	//Prints Battery Voltage Status
	lcd_print(1,1,BATT_Voltage,4);
}


void read_sensor()
{
	Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
	Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
	Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
	
/*	print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3
		*/
	lcd_print(1,1,Left_white_line,3);
	lcd_print(1,5,Center_white_line,3);
	lcd_print(1,9,Right_white_line,3);
	
}

void follow()
{
	//flag=0;
	if(Center_white_line>0x20 && Left_white_line<0x20 && Right_white_line<0x20)
		{
			//flag=1;
			//forward();
			velocity(130,130);
			lcd_print(2,1,130,3);
			lcd_print(2,5,130,3);	
		}

		else if((Left_white_line>0x20 && Center_white_line<0x20) )
		{
			//flag=1;
		//	forward();
			velocity(105,130);
			lcd_print(2,1,105,3);
			lcd_print(2,5,130,3);
		}

		else if((Right_white_line>0x20 && Center_white_line<0x20))
		{
			//flag=1;
			//forward();
			velocity(130,105);
			lcd_print(2,1,130,3);
			lcd_print(2,5,105,3);
		}
		
	
}

int isPlus()
{

	if((Left_white_line >0x20 && Center_white_line>0x20) || (Right_white_line >0x20 && Center_white_line>0x20))
	{
	buzzer_on();
	_delay_ms(100);
	buzzer_off();
	_delay_ms(100);	


	return 1;
	}
	else
	{
	return 0;
	}
}
	
	void turnLeft()
	{
	forward_mm(50);
	stop();
	left();
	_delay_ms(200);
	read_sensor();
	 while(Left_white_line <0x40)
	 {
	 read_sensor();
	   left();
	}
	stop();
	_delay_ms(200);
	read_sensor();
	forward();

	}
	void turnRight()
	{
	forward_mm(50);
	stop();
	right();
	_delay_ms(200);
	read_sensor();
	 while(Right_white_line <0x40 )
	 {
	 read_sensor();
	 right();
	 }
	 stop();
	_delay_ms(200);
	read_sensor();
	forward();
	}


//Function to initialize all the devices
void init_encoders()
{
 cli(); //Clears the global interrupt
 left_encoder_pin_config();
 right_encoder_pin_config();


 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 

 sei();   // Enables the global interrupt 
}


//Function to Initialize PORTS
void port_init()
{
	// Common to Battery and Line Following
	lcd_port_config();
	adc_pin_config();

	// Applicable to Line Following
	motion_pin_config();	

	buzzer_pin_config();
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config	
}



void orient(int value)
{
	switch(value)
	{

		case F:
			
		
			while(isPlus())
			{
				read_sensor();
				follow();
			}

				break;

		case L:
				//turnLeft();
					turnL =1;
			if(dirn == N)
			{
			//	turnLeft();
				dirn = W;
			}
			else if (dirn == E)
			{
			//	turnBack();
				dirn = N;
			}
			else if (dirn == W)
			{
				dirn=S;
			}
			else if (dirn == S)
			{
			//	turnRight();
				dirn = E;
			}
				break;
		case R:
		//turnRight();
		turnR =1;
		if(dirn == N)
			{
				//turnRight();
				dirn = E;
			}
			else if (dirn == E)
			{
				dirn =S;
			}
			else if (dirn == W)
			{
				//turnBack();
				dirn = N;
			}
			else if (dirn == S)
			{
				//turnLeft();
				dirn = W;
			}
			
				break;
		case M:
		stop();
		
	}
	

}

/*
int map(int pindex)
{
	switch(pindex)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 14:
		case 15:
		case 16:
		case 17:
		case 18:
		case 19:
		case 20:
		case 28:
		case 29:
		case 30:
		case 31:
		case 32:
		case 33:
		case 34:
		case 42:
		case 43:
		case 44:
		case 45:
		case 46:
		case 47:
		case 48:return pindex;
		case 7: return 13;
		case 8: return 12;
		case 9: return 11;
		case 10:return 10;
		case 11:return 9;
		case 12:return 8;
		case 13:return 7;
		case 21:return 27;
		case 22:return 26;
		case 23:return 25;
		case 24:return 24;
		case 25:return 23;
		case 26:return 22;
		case 27:return 21;
		case 35:return 41;
		case 36:return 40;
		case 37:return 39;
		case 38:return 38;
		case 39:return 37;
		case 40:return 36;
		case 41:return 35;
	
	}
	return -1;
}
*/


void rotate_left_slowly()
{
	//velocity(150,150);

	while(1)
	{
		stop();
		read_sensor();

		// For center black line
		if(Center_white_line>0x20)
		{
			break;	
		}

		else
		{			
			left_degrees(10);
		}

	}

	//velocity(130,130);
	

}

void rotate_right_slowly()
{
	//velocity(150,150);

	while(1)
	{
		stop();
		read_sensor();

		// For center black line
		if(Center_white_line>0x20)
		{
			break;	
		}

		else
		{
			right_degrees(10);			
		}

	}

	//velocity(130,130);
	//stop();

}



//Main Function
int main()
{
	init_devices();
	init_encoders();
	lcd_set_4bit();
	lcd_init();
	int value=0;
	forward();
	velocity(130,130);
	lcd_print(2,1,130,3);
	lcd_print(2,5,130,3);
	lcd_print(2,9,pathindex,2);
	lcd_print(2,13,dirn,3);


	while(1)
	{	
		read_sensor();
		follow();
		
		if(isPlus())
		{	
			read_sensor();
			value = path[pathindex++];
			
			// Code inserted for calculation of actual location wrt initial starting point , 
			// It will consider direction also.

			if (value == F)
			{
				// Move the bot forward, for location only , No movement on ground.
				move_bot(FR);
			}
			else if (value == L)
			{
				// Move the bot left , for location only , No movement on ground.
				move_bot(LT);
			}
			else if (value == R)
			{
				// Move the bot right, for location only , No movement on ground.
				move_bot(RT);
			}
			else if (value == M)
			{
				// To stop the Bot and then break out
				stop();
				break;
			}
					
			orient(value);

/*			lcd_print(2,9,pathindex,2);
			lcd_print(2,13,dirn,3);
			lcd_print(1,13,turnL,1);
			lcd_print(1,15,turnR,1);
*/
			
		}
		
		if(turnL == 1)
		{/*
		lcd_print(1,13,turnL,1);
		forward_mm(20);
		stop();
		velocity(180,180);
		left_degrees(95);
		//_delay_ms(120);
		read_sensor();
		//	 while(Left_white_line <0x40)
		// {
		//	read_sensor();
		//	left();
		// }
		 stop();
	 	 forward();
		velocity(180,180);
		 turnL = 0;
		 */

		 back_mm(50);
		//stop();
		//velocity(130,130);
		stop();
		left_degrees(50);
		rotate_left_slowly();
	 	forward();
		velocity(130,130);
		turnL = 0;
		}
		
		if(turnR == 1)
		{
		/*
		lcd_print(1,15,turnR,1);
		forward_mm(20);
		stop();
		velocity(180,180);
		right_degrees(95);
		//_delay_ms(200);
		read_sensor();
		// while(Right_white_line <0x30)
		// {
		// read_sensor();
		// right();
		// }
		stop();
		forward();
		//follow();
		velocity(180,180);
		 turnR = 0;
		*/

		back_mm(50);
		//stop();
		//velocity(130,130);
		stop();
		right_degrees(50);
		rotate_right_slowly();
	 	forward();
		velocity(130,130);
		turnR = 0;
		}
	
	}


	// Three Beeps for Interval
	buzzer_on();
	_delay_ms(100);
	buzzer_off();
	_delay_ms(100);

	buzzer_on();
	_delay_ms(100);
	buzzer_off();
	_delay_ms(100);

	buzzer_on();
	_delay_ms(100);
	buzzer_off();
	_delay_ms(100);

	//code to head-back to starting position , i.e. Origin
	return_path_counter = reach_origin();
	
	forward();
	velocity(130,130);
	int counter = 0;
	int intermediate_value = 0;

	while(counter < return_path_counter)
	{	
		read_sensor();
		follow();
		
		if(isPlus())
		{	
			read_sensor();
			value = path_to_origin[counter];
			counter++;
			
			// Code inserted for calculation of actual location wrt initial starting point , 
			// It will consider direction also.

			if (intermediate_value == FR)
			{
				// Move the bot forward, for location only , No movement on ground.
				value = F;
			}
			else if (value == LT)
			{
				// Move the bot left , for location only , No movement on ground.
				value = L;
			}
			else if (value == RT)
			{
				// Move the bot right, for location only , No movement on ground.
				value = R;
			}
			else if (value == ST)
			{
				value = M;
				// specially inserted as break will not allow the bot to stop using "orient(value)".
				orient(value);
				break;
			}
					
			orient(value);
		
		}
		
		if(turnL == 1)
		{
		 back_mm(50);
		//stop();
		//velocity(130,130);
		stop();
		left_degrees(50);
		rotate_left_slowly();
	 	forward();
		velocity(130,130);
		turnL = 0;
		}
		
		if(turnR == 1)
		{
		back_mm(50);
		//stop();
		//velocity(130,130);
		stop();
		right_degrees(50);
		rotate_right_slowly();
	 	forward();
		velocity(130,130);
		turnR = 0;
		}
	
	}


	// Three beeps for Finish
	buzzer_on();
	_delay_ms(100);
	buzzer_off();
	_delay_ms(100);

	buzzer_on();
	_delay_ms(100);
	buzzer_off();
	_delay_ms(100);

	buzzer_on();
	_delay_ms(100);
	buzzer_off();
	_delay_ms(100);

}
