

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function

#include "lcd.c"
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

unsigned int path[]={F,F,F,F,L,
					F,M};

unsigned int pathindex = 0;
unsigned int dirn = N;
unsigned char sensor[3];
volatile long int ShaftCountLeft = 0; //to keep track of left position encoder 
volatile long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning
unsigned int turnL = 0,turnR =0;

unsigned long int ShaftCountC2 = 0;

unsigned int dumpCount = 0;



///////////////////////////Servo Motor Start//////////////////////////


//Configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config (void)
{
 DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
 PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
 DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
 PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config (void)
{
 DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
 PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}


//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)  
{
 float PositionPanServo = 0;
  PositionPanServo = ((float)degrees / 1.86) + 35.0;
 OCR1AH = 0x00;
 OCR1AL = (unsigned char) PositionPanServo;
}


//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
 float PositionTiltServo = 0;
 PositionTiltServo = ((float)degrees / 1.86) + 35.0;
 OCR1BH = 0x00;
 OCR1BL = (unsigned char) PositionTiltServo;
}

//Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees
void servo_3(unsigned char degrees)
{
 float PositionServo = 0;
 PositionServo = ((float)degrees / 1.86) + 35.0;
 OCR1CH = 0x00;
 OCR1CL = (unsigned char) PositionServo;
}

//servo_free functions unlocks the servo motors from the any angle 
//and make them free by giving 100% duty cycle at the PWM. This function can be used to 
//reduce the power consumption of the motor if it is holding load against the gravity.

void servo_1_free (void) //makes servo 1 free rotating
{
 OCR1AH = 0x03; 
 OCR1AL = 0xFF; //Servo 1 off
}

void servo_2_free (void) //makes servo 2 free rotating
{
 OCR1BH = 0x03;
 OCR1BL = 0xFF; //Servo 2 off
}

void servo_3_free (void) //makes servo 3 free rotating
{
 OCR1CH = 0x03;
 OCR1CL = 0xFF; //Servo 3 off
} 
///////////////////////////Servo Motor End//////////////////////////





/////////////////////////////////////////////////
//  C1 , C2 motor code
/////////////////////////////////////////////////

void c2_pin_init(void)
{
	DDRA = 0xCF;
	PORTA = 0x00;

	DDRE = DDRE | 0x04;   //Setting PL3 and PL4 pins as output for PWM generation
 	PORTE = PORTE | 0x04; //PL3 and PL4 pins are for velocity control using PWM.
}

void c2motor_control(unsigned char Direction)
{

unsigned char PortARestore = 0;

 Direction &= 0xC0; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0x3F; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}



void c2forward(void)
{
	c2motor_control(0x40);
	
}

void c2backward()
{
	c2motor_control(0x80);
}

void c2stop()
{
	c2motor_control(0x00);
	
}


void c2_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xBF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x40; //Enable internal pull-up for PORTE 4 pin
}


void c2_position_encoder_interrupt_init (void) //Interrupt 6 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x20; // INT6 is set to trigger with falling edge
 EIMSK = EIMSK | 0x40; // Enable Interrupt INT6 for right position encoder
 sei();   // Enables the global interrupt 
}


ISR(INT6_vect)
{
 ShaftCountC2++;  //increment C2 shaft position count
}


//C2 spec dist mm
void c2_linear_distance_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
 ShaftCountC2 = 0;
 while(1)
 {
  if(ShaftCountC2 > ReqdShaftCountInt)
  {
  	break;
  }
 } 
 c2stop(); //Stop c2
}


void c2forward_mm(unsigned int DistanceInMM)
{
 c2forward();
 c2_linear_distance_mm(DistanceInMM);
}

void c2backward_mm(unsigned int DistanceInMM)
{
 c2backward();
 c2_linear_distance_mm(DistanceInMM);
}




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

//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{

 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.

 // I guess for DC Motors
 //DDRE = 0x08;
 //PORTE = 0x08;
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


//Function to Initialize PORTS
void port_init()
{
	lcd_port_config();
	adc_pin_config();

	c2_pin_init();

	motion_pin_config();	
	buzzer_pin_config();
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config	
	


 	servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
	servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation 
	servo3_pin_config(); //Configure PORTB 7 pin for servo motor 3 operation  
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

//ISR for right position encoder
ISR(INT5_vect)  
{
 ShaftCountRight++;  //increment right shaft position count
}


//SR for left position encoder
ISR(INT4_vect)
{
 ShaftCountLeft++;  //increment left shaft position count
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

 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
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

//Function to initialize all the devices
void init_encoders()
{
 cli(); //Clears the global interrupt
 left_encoder_pin_config();
 right_encoder_pin_config();
 c2_encoder_pin_config();

 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 c2_position_encoder_interrupt_init();

 sei();   // Enables the global interrupt 
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


/////////////////////////////////////////////////////

void initialize_servo_motors()
{
	// Some tuning for Servo_Motor_1
	for (i = 0; i <50; i += 5)
	{
		servo_1(i);
		_delay_ms(50);
	}

	// Some tuning for Servo_Motor_2
	servo_2(0);

}


void sweeper_job()
{
	// dumpCount is for the dumping status
	dumpCount += 1;

	// Stretch the Arm
		c2forward();
		_delay_ms(1300);
		c2stop();
	
	buzzer_on();
	_delay_ms(100);
	buzzer_off();
	_delay_ms(3000);

	// Release the Arm , Servo1
		for (i = 50; i > 0; i -= 5)
 		{
  			servo_1(i);
  			_delay_ms(30);
		}

	buzzer_on();
	_delay_ms(100);
	buzzer_off();
	_delay_ms(3000);

	// Release the Container , Servo2
		for (i = 0; i < 30; i += 5)
 		{
  			servo_2(i);
  			_delay_ms(30);
		}


	buzzer_on();
	_delay_ms(100);
	buzzer_off();
	_delay_ms(3000);

	// Contract(Pull Back) the Arm
		c2backward();
		_delay_ms(1700);
		c2stop();


	buzzer_on();
	_delay_ms(100);
	buzzer_off();
	_delay_ms(3000);

	// Push the Arm slightly ahead
	c2forward();
	_delay_ms(400);
	c2stop();


	buzzer_on();
	_delay_ms(100);
	buzzer_off();
	_delay_ms(3000);

	// Take back the Conatainer , Servo2
	for (i = 30; i > 0; i -= 5)
 	{
  		servo_2(i);
  		_delay_ms(30);
	}

	buzzer_on();
	_delay_ms(100);
	buzzer_off();
	_delay_ms(3000);

	// Pull the Arm , Servo1
	for (i = 0; i < 50; i += 5)
 		{
  			servo_1(i);
  			_delay_ms(30);
		}


	buzzer_on();
	_delay_ms(1000);
	buzzer_off();
	_delay_ms(3000);

}



//Main Function
int main()
{
	init_devices();
	init_encoders();
	initialize_servo_motors();

	lcd_set_4bit();
	lcd_init();
	int value=0;
	forward();
	velocity(200,200);
	lcd_print(2,1,150,3);
	lcd_print(2,5,150,3);
	lcd_print(2,9,pathindex,2);
	lcd_print(2,13,dirn,3);

//	int shaftval = 0;

/*
	while(1)
	{
		buzzer_on();
		_delay_ms(500);
		buzzer_off();
		_delay_ms(500);		
		sweeper_job();
	}
*/

	/*while(1)
	{
		velocity(200,200);
		_delay_ms(2000);
		back_mm(50);
		//stop();
		//velocity(130,130);
		stop();
		left_degrees(50);
		rotate_left_slowly();
	 	forward();
		//velocity(130,130);
		//turnL = 0;
	}
	*/
		
	while(1)
	{	
		read_sensor();
		follow();
		
		if(isPlus())
		{
			// Added by Ravi
			stop();

			read_sensor();
			value = path[pathindex++];		
			orient(value);
			lcd_print(2,9,pathindex,2);
			lcd_print(2,13,dirn,3);
			lcd_print(1,13,turnL,1);
			lcd_print(1,15,turnR,1);
			velocity(130,130);

			//All code to do swipping stuff
			if (turnL != 1 && turnR != 1)
			{
				stop();
				sweeper_job();
				forward();
			}

			
		}
		
		if(turnL == 1)
		{
		/*
		lcd_print(1,13,turnL,1);
		back_mm(50);
		//stop();
		velocity(130,130);
		left_degrees(77);
		//_delay_ms(120);
		read_sensor();

		rotate_left_slowly();

		 stop();
	 	 forward();
		velocity(100,100);
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
		

		// Added by Ravi
		stop();
		sweeper_job();
		forward();
		
		}
		
		if(turnR == 1)
		{
		/*
		lcd_print(1,15,turnR,1);
		back_mm(50);
		stop();
		velocity(130,130);
		right_degrees(77);
		//_delay_ms(200);
		read_sensor();

		stop();
		forward();
		//follow();
		velocity(130,130);
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

		// Added by Ravi
		stop();
		sweeper_job();
		forward();
		}
	
	}


	/*unsigned char i = 0;

	servo_1(0); 

	while(1)
	{	
		for (i = 0; i <90; i++)
		{	
		  servo_1(i);
		  _delay_ms(30);
		  //servo_2(i);
		  //_delay_ms(30);
		  //servo_3(i);
		  //_delay_ms(30);
		 }

	
		 for (i = 90; i > 0; i--)
		 {
		  servo_1(i);
		  _delay_ms(30);
		  //servo_2(i);
		  //_delay_ms(30);
		  //servo_3(i);
		  //_delay_ms(30);
		 }

	}
	*/


	/*	
	while(1)
	{	
		read_sensor();
		follow();
		
		if(isPlus())
		{
			
			read_sensor();
			value = path[pathindex++];		
			orient(value);
			lcd_print(2,9,pathindex,2);
			lcd_print(2,13,dirn,3);
			lcd_print(1,13,turnL,1);
			lcd_print(1,15,turnR,1);
			
			//All code to do swipping stuff
			stop();
			sweeper_job();
			forward();
		}
		
		if(turnL == 1)
		{
		lcd_print(1,13,turnL,1);
		back_mm(50);
		//stop();
		velocity(130,130);
		left_degrees(77);
		//_delay_ms(120);
		read_sensor();

		 stop();
	 	 forward();
		velocity(100,100);
		 turnL = 0;
		}
		
		if(turnR == 1)
		{
		lcd_print(1,15,turnR,1);
		back_mm(50);
		stop();
		velocity(130,130);
		right_degrees(77);
		//_delay_ms(200);
		read_sensor();

		stop();
		forward();
		//follow();
		velocity(100,100);
		 turnR = 0;
		}
	
	}
	*/
	
}
