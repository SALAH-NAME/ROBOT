#include <SoftwareSerial.h>


#define BT_RX 11
#define BT_TX 12


#define M1P1 2
#define M1P2 3
#define M2P1 4
#define M2P2 5

#define M1P3 9
#define M2P3 10


SoftwareSerial BTSerial(BT_RX, BT_TX);


int	speed = 250;


const char FORWARD = 'w';
const char BACKWARD = 's';
const char LEFT = 'a';
const char RIGHT = 'd';
const char STOP = 'x';


void setup() {
	pinMode(M1P1, OUTPUT);
	pinMode(M1P2, OUTPUT);
	pinMode(M1P3, OUTPUT);

	pinMode(M2P1, OUTPUT);
	pinMode(M2P2, OUTPUT);
	pinMode(M2P3, OUTPUT);

	digitalWrite(M1P1, LOW);
	digitalWrite(M1P2, LOW);
	digitalWrite(M1P3, LOW);

	digitalWrite(M2P1, LOW);
	digitalWrite(M2P2, LOW);
	digitalWrite(M3P3, LOW);

	BTSerial.begin(9600);
}

void loop()
{
	if (BTSerial.available() > 0)
	{
		data = BTSerial.read();

		switch (data)
		{
			case FORWARD:
				move_forward();
				break;
			case BACKWARD:
				move_backward();
				break;
			case LEFT:
				turn_left();
				break;
			case RIGHT:
				turn_right();
				break;
			case STOP:
				stop();
				break;
			default:
				break;
		}
	}
}

void move_forward() {
  digitalWrite(M1P1, HIGH);
  digitalWrite(M1P2, LOW);
  digitalWrite(M2P1, LOW);
  digitalWrite(M2P2, HIGH);

  analogWrite(M1P3, speed);
  analogWrite(M1P3, speed);
} 

void move_backward() {
  digitalWrite(M1P2, HIGH);
  digitalWrite(M1P1, LOW);
  digitalWrite(M2P2, LOW);
  digitalWrite(M2P1, HIGH);

  analogWrite(M1P3, speed);
  analogWrite(M1P3, speed);
} 

void turn_left() {
  digitalWrite(M1P1, HIGH);
  digitalWrite(M1P2, LOW);
  digitalWrite(M2P2, LOW);
  digitalWrite(M2P1, HIGH);

  analogWrite(M1P3, speed);
  analogWrite(M1P3, speed);
} 

void turn_right() {
  digitalWrite(M1P2, HIGH);
  digitalWrite(M1P1, LOW);
  digitalWrite(M2P1, LOW);
  digitalWrite(M2P2, HIGH);

  analogWrite(M1P3, speed);
  analogWrite(M1P3, speed);
}

void stop() {
  analogWrite(M1P3, 0);
  analogWrite(M1P3, 0);
}
