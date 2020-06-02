/*
 Name:		VEEGBOT_SLAVE.ino
 Created:	02/06/2020 11:12:56
 Author:	nico_
*/




int PUMP_ONOFF, VACUUM_PUMP_ONOFF;


int MAX_Z_STEPS, MAX_R_STEPS, MAX_T_STEPS;

int Z_ENABLE, Z_DIR, Z_STEP, Z_MIN, Z_MAX, Z_COUNT, Z_SPEED;
int R_ENABLE, R_DIR, R_STEP, R_MIN, R_MAX, R_COUNT, R_SPEED;


const int CW = HIGH;
const int CCW = LOW;


bool enableStateR = false;
bool enableStateZ = false;


int countingStepsR = 0;
int countingStepsZ = 0;

float R_STEPDIST = 3.14 * 10 / 200;
float Z_STEPDIST = 3.14 * 10 / 200;                 //distance pacourue par 1 pas V

int FULL_REVOLUTION_STEPS = 200;

int R_OFFSET = 150;     //mm
int Z_MIN_OFFSET = 50; // mm
int Z_MAX_OFFSET = 50; //mm



//TIMER VARIABLE                // valeur a changer ce sont juste des tests !

long R_HOMMING_INTERVAL = 1000000; //milliseconds
long Z_HOMMING_INTERVAL = 1000000;





#define STEPS 400 // va avec la fonction STEPPER_ACCEL_MOV



bool timer(long startTime, long interval)
{
    unsigned long currentMillis = millis();
    if ((currentMillis - startTime) > interval)
    {
        return (true);
    }
    else
    {
        return (false);
    }
}

int CHECK_ENDSTOP_Z()
{
    if (digitalRead(Z_MIN) == HIGH || digitalRead(Z_MAX) == HIGH)
    {
        //send notification error
        Serial.println("error endswitch sur Z");
    }
}


int CHECK_ENDSTOP_R()
{
    if (digitalRead(R_MIN) == HIGH /*|| digitalRead(R_MAX) == HIGH*/)
    {
        //send notification error
        Serial.println("error endswitch sur R");
    }
}




//***************** POSITION TROLLEY **************************
int R_POS(int compt, int NEW_POS_R, int minSpeed, int maxSpeed)
{
    float  numSTEPS = (compt - NEW_POS_R) / R_STEPDIST;
    if (NEW_POS_R > MAX_R_STEPS) // on verifie que l'on ne depasse pas la longueur du chariot
    {
        NEW_POS_R = MAX_R_STEPS;
    }
    int lowSpeed = minSpeed;
    int highSpeed = maxSpeed;
    int change = 2;
    int rampUpEnding = (lowSpeed - highSpeed) / change;
    if (rampUpEnding > abs(numSTEPS) / 2)
    {
        rampUpEnding = abs(numSTEPS) / 2;
    }
    int rampDownBegin = abs(numSTEPS) - rampUpEnding;
    int speedDelay = lowSpeed;
    digitalWrite(R_ENABLE, LOW);
    // on regarde dans quel sens on part
    if (numSTEPS < 0)
    {
        digitalWrite(R_DIR, HIGH); // verifier sens !
    }
    else
    {
        digitalWrite(R_DIR, LOW); // verifier sens !
    }
    for (int i = 0; i < abs(numSTEPS); i++)   // on avance ou recule du nombre de pas
    {
        digitalWrite(R_STEP, HIGH);
        digitalWrite(R_STEP, LOW);
        delayMicroseconds(speedDelay);
        if (i < rampUpEnding)
        {
            speedDelay -= change;
        }
        else if (i > rampDownBegin)
        {
            speedDelay += change;
        }
        //Serial.println(speedDelay);
    }
    digitalWrite(R_ENABLE, HIGH);
    R_COUNT = NEW_POS_R;
    CHECK_ENDSTOP_R();
}

//********************* HOME TROLLEY **************************
int R_HOME()
// retour chariot horizontal s'arrete quand stop passe de 0 a 1
{
    R_COUNT = 0;
    unsigned long startMillis = millis();
    digitalWrite(R_ENABLE, LOW);
    digitalWrite(R_DIR, LOW); // verifier sens !
    while (digitalRead(R_MIN) == LOW)
    {
        if (timer(startMillis, R_HOMMING_INTERVAL) == true)
        {
            Serial.println("Time out sur R");
            break;
        }
        digitalWrite(R_STEP, HIGH);
        delayMicroseconds(R_SPEED);
        digitalWrite(R_STEP, LOW);
        delayMicroseconds(R_SPEED);
    }
    digitalWrite(R_ENABLE, HIGH);
    R_POS(R_COUNT, R_OFFSET, 2000, 500);
    R_COUNT = R_COUNT - R_OFFSET;
}

//****************** TROLLEY CALIBRATION **********************
int R_CALIBRATION()
{

}




//********************** ARM POSITION *************************
int Z_POS(int compt, int NEW_POS_Z)
{
    int buffer = 0;
    float numSTEPS = (compt - NEW_POS_Z) / Z_STEPDIST;
    if (NEW_POS_Z > MAX_Z_STEPS)    // on verifie que l'on ne depasse pas la longueur du chariot
    {
        NEW_POS_Z = MAX_Z_STEPS;
    }
    digitalWrite(Z_ENABLE, LOW);
    if (numSTEPS < 0)     // on regarde dans quel sens on part
    {
        digitalWrite(Z_DIR, HIGH); // verifier sens !
    }
    else
    {
        digitalWrite(Z_DIR, LOW); // verifier sens !
    }
    for (int i = 0; i < abs(numSTEPS); i++)    // on avance ou recule du nombre de pas
    {
        digitalWrite(Z_STEP, HIGH);
        delayMicroseconds(Z_SPEED);
        digitalWrite(Z_STEP, LOW);
        delayMicroseconds(Z_SPEED);
        if (digitalRead(Z_MAX) == HIGH)         // a modifié : coder pour que le bras descende jusqu'à ce que le Z_MAX s'active pour palper le terrain
        {
            digitalWrite(Z_DIR, LOW);                        // à vérifié le sens
            for (int j = 0; j < Z_MAX_OFFSET; j++)
            {
                digitalWrite(Z_STEP, HIGH);
                delayMicroseconds(Z_SPEED);
                digitalWrite(Z_STEP, LOW);
                delayMicroseconds(Z_SPEED);
                buffer = -j;
            }
            break;
        }
        buffer = buffer + i;
    }
    digitalWrite(Z_ENABLE, HIGH);
    Z_COUNT = NEW_POS_Z;
    CHECK_ENDSTOP_Z();
}

//********************** HOME ARM *****************************
int Z_HOME()
// retour chariot horizontal s'arrete quand stop passe de 0 a 1
{
    Z_COUNT = 0;
    unsigned long startMillis = millis();
    digitalWrite(Z_ENABLE, LOW);
    digitalWrite(Z_DIR, LOW); // verifier sens !
    while (digitalRead(Z_MIN) == LOW)
    {
        if (timer(startMillis, Z_HOMMING_INTERVAL) == true)
        {
            Serial.println("Time out sur Z");
            break;
        }
        digitalWrite(Z_STEP, HIGH);
        delayMicroseconds(Z_SPEED);
        digitalWrite(Z_STEP, LOW);
        delayMicroseconds(Z_SPEED);
    }
    digitalWrite(Z_ENABLE, HIGH);
    Z_POS(Z_COUNT, Z_MIN_OFFSET);
    Z_COUNT = Z_COUNT - Z_MIN_OFFSET;
}





//************** HOMMING ALL MOTOR (Z,R,TETA) *****************
int HOMMING_ALL()
{
    Z_HOME();
    R_HOME();
}



void STEPPER_ACCEL_MOV(int STEP_PIN) {
    int delays[STEPS];
    float angle = 1;
    float accel = 0.01;
    float c0 = 2000 * sqrt(2 * angle / accel) * 0.67703;
    float lastDelay = 0;
    int highSpeed = 100;
    for (int i = 0; i < STEPS; i++) {
        float d = c0;
        if (i > 0)
            d = lastDelay - (2 * lastDelay) / (4 * i + 1);
        if (d < highSpeed)
            d = highSpeed;
        delays[i] = d;
        lastDelay = d;
    }
    // use delays from the array, forward
    for (int i = 0; i < STEPS; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(delays[i]);
        digitalWrite(STEP_PIN, LOW);
    }
    // use delays from the array, backward
    for (int i = 0; i < STEPS; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(delays[STEPS - i - 1]);
        digitalWrite(STEP_PIN, LOW);
    }
}


//****************** ENABLE/DISABLE MOTOR *********************


void ENABLE_R()
{
    if (enableStateR == true)
    {
        digitalWrite(R_ENABLE, !enableStateR);
        enableStateR = !enableStateR;
    }
    else
    {
        digitalWrite(R_ENABLE, !enableStateR);
        enableStateR = !enableStateR;
    }
}


void ENABLE_Z()
{
    if (enableStateZ == true)
    {
        digitalWrite(Z_ENABLE, !enableStateZ);
        enableStateZ = !enableStateZ;
    }
    else
    {
        digitalWrite(Z_ENABLE, !enableStateZ);
        enableStateZ = !enableStateZ;
    }
}


//**************** ENABLE/DISABLE ALL MOTOR *******************
void ENABLE_ALL()
{
    ENABLE_R();
    ENABLE_Z();
}



//************* CONTROL TROUGH SERIAL MONITOR *****************
void SERIAL_COMMAND()
{
    if (Serial.available() > 0)
    {
        if (Serial.peek() == 'H')
        {
            int clrBuffer;
            Serial.read();
            clrBuffer = Serial.parseInt();
            HOMMING_ALL();
        }

        else if (Serial.peek() == 'R')
        {
            int numSteps = 0;
            Serial.read();
            numSteps = Serial.parseInt();
            Serial.println(numSteps);
            R_POS(R_COUNT, numSteps, 2000, 500);

        }
        else if (Serial.peek() == 'Z')
        {
            int numSteps = 0;
            Serial.read();
            numSteps = Serial.parseInt();
            Serial.println(numSteps);
            Z_POS(Z_COUNT, numSteps);
        }
        else if (Serial.peek() == 'E')
        {
            int clrBuffer;
            ENABLE_ALL();
            Serial.read();
            clrBuffer = Serial.parseInt();
        }
        else
        {
            int clrBuffer;
            Serial.read();
            clrBuffer = Serial.parseInt();
        }
        Serial.println(" Degree");
        Serial.print("StepsR : ");
        Serial.print(R_COUNT);
        Serial.println(" cm");
        Serial.print("StepsZ : ");
        Serial.print(Z_COUNT);
        Serial.println(" cm");
        Serial.println(" ");
    }
    else
    {

    }
}


void setup() {

    // open the serial port at 9600 bps:
    Serial.begin(9600);

    //defintion pins chariot radial
    R_ENABLE = 15;      //enable
    R_DIR = 17;         //direction
    R_STEP = 16;        //step
    R_MIN = 27;         //fin de course

    // définition pins chariot vertical
    Z_ENABLE = 26;      //enable
    Z_DIR = 12;         //direction
    Z_STEP = 4;         //step
    Z_MAX = 39;         //fin de course en haut
    Z_MIN = 36;         //fin course contact sol




    // definition pin auxilaire
    PUMP_ONOFF = 55;    //pompe a eau
    VACUUM_PUMP_ONOFF = 56;     //pompe a vide

    // delai en micros entre HIGH et LOW 
    Z_SPEED = 500;
    R_SPEED = 500;


    MAX_R_STEPS = 100000;
    MAX_Z_STEPS = 500000;
    MAX_T_STEPS = 100000; // variable a bien définir 1000 est une valeur random pour le moment ! 

    //definition type pin
    pinMode(Z_ENABLE, OUTPUT);
    pinMode(Z_DIR, OUTPUT);
    pinMode(Z_STEP, OUTPUT);
    pinMode(Z_MAX, INPUT);
    pinMode(Z_MIN, INPUT);

    pinMode(R_ENABLE, OUTPUT);
    pinMode(R_DIR, OUTPUT);
    pinMode(R_STEP, OUTPUT);
    pinMode(R_MIN, INPUT);


    pinMode(PUMP_ONOFF, OUTPUT);
    pinMode(VACUUM_PUMP_ONOFF, OUTPUT);
    digitalWrite(VACUUM_PUMP_ONOFF, LOW);
    digitalWrite(PUMP_ONOFF, LOW);



    //TETA_CALIBRATION();
    //HOMMING_ALL();
}

void loop()
{
    SERIAL_COMMAND();

}


