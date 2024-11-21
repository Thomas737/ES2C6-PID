//code made for ES2C6 Civil group project, group 5
//uses parts of supplied code: testingMotorDrive, testingStrainGauge
//hopefully comments make this understandable


//Objective of code:
//use a PID controller to move actuator to a certain position dependent on measured strain

int began = 0;


float kd = 0;
//PID controller values for tuning
float kp = 4;
float ki = 0.12;

float time;  //other values used by PID controller
float lasttime = 0;
float error = 0;
float lasterror = 0;
float integral = 0;
float derivative = 0;
int output = 0;
int target = 950;

int pwm_a = 3;  //PWM control for motor (speed, 0-255)
int dir_a = 2;  //direction control for motor

int strainPin1 = A0;  //pin that measures strain gauge voltage (analogue 0-1023)
float R1 = 120;       //the resistances are 120 Ohms (confirmed by the technicians)
float R2 = R1;        //all resistors in the wheatstone bridge are equal
float R3 = R1;
float Rg_init = R1;  //unstressed resistance of strain gauge (https://www.tml.jp/e/node/155)
float Vin = 5;       //voltage across the wheatstone bridge supplied by arduino power
float intS1;         //voltage measured from strain gauge (0-5V)

float rolling_average;
float rolling_array[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

int n_rolling = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(strainPin1, INPUT);  //taking input for strain gauge
  pinMode(A2, INPUT);          //potentiometer positional feedback pin :)
  pinMode(pwm_a, OUTPUT);      //Set control pins to be output for voltage
  pinMode(dir_a, OUTPUT);      // set control pins to be output for direction

  analogWrite(pwm_a, 0);     //set  motor to not run (0-255)
  digitalWrite(dir_a, LOW);  //set direction to extend (0-1)
}

void loop() {
  // put your main code here, to run repeatedly

  //
  //strain measurement
  //

  // calibrate strain gauge (find unstressed voltage, and subtract from subsequent measurements)
  if (analogRead(A2) < 940 & began == 0) {
    analogWrite(pwm_a, 255);
    digitalWrite(dir_a, HIGH);
  } else {
    if (began == 0) {
      analogWrite(pwm_a, 0);

      for (int i = 0; i < 6; i++) {
        Serial.println("trying");
        float strainReading1a = analogRead(strainPin1);  //reading in the range 0-1023 (represents 0-5V)
        float strainReading1b = (strainReading1a * 5 / 1023);
        intS1 = strainReading1b;  // unstressed strain reading in voltage units
        Serial.print(intS1);
        delay(500);
      }
      began = 1;
    }
  }
  if (began == 1) {
    float strainReading1a = analogRead(strainPin1);                //reading in the range 0-1023 (represents 0-5V)
    float strainReading1b = (strainReading1a * 5 / 1023) - intS1;  // strain readings in voltage units
    int K = 2.1;                                                   //gauge factor (https://www.tml.jp/e/node/155)
    int rsGain = 495;                                              //gain of the amplifier (AD8426 datasheet p.21; 100Ohm default)

    float Vout1 = strainReading1b / rsGain;       //calculating the output voltage from the wheatstone bridge 1
    float num1 = (R1 + R2) * Vin * R3;            //numerator of Rg equation
    float denom1 = Vout1 * (R1 + R2) + Vin * R2;  //denominator of Rg equation
    float Rg1 = (num1 / denom1) - R3;             //calculating strain gauge resistance
    float dR1 = Rg_init - Rg1;          //calculating the change in strain resistance (should probably not be absolute that may break something)
    rolling_array[n_rolling % 10] = (dR1 / (Rg_init * K));
    float strain1=0;
    for (int j = 0; j < 10; j++) {
        strain1 += rolling_array[j];  // using the strain resistance and dR to calculate the strain (K is gauge factor)
    }
    strain1 = strain1 / 10;


    //
    //PID controller
    //

    target = 800 + constrain(strain1 * 1600000, -700, 150);  //sets a target position dependent on strain, within safe bounds for actuator

    // if (millis()%12000<6000)
    // {target=100;}
    // else{target=300;}



    time = millis();                  //time in ms since program begun
    error = target - analogRead(A2);  //finds current error (difference between target and current positions)

    derivative = (error - lasterror) / ((time - lasttime) / 1000);       //estimates derivative of error using d(error)/d(time)
    integral += ((error + lasterror) / 2) * ((time - lasttime) / 1000);  //estimates integral of error between last measurement time and current time
    integral = constrain(integral, -255, 255);                           //prevents integral value from becoming massive if error persists for a long time
    output = kp * error + ki * integral + kd * derivative;               //uses K and error values to create control signal

    if (output < 0) {  //sets direction to move dependent on control signal
      digitalWrite(dir_a, LOW);
    } else {
      digitalWrite(dir_a, HIGH);
    }

    output = constrain(abs(output), 0, 255);  //constrain control within arduino analogue output range
    if (output > 40) {                        //with a low supply voltage (<~1V), the motor may stop moving, which may damage it if prolonged
      analogWrite(pwm_a, output);
    } else {
      analogWrite(pwm_a, 0);
    }

    lasttime = time;  //sets previous values to current ones, as they are no longer needed in this loop
    lasterror = error;

    // //debug outputs
    Serial.print(0);  // To freeze the lower limit
    Serial.print(" ");
    Serial.print(1000);  // To freeze the upper limit
    Serial.print(" ");
    Serial.print(String(strain1, 6));
    Serial.print(" , ");
    Serial.print(target);
    Serial.print(" , ");
    Serial.print(analogRead(A2));
    Serial.print(" , ");
    // Serial.print(millis());
    // Serial.print(" , ");
    Serial.println(output);
  }
  n_rolling += 1;
  delay(10);  //fiddle with tihs to see how responsive it can get (quite!!!!!!!!)
}