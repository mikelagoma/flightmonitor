int i = 0;
int x = 0;       //integers to store Accelerometer readings 
int y = 0;
int z = 0;
int S0 = 0;      //Select pins of Mux
int S1 = 0;
int Accel = 0;   //which Accelerometer we are selecting

String sensorstring = "";    //String to store readings
boolean sensor_stringcomplete = false; 
// these constants describe the pins. They won't change:
const int xpin = A1;                  // x-axis of the accelerometer
const int ypin = A2;                  // y-axis
const int zpin = A3;                  // z-axis (only on 3-axis models)

int sampleDelay = 10;   //number of milliseconds between readings

void setup()
{
  // initialize the serial communications:
  Serial.begin(9600);

  //Make sure the analog-to-digital converter takes its reference voltage from
  // the AREF pin
  analogReference(EXTERNAL);

  pinMode(xpin, INPUT);
  pinMode(ypin, INPUT);
  pinMode(zpin, INPUT);
  pinMode(2, OUTPUT);    // S0
  pinMode(3, OUTPUT);    // S1
  pinMode(4, OUTPUT);    // P0
}

void loop()
{
  for (Accel=0; Accel<=3; Accel++) {

    // select the bit  
    S0 = bitRead(Accel,0);    // use this with arduino 0013 (and newer versions)     
    S1 = bitRead(Accel,1);    // use this with arduino 0013 (and newer versions)     

    digitalWrite(2, S0);
    digitalWrite(3, S1);

    //Either read or write the multiplexed pin here  36
    x = analogRead(xpin);
    y = analogRead(ypin);
    z = analogRead(zpin);
    sensorstring+= x;
    sensorstring+= y;
    sensorstring+= z;
    //If the final accelerometer is selected, all readings have been collected
    if(Accel==3){
       Serial.println(sensorstring); 
       sensorstring="";
    }
    // delay before next reading:
    delay(sampleDelay);
  }
}
