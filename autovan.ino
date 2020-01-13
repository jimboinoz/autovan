/*
Autovan - Automate caravan environment

This is essentially a sketch to accept commands, execute actions, and/or either return sensor data
or countrol outputs.
Command format is "<command,param1,param2,param3" where command can be:

    Command id Command           Params (in)              Purpose
    ========== ========= ============================ =============================================
        0      noop      none                         Do nothing (default)
        1      reset     none                         Reset Arduino via rst pin
        2      gettemp   pin                          Output temperature in ॰C
        3      gethum    pin                          Output humidity as %
        4      getbattV  pin                          Read battery voltage on analog battVpin
        5      getbattI  pin                          Read battery current (A) on analog battItpin
        6      pinon     pin                          Turn digital pin on (high)
        7      pinnoff   pin                          Turn digital pin off (low)
        8      readpin   pin                          Read digital pin (0 low, 1 high)
        9      readvolt  pin, scale maximum           Read voltage on analog pin
       10      writepwm  pin, voltage, scale maximum  Write analog value to pwm pin
       11      writeRGB  red, green, blue values,     Set RGB led to specified values
                         red, green, blue pins
       12      debug     0=off else turn on           Set debug to output diagnostics or not via tx0
   
*/

//=====================================================================
//==== I N I T I A L I S A T I O N ====================================
//=====================================================================

  #include <Wire.h>
  #include <avr/wdt.h> // add watchdog timer library



  // define Arduino Mega 2560 autovan pin wiring
  // Digital pins
  #define modeledpin 3           // Redarc Mode LED via LDR (IRQ1 change state)
  #define RGBredpin 4            // RGB led pwm red 
  #define RGBgreenpin 5          // RGB led pwm green 
  #define RGBbluepin 6           // RGB led pwm blue
  #define solledpin 18           // Redarc solar LED via LDR (IRQ5 change state)
  #define vehledpin 19           // Redarc vehicle LED via LDR (IRQ4 change state)
  #define SDApin 20              // I2C Serial data - RTC (104) 
  #define SCLpin 21              // I2C Serial clock - RTC (104)
  #define resetpin 22            // Hardware reset output0
  #define fanpin 23              // Fan control
  #define buzzpin 24             // Buzzer control
  #define DHTpin 25              // Digital Humidity and Temperature sensor
  // Analog pins
  #define battVpin 1             // Main caravan battery voltage via resistor divider
  #define battIpin 2             // Main caravan battery current via shunt amplifier
  #define solVpin 3              // Solar Panel output voltage
  #define solIpin 4              // Solar Panel output current
  #define vehVpin 5              // Vehicle output voltage
  #define vehIpin 6              // Vehicle output current
  #define redIpin 7              // Redarc BDDC1240D output current
  #define windIpin 8             // Wind generator output current
  #define enerIpin 9             // Enerdrive 240volt charger output current
  #define water1pin 10           // Watertank 1 (front) level
  #define water2pin 11           // Watertank 2 (rear) level
  #define greywpin 12            // Grey watertank level

  
  // To set the clock, run the sketch and use the serial monitor.
  // Enter T1124154091014; the code will read this and set the clock. See the code for full details.
  //
  #define DS3231_I2C_addr 104 // Set address of RTC to 104
byte seconds, minutes, hours, day, date, month, year;
  char weekDay[4];
  
  byte tMSB, tLSB;
  float temp3231;
  
  
  // definine constants and variables
  
  #define tempon 30.0            // Temperature to turn fan on
  #define tempoff 28.0           // Temperature to turn fan off
  #define maxscale 5             // Standard analog read 0v to 5v (Vcc)
  #define battVmax 20            // Maximum scale when dealing with caravan
  #define battVover 20           // Maximum allowable battery voltage
  #define battVunder 11.8        // Minimum allowable battery voltage
  #define battVcrit  11.2        // Minimum allowable battery voltage
  #define battImax 200           // Maximum scale when dealing with caravan
  #define battcorrect 1.0        // Correction factor to correct voltage divider
  #define currentcorrect 1.0     // Correction factor to correct shunt amplifier output
  #define maxenvloops 5          // maximum loop counter to trigger envirinment check
  #define numcommands 13
  #define commandsize 10
  #define paramsize 8
  #define normmodefanoff 10
  #define normmodefanon 11
  #define debugmode 5
  #define alertmode 2
  #define critmode 1
  
  // variables to hold the data from pi
  const byte messagefrompisize = 44;
  char receivedChars[messagefrompisize];
  char tempChars[messagefrompisize]; // temporary array for use when parsing
  char intcommand[messagefrompisize]; // array to hold internal command
  char messageFromPi[messagefrompisize] = {0};
  char param1FromPi[messagefrompisize] = {0};
  char param2FromPi[messagefrompisize] = {0};
  char param3FromPi[messagefrompisize] = {0};
  char param4FromPi[messagefrompisize] = {0};
  char param5FromPi[messagefrompisize] = {0};
  char param6FromPi[messagefrompisize] = {0};
  boolean newDataFromPi = false;
char RGBmodenormfanoff[messagefrompisize] = "writeRGB,0,50,0" ;   // Set normal mode with fan off to green
  char RGBmodenormfanon[messagefrompisize] = "writeRGB,50,20,20" ;   // Set normal mode with fan on to white
  char RGBmodedebug[messagefrompisize] = "writeRGB,0,50,20" ; // Set debug mode to cyan
  char RGBmodealert[messagefrompisize] = "writeRGB,50,0,50" ; // Set alert mode to magenta
  char RGBmodecrit[messagefrompisize] = "writeRGB,50,0,0" ;  // Set critical mode to red
  char RGBmodeboot[messagefrompisize] = "writeRGB,0,0,50" ;  // Set boot mode to blue  
  char RGBoff[messagefrompisize] = "writeRGB,0,0,0" ;        // Set RGB led to off 

  bool toolong;
  int pinnum;
  int pinmax;
  int pwmpin;
  int pwmduty;
  int RGBduty[3];
  int RGBpin[3];
  int anavalue;
  float voltage;
  float current;
  boolean internalcommand = false;
  boolean commandok;

  // variables for commands
  
  const char* commands[]={"noop", "reset", "gettemp", "gethum",
                          "getbattV", "getbattI", "pinon", "pinoff",
                          "readpin", "readvolt", "writepwm", "writeRGB",
                          "debug"};
  
  char commandFromPi[messagefrompisize];
  char commandtestname[commandsize];
  long loopcounter = 1L;
  long starcounter = 1L;
  bool foundit = false;
  int commandid;

  // Temp and Hum variables
  byte thdat [5];
  byte thdata;
  float temp;
  float hum;  

  // Alarm and power light variables
  boolean alarmon = false;
  byte modeon = normmodefanoff;
  float alarmalertofftime = 2.0;
  float alarmalertontime = 0.05;
  float alarmcritofftime = 0.5;
  float alarmcritontime = 0.5;
  float RGBledofftime = 0.9;
  float RGBledontime = 0.1;
  
  // Environment variables
  long envloopcounter = 10L;
  float envtemp;
  float envhum;  
  float envbattV;
  float envbattI;
  boolean fanon;
  boolean buzzeron = true;

  // Miscellaneous variables
  int i;
  int j;
  boolean debug = false;
  boolean deepdebug = false;
  boolean optemphum = false;

//=====================================================================
//==== S E T U P ======================================================
//=====================================================================

void setup() {
  pinMode(SDApin,INPUT);
  Wire.begin();
  Serial.begin(19200);
  Serial2.begin(19200);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  if (debug) Serial.print("Booting...\n");
  
  // initialise fan
  pinMode(fanpin, OUTPUT);
  digitalWrite(fanpin, HIGH);
  fanon = true;

  // Boot notify
  strcpy(intcommand, RGBmodeboot);
  actioninternalcommand();
  if (buzzeron) {
    pinMode(buzzpin, OUTPUT);
    digitalWrite(buzzpin, HIGH);
    delay(250);
    digitalWrite(buzzpin, LOW);
    delay(250);
    digitalWrite(buzzpin, HIGH);
    delay(250);
    digitalWrite(buzzpin, LOW);
    delay(250);
    digitalWrite(buzzpin, HIGH);
  }
  delay(500); // Pause to show boot light and sound buzzer
  
  strcpy(intcommand, RGBoff);
  actioninternalcommand();
  if (buzzeron) {
    digitalWrite(buzzpin, LOW);
  }
  if (debug || optemphum) {
    Serial.print("Arduino booted!\n");
    get3231Date();
    Serial.print(weekDay); Serial.print(", "); Serial.print(date, DEC); Serial.print("/"); Serial.print(month, DEC); Serial.print("/"); Serial.print(year, DEC); Serial.print(" - ");
    Serial.print(hours, DEC); Serial.print(":"); Serial.print(minutes, DEC); Serial.print(":"); Serial.println(seconds, DEC);
  }
  modeon = normmodefanon;
}

//=====================================================================
//==== M A I N  P R O G R A M =========================================
//=====================================================================

void loop() {
  if (debug){
    for (j=0;j<40;j++) Serial.print("=");
    Serial.print ("\nMain loop: ");
    Serial.println (loopcounter);
    Serial2.println (loopcounter);
  }
  getpicommand(); // go and get a command and parameters from input stream
                  // or from an internally generated command

  // Action Command
  actioncommand();
  loopcounter ++;
  starcounter ++;
  envloopcounter ++; // envloopcounter used to trigger environment update

//---------------------------------------------------------------------
// check Environment and take action if needed

  if (envloopcounter > maxenvloops){
    envloopcounter = 1;
    readEnvironment(); // update environmental data
    get3231Temp(); // update RTC temperature
    if (debug) {
      Serial.print("Environment:");
      Serial.print("Temp = ");
      Serial2.print(temp3231);
      if (fanon){
        Serial2.println(" fan on");
      }
      else {
        Serial2.println(" fan off");
      }
      Serial.print(" / Humidity = ");
      Serial.println(envhum);
      Serial.print("Battery:");
      Serial.print("Voltage = ");
      Serial.print(envbattV);
      Serial.print(" / Current = ");
      Serial.println(envbattI);
    }
    if (optemphum) {
      Serial.print(envtemp);
      Serial.print(" ");
      Serial.print(temp3231);
      Serial.print(" ");
      Serial.print(envhum);
      Serial.print(" ");
      Serial.print(envbattV);
      Serial.print(" ");
      Serial.println(envbattI);
    }
      
    // test envronmental variables against standards
    
    // Temperature
    if (fanon){
      if (temp3231 <= tempoff) {
        // system has cooled so turn fan off
        digitalWrite(fanpin, LOW);
        fanon = false;
        modeon=normmodefanoff;
      }
    }
    else {
      if (temp3231 >= tempon) {
        // system is hot so turn fan on
        digitalWrite(fanpin, HIGH);
        fanon = true;
        modeon=normmodefanon;
      }
    }

    // Battery under
    if (envbattV < battVunder) {
      // Battery is under voltage
    }

    // Battery over
    if (envbattV > battVover) {
      // Battery is over voltage
    }
  }

//---------------------------------------------------------------------
// Deal with alarm and led indicator

  switch (modeon) {
    case 1:
    // critical mode  
      if (buzzeron) {
        digitalWrite(buzzpin, HIGH);
      }
      strcpy(intcommand, RGBmodecrit);
      actioninternalcommand();
      delay(alarmcritontime*1000);
      strcpy(intcommand, RGBoff);
      actioninternalcommand();
      if (buzzeron) {
        digitalWrite(buzzpin, LOW);
      }
      delay(alarmcritofftime*1000);
      break;
    case 2:
    // alert mode  
      if (buzzeron) {
        digitalWrite(buzzpin, HIGH);
      }
      strcpy(intcommand, RGBmodealert);
      actioninternalcommand();
      delay(alarmalertontime*1000);
      if (buzzeron) {
        digitalWrite(buzzpin, LOW);
      }
      strcpy(intcommand, RGBoff);
      actioninternalcommand();
      delay(alarmalertofftime*1000);
      break;
    case 5:
    // debug mode  
      strcpy(intcommand, RGBmodedebug);
      actioninternalcommand();
      delay(RGBledontime*1000);
      strcpy(intcommand, RGBoff);
      actioninternalcommand();
      delay(RGBledofftime*1000);
      break;
    case 11:
    // normal mode with fan on
      strcpy(intcommand, RGBmodenormfanon);
      actioninternalcommand();
      delay(RGBledontime*1000);
      strcpy(intcommand, RGBoff);
      actioninternalcommand();
      delay(RGBledofftime*1000);
      break;

    default:
    // must be normal mode with fan off
      strcpy(intcommand, RGBmodenormfanoff);
      actioninternalcommand();
      delay(RGBledontime*1000);
      strcpy(intcommand, RGBoff);
      actioninternalcommand();
      delay(RGBledofftime*1000);
  }

//---------------------------------------------------------------------

  if (starcounter >= 50){
    starcounter = 1;
    if (debug) Serial.println("");
  }
  if (debug){
    delay(1500); // long delay to keep output from scrolling quickly
  }
  else {
    delay(100);
  }
  wdt_reset();
}

//=====================================================================
//= S U B R O U T I N E S =============================================
//=====================================================================

// Action Command

void actioncommand() {

  if (deepdebug){
    Serial.print("About to do a ");
    Serial.println(commands[commandid]);
  }
  commandok = true;

  switch (commandid) {
    
//---------------------------------------------------------------------
    case 1:
      // Reset
      if (deepdebug) Serial.print("\nExecuting reset ");
      resetArduino();
      break;

//---------------------------------------------------------------------
    case 2:
      // Get Temperature
      pinnum = atoi(param1FromPi);
      if (pinnum == 0){
        pinnum = DHTpin;
      }
      if (deepdebug){
        Serial.print("\nExecuting gettemp ");
      }
      if (valid_dpin(pinnum)){
        read_thdata ();
        Serial2.println(temp);
      }
      else {
        commandok=false;
      }
      break;

//---------------------------------------------------------------------
    case 3:
      // Get Humidity
      pinnum = atoi(param1FromPi);
      if (pinnum == 0){
        pinnum = DHTpin;
      }
      if (deepdebug){
        Serial.print("\nExecuting gethum ");
      }
//---------------------------------------------------------------------
    case 4:
      // Read battery voltage
      pinnum = atoi(param1FromPi);
      if (pinnum == 0){
        pinnum = battVpin;
      }
      if (deepdebug){
        Serial.print("\nExecuting getbattV ");
      }
      if (valid_apin(pinnum)){
        readbattV();
        Serial2.println(voltage);
      }
      else {
        commandok=false;
      }
      break;
    
//---------------------------------------------------------------------
    case 5:
      // Read battery current
      pinnum = atoi(param1FromPi);
      if (pinnum == 0){
        pinnum = battIpin;
      }
      if (deepdebug){
        Serial.print("\nExecuting getbattI ");
      }
      if (valid_apin(pinnum)){
        readbattI();
        Serial2.println(current);
      }
      else {
        commandok=false;
      }
      break;
    
//---------------------------------------------------------------------
    case 6:
      // Turn digital pin on (set high)
      pinnum = atoi(param1FromPi);
      if (deepdebug){
        Serial.print("\nExecuting getbattI ");
      }
      if (valid_dpin(pinnum)){
        pinMode(pinnum, OUTPUT);
        digitalWrite(pinnum, HIGH);
      }
      else {
        commandok=false;
      }
      break;
    
//---------------------------------------------------------------------
    case 7:
      // Turn digital pin off (set low)
      pinnum = atoi(param1FromPi);
      if (deepdebug){
        Serial.print("\nExecuting pinoff ");
      }
      if (valid_dpin(pinnum)){
        pinMode(pinnum, OUTPUT);
        digitalWrite(pinnum, LOW);
      }
      else {
        commandok=false;
      }
      break;
      
//---------------------------------------------------------------------
    case 8:
      // Read digital pin (0 = low, 1 = high)
      pinnum = atoi(param1FromPi);
      if (deepdebug){
        Serial.print("\nExecuting readpin ");
      }
      if (valid_dpin(pinnum)){
        pinMode(pinnum, INPUT);
        if (digitalRead(pinnum)){
          Serial2.println("1");
        } else {
          Serial2.println("0");
        }
      }
      else {
        commandok=false;
      }
      break;
      
//---------------------------------------------------------------------
    case 9:
      // Read voltage (does not work with pins 0 or 1)
      pinnum = atoi(param1FromPi);
      pinmax = atoi(param2FromPi);
      if (pinmax==0) pinmax=maxscale;
      if (deepdebug){
        Serial.print("\nExecuting readvolt ");
      }
      if (valid_apin(pinnum)){
        voltage = anavalue*pinmax/1024.0;      
        Serial2.println(voltage);
      }
      else {
        commandok=false;
      }
      break;
    
//---------------------------------------------------------------------
    case 10:
      // set pin to pwm output (does not work with pins 0 or 1)
      pinnum = atoi(param1FromPi);
      pwmduty = atoi(param2FromPi);
      if (deepdebug){
        Serial.print("\nExecuting writepwm ");
      }
      setpwmpin(pinnum,pwmduty);
      break;
    
//---------------------------------------------------------------------
    case 11:
      // set rgb led output
      RGBduty[0] = atoi(param1FromPi);
      RGBduty[1] = atoi(param2FromPi);
      RGBduty[2] = atoi(param3FromPi);
      RGBpin[0] = atoi(param4FromPi);
      if (RGBpin[0] == 0) RGBpin[0] = RGBredpin;
      RGBpin[1] = atoi(param5FromPi);
      if (RGBpin[1] == 0) RGBpin[1] = RGBgreenpin;
      RGBpin[2] = atoi(param6FromPi);
      if (RGBpin[2] == 0) RGBpin[2] = RGBbluepin;
      if (deepdebug){
        Serial.print("\nExecuting writeRGB ");
        Serial.print(pinnum);
      }
      for (i = 0; i < 3; i++) {
        setpwmpin(RGBpin[i],RGBduty[i]);
      }
      break;
    
//---------------------------------------------------------------------
    case 12:
      // set debug to output diagnostics or not via tx0
      if (atoi(param1FromPi) == 0 && debug){
        Serial.println("\nExecuting debug off");
        debug = false;
        if (fanon){
          modeon = normmodefanon;
        }
        else {
          modeon = normmodefanoff;
        }
      }
      else {
        debug = true;
        modeon = debugmode;
        Serial.println("\nExecuting debug on");
      }
    break;

//---------------------------------------------------------------------
    default: 
      // if Null operation or nothing else matches, default is do nothing
    break;
  }
  if (!internalcommand){
      if (valid_dpin(pinnum)){
        read_thdata ();
        Serial2.println(hum);
      }
      else {
        commandok=false;
      }
    break;
    ){
    Serial.print("Got here with");
    Serial.print(" commandid=");
    Serial.print(commandid);
    Serial.print(" commandok=");
    Serial.print(commandok);
    Serial.print(" internalcommand=");
    Serial.print(internalcommand);
    Serial.println("");
  } 
  if (commandid!=0){
    if (commandok) {
      if (!internalcommand) {
        Serial2.println("OK");
      }
      if (debug) Serial.println("");  
    }
    else {
      Serial2.println("NO");
      if (debug) Serial.println(" INVALID");
    }
    commandid=0;
  }
}
//=====================================================================
// Action internal command

void actioninternalcommand() {
  internalcommand = true;
  getpicommand();
  actioncommand();
  internalcommand = false;
}
//=====================================================================
// Setup Watchdog timer

void watchdogSetup() {
  cli(); // disable all interrupts
  wdt_reset(); // reset the WDT timer
/*
  WDTCSR configuration:
  WDIE = 1: Interrupt Enable
  WDE = 1 :Reset Enable
  WDP3 = 1 :For 8000ms Time-out
  WDP2 = 0 :For 8000ms Time-out
  WDP1 = 0 :For 8000ms Time-out
  WDP0 = 1 :For 8000ms Time-out
*/
  // Enter Watchdog Configuration mode:
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // Set Watchdog settings:
  WDTCSR = (1<<WDIE) | (1<<WDE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);
  sei();
}

//=====================================================================
// read environmental variables

void readEnvironment() {
  // Temperature and Humidity
/* Code removed until DHT reinstalled
        pinnum = DHTpin;
        read_thdata();
        envtemp=temp;
        envhum=hum;
        Also envtemp & envhum set to zero
*/
  envtemp=0;
  envhum=0;
  // Battery voltage and current
  pinnum = battVpin;
  readbattV();
  envbattV = voltage;
  pinnum = battIpin;
  readbattI();
  envbattI = current;
}

//=====================================================================
// Read voltage on pinnum

void readbattV(){
  anavalue = analogRead(pinnum);
  voltage = anavalue*battcorrect*battVmax/1024;      
}

//=====================================================================
// Read current on pinnum

void readbattI(){
  anavalue = analogRead(pinnum);
  current = anavalue*currentcorrect*battImax/1024;      
}

//=====================================================================
// Reset Arduino Mega 2560 via hardware

void resetArduino(){
  pinMode(resetpin,OUTPUT);
  digitalWrite (resetpin, LOW);
  
}

/*=====================================================================
  Make sure Arduino Mega 2560 digital pin is valid
  
  pins 0 and 1 reserved for serial data port 0
  pins 2,3,18 and 19 reserved for interrupts
  pins 16 and 17 reserved for serial data port 2
  pins 20 and 21 reserved for I2C devices
  pin 22 reserved for hardware reset
  pins above 53 (54-69) used as analogue 0 to 15

*/
boolean valid_dpin(int pinid) {
  if ((pinid>=4 && pinid<=15) ||  (pinid>=23 && pinid<=53)){
    return true;
  }
  else {
    return false;
  }
  
}

/*=====================================================================
  Make sure Arduino Mega 2560 pwm pin is valid
  
  pins 0 and 1 reserved for serial data port 0
  pins 2 and 3 reserved for interrupts
  pins above 13 (14-69) not used for pwm

*/
boolean valid_ppin(int pinid) {
  if (pinid>=4 && pinid<=13){
    return true;
  }
  else {
    return false;
  }
}
//=====================================================================
/* Make sure Arduino Mega 2560 analogue pin is valid

  pin 1 reserved for main battery voltage via voltage divider
  pin 2 reserved for main battery current via shunt amplifier
  pin 3 reserved for Solar Panel output voltage
  pin 4 reserved for Solar Panel output current
  pin 5 reserved for Vehicle output voltage
  pin 6 reserved for Vehicle output current
  pin 7 reserved for Redarc BDDC1240D output current
  pin 8 reserved for Wind generator output current
  pin 9 reserved for Enerdrive 240volt charger output current
  pin 10 reserved for Watertank 1 (front) level
  pin 11 reserved for Watertank 2 (rear) level
  pin 12 reserved for Grey watertank level
  
*/
boolean valid_apin(int pinid) {
  if (pinid<=15){
    return true;
  }
  else {
    return false;
  }
  
}

//=====================================================================

void getpicommand() {
  // Receive input string
  commandid=0;
  if(internalcommand) {
    newDataFromPi = true;
    strcpy(receivedChars, intcommand);
  }
  else {
    recvWithStartEndMarkers();
  }
  if (newDataFromPi) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    // because strtok() used in parseData() replaces the commas with \0
    parseData();
    if (debug){
      showParsedData();
    }
    // Search for command
    if (deepdebug) Serial.println("Comparing command against valid commands");
    for (i = 0; i < numcommands; i++) {
      if (deepdebug) {
        Serial.print("Comparing input=(");
        Serial.print (commandFromPi);
        Serial.print(") and commands[");
        Serial.print(i);
        Serial.print("] (");
        Serial.print(commands[i]); 
        Serial.print(") Result: ");
      }
      if (strcmp(commandFromPi, commands[i])==0) {
        commandid = i;
        if (deepdebug){
          Serial.println("True");
          Serial.print("Found command ");
          Serial.print(commandid);
          Serial.print(" ");
          Serial.println(commands[commandid]);
        }
        break;
      }
      else {
        if (deepdebug)Serial.println("False");     
      }
    }
    if(deepdebug){
      for (j=0;j<68;j++) Serial.print("-");
      Serial.println("");
    }
    newDataFromPi = false;
  }
}
  
//=====================================================================

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial2.available() > 0 && newDataFromPi == false) {
        rc = Serial2.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= messagefrompisize) {
                    ndx = messagefrompisize - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newDataFromPi = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//=====================================================================

void parseData() {      // split the data into its parts

    toolong = false;
    char * strtokIndx; // this is used by strtok() as an index
    
    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    strcpy(commandFromPi, strtokIndx); // copy it to commandFromPi

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    strcpy(param1FromPi, strtokIndx); // copy first parameter

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    strcpy(param2FromPi, strtokIndx); // copy second parameter

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    strcpy(param3FromPi, strtokIndx); // copy third parameter

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    strcpy(param4FromPi, strtokIndx); // copy first parameter

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    strcpy(param5FromPi, strtokIndx); // copy second parameter

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    strcpy(param6FromPi, strtokIndx); // copy third parameter

    // Reject if the command or any parameter is too long
    if (toolong){
      commandFromPi[0]=0;
      param1FromPi[0]=0;
      param2FromPi[0]=0;
      param3FromPi[0]=0;
      param4FromPi[0]=0;
      param5FromPi[0]=0;
      param6FromPi[0]=0;
    }
}

//=====================================================================

void showParsedData() {
  if(deepdebug) {
    for (j=0;j<68;j++) Serial.print("-");
    Serial.println("\nParsing input");
    Serial.print("Command    :(");
    Serial.print(commandFromPi);
    Serial.println(")");
    Serial.print("Parameter 1:(");
    Serial.print(param1FromPi);
    Serial.println(")");
    Serial.print("Parameter 2:(");
    Serial.print(param2FromPi);
    Serial.println(")");
    Serial.print("Parameter 3:(");
    Serial.print(param3FromPi);
    Serial.println(")");
    Serial.print("Parameter 4:(");
    Serial.print(param4FromPi);
    Serial.println(")");
    Serial.print("Parameter 5:(");
    Serial.print(param5FromPi);
    Serial.println(")");
    Serial.print("Parameter 6:(");
    Serial.print(param6FromPi);
    Serial.println(")");
    Serial.print("Command (Hex) = ");
    for (i = 0; i < commandsize;i++) {
      Serial.print(commandFromPi[i], HEX);
      Serial.print(" ");
    }
  }
  else {
    if (internalcommand) {
      Serial.print("Internal");
    }
    else {
      Serial.print("External");
    }
    Serial.print(" Command: <");
    Serial.print(commandFromPi);
    Serial.print(",");
    Serial.print(param1FromPi);
    Serial.print(",");
    Serial.print(param2FromPi);
    Serial.print(",");
    Serial.print(param3FromPi);
    Serial.print(",");
    Serial.print(param4FromPi);
    Serial.print(",");
    Serial.print(param5FromPi);
    Serial.print(",");
    Serial.print(param6FromPi);
    Serial.println(">");
  }
}

//=====================================================================

byte read_thbyte () {
  thdata = 0;
  for (int i = 0; i < 8; i ++) {
    if (digitalRead (pinnum) == LOW) {
      while (digitalRead (pinnum) == LOW); // wait for 50us
      delayMicroseconds (30); // determine the duration of the high level to determine the data is '0 'or '1'
      if (digitalRead (pinnum) == HIGH)
        thdata |= (1 << (7-i)); // high front and low in the post
      while (digitalRead (pinnum) == HIGH); // data '1 ', wait for the next one receiver
     }
  }
return thdata;
}

//=====================================================================

void read_thdata () {
  pinMode (pinnum, OUTPUT);
  digitalWrite (pinnum, HIGH);
  delay(100); // let output mode settle
  digitalWrite (pinnum, LOW); // bus down, send start signal
  delay (30); // delay greater than 18ms, so DHT11 start signal can be detected
 
  digitalWrite (pinnum, HIGH);
  delayMicroseconds (20); // Wait for DHT11 response
 
  pinMode (pinnum, INPUT);
  while (digitalRead (pinnum) == HIGH);
  delayMicroseconds (80); // DHT11 response, pulled the bus 80us
  if (digitalRead (pinnum) == LOW) {
    delayMicroseconds (20); // DHT11 80us after the bus pulled to start sending data
  }
  while (digitalRead (pinnum) == HIGH); // wait for start data transmission
 // receive temperature and humidity data, the parity bit is not considered
  for (int i = 0; i < 5; i ++) {
    thdat[i] = read_thbyte ();
  }
  digitalWrite (pinnum, HIGH); // send data once after releasing the bus, wait for the host to open the next Start signal
  temp = thdat[2]+thdat[3]/10.0;
  hum = thdat[0]+thdat[1]/10.0;
  delay(1000); // delay to allow dht sensor to settle ready for next command
}

//=====================================================================

void setpwmpin(int pin,int duty){
  if (duty <0) duty = 0;
  if (duty >255) duty = 255;
  if (valid_ppin(pin) && commandok){
    analogWrite(pin,duty);
  }
  else {
    commandok=false;
  }
}

//=====================================================================

// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return ( (val/10*16) + (val%10) );
}

void watchConsole()
{
  if (Serial.available()) {      // Look for char in serial queue and process if found
    if (Serial.read() == 84) {      //If command = "T" Set Date
      set3231Date();
      get3231Date();
      Serial.println(" ");
    }
  }
}
 
void set3231Date()
{
//T(sec)(min)(hour)(dayOfWeek)(dayOfMonth)(month)(year)
//T(00-59)(00-59)(00-23)(1-7)(01-31)(01-12)(00-99)
//Example: 02-Feb-09 @ 19:57:11 for the 3rd day of the week -> T1157193020209
// T1124154091014
  seconds = (byte) ((Serial.read() - 48) * 10 + (Serial.read() - 48)); // Use of (byte) type casting and ascii math to achieve result.  
  minutes = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
  hours   = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
  day     = (byte) (Serial.read() - 48);
  date    = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
  month   = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
  year    = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
  Wire.beginTransmission(DS3231_I2C_addr);
  Wire.write(0x00);
  Wire.write(decToBcd(seconds));
  Wire.write(decToBcd(minutes));
  Wire.write(decToBcd(hours));
  Wire.write(decToBcd(day));
  Wire.write(decToBcd(date));
  Wire.write(decToBcd(month));
  Wire.write(decToBcd(year));
  Wire.endTransmission();
}


void get3231Date()
{
  // send request to receive data starting at register 0
  Wire.beginTransmission(DS3231_I2C_addr); // 104 is DS3231 device address
  Wire.write(0x00); // start at register 0
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_addr, 7); // request seven bytes

  if(Wire.available()) {
    seconds = Wire.read(); // get seconds
    minutes = Wire.read(); // get minutes
    hours   = Wire.read();   // get hours
    day     = Wire.read();
    date    = Wire.read();
    month   = Wire.read(); //temp month
    year    = Wire.read();
       
    seconds = (((seconds & B11110000)>>4)*10 + (seconds & B00001111)); // convert BCD to decimal
    minutes = (((minutes & B11110000)>>4)*10 + (minutes & B00001111)); // convert BCD to decimal
    hours   = (((hours & B00110000)>>4)*10 + (hours & B00001111)); // convert BCD to decimal (assume 24 hour mode)
    day     = (day & B00000111); // 1-7
    date    = (((date & B00110000)>>4)*10 + (date & B00001111)); // 1-31
    month   = (((month & B00010000)>>4)*10 + (month & B00001111)); //msb7 is century overflow
    year    = (((year & B11110000)>>4)*10 + (year & B00001111));
  }
  else {
    //oh noes, no data!
  }
 
  switch (day) {
    case 1:
      strcpy(weekDay, "Sun");
      break;
    case 2:
      strcpy(weekDay, "Mon");
      break;
    case 3:
      strcpy(weekDay, "Tue");
      break;
    case 4:
      strcpy(weekDay, "Wed");
      break;
    case 5:
      strcpy(weekDay, "Thu");
      break;
    case 6:
      strcpy(weekDay, "Fri");
      break;
    case 7:
      strcpy(weekDay, "Sat");
      break;
  }
}

float get3231Temp()
{
  //temp registers (11h-12h) get updated automatically every 64s
  Wire.beginTransmission(DS3231_I2C_addr);
  Wire.write(0x11);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_addr, 2);
 
  if(Wire.available()) {
    tMSB = Wire.read(); //2's complement int portion
    tLSB = Wire.read(); //fraction portion
   
    temp3231 = (tMSB & B01111111); //do 2's math on Tmsb
    temp3231 += ( (tLSB >> 6) * 0.25 ); //only care about bits 7 & 8
  }
  else {
    //oh noes, no data!
  }
   
  return temp3231;
}
