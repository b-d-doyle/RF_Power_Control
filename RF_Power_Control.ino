#include <Wire.h>
#include <AD9959.h>
//Version 0.02

/* This script controls the AD9959 frequency generator.
 * IMPORTANT: make sure your arduino is running its digital i/o pins at 3.3v!
 * Arduinos are 5v by default, and that is too high for the AD9959
 * 
 * INPUTS:
 * Send an instruction or series of instructions via serial connection
 * 57600 baud, no parity, 8 data bits, 1 stop bit
 * You can send a series of instructions in one line separated by commas.
 * Instructions are probably case sensitive.
 * 
 * To set channel voltage, phase or frequency:
 * [instruction],[channel],[value],...
 * Example command:
 * setVoltage,0,500,setPhase,0,0,setVoltage,1,500,setPhase,1,180
 * 
 * The AD9959 has channels 0 to 3. This controller interprets channel 4
 * as being a command to set all channels to the given value.
 * 
 * To pause or unpause:
 * Send "pause" or "resume" command. Example:
 * resume,setVoltage,4,0
 *    -or-
 * setVoltage,4,0,resume
 * Either command would turn control back on (if it were previously paused)
 * and turn off output of all channels (by setting amplitude to zero).
 * 
 * Another use of pause is to manually control output with a serial monitor, 
 * such as the one in the Arduino IDE (ctrl-shift-M). Example:
 * pause,setDirectVoltage,4,1023,setPhase,0,0,setPhase,1,180
 * 
 */


//-----------------------------------------------------------------
//-----------------------------------------------------------------
//DECLARATIONS

//General stuff
//--------------------
float version_no = 0.02;
bool  debug = false;
bool  pause = false; //Can pause active control during data collection.
bool  reset_adaptation = true;
int   cyclic_counter = 0;
//--------------------

//AD9959 stuff
//--------------------
class MyAD9959 : public AD9959<
  2,        //Reset pin (active = high)
  3,        //Chip Enable (active = low) (I think that makes it actually a Chip Select pin)
  4,        //I/O_UPDATE: Apply config changes (pulse high)
  25000000  //25MHz crystal (optional)
> {};
MyAD9959 dds;
bool needs_update = false;
//--------------------

//Serial control variables
//--------------------
bool   receive_channel;
bool   receive_voltage; //PID control target
bool   receive_phase;
bool   receive_frequency;
bool   receive_direct_voltage; //Directly sends voltage to AD9959
bool   receive_get;
MyAD9959::ChannelNum set_channel;
//--------------------

//AD830X stuff
//--------------------
int analog_rf_in_pin_0    = A0; //AD8307 voltage pin 1
int analog_rf_in_pin_1    = A1; //AD8307 voltage pin 2
int analog_rf_in_pin_2    = A2; //AD8302 relative phase
//int analog_rf_in_pin_3  = A3; //We may eventually want a second AD8302
int analog_rf_in_0        = 0;
int analog_rf_in_1        = 0;
int analog_rf_in_2        = 0;
//int analog_rf_in_3      = 0;
int min_phase             = 20;  //The lowest possible value from AD8302 averaging
int max_phase             = 580;
int avg_counter           = 256; // how many values to add
int avg_sum_shift         = 8;   // that is log_2_avg_counter
unsigned long avg_sum_0   = 0;
unsigned long avg_sum_1   = 0;
unsigned long avg_sum_2   = 0;
//unsigned long avg_sum_3 = 0;
//--------------------

//PID voltage stuff
//--------------------
int v0        = 0;      //|~V|, current voltage (currently in arbitrary AD8307 units)
int v1        = 0;
float target0 = 0.0;    //|~V|, the target voltage (currently in arbitrary AD8307 units)
float target1 = 0.0;
float er0     = 0.0;    //|~V|, "error," targetV - currentV
float er1     = 0.0;
float i0      = 0.0;    //|~V|, time integral of error
float i1      = 0.0;
float old_er0 = 0.0;    //|~V|, last step's error
float old_er1 = 0.0;
float d_er0   = 0.0;    //|~V/step|, "delta error," change in error from last step
float d_er1   = 0.0;
float kp0     = 1.0;    //empirical factor controlling how much to react to er
float kp1     = 1.0;
float ki0     = 0.1;    //empirical factor controlling how much to react to integral of er
float ki1     = 0.1;
float kd0     =-0.2;    //empirical factor controlling how much to react to d_er
float kd1     =-0.2;    //For the moment, let's set it to zero and not use dv/dt at all.
float cv0     = 0.0;    //"control variable," the value by which we change our ad9959 amplitude
float cv1     = 0.0;
int setPoint0 = 1023;   //What to tell the AD9959 to set the amplitude to
int setPoint1 = 1023;
float m0      = 2.0;    //The tolerance; how close do we have to be to be considered matched?
float m1      = 2.0;
//--------------------

//PID phase stuff
//--------------------
bool  neg        = false;
int   v2         = 0;
float target2    = (float)min_phase;
float er2        = 0.0;
float i2         = 0.0;
float old_er2    = 0.0;
float d_er2      = 0.0;
float kp2        = 10.0;
float ki2        = 0.0;
float kd2        =-5.0;
float cv2        = 0.0;
float old_cv2    = 0.0;
int   phasePoint = 8192;
long  running_sum_2 = min_phase*16;
int   log2[16]      = {min_phase,min_phase,min_phase,min_phase,min_phase,min_phase,min_phase,min_phase,min_phase,min_phase,min_phase,min_phase,min_phase,min_phase,min_phase,min_phase};
int   index2        = 0;
float m2         = 2.0;   //The tolerance; how close do we have to be to be considered matched?
//--------------------

//PID matching stuff
//--------------------
bool   matched       = false; //if they're all matched
bool   matched_0     = false; //channel 0 amplitude
bool   matched_1     = false; //channel 1 amplitude
bool   matched_phase = false; //channel 0 and 1 phase difference
unsigned long matchTime; //The time when we match. To know if we're stable yet.
unsigned long now;
//--------------------

//-----------------------------------------------------------------
//-----------------------------------------------------------------
//METHODS

void helpMessage(){
  /*
  //Defining it like this is supposed to reduce memory usage, but it doesn't work...
  const char help_msg[] PROGMEM = {
    "PID controller for voltage and relative phase of two channels.\n"
    "For use with Brandon's COPLA library, arduino_power version 0.01\n\n"
    "Commands:\n"
    "debugOn  -- turns debug mode on\n"
    "debugOff -- turns debug mode off\n"
    "pause    -- pauses PID control\n"
    "resume   -- resumes PID control\n"
    "setVoltage,[channel],[value]\n"
    "setFrequency,[channel],[value]\n"
    "setPhase,[channel],[value]\n"
    "setDirectVoltage,[channel],[value]\n"
    "  Note:[channel] may be 0-3 for a\n"
    "  specific channel, or 4 for all channels\n"
    "get,[value]\n"
    "Use 'get,help' for more help on get\n"
  };*/
  Serial.println("RF-Power-Controller for AD9959, AD8307, and AD8302");
  Serial.print("Version: ");
  Serial.println(version_no,2);
  Serial.println("PID controller for voltage and relative phase of two channels.");
  Serial.println("For use with Brandon's COPLA library, arduino_power version 0.01");
  Serial.println(" ");
  Serial.println("Commands:");
  Serial.println("debugOn  -- turns debug mode on");
  Serial.println("debugOff -- turns debug mode off");
  Serial.println("pause    -- pauses PID control");
  Serial.println("resume   -- resumes PID control");
  Serial.println("setVoltage,[channel],[value]");
  Serial.println("setFrequency,[channel],[value]");
  Serial.println("setPhase,[channel],[value]");
  Serial.println("setDirectVoltage,[channel],[value]");
  Serial.println("  Note:[channel] may be 0-3 for a");
  Serial.println("  specific channel, or 4 for all channels");
  Serial.println("get,[value]");
  Serial.println("Use 'get,help' for more help on get");
}

void(* reset) (void) = 0; //Resets the arduino, as if with reset button.

void clearInputs(){
  receive_channel        = 0;
  receive_voltage        = 0;
  receive_phase          = 0;
  receive_frequency      = 0;
  receive_direct_voltage = 0;
}

void badCommand(){
  clearInputs();
  Serial.println("Bad command. Ignored.");
}

void pauseWarning(){
  Serial.println("Paused. To resume, send 'resume'.");
}

void getMatch(){
  now = millis();
  if(matched && now-matchTime >= 500) Serial.println("true");
  else Serial.println("false");
}

void handleGet(String input){
  if (input == (String)"help"){
    Serial.println("Help message for get. (TODO)");
    Serial.println();
    Serial.println("Options:");
    Serial.println("match");
    Serial.println("v0");
    Serial.println("v1");
    Serial.println("setPoint0");
    Serial.println("setPoint1");
    Serial.println("amp0");
    Serial.println("amp1");
    Serial.println("phase0");
    Serial.println("phase1");
    Serial.println("freq0");
    Serial.println("freq1");
  }
  else if (input == (String)"match") getMatch();
  else if (input == (String)"v0") Serial.println(v0);
  else if (input == (String)"v1") Serial.println(v1);
  else if (input == (String)"setPoint0") Serial.println(setPoint0);
  else if (input == (String)"setPoint1") Serial.println(setPoint1);
  else if (input == (String)"amp0"){
    dds.setChannels(MyAD9959::Channel0);
    uint32_t response = dds.read(MyAD9959::ACR);
    Serial.println(response&0x03FF);
  }
  else if (input == (String)"amp1"){
    dds.setChannels(MyAD9959::Channel1);
    uint32_t response = dds.read(MyAD9959::ACR);
    Serial.println(response&0x03FF);
  }
  else if (input == (String)"amp2"){
    dds.setChannels(MyAD9959::Channel2);
    uint32_t response = dds.read(MyAD9959::ACR);
    Serial.println(response&0x03FF);
  }
  else if (input == (String)"amp3"){
    dds.setChannels(MyAD9959::Channel3);
    uint32_t response = dds.read(MyAD9959::ACR);
    Serial.println(response&0x03FF);
  }
  else if (input == (String)"phase0"){
    dds.setChannels(MyAD9959::Channel0);
    uint32_t response = dds.read(MyAD9959::CPOW);
    Serial.println((double)response*360/16383);
  }
  else if (input == (String)"phase1"){
    dds.setChannels(MyAD9959::Channel1);
    uint32_t response = dds.read(MyAD9959::CPOW);
    Serial.println((double)response*360/16384);
  }
  else if (input == (String)"phase2"){
    dds.setChannels(MyAD9959::Channel2);
    uint32_t response = dds.read(MyAD9959::CPOW);
    Serial.println((double)response*360/16383);
  }
  else if (input == (String)"phase3"){
    dds.setChannels(MyAD9959::Channel3);
    uint32_t response = dds.read(MyAD9959::CPOW);
    Serial.println((double)response*360/16384);
  }
  else if (input == (String)"freq0"){
    dds.setChannels(MyAD9959::Channel0);
    uint32_t response = dds.read(MyAD9959::CFTW);
    Serial.println((uint32_t)((((uint64_t)response<<dds.get_shift())-dds.get_reciprocal()/16)/dds.get_reciprocal()));
  }
  else if (input == (String)"freq1"){
    dds.setChannels(MyAD9959::Channel1);
    uint32_t response = dds.read(MyAD9959::CFTW);
    Serial.println((uint32_t)((((uint64_t)response<<dds.get_shift())-dds.get_reciprocal()/16)/dds.get_reciprocal()));
  }
  else if (input == (String)"freq2"){
    dds.setChannels(MyAD9959::Channel2);
    uint32_t response = dds.read(MyAD9959::CFTW);
    Serial.println((uint32_t)((((uint64_t)response<<dds.get_shift())-dds.get_reciprocal()/16)/dds.get_reciprocal()));
  }
  else if (input == (String)"freq3"){
    dds.setChannels(MyAD9959::Channel3);
    uint32_t response = dds.read(MyAD9959::CFTW);
    Serial.println((uint32_t)((((uint64_t)response<<dds.get_shift())-dds.get_reciprocal()/16)/dds.get_reciprocal()));
  }
  else badCommand();
}

void setChannel(int ch){
  switch (ch){
    case 0:
      set_channel = MyAD9959::Channel0;
      break;
    case 1:
      set_channel = MyAD9959::Channel1;
      break;
    case 2:
      set_channel = MyAD9959::Channel2;
      break;
    case 3:
      set_channel = MyAD9959::Channel3;
      break;
    case 4:
      set_channel = MyAD9959::ChannelAll;
      break;
    default:
      badCommand();
      break;
  }
}

void handleSerialInputs(){
  bool received_command = false;
  bool warn_pause = false;
  needs_update = false;
  if(pause) warn_pause = true; //If we're paused before a command is received
  while(Serial.available()>0)
  {
    received_command = true;
    String input = Serial.readStringUntil(',');
    input.replace("\r","\0");
    input.replace("\n","\0");
    if(debug) Serial.println(input);
    if (receive_channel)        //if we expect a channel number
    {
      setChannel(input.toInt());
      receive_channel=false;
    }
    else if (receive_voltage)   //if we expect a voltage
    {
      if      (set_channel==MyAD9959::Channel0)   target0 = input.toFloat();
      else if (set_channel==MyAD9959::Channel1)   target1 = input.toFloat();
      else if (set_channel==MyAD9959::ChannelAll) target0 = target1 = input.toFloat();
      //reset_adaptation = true; //Why was I resetting here again? Let's comment that out.
      receive_voltage  = false;
      Serial.println("ok");
    }
    else if (receive_phase)     //if we expect a phase
    {
      if(debug) Serial.println("Setting direct phase.");
      dds.setPhase(set_channel,(int)(input.toFloat()*16384/360));
      reset_adaptation = true;
      receive_phase    = false;
      needs_update     = true;
      Serial.println("ok");
    }
    else if (receive_frequency) //if we expect a frequency
    {
      if(debug) Serial.println("Setting frequency.");
      dds.setFrequency(set_channel,input.toInt());
      receive_frequency = false;
      needs_update      = true;
      Serial.println("ok");
    }
    else if (receive_direct_voltage) //if we expect a direct voltage
    {
      int temp = input.toInt();
      if(debug) Serial.println("Setting direct voltage output.");
      if(0 <= temp && temp <= 1023){
        dds.setAmplitude(set_channel,temp);
        needs_update = true;
        if      (set_channel==MyAD9959::Channel0)   setPoint0 = temp;
        else if (set_channel==MyAD9959::Channel1)   setPoint1 = temp;
        else if (set_channel==MyAD9959::ChannelAll) setPoint0 = setPoint1 = temp;
      }
      else{
        Serial.println("Value out of bounds. 0->1023");
        badCommand();
      }
      receive_direct_voltage = false;
      Serial.println("ok");
    }
    else if (receive_get)
    {
      receive_get = false;
      handleGet(input);
    }
    else                           //if we don't know what command to expect
    {
      if      (input.indexOf("setVoltage")>=0)       receive_channel = receive_voltage        = true;
      else if (input.indexOf("setPhase")>=0)         receive_channel = receive_phase          = true;
      else if (input.indexOf("setFrequency")>=0)     receive_channel = receive_frequency      = true;
      else if (input.indexOf("setDirectVoltage")>=0) receive_channel = receive_direct_voltage = true;
      else if (input.indexOf("pause")>=0){
        pause = true;
        Serial.println("ok");
      }
      else if (input.indexOf("resume")>=0){
        pause = warn_pause = false;
        //reset_adaptation = true;
        Serial.println("ok");
      }
      else if (input.indexOf("debugOn")>=0)      debug = true;
      else if (input.indexOf("debugOff")>=0)     debug = false;
      else if (input.indexOf("help")>=0)         helpMessage();
      else if (input.indexOf("getMatch")>=0)     getMatch(); //keeping this for legacy support. MUST COME ABOVE "get"
      else if (input.indexOf("get")>=0)          receive_get = true;
      else if (input.indexOf("reset")>=0){
        Serial.println("ok");
        delay(10);
        reset();
      }
      else badCommand();
    }
  }
  if(received_command && warn_pause) pauseWarning(); //If we received a command AND started paused but did not resume, send warning
  if(needs_update){
    dds.update();
    needs_update = false;
  }
}

void handleAnalogInputs(){
  //Get v^2 from AD8307 (still need to calibrate to volts)
  avg_sum_0 = 0;
  avg_sum_1 = 0;
  avg_sum_2 = 0;
  //avg_sum_3 = 0;
  for (int i=0; i<avg_counter; i++) {
    analog_rf_in_0 = analogRead(analog_rf_in_pin_0);
    analog_rf_in_1 = analogRead(analog_rf_in_pin_1);
    analog_rf_in_2 = analogRead(analog_rf_in_pin_2);
    //analog_rf_in_3 = analogRead(analog_rf_in_pin_3);
    avg_sum_0 += (unsigned long) analog_rf_in_0;
    avg_sum_1 += (unsigned long) analog_rf_in_1;
    avg_sum_2 += (unsigned long) analog_rf_in_2;
    //avg_sum_3 += (unsigned long) analog_rf_in_3;
  }
  v0 = avg_sum_0>>avg_sum_shift;
  v1 = avg_sum_1>>avg_sum_shift;
  v2 = avg_sum_2>>avg_sum_shift;
  //v3 = avg_sum_3>>avg_sum_shift;

  running_sum_2+=v2-log2[index2];
  log2[index2]=v2;
  index2 = (index2+1) & 0x0f;
  
  //Make sure our phase limits are still ok
  if(v2>max_phase) max_phase=v2; //the new upper limit
  if(v2<min_phase) min_phase=v2; //the new lower limit

}

void PID(){
  //PD controller for voltage
  er0 = target0 - v0;            //How far from target?
  er1 = target1 - v1;
  d_er0 = er0 - old_er0;         //How fast approaching/leaving target?
  d_er1 = er1 - old_er1;
  if(abs(er0)<5.0 && abs(er0)!=0.0) i0+=er0;      //We only use the integral part if we're pretty close to the value we want
  else i0=0;
  if(abs(er1)<5.0 && abs(er0)!=0.0) i1+=er1;
  else i1=0;
  cv0 = (kp0*er0) + (ki0*i0) + (kd0*d_er0); //Control variable proportional to er and d_er
  cv1 = (kp1*er1) + (ki1*i1) + (kd1*d_er1);

  if((setPoint0+(int)cv0)>1023) setPoint0=1023; //max
  else if((setPoint0+(int)cv0)<0) setPoint0=0;  //min
  else setPoint0+=(int)cv0;
  
  if((setPoint1+(int)cv1)>1023) setPoint1=1023; //max
  else if((setPoint1+(int)cv1)<0) setPoint1=0;  //min
  else setPoint1+=(int)cv1;
  
  //PID controller for phase
  if(reset_adaptation){
    target2=(float)min_phase;
    running_sum_2=v2*16;
    for(int i=0;i<16;i++) log2[i]=v2;
    phasePoint = 8192;
    reset_adaptation = false;
  }
  else{
    if(v2 < target2) target2 = v2;
    target2 = (((running_sum_2+8) >> 4 ) - 1 - target2)*0.5 + target2;
    //if((target2-5.0)*16>running_sum_2) target2 = (running_sum_2+8)/16 - 5.0;
    //target2 = (running_sum_2+8)/16 - 5.0;
  }
  er2 = v2 - target2;
  if((abs(er2)-abs(old_er2))!=0.0){    //Which side of the minimum are we on?
    if (old_cv2*(abs(er2)-abs(old_er2))>0.0) neg=true;
    else neg=false;
  }
  if(neg) er2 *=-1.0; //If we're on the left side of the minimum, er2 is negative
  d_er2 = er2 - old_er2;
  if(abs(er2)<(target2+10) && abs(er2)>(target2+1)) i2 += er2;
  else i2 = 0.0;
  cv2 = (kp2*er2) + (ki2*i2) + (kd2*d_er2);

  phasePoint = (phasePoint+(int)cv2) & 0x3fff;

  //Now apply it all
  needs_update = false;
  if((int)cv0!=0){
    dds.setAmplitude(MyAD9959::Channel0,setPoint0);  //set CH0 amplitude
    needs_update = true;
  }
  if((int)cv1!=0){
    dds.setAmplitude(MyAD9959::Channel1,setPoint1);  //set CH1 amplitude
    needs_update = true;
  }
  if((int)cv2!=0){
    dds.setPhase(MyAD9959::Channel1,phasePoint);     //control phase via CH1
    needs_update = true;
  }
  if(needs_update){
    dds.update();
    needs_update = false;
  }

  //Store values for next control cycle
  old_er0 = er0;
  old_er1 = er1;
  old_er2 = er2;
  old_cv2 = cv2;
}

void debugMessage(){ //to be edited as needed for debugging
  Serial.print("Target:    ");
  Serial.print(target0);
  Serial.print("  ");
  Serial.println(target1);
  Serial.print("Measured:  ");
  Serial.print(v0);
  Serial.print("     ");
  Serial.println(v1);
  Serial.print("Set:       ");
  Serial.print(setPoint0);
  Serial.print("    ");
  Serial.println(setPoint1);
  Serial.print("Phase:     ");
  Serial.println(v2);
  Serial.print("Target 2:  ");
  Serial.println(target2);
  Serial.print("Phase Set: ");
  Serial.println(phasePoint*360.0/16384.0);
  Serial.print("Matched:   ");
  Serial.println(matched);
  Serial.println(" ");
  //Serial.print("er1:   ");
  //Serial.println(er1);
  //Serial.print("d_er1: ");
  //Serial.println(d_er1);
  //Serial.print("i1:    ");
  //Serial.println(i1);
  //Serial.print("cv1:   ");
  //Serial.println(cv1);
  //Serial.println(" ");
}

void checkMatch(){
  matched_0 = false;
  matched_1 = false;
  matched_phase = false;
  matched = false;
  if(fabs(v0-target0)<=m0 || (target0==0.0&&setPoint0==0))matched_0 = true;
  if(fabs(v1-target1)<=m0 || (target1==0.0&&setPoint1==0))matched_1 = true;
  if(fabs(v2-target2)<=m2 || (target0==0.0&&setPoint0==0) || (target1==0.0&&setPoint1==0)) matched_phase = true;
  //if either channel is set to 0, then comparing phase doesn't make sense
  if(matched_0 && matched_1 && matched_phase) matched = true;
}

void setup() {
  //Set up serial communication
  Serial.begin(57600);
  if(debug){
    Serial.println("RF-Power-Controller for AD9959 and AD8307");
    Serial.print("Version: ");
    Serial.println(version_no,2);
  }
  
  //Initialize AD8307
  clearInputs();
  target0 = 1000; //Start high. User can turn it down.
  target1 = 1000;
  
  //Initialize AD9959
  dds.setFrequency(MyAD9959::ChannelAll,(uint32_t)13560000); //All frequencies to 13.56 MHz
  dds.setAmplitude(MyAD9959::Channel0,1023);                 //CH0 amplitude to max
  dds.setAmplitude(MyAD9959::Channel1,1023);                 //CH1 amplitude to max
  dds.setAmplitude(MyAD9959::Channel2,0);                    //CH2 amplitude to zero
  dds.setAmplitude(MyAD9959::Channel3,0);                    //CH3 amplitude to zero
  dds.setPhase(MyAD9959::Channel1,phasePoint);               //CH1 phase to 180 deg.
  dds.update();                                              //Do it.

  //wait for everything to initialize
  delay(1000);
}

void loop() {
  handleSerialInputs();
  handleAnalogInputs();
  if(!pause) PID();
  checkMatch();

  //Count and wait a moment for other stuff
  cyclic_counter++;
  cyclic_counter = cyclic_counter & 0x3fff;
  if(debug && cyclic_counter%20==0) debugMessage();
  
  delay(50);
}
