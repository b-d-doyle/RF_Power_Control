#include <Wire.h>
#include <AD9959.h>
#include <SerialCommand.h>
//Version 0.03

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
float version_no = 0.03;
bool  debug = false;
bool  pause = false; //Can pause active control during data collection.
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
int active_channels[] = {0,1}; //Which channels we care about (may contain any or all of 0->3, but only once each please!
int num_ch = sizeof active_channels/sizeof active_channels[0];
int ch_addr[6] = {0x10,0x20,0x40,0x80,0xF0,0x00};
               //{CH0 ,CH1 ,CH2 ,CH3 ,ALL ,NONE};
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
/*
int v0        = 0;      //|~V|, current voltage (currently in arbitrary AD8307 units)
int v1        = 0;
int target0   = 0.0;    //|~V|, the target voltage (currently in arbitrary AD8307 units)
int target1   = 0.0;
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
int setPoint0 = 0;      //What to tell the AD9959 to set the amplitude to
int setPoint1 = 0;
float m0      = 2.0;    //The tolerance; how close do we have to be to be considered matched?
float m1      = 2.0;
*/
int v[4]         = {0};      //|~V|, current voltage (currently in arbitrary AD8307 units)
int Vtgt[4]      = {0};    //|~V|, the target voltage (currently in arbitrary AD8307 units)
float Ver[4]     = {0};    //|~V|, "error," targetV - currentV
float Vi[4]      = {0};    //|~V|, time integral of error
float old_Ver[4] = {0};    //|~V|, last step's error
float d_Ver[4]   = {0};    //|~V/step|, "delta error," change in error from last step
float Vkp[4]     = {1.0,1.0,1.0,1.0};    //empirical factor controlling how much to react to er
float Vki[4]     = {0.1,0.1,0.1,0.1};    //empirical factor controlling how much to react to integral of er
float Vkd[4]     = {-0.2,-0.2,-0.2,-0.2};    //empirical factor controlling how much to react to d_er
float Vcv[4]     = {0};    //"control variable," the value by which we change our ad9959 amplitude
int Vsetpoint[4] = {0};      //What to tell the AD9959 to set the amplitude to
float Vtol[4]    = {2.0,2.0,2.0,2.0};    //The tolerance; how close do we have to be to be considered matched?
//--------------------

//PID phase stuff
//--------------------
bool  reset_adaptation = true;
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
bool matched = false;       //if all amplitudes and phases match requested values
unsigned long matchTime;    //To know how long all outputs have matched the requested values. (To know if we're stable yet.)
                            //We'll update this with millis(). I don't bother protecting against overflow, because
                            //millis() is based on unsigned long (32 bit), so it doesn't overflow until 50 days(!)
                            //since the last arduino reset.
//--------------------

//Serial control stuff
//--------------------
SerialCommand sCmd;                 // The SerialCommand object
//This, plus all the commands defined below.
//--------------------


//END DECLARATIONS
//-----------------------------------------------------------------
//-----------------------------------------------------------------

//-----------------------------------------------------------------
//-----------------------------------------------------------------
//SERIAL COMMAND METHODS

void helpMessage(){
  Serial.println(F("RF-Power-Controller for AD9959, AD8307, and AD8302"));
  Serial.print(F("Version: "));
  Serial.println(version_no,2);
  Serial.println(F("PID controller for voltage and relative phase of two channels."));
  Serial.println(F("For use with Brandon's COPLA library, arduino_power version 0.01"));
  
  Serial.println(F("\nCommands:"));
  Serial.println(F("debugOn  -- turns debug mode on"));
  Serial.println(F("debugOff -- turns debug mode off"));
  Serial.println(F("pause    -- pauses PID control"));
  Serial.println(F("resume   -- resumes PID control"));
  Serial.println(F("setVoltage,[channel],[value]"));
  Serial.println(F("setFrequency,[channel],[value]"));
  Serial.println(F("setPhase,[channel],[value]"));
  Serial.println(F("setDirectVoltage,[channel],[value]"));
  Serial.println(F("  Note:[channel] may be 0-3 for a"));
  Serial.println(F("  specific channel, or 4 for all channels"));
  Serial.println(F("get,[value]"));
  Serial.println(F("Use 'get,help' for more help on get"));
  
  Serial.println(F("\nExcept I changed a bunch of stuff. Need to update this. Sorry!"));
}

void(* reset) (void) = 0; //Resets the arduino, as if with reset button.

void clearInputs(){
  //TODO? Maybe not needed anymore...
}

void badCommand(){
  clearInputs();
  Serial.println(F("ERR"));
}

void pauseWarning(){
  Serial.println(F("Paused. To resume, send 'resume'."));
}

int inputChannel(){
  int ch;
  char* arg;
  arg = sCmd.next();   // Get ch from the SerialCommand object buffer
  if (arg == NULL) {   // If no input for ch, return -1
    badCommand();
    Serial.println(F("Expected channel number input"));
    return -1;
  }                    // Otherwise, continue
  ch = atoi(arg);
  if (ch<0 || ch>5){  // If ch out of range of valid channel numbers, return -1
    badCommand();
    Serial.println(F("Channel number input out of valid range (0->4)"));
    return -1;
  }
  return ch;
}

//setVtarget
int setVtarget(){
  char* arg;
  int ch,v;
  
  //Handle channel: 
  ch = inputChannel();      //Get channel from buffer
  if (ch==-1) return -1;    //If failed, return -1
  if (ch== 5) {Serial.println("WRN: CHANNEL_NONE SELECTED");return 0;} //Warn and return 0. Nothing to do.
  
  //get setPoint:
  arg = sCmd.next();                          // Get v from the SerialCommand object buffer
  if (arg == NULL) {badCommand();return 1;}   // If no input, return 1
  v = atof(arg);                              // Convert to int

  //Check if in range:
  if( v<0 || v>1023){                         // If out of range, return 1
    Serial.println(F("Value out of bounds. 0->1023"));
    badCommand();
    return 1;
  }
  
  //Send to worker:
  if (ch==4){ //ch==4 means do all valid channels.
    for(int i=0;i<num_ch;i++) setVtarget_worker(active_channels[i],v);
    return 0;
  }
  return setVtarget_worker(ch,v);
}
int setVtarget_worker(int ch, int v){
  char msg[50];
  sprintf(msg,"VT%i-> %i",ch,v);
  Serial.println(msg);
  Vtgt[ch] = v;
  return 0;
}

//getVtarget
int getVtarget(){
  int   ch;
  char* arg;

  //Handle channel: 
  ch = inputChannel();      //Get channel from buffer
  if (ch==-1) return -1;    //If failed, return -1
  if (ch== 5) {Serial.println("WRN: CHANNEL_NONE SELECTED");return 0;} //Warn and return 0. Nothing to do.

  //Send to worker:
  if (ch==4){ //ch==4 means do all valid channels.
    for(int i=0;i<num_ch;i++) getVtarget_worker(active_channels[i]);
    return 0;
  }
  return getVtarget_worker(ch); //should handle ch=4 or invalid ch here!
}
int getVtarget_worker(int ch){
  char msg[50];
  sprintf(msg,"VT%i:  %i",ch,Vtgt[ch]);
  Serial.println(msg);
  
  return Vtgt[ch];
}

//setV
//Send Voltage directly to AD9959 register
int setV(){
  char* arg;
  int   ch;
  
  //Handle channel: 
  ch = inputChannel();      //Get channel from buffer
  if (ch==-1) return -1;    //If failed, return -1
  if (ch== 5) {Serial.println("WRN: CHANNEL_NONE SELECTED");} //Warn but send it to AD9959
  
  //Handle voltage:
  arg = sCmd.next();                          // Get v from the SerialCommand object buffer
  if (arg == NULL) {badCommand();return 1;}   // If no input, return 1
  int v = atoi(arg);                          // Convert to int

  //Check value in range:
  if( v<0 || v>1023){                         // If out of range, return 1
    Serial.println(F("Value out of bounds. 0->1023"));
    badCommand();
    return 1;
  }                                           // Otherwise, continue
  
  //Report to user:
  char msg[50];
  sprintf(msg,"V%i -> %i",ch,v);
  Serial.println(msg);
  
  //Send to AD9959:
  dds.setAmplitude(ch_addr[ch],v);            // Send amplitude to AD9959
  dds.update();                               // Tell AD9959 to apply it
  Serial.println(F("ok"));
  return 0;
}

//setP
//Send Phase directly to AD9959 register
int setP(){
  char* arg;
  int   ch;
  float p;
  
  //Handle channel: 
  ch = inputChannel();      //Get channel from buffer
  if (ch==-1) return -1;    //If failed, return -1
  if (ch== 5) {Serial.println("WRN: CHANNEL_NONE SELECTED");} //Warn but send it to AD9959

  //Handle phase:
  arg = sCmd.next();                          // Get p from the SerialCommand object buffer
  if (arg == NULL) {badCommand();return 1;}   // If no input, return 1
  p = atof(arg);                        // Convert to float

  //Check value in range:
  //todo
  
  //Report to user:
  char msg[50];
  sprintf(msg,"P%i -> %i",ch,p);
  Serial.println(msg);
  
  //Send to AD9959:
  dds.setPhase(ch_addr[ch],(int)(p*16384/360));//Send to AD9959
  dds.update();                                //Tell AD9959 to do it
  //reset_adaptation = true;                   //Flag that we need to restart on phase PID
  Serial.println(F("ok"));
  return 0;
}

//setF
//Send Frequency directly to AD9959 register
int setF(){
  char* arg;
  int   ch;
  float f;
  
  //Handle channel: 
  ch = inputChannel();      //Get channel from buffer
  if (ch==-1) return -1;    //If failed, return -1
  if (ch== 5) {Serial.println("WRN: CHANNEL_NONE SELECTED");} //Warn but send it to AD9959

  //Handle frequency:
  arg = sCmd.next();                          // Get p from the SerialCommand object buffer
  if (arg == NULL) {badCommand();return 1;}   // If no input, return 1
  f = atof(arg);                          // Convert to int

  //Check value in range:
  //todo
  
  //Report to user:
  char msg[50];
  sprintf(msg,"F%i -> %i",ch,f);
  Serial.println(msg);
  
  //Send to AD9959:
  dds.setFrequency(ch_addr[ch],f);       //Send to AD9959
  dds.update();                          //Tell AD9959 to do it
  //reset_adaptation = true;             //Flag that we need to restart on phase PID
  Serial.println(F("ok"));
  return 0;
}

//getV
//Voltage from AD9959 register
int getV(){
  char* arg;
  int   ch;
  
  //Handle channel: 
  ch = inputChannel();      //Get channel from buffer
  if (ch==-1) return -1;    //If failed, return -1
  if (ch== 5) {Serial.println("WRN: CHANNEL_NONE SELECTED");} //Warn but send it to AD9959
  
  return dds.read(ch_addr[ch]);
}
//getP
//Phase from AD9959 register
int getP(){
  char* arg;
  int   ch;
  
  //Handle channel: 
  ch = inputChannel();      //Get channel from buffer
  if (ch==-1) return -1;    //If failed, return -1
  if (ch== 5) {Serial.println("WRN: CHANNEL_NONE SELECTED");} //Warn but send it to AD9959
  return 0;
}
//getF
//Frequency from AD9959 register
int getF(){
  char* arg;
  int   ch;
  
  //Handle channel: 
  ch = inputChannel();      //Get channel from buffer
  if (ch==-1) return -1;    //If failed, return -1
  if (ch== 5) {Serial.println("WRN: CHANNEL_NONE SELECTED");} //Warn but send it to AD9959
  return 0;
}

//mesV
//Voltage from AD8307 measurement
int mesV(){
  int ch;
  
  //Handle channel: 
  ch = inputChannel();      //Get channel from buffer
  if (ch==-1) return -1;    //If failed, return -1
  if (ch== 5) {Serial.println("WRN: CHANNEL_NONE SELECTED");return 0;} //Warn and return 0. Nothing to do.
  
  //Send to worker:
  if (ch==4){ //ch==4 means do all valid channels.
    for(int i=0;i<num_ch;i++) mesV_worker(active_channels[i]);
    return 0;
  }
  return mesV_worker(ch);
}
int mesV_worker(int ch){
  char msg[50];
  sprintf(msg,"VM%i:  %i",ch,v[ch]);
  Serial.println(msg);
  return v[ch];
}

//mesP
//Phase from AD8302 measurement
int mesP(){
  Serial.println(v2*360/16384);
  return v2;
}

//getMatch
int getMatch(){
  if(matched && millis()-matchTime >= 500) Serial.println(F("true"));
  else Serial.println(F("false"));
  return 0;
}

//pause
int pauseOn(){
  pause = true;
  Serial.println(F("ok"));
  return 0;
}
//resume
int pauseOff(){
  pause = false;
  Serial.println(F("ok"));
  return 0;
}
//debugOn
int debugOn(){
  debug = true;
  Serial.println(F("ok"));
  return 0;
}
//debugOff
int debugOff(){
  debug = false;
  Serial.println(F("ok"));
  return 0;
}
//outputOn
int outputOn(){
  //todo
  return 0;
}
//outputOff
int outputOff(){
  //todo
  return 0;
}

//END SERIAL COMMAND METHODS
//-----------------------------------------------------------------
//-----------------------------------------------------------------

//-----------------------------------------------------------------
//-----------------------------------------------------------------
//MAIN LOOP METHODS

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
  //v0 = avg_sum_0>>avg_sum_shift; //old way
  //v1 = avg_sum_1>>avg_sum_shift; //old way
  v[0] = avg_sum_0>>avg_sum_shift; //Voltage ch 0
  v[1] = avg_sum_1>>avg_sum_shift; //Voltage ch 1
  v2 = avg_sum_2>>avg_sum_shift;   //Phase ch 0 vs ch 1
  //v3 = avg_sum_3>>avg_sum_shift; //Might need to add second AD8302?

  running_sum_2+=v2-log2[index2];
  log2[index2]=v2;
  index2 = (index2+1) & 0x0f;
  
  //Make sure our phase limits are still ok
  if(v2>max_phase) max_phase=v2; //the new upper limit
  if(v2<min_phase) min_phase=v2; //the new lower limit

}

void PID(){
  //PD controller for voltage
  /*
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
  */
  for(int i = 0; i<num_ch; i++){
    int ch = active_channels[i];
    Ver[ch] = Vtgt[ch] - v[ch];          //How far from target?
    d_Ver[ch] = Ver[ch] - old_Ver[ch];   //How fast approaching/leaving target?
    if(abs(Ver[ch])<5.0 && abs(Ver[ch])!=0.0) Vi[ch]+=Ver[ch];      //We only use the integral part if we're pretty close to the value we want
    else Vi[ch]=0;
    Vcv[ch] = (Vkp[ch]*Ver[ch]) + (Vki[ch]*Vi[ch]) + (Vkd[ch]*d_Ver[ch]); //Control variable proportional to er and d_er
  
    if((Vsetpoint[ch]+(int)Vcv[ch])>1023) Vsetpoint[ch]=1023; //max
    else if((Vsetpoint[ch]+(int)Vcv[ch])<0) Vsetpoint[ch]=0;  //min
    else Vsetpoint[ch]+=(int)Vcv[ch];
    old_Ver[ch] = Ver[ch];
  }
  
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
  bool needs_update = false;
  //Apply voltage control to each channel:
  for(int i = 0; i<num_ch; i++){
    int ch = active_channels[i];
    if((int)Vcv[ch]!=0){
      dds.setAmplitude(static_cast<MyAD9959::ChannelNum>(ch+1),Vsetpoint[ch]);  //set ch amplitude
      needs_update = true;
    }
  }
  /*
  if((int)cv0!=0){
    dds.setAmplitude(MyAD9959::Channel0,setPoint0);  //set CH0 amplitude
    needs_update = true;
  }
  if((int)cv1!=0){
    dds.setAmplitude(MyAD9959::Channel1,setPoint1);  //set CH1 amplitude
    needs_update = true;
  }*/
  //Apply phase control to ch1:
  if((int)cv2!=0){
    dds.setPhase(MyAD9959::Channel1,phasePoint);     //control phase via CH1
    needs_update = true;
  }
  //Now ask AD9959 to do it (if we changed anything)
  if(needs_update){
    dds.update();
    needs_update = false;
  }

  //Store values for next control cycle
  //old_er0 = er0; //now done in earlier loop
  //old_er1 = er1; //now done in earlier loop
  old_er2 = er2;
  old_cv2 = cv2;
}

void debugMessage(){ //to be edited as needed for debugging
  Serial.print(F("Target:    "));
  Serial.print(Vtgt[0]);
  Serial.print(F("  "));
  Serial.println(Vtgt[1]);
  Serial.print(F("Measured:  "));
  Serial.print(v[0]);
  Serial.print(F("     "));
  Serial.println(v[1]);
  Serial.print(F("Set:       "));
  Serial.print(Vsetpoint[0]);
  Serial.print(F("    "));
  Serial.println(Vsetpoint[1]);
  Serial.print(F("Phase:     "));
  Serial.println(v2);
  Serial.print(F("Target 2:  "));
  Serial.println(target2);
  Serial.print(F("Phase Set: "));
  Serial.println(phasePoint*360.0/16384.0);
  Serial.print(F("Matched:   "));
  Serial.println(matched);
  Serial.println(F(" "));
}

void checkMatch(){
  bool matched_0 = false;
  bool matched_1 = false;
  bool matched_phase = false;
  bool matched_before = matched; //So we can know whether to reset matchTime
  matched = false; //This one global so we can check outside
  if(fabs(v[0]-Vtgt[0])<=Vtol[0] || (Vtgt[0]==0&&Vsetpoint[0]==0)) matched_0 = true;
  if(fabs(v[1]-Vtgt[1])<=Vtol[1] || (Vtgt[1]==0&&Vsetpoint[1]==0)) matched_1 = true;
  if(fabs(v2-target2)<=m2 || (Vtgt[0]==0&&Vsetpoint[0]==0) || (Vtgt[1]==0&&Vsetpoint[1]==0)) matched_phase = true;
  //if either channel is set to 0, then comparing phase doesn't make sense

  //We're matched if all conditions are met:
  if(matched_0 && matched_1 && matched_phase) matched = true;

  //If we were not matched before, but we are now, reset matchTime to right now:
  if(matched && !matched_before) matchTime=millis();
}
//END MAIN LOOP METHODS
//-----------------------------------------------------------------
//-----------------------------------------------------------------


void setup() {
  //Set up serial communication
  Serial.begin(57600);
  if(debug){
    Serial.println(F("RF-Power-Controller for AD9959 and AD8307"));
    Serial.print(F("Version: "));
    Serial.println(version_no,2);
  }
  
  ///////SERIAL COMMANDS///////
  //General commands:
  sCmd.addCommand("help",       helpMessage); //help               //Displays help message
  sCmd.addCommand("debugOn",    debugOn);     //debugOn            //debug mode sends more info via serial. Intended for use with user control, not for software control.
  sCmd.addCommand("debugOff",   debugOff);    //debugOff           //yep
  sCmd.addCommand("outputOn",   outputOn);    //outputOn           //turns on voltage output. (Not implemented yet)
  sCmd.addCommand("outputOff",  outputOff);   //outputOff          //turns off voltage output. (Not implemented yet)
  sCmd.addCommand("reset",      reset);       //reset              //Resets the Arduino, as if with the reset button.

  //PID commands:
  sCmd.addCommand("setVtarget", setVtarget);  //setVtarget ch v    //channel voltage setpoint
  sCmd.addCommand("getVtarget", getVtarget);  //getVtarget ch      //channel voltage setpoint
  sCmd.addCommand("setVT",      setVtarget);  //setVT      ch v    //Alias for above
  sCmd.addCommand("getVT",      getVtarget);  //getVT      ch      //Alias for above
  sCmd.addCommand("getMatch",   getMatch);    //getMatch           //Asks if actual output "matches" setpoints
  sCmd.addCommand("pause",      pauseOn);     //pause              //Pause PID algorithm, allowing for direct channel control via setV, setP, etc
  sCmd.addCommand("resume",     pauseOff);    //resume             //Resume PID algorithm
  //(no PID phase commands because we'd need 2 AD8302 chips to measure true phase value. For now, we just try to hit 180 deg)
  //^^Is that nonsense? Maybe we can do better with just the one AD8302

  //Direct AD9959 registry commands:
  sCmd.addCommand("setV",       setV);        //setV       ch v    //set AD9959 channel voltage   //Warn if PID not paused??
  sCmd.addCommand("setP",       setP);        //setV       ch p    //set AD9959 channel phase     //Warn if PID not paused??
  sCmd.addCommand("setF",       setF);        //setV       ch f    //set AD9959 channel frequency
  sCmd.addCommand("getV",       getV);        //getV       ch      //ask AD9959 channel voltage
  sCmd.addCommand("getP",       getP);        //getV       ch      //ask AD9959 channel phase
  sCmd.addCommand("getF",       getF);        //getV       ch      //ask AD9959 channel frequency

  //Voltage and phase-difference as measured by AD380X:
  sCmd.addCommand("mesV",       mesV);        //mesV       ch      //voltage from AD8307
  sCmd.addCommand("mesP",       mesP);        //mesP       ch      //phase   from AD8302
  
  sCmd.setDefaultHandler(badCommand);        //If the user sends a bad command, we insult their family and challenge them to a duel.
  /////END SERIAL COMMANDS/////
  
  //Initialize AD9959
  dds.setFrequency(MyAD9959::ChannelAll,(uint32_t)13560000); //All frequencies to 13.56 MHz
  dds.setAmplitude(MyAD9959::ChannelAll,0);                  //All amplitudes to zero
  /*dds.setAmplitude(MyAD9959::Channel0,1023);               //CH0 amplitude to max
  dds.setAmplitude(MyAD9959::Channel1,1023);                 //CH1 amplitude to max
  dds.setAmplitude(MyAD9959::Channel2,0);                    //CH2 amplitude to zero
  dds.setAmplitude(MyAD9959::Channel3,0);                    //CH3 amplitude to zero
  */
  dds.setPhase(MyAD9959::Channel1,phasePoint);               //CH1 phase to 180 deg.
  dds.update();                                              //Do it.

  //wait for everything to initialize
  delay(1000);
}

void loop() {
  sCmd.readSerial();
  handleAnalogInputs();
  if(!pause) PID();
  checkMatch();

  //Count and wait a moment for other stuff
  cyclic_counter++;
  cyclic_counter = cyclic_counter & 0x3fff;
  if(debug && cyclic_counter%20==0) debugMessage();
  
  delay(50);
}
