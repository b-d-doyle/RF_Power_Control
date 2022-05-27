#include <Wire.h>
#include <AD9959.h>
#include <SerialCommand.h>
//Version 0.03

/* This script controls the AD9959 frequency generator.
 * IMPORTANT: make sure your arduino is running its digital i/o pins at 3.3v!
 * Arduinos are 5v by default, and that is too high for the AD9959
 * 
 * Serial commands:
 * 57600 baud, no parity, 8 data bits, 1 stop bit, commands are NL terminated.
 * 
 * Use "help" command for list of commands.
 * 
 * The AD9959 has channels 0 to 3. This controller interprets channel 4
 * as being a command to set all channels to the given value. The AD9959
 * library also has a CHANNEL NONE, which this code sometimes implementes
 * as channel 5 and sometimes doesn't allow (need to make this consistant).
 * 
 * Brandon has written a small C++ script to automatically control this and
 * other RS232 devices using Uwe Konopka's COPLA package (which is in C). But
 * *this code is intended to also be easy to control manually via serial.
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
int* PID_chs = new int[2]{0,1};
int  num_chs = 2;
MyAD9959::ChannelNum ch_addr[6] = {MyAD9959::Channel0,
                                   MyAD9959::Channel1,
                                   MyAD9959::Channel2,
                                   MyAD9959::Channel3,
                                   MyAD9959::ChannelAll,
                                   MyAD9959::ChannelNone};
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
int Vset[4]      = {0};                      //|~V|, the amplitude we send to the AD9959 (0->1024)
int Vmes[4]      = {0};                      //|~V|, current voltage (currently in arbitrary AD8307 units)
int Vtgt[4]      = {0};                      //|~V|, the target voltage (currently in arbitrary AD8307 units)
int Ver[4]       = {0};                      //|~V|, "error," targetV - currentV
float int_Ver[4] = {0};                      //|~V|, time integral of error
float d_Ver[4]   = {0};                      //|~V/step|, "delta error," change in error from last step
float old_Ver[4] = {0};                      //|~V|, last step's error
float Vkp[4]     = {1.0,1.0,1.0,1.0};        //empirical factor controlling how much to react to er
float Vki[4]     = {0.1,0.1,0.1,0.1};        //empirical factor controlling how much to react to integral of er
float Vkd[4]     = {-0.2,-0.2,-0.2,-0.2};    //empirical factor controlling how much to react to d_er
float Vcv[4]     = {0};                      //"control variable," the value by which we change our ad9959 amplitude
float Vtol[4]    = {2.0,2.0,2.0,2.0};        //The tolerance; how close do we have to be to be considered matched?
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
  Serial.println(F("For use with arduino_power version 0.02,"));
  Serial.println(F("which is in Brandon's version of Uwe's COPLA library"));

  Serial.println(F("\n ~~~ Commands ~~~"));
  Serial.println(F("Serial commands and inputs are separated by a space, then terminated with newline."));
  Serial.println(F("ch input may be 0-3, or 4 (ALL), or sometimes 5 (NONE) or 6 (ALL ACTIVE PID CHANNELS)."));
  Serial.println(F("\nGeneral commands:"));
  Serial.println(F("COMMAND_NAME      | INPUT(S)  | DESCRIPTION"));
  Serial.println(F("help              |           | Displays help message"));
  Serial.println(F("debugOn           |           | Debug mode sends periodic info via serial"));
  Serial.println(F("debugOff          |           | Yep"));
  //Serial.println(F("outputOn          |           | Turns on voltage output.  (Not implemented yet)"));
  //Serial.println(F("outputOff         |           | Turns off voltage output. (Not implemented yet)"));
  Serial.println(F("reset             |           | Resets the Arduino, as if with the reset button."));

  Serial.println(F("\nPID commands:"));
  Serial.println(F("COMMAND_NAME      | INPUT(S)  | DESCRIPTION"));
  Serial.println(F("setActiveChannels | ch ch ... | Set which channels are active for PID control."));
  Serial.println(F("getActiveChannels |           | Get which channels are active for PID control."));
  Serial.println(F("setVtarget        | ch v      | Set channel voltage PID target (0-1024)"));
  Serial.println(F("getVtarget        | ch        | Get channel voltage PID target"));
  Serial.println(F("setVT             | ch v      | Alias for above"));
  Serial.println(F("getVT             | ch        | Alias for above"));
  Serial.println(F("getMatch          |           | Asks if actual output matches targets"));
  Serial.println(F("pause             |           | Pause PID algorithm for both voltage and phase"));
  Serial.println(F("resume            |           | Resume PID algorithm"));
  Serial.println(F("Phase control commands not implemented yet."));

  Serial.println(F("\nDirect AD9959 registry commands:"));
  Serial.println(F("COMMAND_NAME      | INPUT(S)  | DESCRIPTION"));
  Serial.println(F("setV              | ch v      |*Set AD9959 channel voltage (0-1024)"));
  Serial.println(F("setP              | ch p      |*Set AD9959 channel phase   (0-360 )"));
  Serial.println(F("setF              | ch f      | Set AD9959 channel frequency"));
  Serial.println(F("getV              | ch        | Ask AD9959 channel voltage"));
  Serial.println(F("getP              | ch        | Ask AD9959 channel phase"));
  Serial.println(F("getF              | ch        | Ask AD9959 channel frequency"));
  Serial.println(F("* PID algorithm will immediately overwrite, unless paused or channel inactive."));

  Serial.println(F("\nVoltage and phase-difference as measured by AD830X:"));
  Serial.println(F("COMMAND_NAME      | INPUT(S)  | DESCRIPTION"));
  Serial.println(F("mesV              | ch        | Get Voltage from AD8307"));
  Serial.println(F("mesP              |           | Get Phase   from AD8302"));
}

void(* reset) (void) = 0; //Resets the arduino, as if with reset button.

void reset_hndlr(){ //says "ok" and calls reset()
  Serial.println(F("ok"));
  Serial.flush(); //Reset kills the buffer, so we need to force flush it
  dds.reset(); //Reset AD9959
  reset();     //Reset Arduino
}
void badCommand(){
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
    return -1;
  }                    // Otherwise, continue
  ch = atoi(arg);
  if (ch<0 || ch>6){   // If ch out of range of valid channel numbers, return -2
    badCommand();
    Serial.println(F("Channel number input out of valid range (0->6). Type \"help\" for details."));
    return -2;
  }
  return ch;
}

int setActiveChannels(){
  //Bring in first channel to make sure it's okay:
  int ch = inputChannel();
  if(ch==-1){badCommand();Serial.println(F("Expected at least one channel number.")); getActiveChannels(); return -1;}
  if(ch==-2){Serial.println(F("First channel number invalid.")); getActiveChannels(); return -2;}
  if(ch==5){Serial.println(F("CHANNEL NONE not implemented for PID control (yet?). Use pause command.")); getActiveChannels(); return -2;}
  if(ch==6){Serial.println(F("CH 6 means \"all active PID channels.\" So setActiveChannels 6 does nothing.")); getActiveChannels(); return 0;}
  if(ch==4 ){ //ch4 means ALL
    delete[] PID_chs;
    PID_chs = new int[4]{0,1,2,3};
    num_chs = 4;
    getActiveChannels();
    return 0;
  }
  
  //Save old info in case of bad input:
  int* old_chs = PID_chs;
  int  old_num = num_chs;
  
  //Prepare for new info:
  int* temp=NULL; //Needed for channel input loop.
  num_chs = 0;
  
  //Handle channels:
  while(ch != -1){
    //Check that channel is valid:
    if(ch==-2||ch==4||ch==5||ch==6){ //If channel number is invalid, revert to old info and make fun of user.
      Serial.println(F("Bad channel list. Reverting to old list"));
      if(PID_chs != old_chs) delete[] PID_chs; //We shouldn't even get here if PID_chs==old_chs, but better to be sure.
      PID_chs = old_chs;
      num_chs = old_num;
      return -1;
    }
    
    //Increase number of channels:
    num_chs++;
    
    //I hate arrays:
    temp = PID_chs;                      //temp holds old address
    PID_chs = new int[num_chs];          //go get a new, bigger address
    for(int i=0;i<(num_chs-1);i++){      //copy from old address to new address
      PID_chs[i]=temp[i];
    }
    PID_chs[num_chs-1]=ch;               //assign last digit of new address
    if(temp!=old_chs) delete[] temp;     //free temp (unless temp is also old_chs)
    
    //Bring in next value:
    ch = inputChannel();
  }//This method is inefficient for long lists of channels from the user, but we're expecting 1 to 4 channels.
  
  //Free memory:
  delete[] old_chs;

  //Clean up PID_chs (sort & remove duplicates):
  clean_array(&PID_chs,&num_chs);

  //Report to user:
  getActiveChannels();
  
  return 0;
}

int getActiveChannels(){
  Serial.print(F("CHs:  "));
  for(int i = 0; i<num_chs; i++){
    if(i>0)Serial.print(',');
    Serial.print(PID_chs[i]);
  }
  Serial.print('\n');
  return 0;
}

//Array stuff for handling PID_chs:
//clean_array()
int clean_array(int* arr_ptr[], int* len_ptr){
  //Calls sort_array(), removes any duplicates, and resizes if necessary.

  //For legibility, let's dereference one layer:
  int* arr = *arr_ptr;
  int  len = *len_ptr;

  //Sort the array:
  sort_array(arr,len);
  
  //Collapse any duplicates in arr. If there *are* duplicates, we'll resize below.
  int i=1; //counts through the whole arr[]
  int j=1; //counts through only unique values of arr[]
  for(i=1; i<len; i++){
    if(arr[j-1]!=arr[i]){ //If not duplicate,
      arr[j++]=arr[i];    //Store value, then increment j.
    }
  }
  
  //If we had some duplicates, resize:
  if(len != j){
    *len_ptr = j;            //Store new len
    *arr_ptr = new int[j];   //Point arr_ptr to new memory location
    for(i=0; i<j; i++){      //Fill *arr_ptr from arr, which still points to old array
      (*arr_ptr)[i] = arr[i];
    }
    delete[] arr;            //Free memory
  }
  return 0;
}

//sort_array()
int sort_array(int arr[],int len){
  //To be used on PID_chs, which will usually be length 4 at most. Let's just do an insertion sort.
  int i,j,number;
  for(i=1; i<len; i++){           //Loop forwards through arr[]
    number = arr[i];              //The number we're trying to insert.
    j=i-1;
    while(j>=0 && number<arr[j]){ //Loop backwards from arr[i] until we find where to insert the number
      arr[j+1] = arr[j];          //Shift rightwards, making room to insert the number (and overwriting where it USED to be)
      j--;
    }
    arr[j+1] = number;            //Insert the number
  }
  return 0;
}

//setVtarget
int setVtarget(){
  char* arg;
  int ch,v;
  
  //Handle channel: 
  ch = inputChannel();      //Get channel from buffer
  if (ch==-1){badCommand(); Serial.println(F("Expected at least one channel number.")); return ch;}
  if (ch==-2) return ch;                                                  //Channel invalid. Already sent error message.
  if (ch== 5) {Serial.println(F("WRN: CHANNEL_NONE SELECTED"));return 0;} //Warn and return 0. Nothing to do.
  
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
  if (ch==4||ch==6){ //CH 6 means all PID channels. CH 4 means all channels, which is interpreted the same way for this.
    if(debug&&ch==4) Serial.println(F("WRN: Only setting active PID channels."));
    for(int i=0;i<num_chs;i++) setVtarget_worker(PID_chs[i],v);
    Serial.println(F("ok"));
    return 0;
  }
  Serial.println(F("ok"));
  return setVtarget_worker(ch,v);
}
int setVtarget_worker(int ch, int v){
  if(debug){
    char msg[50];
    sprintf(msg,"VT%i-> %i",ch,v);
    Serial.println(msg);
  }
  Vtgt[ch] = v;
  return 0;
}

//getVtarget
int getVtarget(){
  int   ch;
  char* arg;

  //Handle channel: 
  ch = inputChannel();      //Get channel from buffer
  if (ch==-1){badCommand(); Serial.println(F("Expected at least one channel number.")); return ch;}
  if (ch==-2) return ch;                                                  //Channel invalid. Already sent error message.
  if (ch== 5) {Serial.println(F("WRN: CHANNEL_NONE SELECTED"));return 0;} //Warn and return 0. Nothing to do.

  //Send to worker:
  if (ch==4||ch==6){ //CH 6 means all PID channels. CH 4 means all channels, which is interpreted the same way for this.
    if(debug&&ch==4) Serial.println(F("WRN: Only getting active PID channels."));
    for(int i=0;i<num_chs;i++) getVtarget_worker(PID_chs[i]);
    return 0;
  }
  return getVtarget_worker(ch);
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
  if (ch==-1){badCommand(); Serial.println(F("Expected at least one channel number.")); return ch;}
  if (ch==-2) return ch;                                         //Channel invalid. Already sent error message.
  if (ch== 5) {Serial.println(F("WRN: CHANNEL_NONE SELECTED"));} //Warn but send it to AD9959
  
  //Handle voltage:
  arg = sCmd.next();                          // Get v from the SerialCommand object buffer
  if (arg == NULL) {badCommand();return 1;}   // If no input, return 1
  int v = atoi(arg);                          // Convert to int

  //Check value in range:
  if( v<0 || v>1024){                         // If out of range, return 1
    Serial.println(F("Amplitude value out of bounds. 0->1024"));
    badCommand();
    return 1;
  }                                           // Otherwise, continue
  
  //Send to worker:
  if (ch==6){ //CH 6 means all PID channels.
    for(int i=0;i<num_chs;i++) setV_worker(PID_chs[i],v);
  }
  else setV_worker(ch,v);
  
  //Tell AD9959 to apply it:
  dds.update();
  
  //Report success:
  Serial.println(F("ok"));
  return 0;
}
int setV_worker(int ch, int v){
  //Report to user:
  if(debug){
    char msg[50];
    sprintf(msg,"V%i-> %i",ch,v);
    Serial.println(msg);
  }
  //Send to AD9959:  
  dds.setAmplitude(ch_addr[ch],v);            // Send amplitude to AD9959
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
  if (ch==-1){badCommand(); Serial.println(F("Expected at least one channel number.")); return ch;}
  if (ch==-2) return ch;                                         //Channel invalid. Already sent error message.
  if (ch== 5) {Serial.println(F("WRN: CHANNEL_NONE SELECTED"));} //Warn but send it to AD9959

  //Handle phase:
  arg = sCmd.next();                          // Get p from the SerialCommand object buffer
  if (arg == NULL) {badCommand();return 1;}   // If no input, return 1
  p = atof(arg);                              // Convert to float

  //Check if in range:
  if( p<0 || p>360){                          // If out of range, return 1
    Serial.println(F("Phase value out of bounds. 0->360"));
    badCommand();
    return 1;
  }
  
  //Send to worker:
  if (ch==6){ //CH 6 means all PID channels.
    for(int i=0;i<num_chs;i++) setP_worker(PID_chs[i],p);
  }
  else setP_worker(ch,p);

  //Ask AD9959 to apply it:
  dds.update();

  //Restart PID phase target adaptation:
  //PID_phase_reset(); //Maybe let's don't, actually.
  
  //Report success:
  Serial.println(F("ok"));
  return 0;
}
int setP_worker(int ch, int p){
  //Report to user:
  if(debug){
    char msg[50];
    sprintf(msg,"P%i-> %i",ch,p);
    Serial.println(msg);
  }
  //Send to AD9959:  
  dds.setPhase(ch_addr[ch],p);            // Send phase to AD9959
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
  if (ch==-1){badCommand(); Serial.println(F("Expected at least one channel number.")); return ch;}
  if (ch==-2) return ch;                                         //Channel invalid. Already sent error message.
  if (ch== 5) {Serial.println(F("WRN: CHANNEL_NONE SELECTED"));} //Warn but send it to AD9959

  //Handle frequency:
  arg = sCmd.next();                          // Get p from the SerialCommand object buffer
  if (arg == NULL) {badCommand();return 1;}   // If no input, return 1
  f = atof(arg);                              // Convert to int

  //Check value in range:
  //todo
  
  //Send to worker:
  if (ch==6){ //CH 6 means all PID channels.
    for(int i=0;i<num_chs;i++) setF_worker(PID_chs[i],f);
  }
  else setF_worker(ch,f);

  //Tell AD9959 to apply it:
  dds.update();

  //Restart PID phase target adaptation:
  //PID_phase_resetAdaptation();  //Actually, maybe let's don't

  //Report success:
  Serial.println(F("ok"));
  return 0;
}
int setF_worker(int ch, int f){
  //Report to user:
  if(debug){
    char msg[50];
    sprintf(msg,"F%i-> %i",ch,f);
    Serial.println(msg);
  }
  //Send to AD9959:  
  dds.setFrequency(ch_addr[ch],f);            // Send phase to AD9959
  return 0;
}

//getV
//Voltage from AD9959 register
int getV(){
  char* arg;
  int   ch;
  
  //Handle channel: 
  ch = inputChannel();      //Get channel from buffer
  if (ch==-1){badCommand(); Serial.println(F("Expected at least one channel number.")); return ch;}
  if (ch==-2) return ch;                                         //Channel invalid. Already sent error message.
  if (ch== 5) {Serial.println(F("WRN: CHANNEL_NONE SELECTED"));} //Warn but send it to AD9959
  
  //Send to worker:
  if (ch==6){ //CH 6 means all PID channels.
    for(int i=0;i<num_chs;i++) getV_worker(PID_chs[i]);
  }
  else getV_worker(ch);
  
  return 0;
}
int getV_worker(int ch){
  dds.setChannels(ch_addr[ch]);
  uint32_t response = dds.read(MyAD9959::ACR);
  
  char msg[50];
  sprintf(msg,"V%i: %i",ch,response & 0x7FF);
  Serial.println(msg);
  return 0;
}
//getP
//Phase from AD9959 register
int getP(){
  char* arg;
  int   ch;
  
  //Handle channel: 
  ch = inputChannel();      //Get channel from buffer
  if (ch==-1){badCommand(); Serial.println(F("Expected at least one channel number.")); return ch;}
  if (ch==-2) return ch;                                         //Channel invalid. Already sent error message.
  if (ch== 5) {Serial.println(F("WRN: CHANNEL_NONE SELECTED"));} //Warn but send it to AD9959
  
  //Send to worker:
  if (ch==6){ //CH 6 means all PID channels.
    for(int i=0;i<num_chs;i++) getP_worker(PID_chs[i]);
  }
  else getV_worker(ch);

  return 0;
}
int getP_worker(int ch){
  dds.setChannels(ch_addr[ch]);
  uint32_t response = dds.read(MyAD9959::CPOW);
  
  char msg[50];
  sprintf(msg,"P%i: %i",ch,response*360/16383);
  Serial.println(msg);
  return 0;
}
//getF
//Frequency from AD9959 register
int getF(){
  char* arg;
  int   ch;
  
  //Handle channel: 
  ch = inputChannel();      //Get channel from buffer
  if (ch==-1){badCommand(); Serial.println(F("Expected at least one channel number.")); return ch;}
  if (ch==-2) return ch;                                         //Channel invalid. Already sent error message.
  if (ch== 5) {Serial.println(F("WRN: CHANNEL_NONE SELECTED"));} //Warn but send it to AD9959

  //Send to worker:
  if (ch==6){ //CH 6 means all PID channels.
    for(int i=0;i<num_chs;i++) getF_worker(PID_chs[i]);
  }
  else getF_worker(ch);

  return 0;
}
int getF_worker(int ch){
  
  dds.setChannels(ch_addr[ch]);
  uint32_t response = dds.read(MyAD9959::CFTW);
  Serial.println(response,HEX); //Still need to decode the CFTW (Channel Frequency Tuning Word). See MyAD9959::frequencyDelta()
  //I think this is it??:
  uint32_t f = (((uint64_t)response << dds.shift) - dds.reciprocal/16) / dds.reciprocal;
  char msg[50];
  sprintf(msg,"F%i: %i",ch,f);
  Serial.println(msg);
  return 0;
}

//mesV
//Voltage from AD8307 measurement
int mesV(){
  int ch;
  
  //Handle channel: 
  ch = inputChannel();      //Get channel from buffer
  if (ch==-1){badCommand(); Serial.println(F("Expected at least one channel number.")); return ch;}
  if (ch==-2) return ch;                                                  //Channel invalid. Already sent error message.
  if (ch== 5) {Serial.println(F("WRN: CHANNEL_NONE SELECTED"));return 0;} //Warn and return 0. Nothing to do.
  
  //Send to worker:
  if (ch==4){             //CH4 means do all channels.
    for(int i=0;i<4;i++) mesV_worker(i);
    return 0;
  }
  if (ch==6){             //CH6 means do all PID active channels.
    for(int i=0;i<num_chs;i++) mesV_worker(PID_chs[i]);
    return 0;
  }
  return mesV_worker(ch); //Otherwise it's just a single channel.
}
int mesV_worker(int ch){
  char msg[50];
  sprintf(msg,"VM%i:  %i",ch,Vmes[ch]);
  Serial.println(msg);
  return Vmes[ch];
}

//mesP
//Phase from AD8302 measurement
int mesP(){
  Serial.println(v2*360/16383);
  return v2;
}

//getMatch
bool getMatch(){
  bool matched_and_stable = false;
  if(matched && millis()-matchTime >= 500) {matched_and_stable=true; Serial.println(F("true"));}
  else Serial.println(F("false"));
  return matched_and_stable;
}

//pause
int pauseOn(){
  pause = true;
  Serial.println(F("paused"));
  return 0;
}
//resume
int pauseOff(){
  pause = false;
  Serial.println(F("resuming"));
  return 0;
}
//debugOn
int debugOn(){
  debug = true;
  Serial.println(F("debug on"));
  return 0;
}
//debugOff
int debugOff(){
  debug = false;
  Serial.println(F("debug off"));
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
  Vmes[0] = avg_sum_0>>avg_sum_shift; //Voltage ch 0
  Vmes[1] = avg_sum_1>>avg_sum_shift; //Voltage ch 1
  v2 = avg_sum_2>>avg_sum_shift;      //Phase ch 0 vs ch 1
  //v3 = avg_sum_3>>avg_sum_shift;    //Might need to add second AD8302?

  running_sum_2+=v2-log2[index2];
  log2[index2]=v2;
  index2 = (index2+1) & 0x0f;
  
  //Make sure our phase limits are still ok
  if(v2>max_phase) max_phase=v2; //the new upper limit
  if(v2<min_phase) min_phase=v2; //the new lower limit

}

void PID(){
  bool needs_update = false; //If nothing changes, no need to update at the end.

  //PID controller for voltage
  for(int i = 0; i<num_chs; i++){
    int ch = PID_chs[i];
    if(PID_voltage(ch)) needs_update=true; //PID_voltage() returns true if it wants to update dds
  }
  
  //PID controller for phase
  if(PID_phase()) needs_update=true;       //PID_phase() returns true if it wants to update dds
  
  //Now ask AD9959 to do it (if we changed anything)
  if(needs_update){
    dds.update();
  }
}

bool PID_voltage(int ch){
  //The PID control variable is a linear combination of ERROR, (d ERROR)/dt, and sometimes int(ERROR dt)
  //Find the three parts of the control variable:
  Ver[ch]     = Vtgt[ch] - Vmes[ch];                                    //ERROR:         How far from target?
  d_Ver[ch]   = Ver[ch]  - old_Ver[ch];                                 //(d ERROR)/dt:  How fast approaching/leaving target?
  //if( abs(Ver[ch])<5 && abs(Ver[ch])!=0 ) int_Ver[ch]+=Ver[ch];         //int(ERROR dt): How long have we been off by how much? (Only used if pretty close)
  //else int_Ver[ch]=0;                                                   //               If we're far away or right on it, set to 0.
  old_Ver[ch] = Ver[ch];                                                //Save old ERROR for next loop.
  
  //Make control variable using empirical control factors:
  Vcv[ch] = (    Ver[ch]*Vkp[ch])  // proportional * k_proportional
          + (int_Ver[ch]*Vki[ch])  // integral     * k_integral
          + (  d_Ver[ch]*Vkd[ch]); // derivative   * k_derivative

  //If control variable isn't zero, apply it:
  bool request_update = false;
  if((int)Vcv[ch]!=0){
    Vset[ch]+=(int)Vcv[ch];                                             //Update setpoint.
    if(Vset[ch]>1024) Vset[ch]=1024;                                    //check for max
    if(Vset[ch]<0   ) Vset[ch]=0;                                       //check for min
    dds.setAmplitude(ch_addr[ch],Vset[ch]);                             //Send ch amplitude to AD9959
    request_update = true;                                              //We'll update all channels at once.
  }
  return request_update;
}

//TODO: generalize to PID_phase(int chA, int chB)
bool PID_phase(){
  //The target we're looking for is noisey and can drift. (ugh). Have to do this phase target adaptation:
  if(v2 < target2) target2 = v2;                             //target2 slowly rises till we find a v2 below it (or something calls PID_phase_resetAdaptation()
  target2 += (((running_sum_2+8) >> 4 ) - 1 - target2)*0.5;

  
  //The PID control variable is a linear combination of ERROR, (d ERROR)/dt, and sometimes int(ERROR dt)
  //Find the three parts of the control variable:
  er2 = v2 - target2;                                           //ERROR:         How far from target?
  if((abs(er2)-abs(old_er2))!=0.0){                             //               But should we consider it positive or negative?
    if (old_cv2*(abs(er2)-abs(old_er2))>0.0) neg=true;          //               v2 (from AD8302) is always a positive value, but we need to know which way to go!
    else neg=false;                                             //
  }                                                             //
  if(neg) er2 *=-1.0;                                           //               If we're on the left side of the minimum, er2 is negative
  d_er2 = er2 - old_er2;                                        //(d ERROR)/dt:  How fast approaching/leaving target?
  if(abs(er2)<10 && abs(er2)>1) i2 += er2;                      //int(ERROR dt): How long have we been off by how much? (Only used if pretty close)
  else i2 = 0.0;                                                //               If we're far away or right on it, set to 0.
  old_er2 = er2;                                                //Save old ERROR for next loop.
  
  //Make control variable using empirical control factors:
  cv2 = (  er2*kp2)  // proportional * k_proportional
      + (   i2*ki2)  // integral     * k_integral
      + (d_er2*kd2); // derivative   * k_derivative
  old_cv2 = cv2;     // Save cv2 for next cycle (to determing which side of voltage minimum we're on)
  
  //Apply phase control to ch1:
  bool request_update=false;
  if((int)cv2!=0){
    phasePoint = (phasePoint+(int)cv2) & 0x3fff;     //What to send to AD9959
    dds.setPhase(ch_addr[1],phasePoint);             //Send it to AD9959
    request_update = true;
  }
  return request_update;
}

int PID_phase_reset(){
  //Resets the adaptive phase target
  target2=(float)min_phase;
  running_sum_2=v2*16;
  for(int i=0;i<16;i++) log2[i]=v2;
  phasePoint = 8192; //180 degrees
}

void debugMessage(){ //to be edited as needed for debugging
  Serial.print(F("Target:    "));
  Serial.print(Vtgt[0]);
  Serial.print(F("  "));
  Serial.println(Vtgt[1]);
  Serial.print(F("Measured:  "));
  Serial.print(Vmes[0]);
  Serial.print(F("     "));
  Serial.println(Vmes[1]);
  Serial.print(F("Set:       "));
  Serial.print(Vset[0]);
  Serial.print(F("    "));
  Serial.println(Vset[1]);
  //Serial.print(F("Phase:     "));
  //Serial.println(v2);
  //Serial.print(F("Target 2:  "));
  //Serial.println(target2);
  //Serial.print(F("Phase Set: "));
  //Serial.println(phasePoint*360.0/16383.0);
  Serial.print(F("Matched:   "));
  if(matched) Serial.println("true");
  else        Serial.println("false");
  Serial.println(F(" "));
}

bool checkMatch(){
  bool matched_before = matched; //So we can know whether to reset matchTime
  matched = true; //This is the global variable for use elsewhere

  //Check voltages okay:
  for(int i = 0; i<num_chs; i++){ //For each PID-active channel,
    int ch = PID_chs[i];
    if( (Vtgt[ch]!=0||Vset[ch]!=0) && (fabs(Vmes[ch]-Vtgt[ch])>Vtol[ch]) ) matched = false;
    //  (Target and setpoint not both 0)&& (measured v outside tolorance )
  }
  
  //Check phase okay:
  if(fabs(v2-target2)>m2 && (Vtgt[0]!=0||Vset[0]!=0) && (Vtgt[1]!=0||Vset[1]!=0)) matched = false;
  // (phase outside tolorance) && (neither channel is set to 0)

  //If we were not matched before, but we are now, reset matchTime to right now:
  if(matched && !matched_before) matchTime=millis();
  return matched;
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
  sCmd.addCommand("reset",      reset_hndlr); //reset              //Resets the Arduino, as if with the reset button.

  //PID commands:
  sCmd.addCommand("setActiveChannels", setActiveChannels); //setActiveChannels {ch,ch..} //Set which channels are active for PID control.
  sCmd.addCommand("getActiveChannels", getActiveChannels); //getActiveChannels           //Returns which channels are active.
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
  dds.setAmplitude(MyAD9959::ChannelAll,(uint16_t)0);        //All amplitudes to zero
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
  //Serial inputs:
  sCmd.readSerial();

  //Analog inputs from AD830x:
  handleAnalogInputs();

  //Maintain target voltages and phases:
  if(!pause) PID();
  checkMatch();

  //Count and wait a moment for other stuff
  cyclic_counter++;
  cyclic_counter = cyclic_counter & 0x3fff;
  if(debug && cyclic_counter%20==0) debugMessage();
  
  //delay(20);
}
