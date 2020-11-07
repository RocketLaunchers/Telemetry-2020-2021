#define HAM //Enables the Ham Radio Shield Variables
//#define BME // Enables the BME Chip if plugged in
//#define MPU // Enabels the MPU 6050
 
// The code can read sent commands but this needs to be enabaled too
#ifdef HAM
//#define HAM_READ
#endif

// VARIABLES FOR TESTING 
#define _rand // Random Data for Testing 
//#define _timer // the time it takes to send Packages
//#define onoff // On and Off abilities
//#define verbos // View All Messages Not Necessery


// Required Components
#define MIC_PIN 3
#define RESET_PIN A3
#define SWITCH_PIN 2
#include <Wire.h>
  
#ifdef HAM //HAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAM
  #include <HamShield.h>
  #include <DDS.h>
  #include <packet.h>
  #include <avr/wdt.h> 
  
  HamShield radio;
  DDS dds;
  AFSK afsk;
  String textmessage = ""; 
  String origin_call = "THROW"; // 6 Chars Long MAX
  String destination_call = "CATCH"; // 6 Chars Long MAX
  int _freq = 144390;
#endif //HAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAM
#ifdef BME //BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB

  #include "SparkFunBME280.h"
  BME280 bme; 
  float localAltitude = 0;
#endif //BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB  
#ifdef MPU //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM 
#endif //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM

/// TESTING PURPOSES
#ifdef onoff
  char _switch;
  bool _continue = false;
#endif 
#ifdef _timer
  unsigned long startTime;
  unsigned long elapsedTime; 
#endif  

String messagebuff = "Banana";
double _temp;  




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// CODE START //////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // NOTE: if not using PWM out, it should be held low to avoid tx noise
  pinMode(MIC_PIN, OUTPUT);
  digitalWrite(MIC_PIN, LOW);
  
  // prep the switch
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  
  // set up the reset control pin
  pinMode(RESET_PIN, OUTPUT);
  // turn on the radio
  digitalWrite(RESET_PIN, HIGH);
  delay(5); // wait for device to come up
  
  Serial.begin(9600);
 
  #ifdef HAM//HAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAM
    radio.initialize(); 
    radio.frequency(144390); // default aprs frequency in North America
    radio.setRfPower(0);
    radio.setVolume1(0xFF);
    radio.setVolume2(0xFF);
    radio.setSQHiThresh(-100);
    radio.setSQLoThresh(-100); 
    radio.bypassPreDeEmph();
    dds.start();
    afsk.start(&dds);
    delay(100);
    radio.setModeReceive();  
    
    Serial.print("Starting on freq ");
    Serial.println(_freq);  
  #endif //HAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAM
  #ifdef BME //BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB  
    Serial.println("CHECKING BME280");
    if (bme.beginI2C() == false) { //Begin communication over I2C
      Serial.println("BME not responding. Please check wiring.");
      while (1); //Freeze
    }
    
    bme.setFilter(4); //Lots of HW filter   
//     Sampleing: 0 through 5, oversampling *0, *1, *2, *4, *8, *16 respectively
    bme.setTempOverSample(5); //Set oversample to 16  
    bme.setPressureOverSample(5); //Set oversample to 16  
    bme.setHumidityOverSample(1); //Turn off humidity sensing  
    localAltitude = bme.readFloatAltitudeFeet(); //Set reference altitude
//    Get the local temperature!  Do this for calibration
    bme.readTempC();
  #endif //BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB     
  #ifdef MPU //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM 
  #endif //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM

  /// EXTRAS FOR TESTING 
  #ifdef onoff
      Serial.println("On Off Function Active, use '`' as the key");   
  #endif 
  #ifdef _timer
      Serial.println("Timer Active, Used to Time the sending of Packets");   
      startTime = millis();
  #endif
  #ifdef _rand
      Serial.println("Random Data Being Generated for Example");  
      randomSeed(analogRead(0));
  #endif 
  
}

void loop() { 
  // CLEAR VARIABLES
  messagebuff = "";
  
  #ifdef onoff  
    _switch = (char)Serial.read(); // Grab one char at a time
    if(_switch == '`') { 
      _continue = !_continue; 
      _switch = '\0';
    }  
    if(_continue) { 
  #endif 
    
    #ifdef _timer
      startTime = millis();
    #endif

    #ifdef _rand 
      _temp = random(0, 100);
      messagebuff += _temp;
      messagebuff += ","; 
      _temp = random(100, 200);
      messagebuff += _temp;
      messagebuff += ","; 
      _temp = random(200, 300);
      messagebuff += _temp;  
      #ifdef verbos 
        delay(100);
        Serial.print("MessageBUFF + RAND:\t");
        Serial.println(messagebuff);
      #endif
    #endif
     
    #ifdef BME //BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB
      _temp = bme.readFloatAltitudeFeet();
      messagebuff += _temp;
      messagebuff += ",";
      _temp = bme.readTempF();
      messagebuff += _temp;
      messagebuff += ",";
      _temp = bme.readFloatPressure(); 
      messagebuff += _temp;
      messagebuff += ",";
      _temp = bme.readFloatHumidity();
      messagebuff += _temp;
      messagebuff += ",";    
      #ifdef verbos 
        delay(100);
        Serial.print("MessageBUFF + BME :\t");
        Serial.println(messagebuff);
      #endif
    #endif //BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB   
    #ifdef MPU //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
      // get current FIFO count
      fifoCount = mpu.getFIFOCount(); 
      
      if(fifoCount >= 1024)
        mpu.resetFIFO(); // reset so we can continue cleanly 
      while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
      } 
      
      // display initial world-frame acceleration, adjusted to remove gravity
      // and rotated based on known orientation from quaternion
      mpu.dmpGetQuaternion(&q, fifoBuffer);
  //        Serial.print("quat\t");
  //        Serial.print(q.w);
  //        Serial.print("\t");
  //        Serial.print(q.x);
  //        Serial.print("\t");
  //        Serial.print(q.y);
  //        Serial.print("\t");
  //        Serial.println(q.z);
      
      mpu.dmpGetEuler(euler, &q);
  //        Serial.print("euler\t");
  //        Serial.print(euler[0] * 180/M_PI);
  //        Serial.print("\t");
  //        Serial.print(euler[1] * 180/M_PI);
  //        Serial.print("\t");
  //        Serial.println(euler[2] * 180/M_PI);
      
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);       mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); 
      messagebuff += (ypr[0] * 180/M_PI);
      messagebuff += ",";
      messagebuff += (ypr[1] * 180/M_PI);
      messagebuff += ",";
      messagebuff += (ypr[2] * 180/M_PI);
      messagebuff += ",";  
      
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      messagebuff += (aaReal.x);
      messagebuff += ",";
      messagebuff += (aaReal.y);
      messagebuff += ",";
      messagebuff += (aaReal.z);
      messagebuff += ",";   
      
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  //        Serial.print("aworld\t");
  //        Serial.print(aaWorld.x);
  //        Serial.print("\t");
  //        Serial.print(aaWorld.y);
  //        Serial.print("\t");
  //        Serial.println(aaWorld.z);   
    }
      #ifdef verbos 
        delay(100);
        Serial.print("MessageBUFF + MPU :\t");
        Serial.println(messagebuff);
      #endif
    #endif //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM

 
    #ifdef HAM //HAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAM
      prepMessage();   
    #endif //HAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAMHAM
    
    #ifdef _timer
      elapsedTime = millis() - startTime;
      Serial.print("Total time: ");
      Serial.println(elapsedTime);
      Serial.println();
    #endif //_timer 
    
  #ifdef onoff
    } // end If(_Continue) 
  #endif //onoff
    
  #ifdef HAM_READ //HRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHR
    if(afsk.decoder.read() || afsk.rxPacketCount()) {
      // A true return means something was put onto the packet FIFO
      // If we actually have data packets in the buffer, process them all now
      while(afsk.rxPacketCount()) {
        AFSK::Packet *packet = afsk.getRXPacket();
        Serial.print(F("Packet: "));
        if(packet) {
          packet->printPacket(&Serial);
          AFSK::PacketBuffer::freePacket(packet);
          
        } // end If(packet)
      } // end While
    } // end Decoder.read() 
  #endif  //HRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHRHR
  
} // end Loop
    
#ifdef HAM //HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH
void prepMessage() { 
  radio.setModeTransmit();
  delay(500);
  textmessage = messagebuff; 
  Serial.println(textmessage);
  
  AFSK::Packet *packet = AFSK::PacketBuffer::makePacket(22 + 32);

  packet->start();
  packet->appendCallsign(origin_call.c_str(),0);
  packet->appendCallsign(destination_call.c_str(),15,true);   
  packet->appendFCS(0x03);
  packet->appendFCS(0xf0);
  packet->print(textmessage);
  packet->finish();

  bool ret = afsk.putTXPacket(packet);

  if(afsk.txReady()) {
    Serial.println(F("txReady"));
    radio.setModeTransmit();
    delay(100);
    if(afsk.txStart()) {
      Serial.println(F("txStart"));
    } else {
      radio.setModeReceive();
    }
  }
  // Wait 2 seconds before we send our beacon again.
  Serial.println("tick");
  // Wait up to 2.5 seconds to finish sending, and stop transmitter.
  // TODO: This is hackery.
  for(int i = 0; i < 500; i++) {
    if(afsk.encoder.isDone())
       break;
    delay(50); // Delay to wait for Encoder
  }
  Serial.println("Done sending");
  radio.setModeReceive();
} 
 

ISR(TIMER2_OVF_vect) {
  TIFR2 = _BV(TOV2);
  static uint8_t tcnt = 0;
  if(++tcnt == 8) {
    dds.clockTick();
    tcnt = 0;
  }
}

ISR(ADC_vect) {
  static uint8_t tcnt = 0;
  TIFR1 = _BV(ICF1); // Clear the timer flag
  dds.clockTick();
  if(++tcnt == 1) {
    afsk.timer();
    tcnt = 0;
  }
}
#endif //HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH
