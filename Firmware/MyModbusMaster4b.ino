
#include <ModbusMaster.h>
#include <MQTT.h>
#include <neopixel.h>
#include <Adafruit_DHT.h>

//#include "Particle.h"

#define XMITDELAY 100
//#define POLLING_TIME  Modbus master 120000
#define POLLING_TIME 40000
// 0 for 0-based, 1 for 1-based numbering
#define BASED_NUMBERING 1
#define DRIVEID  1
#define DHTPIN D1       // Humidity and temp sensor
#define DHTTYPE DHT11		// DHT 11

// IMPORTANT: Set pixel COUNT, PIN and TYPE
#define PIXEL_PIN D0
#define PIXEL_COUNT 1
#define PIXEL_TYPE WS2811
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);
int red;
int green;
int blue;
int white;
int clear_led;
bool tripped = 0;
uint16_t control_wrd         = 0;
bool   DEBUGON               = 0;
unsigned long previousMillis = 0;
unsigned long previousMillis2= 0;
//char version* ="1.0";
//char server1  ="iot4fun.ddns.net";
//Modbus registers CT drive M200_M400 OPEN LOOP
uint16_t frozen_regs[18]        ={101,102,103,169,170,201,301,401,402,417,501,502,503,505,701,702,1040,704};
int16_t regs_dataf[18];
uint16_t regs_CSK[24]           ={736,401,402,617,417,419,420,426,501,502,503,504,505,704,626,734,1020,1040,1811,1812,1813,1814,1815,1816};
int16_t regs_data[24];

char vfd_data[450];
char motor_data[450];
char vfd_frzndata1[300];
char vfd_frzndata2[300];
char buf[350];

//MQTT Settings Mosquitto runing  local on ubunto laptop.
byte server[] = { 192,168,2,13 };//the IP of broker
void callback(char* topic, byte* payload, unsigned int length);
//MQTT client(server, 1883, callback,700);
//MQTT client(server, 1883, callback,700);
MQTT client("iot4fun.ddns.net", 1883, callback,700);
uint16_t qos2messageid = 0;

// instantiate ModbusMaster object as slave ID 1
ModbusMaster node(Serial1,1);

bool state = true;
//int  data[6];   //was uint16_t
int16_t data[2];
int16_t dataf[2];
int color_patern1 [9]; //all colors
int color_patern2 [7]; //white
int color_patern3 [7]; //green
int color_patern4 [7]; //blue
int color_patern5 [7]; //red
//Dht funtion setup pin, an sensor type
DHT dht(DHTPIN, DHTTYPE);
// turn on/off drive funtion name
int turn_On_Off_drive(String command);

//int turnPump(String command);
int  turn_On_Off_drive(String command)
{
	if (command == "ON")
	{
		  //led_patern1 (30);
			 green_led();
			 mb_wr_sr (641, 131);
			 //control_wrd=131;
			 //MBwriteSR();
		  return 1;
	}
	else  if (command == "OFF")
	{
		  red_led();
			mb_wr_sr (641, 128);
			//control_wrd=128;
			//MBwriteSR();
		  return 1;
	}
	else
	{
		return -1;
	}

}


//==========================================================


bool mb_wr_sr (uint16_t regis , int16_t reg_val)
{
uint8_t result3;
result3=node.writeSingleRegister(regis,reg_val);
if (result3 == node.ku8MBSuccess)
{
   Particle.publish("Modbus Write OK");
   return true;

}
else
{
   Particle.publish("Modbus Write FAIL");
   return false;
}
node.clearResponseBuffer(); // if Timeout clearResponseBuffer and clearTransmitBuffer
node.clearTransmitBuffer();
//serialFlushInBuffer();
result3=node.writeSingleRegister(regis,reg_val);
}
//==========================================================


void serialFlushInBuffer()
{
  while(Serial1.read() >= 0);
}
/*
//SYSTEM_MODE(AUTOMATIC);
//============================================================================================
//SYSTEM_MODE(SEMI_AUTOMATIC);
// The semi-automatic mode will not attempt to connect the device to the Cloud automatically.
// However once the device is connected to the Cloud (through some user intervention),
//messages will be processed automatically, as in the automatic mode above.
//============================================================================================

//============================================================================================
//SYSTEM_MODE(MANUAL);
//he "manual" mode puts the device's connectivity completely in the user's control.
//This means that the user is responsible for both establishing a connection to the
//Particle Cloud and handling communications with the Cloud by calling Particle.process() on a regular basis.
//============================================================================================

void setup() {
  // This is called immediately
}

void loop() {
  if (buttonIsPressed()) {
    Particle.connect();
  } else {
    doOfflineStuff();
  }
//or
if (buttonIsPressed()) {
    Particle.connect();
  }
  if (Particle.connected()) {
    Particle.process();
    doOtherStuff();
  }

}

*/
// Returns temperature VFD stack temperature
int getStackTemperature(String args){
    return regs_data[13];
}
// Return VFD DC bus voltage
int getspeed(String args){
    //return regs_data[12]; //DC BUS voltage
		return regs_data[11]; //VFD SPEED
}

void setup() {
  Particle.function("VFD-ONOFF", turn_On_Off_drive);
  Particle.function("Stack-Temp", getStackTemperature);
  Particle.function("Motor-Speed", getspeed);


	delay(2000);
		dht.begin();
		led_patern_setup();

   //led_test();
	 led_patern1 (30) ;
	// initialize Modbus communication baud rate
	node.begin(19200);
	node.enableTXpin(D7); //D7 is the pin used to control the TX enable pin of RS485 driver
	//node.enableDebug();  //Print TX and RX frames out on Serial. Beware, enabling this messes up the timings for RS485 Transactions, causing them to fail.
	Serial.begin(19200);
//	while(!Serial.available()) Particle.process();
	Serial.println("Starting Modbus Transaction:");

	//########################### MQTT ##############################
  //RGB.control(true);

   // connect to the server
  // client.connect("ModbusMaster", "angel","xfilesxvf");  //old credentials
   client.connect("ModbusMaster", "evert","evert_1234");
      // publish/subscribe
   if (client.isConnected()) {
       // it can use messageid parameter at 4.
      // uint16_t messageid;
	 	  sprintf(buf, "{\"NODE_INFO\":{\"IOT NAME\":\"Photon CT VFD\",\"DRIVEID\": %d }}", DRIVEID);
       client.publish("outTopic/message", buf);// "hello world QOS1", MQTT::QOS1, &messageid);
       client.subscribe("inTopic/message");
   }
	  publish_data();
}

void led_patern1 (uint16_t blk_time) {
	uint8_t m;
for (m = 0; m < 9; m++) {
	unsigned long currentMillis = millis();
  //if (currentMillis - previousMillis2 >= 200) {
	strip.setPixelColor(0, color_patern1 [m]);
	strip.show();
	//previousMillis2 = currentMillis;
	delay(blk_time);
  //}
	//previousMillis2 = currentMillis;
 }
}




void loop() {
    if (client.isConnected())
        client.loop();

unsigned long currentMillis = millis();
if (currentMillis - previousMillis >= POLLING_TIME) {
	//Spark.publish("tripped or not ",tripped);


 if ((regs_data[17] & (1<<0))? 1:0)
//if (((regs_data[17] >> 0)&1) & (tripped=0))
    {   //test StatusWord bit 0 to know if drive is healthy or no  (1<<bitnumber)
     Spark.publish("DRIVEHEALTH","VFD IS  OK");
		// publish_data();
		// regs_dataf[16]=regs_data[17];
		// publish_data();
		 publish_data();
     tripped =0;
    } else
		{
			 tripped =1;
			 Spark.publish("DRIVEHEALTH","VFD IS  tripped");
			 frozzen_data() ;
			 regs_data[17]=regs_dataf[16];
             temp_hum ();   // call either enviromental data
		}
/*
		if ((regs_dataf[16] & (1<<0))? 0:1)
	 //if (((regs_data[17] >> 0)&1) & (tripped=0))
	     {   //test StatusWord bit 0 to know if drive is healthy or no  (1<<bitnumber)
	      Spark.publish("DRIVEHEALTH","VFD IS  Tripped");
	 		// publish_data();
	 		 regs_data[17]=regs_dataf[16];
			// frozzen_data() ;
	      tripped =1;
	     }
*/
/*
//else
  //  {
	//   Spark.publish("DRIVEHEALTH","VFD entered in a tripped staate");
	   //frozzen_data() ;
	//regs_data[17]= regs_dataf[16];
	//   tripped = 1;
//    }
if (regs_dataf[16] & (1<<0) & (tripped=1))
    {  //if drive trip have been reset return to send the normal data stream
	//	Spark.publish("DRIVE HAVE BEEN RESET");
	//	publish_data();
		regs_data[17]=regs_dataf[16];
		Spark.publish("DRIVE HAVE BEEN RESET");
		tripped =0;
	  }
	// else
	//  {
	//	 tripped =1;
		 //Spark.publish("DRIVEHEALTH","VFD entered in a tripped staate");
	//  }
*/
/*
		if (tripped=1)
		{
	// publish_data();
	// regs_dataf[16]=regs_data[17];
	   frozzen_data() ;
		 regs_data[17]=regs_dataf[16];
		 }
		 if  (tripped=0) {
			 publish_data();
       regs_dataf[16]= regs_data[17];
		 }
*/
//bool drive_healthy=
//}
previousMillis = currentMillis;
Serial.println("");
Serial.println("Waiting for next cycle..");
Serial.println("");
   }
}


//######################  PUBLISH_DATA  ############################
void publish_data() {
uint8_t k;
Serial.println(F("Starting Reading registers..."));
Serial.println("");
for (k = 0; k < 24; k++) {
 // getHoldingRegisterData( regs[k],  1,  data);
if (getHoldingRegisterData(  regs_CSK[k],  1,  data)) {//symetrical current limit in % 0.1 dec place
delay(10);
	 regs_data[k]=data[0];
	 Serial.printf("Register data : %d: ",  regs_CSK[k]);
	 Serial.print(regs_data[k], DEC);

	 Serial.println(F(" "));
 }
}
/*
String conversion
float myfloat=340000.0;
sprintf(spark_var2, "%s", String(myfloat, 2).c_str());

*/
String dati;
sprintf(motor_data,"{\"VFD_ID\":%d, \"Curr_mag\":%.2f, \"Curr_actv\":%.2f, \"Run_time\":%d, \"Motr_ovld_acc\":%.1f, \"Motr_load\":%.1f, \"Out_torq\":%.1f, \"Out_frec\":%.2f, \"Out_vol\":%d, \"Out_powr\":%.2f, \"Mot_speed\":%d, \"Pr_1815\":%.1f, \"Pr_1816\":%.1f}",DRIVEID, regs_data[1]/100.00, regs_data[2]/100.0, regs_data[3], regs_data[5]/10.0, regs_data[6]/10.0,regs_data[7]/10.0,regs_data[8]/100.0,regs_data[9], regs_data[10]/100.0,regs_data[11],regs_data[22]/10.0,regs_data[23]/10.0);
sprintf(vfd_data, "{\"VFD_ID\":%d, \"Sta_wrd\":%d,  \"Bus_volt\":%d, \"Stack_temp\":%d, \"VFD_temp\":%d, \"Vfd_Thermal_lev\":%d, \"Acc_kw\":%.2f, \"Pr_1811\":%d, \"Pr_1812\":%d, \"Pr_1813\":%d,\"Pr_1814\":%.3f}",DRIVEID, regs_data[17],regs_data[12],regs_data[13],regs_data[15],regs_data[0],regs_data[14]/100.0,regs_data[18],regs_data[19],regs_data[20],regs_data[21]/100.0);

Spark.publish("MOTORDATA",motor_data);
Spark.publish("VFDDATA",vfd_data);

//client.publish("VFDDATA",vfd_data);
Serial.println("");
//if(DEBUGON){
Serial.println(motor_data);
Serial.println("");
Serial.println(vfd_data);
Serial.println("");
//}
 if (client.publish("MOTORDATA",motor_data) & client.publish("VFDDATA",vfd_data)) {  //Mqtt publish data  (node-red)
	 white_led();
	 Serial.println("Publishing data to Mqtt");
  } else {
		    red_led();
		Serial.println("Mqtt Failed ..check Mqtt Broker");
 }
 Serial.println("");
//if(DEBUGON){

//}

temp_hum ();

}

//###################### END_PUBLISH_DATA ##########################


//###################### MODBUS ###############################################################


bool getHoldingRegisterData(uint16_t registerAddress, uint16_t regSize, int16_t*data){

uint8_t j, result;

if(DEBUGON){
  Serial.print(F("Reading register: "));
  Serial.print(registerAddress);
  Serial.print(F(" regSize: "));
  Serial.print(regSize);
  Serial.print(F(" sizeof(data): "));
  Serial.print(sizeof(&data));
  Serial.print(F(" XMITDELAY: "));
  Serial.println(XMITDELAY);
  Serial.println("");
}

// Delay and get register data.

result = node.readHoldingRegisters(registerAddress-BASED_NUMBERING, regSize);
delay(XMITDELAY);

// LT is sleeping, ping it a couple more times.

if(result ==node.ku8MBResponseTimedOut){

  client.publish("ERROR_MSG/message", "Response timed out"); //, MQTT::QOS2, &messageid);
  Serial.println("Response timed out..");
      node.clearResponseBuffer(); // if Timeout clearResponseBuffer and clearTransmitBuffer
      node.clearTransmitBuffer();
			//data[j] = 0;
        if(DEBUGON){
          Serial.println(F("LT: Response timed out. Trying again. "));
          client.publish("ERROR_MSG/message", "Response timed out"); //, MQTT::QOS2, &messageid);
          Serial.println("Response timed out");
        }

		int i =0;
}

if (result == node.ku8MBSuccess) {

  for (j = 0; j < regSize; j++) {
  //int16_t re1821= (int16_t)regs_data[18];
  //data[j] = node.getResponseBuffer(j);
    data[j] = (int16_t)node.getResponseBuffer(j);
    if(DEBUGON){
      Serial.print(data[j], DEC);
      Serial.print(F(" "));
    }
  }

  if(DEBUGON){
	  Serial.println("");
  }
  //uint pr617 =(uint)data[3];
//	Serial.print("PR617: ");
//	Serial.println(pr617);
  node.clearResponseBuffer();
  node.clearTransmitBuffer();
  return true;

}
else{
  if(DEBUGON){
	Serial.print(F("Failed, Response Code: "));
	Serial.println(result, HEX);
  }
  client.publish("ERROR_MSG/message", "NO RESPONSE FROM SLAVE"); //, MQTT::QOS2, &messageid);
  Serial.println("falla-no respuesta del esclavo");
//clear_regs ();
data[j] = 0;
}

node.clearResponseBuffer();
node.clearTransmitBuffer();
return false;
}


void frozzen_data() {
uint8_t u;
Serial.println(F("Starting Reading registers..."));
Serial.println("");
for (u = 0; u < 18; u++) {
 // getHoldingRegisterData( regs[k],  1,  data);
if (getHoldingRegisterData(frozen_regs[u] ,  1,  dataf)) {//symetrical current limit in % 0.1 dec place
delay(10);
	 regs_dataf[u]=dataf[0];
	 Serial.printf("Register data : %d: ",  frozen_regs[u]);
	 Serial.print(regs_dataf[u], DEC);

	 Serial.println(F(" "));
 }
}

sprintf(vfd_frzndata1,"{\"VFD_ID\":%d, \"Ref_sel\":%.2f, \"PreSk_fref\":%.2f, \"PreR_ref\":%.2f, \"Ref_rpm\":%d, \"Clp_ref\":%.2f, \"PostR_Ref\":%.2f, \"Finl_Dref\":%.2f, \"Ana1\":%.2f,\"Ana2\":%.2f}",DRIVEID, regs_dataf[0]/100.00, regs_dataf[1]/100.0, regs_dataf[2]/100.0, regs_dataf[3], regs_dataf[4]/100.0,regs_dataf[5]/100.0,regs_dataf[6]/100.0,regs_dataf[14]/100.0,regs_dataf[15]/100.0);
sprintf(vfd_frzndata2,"{\"VFD_ID\":%d, \"Curr_mag\":%.2f, \"Torq_pcurr\":%.2f, \"Magn_curr\":%.2f, \"Out_frec\":%.2f, \"Out_volt\":%d,\"Out_pwr\":%.2f, \"Dcb_volt\":%d ,\"Stt_wrd\":%d,\"Stack_temp\":%d}",DRIVEID, regs_dataf[7]/100.0, regs_dataf[8]/100.0,regs_dataf[9]/100.0,regs_dataf[10]/100.0,regs_dataf[11],regs_dataf[12]/100.0,regs_dataf[13],regs_dataf[16],regs_dataf[17]);
Spark.publish("VFD_TRIP_FROZEN_DATA1",vfd_frzndata1);
Spark.publish("VFD_TRIP_FROZEN_DATA2",vfd_frzndata2);



//client.publish("VFDDATA",vfd_data);
Serial.println("");
//if(DEBUGON){
Serial.println(vfd_frzndata1);
Serial.println("");
//}
 if (client.publish("VFD_TRIP_FROZEN_DATA1",vfd_frzndata1) & client.publish("VFD_TRIP_FROZEN_DATA2",vfd_frzndata2) ) {  //Mqtt publish data  (node-red)
	 white_led();
	 Serial.println("Publishing data to Mqtt");
  } else {
		    red_led();
		Serial.println("Mqtt Failed ..check Mqtt Broker");
 }
 Serial.println("");

//temp_hum ();

}









void callback(char* topic, byte* payload, unsigned int length) {
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = NULL;
    if (!strcmp(p, "DEBUGON"))
		    data_sent_ok_mqtt();  //blink a led
    else if (!strcmp(p, "DEBUGOFF"))
        led_patern1 (50);    // blink  a led
    delay(10);
}

// if application use QOS1 or QOS2, MQTT server sendback ack message id.
void qoscallback(unsigned int messageid) {
    Serial.print("Ack Message Id:");
    Serial.println(messageid);

    if (messageid == qos2messageid) {
        Serial.println("Release QoS2 Message");
        client.publishRelease(qos2messageid);
    }
}

void clear_regs () {
	uint8_t l ;
	for (l = 0; l < 24; l++) {
   //delay(20);
     regs_data[l]=0;
	 }

}

void led_test(){
  strip.setPixelColor(0, clear_led); strip.show();// set LED 10 to the color in variable c (red)
  strip.setPixelColor(0, red); strip.show();
  delay(80);
  strip.setPixelColor(0, clear_led); strip.show();
  delay(30);
  strip.setPixelColor(0, green); strip.show();
  delay(80);
  strip.setPixelColor(0, clear_led); strip.show();
  delay(30);
  strip.setPixelColor(0, blue); strip.show();
  delay(80);
  strip.setPixelColor(0, clear_led); strip.show();
  delay(30);
  strip.setPixelColor(0, white); strip.show();
  delay(80);
  strip.setPixelColor(0, clear_led); strip.show();
  }

void data_in_Ok(){
  strip.setPixelColor(0, clear_led); strip.show(); // set LED 10 to the color in variable c (red)
  strip.setPixelColor(1, clear_led); strip.show(); // set LED 10 to the color in variable c (red)

  strip.setPixelColor(0, green); strip.show();
  delay (40);
  strip.setPixelColor(0, clear_led); strip.show(); // set LED 10 to the color in variable c (red)
  delay(50);
   strip.setPixelColor(0, green); strip.show();
  delay (40);
  strip.setPixelColor(0, clear_led); strip.show(); // set LED 10 to the color in variable c (red)
  delay(50);
  strip.setPixelColor(0, green); strip.show();
  delay (40);
  strip.setPixelColor(0, clear_led); strip.show(); // set LED 10 to the color in variable c (red)
}

void data_sent_ok_mqtt(){
  strip.setPixelColor(0, clear_led); strip.show(); // set LED 10 to the color in variable c (red)
  strip.setPixelColor(1, clear_led); strip.show(); // set LED 10 to the color in variable c (red)

  strip.setPixelColor(0, blue); strip.show();
  delay (40);
  strip.setPixelColor(0, clear_led); strip.show(); // set LED 10 to the color in variable c (red)
  delay(50);
   strip.setPixelColor(0, blue); strip.show();
  delay (40);
  strip.setPixelColor(0, clear_led); strip.show(); // set LED 10 to the color in variable c (red)
  delay(50);
  strip.setPixelColor(0, blue); strip.show();
  delay (40);
  strip.setPixelColor(0, clear_led); strip.show(); // set LED 10 to the color in variable c (red)
}

void white_led(){
  strip.setPixelColor(0, clear_led); strip.show(); // set LED 10 to the color in variable c (red)
  strip.setPixelColor(1, clear_led); strip.show(); // set LED 10 to the color in variable c (red)

  strip.setPixelColor(0, white); strip.show();
  delay (60);
  strip.setPixelColor(0, clear_led); strip.show(); // set LED 10 to the color in variable c (red)
  delay(50);
   strip.setPixelColor(0, white); strip.show();
  delay (60);
  strip.setPixelColor(0, clear_led); strip.show(); // set LED 10 to the color in variable c (red)
  delay(50);
  strip.setPixelColor(0, white); strip.show();
  delay (60);
  strip.setPixelColor(0, clear_led); strip.show(); // set LED 10 to the color in variable c (red)
}
void red_led(){
  strip.setPixelColor(0, clear_led); strip.show(); // set LED 10 to the color in variable c (red)
  strip.setPixelColor(1, clear_led); strip.show(); // set LED 10 to the color in variable c (red)

  strip.setPixelColor(0, red); strip.show();
  delay (60);
  strip.setPixelColor(0, clear_led); strip.show(); // set LED 10 to the color in variable c (red)
  delay(50);
   strip.setPixelColor(0, red); strip.show();
  delay (60);
  strip.setPixelColor(0, clear_led); strip.show(); // set LED 10 to the color in variable c (red)
  delay(50);
  strip.setPixelColor(0, red); strip.show();
  delay (60);
  strip.setPixelColor(0, clear_led); strip.show(); // set LED 10 to the color in variable c (red)
}

void green_led(){
  strip.setPixelColor(0, clear_led); strip.show(); // set LED 10 to the color in variable c (red)
  strip.setPixelColor(1, clear_led); strip.show(); // set LED 10 to the color in variable c (red)

  strip.setPixelColor(0, green); strip.show();
  delay (60);
  strip.setPixelColor(0, clear_led); strip.show(); // set LED 10 to the color in variable c (red)
  delay(50);
   strip.setPixelColor(0, green); strip.show();
  delay (60);
  strip.setPixelColor(0, clear_led); strip.show(); // set LED 10 to the color in variable c (red)
  delay(50);
  strip.setPixelColor(0, green); strip.show();
  delay (60);
  strip.setPixelColor(0, clear_led); strip.show(); // set LED 10 to the color in variable c (red)
}

void temp_hum (){
	float h = dht.getHumidity();
	float f = dht.getTempFarenheit();
	// Check if any reads failed and exit early (to try again).
	//if (isnan(h) || isnan(f)) {
	//	Serial.println("Failed to read from DHT sensor!");
	//	return;
	//}
	float hi = dht.getHeatIndex();
 float dp = dht.getDewPoint();
 float k = dht.getTempKelvin();
/*
 Serial.println("");
 Serial.print("Humid: ");
 Serial.print(h);
 Serial.print("% - ");
 Serial.print("Temp: ");

 Serial.print(f);
 Serial.print("*F ");
 Serial.print(k);
 Serial.print("*K - ");
 Serial.print("DewP: ");
 Serial.print(dp);
 Serial.print("*C - ");
 Serial.print("HeatI: ");
 Serial.print(hi);
 Serial.println("*C");
 Serial.println(Time.timeStr());
 */

 sprintf(buf, "{\"VFD_ID\":%d, \"Humid\":%.2f, \"Temp_f\":%.2f, \"DewP_C\":%.2f, \"HeatI_C\":%.2f }", DRIVEID, h,f,dp,hi);
 Serial.println("");
 Serial.println(buf);
 client.publish("ENVIROMENT-DATA", buf);// "hello world QOS1", MQTT::QOS1, &messageid);
 Spark.publish("ENVIROMENT-DATA", buf);
}

void led_patern_setup(){
	strip.begin();                         // Initialize strip
	strip.show();        // (R,  G,  B)    // Update all LEDs (= turn OFF, since none of them have been set yet!)
	red       = strip.Color(255, 0, 0);    // Red
	green     = strip.Color(0, 255, 0);    // Green
	blue      = strip.Color(0, 0, 255);    // Blue
	white     = strip.Color(243, 63, 193); // White  was (0,255,100) \\\violeta
	clear_led = strip.Color(0, 0, 0);      // clear
	//patern1 test all colors
	color_patern1 [0]=clear_led;
	color_patern1 [1]=red;
	color_patern1 [2]=clear_led;
	color_patern1 [3]=green;
	color_patern1 [4]=clear_led;
	color_patern1 [5]=blue;
	color_patern1 [6]=clear_led;
	color_patern1 [7]=white;
	color_patern1 [8]=clear_led;

 //patern2 color white
	color_patern2 [0]=clear_led;
	color_patern2 [1]= white;
	color_patern2 [2]=clear_led;
	color_patern2 [3]= white;
	color_patern2 [4]=clear_led;
	color_patern2 [5]= white;
	color_patern2 [6]=clear_led;
 //patern3 color green
 color_patern3 [0]=clear_led;
 color_patern3 [1]= green;
 color_patern3 [2]=clear_led;
 color_patern3 [3]= green;
 color_patern3 [4]=clear_led;
 color_patern3 [5]= green;
 color_patern3 [6]=clear_led;
 //patern4 color blue
 color_patern4 [0]=clear_led;
 color_patern4 [1]= blue;
 color_patern4 [2]=clear_led;
 color_patern4 [3]= blue;
 color_patern4 [4]=clear_led;
 color_patern4 [5]= blue;
 color_patern4 [6]=clear_led;
 //patern5 color red
 color_patern5 [0]=clear_led;
 color_patern5 [1]= red;
 color_patern5 [2]=clear_led;
 color_patern5 [3]= red;
 color_patern5 [4]=clear_led;
 color_patern5 [5]= red;
 color_patern5 [6]=clear_led;



}
