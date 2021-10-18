/*
   RadioLib SX126x Transmit Example

   This example transmits packets using SX1262 LoRa radio module.
   Each packet contains up to 256 bytes of data, in the form of:
    - Arduino String
    - null-terminated char array (C-string)
    - arbitrary binary data (byte array)

   Other modules from SX126x family can also be used.

   For default module settings, see the wiki page
   https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx126x---lora-modem

   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/
*/

// include the library
#include <RadioLib.h>
#include <ros.h>
#include <nav_msgs/Odometry.h> //State-estimation message type

ros::NodeHandle nh: //ROS Node

float vel_x
float vel_y
float distance_x
float distance_y



// SX1262 has the following connections:
// NSS pin:   D36
// DIO1 pin:  D40
// NRST pin:  D44
// BUSY pin:  D39
SX1262 radio = new Module(D36, D40, D44, D39);

// or using RadioShield
// https://github.com/jgromes/RadioShield
//SX1262 radio = RadioShield.ModuleA;

void callback(const nav_msgs::Odometry::ConstPtr &msg){

  vel_x = msg.twist.twist.linear.x; // X-direction Velocity 
  vel_y = msg.twist.twist.linear.y; //Y-direction velocity
  distance_x = msg.pose.pose.position.x; //X-position
  distance_y = msg.pose.pose.position.y; //Y-position  
}

ros::Subscriber<nav_msgs::Odometry> sub("lora", &callback); // Subscriber

void setup() {
  Serial.begin(9600);

  nh.initNode(); // Initialzing the ROS node
  nh.getHardware()->setBaud(9600); // setting the baud rate 
  nh.subscribe(sub); // Subscribing to the code
  
  // initialize SX1262 with default settings
  Serial.print(F("[SX1262] Initializing ... "));
  int state = radio.begin(915.0, 250.0, 7, 5, 0x34, 20, 10, 0, false);;
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // some modules have an external RF switch
  // controlled via two pins (RX enable, TX enable)
  // to enable automatic control of the switch,
  // call the following method
  // RX enable:   4
  // TX enable:   5
  /*
    radio.setRfSwitchPins(4, 5);
  */
}

void loop() {
  Serial.print(F("[SX1262] Transmitting packet ... "));

  // you can transmit C-string or Arduino string up to
  // 256 characters long
  // NOTE: transmit() is a blocking method!
  //       See example SX126x_Transmit_Interrupt for details
  //       on non-blocking transmission method.

  // you can also transmit byte array up to 256 bytes long
    byte byteArr[] = {vel_x,vel_y,distance_x,distance_y};
    int state = radio.transmit(byteArr, 4);


  if (state == ERR_NONE) {
    // the packet was successfully transmitted
    Serial.println(F("success!"));

    // print measured data rate
    Serial.print(F("[SX1262] Datarate:\t"));
    Serial.print(radio.getDataRate());
    Serial.println(F(" bps"));

  } else if (state == ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("too long!"));

  } else if (state == ERR_TX_TIMEOUT) {
    // timeout occured while transmitting packet
    Serial.println(F("timeout!"));

  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);

  }
  
  nh.spinOnce();
  // wait for a second before transmitting again
  delay(1000);
}
