#include <config.hpp>

#include <Arduino.h>
#include <functional>   //geil
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ledstuff.hpp>


#include <ArduinoTcpHardware.h>
#include <ros.h>
#include <std_msgs/Bool.h>

Lights lights;

const char* ssid = "Pilsbot_AP";
const char* password = "pilsbot_net";
const char* deviceName = "pilsbot_lights";
IPAddress staticIP(192, 168, 4, 5); //ESP static ip
IPAddress gateway(192, 168, 4, 1);   //IP Address of your WiFi Router (Gateway)
IPAddress subnet(255, 255, 255, 0);  //Subnet mask
IPAddress dns = gateway;  //DNS
IPAddress ros_server(192,168,4,1);          // Set the rosserial socket ROSCORE SERVER IP address
const uint16_t ros_serverPort = 11411;    // Set the rosserial socket server port

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Bool> il ("lighting/indicator/left",
    [](const std_msgs::Bool& msg){lights.setIndicatorLeft(msg.data);});
ros::Subscriber<std_msgs::Bool> ir ("lighting/indicator/right",
    [](const std_msgs::Bool& msg){lights.setIndicatorRight(msg.data);});
ros::Subscriber<std_msgs::Bool> br ("lighting/brake",
    [](const std_msgs::Bool& msg){lights.setBrake(msg.data);});
ros::Subscriber<std_msgs::Bool> hl ("lighting/headlight",
    [](const std_msgs::Bool& msg){lights.setHeadlight(msg.data);});
ros::Subscriber<std_msgs::Bool> pa ("lighting/party",
    [](const std_msgs::Bool& msg){lights.setParty(msg.data);});

void setupWiFi() {
    WiFi.hostname(deviceName);      // DHCP Hostname (useful for finding device for static lease)
    WiFi.config(staticIP, subnet, gateway, dns);
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        lights.blinkInfo(0x3effe7);
        delay(100);
    }
    lights.clear();
}

void setup()
{
    lights.init();
    setupWiFi();
    nh.getHardware()->setConnection(ros_server, ros_serverPort);
    nh.initNode();

    nh.subscribe(il);
    nh.subscribe(ir);
    nh.subscribe(br);
    nh.subscribe(hl);
    nh.subscribe(pa);

    //not yet connected to rosserial server
    while(!nh.connected())
    {
        nh.spinOnce();
        lights.blinkInfo(0xfe3fa7, true);
        delay(75);
    }
    lights.setParty(true);
}

void loop()
{
    nh.spinOnce();
    lights.update();
    delay(12);
}
