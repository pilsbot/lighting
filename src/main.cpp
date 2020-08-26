#include <config.hpp>

#include <Arduino.h>
#include <functional>   //geil
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ledstuff.hpp>
#include <IotWebConf.h>
#include <IotWebConfCompatibility.h>

#include <ArduinoTcpHardware.h>
#include <ros.h>
#include <std_msgs/Bool.h>

Lights lights;

const char deviceName[] = "pilsbot_lights";
const char initial_pw[] = "test1234";
#define STR_LEN 30
#define CONFIG_VERSION "Drei"
namespace cfg
{
    char staticIP[STR_LEN];
    char gateway[STR_LEN];
    char subnet[STR_LEN];
    char dns[STR_LEN];
    char ros_ip[STR_LEN];   // Set the rosserial socket ROSCORE SERVER IP address
    char ros_port[STR_LEN]; // Set the rosserial socket server port
}

void configSaved(){};
boolean formValidator();

DNSServer dnsServer;
WebServer server(80);
IotWebConf iotWebConf(deviceName, &dnsServer, &server, initial_pw, CONFIG_VERSION);
IotWebConfParameter staticIP = IotWebConfParameter("Static IP", "staticIP", cfg::staticIP, STR_LEN, "text", "192.168.4.5");
IotWebConfParameter gateway  = IotWebConfParameter("Gateway", "gateway", cfg::gateway, STR_LEN, "text", "192.168.4.1");
IotWebConfParameter subnet   = IotWebConfParameter("Subnet", "subnet", cfg::subnet, STR_LEN, "text", "255.255.255.0");
IotWebConfParameter dns      = IotWebConfParameter("DNS", "dns", cfg::dns, STR_LEN, "text", "192.168.4.1");
IotWebConfParameter ros_ip   = IotWebConfParameter("ROS Server IP", "ros_ip", cfg::ros_ip, STR_LEN, "text", "192.168.4.1");
IotWebConfParameter ros_port = IotWebConfParameter("ROS-Serial Port", "ros_port", cfg::ros_port, STR_LEN, "number", "11411");

/*
IPAddress staticIP(192, 168, 4, 5); //ESP static ip
IPAddress gateway(192, 168, 4, 1);   //IP Address of your WiFi Router (Gateway)
IPAddress subnet(255, 255, 255, 0);  //Subnet mask
IPAddress dns = gateway;  //DNS
IPAddress ros_server(192,168,4,1);
const uint16_t ros_serverPort = 11411;
*/

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

void handleRoot();

void setupWiFi() {
    iotWebConf.init();


    iotWebConf.addParameter(&staticIP);
    iotWebConf.addParameter(&gateway);
    iotWebConf.addParameter(&subnet);
    iotWebConf.addParameter(&dns);
    iotWebConf.addParameter(&ros_ip);
    iotWebConf.addParameter(&ros_port);
    iotWebConf.setConfigSavedCallback(&configSaved);
    iotWebConf.setFormValidator(&formValidator);
    iotWebConf.getApTimeoutParameter()->visible = true;

    bool valid_config = iotWebConf.init();

    server.on("/", []{ handleRoot(); });
    server.on("/config", []{ iotWebConf.handleConfig(); });
    server.onNotFound([](){ iotWebConf.handleNotFound(); });

    /*
    WiFi.hostname(deviceName);      // DHCP Hostname (useful for finding device for static lease)
    WiFi.config(staticIP, subnet, gateway, dns);
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        lights.blinkInfo(0x3effe7);
        delay(100);
    }
    */


    if(!valid_config)
    {
        while(true)
        {
            lights.blinkInfo(0x3effe7);
            iotWebConf.delay(100);
        }
    }
    lights.clear();
}

void setup()
{
    lights.init();
    setupWiFi();
    IPAddress ros_ip_addr;
    ros_ip_addr.fromString(cfg::ros_ip);
    nh.getHardware()->setConnection(ros_ip_addr , atoi(cfg::ros_port));
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
        iotWebConf.delay(75);
    }
    lights.clear();
    lights.setParty(true);
}

void loop()
{
    nh.spinOnce();
    lights.update();
    iotWebConf.delay(15);
}

bool formValidator()
{
    IPAddress ros_ip_addr;
    if(!ros_ip_addr.fromString(server.arg(ros_ip.getId())))
        return false;

    if(atoi(server.arg(ros_port.getId()).c_str()) < 1000)
        return false;
    return true;
}

void handleRoot(){  // -- Let IotWebConf test and handle captive portal requests.
    if (iotWebConf.handleCaptivePortal()) { return; }
    String page = "goto <a href=\"/config\">config</a></br>";
    page += "Current config:</br>";
    page += "Ros server: " + String(cfg::ros_ip) + ":" + String(cfg::ros_port);
    page += "</br> Dummer Test: IP 192.168.4.1 in IP address valid: ";

    IPAddress ros_ip_addr;
    if(!ros_ip_addr.fromString("192.168.4.1"))
        page += "No.";
    else
        page += "yes: " + ros_ip_addr.toString();

    page += "</br> Port 11411 valid: ";
    if(atoi("11411") < 1000)
        page += "No.";
    else
        page += "Yes.";

    server.send(200, "text/html", page);
}
