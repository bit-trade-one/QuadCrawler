// copyright to SohtaMei 2019.


#include <Arduino.h>
#include <EEPROM.h>                     // for quadCrawlerRemocon
#include "quadCrawlerRemocon.h"

#include <Wire.h>                       // for Adafruit_PWMServoDriver
#include <Adafruit_PWMServoDriver.h>    // for quadCrawler
#include <Adafruit_NeoPixel.h>          // for quadCrawler
#include "quadCrawler.h"

#define mVersion "QuadCrawler1.0"

const uint8_t sw_table[4] = {3,4,5,6};


enum {
    RSP_BYTE    = 1,
    RSP_SHORT   = 2,
    RSP_LONG    = 3,
    RSP_FLOAT   = 4,
    RSP_DOUBLE  = 5,
    RSP_STRING  = 6,
};

void setup()
{
    
    remoconRobo_init();
    digitalWrite(13, HIGH);
    Serial.begin(115200);
    delay(500);
    digitalWrite(13, LOW);
    
    quadCrawler_init();
    quadCrawler_colorWipe(COLOR_PURPLE);
    quadCrawler_beep(100);
    
    Serial.println("PC mode: " mVersion);
}

#define getByte(n)      (buffer[4+n])
#define getShort(n)     (buffer[4+n]|(buffer[5+n]<<8))
#define getLong(n)      (buffer[4+n]|(buffer[5+n]<<8UL)|(buffer[6+n]<<16UL)|(buffer[7+n]<<24UL))
static uint8_t buffer[52];

static void parseData()
{
    switch(buffer[3]){
        case 1: quadCrawler_Walk(getByte(1),getByte(0));; callOK(); break;
        case 2: quadCrawler_setPose1(0,getByte(0),getByte(1));; callOK(); break;
        case 3: quadCrawler_setPose1(1,getByte(0),getByte(1));; callOK(); break;
        case 4: quadCrawler_setPose1(2,getByte(0),getByte(1));; callOK(); break;
        case 5: quadCrawler_setPose1(3,getByte(0),getByte(1));; callOK(); break;
        case 6: quadCrawler_Walk(200,0);; callOK(); break;
        case 7: quadCrawler_colorWipe(getByte(0));; callOK(); break;
        case 8: quadCrawler_rainbow(getByte(0));; callOK(); break;
        case 9: quadCrawler_beep(getShort(0));; callOK(); break;
        case 10: sendFloat((quadCrawler_getSonner())); break;
        case 11: sendByte((((getByte(0)>=1&&getByte(0)<=4)?digitalRead(sw_table[getByte(0)-1]):0)==0)); break;
        case 19: pinMode(13,OUTPUT);digitalWrite(13,getByte(0));; callOK(); break;
        case 20: pinMode(getByte(0),OUTPUT);digitalWrite(getByte(0),getByte(1));; callOK(); break;
        case 21: pinMode(A0+getByte(0),OUTPUT);digitalWrite(A0+getByte(0),getByte(1));; callOK(); break;
        case 22: sendByte((pinMode(getByte(0),INPUT),digitalRead(getByte(0)))); break;
        case 23: sendByte((pinMode(A0+getByte(0),INPUT),digitalRead(A0+getByte(0)))); break;
        case 24: sendShort((pinMode(A0+getByte(0),INPUT),remoconRobo_getAnalog(A0+getByte(0),1))); break;
        case 25: sendShort((pinMode(A0+getByte(0),INPUT),remoconRobo_getAnalog(A0+getByte(0),getShort(1)))); break;
        
        //### CUSTOMIZED ###
        #ifdef REMOTE_ENABLE	// check remoconRoboLib.h or quadCrawlerRemocon.h
        #define CMD_CHECKREMOTEKEY  0x80
        case CMD_CHECKREMOTEKEY:
        sendRemote();
        break;
        #endif
    }
}

static uint8_t index = 0;
static uint8_t _packetLen = 4;

void loop()
{
    if(Serial.available()>0){
        uint8_t c = Serial.read();
        buffer[index++] = c;
        
        switch(index) {
            case 1:
            _packetLen = 4;
            if(c != 0xff)
            index = 0;
            break;
            case 2:
            if(c != 0x55) 
            index = 0;
            break;
            case 3:
            _packetLen = 3+c;
            break;
        }
        if(index >= _packetLen) {
            parseData();
            index = 0;
        }
        if(index >= sizeof(buffer)) {
            index = 0;
        }
    }
    
    quadCrawler_servoLoop();
    
}

union floatConv { 
    float _float;
    uint8_t _byte[4];
} conv;

union doubleConv {
    double _double;
    uint8_t _byte[8];
};

float getFloat(uint8_t n)
{
    union floatConv conv;
    for(uint8_t i=0; i<4; i++) {
        conv._byte[i] = buffer[4+n+i];
    }
    return conv._float;
}

double getDouble(uint8_t n)
{
    union doubleConv conv;
    for(uint8_t i=0; i<8; i++) {
        conv._byte[i] = buffer[4+n+i];
    }
    return conv._double;
}

char* getString(uint8_t n)
{
    return (char*)buffer+4+n;
}

static void callOK()
{
    Serial.write(0xff);
    Serial.write(0x55);
    Serial.write(0);
}

static void sendByte(uint8_t data)
{
    Serial.write(0xff);
    Serial.write(0x55);
    Serial.write(1+sizeof(uint8_t));
    Serial.write(RSP_BYTE);
    Serial.write(data);
}

static void sendShort(uint16_t data)
{
    Serial.write(0xff);
    Serial.write(0x55);
    Serial.write(1+sizeof(uint16_t));
    Serial.write(RSP_SHORT);
    Serial.write(data&0xff);
    Serial.write(data>>8);
}

static void sendLong(uint32_t data)
{
    Serial.write(0xff);
    Serial.write(0x55);
    Serial.write(1+sizeof(uint32_t));
    Serial.write(RSP_LONG);
    Serial.write(data&0xff);
    Serial.write(data>>8);
    Serial.write(data>>16);
    Serial.write(data>>24);
}

static void sendFloat(float data)
{
    union floatConv conv;
    conv._float = data;
    
    Serial.write(0xff);
    Serial.write(0x55);
    Serial.write(1+sizeof(float));
    Serial.write(RSP_FLOAT);
    Serial.write(conv._byte[0]);
    Serial.write(conv._byte[1]);
    Serial.write(conv._byte[2]);
    Serial.write(conv._byte[3]);
}

static void sendDouble(double data)
{
    union doubleConv conv;
    conv._double = data;
    
    Serial.write(0xff);
    Serial.write(0x55);
    Serial.write(1+sizeof(double));
    Serial.write(RSP_DOUBLE);
    for(uint8_t i=0; i<8; i++) {
        Serial.write(conv._byte[i]);
    }
}

static void sendString(String s)
{
    uint8_t l = s.length();
    
    Serial.write(0xff);
    Serial.write(0x55);
    Serial.write(1+l);
    Serial.write(RSP_STRING);
    for(uint8_t i=0; i<l; i++) {
        Serial.write(s.charAt(i));
    }
}

//### CUSTOMIZED ###
#ifdef REMOTE_ENABLE	// check remoconRoboLib.h or quadCrawlerRemocon.h
static void sendRemote(void)
{
    uint16_t data;
    Serial.write(0xff);
    Serial.write(0x55);
    Serial.write(1+1+2+2);
    Serial.write(CMD_CHECKREMOTEKEY);
    Serial.write(remoconRobo_checkRemoteKey());
    data = remoconRobo_getRemoteX();
    Serial.write(data&0xff);
    Serial.write(data>>8);
    data = remoconRobo_getRemoteY();
    Serial.write(data&0xff);
    Serial.write(data>>8);
}
#endif
