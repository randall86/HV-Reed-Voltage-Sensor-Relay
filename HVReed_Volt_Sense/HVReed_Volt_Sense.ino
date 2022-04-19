// HV Reed Voltage Sensor Relay System
// Rev 1.1 (19/04/2022)
// - Maxtrax

#include <Wire.h>
#include <Adafruit_MCP23008.h>
#include <DTIOI2CtoParallelConverter.h>
#include <ArduinoRS485.h>

#define NOP __asm__("nop\n\t") //"nop" executes in one machine cycle (at 16 MHz) yielding a 62.5 ns delay

const char * app_ver = "v1.1";

const byte LED_PIN = 16;

const byte RS485_RE = 0;
const byte RS485_DE = 0;
const byte RS485_RX = 13;
const byte RS485_TX = 14;

const byte IOEXP_RESET_PIN = 1;
const byte IOEXP_INT1_PIN = 4;
const byte IOEXP_INT2_PIN = 5;
const byte IOEXP_INT3_PIN = 6;
const byte IOEXP_INT4_PIN = 7;

const byte LEVEL_SHIFTER_OE = 3;

const byte TOTAL_IOEXP_PORTS = 8;

const int MAX_BUFFERED_CMD = 16;

const byte LF = 0x0A;

typedef union _port_t
{
    byte port;
    struct
    {
        byte pin_0 : 1;
        byte pin_1 : 1;
        byte pin_2 : 1;
        byte pin_3 : 1;
        byte pin_4 : 1;
        byte pin_5 : 1;
        byte pin_6 : 1;
        byte pin_7 : 1;
    }pins;
}port_t;

typedef struct _port_data_t
{
    port_t *p_expander_port;
    byte max_pins;
    byte pin_offset;
}port_data_t;

DTIOI2CtoParallelConverter ioExp1_U2(0x74);  //PCA9539 I/O Expander (with A1 = 0 and A0 = 0)
DTIOI2CtoParallelConverter ioExp2_U3(0x75);  //PCA9539 I/O Expander (with A1 = 0 and A0 = 1)
DTIOI2CtoParallelConverter ioExp3_U4(0x76);  //PCA9539 I/O Expander (with A1 = 1 and A0 = 0)
DTIOI2CtoParallelConverter ioExp4_U5(0x77);  //PCA9539 I/O Expander (with A1 = 1 and A0 = 1)

Adafruit_MCP23008 ioExp_MCP;

byte board_ID = 0; //own board ID
byte check_bit = 0; //use bits to represent the interrupt triggerd expander
int cmd_idx = 0;
char cmd_str[MAX_BUFFERED_CMD] = {};
String result = "";

//DUT not short - input pin values high
//DUT shorted - input pin values low
port_t ioExp1_port0 = {0};
port_t ioExp1_port1 = {0};
port_t ioExp2_port0 = {0};
port_t ioExp2_port1 = {0};
port_t ioExp3_port0 = {0};
port_t ioExp3_port1 = {0};
port_t ioExp4_port0 = {0};
port_t ioExp4_port1 = {0};
port_data_t expander_mapping[TOTAL_IOEXP_PORTS] = { {&ioExp1_port0, 8, 0}, {&ioExp1_port1, 6, 8},
                                                    {&ioExp2_port0, 8, 14}, {&ioExp2_port1, 6, 22},
                                                    {&ioExp3_port0, 8, 28}, {&ioExp3_port1, 6, 36},
                                                    {&ioExp4_port0, 8, 42}, {&ioExp4_port1, 6, 50} };

void sendRS485Data(char data[])
{
    RS485.beginTransmission();
    
    digitalWrite(LED_PIN, HIGH);
    RS485.println(data);
    digitalWrite(LED_PIN, LOW);
    
    RS485.endTransmission();
}

void resetLevelShifter()
{
    RS485.noReceive();
    
    digitalWrite(LEVEL_SHIFTER_OE, LOW);
    NOP; //60ns delay
    digitalWrite(LEVEL_SHIFTER_OE, HIGH);
    delayMicroseconds(1); //only need 200ns
    
    RS485.receive();
}

void resetIOExpanders()
{
    digitalWrite(IOEXP_RESET_PIN, LOW);
    NOP; //60ns delay
    digitalWrite(IOEXP_RESET_PIN, HIGH);
    delayMicroseconds(1); //only need 400ns
}

void checkIOExpanderPins(port_data_t *p_expander)
{
    if(NULL != p_expander)
    {
        byte pin_count = 0;
        if( (pin_count++ < p_expander->max_pins) && (HIGH == p_expander->p_expander_port->pins.pin_0) )
        {
            result += (String(1 + p_expander->pin_offset) + String(","));
        }
        
        if( (pin_count++ < p_expander->max_pins) && (HIGH == p_expander->p_expander_port->pins.pin_1) )
        {
            result += (String(2 + p_expander->pin_offset) + String(","));
        }
        
        if( (pin_count++ < p_expander->max_pins) && (HIGH == p_expander->p_expander_port->pins.pin_2) )
        {
            result += (String(3 + p_expander->pin_offset) + String(","));
        }
        
        if( (pin_count++ < p_expander->max_pins) && (HIGH == p_expander->p_expander_port->pins.pin_3) )
        {
            result += (String(4 + p_expander->pin_offset) + String(","));
        }
        
        if( (pin_count++ < p_expander->max_pins) && (HIGH == p_expander->p_expander_port->pins.pin_4) )
        {
            result += (String(5 + p_expander->pin_offset) + String(","));
        }
        
        if( (pin_count++ < p_expander->max_pins) && (HIGH == p_expander->p_expander_port->pins.pin_5) )
        {
            result += (String(6 + p_expander->pin_offset) + String(","));
        }
        
        if( (pin_count++ < p_expander->max_pins) && (HIGH == p_expander->p_expander_port->pins.pin_6) )
        {
            result += (String(7 + p_expander->pin_offset) + String(","));
        }
        
        if( (pin_count++ < p_expander->max_pins) && (HIGH == p_expander->p_expander_port->pins.pin_7) )
        {
            result += (String(8 + p_expander->pin_offset) + String(","));
        }
    }
}

void ioExp1InterruptHandler()
{
    ioExp1_U2.digitalReadPort0(ioExp1_port0.port);
    ioExp1_U2.digitalReadPort1(ioExp1_port1.port);
    check_bit |= 0x03; 
}

void ioExp2InterruptHandler()
{
    ioExp2_U3.digitalReadPort0(ioExp2_port0.port);
    ioExp2_U3.digitalReadPort1(ioExp2_port1.port);
    check_bit |= 0x0C; 
}

void ioExp3InterruptHandler()
{
    ioExp3_U4.digitalReadPort0(ioExp3_port0.port);
    ioExp3_U4.digitalReadPort1(ioExp3_port1.port);
    check_bit |= 0x30;
}

void ioExp4InterruptHandler()
{
    ioExp4_U5.digitalReadPort0(ioExp4_port0.port);
    ioExp4_U5.digitalReadPort1(ioExp4_port1.port);
    check_bit |= 0xC0;
}

void setup()
{
    Serial.begin(9600);
    //while (!Serial);
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    
    Serial.print("HV Reed Voltage Sensor Relay");
    Serial.println(app_ver);
    
    RS485.setPins(RS485_TX, RS485_DE, RS485_RE);
    RS485.begin(9600);
    RS485.receive();
    
    Wire.begin(); //need to start the Wire for I2C devices to function
    
    ioExp_MCP.begin(0x20);  //MCP23008 (with A2 = 0, A1 = 0 and A0 = 0)
    
    // configure all pins as INPUT with INTERNAL PULLUP - turn on a 100K pullup internally
    for (byte pin = 0; pin < 8; pin++)
    {
        ioExp_MCP.pinMode(pin, INPUT);
        ioExp_MCP.pullUp(pin, HIGH); 
    }
    
    //get the board ID
    board_ID = ioExp_MCP.readGPIO();
    
    pinMode(IOEXP_RESET_PIN, OUTPUT);
    resetIOExpanders();
    
    pinMode(LEVEL_SHIFTER_OE, OUTPUT);
    resetLevelShifter();
    
    ioExp1_U2.portMode0(ALLINPUT);
    ioExp1_U2.portMode1(ALLINPUT);
    
    ioExp2_U3.portMode0(ALLINPUT);
    ioExp2_U3.portMode1(ALLINPUT);
    
    ioExp3_U4.portMode0(ALLINPUT);
    ioExp3_U4.portMode1(ALLINPUT);
    
    ioExp4_U5.portMode0(ALLINPUT);
    ioExp4_U5.portMode1(ALLINPUT);
    
    pinMode(IOEXP_INT1_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IOEXP_INT1_PIN), ioExp1InterruptHandler, CHANGE);
    
    pinMode(IOEXP_INT2_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IOEXP_INT2_PIN), ioExp2InterruptHandler, CHANGE);
    
    pinMode(IOEXP_INT3_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IOEXP_INT3_PIN), ioExp3InterruptHandler, CHANGE);
    
    pinMode(IOEXP_INT4_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IOEXP_INT4_PIN), ioExp4InterruptHandler, CHANGE);
    
    digitalWrite(LED_PIN, LOW);
}

void loop()
{
    //interrupt triggered process the read data here
    while(0 != check_bit)
    {
        byte port_mask = 0;
        do
        {
            //loop for which expander interrupt triggered
            if(check_bit & (1 << port_mask))
            {
                checkIOExpanderPins(&expander_mapping[port_mask]);
                check_bit &= ~(1 << port_mask); //clear bit after handle
            }
        }while( (++port_mask < TOTAL_IOEXP_PORTS) && (0 != check_bit) );
    }
    
    //get request from Master
    if (RS485.available())
    {
        cmd_str[cmd_idx] = RS485.read();
        if (cmd_str[cmd_idx] == LF)
        {
            cmd_str[cmd_idx-1] = 0;
            Serial.println(cmd_str); //the request command from Master
            cmd_idx = -1;
            
            String reply = (String(board_ID) + String(",") + result);
            reply.setCharAt(reply.length(), 'S'); //use 'S' for END char
            sendRS485Data(const_cast<char *>(reply.c_str()));
            result = ""; //clear the result after sending the reply
        }
        cmd_idx++;
    }
}