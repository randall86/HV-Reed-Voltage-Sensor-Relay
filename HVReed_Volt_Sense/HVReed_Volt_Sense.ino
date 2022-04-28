// HV Reed Voltage Sensor Relay System
// Rev 1.1 (24/04/2022)
// - Maxtrax

#include <Wire.h>
#include <Scheduler.h>
#include <Adafruit_MCP23008.h>
#include <DTIOI2CtoParallelConverter.h>
#include <ArduinoRS485.h>

#define NOP __asm__("nop\n\t") //"nop" executes in one machine cycle (at 16 MHz) yielding a 62.5 ns delay

const char * app_ver = "v1.1";

//Master commands
const char * ACK_STR = "ACK";
const char * NACK_STR = "NACK";
const char * QUERY_BRD_STR = "QBS";
const char * QUERY_CH_STR = "QCS";
const char * END_STR = "END";
const char * DBG_STR = "HDB";
const char DELIM = ',';
const byte CMD_LEN = 3;

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

const int MAX_BUFFERED_CMD = 32;

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

byte board_ID = 0;
byte check_bit = 0; //use bits to represent the interrupt triggerd expander
bool first_delim_found = false;
bool second_delim_found = false;
int delim_idx = 0;
int end_idx = 0;
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
            result += (String(1 + p_expander->pin_offset) + String(DELIM));
        }
        
        if( (pin_count++ < p_expander->max_pins) && (HIGH == p_expander->p_expander_port->pins.pin_1) )
        {
            result += (String(2 + p_expander->pin_offset) + String(DELIM));
        }
        
        if( (pin_count++ < p_expander->max_pins) && (HIGH == p_expander->p_expander_port->pins.pin_2) )
        {
            result += (String(3 + p_expander->pin_offset) + String(DELIM));
        }
        
        if( (pin_count++ < p_expander->max_pins) && (HIGH == p_expander->p_expander_port->pins.pin_3) )
        {
            result += (String(4 + p_expander->pin_offset) + String(DELIM));
        }
        
        if( (pin_count++ < p_expander->max_pins) && (HIGH == p_expander->p_expander_port->pins.pin_4) )
        {
            result += (String(5 + p_expander->pin_offset) + String(DELIM));
        }
        
        if( (pin_count++ < p_expander->max_pins) && (HIGH == p_expander->p_expander_port->pins.pin_5) )
        {
            result += (String(6 + p_expander->pin_offset) + String(DELIM));
        }
        
        if( (pin_count++ < p_expander->max_pins) && (HIGH == p_expander->p_expander_port->pins.pin_6) )
        {
            result += (String(7 + p_expander->pin_offset) + String(DELIM));
        }
        
        if( (pin_count++ < p_expander->max_pins) && (HIGH == p_expander->p_expander_port->pins.pin_7) )
        {
            result += (String(8 + p_expander->pin_offset) + String(DELIM));
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

void updateResult()
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
    
    yield(); //yield to pass control to other tasks
}

void resetBuffer()
{
    delim_idx = 0;
    end_idx = 0;
    first_delim_found = false;
    second_delim_found = false;
    memset(cmd_str, 0, MAX_BUFFERED_CMD);
}

void setup()
{
    Serial.begin(9600);
    //while (!Serial);
    
    randomSeed(analogRead(0));
    
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
    
    Scheduler.startLoop(updateResult); //update result from interrupt
}

void loop()
{
    //get request from Master
    while (RS485.available())
    {
        cmd_str[cmd_idx] = RS485.read();
        
        if (cmd_str[cmd_idx] == DELIM)
        {
            //1st delimiter found
            if (!first_delim_found)
            {
                delim_idx = cmd_idx;
                first_delim_found = true;
            }
            //2nd delimiter found
            else
            {
                end_idx = cmd_idx;
                second_delim_found = true;
            }
        }
        
        //2nd delimiter found and at least 3 bytes arrived after 2nd delimiter
        if ( (second_delim_found) && (cmd_idx >= end_idx + CMD_LEN) )
        {
            //check for END
            if (strncmp(&cmd_str[end_idx+1], END_STR, CMD_LEN) == 0)
            {
                //check for matching board ID
                int tmp_board_id = atoi(&cmd_str[delim_idx+1]);
                if (tmp_board_id == board_ID)
                {
                    //parse first 3 bytes of data for request type
                    if (strncmp(cmd_str, QUERY_BRD_STR, CMD_LEN) == 0)
                    {
                        sendRS485Data(const_cast<char *>(ACK_STR));
                    }
                    else if (strncmp(cmd_str, QUERY_CH_STR, CMD_LEN) == 0)
                    {
                        String reply = (String(board_ID) + String(DELIM) + result + String(END_STR));
                        sendRS485Data(const_cast<char *>(reply.c_str()));
                        result = ""; //clear the result after sending the reply
                    }
                    else if (strncmp(cmd_str, DBG_STR, CMD_LEN) == 0)
                    {
                        String reply = (String(board_ID) + String(DELIM) + String(random(1, 56)) + String(DELIM) + String(END_STR));
                        sendRS485Data(const_cast<char *>(reply.c_str()));
                    }
                    else //unknown command send NACK
                    {
                        sendRS485Data(const_cast<char *>(NACK_STR));
                    }
                }
                
                resetBuffer();
                cmd_idx = -1;
            }
        }

        cmd_idx++;
        
        if(cmd_idx >= MAX_BUFFERED_CMD) //buffer overflowing!
        {
            resetBuffer();
            cmd_idx = 0;
        }
    }
}