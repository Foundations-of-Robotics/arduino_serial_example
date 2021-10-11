#include <Arduino.h>
#include <ArduinoSTL.h>
#include <vector>

using namespace std;

//Define our serial port
#define _serialPort Serial
//Define the bitwise functions for message interpretation
#define INT_JOIN_BYTE(u, l) (u << 8) | l
#define LOWER_BYTE(b) b & 0xff
#define UPPER_BYTE(b) (b >> 8)

// a structure holdinga messsage packet received from serial
struct messageData {
    uint8_t motorID;   
    uint8_t commandID; 
    uint16_t _data;
};

vector<messageData> readSerialCommunication(){
    //declare our payload variable which holds the data we will be returning
    vector<messageData> payload;

    //Read and save the first two Serial messages
    char header[2];
    _serialPort.readBytes(header, 2);
        
    uint8_t zeroHeader = (uint8_t)header[0];
    uint8_t payloadSize = (uint8_t)header[1];

    //checks if our header is 199, remembering that's what we defined our header value as
    if(zeroHeader != 199){ 
        _serialPort.flush();
        return payload;     
    }
    
    char payloadFooter[payloadSize + 1];
    _serialPort.readBytes(payloadFooter, payloadSize + 1);

    if(payloadFooter[payloadSize] != 101){ 
        _serialPort.flush();
        return payload;     
    }

    for(int i = 0; i < payloadSize / 4; i++) {
        payload.push_back((messageData){
            .motorID = (uint8_t)payloadFooter[i*4],
            .commandID = (uint8_t)payloadFooter[i * 4 + 1],
            ._data = (uint16_t)INT_JOIN_BYTE((uint8_t)payloadFooter[i*4+2], (uint8_t)payloadFooter[i*4+3])});
    }

    return payload;
}

void sendDataBackToRosSystem(vector<messageData> ToSendBack){
  uint8_t headerByte = 199;
  uint8_t payloadSize = ToSendBack.size()*4;
  uint8_t footer = 101;
  uint8_t arrayToSend[payloadSize+3];
  arrayToSend[0] = headerByte;
  arrayToSend[1] = payloadSize;
  arrayToSend[payloadSize+2] = footer;

  int startIndex = 2;
// Advance the iterator by 2 positions
  for(int i = 0; i < payloadSize; i++) {
    arrayToSend[startIndex+(i*4)] = (uint8_t)ToSendBack[i].motorID;
    arrayToSend[startIndex+(i*4)+1] = (uint8_t)ToSendBack[i].commandID;
    arrayToSend[startIndex+(i*4)+2] = (uint8_t)UPPER_BYTE(ToSendBack[i]._data);
    arrayToSend[startIndex+(i*4)+3] = (uint8_t)LOWER_BYTE(ToSendBack[i]._data);
  }

  _serialPort.write(arrayToSend,(int)payloadSize+3);
}

//Start our main initialization code
void setup() {
  //Start the serial port with a baud rate of 115200
  
  _serialPort.begin(115200);
  _serialPort.println("This is a test line of code to start out communication with and validate our sketch works");
}

//Define a simple variable which holds our recieved serial messages from the PC
vector<messageData> recievedFromPC;

//Begin our loop of message recieving and interpolation
void loop() {

  //We simply wait for a serial message from the host PC
  if(_serialPort.available()>2){

    //read the data and save it to the micro controller
    recievedFromPC = readSerialCommunication();
    //Next we are simply sending the data back to command
    if(recievedFromPC.size()>0)
    sendDataBackToRosSystem(recievedFromPC);
  }

}