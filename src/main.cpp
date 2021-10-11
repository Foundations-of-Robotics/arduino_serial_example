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

// a structure holding a messsage packet received from serial
struct messageData {
    uint8_t motorID;   
    uint8_t commandID; 
    uint16_t _data;
};


//Recieves the data from the ROS pc and saves it to the micro controller
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
    
    char payloadFooter[payloadSize];
    _serialPort.readBytes(payloadFooter, payloadSize);
    //validates footer
    uint8_t footer = (uint8_t)payloadFooter[payloadSize-1];
    if(footer != 101){ 
        _serialPort.flush();
        return payload;     
    }
    //Saves recieved data and performs bitwise operations
    for(int i = 0; i < payloadSize / 4; i++) {
        payload.push_back((messageData){
            .motorID = (uint8_t)payloadFooter[i*4],
            .commandID = (uint8_t)payloadFooter[i * 4 + 1],
            ._data = (uint16_t)INT_JOIN_BYTE((uint8_t)payloadFooter[i*4+2], (uint8_t)payloadFooter[i*4+3])});
    }

    return payload;
}

//Sends the data back to ROS PC
void sendDataBackToRosSystem(vector<messageData> ToSendBack){
  uint8_t headerByte = 199;
  uint8_t payloadSize = ToSendBack.size()*4;
  uint8_t footer = 101;
  uint8_t arrayToSend[payloadSize+3];
  int counter = 0;
  arrayToSend[counter++] = headerByte;
  arrayToSend[counter++] = payloadSize;
// Advance the iterator by 2 positions
  for(int i = 0; i < ToSendBack.size(); i++) {
    arrayToSend[counter++] = (uint8_t)ToSendBack[i].motorID;
    arrayToSend[counter++] = (uint8_t)ToSendBack[i].commandID;
    arrayToSend[counter++] = (uint8_t)UPPER_BYTE(ToSendBack[i]._data);
    arrayToSend[counter++] = (uint8_t)LOWER_BYTE(ToSendBack[i]._data);
  }

  arrayToSend[counter++] = footer;

  _serialPort.write(arrayToSend,payloadSize+3);
}

//Start our main initialization code
void setup() {
  //start serial port
  _serialPort.begin(115200);
}

//Define a simple variable which holds our recieved serial messages from the PC
vector<messageData> recievedFromPC;

//Begin our loop of message recieving and interpolation
void loop() {

  //We simply wait for a serial message from the host PC
  if(_serialPort.available()>2){

    //read the data and save it to the micro controller
    recievedFromPC = readSerialCommunication();
    //Next we are simply sending the data back to the PC
    if(recievedFromPC.size()>0){
    sendDataBackToRosSystem(recievedFromPC);
    //empty the list
    recievedFromPC.clear();
    }
  }

}