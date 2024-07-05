# ModBusslavenode
#include <avr/interrupt.h> 
#include <avr/io.h> 
#define myubbr (16000000/16/9600-1)
volatile char ReceivedChar;

unsigned char SC;  // slave code
unsigned char SerialData[8]; //incoming data from UART
unsigned char i=0;

byte QA; // quantity of outputs
byte in=0;

unsigned short int checkSum; //is returned in CRC16
byte checkSumHigh; // high byte of checksum
byte checkSumLow; // low byte of checksum
byte coilLength;
byte startNumberFirstCoil;
byte sendData[54];
unsigned char digital[24];

byte digital_output[2];

unsigned char sendDataLength = 0;
unsigned char j=0;
unsigned char readHoldingRegisterNumber=0;
unsigned char readHoldingRegiserStart=0;

unsigned char readCoil_FC = 1; //modbus readCoil register number
unsigned char readInputstatus_FC = 2; //modbus read input status
unsigned char forcesinglecoil_FC = 5; //modbus force single coil 
byte RxGo=0; // if RxGo = 1 then communication is available without exception
byte readCoilGo = 0;
byte read_input = 0;
byte force_coil = 0;
byte readHoldingRegisterGo=0;
char a=240;
double remainder;  // to find how many coil as a byte to read ( for instance 14 coil is need to read, that is 2 byte )
// all possible reminders for checksum
unsigned char auchCRCLo[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
};

unsigned char auchCRCHi[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
};


boolean ADCgo = 0;
unsigned char ba = 0;

const int Dpin1 =  22; // digital input pin 1
const int Dpin2 =  23; // digital input pin 2
const int Dpin3 =  24; // digital input pin 3
const int Dpin4 =  25; // digital input pin 4
const int Dpin5 =  26; // digital input pin 5
const int Dpin6 =  27; // digital input pin 6
const int Dpin7 =  28; // digital input pin 7
const int Dpin8 =  29; // digital input pin 8
const int Dpin9 =  30; // digital input pin 9
const int Dpin10 =  31; // digital input pin 10
const int Dpin11 =  32; // digital input pin 11
const int Dpin12 =  33; // digital input pin 12
const int Dpin13 =  34; // digital input pin 13
const int Dpin14 =  35; // digital input pin 14
const int Dpin15 =  36; // digital input pin 15
const int Dpin16 =  37; // digital input pin 16

const int ena1 = 3; 
const int ena2 = 2;

const int output1 = 6;
const int output2 = 7;
const int output3 = 8;
const int output4 = 9;
const int output5 = 10;
const int output6 = 11;
const int output7 = 12;
const int output8 = 13;


void setup()
{
  SC = 3;
  UBRR0H = (unsigned char)(myubbr>>8);
  UBRR0L = (unsigned char)myubbr;
   //UCSR0C |= (1 << UCSZ00) | (1 << UCSZ10); // Use 8-bit character sizes 
  // UCSR0C |= (1 << UCSZ10) | (1 << UCSZ11) | (0 << UCSZ12);
  UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);   // Turn on the transmission, reception, and Receive interrupt       
    interrupts();
    pinMode(Dpin1, INPUT_PULLUP);
    pinMode(Dpin2, INPUT_PULLUP);
    pinMode(Dpin3, INPUT_PULLUP);
    pinMode(Dpin4, INPUT_PULLUP);
    pinMode(Dpin5, INPUT_PULLUP);
    pinMode(Dpin6, INPUT_PULLUP);
    pinMode(Dpin7, INPUT_PULLUP);
    pinMode(Dpin8, INPUT_PULLUP);

    pinMode(Dpin9,  INPUT_PULLUP);
    digitalWrite(Dpin1, HIGH);
    digitalWrite(Dpin2, HIGH);
    digitalWrite(Dpin3, HIGH);
    digitalWrite(Dpin4, HIGH);
    digitalWrite(Dpin5, HIGH);
    digitalWrite(Dpin6, HIGH);
    digitalWrite(Dpin7, HIGH);
    digitalWrite(Dpin8, HIGH);
    digitalWrite(Dpin9, HIGH);
 
    
    pinMode(Dpin10, OUTPUT);
    pinMode(Dpin11, OUTPUT);
    pinMode(Dpin12, OUTPUT);
    pinMode(Dpin13, OUTPUT);
    pinMode(Dpin14, OUTPUT);
    pinMode(Dpin15, OUTPUT);
    pinMode(Dpin16, OUTPUT);
    
    pinMode(output1, OUTPUT);
    pinMode(output2, OUTPUT); 
    pinMode(output3, OUTPUT); 
    pinMode(output4, OUTPUT);
    pinMode(output5, OUTPUT);
    pinMode(output6, OUTPUT);
    pinMode(output7, OUTPUT); 
    pinMode(output8, OUTPUT); 
    pinMode(ena1, OUTPUT);   
    pinMode(ena2, OUTPUT);
    //pinMode(RS485_EN_R, OUTPUT); 
    
    
    
    
    digitalWrite(output1, LOW);
    digitalWrite(output2, LOW);
    digitalWrite(output3, LOW);
    digitalWrite(output4, LOW);
    digitalWrite(output5, LOW);
    digitalWrite(output6, LOW);
    digitalWrite(output7, LOW);
    digitalWrite(output8, LOW);
    digitalWrite(ena2, LOW);   // receive
    digitalWrite(ena1, HIGH);   // transmit
    
}

void loop()
{
  digital[0]  = digitalRead(Dpin1);
  digital[1]  = digitalRead(Dpin2); 
  digital[2]  = digitalRead(Dpin3);
  digital[3]  = digitalRead(Dpin4);
  digital[4]  = digitalRead(Dpin5);
  digital[5]  = digitalRead(Dpin6);
  digital[6]  = digitalRead(Dpin7);
  digital[7]  = digitalRead(Dpin8);
  digital[8]  = digitalRead(Dpin9);
  digital[9]  = digitalRead(Dpin10);
  digital[10] = digitalRead(Dpin11);
  digital[11] = digitalRead(Dpin12);
  digital[12] = digitalRead(Dpin13);
  digital[13] = digitalRead(Dpin14);
  digital[14] = digitalRead(Dpin15);
  digital[15] = digitalRead(Dpin16);

  digital[16] = 1;
  digital[17] = 0;
  digital[18] = 0;
  digital[19] = 0;
  digital[20] = 1;
  digital[21] = 1;
  digital[22] = 0;
  digital[23] = 1;
 //digitalWrite(output8, HIGH);
// delay(10);
// digitalWrite(output8, LOW);
// delay(10);
  
    if(RxGo == 1) // transmission will start
    {
      //if (SerialData[1] == readCoil){
      RxGo = 0;
      sendData[0] = SerialData[0]; // assign first element of the receiving data ( Slave Code ) to the first element of the tramsmitted data
      sendData[1] = SerialData[1]; // assing second element of the receiving data ( Function Code ) to the second element of the transmitted data
      
    /*----------------------------- Start Of ReadCoil --------------------------------------------- */
      if(readCoilGo == 1){  // start of readCoil    
      readCoilGo = 0;
    //  remainder = (SerialData[4]*256 + SerialData[5])*0.125; // to find how many coils as a byte
      remainder = (QA)*0.125; // to find how many coils as a byte
      if((remainder) == 0)
      {
      sendData[2] = remainder;
      }
      else if(remainder  <= 1)
      {
       sendData[2] = 1; 
       }
      else if(remainder  == 2)
      {
       sendData[2] = 2;
       }
      else if(remainder  == 3)
      { sendData[2] = 3;
      }
      else
      {
      sendData[2] = (unsigned short int)(remainder+1);
      }
 
      sendDataLength=3; // SerialData[0]+Serialdata[1]+SerialData[2] 
    // Read Digital Inputs - start - Depending on sendData[2] ( number of byte of the sending data) 
      coilLength = SerialData[4]*256+SerialData[5]; // number of coil wanted to read
      startNumberFirstCoil = SerialData[2]*256+SerialData[3]; // first number of coil (-1 çıkartıldı)

      if(coilLength > 24 ){coilLength = 24;}
      if(sendData[2] == 1){
        sendDataLength=4;
        for(i=0;i < (coilLength); i++){    
          digital[( (startNumberFirstCoil) + i )] = digital[( (startNumberFirstCoil)  + i)] << ( i );
            
            if(i == ( (coilLength) -1) ){
            
              for(i=0; i<(coilLength); i++ ){
              
                sendData[sendDataLength-1] = sendData[sendDataLength-1] + digital[(startNumberFirstCoil)  + i];
              
              }
            
            }
        }         
      }
      else if(sendData[2] == 2){
        sendDataLength = 5;
        for(i=0;i < (coilLength); i++){          
          if(i<8)
          {
            digital[( (startNumberFirstCoil) + i )] = digital[( (startNumberFirstCoil)  + i)] << ( i );
          }
          else
          {
            digital[( (startNumberFirstCoil) + i )] = digital[( (startNumberFirstCoil)  + i)] << ( i-8 );
          }          
              if(i == 7 ){          
                for(j=0; j<8; j++ ){             
                  sendData[sendDataLength-2] = sendData[sendDataLength-2] + digital[(startNumberFirstCoil)  + j];    
                }  
              }
              if(i == ( (coilLength) -1) ){
            
                for(i=8; i<(coilLength); i++ ){
              
                  sendData[sendDataLength-1] = sendData[sendDataLength-1] + digital[(startNumberFirstCoil)  + i ];
              
                }
            
              }
        }              
      }
      else if(sendData[2] == 3){
      
        sendDataLength = 6;

        for(i=0;i < (coilLength); i++)
        {
          
        if(i<8)
        {
        
                digital[( (startNumberFirstCoil) + i )] = digital[( (startNumberFirstCoil) + i )] << ( i );
        
        }else if(i>7 && i<16){
        
                digital[( (startNumberFirstCoil) + i  )] = digital[( (startNumberFirstCoil) + i )] << ( i-8 );
        
        
        }else if(i>15){ digital[( (startNumberFirstCoil) + i  )] = digital[( (startNumberFirstCoil) + i )] << ( i-16 );
        
        
        } 

        
              if(i == 7 ){
            
                for(j=0; j<8; j++ ){
              
                sendData[sendDataLength-3] = sendData[sendDataLength-3] + digital[(startNumberFirstCoil)  + j];
              
                }
            
              }

              if(i==15){
              
                for(j=8; j<16; j++ ){
              
                sendData[sendDataLength-2] = sendData[sendDataLength-2] + digital[(startNumberFirstCoil)  + j];
              
                }
              
              
              }

              if(i == (coilLength-1) )  {
              
                for(i=16; i<(coilLength); i++ ){
              
                sendData[sendDataLength-1] = sendData[sendDataLength-1] + digital[(startNumberFirstCoil)  + i ];
              
                }
                         
              }
        
            }

      }
      // Read Digital Inputs - end

      } //end of readCoil  -- readCoilGo is assigned to zero (readCoilgo = 0)

     if(read_input == 1){  // start of read digital input    
      read_input = 0;
      sendData[2] = 1;   
      sendData[3] = 0x00;
      //sendData[3] = 0xFF;
      sendDataLength = 4;
        for(i=0;i < 8; i++){    
         digital[i] = digital[i] << (i);    
          }     
             for(i=0; i<8; i++ ){
              sendData[3] = sendData[3]^digital[i];
              }  
                     
      //sendData[3] = digital[7];


         /*-------------- Start CheckSum - CRC16 ------------------------------- */     
      checkSum = mbCRC16(&sendData[0],sendDataLength);
      checkSumLow = checkSum & 0xFF; // low byte of chechsum
      checkSumHigh = checkSum>>8 & 0xFF; // high byte of checksum
         
      sendData[sendDataLength] = checkSumLow;
      sendData[sendDataLength+1] = checkSumHigh; // MSB

      /* ------------- End of CheckSumm - CRC16 -------------------------*/
      sendDataLength = sendDataLength + 2; // longer ( +2 )  because of checkSum values ( +2 )
      digitalWrite(ena1, HIGH);   // RECEIVE DISABLE
      digitalWrite(ena2, HIGH);   // transmit ENABLE mode
      delay(4);
      
      for(i=0;i<sendDataLength;i++){     
      UDR0 = sendData[i];
      while ( !( UCSR0A & (1<<UDRE0)) );

      }  
      //digitalWrite(output8, LOW);

      delay(5);
      
      digitalWrite(ena1, LOW);    // TRANSMIT DISABLE
      digitalWrite(ena2, LOW);   // ENABLE Receive
                                         
      for(i=0;i<sendDataLength;i++){      
      sendData[i] = 0;
      }
      i=0;                                         
      SerialData[0]=0;
      SerialData[1]=0;      
      //ES0 = 1; // enable uart interrupt 
      i=0;  // when interrupt is ready to get the data i should be setted to 0
      } //end of read digital input



      if(force_coil == 1){  // start of forcesingle coil   
       force_coil = 0;
       sendData[2] = SerialData[2]; 
       sendData[3] = SerialData[3]; 
       sendData[4] = SerialData[4]; 
      // if(sendData[4] == 0xFF){

         if(sendData[3] == 0){
          if(sendData[4] == 0xFF){
               digitalWrite(output1, HIGH);
              }
           else{
               digitalWrite(output1, LOW);
              }

             }
        if(sendData[3] == 1){
          if(sendData[4] == 0xFF){
               digitalWrite(output2, HIGH);
              }
           else{
               digitalWrite(output2, LOW);
              }

              }
        if(sendData[3] == 2){
          if(sendData[4] == 0xFF){
               digitalWrite(output3, HIGH);
              }
           else{
               digitalWrite(output3, LOW);
              }

        }

      if(sendData[3] == 3){
          if(sendData[4] == 0xFF){
               digitalWrite(output4, HIGH);
              }
           else{
               digitalWrite(output4, LOW);
              }

        }

       if(sendData[3] == 4){
          if(sendData[4] == 0xFF){
               digitalWrite(output5, HIGH);
              }
           else{
               digitalWrite(output5, LOW);
              }

        }

       if(sendData[3] == 5){
          if(sendData[4] == 0xFF){
               digitalWrite(output6, HIGH);
              }
           else{
               digitalWrite(output6, LOW);
              }

        }

       if(sendData[3] == 6){
          if(sendData[4] == 0xFF){
               digitalWrite(output7, HIGH);
              }
           else{
               digitalWrite(output7, LOW);
              }

        }

       if(sendData[3] == 7){
          if(sendData[4] == 0xFF){
               digitalWrite(output8, HIGH);
              }
           else{
               digitalWrite(output8, LOW);
              }

        }

        
       
       
      // }

      /* else{
        digitalWrite(output1, LOW);
        digitalWrite(output2, LOW);
        digitalWrite(output3, LOW);
        digitalWrite(output4, LOW);
        digitalWrite(output5, LOW);
        digitalWrite(output6, LOW);
        digitalWrite(output7, LOW);
        digitalWrite(output8, LOW);
       }*/
       sendData[5] = SerialData[5];
       sendDataLength = 6;


                  /*-------------- Start CheckSum - CRC16 ------------------------------- */     
      checkSum = mbCRC16(&sendData[0],sendDataLength);
      checkSumLow = checkSum & 0xFF; // low byte of chechsum
      checkSumHigh = checkSum>>8 & 0xFF; // high byte of checksum
         
      sendData[sendDataLength] = checkSumLow;
      sendData[sendDataLength+1] = checkSumHigh; // MSB

      /* ------------- End of CheckSumm - CRC16 -------------------------*/
      sendDataLength = sendDataLength + 2; // longer ( +2 )  because of checkSum values ( +2 )
      digitalWrite(ena1, HIGH);   // RECEIVE DISABLE
      digitalWrite(ena2, HIGH);   // transmit ENABLE mode
      delay(4); 
      for(i=0;i<sendDataLength;i++){     
      UDR0 = sendData[i];
      while ( !( UCSR0A & (1<<UDRE0)) );
      }  
      //digitalWrite(output8, LOW);
      delay(5); 
      digitalWrite(ena1, LOW);    // TRANSMIT DISABLE
      digitalWrite(ena2, LOW);   // ENABLE Receive                                  
      for(i=0;i<sendDataLength;i++){      
      sendData[i] = 0;
      }
      i=0;                                         
      SerialData[0]=0;
      SerialData[1]=0;      
  
      } //end of forcesinglecoil  


      /*----------------------------- End Of ReadCoil --------------------------------------------- */
  
   
    } // end of RxGo == 1 
 }

ISR(USART0_RX_vect)
{  
  //digitalWrite(output8, HIGH);
   SerialData[ba]  = UDR0;                       // Read data from the RX buffer
   while ( !( UCSR0A & (1<<UDRE0)) );
  if(SerialData[0] == SC){ // SC slave code   st+ng address 
   /*   if( SerialData[1] == 1){
            if(SerialData[2] == 0){
              if(SerialData[3] == 19){
                if(SerialData[4] == 0){
                 if(SerialData[5] == 37){
                    if(SerialData[6] == 14){
                      if(SerialData[7] == 132){
                       RxGo = 1;   
                      }       
                    }                
                  }             
                }            
              }       
            }
        }  */
  Read_Coil();
  Read_Input();
  force_singlecoil();
  //digitalWrite(output8, HIGH);
  //Holding_Register();
  ba = ba + 1;
  if(ba==8)
  {  ba=0;  }
  
    } 
    
  else  {
  ba=0;
  //SerialData[0]= 0; /* if slave code and number of function code are */
      //SerialData[1]= 0; /*  not equal, wait first bytes to start         */  
      }
   // UDR0 = ReceivedChar; 
  }

void Read_Coil(void){
  if(SerialData[1] == readCoil_FC){
      if(ba==7){
        QA = SerialData[4]*256+SerialData[5]; // Quantity of coils
        checkSum = mbCRC16(&SerialData[0],6);
        checkSumLow = checkSum & 0xFF; // low byte of checksum
        checkSumHigh = checkSum>>8 & 0xFF; // high byte of checksum
          if(SerialData[1] == readCoil_FC){  
              if(QA>=1 && QA<=2000){
                 if(((SerialData[2]*256+SerialData[3])+QA) <25 ){
                    if(SerialData[6] == checkSumLow && SerialData[7]== checkSumHigh){
                        checkSum = 0;
                        RxGo = 1; // ready for transmitting true info
                        readCoilGo = 1;
                       //ES0 = 0; // disable uart interrupt 
                           }
                       }
                     else{
                         //SerialData[0] = 0;
                         //exception02 = 1; // exception 2
                         ba=0;
                         }
                     }
          else {
             //Exception  3
             //SerialData[0] = 0;
            //exception03 = 1;
                ba=0;
             }
  
        }
       else   {
                  //SerialData[0] = 0;
           //exception01 = 1;
           ba=0;
          } 
           } // end of i==7
         }
       else  {
        //exception 1
       //i=0; // if any bit in the byte take address of the slave then stop i = 0
        //SerialData[0] = 1;
       //exception01 = 1;
        }
     }// end of Read_Coil()
     

void Read_Input(){
    if(SerialData[1] == readInputstatus_FC){
      if(ba==7){
        checkSum = mbCRC16(&SerialData[0],6);
        checkSumLow = checkSum & 0xFF; // low byte of checksum
        checkSumHigh = checkSum>>8 & 0xFF; // high byte of checksum
          if(SerialData[1] == readInputstatus_FC){  
                   if(SerialData[6] == checkSumLow && SerialData[7]== checkSumHigh){
                        checkSum = 0;
                        RxGo = 1; // ready for transmitting true info
                        read_input = 1;
                       //ES0 = 0; // disable uart interrupt 
                           }
                   
  
        }
       else   {
                  //SerialData[0] = 0;
           //exception01 = 1;
           ba=0;
          } 
           } // end of i==7
         }
       else  {
        //exception 1
       //i=0; // if any bit in the byte take address of the slave then stop i = 0
        //SerialData[0] = 1;
       //exception01 = 1;
        } 
 }
 
     
void force_singlecoil(){
    if(SerialData[1] == forcesinglecoil_FC){
      //digitalWrite(output8, HIGH);
      if(ba==7){
        checkSum = mbCRC16(&SerialData[0],6);
        checkSumLow = checkSum & 0xFF; // low byte of checksum
        checkSumHigh = checkSum>>8 & 0xFF; // high byte of checksum
          if(SerialData[1] == forcesinglecoil_FC){  
                   if(SerialData[6] == checkSumLow && SerialData[7]== checkSumHigh){
                        checkSum = 0;
                        RxGo = 1; // ready for transmitting true info
                        force_coil = 1;
                       //ES0 = 0; // disable uart interrupt 
                           }
                   
  
        }
       else   {
                  //SerialData[0] = 0;
           //exception01 = 1;
           ba=0;
          } 
           } // end of i==7
         }
       else  {
        //exception 1
       //i=0; // if any bit in the byte take address of the slave then stop i = 0
        //SerialData[0] = 1;
       //exception01 = 1;
        }

  
 }

/*   CRC16 calculation. Do not try to use uchCCRCHi and uchCRCLo, they should be swapped.
Instead use them, just use directly checksum variable  */
unsigned short int mbCRC16(unsigned char* puchMsg, unsigned char usDataLen) 
{
  unsigned char uchCRCHi = 0xFF ; /* high byte of CRC initialized */
  unsigned char uchCRCLo = 0xFF ; /* low byte of CRC initialized */
  unsigned char  uIndex ;

  uchCRCHi = 0xFF ;
  uchCRCLo = 0xFF ;
while (usDataLen--)
  {
  uIndex = uchCRCHi ^ *puchMsg++;
    uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
    uchCRCLo = auchCRCLo[uIndex];
  }
  return ( uchCRCLo<< 8 | uchCRCHi) ;
   }
