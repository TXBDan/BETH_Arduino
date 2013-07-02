#include <Arduino.h>
#include <Commander.h>
#include "ax12dan.h"
#include "init.h"
#include "ik.h"

Commander command = Commander();

commanderStruct commanderInput;
legStruct leg[6];
unsigned long currentTime;
unsigned long previousTime;
int tick;
int caseStep[6] = {1,3,1,3,1,3}; //for tripod gait
//int caseStep[6] = {1,3,5,7,9,11}; //for ripple gait


/**************************************************************************************************
  setup()
***************************************************************************************************/
void setup(){
      
    Serial.begin( 38400 );            // setup serial for FTDI or XBee
    ax12Init(1000000);                // setup serial for servos

    Serial.println("DAN'S HEXAPOD!!!");
    pinMode( ledPin, OUTPUT );        // pin 0 set to ouput for LED output
    
    //wait, then check the voltage (LiPO safety)
    delay (1000);
    float voltage = (ax12GetRegister (1, AX_PRESENT_VOLTAGE, 1)) / 10.0;
    Serial.print ("System Voltage: ");
    Serial.print (voltage);
    Serial.println (" volts.");
    if (voltage < 10.0){
        while(1){
            digitalWrite( ledPin, HIGH );     // turn on LED
            delay (150);
            digitalWrite( ledPin, LOW );      // turnon LED
            delay (150);
        }
    }

    /* INITIAL FOOT POSITIONS */
    leg[RIGHT_FRONT].initialFootPos.x = round( sin(radians(COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR) );
    leg[RIGHT_FRONT].initialFootPos.y = round( cos(radians(COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR) );
    leg[RIGHT_FRONT].initialFootPos.z = LENGTH_TIBIA + rideHeightOffset;
    leg[RIGHT_FRONT].legBasePos.x = X_COXA;
    leg[RIGHT_FRONT].legBasePos.y = Y_COXA_FB;
    leg[RIGHT_FRONT].legBasePos.z = 0;
    
    leg[RIGHT_MIDDLE].initialFootPos.x = 0;
    leg[RIGHT_MIDDLE].initialFootPos.y = (LENGTH_COXA+LENGTH_FEMUR);
    leg[RIGHT_MIDDLE].initialFootPos.z = LENGTH_TIBIA + rideHeightOffset;
    leg[RIGHT_MIDDLE].legBasePos.x = 0;
    leg[RIGHT_MIDDLE].legBasePos.y = Y_COXA_M;
    leg[RIGHT_MIDDLE].legBasePos.z = 0;
    
    leg[RIGHT_REAR].initialFootPos.x = round( sin(radians(-COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR) );
    leg[RIGHT_REAR].initialFootPos.y = round( cos(radians(COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR) );
    leg[RIGHT_REAR].initialFootPos.z = LENGTH_TIBIA + rideHeightOffset;
    leg[RIGHT_REAR].legBasePos.x = -X_COXA;
    leg[RIGHT_REAR].legBasePos.y = Y_COXA_FB;
    leg[RIGHT_REAR].legBasePos.z = 0;
    
    leg[LEFT_REAR].initialFootPos.x = round( sin(radians(-COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR) );
    leg[LEFT_REAR].initialFootPos.y = -round( cos(radians(COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR) );
    leg[LEFT_REAR].initialFootPos.z = LENGTH_TIBIA + rideHeightOffset;
    leg[LEFT_REAR].legBasePos.x = -X_COXA;
    leg[LEFT_REAR].legBasePos.y = -Y_COXA_FB;
    leg[LEFT_REAR].legBasePos.z = 0;
    
    leg[LEFT_MIDDLE].initialFootPos.x = 0;
    leg[LEFT_MIDDLE].initialFootPos.y = -(LENGTH_COXA+LENGTH_FEMUR);
    leg[LEFT_MIDDLE].initialFootPos.z = LENGTH_TIBIA + rideHeightOffset;
    leg[LEFT_MIDDLE].legBasePos.x = 0;
    leg[LEFT_MIDDLE].legBasePos.y = -Y_COXA_M;
    leg[LEFT_MIDDLE].legBasePos.z = 0;
    
    leg[LEFT_FRONT].initialFootPos.x = round( sin(radians(COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR) );
    leg[LEFT_FRONT].initialFootPos.y = -round( cos(radians(COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR) );
    leg[LEFT_FRONT].initialFootPos.z = LENGTH_TIBIA + rideHeightOffset;
    leg[LEFT_FRONT].legBasePos.x = X_COXA;
    leg[LEFT_FRONT].legBasePos.y = -Y_COXA_FB;
    leg[LEFT_FRONT].legBasePos.z = 0;
 
}

/********************************************************************************************************
  loop()
  Main loop
*********************************************************************************************************/
void loop(){
    
    currentTime = millis();
    if(currentTime - previousTime >= SERVO_UPDATE_PERIOD) {      // wait until its been 20ms for servo update  
        previousTime = currentTime;         
         
        //Serial.println("Crunching... "); 
        readCommandInputs();                        // Read in input from controller
        //Serial.print("time after readCommandInputs(): "); Serial.println(millis()-currentTime);
        runIK();
        //Serial.print("time after runIK(): "); Serial.println(millis()-currentTime);
        tripodGait();  
        //rippleGait();
        //Serial.print("time after tripodWalk(): "); Serial.println(millis()-currentTime);

    }
    //Serial.print("Total frame time: ");
    //Serial.println(millis()-currentTime);
   

} //end main loop()




/*************************************************
  tripodGait()
  
**************************************************/
void tripodGait(){
  
    float sinRotZ, cosRotZ;
    int totalX, totalY;
    int strideRotOffsetX[6], strideRotOffsetY[6];
    int height = -35;
    int duration;
    int numTicks;
    int speedX, speedY, speedR;
    int strideX[6], strideY[6];
          
    if( (abs(commanderInput.Xspeed) > 5) || (abs(commanderInput.Yspeed) > 5) || (abs(commanderInput.Rspeed) > 5 ) ){
                             
        duration = 800;                               //duration of one step cycle (ms)      
        numTicks = duration / SERVO_UPDATE_PERIOD / 4; //total ticks divided into the four cases   
              
        speedX = 180*commanderInput.Xspeed/127;        //200mm/s top speed
        speedY = 180*commanderInput.Yspeed/127;        //200mm/s top speed
        speedR = 40*commanderInput.Rspeed/127;         //40deg/s top rotation speed
                    
        sinRotZ = sin(radians(speedR));
        cosRotZ = cos(radians(speedR));
                 
        for( int legNum=0; legNum<6; legNum++){
          
            totalX = leg[legNum].initialFootPos.x + leg[legNum].legBasePos.x; 
            totalY = leg[legNum].initialFootPos.y + leg[legNum].legBasePos.y;
            
            strideRotOffsetX[legNum] = round( totalY*sinRotZ + totalX*cosRotZ - totalX);  
            strideRotOffsetY[legNum] = round( totalY*cosRotZ - totalX*sinRotZ - totalY); 
            
            if(abs(speedR*5) > abs(speedX) && abs(speedR*5) > abs(speedY))  height = -abs(speedR);  
            else{
              if(abs(speedX) >= abs(speedY)) height = -abs(speedX/5);
              else height = -abs(speedY/5);  
            } 
            
             
            switch (caseStep[legNum]){
            
                case 1: //forward raise
                                      
                    leg[legNum].footPos.x = ((long)(speedX + strideRotOffsetX[legNum])*tick*SERVO_UPDATE_PERIOD)/1000 - (speedX + strideRotOffsetX[legNum])/4; 
                    leg[legNum].footPos.y = ((long)(speedY + strideRotOffsetY[legNum])*tick*SERVO_UPDATE_PERIOD)/1000 - (speedY + strideRotOffsetY[legNum])/4;
                    leg[legNum].footPos.z = (height*tick)/numTicks;
                        
                    if( tick >= numTicks-1 ) caseStep[legNum] = 2;
                    break;
                    
                case 2: // forward lower
                
                    leg[legNum].footPos.x = ((long)(speedX + strideRotOffsetX[legNum])*tick*SERVO_UPDATE_PERIOD)/1000;
                    leg[legNum].footPos.y = ((long)(speedY + strideRotOffsetY[legNum])*tick*SERVO_UPDATE_PERIOD)/1000;
                    leg[legNum].footPos.z = height - (height*tick)/numTicks ;
                     
                    if( tick >= numTicks-1 ) caseStep[legNum] = 3;
                    break;
                  
                case 3: // down pull back
                
                    leg[legNum].footPos.x = -((long)(speedX + strideRotOffsetX[legNum])*tick*SERVO_UPDATE_PERIOD)/1000 + (speedX + strideRotOffsetX[legNum])/4;
                    leg[legNum].footPos.y = -((long)(speedY + strideRotOffsetY[legNum])*tick*SERVO_UPDATE_PERIOD)/1000 + (speedY + strideRotOffsetY[legNum])/4;
                    leg[legNum].footPos.z = 0;
                        
                    if( tick >= numTicks-1 ) caseStep[legNum] = 4;
                    break;
                    
                case 4: // down pull back
                    
                    leg[legNum].footPos.x = -((long)(speedX + strideRotOffsetX[legNum])*tick*SERVO_UPDATE_PERIOD)/1000;
                    leg[legNum].footPos.y = -((long)(speedY + strideRotOffsetY[legNum])*tick*SERVO_UPDATE_PERIOD)/1000;
                    leg[legNum].footPos.z = 0;
    
                    if( tick >= numTicks-1 ) caseStep[legNum] = 1;
                    break;
       
          }// end of case statement
                             
        }// end of loop over legs
        if (tick < numTicks-1) tick++;
        else tick = 0;
      
    }//end if joystick active

}

/*************************************************
  rippleGait()
  
**************************************************/
void rippleGait(){
  
    float sinRotZ, cosRotZ;
    int totalX, totalY;
    int strideRotOffsetX[6], strideRotOffsetY[6];
    int height[6];
    int duration;
    int numTicks;
    int speedX, speedY, speedR;
    int strideX[6], strideY[6];
          
    if( (abs(commanderInput.Xspeed) > 5) || (abs(commanderInput.Yspeed) > 5) || (abs(commanderInput.Rspeed) > 5 ) ){
        Serial.print("***TICK***: ");Serial.println(tick);
                             
        duration = 1000; //3000ms  
        Serial.print("duration: "); Serial.println(duration);    
         
        numTicks = duration / SERVO_UPDATE_PERIOD / 4;        //total ticks divided into the 12 cases   
        //Serial.print("numTicks: "); Serial.println(numTicks);
              
        speedX = 100*commanderInput.Xspeed/127;        //100mm/s top speed
        //Serial.print("speedX: "); Serial.println(speedX);
        speedY = 100*commanderInput.Yspeed/127;        //100mm/s top speed
        //Serial.print("speedY: "); Serial.println(speedY);
        speedR = 15*commanderInput.Rspeed/127;         //15deg/s top rotation speed
        //Serial.print("speedR: "); Serial.println(speedR);
                    
        sinRotZ = sin(radians(speedR));
        cosRotZ = cos(radians(speedR));
            
            
        for( int legNum=0; legNum<6; legNum++){
            //Serial.print("Leg: "); Serial.println(legNum+1);
          
            totalX = leg[legNum].initialFootPos.x + leg[legNum].legBasePos.x; 
            totalY = leg[legNum].initialFootPos.y + leg[legNum].legBasePos.y;
            
            strideRotOffsetX[legNum] = round( totalY*sinRotZ + totalX*cosRotZ - totalX);  
            //Serial.print("strideRotOffsetX: "); Serial.println(strideRotOffsetX[legNum]);
            strideRotOffsetY[legNum] = round( totalY*cosRotZ - totalX*sinRotZ - totalY);
            //Serial.print("strideRotOffsetY: "); Serial.println(strideRotOffsetY[legNum]);
                            
            strideX[legNum] = speedX*duration/1000 + strideRotOffsetX[legNum];         // speedX*duration/1000 : mm/s * s
            //Serial.print("strideX: "); Serial.println(strideX[legNum]);
            strideY[legNum] = speedY*duration/1000 + strideRotOffsetY[legNum];  
            //Serial.print("strideY: "); Serial.println(strideY[legNum]);
            
            if(abs(strideX[legNum]) >= abs(strideY[legNum])) height[legNum] = -abs(strideX[legNum]/2);
            else height[legNum] = -abs(strideY[legNum]/2);
            //Serial.print("height: "); Serial.println(height[legNum]);   
             
            switch (caseStep[legNum]){
            
                case 1: //forward raise
                                      
                    //Serial.print("1ST QUARTER  Tick: "); Serial.println(tick);
                    leg[legNum].footPos.x = (strideX[legNum]*tick)/(2*numTicks) - strideX[legNum]/2 ;
                    leg[legNum].footPos.y = (strideY[legNum]*tick)/(2*numTicks) - strideY[legNum]/2 ;
                    leg[legNum].footPos.z = (height[legNum]*tick)/numTicks;
                        
                    if( tick >= numTicks-1 ) caseStep[legNum] = 2;
                    break;
                    
                case 2: // forward lower
                
                    //Serial.print("2ND QUARTER  Tick: "); Serial.println(tick);
                    leg[legNum].footPos.x = (strideX[legNum]*tick)/(2*numTicks) ;
                    leg[legNum].footPos.y = (strideY[legNum]*tick)/(2*numTicks) ;
                    leg[legNum].footPos.z = height[legNum] - (height[legNum]*tick)/numTicks ;
                     
                    if( tick >= numTicks-1 ) caseStep[legNum] = 3;
                    break;
                  
                case 3: // down pull back
                
                    //Serial.print("3RD QUARTER  Tick: "); Serial.println(tick);
                    leg[legNum].footPos.x = -(strideX[legNum]*tick)/(10*numTicks) + strideX[legNum]/2 ;
                    leg[legNum].footPos.y = -(strideY[legNum]*tick)/(10*numTicks) + strideY[legNum]/2 ;
                    leg[legNum].footPos.z = 0;
                        
                    if( tick >= numTicks-1 ) caseStep[legNum] = 4;
                    break;
                    
                case 4: // down pull back
                    
                    //Serial.print("4TH QUARTER  Tick: "); Serial.println(tick);
                    leg[legNum].footPos.x = -(strideX[legNum]*tick)/(10*numTicks) + strideX[legNum]*2/5;
                    leg[legNum].footPos.y = -(strideY[legNum]*tick)/(10*numTicks) + strideY[legNum]*2/5;
                    leg[legNum].footPos.z = 0;
    
                    if( tick >= numTicks-1 ) caseStep[legNum] = 5;
                    break;
                    
                case 5: // down pull back
                
                    //Serial.print("3RD QUARTER  Tick: "); Serial.println(tick);
                    leg[legNum].footPos.x = -(strideX[legNum]*tick)/(10*numTicks) + strideX[legNum]*3/10 ;
                    leg[legNum].footPos.y = -(strideY[legNum]*tick)/(10*numTicks) + strideY[legNum]*3/10 ;
                    leg[legNum].footPos.z = 0;
                        
                    if( tick >= numTicks-1 ) caseStep[legNum] = 6;
                    break;
                    
                case 6: // down pull back
                    
                    //Serial.print("4TH QUARTER  Tick: "); Serial.println(tick);
                    leg[legNum].footPos.x = -(strideX[legNum]*tick)/(10*numTicks) + strideX[legNum]/5;
                    leg[legNum].footPos.y = -(strideY[legNum]*tick)/(10*numTicks) + strideY[legNum]/5;
                    leg[legNum].footPos.z = 0;
    
                    if( tick >= numTicks-1 ) caseStep[legNum] = 7;
                    break;      
                    
                case 7: // down pull back
                
                    //Serial.print("3RD QUARTER  Tick: "); Serial.println(tick);
                    leg[legNum].footPos.x = -(strideX[legNum]*tick)/(10*numTicks) + strideX[legNum]/10 ;
                    leg[legNum].footPos.y = -(strideY[legNum]*tick)/(10*numTicks) + strideY[legNum]/10 ;
                    leg[legNum].footPos.z = 0;
                        
                    if( tick >= numTicks-1 ) caseStep[legNum] = 8;
                    break;
                    
                case 8: // down pull back
                    
                    //Serial.print("4TH QUARTER  Tick: "); Serial.println(tick);
                    leg[legNum].footPos.x = -(strideX[legNum]*tick)/(10*numTicks) ;
                    leg[legNum].footPos.y = -(strideY[legNum]*tick)/(10*numTicks) ;
                    leg[legNum].footPos.z = 0;
    
                    if( tick >= numTicks-1 ) caseStep[legNum] = 9;
                    break;
  
                case 9: // down pull back
                
                    //Serial.print("3RD QUARTER  Tick: "); Serial.println(tick);
                    leg[legNum].footPos.x = -(strideX[legNum]*tick)/(10*numTicks) - strideX[legNum]/10 ;
                    leg[legNum].footPos.y = -(strideY[legNum]*tick)/(10*numTicks) - strideY[legNum]/10 ;
                    leg[legNum].footPos.z = 0;
                        
                    if( tick >= numTicks-1 ) caseStep[legNum] = 10;
                    break;
                    
                case 10: // down pull back
                    
                    //Serial.print("4TH QUARTER  Tick: "); Serial.println(tick);
                    leg[legNum].footPos.x = -(strideX[legNum]*tick)/(10*numTicks) - strideX[legNum]/5;
                    leg[legNum].footPos.y = -(strideY[legNum]*tick)/(10*numTicks) - strideY[legNum]/5;
                    leg[legNum].footPos.z = 0;
    
                    if( tick >= numTicks-1 ) caseStep[legNum] = 11;
                    break;      
                    
                case 11: // down pull back
                
                    //Serial.print("3RD QUARTER  Tick: "); Serial.println(tick);
                    leg[legNum].footPos.x = -(strideX[legNum]*tick)/(10*numTicks) - strideX[legNum]*3/10 ;
                    leg[legNum].footPos.y = -(strideY[legNum]*tick)/(10*numTicks) - strideY[legNum]*3/10 ;
                    leg[legNum].footPos.z = 0;
                        
                    if( tick >= numTicks-1 ) caseStep[legNum] = 12;
                    break;
                    
                case 12: // down pull back
                    
                    //Serial.print("4TH QUARTER  Tick: "); Serial.println(tick);
                    leg[legNum].footPos.x = -(strideX[legNum]*tick)/(10*numTicks) - strideX[legNum]*2/5;
                    leg[legNum].footPos.y = -(strideY[legNum]*tick)/(10*numTicks) - strideY[legNum]*2/5;
                    leg[legNum].footPos.z = 0;
    
                    if( tick >= numTicks-1 ) caseStep[legNum] = 1;
                    break;      
                    
          } // end of case statement
          
          //Serial.print("footPos.x: "); Serial.println(leg[legNum].footPos.x);
          //Serial.print("footPos.y: "); Serial.println(leg[legNum].footPos.y);
          //Serial.print("footPos.z: "); Serial.println(leg[legNum].footPos.z);

                    
        }//loop over legs
        if (tick < numTicks-1) tick++;
        else tick = 0;
      
    }//end if joystick active

}


/*************************************************
  readCommandInputs()
  Reads input from Commander controller and saves them as variables
**************************************************/
void readCommandInputs(){
  
  //commanderInput.Xspeed = 80;
  //commanderInput.Yspeed = 100;
  //commanderInput.Rspeed = 80;
  
//  caseStep[0] = 1; //for tripod gait DOES NOT WORK
//  caseStep[1] = 3;
//  caseStep[2] = 1;
//  caseStep[3] = 3;
//  caseStep[4] = 1;
//  caseStep[5] = 3;
  
  if(command.ReadMsgs() > 0){
      digitalWrite( ledPin, HIGH-digitalRead(ledPin) );
      
      //read in Command controller inputs
      if( abs(command.walkV) > 5 ){
          commanderInput.Xspeed = command.walkV;
      }
      else commanderInput.Xspeed = 0;
     
      if( abs(command.walkH) > 5 ){   
          commanderInput.Yspeed = command.walkH;
      }
      else commanderInput.Yspeed = 0;
      
      if( abs(command.lookH) > 5 ){
          commanderInput.Rspeed = -command.lookH;
      }
      else commanderInput.Rspeed = 0;
          
      if( command.buttons&BUT_RT ){
          commanderInput.bodyRotX = -(float)command.walkH / 10.0;
          commanderInput.bodyRotY = (float)command.walkV / 10.0;
          commanderInput.bodyRotZ = -(float)command.lookH / 7.0;      
          commanderInput.Rspeed = 0;
          commanderInput.Xspeed = 0;
          commanderInput.Yspeed = 0;  
      }
      else{
          commanderInput.bodyRotX = 0.0;
          commanderInput.bodyRotY = 0.0;
          commanderInput.bodyRotZ = 0.0;  
      }
      
      if( command.buttons&BUT_LT ){
          commanderInput.bodyTransX = command.walkV / 3;
          commanderInput.bodyTransY = command.walkH / 3;
          commanderInput.bodyTransZ = command.lookV / 3;  
          commanderInput.Rspeed = 0;
          commanderInput.Xspeed = 0;
          commanderInput.Yspeed = 0;  
      }
      else{
          commanderInput.bodyTransX = 0; 
          commanderInput.bodyTransY = 0;
          commanderInput.bodyTransZ = 0;  
      }
  }
}


