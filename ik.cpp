#include <Arduino.h>
#include "init.h"
#include "ik.h"
#include "ax12dan.h"

/******************************************************************************
 * Inverse Kinematics for hexapod
 *
 * FRONT VIEW       ^        ==0         0==
 *     /\___/\      |       |  0==[___]==0  |
 *    /       \     -Z      |               |
 *
 * TOP VIEW
 *    \       /     ^
 *     \_____/      |
 *  ___|     |___   X
 *     |_____|
 *     /     \      Y->
 *    /       \
 *****************************************************************************/


/*********************************************************************************************************
    runIK()
**********************************************************************************************************/
void runIK(){
  
    footPosCalc();
    legIK(); 
    driveServos();
}

/**********************************************************************************************************
    footPosCalc()
    Calculates necessary foot position (leg space) to acheive commanded body rotations, translations, and gait inputs
***********************************************************************************************************/
void footPosCalc(){
    
    float sinRotX, cosRotX, sinRotY, cosRotY, sinRotZ, cosRotZ;
    int totalX, totalY, totalZ;
    int tempFootPosX[6], tempFootPosY[6], tempFootPosZ[6];
    int bodyRotOffsetX[6], bodyRotOffsetY[6], bodyRotOffsetZ[6];
  
    sinRotX = sin(radians(-commanderInput.bodyRotX));
    cosRotX = cos(radians(-commanderInput.bodyRotX));
    sinRotY = sin(radians(-commanderInput.bodyRotY));
    cosRotY = cos(radians(-commanderInput.bodyRotY));
    sinRotZ = sin(radians(-commanderInput.bodyRotZ));
    cosRotZ = cos(radians(-commanderInput.bodyRotZ));
  
    for( int legNum=0; legNum<6; legNum++){ 
        //Serial.print ("footPosCalc() Leg: "); Serial.println (legNum+1);

        //sinRotZ = sin(radians(leg[legNum].bodyRotZ - commanderInput.bodyRotZ));
        //cosRotZ = cos(radians(leg[legNum].bodyRotZ - commanderInput.bodyRotZ));
    
        totalX = leg[legNum].initialFootPos.x + leg[legNum].legBasePos.x; 
        totalY = leg[legNum].initialFootPos.y + leg[legNum].legBasePos.y;
        totalZ = leg[legNum].initialFootPos.z + leg[legNum].legBasePos.z;

        bodyRotOffsetX[legNum] = round(( totalY*cosRotY*sinRotZ + totalY*cosRotZ*sinRotX*sinRotY + totalX*cosRotZ*cosRotY - totalX*sinRotZ*sinRotX*sinRotY - totalZ*cosRotX*sinRotY) - totalX);  
        bodyRotOffsetY[legNum] = round(  totalY*cosRotX*cosRotZ - totalX*cosRotX*sinRotZ         + totalZ*sinRotX         - totalY);
        bodyRotOffsetZ[legNum] = round(( totalY*sinRotZ*sinRotY - totalY*cosRotZ*cosRotY*sinRotX + totalX*cosRotZ*sinRotY + totalX*cosRotY*sinRotZ*sinRotX + totalZ*cosRotX*cosRotY) - totalZ);   
      
        // Calculated foot positions to acheive xlation/rotation input. Not coxa mounting angle corrected
        tempFootPosX[legNum] = leg[legNum].initialFootPos.x + bodyRotOffsetX[legNum] - commanderInput.bodyTransX + leg[legNum].footPos.x;                                              
        tempFootPosY[legNum] = leg[legNum].initialFootPos.y + bodyRotOffsetY[legNum] - commanderInput.bodyTransY + leg[legNum].footPos.y;
        tempFootPosZ[legNum] = leg[legNum].initialFootPos.z + bodyRotOffsetZ[legNum] - commanderInput.bodyTransZ + leg[legNum].footPos.z;
    }
     
    // Rotates X,Y about coxa to compensate for coxa mounting angles.
    leg[0].footPosCalc.x = round( tempFootPosY[0]*cos(radians(COXA_ANGLE))   - tempFootPosX[0]*sin(radians(COXA_ANGLE)) ); 
    leg[0].footPosCalc.y = round( tempFootPosY[0]*sin(radians(COXA_ANGLE))   + tempFootPosX[0]*cos(radians(COXA_ANGLE)) );
    leg[0].footPosCalc.z =        tempFootPosZ[0];
    leg[1].footPosCalc.x = round( tempFootPosY[1]*cos(radians(COXA_ANGLE*2)) - tempFootPosX[1]*sin(radians(COXA_ANGLE*2)) );
    leg[1].footPosCalc.y = round( tempFootPosY[1]*sin(radians(COXA_ANGLE*2)) + tempFootPosX[1]*cos(radians(COXA_ANGLE*2)) );
    leg[1].footPosCalc.z =        tempFootPosZ[1];
    leg[2].footPosCalc.x = round( tempFootPosY[2]*cos(radians(COXA_ANGLE*3)) - tempFootPosX[2]*sin(radians(COXA_ANGLE*3)) );
    leg[2].footPosCalc.y = round( tempFootPosY[2]*sin(radians(COXA_ANGLE*3)) + tempFootPosX[2]*cos(radians(COXA_ANGLE*3)) );
    leg[2].footPosCalc.z =        tempFootPosZ[2];
    leg[3].footPosCalc.x = round( tempFootPosY[3]*cos(radians(COXA_ANGLE*5)) - tempFootPosX[3]*sin(radians(COXA_ANGLE*5)) );
    leg[3].footPosCalc.y = round( tempFootPosY[3]*sin(radians(COXA_ANGLE*5)) + tempFootPosX[3]*cos(radians(COXA_ANGLE*5)) );
    leg[3].footPosCalc.z =        tempFootPosZ[3];
    leg[4].footPosCalc.x = round( tempFootPosY[4]*cos(radians(COXA_ANGLE*6)) - tempFootPosX[4]*sin(radians(COXA_ANGLE*6)) );
    leg[4].footPosCalc.y = round( tempFootPosY[4]*sin(radians(COXA_ANGLE*6)) + tempFootPosX[4]*cos(radians(COXA_ANGLE*6)) );
    leg[4].footPosCalc.z =        tempFootPosZ[4];
    leg[5].footPosCalc.x = round( tempFootPosY[5]*cos(radians(COXA_ANGLE*7)) - tempFootPosX[5]*sin(radians(COXA_ANGLE*7)) );
    leg[5].footPosCalc.y = round( tempFootPosY[5]*sin(radians(COXA_ANGLE*7)) + tempFootPosX[5]*cos(radians(COXA_ANGLE*7)) );
    leg[5].footPosCalc.z =        tempFootPosZ[5];
    
    //for( int legNum=0; legNum<6; legNum++){ 
        //Serial.print ("footPosCalc() Leg: "); Serial.println (legNum+1);
        //Serial.print("footPosCalcX: "); Serial.println(leg[legNum].footPosCalc.x);  //these are off by +/- 1
        //Serial.print("footPosCalcY: "); Serial.println(leg[legNum].footPosCalc.y);
        //Serial.print("footPosCalcZ: "); Serial.println(leg[legNum].footPosCalc.z);
    //}
    
}


/**************************************************************************************************************
    legIK()
    Translates foot x,y,z positions (body space) to leg space and adds goal foot positon input (leg space).
    Calculates the coxa, femur, and tibia angles for these foot positions (leg space).
***************************************************************************************************************/
void legIK(){
  
    float CoxaFootDist, IKSW, IKA1, IKA2, tibAngle;
                                                         
    for( int legNum=0; legNum<6; legNum++ ){
      
        //Serial.print ("legIK() Leg: "); Serial.println (legNum+1);
   
        CoxaFootDist = sqrt( sq(leg[legNum].footPosCalc.y) + sq(leg[legNum].footPosCalc.x) );
        IKSW = sqrt( sq(CoxaFootDist-LENGTH_COXA) + sq(leg[legNum].footPosCalc.z) );
        IKA1 = atan2( (CoxaFootDist - LENGTH_COXA) , leg[legNum].footPosCalc.z );
        IKA2 = acos( (sq(LENGTH_TIBIA) - sq(LENGTH_FEMUR) - sq(IKSW) ) / (-2*IKSW*LENGTH_FEMUR) );
        tibAngle = acos( (sq(IKSW) - sq(LENGTH_TIBIA) - sq(LENGTH_FEMUR)) / (-2*LENGTH_FEMUR*LENGTH_TIBIA) );
        
        leg[legNum].jointAngles.coxa  = 90 - degrees( atan2( leg[legNum].footPosCalc.y , leg[legNum].footPosCalc.x) ); 
        leg[legNum].jointAngles.femur = 90 - degrees( IKA1 + IKA2 );
        leg[legNum].jointAngles.tibia = 90 - degrees( tibAngle );
                      
        //Serial.print("Coxa Angle: "); Serial.println(leg[legNum].jointAngles.coxa);
        //Serial.print("Femur Angle: "); Serial.println(leg[legNum].jointAngles.femur); 
        //Serial.print("Tibia Angle: "); Serial.println(leg[legNum].jointAngles.tibia);
    }
    
    // Applies necessary corrections to servo angles to account for hardware
    for( int legNum=0; legNum<3; legNum++ ){
        leg[legNum].jointAngles.coxa  = leg[legNum].jointAngles.coxa;
        leg[legNum].jointAngles.femur = leg[legNum].jointAngles.femur - 13.58;              // accounts for offset servo bracket on femur
        leg[legNum].jointAngles.tibia = leg[legNum].jointAngles.tibia - 48.70 + 13.58 + 90; //counters offset servo bracket on femur, accounts for 90deg mounting, and bend of tibia
    }    
   for( int legNum=3; legNum<6; legNum++ ){
     
        leg[legNum].jointAngles.coxa  =   leg[legNum].jointAngles.coxa;
        leg[legNum].jointAngles.femur = -(leg[legNum].jointAngles.femur - 13.58);
        leg[legNum].jointAngles.tibia = -(leg[legNum].jointAngles.tibia - 48.70 + 13.58 + 90);
    }
    
}



/*************************************************
    driveServos()
    Commands servos to angles in joinitAngles
    converts to AX12 definition of angular rotation and divides by number of degrees per bit.
    0deg = 512 = straight servo
    ax12SyncWrite() writes all servo values out to the AX12 bus using the SYNCWRITE instruction
**************************************************/
void driveServos(){
  
    for( int legNum=0; legNum<6; legNum++ ){
        leg[legNum].servoPos.coxa  = round((abs( leg[legNum].jointAngles.coxa  - 210) - 60 )  / 0.293);
        leg[legNum].servoPos.femur = round((abs( leg[legNum].jointAngles.femur - 210) - 60 )  / 0.293);
        leg[legNum].servoPos.tibia = round((abs( leg[legNum].jointAngles.tibia - 210) - 60 )  / 0.293);
    }
      
    ax12SyncWriteServos();  
}
