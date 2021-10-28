#include "dspic.h"

DsPIC::DsPIC(){
    fd = serialOpen ("/dev/serial0", BAUDRATE);
}
DsPIC::~DsPIC(){

}
void DsPIC::initVarDspic(){    //Init PID,odometry,acceleration,speed
    setVarDouble64b(CODE_VAR_P_SPEED_0_LD,0.1);
    setVarDouble64b(CODE_VAR_P_SPEED_1_LD,0.1);
    setVarDouble64b(CODE_VAR_P_SPEED_2_LD,0.1);
    
    setVarDouble64b(CODE_VAR_I_SPEED_0_LD,0.2);
    setVarDouble64b(CODE_VAR_I_SPEED_1_LD,0.2);
    setVarDouble64b(CODE_VAR_I_SPEED_2_LD,0.2);

	setVarDouble64b(CODE_VAR_D_SPEED_0_LD,0.0);
	setVarDouble64b(CODE_VAR_D_SPEED_1_LD,0.0);
	setVarDouble64b(CODE_VAR_D_SPEED_2_LD,0.0);
    
    //setVarDouble64b(CODE_VAR_P_DISTANCE_LD,0.02);
    //setVarDouble64b(CODE_VAR_P_DISTANCE_LD,0);
    
	//setVarDouble64b(CODE_VAR_P_DISTANCE_LD,0.1);
    
    //setVarDouble64b(CODE_VAR_P_ANGLE_LD,0.05);
	
	setVarDouble64b(CODE_VAR_P_DISTANCE_LD,6);
    
    setVarDouble64b(CODE_VAR_P_ANGLE_LD,3);
    
    setVarDouble64b(CODE_VAR_TRAJ_LIN_SPEED_LD,200);
    setVarDouble64b(CODE_VAR_TRAJ_LIN_ACC_LD,200);
    
    double wheelDiameter0 = 63.8;
    double wheelDiameter1 = 63.8;
    double wheelDiameter2 = wheelDiameter1 * 0.95;
    
    setVarDouble64b(CODE_VAR_WHEEL_DIAMETER0_LD,wheelDiameter0);
    setVarDouble64b(CODE_VAR_WHEEL_DIAMETER1_LD,wheelDiameter1);
    setVarDouble64b(CODE_VAR_WHEEL_DIAMETER2_LD,wheelDiameter2);
}
void DsPIC::servo(uint8_t id, uint16_t value){
    uint8_t buffer[RX_SIZE_SERVO + 1];
    buffer[0] = RX_SIZE_SERVO;
    buffer[1] = RX_CODE_SERVO;
    buffer[2] = id;
    buffer[3] = (uint8_t)(value >> 8);
    buffer[4] = (uint8_t)(value & 0xFF);
    buffer[5] = 0;
    for(int i = 0; i < RX_SIZE_SERVO; i++){
        buffer[5] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_SERVO + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}
void DsPIC::AX12(uint8_t id, uint16_t value){
    uint8_t buffer[RX_SIZE_AX12 + 1];
    buffer[0] = RX_SIZE_AX12;
    buffer[1] = RX_CODE_AX12;
    buffer[2] = id;
    buffer[3] = (uint8_t)(value >> 8);
    buffer[4] = (uint8_t)(value & 0xFF);
    buffer[5] = 0;
    for(int i = 0; i < RX_SIZE_AX12; i++){
        buffer[5] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_AX12 + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}
void DsPIC::motor(uint8_t id, int8_t value){
    uint8_t buffer[RX_SIZE_MOTOR + 1];
    buffer[0] = RX_SIZE_MOTOR;
    buffer[1] = RX_CODE_MOTOR;
    buffer[2] = id;
    buffer[3] = value;
    buffer[4] = 0;
    for(int i = 0; i < RX_SIZE_MOTOR; i++){
        buffer[4] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_MOTOR + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}
void DsPIC::start(){
    uint8_t buffer[RX_SIZE_START + 1];
    buffer[0] = RX_SIZE_START;
    buffer[1] = RX_CODE_START;
    buffer[2] = 0;
    for(int i = 0; i < RX_SIZE_START; i++){
        buffer[2] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_START + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}
void DsPIC::stop(){
    uint8_t buffer[RX_SIZE_STOP + 1];
    buffer[0] = RX_SIZE_STOP;
    buffer[1] = RX_CODE_STOP;
    buffer[2] = 0;
    for(int i = 0; i < RX_SIZE_STOP; i++){
        buffer[2] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_STOP + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}

/*
Tells the robot to go to point(x,y) 
    - x : x mm
    - y : y mm
    - rev : reverse, allows to the point in reverse (useless for holonome)
    - relative : activates relative mode, the reference is the actual position of the robot
*/
void DsPIC::go(int16_t x, int16_t y,unsigned char rev, unsigned char relative){
    uint8_t option = 0;
    if(rev){
        option += MASK_OPTION_REVERSE;
    }
    if(relative){
        option += MASK_OPTION_RELATIVE;
    }
    uint8_t buffer[RX_SIZE_GO + 1];
    buffer[0] = RX_SIZE_GO;
    buffer[1] = RX_CODE_GO;
    buffer[2] = option;
    buffer[3] = (uint8_t)(x >> 8);
    buffer[4] = (uint8_t)(x & 0xFF);
    buffer[5] = (uint8_t)(y >> 8);
    buffer[6] = (uint8_t)(y & 0xFF);
    buffer[7] = 0;
    for(int i = 0; i < RX_SIZE_GO; i++){
        buffer[7] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_GO + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}

/*
Tells the robot to go to point(x,y) 
    - t : angle in degrees   
    - rev : reverse, allows to the point in reverse (useless for holonome)
    - relative : activates relative mode, the reference is the actual position of the robot
*/
void DsPIC::turn(int16_t t,unsigned char rev, unsigned char relative){
    uint8_t option = 0;
    if(rev){
        option += MASK_OPTION_REVERSE;
    }
    if(relative){
        option += MASK_OPTION_RELATIVE;
    }
    uint8_t buffer[RX_SIZE_TURN + 1];
    buffer[0] = RX_SIZE_TURN;
    buffer[1] = RX_CODE_TURN;
    buffer[2] = option;
    buffer[3] = (uint8_t)(t >> 8);
    buffer[4] = (uint8_t)(t & 0xFF);
    buffer[5] = 0;
    for(int i = 0; i < RX_SIZE_TURN; i++){
        buffer[5] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_TURN + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}
void DsPIC::initPos(double x, double y, double t){
    //init position & angle
    setVarDouble64b(CODE_VAR_X_LD,x);
    setVarDouble64b(CODE_VAR_Y_LD,y);
    setVarDouble64b(CODE_VAR_T_LD,t);
    //set setPoint to new position
    setVarDouble64b(CODE_VAR_XC_LD,x);
    setVarDouble64b(CODE_VAR_YC_LD,y);
    setVarDouble64b(CODE_VAR_TC_LD,t);
    //set aimed point to new position
    setVarDouble64b(CODE_VAR_XF_LD,x);
    setVarDouble64b(CODE_VAR_YF_LD,y);
    setVarDouble64b(CODE_VAR_TF_LD,t);
}
void DsPIC::setSpPosition(double x, double y, double t){
	setVarDouble64b(CODE_VAR_XC_LD,x);
    setVarDouble64b(CODE_VAR_YC_LD,y);
    setVarDouble64b(CODE_VAR_TC_LD,t);
	
	
	setVarDouble64b(CODE_VAR_XF_LD,x);
    setVarDouble64b(CODE_VAR_YF_LD,y);
    setVarDouble64b(CODE_VAR_TF_LD,t);
}
void DsPIC::setSpSpeed(double vx, double vy, double vt){
	setVarDouble64b(CODE_VAR_SPEED_X_LD,vx);
    setVarDouble64b(CODE_VAR_SPEED_Y_LD,vy);
    setVarDouble64b(CODE_VAR_SPEED_T_LD,vt);
}
void DsPIC::setVarDouble64b(uint8_t varCode, double Var){
    double *ptrVar = &Var;
    uint8_t *ptr = (uint8_t*)ptrVar;
    uint8_t buffer[RX_SIZE_SET_64b + 1];
    buffer[0] = RX_SIZE_SET_64b;
    buffer[1] = RX_CODE_SET;
    buffer[2] = varCode;
    buffer[3] = VAR_LD_64b;
    buffer[4] = ptr[0];
    buffer[5] = ptr[1];
    buffer[6] = ptr[2];
    buffer[7] = ptr[3];
    buffer[8] = ptr[4];
    buffer[9] = ptr[5];
    buffer[10] = ptr[6];
    buffer[11] = ptr[7];
    buffer[12] = 0;
    for(int i = 0; i < RX_SIZE_SET_64b; i++){
        buffer[12] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_SET_64b + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}
void DsPIC::setVar32(uint8_t varCode, uint32_t var){

    uint8_t buffer[RX_SIZE_SET_32b + 1];
    buffer[0] = RX_SIZE_SET_32b;
    buffer[1] = RX_CODE_SET;
    buffer[2] = varCode;
    buffer[3] = VAR_32b;
    buffer[4] = (uint8_t)(var >> 24);
    buffer[5] = (uint8_t)(var >> 16);
    buffer[6] = (uint8_t)(var >> 8);
    buffer[7] = (uint8_t)(var & 0xFF);
    buffer[8] = 0;
    for(int i = 0; i < RX_SIZE_SET_32b; i++){
        buffer[8] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_SET_32b + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}

void DsPIC::setVar8(uint8_t varCode, uint8_t var){

    uint8_t buffer[RX_SIZE_SET_8b + 1];
    buffer[0] = RX_SIZE_SET_8b;
    buffer[1] = RX_CODE_SET;
    buffer[2] = varCode;
    buffer[3] = VAR_8b;
    buffer[4] = var;
    buffer[5] = 0;
    for(int i = 0; i < RX_SIZE_SET_8b; i++){
        buffer[5] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_SET_8b + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}
void DsPIC::getVar(uint8_t varCode){
    uint8_t buffer[RX_SIZE_GET + 1];
    buffer[0] = RX_SIZE_GET;
    buffer[1] = RX_CODE_GET;
    buffer[2] = varCode;
    buffer[3] = 0;
    for(int i = 0; i < RX_SIZE_GET; i++){
        buffer[3] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_GET + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}
void DsPIC::loadPID(){
    getVar(CODE_VAR_ALLPID);
}
std::string DsPIC::async_read(){
    std::string s("");
    while (serialDataAvail(fd)){
      s += serialGetchar(fd);
    }
    return s;
}
 std::vector<uint8_t> DsPIC::readMsg(){
    //double delayUs = 1000000 / BAUDRATE;  // T = 1/f en µs    (0.5Mbaud => 2µs)
    
    int foo = serialGetchar(fd);
    while(foo == -1){   //no reception during 10 sec
        //delayMicroseconds(delayUs);
        foo = serialGetchar(fd);
    }
    uint8_t RxSize = foo;
    std::vector<uint8_t> RxBuf;
    RxBuf.push_back(RxSize);

    for(int i = 0; i < RxSize; i++){
        foo = serialGetchar(fd);
        while(foo == -1){
            //delayMicroseconds(delayUs);
            foo = serialGetchar(fd);
        }
        RxBuf.push_back(foo);
        //delayMicroseconds(delayUs);
        //delayMicroseconds(5);
    }

    return RxBuf;
}
