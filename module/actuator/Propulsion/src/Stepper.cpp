#include <thread>
#include "Stepper.h"

namespace module {
namespace actuator {

Stepper::Stepper(utility::io::uart& rs485)
    : rs485(rs485)
    , homed(false)
{}

void Stepper::read(){

    for (int v=rs485.get(); v>=0; v=rs485.get())
    {
        if (v=='\r'){
            process(buffer,lastCommand);
            buffer.clear();
        }
        else {
            buffer.push_back(v);
        }
    }
}

void Stepper::process(std::string buff,std::string lc){

    std::regex errM("?(\w+)");
    std::regex fine("OK");
    std::regex move("move");
    std::regex stat("status");
    std::regex pos("pos");

    if (std::regex_search(buff.begin(),buff.end(),errM)){
        error(buff);

    }
    else if (std::regex_search(buff.begin(),buff.end(),fine)){

    }
    else if (std::regex_match(lc.begin(),lc.end(),stat)){
        // motor status
        statGrab(buff);

    }
    else if (std::regex_match(lc.begin(),lc.end(),pos)){
        // encoder position
        posEnc = atoi(buff.c_str());
        encBearing = (float)posEnc/conv;

    }

}

void Stepper::statGrab(std::string bf){

    int statInt = atoi(bf.c_str());


    if (statInt == 0){
        // motor is idle
        motorMoving = false;
    }
    else if (statInt>0 && statInt<8){
        // motor is moving
        motorMoving = true;
    }
    else if (statInt>=16 && statInt<32){
        motorMoving = true;
        minLim = true;
        motorStop(false);
    }
    else if (statInt>=32 && statInt<64){
        motorMoving = true;
        maxLim = true;
        motorStop(false);
    }
    else if (statInt>=64 && statInt<=80){
        motorMoving = false;
        minLimErr = true;
        minLim = true;
        if (statInt < 80){
            minLim = false;
        }
        command("@01CLR","");

    }
    else if (statInt>=128 && statInt<=160){
        motorMoving = false;
        maxLimErr = true;
        maxLim = true;
        if (statInt < 160){
            maxLim = false;
        }
        command("@01CLR","");
    }


}

void Stepper::error(std::string buff){

    std::regex errMoving("?Moving");
    std::regex errNoMove("?ABS/INC");
    std::regex errState("?State Error");

    // reinitiating last position move command with OTF target change
    if (std::regex_match(buff.begin(),buff.end(),errMoving)){
        std::string a = "@01T" + pulsePos;
        command(a,"move");
    }
    // reinitiating last position move command with non-OTF target change
    else if (std::regex_search(buff.begin(),buff.end(),errNoMove)){
        std::string a = "@01X" + pulsePos;
        command(a,"move");
    }
    // clearing error state
    else if (std::regex_match(buff.begin(),buff.end(),errState)){
        command("@01CLR","");
    }

}

void Stepper::command(std::string com,std::string comType){

    // get length of command string
    size_t len = com.length();

    rs485.utility::io::uart::write(com.c_str(),len);
    lastCommand = comType;

}

void Stepper::begin(){

    std::string pref = "@01";

    // Driver configurations dn is device address (01 or 02)
    // Absolute mode @dnABS
    std::string abs = pref+"ABS";

    command(abs,"init");

    // Acceleration @dnACC=100 (value in milliseconds)

    uint8_t accel = 100;
    std::string acc = pref + "ACC=" + std::to_string(accel);

    command(acc,"init");

    // Set current limit @dnDRVIC=2800 (2800mA)

    int curr = 2800;
    std::string currLim = pref+"DRVIC="+std::to_string(curr);//+curr;

    command(currLim,"init");

    // Set micro-stepping @dnDRVMS=2

    std::string mStep = pref+"DVRMS=2";

    command(mStep,"init");

    // Set high & low speed @dnLSPD=(value in PPS) @dnHSPD=(value in PPS)


    std::string HSP = pref+"HSPD="+std::to_string(HSPD);//+HSPD;
    std::string LSP = pref+"LSPD="+std::to_string(LSPD);//+LSPD;

    command(HSP,"init");
    command(LSP,"init");

    // Write driver parameters
    // @dnRW

    std::string drivPar = pref+"RW";

    command(drivPar,"init");

}

void Stepper::motorHome(){
    // send motor to -Limit (either with jog or low speed home, to be tested)
    // @dnJ- (for jog) @dnHL- (LShome)

    std::string pref = "@01";

    std::string hHSP = pref+"HSPD="+std::to_string(LSPD*2);//+HSPD;
    std::string HSP = pref+"HSPD="+std::to_string(HSPD);//+HSPD;

    // set high speed limit to lower speed for homing
    command(hHSP,"init");

    command("@01J-","move");

    // need to get error code from the read thread
    // clear error & set encoder or position to zero
    // @dnPX=0 (position) @dnEX=0 (encoder) check not sure if internal or external

    while(!minLim){
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
       // wait for motor status, -Limit error code (possibly add in +Limit also in case)
    }

    command("@01CLR","");   // clear error command
    command("@01PX=0","");  // set position to zero

    // send motor to +Limit
    // @dnJ+ or @dnHL+
    command("@01J+","move");

    while(!maxLim){
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        // wait for motor status, +Limit error code
    }

    //wait for error code
    //clear error dode & get pos/enc for delta
    command("@01CLR","");

    // set speed to high again
    command(HSP,"init");

    // get position from poll function
    int maxLimit = posEnc;
    //

    int centrePos = maxLimit/2; // will need to be a large integer

    // move motor to absolute deltaPos/2
    // @dnXdeltaPos/2

    std::string centreHome = "@01X" + std::to_string(centrePos);
    command(centreHome,"move");

    // set this to position zero (for "home")
    command("@01PX=0","");

    homed = true;
}

void Stepper::motorEnable(bool motEn){
    // create ascii string to do things

    if (motEn) {
        // String is @dnEO=1
        std::string en = "@01EO=1";
        command(en,"init");
    }
    else {
        std::string dis = "@01EO=0";
        // String is @dnEO=0
        command(dis,"init");
    }

//    rs485.write(string,length(string))
}

void Stepper::motorStop(bool motStop){

    if (motStop){
        // Strin is @dnSTOP
        std::string stop = "@01STOP";
        command(stop,"");

        motStop = false;
    }
    else  {
        // String is @dnABORT
        std::string ab = "@01ABORT";
        command(ab,"");
    }
}

void Stepper::azimuth(float bear){

    // convert input target bearing into pulse postion

    pulsePos = (int)(bear*conv);

    // Check if target pulse position is within limits

   /* if (pulsePos>maxPulse or pulsePos<minPulse){ // Not sure about or???

        // changing target bearing to within limits
        if (pulsePos>maxPulse)
        {
            pulsePos=maxPulse;
            error(,pulsePos/conv); // return error code, and new target position
        }
        else {
            pulsePos=minPulse;
            error(,pulsePos/conv); // return error code, and new target position
        }

    }*/

    // move to position @dnX[value]
    // get motor code, to see if motor is moving or not

    if (motorMoving){

        std::string target = "@01T="+pulsePos;

        // need to find length of target string

        command(target,"move");
    }
    else if (!motorMoving){
        std::string Xpos = "@01X="+pulsePos;

        command(Xpos,"move");
    }

}

void Stepper::motorStatus(){
    // run this command at regular intervals so that we always know what is going on...
    // Ask for motor status
    command("@01MST","status");
}

void Stepper::motorPos(){
    // run this command at reg intervals
    // Ask for motor encoder position
    command("@01PX","pos");
}

}

}
