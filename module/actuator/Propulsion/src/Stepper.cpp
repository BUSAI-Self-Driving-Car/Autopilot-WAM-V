#include <thread>
#include "Stepper.h"

#include <iostream>
#include <nuclear>

using namespace module::actuator;

Stepper::Stepper(utility::io::uart& rs485)
    : rs485(rs485)
    , homed(false)
    , writing(false)
    , motorMoving(false)
    , pref("@01")
{}

void Stepper::read()
{

    for (int v=rs485.get(); v>=0; v=rs485.get())
    {
        if (v=='\r'){
            process(buffer, lastCommand);
            buffer.clear();
            command_sent();
        }
        else {
            buffer.push_back(v);
        }
    }
}

void Stepper::command_sent()
{
    {
        std::lock_guard<std::mutex> lg(command_mtx);
        if (!command_queue.empty()) { command_queue.pop(); }
    }

    writing = false;
    write_command();
}

void Stepper::process(std::string buff, std::string lc)
{
    std::regex errM("\\?");
    std::regex fine("OK");
    std::regex stat("MST");
    std::regex get_pos("PX");
    std::regex pos("pos");

    std::cout << "RX: " << buff << std::endl;

    if (std::regex_search(buff, errM))
    {
        error(buff);
    }
    else if (std::regex_search(buff, fine))
    {
    }
    else if (std::regex_search(lc, stat))
    {
        // motor status
        std::regex decimals("(\\d+)");
        std::smatch match;
        auto result = std::regex_search(buff, match, decimals);
        if (result) statGrab(match.str(1));
    }
    else if (std::regex_search(lc, get_pos))
    {
        // motor status
        std::regex decimals("(\\d+)");
        std::smatch match;
        auto result = std::regex_search(buff, match, decimals);
        if (result) pulsePos = stoi(match.str(1));
        if (posRequest)
        {
            posRequest = false;
            pos_cond_var.notify_all();
        }
    }
    else if (std::regex_match(lc, pos))
    {
        // encoder position
        posEnc = atoi(buff.c_str());
        encBearing = (float)posEnc/conv;
    }

}

void Stepper::statGrab(std::string bf)
{
    int statInt = stoi(bf);

    if ((statInt & CONSTANT_SPEED)
            || (statInt & ACCELERATING)
            || (statInt & DECELERATING))
    {
        motorMoving = true;
    }
    else
    {
        if (motorMoving)
        {
            stop_cond_var.notify_all();
            motorMoving = false;
        }
    }

    if ((statInt & MINUS_LIMIT_ERROR)
            || (statInt & PLUS_LIMIT_ERROR))
    {
        command("CLR");
        if (!limit)
        {
            limit = true;
            std::cout << "LIMIT ERROR" << std::endl;
        }
    }
    else
    {
        limit = false;
    }

    std::cout << "STAT: " << bf << ": " << statInt << std::endl;
}

void Stepper::error(std::string buff)
{
    std::regex errMoving("\\?Moving");
    std::regex errNoMove("\\?ABS/INC");
    std::regex errState("\\?State Error");

    std::cout << buff << std::endl;

    // reinitiating last position move command with OTF target change
    if (std::regex_match(buff.begin(), buff.end(), errMoving))
    {
        command("T" + pulsePos);
    }
    // reinitiating last position move command with non-OTF target change
    else if (std::regex_search(buff.begin(), buff.end(), errNoMove))
    {
        command("X" + pulsePos);
    }
    // clearing error state
    else if (std::regex_search(buff.begin(), buff.end(), errState))
    {
        std::cout << "ERROR STATE" << std::endl;
        command("CLR");
    }
}

void Stepper::run()
{
    const auto time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - timeSent);

    if (time.count() > 500)
    {
        // No packet has been received in time
        {
            std::lock_guard<std::mutex> lg(command_mtx);
            if (!command_queue.empty()) command_queue.pop();
            writing = false;
        }
        write_command();
    }
}

void Stepper::write_command()
{
    if (!writing)
    {
        std::lock_guard<std::mutex> lg(command_mtx);
        if (command_queue.empty()) { return; }
        const auto& com = command_queue.front();
        lastCommand = com;

        // get length of command string
        std::cout << "TX: " << com << std::endl;
        size_t len = com.length();

        buffer.clear();
        rs485.utility::io::uart::write(com.c_str(),len);
        timeSent = std::chrono::high_resolution_clock::now();
        writing = true;
    }
}

void Stepper::command(std::string com)
{
    {
        std::lock_guard<std::mutex> lg(command_mtx);
        command_queue.push(pref + com + "\r");
    }
    write_command();
}

void Stepper::begin()
{
    // Absolute mode @dnABS
    command("ABS");

    // Acceleration @dnACC=100 (value in milliseconds)
    uint8_t accel = 100;
    command("ACC=" + std::to_string(accel));

    // Set current limit @dnDRVIC=2800 (2800mA)
    int curr = 2800;
    command("DRVIC="+std::to_string(curr));

    // Set micro-stepping @dnDRVMS=2
    command("DVRMS=2");

    // Set high & low speed @dnLSPD=(value in PPS) @dnHSPD=(value in PPS)
    command("HSPD="+std::to_string(HSPD));
    command("LSPD="+std::to_string(LSPD));

    // Write driver parameters
    // @dnRW
    command("RW");
}

int Stepper::get_current_position()
{
    std::cout << "REQ" << std::endl;
    posRequest = true;
    command("PX");
    std::unique_lock<std::mutex> lk(pos_mtx);
    pos_cond_var.wait(lk);

    std::cout << "DONE" << std::endl;
    return pulsePos;
}

void Stepper::move(int x)
{
    command("X" + std::to_string(x));
}

void Stepper::motorHome()
{
    // send motor to -Limit (either with jog or low speed home, to be tested)
    // @dnJ- (for jog) @dnHL- (LShome)
    command("L-");

    std::unique_lock<std::mutex> lk(stop_mtx);
    motorMoving = true;
    stop_cond_var.wait(lk);

    // send motor to +Limit
    command("J+");

    motorMoving = true;
    stop_cond_var.wait(lk);
    std::cout << "MAXIMUM LIMIT REACHED" << std::endl;

    auto pos = get_current_position();

    move(pos/2);
    pulseMin = -pos/2;
    pulseMax = pos/2;
    conv = pos / (thetaMax - thetaMin);

    motorMoving = true;
    stop_cond_var.wait(lk);
    command("PX=0");

    homed = true;
}

void Stepper::motorEnable(bool motEn)
{
    if (motEn) { command("EO=1"); }
    else { command("EO=0"); }
}

void Stepper::motorStop(bool motStop)
{
    if (motStop) { command("STOP"); }
    else { command("ABORT"); }
}

void Stepper::azimuth(float theta)
{
    const auto thetaSat = std::max(thetaMin, std::min(thetaMax, theta));
    int pos = thetaSat*conv;
    move(pos);
}

void Stepper::motorStatus()
{
    command("MST");
}

void Stepper::motorPos()
{
    command("PX");
}

