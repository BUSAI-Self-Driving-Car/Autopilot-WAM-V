#include <thread>
#include "Stepper.h"

#include <iostream>
#include <nuclear>

using namespace module::actuator;

Stepper::Stepper(utility::io::uart& rs485, float thetaMax, float thetaMin)
    : rs485(rs485)
    , homed(false)
    , writing(false)
    , motorMoving(false)
    , pref("@01")
    , thetaMax(thetaMax)
    , thetaMin(thetaMin)
{}

void Stepper::read()
{
    for (int v=rs485.get(); v>=0; v=rs485.get())
    {
        if (v=='\r')
        {
            process(buffer, lastCommand);
            buffer.clear();
            command_sent();
        }
        else
        {
            buffer.push_back(v);
        }
    }
}

void Stepper::command_sent()
{
    {
        std::lock_guard<std::mutex> lg(command_mtx);
        if (!command_queue.empty()) { command_queue.pop_front(); }
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

    if (std::regex_search(buff, errM))
    {
        error(buff);
    }
    else if (std::regex_search(buff, fine))
    {
        {
            std::lock_guard<std::mutex> lg(command_mtx);
            if (!command_queue.empty()) { command_queue.pop_front(); }
        }
        writing = false;
        write_command();
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
        command("CLR", true);
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
}

void Stepper::error(std::string buff)
{
    std::regex errMoving("\\?Moving");
    std::regex errNoMove("\\?ABS/INC");
    std::regex errState("\\?State Error");

    // clearing error state
    if (std::regex_search(buff, errState))
    {
        write_command("CLR");
    }

    // reinitiating last position move command with OTF target change
    {
        std::lock_guard<std::mutex> lg(command_mtx);
        if (!command_queue.empty())
        {
            std::regex move("@01[XT]");

            if (std::regex_search(command_queue.front().first, move))
            {
                if (std::regex_match(buff, errMoving))
                {
                    command_queue.front().first = pref + "T" + std::to_string(targetPos) + "\r";
                }
                // reinitiating last position move command with non-OTF target change
                else if (std::regex_search(buff, errNoMove))
                {
                    command_queue.front().first = pref + "X" + std::to_string(targetPos) + "\r";
                }
                else { return; }
            }
            else { return; }
        }
        else { return; }
    }

    writing = false;
    write_command();
}

void Stepper::run()
{
    const auto time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - timeSent);

    if (time > std::chrono::milliseconds(200))
    {
        // No packet has been received in time
        {
            {
                std::lock_guard<std::mutex> lg(command_mtx);
                if (!command_queue.empty())
                {
                    if (!command_queue.front().second)
                    {
                        command_queue.pop_front();
                    }
                }
            }
            writing = false;
        }

        write_command();
    }

    // Send position command
    if (homed) { command("X"+std::to_string(targetPos), false); }
}

void Stepper::write_command(std::string command)
{
    std::lock_guard<std::mutex> lg(command_mtx);
    command_queue.push_front(std::make_pair(pref + command + "\r", true));
    writing = false;

    // get length of command string
    size_t len = command.length();

    buffer.clear();
    rs485.utility::io::uart::write(command.c_str(),len);
    timeSent = std::chrono::high_resolution_clock::now();
    writing = true;
}

void Stepper::write_command()
{
    std::lock_guard<std::mutex> lg(command_mtx);

    if (!writing)
    {
        if (command_queue.empty()) { return; }
        const auto& com = command_queue.front().first;
        lastCommand = com;

        // get length of command string
        size_t len = com.length();

        buffer.clear();
        rs485.utility::io::uart::write(com.c_str(),len);
        timeSent = std::chrono::high_resolution_clock::now();
        writing = true;
    }
}

void Stepper::command(std::string com, bool critical)
{
    {
        std::lock_guard<std::mutex> lg(command_mtx);
        command_queue.push_back(std::make_pair(pref + com + "\r", critical));
    }
    write_command();
}

void Stepper::begin()
{
    // Absolute mode @dnABS
    command("ABS", true);

    // Acceleration @dnACC=100 (value in milliseconds)
    uint8_t accel = 100;
    command("ACC=" + std::to_string(accel), true);

    // Set current limit @dnDRVIC=2800 (2800mA)
    int curr = 2800;
    command("DRVIC="+std::to_string(curr), true);

    // Set micro-stepping @dnDRVMS=2
    command("DVRMS=2", true);

    // Set high & low speed @dnLSPD=(value in PPS) @dnHSPD=(value in PPS)
    command("HSPD="+std::to_string(HSPD), true);
    command("LSPD="+std::to_string(LSPD), true);

    // Write driver parameters
    // @dnRW
    command("RW", true);
}

int Stepper::get_current_position()
{
    std::cout << "REQ" << std::endl;
    posRequest = true;
    command("PX", true);
    std::unique_lock<std::mutex> lk(pos_mtx);
    pos_cond_var.wait(lk);

    return pulsePos;
}

void Stepper::motorHome()
{
    homed = false;
    // send motor to -Limit (either with jog or low speed home, to be tested)
    // @dnJ- (for jog) @dnHL- (LShome)
    command("L-", true);

    std::unique_lock<std::mutex> lk(stop_mtx);
    motorMoving = false;
    stop_cond_var.wait(lk);
    std::cout << "MINIMUM LIMIT REACHED" << std::endl;

    // send motor to +Limit
    command("J+", true);

    motorMoving = false;
    stop_cond_var.wait(lk);
    std::cout << "MAXIMUM LIMIT REACHED" << std::endl;

    auto pos = get_current_position();

    pulseMin = -pos/2;
    pulseMax = pos/2;
    conv = pos / (thetaMax - thetaMin);

    targetPos = 0;
    command("PX="+std::to_string(pos/2), true);

    homed = true;
}

void Stepper::motorEnable(bool motEn)
{
    if (motEn) { command("EO=1", true); }
    else { command("EO=0", true); }
}

void Stepper::motorStop(bool motStop)
{
    if (motStop) { command("STOP", true); }
    else { command("ABORT", true); }
}

void Stepper::azimuth(float theta)
{
    const auto thetaSat = std::max(thetaMin, std::min(thetaMax, theta));
    targetPos = thetaSat*conv;
}

void Stepper::motorStatus()
{
    command("MST", false);
}

void Stepper::motorPos()
{
    command("PX", false);
}

