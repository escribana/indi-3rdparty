/*******************************************************************************
 Edited version of the Dome Simulator : 2020/21 T. Schriber
 Copyright(c) 2014 Jasem Mutlaq. All rights reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.
 .
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.
 .
 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
*******************************************************************************/


/*
 * Edited version of the rolloff roof simulator.
 * This is specific to a particular installation using an Siemens LOGO! to open and close the roof.
 * The driver uses libmodus to communicate with this controller (SPS).
 *
 */
#include "rollogo.h"
#include "indicom.h" 
#include "termios.h"
#include "connectionplugins/connectioninterface.h"
#include "connectionplugins/connectiontcp.h"

#include <cmath>
#include <cstring>
#include <ctime>
#include <memory>
#include <errno.h>

// Modbus descrete inputs
#define SWITCH_ADDRESS_START  0      // Discrete input first address (offset -1!)
#define SWITCH_ADDRESS_COUNT  4
#define SWITCH_OPEN_SOUTH     0      // LOGO I1 input: switch open means roof open!
#define SWITCH_CLOSED_SOUTH   1      // LOGO I2 input: switch open means roof closed!
#define SWITCH_OPEN_NORTH     3      // LOGO I4 input: switch open means roof open!
#define SWITCH_CLOSED_NORTH   2      // LOGO I3 input: switch open means roof closed!

// Modbus coils
#define RELAY_ROOF_OPEN       8256   // LOGO M1 flag
#define RELAY_ROOF_CLOSE      8257   // LOGO M2 flag

// We declare an auto pointer
std::unique_ptr<RolLOGO> rolLOGO(new RolLOGO());

void ISPoll(void *p);

RolLOGO::RolLOGO()
{
    SetDomeCapability(DOME_CAN_ABORT | DOME_CAN_PARK);           // Need the DOME_CAN_PARK capability for the scheduler
}

/**************************************************************************************
** INDI is asking us for our default device name. 
** Check that it matches Ekos selection menu and ParkData.xml names
***************************************************************************************/
const char *RolLOGO::getDefaultName()
{
    return (const char *)"RollOff LOGO";
}
/**************************************************************************************
** INDI request to init properties.
***************************************************************************************/
bool RolLOGO::initProperties()
{
    INDI::Dome::initProperties();

    IUFillLight(&RoofStatusL_North[ROOF_STATUS_OPENED], "open", "", IPS_IDLE);
    IUFillLight(&RoofStatusL_North[ROOF_STATUS_CLOSED], "closed", "", IPS_IDLE);
    IUFillLightVector(&RoofStatusLP_North, RoofStatusL_North, 2, getDeviceName(), "Roof North", "", MAIN_CONTROL_TAB, IPS_BUSY);
    IUFillLight(&RoofStatusL_South[ROOF_STATUS_OPENED], "open", "", IPS_IDLE);
    IUFillLight(&RoofStatusL_South[ROOF_STATUS_CLOSED], "closed", "", IPS_IDLE);
    IUFillLightVector(&RoofStatusLP_South, RoofStatusL_South, 2, getDeviceName(), "Roof South", "", MAIN_CONTROL_TAB, IPS_BUSY);

    IUFillNumber(&TimeoutP[0], "TIMEOUT", "Timeframe (s)", "%.0f", 0.0, 150.0, 1, 0);
    IUFillNumberVector(&TimeoutNP, TimeoutP, 1, getDeviceName(), "TIMEOUT", "Motion Timout", OPTIONS_TAB, IP_RW, 60, IPS_IDLE);

    SetParkDataType(PARK_NONE); // Sets "indidome" to rolloff! (eg. button "Motion")
    addAuxControls();
    return true;
}

/************************************************************************************
 * Called from Dome, BaseDevice to establish contact with device
 ************************************************************************************/
bool RolLOGO::Handshake()
{
    if (PortFD <= 0)
    {
        DEBUG(INDI::Logger::DBG_WARNING,"Connection to port not established");
        return false;
    }
    return true;
}

/**************************************************************************************
** Client is asking us to establish connection to the device
** Standard connection method is only used for registration. The connection is replaced
** with an actual modbus-connection in "updateProperties()".
** This is a quick & dirty solution to circumvent the creation of a new connectionplugin
** "connectionModBus" (-> expansion[s] of Connection::Interface)
***************************************************************************************/
bool RolLOGO::Connect()
{
    bool status = INDI::Dome::Connect();
    return status;
}

/**************************************************************************************
** Client is asking us to terminate connection to the device
***************************************************************************************/
bool RolLOGO::Disconnect()
{
    bool status = false;

    if (!(status = clearFlags())) // no relay should be active when closing connection
        LOG_ERROR("Controller relays cannot be cleared on disconnect!");
    else
    {
        modbus_close(ctx);
        modbus_free(ctx);
        LOG_INFO("ModBus disconnected!");
        status = INDI::Dome::Disconnect();
    }
    return status;
}

/**************************************************************************************
** Motion timeout has to be loaded, so "MotionTimeframe" can be set before connection
***************************************************************************************/
void RolLOGO::ISGetProperties(const char * dev)
{
    //  First we let our parent populate
    Dome::ISGetProperties(dev);

    defineProperty(&TimeoutNP);
    loadConfig(true, "TIMEOUT");
    MotionTimeFrame = TimeoutP[0].value; // initialise timeframe
    if (MotionTimeFrame <= 0)
        LOG_WARN("Please set motion timeout in 'Options' for roof control!");
    return;
}

/********************************************************************************************
** INDI request to update the properties because there is a change in CONNECTION status
** This function is called whenever the device is connected or disconnected.
** Can't hide and have Park.Park property (INDI::Dome::deleteProperty(ParkSP.name))!
*********************************************************************************************/
bool RolLOGO::updateProperties()
{
    Connection::Interface::Type ctype;
    Connection::Interface *activeConnection = nullptr;
    const char *mb_host = "";
    uint32_t mb_port;
    int mbstat = -1;

    INDI::Dome::updateProperties();
    if (isConnected())
    {
        activeConnection = INDI::Dome::getActiveConnection();
        ctype = activeConnection->type();
        if (activeConnection->Disconnect())  // cf. Infoheader RolLOGO::Connect()!
        {
            if (ctype == Connection::Interface::CONNECTION_TCP)
            {
                mb_host = INDI::Dome::tcpConnection->host();
                mb_port = INDI::Dome::tcpConnection->port();
                ctx = modbus_new_tcp(mb_host,mb_port);
                modbus_set_debug(ctx, true);
                mbstat = modbus_connect(ctx);
            }
            else
            {
                LOG_ERROR("ModBus-RTU connection not yet handled!");
                return false;
            }
            if (mbstat == 0)
            {
                LOG_INFO("ModBus connected!");
                // defineProperty(&TimeoutNP); -> see ISGetProperties()
                defineProperty(&RoofStatusLP_North);
                defineProperty(&RoofStatusLP_South);
            }
            else
            {
                LOGF_ERROR("ModBus connnect error: %s.", modbus_strerror(errno));
                modbus_free(ctx);
                return false;
            }
            return (initRoofStatus());
        }
        LOG_ERROR("No active connection!");
        return false;
    }
    else
    {
        // deleteProperty(TimeoutNP.name); -> see ISGetProperties()
        deleteProperty(RoofStatusLP_North.name);  // Light
        deleteProperty(RoofStatusLP_South.name);  // Light
        return true;
     }
}

/********************************************************************************************
** Roof status on a connect.
*********************************************************************************************/
bool RolLOGO::initRoofStatus()
{
    bool status_ok = false;

    checkRoofStatus();
    InitPark(); // saved dome parking data (XML-file); returns unparked if XML-file error
    if (isParked())
    {
        if ((is_closed_North == ISS_OFF) || (is_closed_South == ISS_OFF))
        {
            LOG_INFO("Roof is already open or opening");
            MotionTimeLeft = MotionTimeFrame;
            DomeMotionSP.s = IPS_BUSY;
            DomeMotionS[DOME_CW].s = ISS_ON;    // Roof open switch
            DomeMotionS[DOME_CCW].s = ISS_OFF;  // Roof close switch
        }
        status_ok = true;
    }
    else  // unparkes (or XML-file error: to improve)
    {
        if ((is_open_North == ISS_OFF) || (is_open_South == ISS_OFF))
        {
            LOG_INFO("Roof is already closed or closeing");
            MotionTimeLeft = MotionTimeFrame;
            DomeMotionSP.s = IPS_BUSY;
            DomeMotionS[DOME_CW].s = ISS_OFF;   // Roof open switch
            DomeMotionS[DOME_CCW].s = ISS_ON;   // Roof close switch
        }
        status_ok = true;
    }
    return status_ok;
}

void RolLOGO::checkRoofStatus()
{
    int rc;
    int nb = SWITCH_ADDRESS_COUNT;
    uint8_t *bits;
    bits = (uint8_t *) malloc(nb*sizeof(uint8_t));
    memset(bits, 0, nb*sizeof(uint8_t));
    rc = modbus_read_input_bits(ctx, SWITCH_ADDRESS_START, nb, bits);

    if (rc == nb)
    {
        RoofStatusL_North[ROOF_STATUS_CLOSED].s = IPS_IDLE;
        RoofStatusL_North[ROOF_STATUS_OPENED].s = IPS_IDLE;
        RoofStatusL_South[ROOF_STATUS_CLOSED].s = IPS_IDLE;
        RoofStatusL_South[ROOF_STATUS_OPENED].s = IPS_IDLE;
        RoofStatusLP_North.s = IPS_IDLE;
        RoofStatusLP_South.s = IPS_IDLE;

        // Look out for inverted logic! (see defines)
        if ((is_open_North = bits[SWITCH_OPEN_NORTH] ? ISS_OFF : ISS_ON))
        {
            RoofStatusL_North[ROOF_STATUS_OPENED].s = IPS_OK;
            RoofStatusLP_North.s = IPS_OK;
        }
        else if ((is_closed_North = bits[SWITCH_CLOSED_NORTH] ? ISS_OFF : ISS_ON))
        {
            RoofStatusL_North[ROOF_STATUS_CLOSED].s = IPS_OK;
            RoofStatusLP_North.s = IPS_OK;
        }
        else
        {
            RoofStatusL_North[ROOF_STATUS_CLOSED].s = IPS_BUSY;
            RoofStatusL_North[ROOF_STATUS_OPENED].s = IPS_BUSY;
            RoofStatusLP_North.s = IPS_BUSY;
        }
        if ((is_open_South = bits[SWITCH_OPEN_SOUTH] ? ISS_OFF : ISS_ON))
        {
            RoofStatusL_South[ROOF_STATUS_OPENED].s = IPS_OK;
            RoofStatusLP_South.s = IPS_OK;
        }
        else if ((is_closed_South = bits[SWITCH_CLOSED_SOUTH] ? ISS_OFF : ISS_ON))
        {
            RoofStatusL_South[ROOF_STATUS_CLOSED].s = IPS_OK;
            RoofStatusLP_South.s = IPS_OK;
        }
        else
        {
            RoofStatusL_South[ROOF_STATUS_CLOSED].s = IPS_BUSY;
            RoofStatusL_South[ROOF_STATUS_OPENED].s = IPS_BUSY;
            RoofStatusLP_South.s = IPS_BUSY;
        }
    }
    else
    {
        RoofStatusLP_North.s = IPS_ALERT;
        RoofStatusLP_South.s = IPS_ALERT;
        LOGF_ERROR("ModBus read error: %s.", modbus_strerror(errno));
    }

    free(bits);
    IDSetLight(&RoofStatusLP_North, nullptr);
    IDSetLight(&RoofStatusLP_South, nullptr);
}

/********************************************************************************************
** Client request to update a switch
*********************************************************************************************/
bool RolLOGO::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    return INDI::Dome::ISNewSwitch(dev, name, states, names, n);
}

bool RolLOGO::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (!strcmp(name, TimeoutNP.name))
        {
            int timeout  = round(values[0]);
            TimeoutP[0].value = static_cast<int>(timeout);
            TimeoutNP.s = IPS_OK;
            IDSetNumber(&TimeoutNP, nullptr);
            return true;
        }
    }

    return INDI::Dome::ISNewNumber(dev, name, values, names, n);
}


/********************************************************************************************
** Each 1 second timer tick
********************************************************************************************/
void RolLOGO::TimerHit()
{

    uint32_t delay = 1000 * 300;   // default timer setting if no interface activity

    if (!isConnected())
        return; //  No need to reset timer if we are not connected anymore

    checkRoofStatus();
    if (DomeMotionSP.s == IPS_BUSY)
    {

        if (DomeMotionS[DOME_CW].s == ISS_ON) // Roll off is open/ing
        {
            if ((is_open_North == ISS_ON) && (is_open_South == ISS_ON))
            {
                LOG_INFO("Roof is open");
                MotionTimeLeft = 0;
                clearFlags();
                SetParked(false);
                DomeMotionSP.s = IPS_OK;             // show success in indi-UI
                IDSetSwitch(&DomeMotionSP, nullptr); // DOME_UNPARKED shows IPS_IDLE!
            }
            else if (MotionTimeLeft <= 0)
                LOG_WARN("Rolloff motion timeout has expired while opening!");
            else
            {
                MotionTimeLeft -= 1;
                delay = 1000;           // opening active
            }
        }
        else if (DomeMotionS[DOME_CCW].s == ISS_ON) // Roll Off is close/ing
        {
            if ((is_closed_North == ISS_ON) && (is_closed_South == ISS_ON))
            {
                LOG_INFO("Roof is closed");
                MotionTimeLeft = 0;
                clearFlags();
                SetParked(true);
                DomeMotionSP.s = IPS_OK;             // show success in indi-UI
                IDSetSwitch(&DomeMotionSP, nullptr); // DOME_UNPARKED shows IPS_IDLE!
            }
            else if (MotionTimeLeft <= 0)
                LOG_WARN("Rolloff motion timeout has expired while closing!");
            else
            {
                delay = 1000;           // closing active
                MotionTimeLeft -= 1;
            }
        }
        else
        {
            clearFlags();
            setDomeState(DOME_IDLE);
        }
    }
    /* Even when no roof movement requested, will come through occasionally. Use timer to update roof status
    * in case roof has been operated externally by a remote control, locks applied...
    * LOGF_INFO("*** Timerhit: Delay set to: %ds", delay/1000);
    * LOGF_INFO("*** Timerhit: Time left for motion: %ds", MotionTimeLeft);
    */
    LoopID = SetTimer(delay);
}

bool RolLOGO::saveConfigItems(FILE *fp)
{
    LOG_DEBUG(__FUNCTION__);
    IUSaveConfigNumber(fp, &TimeoutNP);
    return INDI::Dome::saveConfigItems(fp);
}

/*
 * Direction: DOME_CW Clockwise = Open; DOME-CCW Counter clockwise = Close
 * Operation: MOTION_START, | MOTION_STOP
 */
IPState RolLOGO::Move(DomeDirection dir, DomeMotionCommand operation)
{
    MotionTimeFrame = TimeoutP[0].value;
    if (MotionTimeFrame == 0)
    {
        LOG_WARN("Please set motion timeout in 'Options' for proper operation!");
        return IPS_ALERT;
    }
    else
    {
        checkRoofStatus();
        if ((operation == MOTION_START) && (DomeMotionSP.s != IPS_BUSY))
        {
            if (dir == DOME_CW) // Open Roof
            {
                if ((is_open_North == ISS_ON) && (is_open_South == ISS_ON))
                {
                    LOG_WARN("Roof is already fully opened");
                    return IPS_ALERT;
                }
                // Initiate action
                if (setFlag(RELAY_ROOF_OPEN, 1))
                    LOG_INFO("Roof opening initiated ...");
                else
                {
                    LOG_WARN("Roof opening failed");
                    return IPS_ALERT;
                }
            }
            else if (dir == DOME_CCW) // Close Roof
            {
                if ((is_closed_North == ISS_ON) && (is_closed_South == ISS_ON))
                {
                    LOG_WARN("Roof is already fully closed");
                    return IPS_ALERT;
                }
                else if (INDI::Dome::isLocked())
                {
                    DEBUG(INDI::Logger::DBG_WARNING, "Cannot close dome when mount is locking. See: Telescope parking policy!");
                    return IPS_ALERT;
                }
                // Initiate action
                if (setFlag(RELAY_ROOF_CLOSE, 1))
                    LOG_INFO("Roof closing initiated ...");
                else
                {
                    LOG_WARN("Roof closing failed");
                    return IPS_ALERT;
                }
            }
            MotionTimeLeft = MotionTimeFrame;
            //early stop of Mainloop
            RemoveTimer(LoopID);
            LoopID = SetTimer(1000);
            return IPS_BUSY; // let TimerHit() do the work!
        }
        else // MOTION_STOP || already moving
            return (Abort() ? IPS_IDLE : IPS_ALERT);
    }
}
/*
 * Close Roof
 *
 */
IPState RolLOGO::Park()
{
    IPState rc = INDI::Dome::Move(DOME_CCW, MOTION_START);

    if (rc == IPS_BUSY)
    {
        LOG_INFO("RollOff roof is parking...");
        return IPS_BUSY;
    }
    else
        return IPS_ALERT;
}

/*
 * Open Roof
 *
 */
IPState RolLOGO::UnPark()
{
    IPState rc = INDI::Dome::Move(DOME_CW, MOTION_START);
    if (rc == IPS_BUSY)
    {
        LOG_INFO("RollOff roof is unparking...");
        return IPS_BUSY;
    }
    else
        return IPS_ALERT;
}

/*
 * Abort motion
 */
bool RolLOGO::Abort()
{
    bool opened = false;
    bool closed = false;

    checkRoofStatus();
    opened = ((is_open_North == ISS_ON) && (is_open_South == ISS_ON));
    closed = ((is_closed_North == ISS_ON) && (is_closed_South == ISS_ON));

    if (closed && DomeMotionSP.s != IPS_BUSY)
    {
        LOG_WARN("Roof appears to be closed and stationary, no action taken");
        return true;
    }
    else if (opened && DomeMotionSP.s != IPS_BUSY)
    {
        LOG_WARN("Roof appears to be open and stationary, no action taken");
        return true;
    }
    else if (DomeMotionSP.s != IPS_BUSY)
    {
        LOG_WARN("Roof appears to be partially open and stationary, no action taken");
    }
    else if (DomeMotionSP.s == IPS_BUSY)
    {
        if (DomeMotionS[DOME_CW].s == ISS_ON)
        {
            LOG_WARN("Abort action requested on opening roof");
        }
        else if (DomeMotionS[DOME_CCW].s == ISS_ON)
        {
            LOG_WARN("Abort action requested on closing roof");
        }
        if (clearFlags())
            setDomeState(DOME_IDLE);
        else
            setDomeState(DOME_ERROR);
    }

    // If all limit switches are on, then we're neither parked nor unparked.
    if (!(opened) && !(closed))
    {
        IUResetSwitch(&ParkSP);
        ParkSP.s = IPS_IDLE;
        IDSetSwitch(&ParkSP, nullptr);
    }
    return true;
}

bool RolLOGO::setFlag(int addr, int bitval)
{
    int rc;
    uint8_t *bit;
    bit = (uint8_t *) malloc(sizeof(uint8_t));
    memset(bit, bitval, sizeof(uint8_t));

    rc = modbus_write_bits(ctx, addr, 1, bit);
    if (rc != 1)
    {
        LOGF_ERROR("ModBus write error: %s.", modbus_strerror(errno));
        return false;
    }
    return true;
}

bool RolLOGO::clearFlags()
{
    if (!((setFlag(RELAY_ROOF_OPEN, 0)) && (setFlag(RELAY_ROOF_CLOSE, 0))))
    {
        LOGF_ERROR("ModBus write error: %s.", modbus_strerror(errno));
        return false;
    }
    return true;
}
