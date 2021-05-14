/*******************************************************************************
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

#pragma once

#include "indidome.h"
#include <modbus/modbus-tcp.h>

class RolLOGO : public INDI::Dome
{
  public:
    RolLOGO();
    virtual ~RolLOGO() override = default;

    virtual bool initProperties() override;
    const char *getDefaultName() override;
    virtual void ISGetProperties(const char * dev) override;
    bool updateProperties() override;
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
    virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
    virtual bool saveConfigItems(FILE *fp) override;
    virtual bool Handshake() override;

    modbus_t *ctx;

  protected:

    // Roof motion timoeut
    INumber TimeoutP[1];
    INumberVectorProperty TimeoutNP;

    virtual bool Connect() override;
    virtual bool Disconnect() override;

    void TimerHit() override;

    virtual IPState Move(DomeDirection dir, DomeMotionCommand operation) override;
    virtual IPState Park() override;
    virtual IPState UnPark() override;
    virtual bool Abort() override;

    void checkRoofStatus();
    bool roofAbort();
    bool setFlag(int addr, int bitval);
    bool clearFlags();
    bool initialContact();

private:
    bool initRoofStatus();
    float CalcTimeLeft(timeval start);
    int MotionTimeFrame { -1 };
    int MotionTimeLeft = 0;
    int LoopID;

    ILight RoofStatusL_North[2];
    ILightVectorProperty RoofStatusLP_North;
    ILight RoofStatusL_South[2];
    ILightVectorProperty RoofStatusLP_South;
    enum { ROOF_STATUS_OPENED, ROOF_STATUS_CLOSED };

    ISState is_open_North { ISS_OFF };
    ISState is_closed_North { ISS_OFF };
    ISState is_open_South { ISS_OFF };
    ISState is_closed_South { ISS_OFF };

};

