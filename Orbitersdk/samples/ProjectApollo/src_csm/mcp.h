/***************************************************************************
This file is part of Project Apollo - NASSP
Copyright 2021

Mission Control Programer (Header)

Project Apollo is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

Project Apollo is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Project Apollo; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

See http://nassp.sourceforge.net/license/ for more details.

**************************************************************************/

#pragma once

#include "DelayTimer.h"

class Saturn;

//Ground Command Controller
class MCP_GCC
{

};

//Spacecraft Command Controller
class MCP_SCC
{
public:
	MCP_SCC();
	void Init(Saturn *s);
	void Timestep(double simdt);
	void ProgramerReset();

	//Signals to external systems
	bool GetFireArm(bool IsSysA);
	bool GetFireSafe(bool IsSysA);
	bool GetLESMotorFire(bool IsSysA);
protected:

	bool IsPowered();
	void DeterminePowerState();

	bool bPower;

	bool GetFireSafeA();
	bool GetFireSafeB();
	bool GetFireArmA();
	bool GetFireArmB();
	bool GetLESMotorFireA();
	bool GetLESMotorFireB();

	//Relays (latching)

	//SECS
	bool R2K9A;
	bool R2K9B;
	bool R2K15A;
	bool R2K15B;
	bool R2K10AB;
	bool R2K16AB;
	bool R2K135;
	bool R2K222;

	//Relays (non-latching)
	//LET Jettison
	bool R2K187;

	//Timers
	DelayTimer LESMotorFireTimer;

	Saturn *Sat;
};

//Attitude and Deceleration Sensor
class MCP_ADS
{

};