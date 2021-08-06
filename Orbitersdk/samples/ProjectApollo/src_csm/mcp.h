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
public:
	MCP_GCC();
	void Init(Saturn *s);
	void Timestep(double simdt);

	void SetLETJettison() { RTC40LETJettison = true;}
	void SetAbort() { RTC71Abort = true; }

	bool GetLETJettison() { return RTC40LETJettison; }
	bool GetAbort() { return RTC71Abort; }
protected:
	Saturn *Sat;

	//RTCs
	bool RTC40LETJettison;
	bool RTC71Abort;
};

//Spacecraft Command Controller
class MCP_SCC
{
public:
	MCP_SCC();
	void Init(Saturn *s, MCP_GCC*g);
	void Timestep(double simdt);
	void ProgramerReset();

	//Signals to external systems
	bool GetFireArm(bool IsSysA);
	bool GetFireSafe(bool IsSysA);
	bool GetLESMotorFire(bool IsSysA);
	bool GetCMSMSep(bool IsSysA);
	bool GetELSActive(bool IsSysA);
	bool GetRestartSignal();
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
	bool GetCMSMSepA();
	bool GetCMSMSepB();
	bool GetELSActiveA();
	bool GetELSActiveB();

	//5?
	bool LVSCSep25sSignal;
	//6
	bool LESAbortSignal;
	//7
	bool CSMSepSignal;
	//8
	bool LETJettisonSignal;
	//9
	bool b005GSignal;
	//13
	bool LVSCSepSignal;

	//Relays (latching)

	//SECS
	bool R2K9A;
	bool R2K9B;
	bool R2K11A;
	bool R2K11B;
	bool R2K14A;
	bool R2K14B;
	bool R2K15A;
	bool R2K15B;
	bool R2K17A;
	bool R2K17B;
	bool R2K20A;
	bool R2K20B;
	bool R2K10AB;
	bool R2K16AB;
	bool R2K132; //Liftoff TBD: Check number
	bool R2K133A; //Check
	bool R2K133B; //Check
	bool R2K135;
	bool R2K137;
	bool R2K138A;
	bool R2K138B;
	bool R2K139A; //Check
	bool R2K139B; //Check
	bool R2K142;
	bool R2K222;

	//Relays (non-latching)
	//LET Jettison
	bool R2K187;
	//CM/SM Sep
	bool R2K188;
	//Abort Inhibit?
	bool R2K140;
	//LET Jet
	bool R2K136;
	//Liftoff +42 seconds
	bool R2K177;
	bool R2K141;
	//LV/SC + 2.5 seconds
	bool R2K202; //Check
	//Low LES Abort
	bool R2K133;

	//Timers
	DelayTimer LESMotorFireTimer;
	DelayTimer CMSMSepTimer;
	DelayTimer LiftoffTimer;
	DelayTimer LVSCSep25sTimer;

	Saturn *Sat;
	MCP_GCC *gcc;
};

//Attitude and Deceleration Sensor
class MCP_ADS
{

};