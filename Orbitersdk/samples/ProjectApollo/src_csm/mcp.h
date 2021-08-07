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
#include "MechanicalAccelerometer.h"

class Saturn;

class Differentiator
{
public:
	Differentiator();
	bool EvaluateState(bool in);
protected:
	bool State;
};



//Attitude and Deceleration Sensor
class MCP_ADS
{
public:
	MCP_ADS();
	void Init(VESSEL *v);
	void Timestep(double simdt);
	void SaveState(FILEHANDLE scn);
	void LoadState(FILEHANDLE scn);

	bool GetImpactSwitch() { return ImpactSwitch; }
	bool Get005GSwitch() { return b005GSwitch; }
	bool GetAttitudeSwitch() { return AttitudeSwitch; }
protected:
	MechanicalAccelerometer Accelerometer;

	bool ImpactSwitch;
	bool b005GSwitch;
	bool AttitudeSwitch; //false = Stable I, true = Stablie II 
};

class MCP_SCC;

//Ground Command Controller
class MCP_GCC
{
public:
	MCP_GCC();
	void Init(Saturn *s, MCP_SCC *sc);
	void Timestep(double simdt);
	void SaveState(FILEHANDLE scn);
	void LoadState(FILEHANDLE scn);

	void RealTimeCommand(int cmd);
	void ProgramerReset();

	bool GetLETJettison() { return RTC40LETJettison; }
	bool GetAbort() { return RTC71Abort; }
	bool GetCSMSep() { return RTC61CSMSep; }
	bool GetRTC53GNAntennaSwitching() { return RTC53GNAntennaSwitching; }

	bool GetGNFail() { return R1K60; }
	bool GetGNFailInhibit() { return R1K61; }
protected:

	void InputReset();

	Saturn *Sat;

	//RTCs
	bool RTC40LETJettison;
	bool RTC41GNFail;
	bool RTC42GNFailInhibit;
	bool RTC51MinusVHFAntennaOn;
	bool RTC52PlusVHFAntennaOn;
	bool RTC53GNAntennaSwitching;
	bool RTC61CSMSep;
	bool RTC71Abort;

	//Relays

	//G&N
	bool R1K60;
	bool R1K61;

	//COMM
	bool R1K33;
	bool R1K34;

	MCP_SCC *scc;
};

//Spacecraft Command Controller
class MCP_SCC
{
public:
	MCP_SCC();
	void Init(Saturn *s, MCP_GCC*g, MCP_ADS *a);
	void Timestep(double simdt);
	void SaveState(FILEHANDLE scn);
	void LoadState(FILEHANDLE scn);

	void ProgramerReset();

	//Signals from GSE
	void MESCLogicBusesArm(bool set);
	void MESCLogicBusASafe(bool set);
	void MESCLogicBusBSafe(bool set);
	void MESCPyroBusesArm(bool set);
	void MESCPyroBusASafe(bool set);
	void MESCPyroBusBSafe(bool set);
	void ResetGSESignals();

	//Signals to external systems
	bool GetFireArm(bool IsSysA);
	bool GetFireSafe(bool IsSysA);
	bool GetLESMotorFire(bool IsSysA);
	bool GetCMSMSep(bool IsSysA);
	bool GetELSActive(bool IsSysA);
	bool GetRestartSignal();
	bool GetMESCLogicBusArm(bool IsSysA);
	bool GetMESCPyroBusArm(bool IsSysA);
	bool GetSeparateAbortSignal(bool IsSysA);
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
	bool GetMESCLogicBusArmA();
	bool GetMESCLogicBusArmB();
	bool GetMESCPyroBusArmA();
	bool GetMESCPyroBusArmB();
	bool GetSeparateAbortSignalA();
	bool GetSeparateAbortSignalB();

	//1
	bool GNFailSignal;
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
	//10
	bool NoAbort;
	//11
	bool b12KBaroSwitchPlus20sSignal;
	//12?
	bool LVSCSepSignal;
	//13
	bool ImpactSignal;

	bool ImpactPlus11Signal;
	bool HFOnPlus10sSignal;
	bool ImpactPlus11DiffSignal;

	bool GSEMESCLogicBusABArm;
	bool GSEMESCLogicBusASafe;
	bool GSEMESCLogicBusBSafe;
	bool GSEMESCPyroBusABArm;
	bool GSEMESCPyroBusASafe;
	bool GSEMESCPyroBusBSafe;

	//Relays (latching)

	//SECS
	bool R2K9A;
	bool R2K9B;
	bool R2K11A;
	bool R2K11B;
	bool R2K12A; //MESC Logic Bus B Arm
	bool R2K12B; //MESC Logic Bus B Arm
	bool R2K13A; //MESC Pyro Bus B Arm
	bool R2K13B; //MESC Pyro Bus B Arm
	bool R2K14A;
	bool R2K14B;
	bool R2K15A;
	bool R2K15B;
	bool R2K17A;
	bool R2K17B;
	bool R2K18A; //MESC Logic Bus A Arm
	bool R2K18B; //MESC Logic Bus A Arm
	bool R2K19A; //MESC Pyro Bus A Arm
	bool R2K19B; //MESC Pyro Bus A Arm
	bool R2K20A;
	bool R2K20B;
	bool R2K10AB;
	bool R2K16AB;
	bool R2K43A;
	bool R2K43B;
	bool R2K44A;
	bool R2K44B;
	bool R2K55ABC;
	bool R2K67ABC;
	bool R2K110;
	bool R2K111;
	bool R2K130; //Impact + 11s
	bool R2K131; //CSM Sep
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
	bool R2K173; //G&N Fail
	bool R2K222;

	//COMM
	bool R2K116;

	//ELS
	bool R2K147ABC; //Main impact
	bool R2K147DEF; //Backup impact

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
	//LV/SC + 6.0 seconds
	bool R2K181;
	//LV/SC + 2.5 seconds
	bool R2K202; //Check
	//Low LES Abort
	bool R2K133;
	//G&N Fail Inhibit
	bool R2K174;
	//LES Abort
	bool R2K125;
	//G&N Fail
	bool R2K126;
	//LV/SC Sep without abort?
	bool R2K129;
	//Baro switch
	bool R2K145_146;
	//Backup impact
	bool R2K192;
	//Main impact
	bool R2K193;
	//Stable II
	bool R2K149;
	bool R2K186;

	//Differentiators
	Differentiator CSMSepDiff;
	Differentiator LVSCSep60sDiff;
	Differentiator HFOnPlus10sDiff;
	Differentiator SepAbortDiff;
	Differentiator ImpactDiff;
	Differentiator ImpactPlus11Diff;

	//Timers
	DelayTimer LESMotorFireTimer;
	DelayTimer CMSMSepTimer;
	DelayTimer LiftoffTimer;
	DelayTimer LVSCSep25sTimer;
	DelayTimer LVSCSep30sTimer;
	DelayTimer LVSCSep60sTimer;
	DelayTimer BaroSwitchTimer;
	DelayTimer ImpactTimer;
	DelayTimer Impact11sTimer;
	DelayTimer HFPlus1sTimer;
	DelayTimer HFPlus2sTimer;

	Saturn *Sat;
	MCP_GCC *gcc;
	MCP_ADS *ads;
};

#define MCP_ADS_START_STRING		"MCP_ADS_BEGIN"
#define MCP_ADS_END_STRING			"MCP_ADS_END"

#define MCP_GCC_START_STRING		"MCP_GCC_BEGIN"
#define MCP_GCC_END_STRING			"MCP_GCC_END"

#define MCP_SCC_START_STRING		"MCP_SCC_BEGIN"
#define MCP_SCC_END_STRING			"MCP_SCC_END"