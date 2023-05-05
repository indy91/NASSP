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

	bool GetState() { return State; }
	void SetState(bool s) { State = s; }
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

	//From GSE
	void MasterControlTransfer();
	void ProgramerReset();

	bool GetLETJettison() { return RTC40LETJettison; }
	bool GetAbort() { return RTC71Abort; }
	bool GetCSMSep() { return RTC61CSMSep; }
	bool GetRTC53GNAntennaSwitching() { return RTC53GNAntennaSwitching; }

	bool GetGNFail() { return R1K60; }
	bool GetGNFailInhibit() { return R1K61; }
	bool GetSCSDirectThrustOn() { return R1K12ABCD; }
	bool GetDirectThrustOn() { return R1K78ABCD; }
	bool GetDirectThrustOff() { return R1K77AB; }
	bool GetDirectUllage() { return R1K72ABCD; }
	bool GetFDAIAlign() { return R1K71AB; }
	bool GetSPSInjectorValves12Open() { return R1K47AB; }
	bool GetSPSInjectorValves34Open() { return R1K56AB; }
	bool GetSCSDirectUllage() { return R1K31; }
	bool GetRCSPitchChannelDisable() { return R1K9AB; }
	bool GetRCSYawChannelDisable() { return R1K10AB; }
	bool GetLiftingEntry() { return R1K11AB; }
protected:

	void InputReset();

	Saturn *Sat;

	//RTCs
	bool RTC24SMRCSAOff;
	bool RTC25SMRCSBOff;
	bool RTC26SMRCSCOff;
	bool RTC27SMRCSDOff;
	bool RTC32SMRCSAOn;
	bool RTC33SMRCSBOn;
	bool RTC34SMRCSCOn;
	bool RTC35SMRCSDOn;
	bool RTC40LETJettison;
	bool RTC41GNFail;
	bool RTC42GNFailInhibit;
	bool RTC51MinusVHFAntennaOn;
	bool RTC52PlusVHFAntennaOn;
	bool RTC53GNAntennaSwitching;
	bool RTC61CSMSep;
	bool RTC62SBandReceiverOn;
	bool RTC63UHFReceiverOn;
	bool RTC71Abort;

	//Relays

	bool R1K1ABCD;	// FC 1 Purge
	bool R1K2ABCD;	// FC 2 Purge
	bool R1K3ABCD;	// FC 3 Purge
	bool R1K4AB;	// Roll Rate Backup
	bool R1K5AB;	// Pitch Rate Backup
	bool R1K6AB;	// Yaw Rate Backup
	bool R1K7AB;	// Roll A&C Channel Disable
	bool R1K8AB;	// Roll B&D Channel Disable
	bool R1K9AB;	// Pitch Channel Disable
	bool R1K10AB;	// Yaw Channel Disable
	bool R1K11AB;	// Lift entry
	bool R1K12ABCD;	// Direct Thrust On
	bool R1K13AB;	// +Pitch Direct Rotation
	bool R1K14AB;	// -Pitch Direct Rotation
	bool R1K15AB;	// +Yaw Direct Rotation
	bool R1K16AB;	// -Yaw Direct Rotation
	bool R1K17AB;	// +Roll Direct Rotation
	bool R1K18AB;	// -Roll Direct Rotation
	bool R1K31;		// Direct Ullage
	bool R1K33;
	bool R1K34;
	bool R1K37ABC;	// Receiver
	bool R1K47AB;
	bool R1K50;		// Batteries off at impact + 11 seconds
	bool R1K56AB;

	bool R1K60;
	bool R1K61;
	bool R1K62;		// H2 Tank No. 2 Heater and Fans
	bool R1K63;		// O2 Tank No. 2 Heater and Fans
	bool R1K64;		// H2 Tank No. 1 Heater and Fans
	bool R1K65;		// O2 Tank No. 1 Heater and Fans
	bool R1K71AB;	// FDAI Align
	bool R1K72ABCD;	// Direct Ullage
	bool R1K77AB;	// Direct Thrust Off
	bool R1K78ABCD;	// Direct Thrust On

	bool R1K100;	// Master Control Transfer

	//Differentiators
	Differentiator SIVBRestartDiff;	

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
	void MasterControlTransfer();
	void ResetGSESignals();

	//Signals to external systems
	bool GetRCSDump(bool IsSysA);	// K1
	bool GetRCSPurge(bool IsSysA);	// K2
	bool GetOxidDump(bool IsSysA);	// K3
	bool GetFireArm(bool IsSysA);
	bool GetFireSafe(bool IsSysA);
	bool GetLESMotorFire(bool IsSysA);
	bool GetCMSMSep(bool IsSysA);
	bool GetELSActive(bool IsSysA);
	bool GetRestartSignal();
	bool GetMESCLogicBusArm(bool IsSysA);
	bool GetMESCPyroBusArm(bool IsSysA);
	bool GetSeparateAbortSignal(bool IsSysA);
	bool GetGimbalStart(bool yaw, int num);
	bool GetGimbalOn(bool yaw, int num);
	bool GetGimbalOff(bool yaw, int num);
	bool GetGNAttitudeControl() { return R2K32; }
	bool GetGNEntryMode() { return R2K34; }
	bool GetGNDVMode() { return R2K36; }
	bool GetMonitorMode() { return R2K31; }
	bool GetSCSDVMode() { return R2K57; }
	bool GetSCSEntryMode() { return false; }
	bool GetSPSPrePilotValveAOpen() { return R2K58; }
	bool GetSPSPrePilotValveBOpen() { return R2K59; }
	bool GetSPSArmSignal() { return SPSArmSignal; }
	bool Get005gSignal() { return R2K29; }
	bool GetXTranslation() { return R2K42; }
	bool GetDeadbandSelect() { return IsPowered() && R2K100 && !R2K69; }
	bool GetEntryBattsOnMainSignal() { return EntryBattsOnMainSignal; }
	bool GetEntryBattsOffMainSignal() { return EntryBattsOffMainSignal; }
	bool GetImpactPlus11SecondsSignal() { return ImpactPlus11Signal; }
protected:

	bool IsPowered();
	void DeterminePowerState();

	bool bPower;

	bool GetRCSDumpA();
	bool GetRCSDumpB();
	bool GetRCSPurgeA();
	bool GetRCSPurgeB();
	bool GetOxidDumpA();
	bool GetOxidDumpB();
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
	//2
	bool GimbalMotorsOn;
	//4
	bool SPSEngineHold;
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
	//12
	bool CountdownResetSignal;

	bool ImpactPlus11Signal;
	bool HFOnPlus10sSignal;
	bool ImpactPlus11DiffSignal;
	bool SPSArmSignal;
	bool EntryBattsOnMainSignal;
	bool EntryBattsOffMainSignal;

	bool GSEMESCLogicBusABArm;
	bool GSEMESCLogicBusASafe;
	bool GSEMESCLogicBusBSafe;
	bool GSEMESCPyroBusABArm;
	bool GSEMESCPyroBusASafe;
	bool GSEMESCPyroBusBSafe;
	bool GSEECSStart;

	//Relays

	bool R2K1;		// RCS Dump A
	bool R2K2;		// RCS Purge Activate
	bool R2K3;		// Oxid Dump A
	bool R2K6;		// Glycol Wetness Control
	bool R2K7;		// O2 Isolation Valve Open
	bool R2K8;		// Glycol Shutoff Valve Open
	bool R2K9;		// Escape Tower Jettison Fire Arm A
	bool R2K10;		// LES Motor Fire B
	bool R2K11;		// CSM Sep B
	bool R2K12;		// MESC Logic Bus B Arm
	bool R2K13;		// MESC Pyro Bus B Arm
	bool R2K14;		// ELS Activate B 
	bool R2K15;		// Escape Tower Jettison Fire Arm B
	bool R2K16;		// LES Motor Fire A
	bool R2K17;		// CSM Sep A
	bool R2K18;		// MESC Logic Bus A Arm
	bool R2K19;		// MESC Pyro Bus A Arm
	bool R2K20;		// ELS Activate A
	bool R2K21;		// Yaw 1 Start
	bool R2K22;		// Yaw 1 On
	bool R2K23;		// Pitch 1 ON
	bool R2K24;		// Pitch 1 Start
	bool R2K25;		// Yaw 2 Start On
	bool R2K26;		// Yaw 2 On
	bool R2K27;		// Pitch 2 Start
	bool R2K28;		// Pitch 2 On
	bool R2K29;		// 0.05g
	bool R2K30;		// Lift Entry
	bool R2K31;		// Monitor Mode
	bool R2K32;		// G&N Attitude Control
	bool R2K34;		// G&N Entry Mode
	bool R2K35;		// SCS Entry Mode
	bool R2K36;		// G&N DV Mode
	bool R2K37;		// Roll Rate On
	bool R2K39;		// Gimbal Position Set
	bool R2K40;		// Gimbal Position Set
	bool R2K42;		// X-Translation
	bool R2K43;		// Sep/Abort A Off
	bool R2K44;		// Sep/Abort B Off
	bool R2K53;		// Gimbal Position Set
	bool R2K55;		// Main Chute Disconnect A
	bool R2K56;		// Backpressure Control
	bool R2K57;		// SCS DV Mode
	bool R2K58;		// SPS Pre-Pilot Valve A
	bool R2K59;		// SPS Pre-Pilot Valve B
	bool R2K63;		// Vent Bags On, Uprighting Control
	bool R2K66;		// Pumps Off, Uprighting Control
	bool R2K67;		// Main Chute Disconnect B
	bool R2K68;		// Pseudo Rate Out
	bool R2K69;		// Deadband Select
	bool R2K70;		// RCS Dump B
	bool R2K71;		// RCS Purge Activate
	bool R2K72;		// Oxid Dump B
	bool R2K100;	// Master Control
	bool R2K110;	// Stable II
	bool R2K111;	// Stable II plus 11 seconds?
	bool R2K116;	// VHF Antenna
	bool R2K117;	// Lift Entry
	bool R2K119;	// Direct Thrust Off
	bool R2K125;	// LES Abort
	bool R2K126;	// G&N Fail
	bool R2K129;	// LV/SC Sep without abort?
	bool R2K130;	// Impact + 11s
	bool R2K131;	// CSM Sep
	bool R2K132;	// Bar switch plus 20 seconds
	bool R2K133;	// Low LES Abort, pretty sure it's R2K133AB
	bool R2K134;	// Liftoff
	bool R2K135;	// LET Jettison
	bool R2K136;	// CSM Sep
	bool R2K137;	// LV/SC Sep
	bool R2K138;	// CSM Sep
	bool R2K139;	// CSM Sep
	bool R2K140;	// Abort Inhibit?
	bool R2K141;	// Liftoff +42 seconds
	bool R2K142;	// Low LES Abort
	bool R2K145;	// Baro switch
	bool R2K146;	// Baro switch
	bool R2K147;	// Impact
	bool R2K149;	// Stable II
	bool R2K153;	// Gimbal Motors Off
	bool R2K156;	// LET Jet
	bool R2K173;	// G&N Fail
	bool R2K174;	// G&N Fail Inhibit
	bool R2K177;	// Liftoff plus 42 seconds
	bool R2K181;	// LV/SC + 6.0 seconds
	bool R2K186;	// Stable II
	bool R2K187;	// LET Jettison
	bool R2K188;	// CM/SM Sep
	bool R2K192;	// Backup impact
	bool R2K193;	// Main impact
	bool R2K202;	// LV/SC + 2.5 seconds, Check
	bool R2K222;	// S-IVB Restart	

	//Differentiators
	Differentiator CSMSepDiff;
	Differentiator LVSCSep60sDiff;
	Differentiator HFOnPlus10sDiff;
	Differentiator SepAbortDiff;
	Differentiator ImpactDiff;
	Differentiator ImpactPlus11Diff;
	Differentiator GimbalMotorsDiff;
	Differentiator CountdownResetDiff;
	Differentiator StableIIPlus1MinDiff;
	Differentiator EntryBattsOnMainDiff;
	Differentiator EntryBattsOffMainDiff;

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
	DelayTimer RCSPurge80sTimer;
	DelayTimer RCSPurge250sTimer;
	DelayTimer GimbalMotors30sTimer;
	DelayTimer GimbalMotors80sTimer;
	DelayTimer GimbalMotorYaw1StartTimer;
	DelayTimer GimbalMotorYaw1OnTimer;
	DelayTimer GimbalMotorYaw2StartTimer;
	DelayTimer GimbalMotorYaw2OnPitch2StartTimer;
	DelayTimer GimbalMotorPitch2OnTimer;
	DelayTimer StableIIPlus1MinTimer;
	DelayTimer FillBagsTimer1; //These three left to right in the schematic 11.3.3 AS-501 Systems Handbook
	DelayTimer FillBagsTimer2;
	DelayTimer FillBagsTimer3;

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