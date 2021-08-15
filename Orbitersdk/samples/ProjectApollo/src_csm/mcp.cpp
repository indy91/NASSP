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

#include "Orbitersdk.h"
#include "soundlib.h"
#include "saturn.h"
#include "mcp.h"
#include "papi.h"

Differentiator::Differentiator()
{
	State = false;
}

bool Differentiator::EvaluateState(bool in)
{
	//bool ReturnVal = (State != in);
	bool ReturnVal = (State == false && in == true);
	State = in;

	return ReturnVal;
}

MCP_ADS::MCP_ADS()
{
	ImpactSwitch = false;
	b005GSwitch = false;
	AttitudeSwitch = false;
}

void MCP_ADS::Init(VESSEL *v)
{
	Accelerometer.Init(v);
}

void MCP_ADS::Timestep(double simdt)
{
	Accelerometer.Timestep(simdt);

	if (Accelerometer.GetXAccel() > 0.05*9.80665)
	{
		b005GSwitch = true;
	}
	else
	{
		b005GSwitch = false;
	}
	if (Accelerometer.GetXAccel() > 5.0*9.80665)
	{
		ImpactSwitch = true;
	}
	else
	{
		ImpactSwitch = false;
	}
	if (Accelerometer.GetXAccel() < 0.0)
	{
		AttitudeSwitch = true;
	}
	else
	{
		AttitudeSwitch = false;
	}
}

void MCP_ADS::SaveState(FILEHANDLE scn)
{
	oapiWriteLine(scn, MCP_ADS_START_STRING);
	
	papiWriteScenario_bool(scn, "b005GSwitch", b005GSwitch);
	papiWriteScenario_bool(scn, "ImpactSwitch", ImpactSwitch);
	papiWriteScenario_bool(scn, "AttitudeSwitch", AttitudeSwitch);

	Accelerometer.SaveState(scn);

	oapiWriteLine(scn, MCP_ADS_END_STRING);
}

void MCP_ADS::LoadState(FILEHANDLE scn)
{
	char *line;

	while (oapiReadScenario_nextline(scn, line)) {
		if (!strnicmp(line, MCP_ADS_END_STRING, sizeof(MCP_ADS_END_STRING)))
			break;

		papiReadScenario_bool(line, "b005GSwitch", b005GSwitch);
		papiReadScenario_bool(line, "ImpactSwitch", ImpactSwitch);
		papiReadScenario_bool(line, "AttitudeSwitch", AttitudeSwitch);

		if (!strnicmp(line, MECHACCEL_START_STRING, sizeof(MECHACCEL_START_STRING))) {
			Accelerometer.LoadState(scn);
		}
	}
}

MCP_GCC::MCP_GCC()
{
	R1K1ABCD = false;
	R1K2ABCD = false;
	R1K3ABCD = false;
	R1K4AB = false;
	R1K5AB = false;
	R1K6AB = false;
	R1K7AB = false;
	R1K8AB = false;
	R1K9AB = false;
	R1K10AB = false;
	R1K11AB = false;
	R1K12ABCD = false;
	R1K13AB = false;
	R1K14AB = false;
	R1K15AB = false;
	R1K16AB = false;
	R1K17AB = false;
	R1K18AB = false;
	R1K31 = false;
	R1K33 = false;
	R1K34 = false;
	R1K37ABC = false;
	R1K47AB = false;
	R1K50 = false;
	R1K56AB = false;
	R1K62 = false;
	R1K63 = false;
	R1K64 = false;
	R1K65 = false;
	R1K71AB = false;
	R1K100 = false;
	R1K78ABCD = false;
	R1K77AB = false;

	InputReset();
}

void MCP_GCC::Init(Saturn *s, MCP_SCC* sc)
{
	Sat = s;
	scc = sc;
}

void MCP_GCC::InputReset()
{
	RTC24SMRCSAOff = false;
	RTC25SMRCSBOff = false;
	RTC26SMRCSCOff = false;
	RTC27SMRCSDOff = false;
	RTC32SMRCSAOn = false;
	RTC33SMRCSBOn = false;
	RTC34SMRCSCOn = false;
	RTC35SMRCSDOn = false;
	RTC40LETJettison = false;
	RTC41GNFail = false;
	RTC42GNFailInhibit = false;
	RTC51MinusVHFAntennaOn = false;
	RTC52PlusVHFAntennaOn = false;
	RTC61CSMSep = false;
	RTC62SBandReceiverOn = false;
	RTC63UHFReceiverOn = false;
	RTC71Abort = false;
}

void MCP_GCC::Timestep(double simdt)
{
	//G&N
	if (RTC41GNFail)
	{
		R1K60 = true;
	}
	if (RTC42GNFailInhibit)
	{
		R1K61 = true;
	}
	R1K31 = R1K72ABCD;

	//Comm
	if (RTC52PlusVHFAntennaOn) //TBD: Signal from SCC
	{
		R1K33 = true;
		R1K34 = false;
	}
	if (RTC51MinusVHFAntennaOn) //TBD: Signal from SCC
	{
		R1K33 = false;
		R1K34 = true;
	}
	bool SIVBRestartDiffSignal = SIVBRestartDiff.EvaluateState(scc->GetRestartSignal());
	if (RTC62SBandReceiverOn || SIVBRestartDiffSignal)
	{
		R1K37ABC = true;
	}
	else if (RTC63UHFReceiverOn)
	{
		R1K37ABC = false;
	}

	//PROP
	R1K12ABCD = R1K78ABCD;

	if (scc->GetSPSArmSignal())
	{
		R1K47AB = true;
		R1K56AB = true;
	}
	else if (R1K77AB)
	{
		R1K47AB = false;
		R1K56AB = false;
	}

	if (scc->GetImpactPlus11SecondsSignal())
	{
		R1K50 = true;
	}
	if (scc->GetEntryBattsOnMainSignal())
	{
		Sat->MainBusAController.SetTieState(2);
		Sat->MainBusBController.SetTieState(2);
	}
	else if (scc->GetEntryBattsOffMainSignal() || R1K50)
	{
		Sat->MainBusAController.SetTieState(0);
		Sat->MainBusBController.SetTieState(0);
	}

	//UDL input duration is 25-35 milliseconds, so just reset it all on the next timestep. SCC has one timestep to recognize signal
	InputReset();
}

void MCP_GCC::ProgramerReset()
{
	R1K33 = false;
	R1K34 = false;
	R1K37ABC = false;
	R1K100 = false;
	R1K47AB = false;
	R1K50 = false;
	R1K56AB = false;
}

void MCP_GCC::MasterControlTransfer()
{
	R1K100 = true;
}

void MCP_GCC::RealTimeCommand(int cmd)
{
	switch (cmd)
	{
	case 02: //FC 1 Purge
		R1K1ABCD = true;
		break;
	case 03: //FC 2 Purge
		R1K2ABCD = true;
		break;
	case 04: //FC 2 Purge
		R1K3ABCD = true;
		break;
	case 05: // Reset RTC 2, 3 and 4
		R1K1ABCD = false;
		R1K2ABCD = false;
		R1K3ABCD = false;
		break;
	case 010: //Lift Entry
		R1K11AB = true;
		break;
	case 011: //Direct Thrust On
		R1K78ABCD = true;
		break;
	case 012: //Direct Thrust Off
		R1K77AB = false;
		break;
	case 013: //Reset RTC 10, 11, 12
		R1K78ABCD = false;
		R1K77AB = false;
		R1K11AB = false;
		break;
	case 014: // +Pitch Direct Rotation
		R1K13AB = true;
		break;
	case 015: // -Pitch Direct Rotation
		R1K14AB = true;
		break;
	case 016: // +Yaw Direct Rotation
		R1K15AB = true;
		break;
	case 017: // -Yaw Direct Rotation
		R1K16AB = true;
		break;
	case 020: // +Roll Direct Rotation
		R1K17AB = true;
		break;
	case 021: // -Roll Direct Rotation
		R1K18AB = true;
		break;
	case 022: //Direct Ullage
		R1K72ABCD = true;
		break;
	case 023: //Reset 14, 15, 16, 17, 20, 21, 22
		R1K72ABCD = false;
		R1K18AB = false;
		R1K17AB = false;
		R1K16AB = false;
		R1K15AB = false;
		R1K14AB = false;
		R1K13AB = false;
		break;
	case 024: //Propellant OFF SM quad A
		RTC24SMRCSAOff = true;
		break;
	case 025: //Propellant OFF SM quad B
		RTC25SMRCSBOff = true;
		break;
	case 026: //Propellant OFF SM quad C
		RTC26SMRCSCOff = true;
		break;
	case 027: //Propellant OFF SM quad D
		RTC27SMRCSDOff = true;
		break;
	case 032: //Propellant ON SM quad A
		RTC32SMRCSAOn = true;
		break;
	case 033: //Propellant ON SM quad B
		RTC33SMRCSBOn = true;
		break;
	case 034: //Propellant ON SM quad C
		RTC34SMRCSCOn = true;
		break;
	case 035: //Propellant ON SM quad D
		RTC35SMRCSDOn = true;
		break;
	case 040: //Launch escape tower jettison
		RTC40LETJettison = true;
		break;
	case 041: //G&N Fail
		RTC41GNFail = true;
		break;
	case 042: //G&N Fail Inhibit
		RTC42GNFailInhibit = true;
		break;
	case 043: //Reset RTC 41, 42
		R1K60 = false;
		R1K61 = false;
		break;
	case 044: //Roll rate backup
		R1K4AB = true;
		break;
	case 045: //Pitch rate backup
		R1K5AB = true;
		break;
	case 046: //Yaw rate backup
		R1K6AB = true;
		break;
	case 047: //FDAI Align
		R1K71AB = true;
		break;
	case 050: //Reset RTC 44-47
		R1K71AB = false;
		R1K4AB = false;
		R1K5AB = false;
		R1K6AB = false;
		break;
	case 051: //Negative-Z antenna ON (VHF scimitar only)
		RTC51MinusVHFAntennaOn = true;
		break;
	case 052: //Positive-Z antenna ON (VHF scimitar only)
		RTC52PlusVHFAntennaOn = true;
		break;
	case 053:
		RTC53GNAntennaSwitching = true;
		break;
	case 054: //Roll A and C channel disable
		R1K7AB = true;
		break;
	case 055: //Roll B and D channel disable
		R1K8AB = true;
		break;
	case 056: //Pitch channel disable
		R1K9AB = true;
		break;
	case 057: //Yaw channel disable
		R1K10AB = true;
		break;
	case 060: //Reset RTC 54-57
		R1K7AB = false;
		R1K8AB = false;
		R1K9AB = false;
		R1K10AB = false;
		break;
	case 061: //CM/SM Separation
		RTC61CSMSep = true;
		break;
	case 062: //Updata link S-band receiver select
		RTC62SBandReceiverOn = true;
		break;
	case 063: //Updata link UHF receiver select
		RTC63UHFReceiverOn = true;
		break;
	case 064: //H2 Tank No. 2 Heater and Fans
		R1K62 = true;
		break;
	case 065: //O2 Tank No. 2 Heater and Fans
		R1K63 = true;
		break;
	case 066: //H2 Tank No. 1 Heater and Fans
		R1K64 = true;
		break;
	case 067: //O2 Tank No. 1 Heater and Fans
		R1K65 = true;
		break;
	case 070: //Reset RTC 64-67
		R1K62 = false;
		R1K63 = false;
		R1K64 = false;
		R1K65 = false;
		break;
	case 071: //Launch escape tower abort and MCP separation
		RTC71Abort = true;
		break;
	case 074: //C-band OFF
		break;
	case 075: //C-band ON (2 pulse)
		break;
	case 076: //VHF transmitter OFF
		break;
	case 077: //VHF transmitter ON
		break;
	}
}

void MCP_GCC::SaveState(FILEHANDLE scn)
{
	oapiWriteLine(scn, MCP_GCC_START_STRING);

	bool arr[15];

	arr[0] = R1K1ABCD; arr[1] = R1K2ABCD; arr[2] = R1K3ABCD; arr[3] = R1K4AB; arr[4] = R1K5AB; arr[5] = R1K6AB; arr[6] = R1K7AB; arr[7] = R1K8AB;
	arr[8] = R1K9AB; arr[9] = R1K10AB; arr[10] = R1K11AB; arr[11] = R1K12ABCD; arr[12] = R1K13AB; arr[13] = R1K14AB; arr[14] = R1K15AB;
	papiWriteScenario_boolarr(scn, "RELAYS1", arr, 15);

	arr[0] = R1K16AB; arr[1] = R1K17AB; arr[2] = R1K18AB; arr[3] = false; arr[4] = false; arr[5] = false; arr[6] = false; arr[7] = false;
	arr[8] = false; arr[9] = false; arr[10] = false; arr[11] = false; arr[12] = false; arr[13] = false; arr[14] = false;
	papiWriteScenario_boolarr(scn, "RELAYS2", arr, 15);

	arr[0] = R1K31; arr[1] = false; arr[2] = R1K33; arr[3] = R1K34; arr[4] = false; arr[5] = false; arr[6] = R1K37ABC; arr[7] = false;
	arr[8] = false; arr[9] = false; arr[10] = false; arr[11] = false; arr[12] = false; arr[13] = false; arr[14] = false;
	papiWriteScenario_boolarr(scn, "RELAYS3", arr, 15);

	arr[0] = false; arr[1] = R1K47AB; arr[2] = false; arr[3] = false; arr[4] = R1K50; arr[5] = false; arr[6] = false; arr[7] = false;
	arr[8] = false; arr[9] = false; arr[10] = R1K56AB; arr[11] = false; arr[12] = false; arr[13] = false; arr[14] = R1K60;
	papiWriteScenario_boolarr(scn, "RELAYS4", arr, 15);

	arr[0] = R1K61; arr[1] = R1K62; arr[2] = R1K63; arr[3] = R1K64; arr[4] = R1K65; arr[5] = false; arr[6] = false; arr[7] = false;
	arr[8] = false; arr[9] = false; arr[10] = R1K71AB; arr[11] = R1K72ABCD; arr[12] = false; arr[13] = false; arr[14] = false;
	papiWriteScenario_boolarr(scn, "RELAYS5", arr, 15);

	arr[0] = false; arr[1] = R1K77AB; arr[2] = R1K78ABCD; arr[3] = false; arr[4] = false; arr[5] = false; arr[6] = false; arr[7] = false;
	arr[8] = false; arr[9] = false; arr[10] = false; arr[11] = false; arr[12] = false; arr[13] = false; arr[14] = R1K100;
	papiWriteScenario_boolarr(scn, "RELAYS6", arr, 15);

	arr[0] = SIVBRestartDiff.GetState();
	papiWriteScenario_boolarr(scn, "DIFFERENTIATORS", arr, 1);

	oapiWriteLine(scn, MCP_GCC_END_STRING);
}

void MCP_GCC::LoadState(FILEHANDLE scn)
{
	char *line;
	bool arr[15];

	while (oapiReadScenario_nextline(scn, line)) {
		if (!strnicmp(line, MCP_GCC_END_STRING, sizeof(MCP_GCC_END_STRING)))
			break;

		if (papiReadScenario_boolarr(line, "RELAYS1", arr, 15))
		{
			R1K1ABCD = arr[0]; R1K2ABCD = arr[1]; R1K3ABCD = arr[2]; R1K4AB = arr[3]; R1K5AB = arr[4]; R1K6AB = arr[5]; R1K7AB = arr[6]; R1K8AB = arr[7];
			R1K9AB = arr[8]; R1K10AB = arr[9]; R1K11AB = arr[10]; R1K12ABCD = arr[11]; R1K13AB = arr[12]; R1K14AB = arr[13]; R1K15AB = arr[14];
		}
		else if (papiReadScenario_boolarr(line, "RELAYS2", arr, 15))
		{
			R1K16AB = arr[0]; R1K17AB = arr[1]; R1K18AB = arr[2];
		}
		else if (papiReadScenario_boolarr(line, "RELAYS3", arr, 15))
		{
			R1K31 = arr[0]; R1K33 = arr[2]; R1K34 = arr[3]; R1K37ABC = arr[6];
		}
		else if (papiReadScenario_boolarr(line, "RELAYS4", arr, 15))
		{
			R1K47AB = arr[1]; R1K50 = arr[4]; R1K56AB = arr[10]; R1K60 = arr[14];
		}
		else if (papiReadScenario_boolarr(line, "RELAYS5", arr, 15))
		{
			R1K61 = arr[0]; R1K62 = arr[1]; R1K63 = arr[2]; R1K64 = arr[3]; R1K65 = arr[4]; R1K71AB = arr[10]; R1K72ABCD = arr[11];
		}
		else if (papiReadScenario_boolarr(line, "RELAYS6", arr, 15))
		{
			R1K77AB = arr[1]; R1K78ABCD = arr[2]; R1K100 = arr[14];
		}
		else if (papiReadScenario_boolarr(line, "DIFFERENTIATORS", arr, 1))
		{
			SIVBRestartDiff.SetState(arr[0]);
		}
	}
}

MCP_SCC::MCP_SCC() :
	LESMotorFireTimer(0.5),
	CMSMSepTimer(1.0),
	LiftoffTimer(42.0),
	LVSCSep25sTimer(2.5),
	LVSCSep60sTimer(6.0),
	LVSCSep30sTimer(0.5), //Gets input from 2.5s timer, so duration is correct
	BaroSwitchTimer(20.0),
	ImpactTimer(14.0*60.0),
	Impact11sTimer(11.0),
	HFPlus1sTimer(1.0),
	HFPlus2sTimer(1.0),
	RCSPurge80sTimer(80.0),
	RCSPurge250sTimer(250.0),
	GimbalMotors30sTimer(3.0),
	GimbalMotors80sTimer(8.0),
	GimbalMotorYaw1StartTimer(1.0),
	GimbalMotorYaw1OnTimer(1.5),
	GimbalMotorYaw2StartTimer(2.0),
	GimbalMotorYaw2OnPitch2StartTimer(2.5),
	GimbalMotorPitch2OnTimer(3.0),
	StableIIPlus1MinTimer(60.0),
	FillBagsTimer1(300.0),
	FillBagsTimer2(300.0),
	FillBagsTimer3(300.0)
{
	bPower = false;

	R2K1 = false;
	R2K2 = false;
	R2K3 = false;
	R2K6 = false;
	R2K7 = false;
	R2K8 = false;
	R2K9 = false;
	R2K10 = false;
	R2K11 = false;
	R2K12 = false;
	R2K13 = false;
	R2K14 = false;
	R2K15 = false;
	R2K16 = false;
	R2K17 = false;
	R2K18 = false;
	R2K19 = false;
	R2K20 = false;
	R2K21 = false;
	R2K22 = false;
	R2K23 = false;
	R2K24 = false;
	R2K25 = false;
	R2K26 = false;
	R2K27 = false;
	R2K28 = false;
	R2K29 = false;
	R2K30 = false;
	R2K31 = false;
	R2K32 = false;
	R2K34 = false;
	R2K35 = false;
	R2K36 = false;
	R2K42 = false;
	R2K43 = false;
	R2K44 = false;
	R2K55 = false;
	R2K56 = false;
	R2K57 = false;
	R2K58 = false;
	R2K59 = false;
	R2K63 = false;
	R2K66 = false;
	R2K67 = false;
	R2K68 = false;
	R2K69 = false;
	R2K70 = false;
	R2K71 = false;
	R2K72 = false;
	R2K132 = false;
	R2K134 = false;
	R2K135 = false;
	R2K136 = false;
	R2K137 = false;
	R2K138 = false;
	R2K139 = false;
	R2K140 = false;
	R2K147 = false;
	R2K156 = false;
	R2K141 = false;
	R2K177 = false;
	R2K188 = false;
	R2K222 = false;	
	R2K202 = false;
	R2K133 = false;
	R2K142 = false;
	R2K131 = false;
	R2K116 = false;
	R2K181 = false;
	
	R2K173 = false;
	R2K174 = false;
	R2K125 = false;
	R2K126 = false;
	R2K129 = false;
	R2K192 = false;
	R2K193 = false;

	R2K130 = false;
	R2K149 = false;
	R2K110 = true; //??
	R2K111 = true; //??
	R2K186 = false;
	R2K145 = false;
	R2K146 = false;
	
	R2K153 = false;
	R2K119 = false;

	GNFailSignal = false;
	GimbalMotorsOn = false;
	SPSEngineHold = false;
	LVSCSep25sSignal = false;
	LESAbortSignal = false;
	CSMSepSignal = false;
	LETJettisonSignal = false;
	b005GSignal = false;
	NoAbort = false;
	b12KBaroSwitchPlus20sSignal = false;
	LVSCSepSignal = false;
	ImpactSignal = false;
	CountdownResetSignal = false;
	ImpactPlus11Signal = false;
	HFOnPlus10sSignal = false;
	ImpactPlus11DiffSignal = false;
	SPSArmSignal = false;
	EntryBattsOnMainSignal = false;
	EntryBattsOffMainSignal = false;

	ResetGSESignals();
}

void MCP_SCC::Init(Saturn *s, MCP_GCC *g, MCP_ADS *a)
{
	Sat = s;
	gcc = g;
	ads = a;
}

void MCP_SCC::Timestep(double simdt)
{
	LESMotorFireTimer.Timestep(simdt);
	CMSMSepTimer.Timestep(simdt);
	LiftoffTimer.Timestep(simdt);
	LVSCSep25sTimer.Timestep(simdt);
	LVSCSep30sTimer.Timestep(simdt);
	LVSCSep60sTimer.Timestep(simdt);
	BaroSwitchTimer.Timestep(simdt);
	ImpactTimer.Timestep(simdt);
	Impact11sTimer.Timestep(simdt);
	HFPlus1sTimer.Timestep(simdt);
	HFPlus2sTimer.Timestep(simdt);
	RCSPurge80sTimer.Timestep(simdt);
	RCSPurge250sTimer.Timestep(simdt);
	GimbalMotors30sTimer.Timestep(simdt);
	GimbalMotors80sTimer.Timestep(simdt);
	GimbalMotorYaw1StartTimer.Timestep(simdt);
	GimbalMotorYaw1OnTimer.Timestep(simdt);
	GimbalMotorYaw2StartTimer.Timestep(simdt);
	GimbalMotorYaw2OnPitch2StartTimer.Timestep(simdt);
	GimbalMotorPitch2OnTimer.Timestep(simdt);
	StableIIPlus1MinTimer.Timestep(simdt);
	FillBagsTimer1.Timestep(simdt);
	FillBagsTimer2.Timestep(simdt);
	FillBagsTimer3.Timestep(simdt);

	DeterminePowerState();

	//Input signals
	//IU
	//Tower Jettison
	if (!R2K135 && (gcc->GetLETJettison() || Sat->iuCommandConnector.GetTowerJettisonCmdA() || Sat->iuCommandConnector.GetTowerJettisonCmdB()))
	{
		R2K135 = true;
	}
	LETJettisonSignal = R2K135;

	//Restart
	if (!R2K222 && Sat->GetSIISepState())
	{
		R2K222 = true;
	}

	//Liftoff
	if (!R2K134 && (Sat->iuCommandConnector.GetLiftOffCircuit(true) || Sat->iuCommandConnector.GetLiftOffCircuit(false)))
	{
		R2K134 = true;
	}
	bool LiftoffSignal = R2K134;

	//CSM Sep
	if (!R2K131 && (gcc->GetCSMSep() || Sat->dsky.GetCRelay(25)))
	{
		R2K131 = true;
	}

	//Internal processing
	//G&N
	if (IsPowered() && Sat->dsky.GetCRelay(27))
	{
		R2K173 = true;
	}
	R2K174 = gcc->GetGNFailInhibit();
	GNFailSignal = (gcc->GetGNFail() || (R2K173 && !R2K174));

	CSMSepSignal = IsPowered() && R2K131;
	bool R2K113ABC = CSMSepSignal;
	bool R2K114ABC = R2K113ABC;

	R2K125 = LESAbortSignal;
	R2K126 = GNFailSignal;

	NoAbort = (IsPowered() && !R2K125 && !R2K126);
	R2K129 = (LVSCSep25sSignal && NoAbort);
	bool GNModesAllowed = R2K129;

	//X-Translation
	R2K42 = GNModesAllowed && Sat->dsky.GetCRelay(26);
	//G&N Attitude Control Mode
	R2K32 = GNModesAllowed && Sat->dsky.GetCRelay(22);
	//G&N Entry Mode
	R2K34 = GNModesAllowed && Sat->dsky.GetCRelay(24);
	//G&N DV Mode
	R2K36 = GNModesAllowed && Sat->dsky.GetCRelay(23);

	bool LVSepAndGNFail = GNFailSignal && LVSCSepSignal;

	//SCS DV Mode
	R2K57 = LVSepAndGNFail && !R2K114ABC;
	//SCS Entry Mode
	R2K35 = LVSepAndGNFail && R2K113ABC;
	//Monitor Mode
	R2K31 = R2K57 || R2K35 || R2K32 || R2K36 || R2K34;

	//0.05g
	b005GSignal = (ads->Get005GSwitch() || (CSMSepSignal && R2K129 && Sat->dsky.GetCRelay(28)));
	R2K29 = b005GSignal;

	//sprintf(oapiDebugString(), "0.05g %d ADS %d CSMSep %d R2K129 %d AGC %d", b005GSignal, ads->Get005GSwitch(), CSMSepSignal, R2K129, Sat->dsky.GetCRelay(28));

	//FDAI Align
	bool R2K38AB = NoAbort && (gcc->GetFDAIAlign() || Sat->dsky.GetCRelay(30));

	if (IsPowered() && LiftoffSignal)
		LiftoffTimer.SetRunning(true);

	if (LiftoffTimer.ContactClosed())
		R2K177 = true;
	else
		R2K177 = false;

	bool LiftoffPlus42sSignal = R2K141 = IsPowered() && R2K177;
	if (LiftoffPlus42sSignal && !R2K133)
	{
		R2K3 = true;
		R2K72 = true;
	}

	if (LETJettisonSignal)
	{
		R2K9 = true;
		R2K15 = true;
		LESMotorFireTimer.SetRunning(true);
	}

	if (LESMotorFireTimer.ContactClosed())
	{
		R2K187 = true;
	}
	else
	{
		R2K187 = false;
	}

	if (IsPowered() && R2K187)
	{
		R2K10 = true;
		R2K16 = true;
	}

	//LES Abort
	R2K140 = LETJettisonSignal;
	LESAbortSignal = gcc->GetAbort() && !R2K140;
	if (!R2K142 && LESAbortSignal && !R2K141)
	{
		R2K142 = true;
	}

	R2K132 = b12KBaroSwitchPlus20sSignal;
	if (b12KBaroSwitchPlus20sSignal)
	{
		RCSPurge250sTimer.SetRunning(true);
	}
	bool LowLESAbortSignal = R2K133 = IsPowered() && R2K142;

	if (LowLESAbortSignal && !R2K132)
	{
		RCSPurge80sTimer.SetRunning(true);
	}

	if (b12KBaroSwitchPlus20sSignal)
	{
		R2K1 = true;
		R2K70 = true;
	}
	if (RCSPurge250sTimer.ContactClosed())
	{
		R2K2 = true;
	}
	if (RCSPurge80sTimer.ContactClosed())
	{
		R2K71 = true;
	}

	if (CSMSepSignal || LESAbortSignal)
	{
		R2K138 = true;
		R2K136 = true;
		R2K139 = true;
	}
	else
	{
		R2K138 = false;
		R2K136 = false;
		R2K139 = false;
	}

	//LV/SC Sep
	R2K156 = LETJettisonSignal;
	if (!R2K137 && ((R2K156 && gcc->GetAbort())|| (Sat->iuCommandConnector.GetLVSCSepCmdA() || Sat->iuCommandConnector.GetLVSCSepCmdB())))
	{
		R2K137 = true;
	}
	LVSCSepSignal = (IsPowered() && R2K137);

	if (LVSCSepSignal)
	{
		LVSCSep25sTimer.SetRunning(true);
		LVSCSep60sTimer.SetRunning(true);
	}
	
	R2K202 = (LVSCSep25sTimer.ContactClosed());
	LVSCSep25sSignal = R2K202;
	R2K181 = (LVSCSep60sTimer.ContactClosed());

	//CM/SM Sep
	if (CSMSepSignal)
	{
		CMSMSepTimer.SetRunning(true);
	}

	if (CMSMSepTimer.ContactClosed())
		R2K188 = true;
	else
		R2K188 = false;

	if (IsPowered() && R2K188)
	{
		R2K11 = true;
		R2K17 = true;
	}

	if (CSMSepSignal && b005GSignal)
	{
		R2K14 = true;
		R2K20 = true;
	}

	//MESC Power

	//MESC Logic
	bool CSMSepDiffSignal = CSMSepDiff.EvaluateState(CSMSepSignal);
	if (CSMSepDiffSignal || GSEMESCLogicBusABArm)
	{
		R2K18 = true;
		R2K12 = true;
	}

	bool LVSCSep60sDiffSignal = LVSCSep60sDiff.EvaluateState(IsPowered() && LVSCSep60sTimer.ContactClosed());
	if (ImpactPlus11DiffSignal || LVSCSep60sDiffSignal || GSEMESCLogicBusASafe)
	{
		R2K18 = false;
	}
	if (ImpactPlus11Signal || LVSCSep60sDiffSignal || GSEMESCLogicBusBSafe)
	{
		R2K12 = false;
	}
	CountdownResetSignal = CountdownResetDiff.EvaluateState(ImpactPlus11Signal || LVSCSep60sDiffSignal || GSEMESCLogicBusBSafe);

	//MESC Pyro
	if (GSEMESCPyroBusABArm || CSMSepSignal)
	{
		R2K19 = true;
		R2K13 = true;
	}
	bool HFOnPlus10sDiffSignal = HFOnPlus10sDiff.EvaluateState(HFOnPlus10sSignal);
	if (HFOnPlus10sDiffSignal || LVSCSep60sDiffSignal || GSEMESCPyroBusASafe)
	{
		R2K19 = false;
	}
	if (HFOnPlus10sDiffSignal || LVSCSep60sDiffSignal || GSEMESCPyroBusBSafe)
	{
		R2K13 = false;
	}

	//Sep/Abort
	bool SepAbortDiffSignal = SepAbortDiff.EvaluateState(LVSCSepSignal || LESAbortSignal);

	if (SepAbortDiffSignal)
	{
		R2K43 = true;
		R2K44 = true;
	}

	if (LVSCSep25sSignal) LVSCSep30sTimer.SetRunning(true);
	//No need for relay R2K107ABC

	if (IsPowered() && LVSCSep30sTimer.ContactClosed())
	{
		R2K43 = false;
		R2K44 = false;
	}

	if (CSMSepSignal)
	{
		R2K68 = true;
		R2K69 = true;
	}

	//ELS
	R2K146 = Sat->els.ELSCA.GetMainParachuteDeployRelay();
	R2K145 = Sat->els.ELSCB.GetMainParachuteDeployRelay();

	if (IsPowered() && (R2K145 || R2K146))
	{
		BaroSwitchTimer.SetRunning(true);
		ImpactTimer.SetRunning(true);
	}

	R2K193 = BaroSwitchTimer.ContactClosed();
	R2K192 = ImpactTimer.ContactClosed();
	b12KBaroSwitchPlus20sSignal = R2K193;
	bool ImpactDiffSignal = ImpactDiff.EvaluateState(ads->GetImpactSwitch());

	if (R2K192 || (b12KBaroSwitchPlus20sSignal && ImpactDiffSignal))
	{
		R2K147 = true;
	}

	ImpactSignal = (IsPowered() && R2K147);

	if (ImpactSignal)
	{
		R2K55 = true;
		R2K67 = true;
	}

	if (ImpactSignal || (R2K147 && !R2K111))
	{
		Impact11sTimer.SetRunning(true);
	}

	if (Impact11sTimer.ContactClosed())
	{
		R2K130 = true;
	}

	bool FPLPower = Sat->FlightPostLandingBus.Voltage() > SP_MIN_DCVOLTAGE;
	bool ImpactFPL = FPLPower && R2K147;

	if (ImpactFPL && ads->GetAttitudeSwitch())
	{
		R2K149 = true;
	}
	else
	{
		R2K149 = false;
	}
	bool StableISignal = (ImpactFPL && (!R2K149));
	bool StableIISignal = (ImpactFPL && (R2K149));

	R2K186 = StableIISignal;
	if (StableIISignal)
	{
		R2K110 = true;
	}
	ImpactPlus11Signal = (FPLPower && !R2K110 && R2K130);
	bool StableIAndImpactPlus11Signal = (ImpactPlus11Signal && !R2K186);

	if (StableIAndImpactPlus11Signal)
	{
		HFPlus1sTimer.SetRunning(true);
	}
	HFOnPlus10sSignal = FPLPower && HFPlus1sTimer.ContactClosed();
	if (HFOnPlus10sSignal)
	{
		HFPlus2sTimer.SetRunning(true);
	}
	if (HFPlus2sTimer.ContactClosed())
	{
		R2K110 = false;
		R2K111 = false;
	}

	ImpactPlus11DiffSignal = ImpactPlus11Diff.EvaluateState(ImpactPlus11Signal);

	if (StableIISignal)
	{
		StableIIPlus1MinTimer.SetRunning(true);
	}
	bool R2K194ABC = StableIIPlus1MinTimer.ContactClosed();
	bool StableIIPlus1MinSignal = FPLPower && R2K194ABC;
	if (StableIIPlus1MinSignal)
	{
		FillBagsTimer1.SetRunning(true);
	}
	bool R2K197ABC = FillBagsTimer1.ContactClosed();
	bool R2K150AB = FPLPower && R2K197ABC;
	bool StableIIPlus1MinDiffSignal = StableIIPlus1MinDiff.EvaluateState(StableIIPlus1MinSignal);
	if (FPLPower && R2K197ABC)
	{
		FillBagsTimer2.SetRunning(true);
	}
	bool R2K195ABC = FillBagsTimer2.ContactClosed();
	bool R2K151AB = FPLPower && R2K195ABC;
	bool R2K60AB = FPLPower && R2K194ABC && !R2K150AB;
	bool R2K61AB = FPLPower && R2K197ABC && !R2K151AB;
	if (R2K151AB)
	{
		FillBagsTimer3.SetRunning(true);
	}
	bool R2K196ABC = FillBagsTimer3.ContactClosed();
	bool R2K152AB = FPLPower && R2K196ABC;
	bool R2K62AB = R2K151AB && !R2K152AB;

	if (StableIIPlus1MinDiffSignal)
	{
		R2K63 = true;
		R2K66 = true;
	}
	if (FPLPower && R2K196ABC)
	{
		R2K66 = false;
	}

	//ECS
	if (LETJettisonSignal)
	{
		R2K6 = true;
	}
	if (CSMSepSignal || LESAbortSignal || GSEECSStart)
	{
		R2K8 = true;
		R2K7 = true;
	}
	else if (CountdownResetSignal)
	{
		R2K8 = false;
		R2K7 = false;
	}
	if (GSEECSStart)
	{
		R2K56 = true;
	}

	//PROP
	bool R2K154ABC = (NoAbort && Sat->dsky.GetCRelay(29)); //Gimbal Motors On
	bool GimbalMotorsOn1 = (IsPowered() && R2K154ABC);
	bool GimbalMotorsOn2 = gcc->GetDirectUllage() || gcc->GetDirectThrustOn();
	bool GimbalMotorsOn3 = LiftoffSignal && !R2K153;
	GimbalMotorsOn = GimbalMotorsOn1 || GimbalMotorsOn2 || GimbalMotorsOn3;
	if (IsPowered() && !(GimbalMotorsOn1 || GimbalMotorsOn2))
	{
		GimbalMotors30sTimer.SetRunning(true);
	}
	R2K153 = GimbalMotors30sTimer.ContactClosed();

	if (LiftoffSignal && !R2K153)
	{
		GimbalMotors80sTimer.SetRunning(true);
	}
	bool R2K122 = GimbalMotors80sTimer.ContactClosed();
	bool GimbalMotorsStartSequence = GimbalMotorsOn && !R2K122;
	bool GimbalMotorsStopSignal = GimbalMotorsDiff.EvaluateState(GimbalMotors30sTimer.ContactClosed());

	//sprintf(oapiDebugString(), "R2K153 %d R2K154 %d Gim1 %d Gim2 %d Gim3 %d Start %d Stop %d StopState %d", R2K153ABC, R2K154ABC, GimbalMotorsOn1, GimbalMotorsOn2, GimbalMotorsOn3, GimbalMotorsStartSequence, GimbalMotorsStopSignal, GimbalMotorsDiff.GetState());

	if (GimbalMotorsStartSequence)
	{
		GimbalMotorYaw1StartTimer.SetRunning(true);
		GimbalMotorYaw1OnTimer.SetRunning(true);
		GimbalMotorYaw2StartTimer.SetRunning(true);
		GimbalMotorYaw2OnPitch2StartTimer.SetRunning(true);
		GimbalMotorPitch2OnTimer.SetRunning(true);
	}
	bool R2K167AB = GimbalMotorYaw1StartTimer.ContactClosed();
	if (R2K167AB)
	{
		R2K21 = true;
	}
	bool R2K168AB = GimbalMotorYaw1OnTimer.ContactClosed();
	if (R2K168AB)
	{
		R2K22 = true;
		R2K24 = true;
	}
	bool R2K169AB = GimbalMotorYaw2StartTimer.ContactClosed();
	if (R2K169AB)
	{
		R2K23 = true;
		R2K25 = true;
	}
	bool R2K170AB = GimbalMotorYaw2OnPitch2StartTimer.ContactClosed();
	if (R2K170AB)
	{
		R2K26 = true;
		R2K27 = true;
	}
	bool R2K171AB = GimbalMotorPitch2OnTimer.ContactClosed();
	if (R2K171AB)
	{
		R2K28 = true;
	}
	if (GimbalMotorsStopSignal)
	{
		R2K21 = false;
		R2K22 = false;
		R2K23 = false;
		R2K24 = false;
		R2K25 = false;
		R2K26 = false;
		R2K27 = false;
		R2K28 = false;
	}

	R2K119 = gcc->GetDirectThrustOff(); //TBD: Programer Reset?
	SPSArmSignal = LVSCSep25sSignal && !R2K119;
	if (SPSArmSignal && GimbalMotorsOn)
	{
		R2K58 = true;
		R2K59 = true;
	}
	else
	{
		R2K58 = false;
		R2K59 = false;
	}

	//EPS
	EntryBattsOnMainSignal = EntryBattsOnMainDiff.EvaluateState(CSMSepSignal || GimbalMotorsOn);
	bool R2K144ABC = CSMSepSignal || GimbalMotorsOn || LESAbortSignal;
	EntryBattsOffMainSignal = EntryBattsOffMainDiff.EvaluateState(IsPowered() && !R2K144ABC);

	//sprintf(oapiDebugString(), "SPSArm %d GimbalMotorsOn %d GimbalTimers %lf %lf Stop %d", SPSArmSignal, GimbalMotorsOn, GimbalMotors30sTimer.GetTime(),GimbalMotors80sTimer.GetTime(), GimbalMotorsStopSignal);
	//sprintf(oapiDebugString(), "Logic Bus %d %d Pyro Bus %d %d Oxid Dump %d %d Tower %d %d LES %d %d", R2K18AB, R2K12AB, R2K19AB, R2K13AB, R2K3AB, R2K72AB, R2K9AB, R2K15AB, R2K16AB, R2K10AB);
}

void MCP_SCC::ProgramerReset()
{
	R2K6 = false;
	R2K7 = false;
	R2K8 = false;
	R2K9 = false;
	R2K10 = false;
	R2K11 = false;
	R2K15 = false;
	R2K16 = false;
	R2K17 = false;
	R2K21 = false;
	R2K22 = false;
	R2K23 = false;
	R2K24 = false;
	R2K25 = false;
	R2K26 = false;
	R2K27 = false;
	R2K28 = false;
	R2K43 = false;
	R2K44 = false;
	R2K55 = false;
	R2K56 = false;
	R2K63 = false;
	R2K66 = false;
	R2K67 = false;
	R2K68 = false;
	R2K69 = false;
	R2K70 = false;
	R2K71 = false;
	R2K72 = false;

	R2K135 = false;
	R2K222 = false;
	R2K137 = false;
	R2K134 = false;
	R2K142 = false;
	R2K131 = false;

	R2K173 = false;
	R2K147 = false;
	R2K110 = true; //Is this right?
	R2K111 = true;
	R2K130 = false;
	R2K100 = false;
}

bool MCP_SCC::GetFireSafeA()
{
	if (!R2K15)
	{
		return true;
	}
	return false;
}

bool MCP_SCC::GetFireSafeB()
{
	if (!R2K9)
	{
		return true;
	}
	return false;
}

bool MCP_SCC::GetFireArmA()
{
	if (R2K15)
	{
		return true;
	}
	return false;
}

bool MCP_SCC::GetFireArmB()
{
	if (R2K9)
	{
		return true;
	}
	return false;
}

bool MCP_SCC::GetFireArm(bool IsSysA)
{
	if (IsSysA)
	{
		return GetFireArmA();
	}
	else
	{
		return GetFireArmB();
	}
}

bool MCP_SCC::GetFireSafe(bool IsSysA)
{
	if (IsSysA)
	{
		return GetFireSafeA();
	}
	else
	{
		return GetFireSafeB();
	}
}

bool MCP_SCC::GetLESMotorFireA()
{
	return R2K16;
}

bool MCP_SCC::GetLESMotorFireB()
{
	return R2K10;
}

bool MCP_SCC::GetLESMotorFire(bool IsSysA)
{
	if (IsSysA)
	{
		return GetLESMotorFireA();
	}
	else
	{
		return GetLESMotorFireB();
	}
}

bool MCP_SCC::IsPowered()
{
	return bPower;
}

void MCP_SCC::DeterminePowerState()
{
	bPower = true;
}

bool MCP_SCC::GetCMSMSepA()
{
	if (R2K11) return true;
	return false;
}

bool MCP_SCC::GetCMSMSepB()
{
	if (R2K17) return true;
	return false;
}

bool MCP_SCC::GetCMSMSep(bool IsSysA)
{
	if (IsSysA) return GetCMSMSepA();
	return GetCMSMSepB();
}

bool MCP_SCC::GetELSActiveA()
{
	if (R2K20) return true;
	return false;
}

bool MCP_SCC::GetELSActiveB()
{
	if (R2K14) return true;
	return false;
}

bool MCP_SCC::GetELSActive(bool IsSysA)
{
	if (IsSysA) return GetELSActiveA();
	return GetELSActiveB();
}

bool MCP_SCC::GetRestartSignal()
{
	return R2K222;
}

void MCP_SCC::MESCLogicBusesArm(bool set)
{
	GSEMESCLogicBusABArm = true;
}

void MCP_SCC::MESCLogicBusASafe(bool set)
{
	GSEMESCLogicBusASafe = set;
}

void MCP_SCC::MESCLogicBusBSafe(bool set)
{
	GSEMESCLogicBusBSafe = set;
}

bool MCP_SCC::GetMESCLogicBusArm(bool IsSysA)
{
	if (IsSysA) return GetMESCLogicBusArmA();
	return GetMESCLogicBusArmB();
}

bool MCP_SCC::GetMESCLogicBusArmA()
{
	return R2K18;
}

bool MCP_SCC::GetMESCLogicBusArmB()
{
	return R2K12;
}

void MCP_SCC::MESCPyroBusesArm(bool set)
{
	GSEMESCPyroBusABArm = set;
}

void MCP_SCC::MESCPyroBusASafe(bool set)
{
	GSEMESCPyroBusASafe = set;
}

void MCP_SCC::MESCPyroBusBSafe(bool set)
{
	GSEMESCPyroBusBSafe = set;
}

void MCP_SCC::MasterControlTransfer()
{
	R2K100 = true;
}

void MCP_SCC::ResetGSESignals()
{
	GSEMESCLogicBusABArm = false;
	GSEMESCLogicBusASafe = false;
	GSEMESCLogicBusBSafe = false;
	GSEMESCPyroBusABArm = false;
	GSEMESCPyroBusASafe = false;
	GSEMESCPyroBusBSafe = false;
	GSEECSStart = false;
}

bool MCP_SCC::GetMESCPyroBusArm(bool IsSysA)
{
	if (IsSysA) return GetMESCPyroBusArmA();
	return GetMESCPyroBusArmB();
}

bool MCP_SCC::GetMESCPyroBusArmA()
{
	return R2K19;
}

bool MCP_SCC::GetMESCPyroBusArmB()
{
	return R2K13;
}

bool MCP_SCC::GetSeparateAbortSignal(bool IsSysA)
{
	if (IsSysA) return GetSeparateAbortSignalA();
	return GetSeparateAbortSignalB();
}

bool MCP_SCC::GetSeparateAbortSignalA()
{
	return R2K43;
}

bool MCP_SCC::GetSeparateAbortSignalB()
{
	return R2K44;
}

bool MCP_SCC::GetRCSDump(bool IsSysA)
{
	if (IsSysA) return GetRCSDumpA();
	return GetRCSDumpB();
}

bool MCP_SCC::GetRCSDumpA()
{
	return R2K1;
}

bool MCP_SCC::GetRCSDumpB()
{
	return R2K70;
}

bool MCP_SCC::GetOxidDump(bool IsSysA)
{
	if (IsSysA) return GetOxidDumpA();
	return GetRCSDumpB();
}

bool MCP_SCC::GetOxidDumpA()
{
	return R2K3;
}

bool MCP_SCC::GetOxidDumpB()
{
	return R2K72;
}

bool MCP_SCC::GetRCSPurge(bool IsSysA)
{
	if (IsSysA) return GetRCSPurgeA();
	return GetRCSPurgeB();
}

bool MCP_SCC::GetRCSPurgeA()
{
	return R2K2;
}

bool MCP_SCC::GetRCSPurgeB()
{
	return R2K71;
}

bool MCP_SCC::GetGimbalStart(bool yaw, int num)
{
	if (yaw)
	{
		if (num == 1)
		{
			return R2K21 && !R2K22;
		}
		else
		{
			return R2K25 && !R2K26;
		}
	}
	else
	{
		if (num == 1)
		{
			return R2K24 && !R2K23;
		}
		else
		{
			return R2K27 && !R2K28;
		}
	}
}

bool MCP_SCC::GetGimbalOn(bool yaw, int num)
{
	if (yaw)
	{
		if (num == 1)
		{
			return R2K21 && R2K22;
		}
		else
		{
			return R2K25 && R2K26;
		}
	}
	else
	{
		if (num == 1)
		{
			return R2K24 && R2K23;
		}
		else
		{
			return R2K27 && R2K28;
		}
	}
}

bool MCP_SCC::GetGimbalOff(bool yaw, int num)
{
	if (yaw)
	{
		if (num == 1)
		{
			return !R2K21;
		}
		else
		{
			return !R2K25;
		}
	}
	else
	{
		if (num == 1)
		{
			return !R2K24;
		}
		else
		{
			return !R2K27;
		}
	}
}

void MCP_SCC::SaveState(FILEHANDLE scn)
{
	bool arr[15];

	oapiWriteLine(scn, MCP_SCC_START_STRING);

	arr[0] = GNFailSignal; arr[1] = GimbalMotorsOn; arr[2] = false; arr[3] = SPSEngineHold; arr[4] = LVSCSep25sSignal; arr[5] = LESAbortSignal; arr[6] = CSMSepSignal; arr[7] = LETJettisonSignal;
	arr[8] = b005GSignal; arr[9] = NoAbort; arr[10] = b12KBaroSwitchPlus20sSignal; arr[11] = LVSCSepSignal; arr[12] = ImpactSignal; arr[13] = CountdownResetSignal; arr[14] = false;
	papiWriteScenario_boolarr(scn, "SIGNALS1", arr, 15);

	arr[0] = ImpactPlus11Signal; arr[1] = HFOnPlus10sSignal; arr[2] = ImpactPlus11DiffSignal; arr[3] = GSEMESCLogicBusABArm; arr[4] = GSEMESCLogicBusASafe; arr[5] = GSEMESCLogicBusBSafe;
	arr[6] = GSEMESCPyroBusABArm; arr[7] = GSEMESCPyroBusASafe; arr[8] = GSEMESCPyroBusBSafe;
	papiWriteScenario_boolarr(scn, "SIGNALS2", arr, 9);

	arr[0] = R2K1; arr[1] = R2K2; arr[2] = R2K3; arr[3] = false; arr[4] = false; arr[5] = R2K6; arr[6] = R2K7; arr[7] = R2K8;
	arr[8] = R2K9; arr[9] = R2K10; arr[10] = R2K11; arr[11] = R2K12; arr[12] = R2K13; arr[13] = R2K14; arr[14] = R2K15;
	papiWriteScenario_boolarr(scn, "RELAYS1", arr, 15);

	arr[0] = R2K16; arr[1] = R2K17; arr[2] = R2K18; arr[3] = R2K19; arr[4] = R2K20; arr[5] = R2K21; arr[6] = R2K22; arr[7] = R2K23;
	arr[8] = R2K24; arr[9] = R2K25; arr[10] = R2K26; arr[11] = R2K27; arr[12] = R2K28; arr[13] = R2K29; arr[14] = R2K30;
	papiWriteScenario_boolarr(scn, "RELAYS2", arr, 15);

	arr[0] = R2K31; arr[1] = R2K32; arr[2] = false; arr[3] = R2K34; arr[4] = R2K35; arr[5] = R2K36; arr[6] = false; arr[7] = false;
	arr[8] = R2K39; arr[9] = R2K40; arr[10] = false; arr[11] = R2K42; arr[12] = R2K43; arr[13] = R2K44; arr[14] = false;
	papiWriteScenario_boolarr(scn, "RELAYS3", arr, 15);

	arr[0] = false; arr[1] = false; arr[2] = false; arr[3] = false; arr[4] = false; arr[5] = false; arr[6] = false; arr[7] = R2K53;
	arr[8] = false; arr[9] = R2K55; arr[10] = R2K56; arr[11] = R2K57; arr[12] = R2K58; arr[13] = R2K59; arr[14] = false;
	papiWriteScenario_boolarr(scn, "RELAYS4", arr, 15);

	arr[0] = false; arr[1] = false; arr[2] = R2K63; arr[3] = false; arr[4] = false; arr[5] = R2K66; arr[6] = R2K67; arr[7] = R2K68;
	arr[8] = R2K69; arr[9] = R2K70; arr[10] = R2K71; arr[11] = R2K72; arr[12] = false; arr[13] = false; arr[14] = false;
	papiWriteScenario_boolarr(scn, "RELAYS5", arr, 15);

	arr[0] = false; arr[1] = false; arr[2] = false; arr[3] = false; arr[4] = false; arr[5] = false; arr[6] = false; arr[7] = false;
	arr[8] = false; arr[9] = R2K100; arr[10] = false; arr[11] = false; arr[12] = false; arr[13] = false; arr[14] = false;
	papiWriteScenario_boolarr(scn, "RELAYS7", arr, 15);

	arr[0] = false; arr[1] = false; arr[2] = false; arr[3] = false; arr[4] = R2K110; arr[5] = R2K111; arr[6] = false; arr[7] = false;
	arr[8] = false; arr[9] = false; arr[10] = R2K116; arr[11] = false; arr[12] = false; arr[13] = R2K119; arr[14] = false;
	papiWriteScenario_boolarr(scn, "RELAYS8", arr, 15);

	arr[0] = false; arr[1] = false; arr[2] = false; arr[3] = false; arr[4] = R2K125; arr[5] = R2K126; arr[6] = false; arr[7] = false;
	arr[8] = R2K129; arr[9] = R2K130; arr[10] = R2K131; arr[11] = R2K132; arr[12] = R2K133; arr[13] = R2K134; arr[14] = R2K135;
	papiWriteScenario_boolarr(scn, "RELAYS9", arr, 15);

	arr[0] = R2K136; arr[1] = R2K137; arr[2] = R2K138; arr[3] = R2K139; arr[4] = R2K140; arr[5] = R2K141; arr[6] = R2K142; arr[7] = false;
	arr[8] = false; arr[9] = R2K145; arr[10] = R2K146; arr[11] = R2K147; arr[12] = false; arr[13] = R2K149; arr[14] = false;
	papiWriteScenario_boolarr(scn, "RELAYS10", arr, 15);

	arr[0] = false; arr[1] = false; arr[2] = R2K153; arr[3] = false; arr[4] = false; arr[5] = R2K156; arr[6] = false; arr[7] = false;
	arr[8] = false; arr[9] = false; arr[10] = false; arr[11] = false; arr[12] = false; arr[13] = false; arr[14] = false;
	papiWriteScenario_boolarr(scn, "RELAYS11", arr, 15);

	arr[0] = false; arr[1] = false; arr[2] = false; arr[3] = false; arr[4] = false; arr[5] = false; arr[6] = false; arr[7] = R2K173;
	arr[8] = R2K174; arr[9] = false; arr[10] = false; arr[11] = R2K177; arr[12] = false; arr[13] = false; arr[14] = false;
	papiWriteScenario_boolarr(scn, "RELAYS12", arr, 15);

	arr[0] = R2K181; arr[1] = false; arr[2] = false; arr[3] = false; arr[4] = false; arr[5] = R2K186; arr[6] = R2K187; arr[7] = R2K188;
	arr[8] = false; arr[9] = false; arr[10] = false; arr[11] = R2K192; arr[12] = R2K193; arr[13] = false; arr[14] = false;
	papiWriteScenario_boolarr(scn, "RELAYS13", arr, 15);

	arr[0] = false; arr[1] = false; arr[2] = false; arr[3] = false; arr[4] = false; arr[5] = false; arr[6] = R2K202; arr[7] = false;
	arr[8] = false; arr[9] = false; arr[10] = false; arr[11] = false; arr[12] = false; arr[13] = false; arr[14] = false;
	papiWriteScenario_boolarr(scn, "RELAYS14", arr, 15);

	arr[0] = false; arr[1] = false; arr[2] = false; arr[3] = false; arr[4] = false; arr[5] = false; arr[6] = false; arr[7] = false;
	arr[8] = false; arr[9] = false; arr[10] = false; arr[11] = R2K222; arr[12] = false; arr[13] = false; arr[14] = false;
	papiWriteScenario_boolarr(scn, "RELAYS15", arr, 15);

	arr[0] = CSMSepDiff.GetState(); arr[1] = LVSCSep60sDiff.GetState(); arr[2] = HFOnPlus10sDiff.GetState(); arr[3] = SepAbortDiff.GetState(); arr[4] = ImpactDiff.GetState();
	arr[5] = ImpactPlus11Diff.GetState(); arr[6] = GimbalMotorsDiff.GetState(); arr[7] = CountdownResetDiff.GetState(); arr[8] = StableIIPlus1MinDiff.GetState();
	arr[9] = EntryBattsOnMainDiff.GetState(); arr[10] = EntryBattsOffMainDiff.GetState();
	papiWriteScenario_boolarr(scn, "DIFFERENTIATORS", arr, 11);

	LESMotorFireTimer.SaveState(scn, "LESMotorFireTimer_BEGIN", "TD_END");
	LiftoffTimer.SaveState(scn, "LiftoffTimer_BEGIN", "TD_END");
	LVSCSep25sTimer.SaveState(scn, "LVSCSep25sTimer_BEGIN", "TD_END");
	LVSCSep30sTimer.SaveState(scn, "LVSCSep30sTimer_BEGIN", "TD_END");
	LVSCSep60sTimer.SaveState(scn, "LVSCSep60sTimer_BEGIN", "TD_END");
	BaroSwitchTimer.SaveState(scn, "BaroSwitchTimer_BEGIN", "TD_END");
	ImpactTimer.SaveState(scn, "ImpactTimer_BEGIN", "TD_END");
	Impact11sTimer.SaveState(scn, "Impact11sTimer_BEGIN", "TD_END");
	HFPlus1sTimer.SaveState(scn, "HFPlus1sTimer_BEGIN", "TD_END");
	HFPlus2sTimer.SaveState(scn, "HFPlus2sTimer_BEGIN", "TD_END");
	RCSPurge80sTimer.SaveState(scn, "RCSPurge80sTimer_BEGIN", "TD_END");
	RCSPurge250sTimer.SaveState(scn, "RCSPurge250sTimer_BEGIN", "TD_END");
	GimbalMotors30sTimer.SaveState(scn, "GimbalMotors30sTimer_BEGIN", "TD_END");
	GimbalMotors80sTimer.SaveState(scn, "GimbalMotors80sTimer_BEGIN", "TD_END");
	GimbalMotorYaw1StartTimer.SaveState(scn, "GimbalMotorYaw1StartTimer_BEGIN", "TD_END");
	GimbalMotorYaw1OnTimer.SaveState(scn, "GimbalMotorYaw1OnTimer_BEGIN", "TD_END");
	GimbalMotorYaw2StartTimer.SaveState(scn, "GimbalMotorYaw2StartTimer_BEGIN", "TD_END");
	GimbalMotorYaw2OnPitch2StartTimer.SaveState(scn, "GimbalMotorYaw2OnPitch2StartTimer_BEGIN", "TD_END");
	GimbalMotorPitch2OnTimer.SaveState(scn, "GimbalMotorPitch2OnTimer_BEGIN", "TD_END");
	StableIIPlus1MinTimer.SaveState(scn, "StableIIPlus1MinTimer_BEGIN", "TD_END");
	FillBagsTimer1.SaveState(scn, "FillBagsTimer1_BEGIN", "TD_END");
	FillBagsTimer2.SaveState(scn, "FillBagsTimer2_BEGIN", "TD_END");
	FillBagsTimer3.SaveState(scn, "FillBagsTimer3_BEGIN", "TD_END");

	oapiWriteLine(scn, MCP_SCC_END_STRING);
}

void MCP_SCC::LoadState(FILEHANDLE scn)
{
	bool arr[15];
	char *line;

	while (oapiReadScenario_nextline(scn, line)) {
		if (!strnicmp(line, MCP_SCC_END_STRING, sizeof(MCP_SCC_END_STRING)))
			break;

		if (papiReadScenario_boolarr(line, "SIGNALS1", arr, 15))
		{
			GNFailSignal = arr[0]; GimbalMotorsOn = arr[1]; SPSEngineHold = arr[3]; LVSCSep25sSignal = arr[4]; LESAbortSignal = arr[5]; CSMSepSignal = arr[6]; LETJettisonSignal = arr[7];
			b005GSignal = arr[8]; NoAbort = arr[9]; b12KBaroSwitchPlus20sSignal = arr[10]; LVSCSepSignal = arr[11]; ImpactSignal = arr[12]; CountdownResetSignal = arr[13];
		}
		else if (papiReadScenario_boolarr(line, "SIGNALS2", arr, 9))
		{
			ImpactPlus11Signal = arr[0];
			HFOnPlus10sSignal = arr[1];
			ImpactPlus11DiffSignal = arr[2];
			GSEMESCLogicBusABArm = arr[3];
			GSEMESCLogicBusASafe = arr[4];
			GSEMESCLogicBusBSafe = arr[5];
			GSEMESCPyroBusABArm = arr[6];
			GSEMESCPyroBusASafe = arr[7];
			GSEMESCPyroBusBSafe = arr[8];
		}
		else if (papiReadScenario_boolarr(line, "RELAYS1", arr, 15))
		{
			R2K1 = arr[0]; R2K2 = arr[1]; R2K3 = arr[2]; R2K6 = arr[5]; R2K7 = arr[6]; R2K8 = arr[7]; R2K9 = arr[8]; R2K10 = arr[9];
			R2K11 = arr[10]; R2K12 = arr[11]; R2K13 = arr[12]; R2K14 = arr[13]; R2K15 = arr[14];
		}
		else if (papiReadScenario_boolarr(line, "RELAYS2", arr, 15))
		{
			R2K16 = arr[0]; R2K17 = arr[1]; R2K18 = arr[2]; R2K19 = arr[3]; R2K20 = arr[4]; R2K21 = arr[5]; R2K22 = arr[6]; R2K23 = arr[7];
			R2K24 = arr[8]; R2K25 = arr[9]; R2K26 = arr[10]; R2K27 = arr[11]; R2K28 = arr[12]; R2K29 = arr[13]; R2K30 = arr[14];
		}
		else if (papiReadScenario_boolarr(line, "RELAYS3", arr, 15))
		{
			R2K31 = arr[0]; R2K32 = arr[1]; R2K34 = arr[3]; R2K35 = arr[4]; R2K36 = arr[5];
			R2K39 = arr[8]; R2K40 = arr[9]; R2K42 = arr[11]; R2K43 = arr[12]; R2K44 = arr[13];
		}
		else if (papiReadScenario_boolarr(line, "RELAYS4", arr, 15))
		{
			R2K53 = arr[7]; R2K55 = arr[9]; R2K56 = arr[10]; R2K57 = arr[11]; R2K58 = arr[12]; R2K59 = arr[13];
		}
		else if (papiReadScenario_boolarr(line, "RELAYS5", arr, 15))
		{
			R2K63 = arr[2]; R2K66 = arr[5]; R2K67 = arr[6]; R2K68 = arr[7]; R2K69 = arr[8]; R2K70 = arr[9]; R2K71 = arr[10]; R2K72 = arr[11];
		}
		else if (papiReadScenario_boolarr(line, "RELAYS7", arr, 15))
		{
			R2K100 = arr[9];
		}
		else if (papiReadScenario_boolarr(line, "RELAYS8", arr, 15))
		{
			R2K110 = arr[4]; R2K111 = arr[5]; R2K116 = arr[10]; R2K119 = arr[13];
		}
		else if (papiReadScenario_boolarr(line, "RELAYS9", arr, 15))
		{
			R2K125 = arr[4]; R2K126 = arr[5]; R2K129 = arr[8]; R2K130 = arr[9]; R2K131 = arr[10]; R2K132 = arr[11]; R2K133 = arr[12]; R2K134 = arr[13]; R2K135 = arr[14];
		}
		else if (papiReadScenario_boolarr(line, "RELAYS10", arr, 15))
		{
			R2K136 = arr[0]; R2K137 = arr[1]; R2K138 = arr[2]; R2K139 = arr[3]; R2K140 = arr[4]; R2K141 = arr[5]; R2K142 = arr[6]; R2K145 = arr[9]; R2K146 = arr[10];
			R2K147 = arr[11]; R2K149 = arr[13];
		}
		else if (papiReadScenario_boolarr(line, "RELAYS11", arr, 15))
		{
			R2K153 = arr[2]; R2K156 = arr[5];
		}
		else if (papiReadScenario_boolarr(line, "RELAYS12", arr, 15))
		{
			R2K173 = arr[7]; R2K173 = arr[8]; R2K177 = arr[11];
		}
		else if (papiReadScenario_boolarr(line, "RELAYS13", arr, 15))
		{
			R2K181 = arr[0]; R2K186 = arr[5];  R2K187 = arr[6]; R2K188 = arr[7]; R2K192 = arr[11]; R2K193 = arr[12];
		}
		else if (papiReadScenario_boolarr(line, "RELAYS14", arr, 15))
		{
			R2K202 = arr[6];
		}
		else if (papiReadScenario_boolarr(line, "RELAYS15", arr, 15))
		{
			R2K222 = arr[11];
		}
		else if (papiReadScenario_boolarr(line, "DIFFERENTIATORS", arr, 11))
		{
			CSMSepDiff.SetState(arr[0]); LVSCSep60sDiff.SetState(arr[1]); HFOnPlus10sDiff.SetState(arr[2]); SepAbortDiff.SetState(arr[3]); ImpactDiff.SetState(arr[4]);
			ImpactPlus11Diff.SetState(arr[5]); GimbalMotorsDiff.SetState(arr[6]); CountdownResetDiff.SetState(arr[7]); StableIIPlus1MinDiff.SetState(arr[8]);
			EntryBattsOnMainDiff.SetState(arr[9]); EntryBattsOffMainDiff.SetState(arr[10]);
		}
		else if (!strnicmp(line, "LESMotorFireTimer_BEGIN", sizeof("LESMotorFireTimer_BEGIN"))) {
			LESMotorFireTimer.LoadState(scn, "TD_END");
		}
		else if (!strnicmp(line, "CMSMSepTimer_BEGIN", sizeof("CMSMSepTimer_BEGIN"))) {
			CMSMSepTimer.LoadState(scn, "TD_END");
		}
		else if (!strnicmp(line, "LiftoffTimer_BEGIN", sizeof("LiftoffTimer_BEGIN"))) {
			LiftoffTimer.LoadState(scn, "TD_END");
		}
		else if (!strnicmp(line, "LVSCSep25sTimer_BEGIN", sizeof("LVSCSep25sTimer_BEGIN"))) {
			LVSCSep25sTimer.LoadState(scn, "TD_END");
		}
		else if (!strnicmp(line, "LVSCSep30sTimer_BEGIN", sizeof("LVSCSep30sTimer_BEGIN"))) {
			LVSCSep30sTimer.LoadState(scn, "TD_END");
		}
		else if (!strnicmp(line, "LVSCSep60sTimer_BEGIN", sizeof("LVSCSep60sTimer_BEGIN"))) {
			LVSCSep60sTimer.LoadState(scn, "TD_END");
		}
		else if (!strnicmp(line, "BaroSwitchTimer_BEGIN", sizeof("BaroSwitchTimer_BEGIN"))) {
			BaroSwitchTimer.LoadState(scn, "TD_END");
		}
		else if (!strnicmp(line, "ImpactTimer_BEGIN", sizeof("ImpactTimer_BEGIN"))) {
			ImpactTimer.LoadState(scn, "TD_END");
		}
		else if (!strnicmp(line, "Impact11sTimer_BEGIN", sizeof("Impact11sTimer_BEGIN"))) {
			Impact11sTimer.LoadState(scn, "TD_END");
		}
		else if (!strnicmp(line, "HFPlus1sTimer_BEGIN", sizeof("HFPlus1sTimer_BEGIN"))) {
			HFPlus1sTimer.LoadState(scn, "TD_END");
		}
		else if (!strnicmp(line, "HFPlus2sTimer_BEGIN", sizeof("HFPlus2sTimer_BEGIN"))) {
			HFPlus2sTimer.LoadState(scn, "TD_END");
		}
		else if (!strnicmp(line, "RCSPurge80sTimer_BEGIN", sizeof("RCSPurge80sTimer_BEGIN"))) {
			RCSPurge80sTimer.LoadState(scn, "TD_END");
		}
		else if (!strnicmp(line, "RCSPurge250sTimer_BEGIN", sizeof("RCSPurge250sTimer_BEGIN"))) {
			RCSPurge250sTimer.LoadState(scn, "TD_END");
		}
		else if (!strnicmp(line, "GimbalMotors30sTimer_BEGIN", sizeof("GimbalMotors30sTimer_BEGIN"))) {
			GimbalMotors30sTimer.LoadState(scn, "TD_END");
		}
		else if (!strnicmp(line, "GimbalMotors80sTimer_BEGIN", sizeof("GimbalMotors80sTimer_BEGIN"))) {
			GimbalMotors80sTimer.LoadState(scn, "TD_END");
		}
		else if (!strnicmp(line, "GimbalMotorYaw1StartTimer_BEGIN", sizeof("GimbalMotorYaw1StartTimer_BEGIN"))) {
			GimbalMotorYaw1StartTimer.LoadState(scn, "TD_END");
		}
		else if (!strnicmp(line, "GimbalMotorYaw1OnTimer_BEGIN", sizeof("GimbalMotorYaw1OnTimer_BEGIN"))) {
			GimbalMotorYaw1OnTimer.LoadState(scn, "TD_END");
		}
		else if (!strnicmp(line, "GimbalMotorYaw2StartTimer_BEGIN", sizeof("GimbalMotorYaw2StartTimer_BEGIN"))) {
			GimbalMotorYaw2StartTimer.LoadState(scn, "TD_END");
		}
		else if (!strnicmp(line, "GimbalMotorYaw2OnPitch2StartTimer_BEGIN", sizeof("GimbalMotorYaw2OnPitch2StartTimer_BEGIN"))) {
			GimbalMotorYaw2OnPitch2StartTimer.LoadState(scn, "TD_END");
		}
		else if (!strnicmp(line, "GimbalMotorPitch2OnTimer_BEGIN", sizeof("GimbalMotorPitch2OnTimer_BEGIN"))) {
			GimbalMotorPitch2OnTimer.LoadState(scn, "TD_END");
		}
		else if (!strnicmp(line, "StableIIPlus1MinTimer_BEGIN", sizeof("StableIIPlus1MinTimer_BEGIN"))) {
			StableIIPlus1MinTimer.LoadState(scn, "TD_END");
		}
		else if (!strnicmp(line, "FillBagsTimer1_BEGIN", sizeof("FillBagsTimer1_BEGIN"))) {
			FillBagsTimer1.LoadState(scn, "TD_END");
		}
		else if (!strnicmp(line, "FillBagsTimer2_BEGIN", sizeof("FillBagsTimer2_BEGIN"))) {
			FillBagsTimer2.LoadState(scn, "TD_END");
		}
		else if (!strnicmp(line, "FillBagsTimer3_BEGIN", sizeof("FillBagsTimer3_BEGIN"))) {
			FillBagsTimer3.LoadState(scn, "TD_END");
		}
	}
}