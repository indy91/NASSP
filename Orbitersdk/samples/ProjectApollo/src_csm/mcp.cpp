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
	float flt = 0;

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
	R1K33 = false;
	R1K34 = false;

	InputReset();
}

void MCP_GCC::Init(Saturn *s, MCP_SCC* sc)
{
	Sat = s;
	scc = sc;
}

void MCP_GCC::InputReset()
{
	RTC40LETJettison = false;
	RTC41GNFail = false;
	RTC42GNFailInhibit = false;
	RTC51MinusVHFAntennaOn = false;
	RTC52PlusVHFAntennaOn = false;
	RTC71Abort = false;
	RTC61CSMSep = false;
}

void MCP_GCC::Timestep(double simdt)
{
	//G&N
	if (RTC41GNFail)
	{
		R1K60 = true;
		R1K60 = false;
	}
	if (RTC42GNFailInhibit)
	{
		R1K60 = false;
		R1K61 = true;
	}

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

	//UDL input duration is 25-35 milliseconds, so just reset it all on the next timestep. SCC has one timestep to recognize signal
	InputReset();
}

void MCP_GCC::ProgramerReset()
{
	R1K33 = false;
	R1K34 = false;
}

void MCP_GCC::RealTimeCommand(int cmd)
{
	switch (cmd)
	{
	case 041:
		RTC41GNFail = true;
		break;
	case 042:
		RTC42GNFailInhibit = true;
		break;
	case 051:
		RTC51MinusVHFAntennaOn = true;
		break;
	case 052:
		RTC52PlusVHFAntennaOn = true;
		break;
	case 053:
		RTC53GNAntennaSwitching = true;
		break;
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
	HFPlus2sTimer(1.0)
{
	bPower = false;

	R2K135 = false;
	R2K9A = false;
	R2K9B = false;
	R2K15A = false;
	R2K15B = false;
	R2K10AB = false;
	R2K16AB = false;
	R2K222 = false;
	R2K11A = false;
	R2K11B = false;
	R2K17A = false;
	R2K17B = false;
	R2K188 = false;
	R2K14A = false;
	R2K14B = false;
	R2K20A = false;
	R2K20B = false;
	R2K140 = false;
	R2K137 = false;
	R2K132 = false;
	R2K177 = false;
	R2K141 = false;
	R2K138A = false;
	R2K138B = false;
	R2K133A = false;
	R2K133B = false;
	R2K139A = false;
	R2K139B = false;
	R2K202 = false;
	R2K133 = false;
	R2K142 = false;
	R2K131 = false;
	R2K12A = false;
	R2K12B = false;
	R2K18A = false;
	R2K18B = false;
	R2K116 = false;
	R2K13A = false;
	R2K13B = false;
	R2K19A = false;
	R2K19B = false;
	R2K181 = false;
	R2K43A = false;
	R2K43B = false;
	R2K44A = false;
	R2K44B = false;
	R2K173 = false;
	R2K174 = false;
	R2K125 = false;
	R2K126 = false;
	R2K129 = false;
	R2K147ABC = false;
	R2K147DEF = false;
	R2K192 = false;
	R2K193 = false;
	R2K55ABC = false;
	R2K67ABC = false;
	R2K130 = false;
	R2K149 = false;
	R2K110 = false;
	R2K111 = false;
	R2K186 = false;

	GNFailSignal = false;
	NoAbort = false;
	b12KBaroSwitchPlus20sSignal = false;
	ImpactSignal = false;
	ImpactPlus11DiffSignal = false;

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
	if (!R2K132 && (Sat->iuCommandConnector.GetLiftOffCircuit(true) || Sat->iuCommandConnector.GetLiftOffCircuit(false)))
	{
		R2K132 = true;
	}

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

	CSMSepSignal = R2K131;

	R2K125 = LESAbortSignal;
	R2K126 = GNFailSignal;

	NoAbort = (IsPowered() && !R2K125 && !R2K126);
	R2K129 = (LVSCSep25sSignal && NoAbort);

	if (ads->Get005GSwitch() || (CSMSepSignal && R2K129 && Sat->dsky.GetCRelay(28)))

	if (IsPowered() && R2K132)
		LiftoffTimer.SetRunning(true);

	if (LiftoffTimer.ContactClosed())
		R2K177 = true;
	else
		R2K177 = false;

	R2K141 = IsPowered() && R2K177;

	if (LETJettisonSignal)
	{
		R2K9A = true;
		R2K9B = true;
		R2K15A = true;
		R2K15B = true;
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
		R2K10AB = true;
		R2K16AB = true;
	}

	//LES Abort
	R2K140 = LETJettisonSignal;
	LESAbortSignal = gcc->GetAbort() && !R2K140;
	if (!R2K142 && LESAbortSignal && !R2K141)
	{
		R2K142 = true;
	}

	R2K133 = IsPowered() && R2K142;

	if (CSMSepSignal || LESAbortSignal)
	{
		R2K138A = true;
		R2K138B = true;
		R2K133A = true;
		R2K133B = true;
		R2K139A = true;
		R2K139B = true;
	}
	else
	{
		R2K138A = false;
		R2K138B = false;
		R2K133A = false;
		R2K133B = false;
		R2K139A = false;
		R2K139B = false;
	}

	//LV/SC Sep
	R2K136 = LETJettisonSignal;
	if (!R2K137 && ((R2K136 && gcc->GetAbort())|| (Sat->iuCommandConnector.GetLVSCSepCmdA() || Sat->iuCommandConnector.GetLVSCSepCmdB())))
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
		R2K11A = true;
		R2K11B = true;
		R2K17A = true;
		R2K17B = true;
	}

	if (b005GSignal)
	{
		R2K14A = true;
		R2K14B = true;
		R2K20A = true;
		R2K20B = true;
	}

	//MESC Power

	//MESC Logic
	bool CSMSepDiffSignal = CSMSepDiff.EvaluateState(CSMSepSignal);
	if (CSMSepDiffSignal || GSEMESCLogicBusABArm)
	{
		R2K18A = true;
		R2K18B = true;
		R2K12A = true;
		R2K12B = true;
	}

	bool LVSCSep60sDiffSignal = LVSCSep60sDiff.EvaluateState(IsPowered() && LVSCSep60sTimer.ContactClosed());
	if (ImpactPlus11DiffSignal || LVSCSep60sDiffSignal || GSEMESCLogicBusASafe)
	{
		R2K18A = false;
		R2K18B = false;
	}
	if (ImpactPlus11Signal || LVSCSep60sDiffSignal || GSEMESCLogicBusBSafe)
	{
		R2K12A = false;
		R2K12B = false;
	}

	//MESC Pyro
	if (GSEMESCPyroBusABArm || CSMSepSignal)
	{
		R2K19A = true;
		R2K19B = true;
		R2K13A = true;
		R2K13B = true;
	}
	bool HFOnPlus10sDiffSignal = HFOnPlus10sDiff.EvaluateState(HFOnPlus10sSignal);
	if (HFOnPlus10sDiffSignal || LVSCSep60sDiffSignal || GSEMESCPyroBusASafe)
	{
		R2K19A = false;
		R2K19B = false;
	}
	if (HFOnPlus10sDiffSignal || LVSCSep60sDiffSignal || GSEMESCPyroBusBSafe)
	{
		R2K13A = false;
		R2K13B = false;
	}

	//Sep/Abort
	bool SepAbortDiffSignal = SepAbortDiff.EvaluateState(LVSCSepSignal || LESAbortSignal);

	if (SepAbortDiffSignal)
	{
		R2K43A = true;
		R2K43B = true;
		R2K44A = true;
		R2K44B = true;
	}

	if (LVSCSep25sSignal) LVSCSep30sTimer.SetRunning(true);
	//No need for relay R2K107ABC

	if (IsPowered() && LVSCSep30sTimer.ContactClosed())
	{
		R2K43A = false;
		R2K43B = false;
		R2K44A = false;
		R2K44B = false;
	}

	//ELS
	R2K145_146 = Sat->els.BaroSwitch10k.IsClosed();

	if (IsPowered() && R2K145_146)
	{
		BaroSwitchTimer.SetRunning(true);
		ImpactTimer.SetRunning(true);
	}

	R2K193 = BaroSwitchTimer.ContactClosed();
	R2K192 = ImpactTimer.ContactClosed();
	b12KBaroSwitchPlus20sSignal = R2K193;
	bool ImpactDiffSignal = ImpactDiff.EvaluateState(ads->GetImpactSwitch());

	if (R2K192)
	{
		R2K147DEF = true;
	}
	if (R2K192 || (b12KBaroSwitchPlus20sSignal && ImpactDiffSignal))
	{
		R2K147ABC = true;
	}

	ImpactSignal = (IsPowered() && R2K147DEF);

	if (ImpactSignal)
	{
		R2K55ABC = true;
		R2K67ABC = true;
	}

	if (ImpactSignal || (R2K147ABC && !R2K111))
	{
		Impact11sTimer.SetRunning(true);
	}

	if (Impact11sTimer.ContactClosed())
	{
		R2K130 = true;
	}

	bool FPLPower = true;
	bool ImpactFPL = FPLPower && R2K147ABC;

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
}

void MCP_SCC::ProgramerReset()
{
	R2K9A = false;
	R2K9B = false;
	R2K15A = false;
	R2K15B = false;
	R2K135 = false;
	R2K10AB = false;
	R2K16AB = false;
	R2K222 = false;
	R2K11A = false;
	R2K11B = false;
	R2K17A = false;
	R2K17B = false;
	R2K137 = false;
	R2K132 = false;
	R2K142 = false;
	R2K131 = false;
	R2K43A = false;
	R2K43B = false;
	R2K44A = false;
	R2K44B = false;
	R2K173 = false;
	R2K147ABC = false;
	R2K147DEF = false;
	R2K55ABC = false;
	R2K67ABC = false;
	R2K110 = true; //Is this right?
	R2K111 = true;
	R2K130 = false;
}

bool MCP_SCC::GetFireSafeA()
{
	if (!R2K15A || !R2K15B)
	{
		return true;
	}
	return false;
}

bool MCP_SCC::GetFireSafeB()
{
	if (!R2K9A || !R2K9B)
	{
		return true;
	}
	return false;
}

bool MCP_SCC::GetFireArmA()
{
	if (R2K15A && R2K15B)
	{
		return true;
	}
	return false;
}

bool MCP_SCC::GetFireArmB()
{
	if (R2K9A && R2K9B)
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
	return R2K16AB;
}

bool MCP_SCC::GetLESMotorFireB()
{
	return R2K10AB;
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
	if (R2K11A && R2K11B) return true;
	return false;
}

bool MCP_SCC::GetCMSMSepB()
{
	if (R2K17A && R2K17B) return true;
	return false;
}

bool MCP_SCC::GetCMSMSep(bool IsSysA)
{
	if (IsSysA) return GetCMSMSepA();
	return GetCMSMSepB();
}

bool MCP_SCC::GetELSActiveA()
{
	if (R2K20A && R2K20B) return true;
	return false;
}

bool MCP_SCC::GetELSActiveB()
{
	if (R2K14A && R2K14B) return true;
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
	return (R2K18A || R2K18B);
}

bool MCP_SCC::GetMESCLogicBusArmB()
{
	return (R2K12A || R2K12B);
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

void MCP_SCC::ResetGSESignals()
{
	GSEMESCLogicBusABArm = false;
	GSEMESCLogicBusASafe = false;
	GSEMESCLogicBusBSafe = false;
	GSEMESCPyroBusABArm = false;
	GSEMESCPyroBusASafe = false;
	GSEMESCPyroBusBSafe = false;
}

bool MCP_SCC::GetMESCPyroBusArm(bool IsSysA)
{
	if (IsSysA) return GetMESCPyroBusArmA();
	return GetMESCPyroBusArmB();
}

bool MCP_SCC::GetMESCPyroBusArmA()
{
	return (R2K19A || R2K19B);
}

bool MCP_SCC::GetMESCPyroBusArmB()
{
	return (R2K13A || R2K13B);
}

bool MCP_SCC::GetSeparateAbortSignal(bool IsSysA)
{
	if (IsSysA) return GetSeparateAbortSignalA();
	return GetSeparateAbortSignalB();
}

bool MCP_SCC::GetSeparateAbortSignalA()
{
	return (R2K43A && R2K43B);
}

bool MCP_SCC::GetSeparateAbortSignalB()
{
	return (R2K44A && R2K44B);
}