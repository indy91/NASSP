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

MCP_GCC::MCP_GCC()
{
	RTC40LETJettison = false;
	RTC71Abort = false;
}

void MCP_GCC::Init(Saturn *s)
{
	Sat = s;
}

void MCP_GCC::Timestep(double simdt)
{

}

MCP_SCC::MCP_SCC() : LESMotorFireTimer(0.5), CMSMSepTimer(1.0), LiftoffTimer(42.0), LVSCSep25sTimer(2.5)
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
}

void MCP_SCC::Init(Saturn *s, MCP_GCC *g)
{
	Sat = s;
	gcc = g;
}

void MCP_SCC::Timestep(double simdt)
{
	LESMotorFireTimer.Timestep(simdt);
	CMSMSepTimer.Timestep(simdt);
	LiftoffTimer.Timestep(simdt);
	LVSCSep25sTimer.Timestep(simdt);

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
	if (Sat->iuCommandConnector.GetLiftOffCircuit(true) || Sat->iuCommandConnector.GetLiftOffCircuit(false))
	{
		R2K132 = true;
	}

	//Internal processing
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

	if (LVSCSepSignal) LVSCSep25sTimer.SetRunning(true);
	
	R2K202 = (LVSCSep25sTimer.ContactClosed());
	LVSCSep25sSignal = R2K202;

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