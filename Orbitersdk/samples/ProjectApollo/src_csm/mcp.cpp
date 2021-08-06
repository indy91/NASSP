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

MCP_SCC::MCP_SCC() : LESMotorFireTimer(0.5)
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
}

void MCP_SCC::Init(Saturn *s)
{
	Sat = s;
}

void MCP_SCC::Timestep(double simdt)
{
	LESMotorFireTimer.Timestep(simdt);

	DeterminePowerState();

	//Input signals
	//IU
	//Tower Jettison
	if (!R2K135 && (Sat->iuCommandConnector.GetTowerJettisonCmdA() || Sat->iuCommandConnector.GetTowerJettisonCmdB()))
	{
		R2K135 = true;
	}
	//Restart
	if (!R2K222 && Sat->GetSIISepState())
	{
		R2K222 = true;
	}

	//Internal processing
	if (R2K135)
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