/***************************************************************************
This file is part of Project Apollo - NASSP
Copyright 2017

Coupling Data Unit

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
#include "apolloguidance.h"
#include "cdu.h"
#include "papi.h"
#include "ioChannels.h"

#define CHECK_BIT(var,pos) ( (((var) & (pos)) > 0 ) ? (1) : (0) )

// The auto optics don't work right with the current logic for some reason. What the CMC wants is a total desired angle change,
// but what it sends is increments that always make the auto optics overshoot the target. This flag can be used to let the CMC
// send the total new value of the error counter to the OCDUs instead of an increment. The downside is that the optics drive never
// reaches the maximum speed, which probably isn't correct either. This logic is only used in the OCDUs and only when the
// coarse align mode is active.
#define OCDU_CA_ERROR_INCREMENT true

CDU::CDU(ApolloGuidance &comp, int l, int err, int cdutype) : agc(comp)
{
	loc = l;
	err_channel = err;
	CDUType = cdutype;

	if (CDUType == 0)
	{
		CDUZeroBit = 4;
		ErrorCounterBit = 5;
		AltOutBit = 8;
	}
	else
	{
		CDUZeroBit = 0;
		ErrorCounterBit = 1;
		AltOutBit = 7;
	}

	ReadCounter = 0;
	ErrorCounter = 0;
	NewReadCounter = 0.0;
	ZeroCDU = false;
	ErrorCounterEnabled = false;
	AltOutput = 0;
	CA = false;
}

void CDU::Timestep(double simdt)
{
	ChannelValue val12;

	val12 = agc.GetOutputChannel(012);

	//Coarse align logic
	if (CDUType == 0)
	{
		CA = (val12[3] == 1);
	}
	else if (CDUType == 2)
	{
		CA = (val12[ErrorCounterBit] == 1) && (val12[AltOutBit] != 1);
	}
	else
	{
		CA = false;
	}

	if (ZeroCDU == false)
	{
		uint16_t NewReadCounterInt = (uint16_t)(NewReadCounter / CDU_STEP);

		if (NewReadCounterInt != ReadCounter)
		{
			bool dirup;
			if (NewReadCounterInt > ReadCounter)
			{
				if (NewReadCounterInt - ReadCounter > 077777)
				{
					dirup = false;
				}
				else
				{
					dirup = true;
				}
			}
			else
			{
				if (ReadCounter - NewReadCounterInt > 077777)
				{
					dirup = true;
				}
				else
				{
					dirup = false;
				}
			}

			uint16_t readcountertemp = ReadCounter;

			if (dirup)
			{
				while (NewReadCounterInt != ReadCounter)
				{
					ReadCounter++;

					if (CHECK_BIT(ReadCounter, 2) != CHECK_BIT(readcountertemp, 2))
					{
						agc.vagc->memory[loc]++;
						agc.vagc->memory[loc] &= 077777;
					}
					if (ErrorCounterEnabled && CA && (ErrorCounter > -0600) && (CHECK_BIT(ReadCounter, 8) != CHECK_BIT(readcountertemp, 8)))
					{
						ErrorCounter--;
					}
					readcountertemp = ReadCounter;
				}
			}
			else
			{
				while (NewReadCounterInt != ReadCounter)
				{
					ReadCounter--;

					if (CHECK_BIT(ReadCounter, 2) != CHECK_BIT(readcountertemp, 2))
					{
						agc.vagc->memory[loc]--;
						agc.vagc->memory[loc] &= 077777;
					}
					if (ErrorCounterEnabled && CA && (ErrorCounter < 0600) && (CHECK_BIT(ReadCounter, 8) != CHECK_BIT(readcountertemp, 8)))
					{
						ErrorCounter++;
					}
					readcountertemp = ReadCounter;
				}
			}
		}
	}

	if (val12[AltOutBit] == 1)
	{
		AltOutput = ErrorCounter;
	}
	else
	{
		AltOutput = 0;
	}

	//sprintf(oapiDebugString(), "ReadCounter %f NewReadCounter %f ZeroCDU %d CDUZeroBit %d", ReadCounter*DEG, NewReadCounter*DEG, ZeroCDU, CDUZeroBit);
	//sprintf(oapiDebugString(), "ReadCounter %d ErrorCounter %d ErrorCounterEnabled %d", ReadCounter, ErrorCounter, ErrorCounterEnabled);
}

void CDU::ChannelOutput(int address, ChannelValue val)
{
	if (address == 012)
	{
		if (val[CDUZeroBit] == 1) {
			DoZeroCDU();
		}
		else
		{
			ZeroCDU = false;
		}

		if (val[ErrorCounterBit] == 1) {
			if (ErrorCounterEnabled == false)
			{
				ErrorCounter = 0;
				ErrorCounterEnabled = true;
			}
		}
		else
		{
			if (ErrorCounterEnabled == true) {
				ErrorCounter = 0;
			}
			ErrorCounterEnabled = false;
		}
	}

	if (address == err_channel)
	{
		if (ErrorCounterEnabled) {
			int delta = val.to_ulong();
			int ErrorCounterOld = ErrorCounter;

			if (delta & 040000) { // Negative
				if (CDUType == 2 && CA && !OCDU_CA_ERROR_INCREMENT)
				{
					ErrorCounter = -((~delta) & 077777);
				}
				else
				{
					ErrorCounter += -((~delta) & 077777);
				}
				if (ErrorCounter < -0600)
				{
					ErrorCounter = -0600;
				}
			}
			else {
				if (CDUType == 2 && CA && !OCDU_CA_ERROR_INCREMENT)
				{
					ErrorCounter = delta & 077777;
				}
				else
				{
					ErrorCounter += delta & 077777;
				}
				if (ErrorCounter > 0600)
				{
					ErrorCounter = 0600;
				}
			}
			//sprintf(oapiDebugString(), "Channel %d ErrorCounter %d Delta %d", err_channel, ErrorCounter, ErrorCounter - ErrorCounterOld);
		}
	}

	//sprintf(oapiDebugString(), "Channel %d ReadCounter %f ErrorCounter %o", err_channel, ReadCounter*DEG, ErrorCounter);
}

void CDU::DoZeroCDU()
{
	ReadCounter = 0;
	NewReadCounter = 0;
	ZeroCDU = true;
}

void CDU::SetReadCounter(double angle)
{
	if (!ZeroCDU)
	{
		NewReadCounter = angle;

		if (NewReadCounter >= PI2) {
			NewReadCounter -= PI2;
		}
		if (NewReadCounter < 0) {
			NewReadCounter += PI2;
		}
	}
}

int CDU::GetErrorCounter()
{ 
	if (ErrorCounterEnabled)
	{
		return ErrorCounter;
	}
	
	return 0;
}

int CDU::GetAltOutput()
{
	return AltOutput;
}

void CDU::SaveState(FILEHANDLE scn, char *start_str, char *end_str) {
	oapiWriteLine(scn, start_str);

	oapiWriteScenario_int(scn, "READCOUNTER", ReadCounter);
	papiWriteScenario_double(scn, "NEWREADCOUNTER", NewReadCounter);
	oapiWriteScenario_int(scn, "ERRORCOUNTER", ErrorCounter);
	papiWriteScenario_bool(scn, "ZEROCDU", ZeroCDU);
	papiWriteScenario_bool(scn, "ERRORCOUNTERENABLED", ErrorCounterEnabled);

	oapiWriteLine(scn, end_str);
}

void CDU::LoadState(FILEHANDLE scn, char *end_str) {
	char *line;
	int tmp = 0; // Used in boolean type loader
	int end_len = strlen(end_str);

	while (oapiReadScenario_nextline(scn, line)) {
		if (!strnicmp(line, end_str, end_len)) {
			break;
		}
		if (papiReadScenario_int(line, "READCOUNTER", tmp))
		{
			ReadCounter = tmp;
		}
		papiReadScenario_double(line, "NEWREADCOUNTER", NewReadCounter);
		papiReadScenario_int(line, "ERRORCOUNTER", ErrorCounter);
		papiReadScenario_bool(line, "ZEROCDU", ZeroCDU);
		papiReadScenario_bool(line, "ERRORCOUNTERENABLED", ErrorCounterEnabled);
	}
}

BlockICDU::BlockICDU(ApolloGuidance &comp, int sb, int reg) : agc(comp)
{
	ShaftAngle = 0.0;
	SystemBit = sb;
	AGCRegister = reg;
	sin_05x = sin_1x = sin_16x = 0.0;
	cos_1x = 0.0;
}

void BlockICDU::Timestep(double simdt)
{
	if (IsPowered() == false) return;

	//Normalize
	if (ShaftAngle >= PI2)
	{
		ShaftAngle -= PI2;
	}
	else if (ShaftAngle < 0)
	{
		ShaftAngle += PI2;
	}

	//Resolvers
	double sin_1x_theta, cos_1x_theta, sin_16x_theta, cos_16x_theta;

	if (agc.GetZeroEncoderMode() == false)
	{
		sin_1x_theta = sin(*GimbalAngle);
		cos_1x_theta = cos(*GimbalAngle);
		sin_16x_theta = sin(16.0*(*GimbalAngle));
		cos_16x_theta = cos(16.0*(*GimbalAngle));
	}
	else
	{
		sin_1x_theta = 0.0;
		cos_1x_theta = 1.0;
		sin_16x_theta = 0.0;
		cos_16x_theta = 1.0;
	}

	sin_05x = agc.GetZeroEncoderMode() ? -sin(0.5*ShaftAngle) : 0.0;
	sin_1x = sin_1x_theta * cos(ShaftAngle) - cos_1x_theta * sin(ShaftAngle);
	sin_16x = sin_16x_theta * cos(16.0*ShaftAngle) - cos_16x_theta * sin(16.0*ShaftAngle);

	//Motor Drive Amp
	ShaftAngle += simdt*0.5*(sin_05x + (agc.GetIMUFineAlign() ? (sin_1x + 0.1*sin_16x) : 0.0));

	//"Pulses" to AGC
	if (agc.GetZeroEncoderMode() == false)
	{
		int pulses = (int)(((double)radToGyroPulses(ShaftAngle)) / 64.0);
		agc.ProcessIMUCDUReadCount(AGCRegister, (pulses & 077777));
	}
	else
	{
		agc.ProcessIMUCDUReadCount(AGCRegister, 0);
	}

	//sprintf(oapiDebugString(), "Coarse %d Fine %d Zero %d ShaftAngle %lf GimbalAngle %lf sin_05x %lf sin_1x %lf sin_16x %lf", agc.GetIMUCoarseAlign(), agc.GetIMUFineAlign(), agc.GetZeroEncoderMode(), ShaftAngle*DEG, (*GimbalAngle)*DEG, sin_05x, sin_1x, sin_16x);
}

void BlockICDU::ProcessChannel12(ChannelValue val)
{
	if (agc.GetIMUFineAlign() || val[SystemBit] == false) return;
	if (agc.vagc->memory[042] == 0)
	{
		sprintf(oapiDebugString(), "SystemBit %d inp %o", SystemBit, agc.vagc->memory[042]);
		return;
	}

	int value = (040000 - agc.vagc->memory[042]);
	if (value > 384)
	{
		value = 384;
	}

	if (val[IMUPlus])
	{
	}
	else if (val[IMUMinus])
	{
		value = -value;//040000 + sign;
	}
	else
	{
		value = 0;
	}

	ShaftAngle += DigitalToAnalogConverter(value);
	//sprintf(oapiDebugString(), "SystemBit %d inp %o DAC: %d", SystemBit, agc.vagc->memory[042], value);

	//agc.vagc->memory[042] = 0;
}

int BlockICDU::radToGyroPulses(double angle)
{
	return (int)((angle * 2097152.0) / PI2);
}

double BlockICDU::DigitalToAnalogConverter(int val)
{
	return 1.917475984857051e-04*(double)(val);
}

void BlockICDU::SetAngleDevice(double *pAngle)
{
	GimbalAngle = pAngle;
}

double BlockICDU::GetAttitudeError()
{
	if (agc.GetIMUCoarseAlign()) return 0.0;

	return sin_1x;
}

bool BlockICDU::IsPowered()
{
	return true;
}

void BlockICDU::SaveState(FILEHANDLE scn, char *start_str, char *end_str) {
	oapiWriteLine(scn, start_str);

	papiWriteScenario_double(scn, "ShaftAngle", ShaftAngle);

	oapiWriteLine(scn, end_str);
}

void BlockICDU::LoadState(FILEHANDLE scn, char *end_str) {
	char *line;
	int tmp = 0; // Used in boolean type loader
	int end_len = strlen(end_str);

	while (oapiReadScenario_nextline(scn, line)) {
		if (!strnicmp(line, end_str, end_len)) {
			break;
		}
		papiReadScenario_double(line, "ShaftAngle", ShaftAngle);
	}
}