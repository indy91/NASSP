/***************************************************************************
  This file is part of Project Apollo - NASSP
  Copyright 2004-2005 Mark Grant

  ORBITER vessel module: Saturn CSM computer

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

// To force orbitersdk.h to use <fstream> in any compiler version
#pragma include_alias( <fstream.h>, <fstream> )
#include "Orbitersdk.h"
#include "stdio.h"
#include "math.h"

#include "soundlib.h"
#include "nasspsound.h"
#include "nasspdefs.h"

#include "apolloguidance.h"
#include "dsky.h"
#include "csmcomputer.h"
#include "toggleswitch.h"
#include "saturn.h"
#include "ioChannels.h"
#include "papi.h"
#include "thread.h"

CSMcomputer::CSMcomputer(SoundLib &s, DSKY &display, DSKY &display2, IMU &im, CDU &sc, CDU &tc, BlockICDU &og, BlockICDU &ig, BlockICDU &mg, PanelSDK &p) :
	ApolloGuidance(s, display, im, sc, tc, og, ig, mg, p), dsky2(display2), IMUTurnOnDelayTimer(40.0)

{
	isLGC = false;

	//
	// Last RCS settings.
	//

	LastOut5 = 0;
	LastOut6 = 0;
	LastOut11 = 0;

	thread.Resume ();

	for (int i = 0;i < 13;i++)
	{
		KRelays[i] = false;
	}
}

CSMcomputer::~CSMcomputer()

{
	//
	// Nothing for now.
	//
}

void CSMcomputer::SetMissionInfo(std::string AGCVersion, char *OtherVessel)

{
	ApolloGuidance::SetMissionInfo(AGCVersion, OtherVessel);

	//
	// Pick the appropriate AGC binary file based on name.
	//
	//

	char Buffer[100];
	sprintf(Buffer, "Config/ProjectApollo/%s.bin", AGCVersion.c_str());

	InitVirtualAGC(Buffer);
}

void CSMcomputer::agcTimestep(double simt, double simdt)
{
	// Do single timesteps to maintain sync with telemetry engine
	SingleTimestepPrep(simt, simdt);        // Setup
	if (LastCycled == 0) {					// Use simdt as difference if new run
		LastCycled = (simt - simdt); 
		sat->pcm.last_update = LastCycled;
	}	  
	double ThisTime = LastCycled;			// Save here
	
	uint64_t cycles = (uint64_t)((simt - LastCycled) / 0.00001171875);	// Get number of CPU cycles to do
	uint64_t cyclestart = vagc->countMCT;
	uint64_t cycleend = cyclestart + cycles;
	//long x = 0; 
	while(vagc->countMCT < cycleend) {
		SingleTimestep();
		//ThisTime += 0.00001171875;								// Add time
		//if((ThisTime - sat->pcm.last_update) > 0.00015625) {	// If a step is needed
		//	sat->pcm.TimeStep(ThisTime);						// do it
		//}
		//x++;
	}
	sat->pcm.TimeStep(simt);
	LastCycled += (0.00001171875 * (vagc->countMCT - cyclestart));						// Preserve the remainder
}

void CSMcomputer::Run ()
	{
		while(true)
		{
			timeStepEvent.Wait();
			{
				Lock lock(agcCycleMutex);
				agcTimestep(thread_simt,thread_simdt);
			}
		}
	};


void CSMcomputer::Timestep(double simt, double simdt)
{
	IMUTurnOnDelayTimer.Timestep(simdt);

	/*VECTOR3 RN, VN;
	RN.x = ConvertAGCDoubleToDouble(vagc->memory[0765], vagc->memory[0766], pow(2, 24));
	RN.y = ConvertAGCDoubleToDouble(vagc->memory[0767], vagc->memory[0770], pow(2, 24));
	RN.z = ConvertAGCDoubleToDouble(vagc->memory[0771], vagc->memory[0772], pow(2, 24));
	VN.x = ConvertAGCDoubleToDouble(vagc->memory[0773], vagc->memory[0774], pow(2, 7)*100.0);
	VN.y = ConvertAGCDoubleToDouble(vagc->memory[0775], vagc->memory[0776], pow(2, 7)*100.0);
	VN.z = ConvertAGCDoubleToDouble(vagc->memory[0777], vagc->memory[01000], pow(2, 7)*100.0);*/

	//sprintf(oapiDebugString(), "RN %lf %lf %lf VN %lf %lf %lf R %lf V %lf", RN.x, RN.y, RN.z, VN.x, VN.y, VN.z, length(RN), length(VN));

	//sprintf(oapiDebugString(), "LST1 %o %o %o %o %o %o", vagc->memory[0552], vagc->memory[0553], vagc->memory[0554], vagc->memory[0555], vagc->memory[0556], vagc->memory[0557]);
	//sprintf(oapiDebugString(), "LST2 %o %o %o %o %o %o", vagc->memory[0561], vagc->memory[0562], vagc->memory[0563], vagc->memory[0564], vagc->memory[0565], vagc->memory[0566]);
	//sprintf(oapiDebugString(), "POLYORDR %o", vagc->memory[01574]);
	//sprintf(oapiDebugString(), "TEL? %o", vagc->memory[01350]);
	//sprintf(oapiDebugString(), "TBASE2 %o TBASE3 %o TBASE4 %o TBASE5 %o", vagc->memory[0664], vagc->memory[0665], vagc->memory[0666], vagc->memory[0667]);
	//sprintf(oapiDebugString(), "FLAGWRD1 %o", agc.memory[0646]);
	//sprintf(oapiDebugString(), "Z %o BANK %o", regZ, regBank);
	sprintf(oapiDebugString(), "TENDROLL %d", vagc->memory[01561]);
	//sprintf(oapiDebugString(), "TIME2 %o TIME1 %o", vagc->memory[035], vagc->memory[036]);
	//sprintf(oapiDebugString(), "TMONITOR %o %o", vagc->memory[01570], vagc->memory[01571]);
	//sprintf(oapiDebugString(), "TIME2GR %o TIME1GR %o DTEAROT %o %o", vagc->memory[01466], vagc->memory[01467], vagc->memory[01144], vagc->memory[01145]);
	//sprintf(oapiDebugString(), "TPACIF1 %o %o TPACIFIC %o %o", vagc->memory[01627], vagc->memory[01630], vagc->memory[01536], vagc->memory[01537]);

	// DS20060302 For joystick stuff below
	sat = (Saturn *) OurVessel;

	//Always set this input bit
	if (sat->LVGuidanceSwitch.IsUp())
	{
		SetInputChannelBit(07, TransferSwitch, true);
	}
	else
	{
		SetInputChannelBit(07, TransferSwitch, false);
	}

	bool IMUHasPower = sat->imu.IsPowered();

	if (IMUHasPower)
	{
		IMUTurnOnDelayTimer.SetRunning(true);
	}

	if (IMUHasPower && IMUTurnOnDelayTimer.ContactClosed())
	{
		KRelays[5] = true;
	}
	else
	{
		KRelays[5] = false;
	}
	if (IMUHasPower && !KRelays[5])
	{
		KRelays[6] = true;
	}
	else
	{
		KRelays[6] = false;
	}

	//Zero Encoder
	if (IMUHasPower && sat->dsky.GetCRelay(0)) //TBD: Manual switch
		KRelays[0] = true;
	else
		KRelays[0] = false;

	//Coarse align
	if (IMUHasPower && (sat->dsky.GetCRelay(1) || KRelays[6])) //TBD: Manual switch
		KRelays[1] = true;
	else
		KRelays[1] = false;

	//Lock CDU
	if (IMUHasPower && sat->dsky.GetCRelay(2)) //TBD: Manual switch
		KRelays[2] = true;
	else
		KRelays[2] = false;

	//Fine Align
	if (IMUHasPower && sat->dsky.GetCRelay(3)) //TBD: Manual switch
		KRelays[3] = true;
	else
		KRelays[3] = false;

	//Entry
	if (IMUHasPower && sat->dsky.GetCRelay(10)) //TBD: Manual switch
		KRelays[4] = true;
	else
		KRelays[4] = false;

	//Attitude control
	if (IMUHasPower && sat->dsky.GetCRelay(9)) //TBD: Manual switch
		KRelays[11] = true;
	else
		KRelays[11] = false;

	//Signals to CMC
	if (KRelays[0])
		SetInputChannelBit(07, ZeroEncoderMode, true);
	else
		SetInputChannelBit(07, ZeroEncoderMode, false);

	if (KRelays[1])
		SetInputChannelBit(07, CoarseAlign, true);
	else
		SetInputChannelBit(07, CoarseAlign, false);

	if (KRelays[2])
		SetInputChannelBit(07, ManualCDU, true);
	else
		SetInputChannelBit(07, ManualCDU, false);

	if (KRelays[3])
		SetInputChannelBit(07, FineAlign, true);
	else
		SetInputChannelBit(07, FineAlign, false);

	if (KRelays[11])
		SetInputChannelBit(07, AttitudeControl, true);
	else
		SetInputChannelBit(07, AttitudeControl, false);

	if (KRelays[4])
		SetInputChannelBit(07, EntryMode, true);
	else
		SetInputChannelBit(07, EntryMode, false);

	bool ORofC = false;

	for (int i = 0;i < 33;i++)
	{
		if (sat->dsky.GetCRelay(i))
		{
			ORofC = true;
			break;
		}
	}

	if (ORofC)
	{
		SetInputChannelBit(07, ORofC1C33, true);
	}
	else
	{
		SetInputChannelBit(07, ORofC1C33, false);
	}

	//sprintf(oapiDebugString(), "%d %d %d %d %d %d", KRelays[0], KRelays[1], KRelays[2], KRelays[3], KRelays[4], KRelays[11]);

		//
		// Reduce time acceleration as per configured, not to jump to x100 or x1000 and freeze the simulation
		//
		
		if( sat->maxTimeAcceleration>0 )
		{
			if( oapiGetTimeAcceleration() > (double)sat->maxTimeAcceleration )
				oapiSetTimeAcceleration(sat->maxTimeAcceleration);
		}
		
		//
		// Do nothing if we have no power. (vAGC)
		//
		if (!IsPowered()) {
			// HARDWARE MUST RESTART

			// Clear flip-flop based registers
			//for (int i = 0;i < 053;i++)
			//{
			//	vagc->memory[i] = 0;
			//}
			vagc->memory[00] = 0;		// A
			vagc->memory[01] = 0;		// Q
			vagc->memory[02] = 02030;	// Z
			/*vagc->memory[04] = 0;		// IN0
			vagc->memory[05] = 0;		// IN1
			vagc->memory[06] = 0;		// IN2
			vagc->memory[07] = 0;		// IN3
			vagc->memory[010] = 0;		// OUT0
			vagc->memory[011] = 0;		// OUT1
			vagc->memory[012] = 0;		// OUT2
			vagc->memory[013] = 0;		// OUT3
			vagc->memory[014] = 0;		// OUT4
			vagc->memory[015] = 0;		// BANKREG
			vagc->memory[016] = 0;		// RELINT
			vagc->memory[017] = 0;		// INHINT
			vagc->memory[020] = 0;		// CYR
			vagc->memory[021] = 0;		// SR
			vagc->memory[022] = 0;		// CYL
			vagc->memory[023] = 0;		// SL
			vagc->memory[024] = 0;		// ZRUPT
			vagc->memory[025] = 0;		// BRUPT
			vagc->memory[026] = 0;		// ARUPT
			vagc->memory[027] = 0;		// QRUPT
			vagc->memory[030] = 0;		// BANKRUPT
			vagc->memory[031] = 0;		// OVRUPT
			vagc->memory[032] = 0;		// LPRUPT
			vagc->memory[033] = 0;		// DSRUPTSW
			vagc->memory[034] = 0;		// OVCTR
			vagc->memory[035] = 0;		// TIME2
			vagc->memory[036] = 0;		// TIME1
			vagc->memory[037] = 0;		// TIME3
			vagc->memory[040] = 0;		// TIME4*/

			vagc->countMCT = 0;
			vagc->nextTimerIncrement = 1280;
			vagc->INDEX = 0;
			vagc->INTERRUPTED = 0;
			vagc->B = 0;
			vagc->ruptFlatAddress = 0;
			vagc->ruptLastINDEX = 0;
			vagc->ruptLastZ = 0;
			vagc->overflowedTIME3 = 0;
			vagc->overflowedTIME4 = 0;
			vagc->uplinkReady = 0;
			vagc->downlinkReady = 0;
			agc.instructionCountDown = 1;
			// Reset last cycling time
			LastCycled = 0;

			// Clear ISR flag
			/*vagc.InIsr = 0;
			// Clear interrupt requests
			vagc.InterruptRequests[0] = 0;
			vagc.InterruptRequests[1] = 0;
			vagc.InterruptRequests[2] = 0;
			vagc.InterruptRequests[3] = 0;
			vagc.InterruptRequests[4] = 0;
			vagc.InterruptRequests[5] = 0;
			vagc.InterruptRequests[6] = 0;
			vagc.InterruptRequests[7] = 0;
			vagc.InterruptRequests[8] = 0;
			vagc.InterruptRequests[9] = 0;
			vagc.InterruptRequests[10] = 0;
			// Reset cycle counter and Extracode flags
			vagc.CycleCounter = 0;
			vagc.ExtraCode = 0;
			vagc.ExtraDelay = 2; // GOJAM and TC 4000 both take 1 MCT to execute
			// No idea about the interrupts/pending/etc so we reset those
			vagc.AllowInterrupt = 1;
			vagc.PendFlag = 0;
			vagc.PendDelay = 0;
			// Don't disturb erasable core
			// IO channels are flip-flop based and should reset, but that's difficult, so we'll ignore it.
			// Reset standby flip-flop
			vagc.Standby = 0;
			// Turn on EL display and CMC Light (DSKYWarn).
			vagc.DskyChannel163 = 1;
			SetOutputChannel(0163, 1);
			// Light OSCILLATOR FAILURE to signify power transient, and be forceful about it.
			vagc.InputChannel[033] &= 037777;
			OutputChannel[033] &= 037777;
			// Also, simulate the operation of the VOLTAGE ALARM, turn off STBY and RESTART light while power is off.
			// The RESTART light will come on as soon as the AGC receives power again.
			// This happens externally to the AGC program. See CSM 104 SYS HBK pg 399
			vagc.RestartLight = 1;
			dsky.ClearRestart();
			dsky2.ClearRestart();
			dsky.ClearStby();
			dsky2.ClearStby();*/

			// We should issue telemetry though.
			sat->pcm.TimeStep(simt);
			return;
		}

		//
		// Initial startup hack for Yaagc.
		//
		if (!PadLoaded)
		{
			double intpart;
			double fractpart = modf(oapiGetSimMJD(), &intpart);
			double clock = fractpart * 8640000. * pow((double) 2., (double)-28.);

			vagc->memory[035] = ConvertDecimalToAGCOctal(clock, true);
			vagc->memory[036] = ConvertDecimalToAGCOctal(clock, false);

			//vagc->instructionCountDown = -1;

			PadLoaded = true;
		}

		//
		// If MultiThread is enabled and the simulation is accellerated, the run vAGC in the AGC Thread,
		// otherwise run in main thread. at x1 acceleration, it is better to run vAGC totally synchronized
		//
		if(sat->IsMultiThread && oapiGetTimeAcceleration() > 1.0)
		{
			
			Lock lock(agcCycleMutex);
			thread_simt = simt;
			thread_simdt = simdt;
			timeStepEvent.Raise();
		}
		else
			agcTimestep(simt,simdt);

		//
		// Check nonspherical gravity sources
		//
		if (!OurVessel->NonsphericalGravityEnabled()) {
			sprintf(oapiDebugString(), "*** PLEASE ENABLE NONSPHERICAL GRAVITY SOURCES ***");
		}
		// Done!
		//sprintf(oapiDebugString(), "Standby: %d %d %I64d", sat->agc.vagc.Standby, sat->agc.vagc.SbyPressed, sat->agc.vagc.CycleCounter);

		return;
}

//
// Access simulated erasable memory.
//

bool CSMcomputer::ReadMemory(unsigned int loc, int &val)

{
	return GenericReadMemory(loc, val);
}

void CSMcomputer::WriteMemory(unsigned int loc, int val)

{
	GenericWriteMemory(loc, val);
}

void CSMcomputer::SetInputChannelBit(int channel, int bit, bool val){
	ApolloGuidance::SetInputChannelBit(channel, bit, val);
}

void CSMcomputer::SetOutputChannel(int channel, ChannelValue val){
	ApolloGuidance::SetOutputChannel(channel, val);
}

//
// We need to pass these I/O channels to both DSKYs.
//

void CSMcomputer::ProcessChannel10(ChannelValue val){
	dsky.ProcessChannel10(val);
	dsky2.ProcessChannel10(val);

	// Gimbal Lock & Prog alarm
	/*ChannelValue10 val10;
	val10.Value = val.to_ulong();
	if (val10.Bits.a == 12) {
		// Gimbal Lock
		GimbalLockAlarm = ((val10.Value & (1 << 5)) != 0);
		// Tracker alarm
		TrackerAlarm = ((val10.Value & (1 << 7)) != 0);
		// Prog alarm
		ProgAlarm = ((val10.Value & (1 << 8)) != 0);
	}*/
}

void CSMcomputer::ProcessChannel11Bit(int bit, bool val){

	LastOut11 = GetOutputChannel(011);
}

void CSMcomputer::ProcessChannel11(ChannelValue val){
	dsky.ProcessChannel11(val);
	dsky2.ProcessChannel11(val);

	LastOut11 = val.to_ulong();
}

void CSMcomputer::ProcessIMUCDUReadCount(int channel, int val) {
	SetErasable(channel, val);
}

// DS20060308 FDAI
void CSMcomputer::ProcessIMUCDUErrorCount(int channel, ChannelValue val){
	// These pulses work like the TVC pulses.
	// FULL NEEDLE DEFLECTION is 16.88 DEGREES
	// 030 PULSES = MAX IN ONE RELAY EVENT
	// 22 PULSES IS ONE DEGREE, 384 PULSES = FULL SCALE
	// 0.10677083 PIXELS PER PULSE

	Saturn *sat = (Saturn *) OurVessel;
	ChannelValue val12;
	if(channel != 012){ val12 = GetOutputChannel(012); }else{ val12 = val; }
	// 174 = X, 175 = Y, 176 = Z
	/*if(GetIMUCoarseAlign()){ return; } // Does not apply to us here.
	switch(channel){
	case 012:
		// Reset FDAI
		if (val12[EnableIMUCDUErrorCounters]) {
			if (sat->gdc.fdai_err_ena == 0) {
				// sprintf(oapiDebugString(),"FDAI: RESET");						
				sat->gdc.fdai_err_x = 0;
				sat->gdc.fdai_err_y = 0;
				sat->gdc.fdai_err_z = 0;
				sat->gdc.fdai_err_ena = 1;
			}
		} else {
			if (sat->gdc.fdai_err_ena == 1) {
				// sprintf(oapiDebugString(),"FDAI: RESET");
				sat->gdc.fdai_err_x = 0;
				sat->gdc.fdai_err_y = 0;
				sat->gdc.fdai_err_z = 0;
			}
			sat->gdc.fdai_err_ena = 0;
		}

		break;
		
	case 0174: // FDAI ROLL ERROR
		if(val12[EnableIMUCDUErrorCounters]){
			int delta = (val.to_ulong()&0777);
			// Direction for these is inverted.
			if(val.to_ulong()&040000){
				sat->gdc.fdai_err_x += delta;
			}else{
				sat->gdc.fdai_err_x -= delta;
			}
		}
//		sprintf(oapiDebugString(),"FDAI: NEEDLES: %d %d %d",sat->gdc.fdai_err_x,sat->gdc.fdai_err_y,sat->gdc.fdai_err_z);
		break;
	
	case 0175: // FDAI PITCH ERROR
		if(val12[EnableIMUCDUErrorCounters]){
			int delta = val.to_ulong()&0777;
			if(val.to_ulong()&040000){
				sat->gdc.fdai_err_y -= delta;
			}else{
				sat->gdc.fdai_err_y += delta;
			}
		}
//		sprintf(oapiDebugString(),"FDAI: NEEDLES: %d %d %d",sat->gdc.fdai_err_x,sat->gdc.fdai_err_y,sat->gdc.fdai_err_z);
		break;

	case 0176: // FDAI YAW ERROR
		if(val12[EnableIMUCDUErrorCounters]){
			int delta = val.to_ulong()&0777;
			if(val.to_ulong()&040000){
				sat->gdc.fdai_err_z += delta;
			}else{
				sat->gdc.fdai_err_z -= delta;
			}
		}
//		sprintf(oapiDebugString(),"FDAI: NEEDLES: %d %d %d",sat->gdc.fdai_err_x,sat->gdc.fdai_err_y,sat->gdc.fdai_err_z);
		break;
	}*/
}

void CSMcomputer::ProcessChannel14(ChannelValue val){
	// This entire deal is no longer necessary, but we'll leave the stub here in case it's needed later.
	/*
	ChannelValue12 val12;
	ChannelValue14 val14;
	val12.Value = GetOutputChannel(012);
	val14.Value = val;
	Saturn *sat = (Saturn *) OurVessel;	

	if(val12.Bits.TVCEnable){
		return; // Ignore
	} */
}

VESSEL *CSMcomputer::GetLM()
{
	OBJHANDLE hcsm = oapiGetVesselByName(OtherVesselName);
	if (hcsm)
	{
		VESSEL *LMVessel = oapiGetVesselInterface(hcsm);
		return LMVessel;
	}

	return NULL;
}

void CSMcomputer::SaveState(FILEHANDLE scn)
{
	oapiWriteLine(scn, AGC_START_STRING);
	ApolloGuidance::SaveState(scn);
	papiWriteScenario_boolarr(scn, "KRELAYS", KRelays, 13);
	IMUTurnOnDelayTimer.SaveState(scn, "DELAYTIMER_START", "DELAYTIMER_END");
	oapiWriteLine(scn, AGC_END_STRING);
}

void CSMcomputer::LoadState(FILEHANDLE scn)
{
	char *line;
	int tmp = 0; // Used in boolean type loader
	int end_len = strlen(AGC_END_STRING);

	while (oapiReadScenario_nextline(scn, line)) {
		if (!strnicmp(line, AGC_END_STRING, end_len)) {
			break;
		}

		ApolloGuidance::LoadState(line);

		papiReadScenario_boolarr(line, "KRELAYS", KRelays, 13);
		if (!strnicmp(line, "DELAYTIMER_START", sizeof("DELAYTIMER_START"))) {
			IMUTurnOnDelayTimer.LoadState(scn, "DELAYTIMER_END");
		}
	}
}


//
// CM Optics class code
//

CMOptics::CMOptics() {

	sat = NULL;
	SextShaft = 0.0;
	TeleShaft = 0.0;
	SextTrunion = 0.0;
	TeleTrunion = 0.0;
	TeleShaftRate = 0.0;
	TeleTrunionRate = 0.0;
	dTrunion = 0.0;
	dShaft = 0.0;
	OpticsManualMovement = 0;
	Powered = 0;
	SextDualView = false;
	SextDVLOSTog = false;
	SextDVTimer = 0.0;
	OpticsCovered = true;
}

void CMOptics::Init(Saturn *vessel) {

	sat = vessel;
}

void CMOptics::SystemTimestep(double simdt) {

	// Optics system apparently uses 124.4 watts of power to operate.
	// This should probably vary up and down when the motors run, but I couldn't find data for it.
	Powered = 0; // Reset
	if (sat->GNOpticsMnACircuitBraker.Voltage() > SP_MIN_DCVOLTAGE){
		Powered |= 1;
	}
	if (sat->GNOpticsMnBCircuitBraker.Voltage() > SP_MIN_DCVOLTAGE){
		Powered |= 2;
	}
	if (sat->GNPowerOpticsSwitch.IsDown()) {
		Powered = 0;
	}
	switch(Powered){
		case 0: // OFF
			break;
		case 1: // MNA
			sat->GNOpticsMnACircuitBraker.DrawPower(124.4);
			break;
		case 2: // MNB
			sat->GNOpticsMnBCircuitBraker.DrawPower(124.4);
			break;
		case 3: // BOTH
			sat->GNOpticsMnACircuitBraker.DrawPower(62.2);
			sat->GNOpticsMnBCircuitBraker.DrawPower(62.2);
			break;
	}

}

// Paint counters. The documentation is not clear if the displayed number is supposed to be decimal degrees or CDU counts.
// The counters are mechanically connected to the telescope, so it is assumed to be decimal degrees.

bool CMOptics::PaintShaftDisplay(SURFHANDLE surf, SURFHANDLE digits){
	int value = (int)(TeleShaft*100.0*DEG);
	if (value < 0) { value += 36000; }
	return PaintDisplay(surf, digits, value);
}

bool CMOptics::PaintTrunnionDisplay(SURFHANDLE surf, SURFHANDLE digits){
	int value = (int)(TeleTrunion*100.0*DEG);
	if (value < 0) { value += 36000; }
	return PaintDisplay(surf, digits, value);
}

bool CMOptics::PaintDisplay(SURFHANDLE surf, SURFHANDLE digits, int value){
	int srx, sry, digit[5];
	int x=value;
	digit[0] = (x%10);
	digit[1] = (x%100)/10;
	digit[2] = (x%1000)/100;
	digit[3] = (x%10000)/1000;
	digit[4] = x/10000;

	srx = 8 + (digit[4] * 25);
	if (digit[4])
		sry = 33;
	else
		sry = 22;
	oapiBlt(surf, digits, 0, 0, srx, sry, 9, 12, SURF_PREDEF_CK);

	srx = 8 + (digit[3] * 25);
	if (digit[4] || digit[3])
		sry = 33;
	else
		sry = 22;
	oapiBlt(surf, digits, 10, 0, srx, sry, 9, 12, SURF_PREDEF_CK);

	srx = 8 + (digit[2] * 25);
	oapiBlt(surf, digits, 20, 0, srx, 33, 9, 12, SURF_PREDEF_CK);
	srx = 8 + (digit[1] * 25);
	oapiBlt(surf, digits, 30, 0, srx, 33, 9, 12, SURF_PREDEF_CK);
	srx = 8 + (digit[0] * 25);
	sry = (int)(digit[0] * 1.2);
	oapiBlt(surf, digits, 40, 0, srx, 33, 9, 12, SURF_PREDEF_CK);

	oapiColourFill(surf, oapiGetColour(255, 255, 255), 29, 5, 1, 2);
	return true;
}

void CMOptics::OpticsSwitchToggled()
{
	if (sat->OpticsZeroSwitch.IsUp())
	{
		sat->agc.SetInputChannelBit(07, ZeroOptics, true);
	}
	else
	{
		sat->agc.SetInputChannelBit(07, ZeroOptics, false);
	}
	if (sat->OpticsModeSwitch.IsUp() && sat->OpticsZeroSwitch.IsDown())
	{
		sat->agc.SetInputChannelBit(07, CMCControl, true);
	}
	else
	{
		sat->agc.SetInputChannelBit(07, CMCControl, false);
	}
}

void CMOptics::TimeStep(double simdt) {

	double ShaftRate = 0;
	double TrunRate = 0;

	SextDVTimer = SextDVTimer+simdt;
	if (SextDVTimer >= 0.06666){
		SextDVTimer = 0.0;
		SextDVLOSTog=!SextDVLOSTog;
	}

	// Optics cover handling
	if (OpticsCovered && sat->GetStage() >= STAGE_ORBIT_SIVB) {
		if (TeleShaft > 150. * RAD) {
			OpticsCovered = false;			
			sat->SetOpticsCoverMesh();
			sat->JettisonOpticsCover();
		}
	}

	if (Powered == 0) { return; }

	//Rates
	if (sat->OpticsZeroSwitch.IsUp())
	{
		//Optics zero speed is twice the angle, limit to max drive rate
		ShaftRate = min(abs(2.0*SextShaft), 19.5*RAD);
		TrunRate = min(abs(2.0*SextTrunion), 10.0*RAD);
	}
	else
	{
		// Generate rates for telescope and manual mode
		switch (sat->ControllerSpeedSwitch.GetState()) {
		case THREEPOSSWITCH_UP:       // HI
			ShaftRate = 19.5*RAD;
			TrunRate = 10.0*RAD;
			break;
		case THREEPOSSWITCH_CENTER:   // MED
			ShaftRate = 2.0*RAD;
			TrunRate = 1.0*RAD;
			break;
		case THREEPOSSWITCH_DOWN:     // LOW
			ShaftRate = 0.2*RAD;
			TrunRate = 0.1*RAD;
			break;
		}
	}

	dTrunion = 0.0;
	dShaft = 0.0;

	//ZERO OPTICS
	if (sat->OpticsZeroSwitch.IsUp())
	{
		if (SextShaft > 0)
		{
			dShaft = -ShaftRate * simdt;
		}
		else
		{
			dShaft = ShaftRate * simdt;
		}
		if (SextTrunion > 0)
		{
			dTrunion = -TrunRate * simdt;
		}
		else
		{
			dTrunion = TrunRate * simdt;
		}
	}
	else
	{
		// MANUAL
		if (sat->OpticsModeSwitch.IsDown())
		{
			double A_t_dot, A_s_dot;
			A_t_dot = 0.0;
			A_s_dot = 0.0;

			if ((OpticsManualMovement & 0x01) != 0) {
				A_t_dot = TrunRate * simdt;
			}
			if ((OpticsManualMovement & 0x02) != 0) {
				A_t_dot = -TrunRate * simdt;
			}
			if ((OpticsManualMovement & 0x04) != 0) {
				A_s_dot = -ShaftRate * simdt;
			}
			if ((OpticsManualMovement & 0x08) != 0) {
				A_s_dot = ShaftRate * simdt;
			}

			// DIRECT
			if (sat->ControllerCouplingSwitch.IsUp())
			{
				dShaft += A_s_dot;
				dTrunion += A_t_dot;
			}
			// RESOLVED
			else
			{
				dShaft += (A_s_dot*cos(SextShaft) - A_t_dot * sin(SextShaft)) / max(sin(10.0*RAD), sin(SextTrunion));
				dTrunion += A_s_dot * sin(SextShaft) + A_t_dot * cos(SextShaft);
			}
		}

		/*if (sat->agc.GetOutputChannelBit(012, DisengageOpticsDAC) == false)
		{
			//26mV per bit, 30.8 revolutions per second per volt, 1/3080 gear ratio (Shaft), 2/11780 gear ratio (Trunnion)
			dShaft += 0.026*30.8*PI2*1.0 / 3080.0*simdt*(double)sat->scdu.GetErrorCounter();
			dTrunion += 0.026*30.8*PI2*2.0 / 11780.0*simdt*(double)sat->tcdu.GetErrorCounter();
		}*/

		//sprintf(oapiDebugString(), "Trun Err: %lf Shaft Err: %lf", (double)sat->tcdu.GetErrorCounter()*180.0*pow(2, -14), (double)sat->scdu.GetErrorCounter()*180.0*pow(2, -12));
		//sprintf(oapiDebugString(), "Trun: %lf %d Shaft: %lf %d", dTrunion / simdt * DEG, sat->tcdu.GetErrorCounter(), dShaft / simdt * DEG, sat->scdu.GetErrorCounter());
	}

	SextShaft += dShaft;
	SextTrunion += dTrunion;

	//Limits
	if (SextShaft > 270.0*RAD)
	{
		SextShaft = 270.0*RAD;
	}
	if (SextShaft < -270.0*RAD)
	{
		SextShaft = -270.0*RAD;
	}
	if (SextTrunion < -59.0*RAD)
	{
		SextTrunion = -59.0*RAD;
	}
	if (SextTrunion > 59.0*RAD)
	{
		SextTrunion = 59.0*RAD;
	}

	sat->tcdu.SetReadCounter(SextTrunion * 4.0);
	sat->scdu.SetReadCounter(SextShaft);

	//sprintf(oapiDebugString(), "%d %d", sat->tcdu.GetErrorCounter(), sat->scdu.GetErrorCounter());

	// TELESCOPE TRUNNION MAINTENANCE (happens in all modes)
	// If the CMC issued pulses, they will have happened before we got here, so the sextant angle will be right.
	// If the order of timestep() calls is changed, this will "lag".

	double TeleTrunionTarget = 0;
	switch(sat->ControllerTelescopeTrunnionSwitch.GetState()){
		case THREEPOSSWITCH_UP:			// SLAVE TO SEXTANT			
			TeleTrunionTarget = SextTrunion;
			break;
		case THREEPOSSWITCH_CENTER:		// 0 DEG
			TeleTrunionTarget = 0;
			break;
		case THREEPOSSWITCH_DOWN:		// OFFSET 25 DEG
			TeleTrunionTarget = SextTrunion + 0.218166156; // Add 12.5 degrees to sextant angle
			break;
	}

	//Telescope Servo Drive
	TelescopeServoDrive(simdt, TeleTrunionTarget, TeleTrunion, TeleTrunionRate);
	TelescopeServoDrive(simdt, SextShaft, TeleShaft, TeleShaftRate);
	//sprintf(oapiDebugString(), "TA %lf %lf %lf SH %lf %lf %lf", TeleTrunionTarget*DEG, TeleTrunion*DEG, TeleTrunionRate*DEG, SextShaft*DEG, TeleShaft*DEG, TeleShaftRate*DEG);

	//Limits
	if (TeleShaft > 270.0*RAD)
	{
		TeleShaft = 270.0*RAD;
		TeleShaftRate = 0.0;
	}
	if (TeleShaft < -270.0*RAD)
	{
		TeleShaft = -270.0*RAD;
		TeleShaftRate = 0.0;
	}
	if (TeleTrunion < -59.0*RAD)
	{
		TeleTrunion = -59.0*RAD;
		TeleTrunionRate = 0.0;
	}
	if (TeleTrunion > 59.0*RAD)
	{
		TeleTrunion = 59.0*RAD;
		TeleTrunionRate = 0.0;
	}

	//sprintf(oapiDebugString(), "Optics Shaft %.2f, Sext Trunion %.2f, Tele Trunion %.2f", OpticsShaft/RAD, SextTrunion/RAD, TeleTrunion/RAD);
	//sprintf(oapiDebugString(), "Sext Trunion EMEM %o", sat->agc.vagc.Erasable[0][RegOPTY]);
}

void CMOptics::TelescopeServoDrive(double dt, double sxt_angle, double &sct_angle, double &sct_rate)
{
	//Direct solution of the transfer function in the Apollo 15 Delco manual
	double C1, C2, TEMP1, TEMP2, TEMP3;
	static const double SCT_SERVO_CONST1 = 1.98673;
	static const double SCT_SERVO_CONST2 = 1.77;

	C2 = sct_angle - sxt_angle;
	C1 = (sct_rate + SCT_SERVO_CONST2 * C2) / SCT_SERVO_CONST1;
	TEMP3 = exp(-SCT_SERVO_CONST2 * dt);
	TEMP1 = TEMP3 * sin(SCT_SERVO_CONST1*dt);
	TEMP2 = TEMP3 * cos(SCT_SERVO_CONST1*dt);

	sct_rate = -SCT_SERVO_CONST2 * C1*TEMP1 + SCT_SERVO_CONST1 * C1*TEMP2 - SCT_SERVO_CONST1 * C2*TEMP1 - SCT_SERVO_CONST2 * C2*TEMP2;
	sct_angle = sxt_angle + C1 * TEMP1 + C2 * TEMP2;
}

void CMOptics::SaveState(FILEHANDLE scn) {

	oapiWriteLine(scn, CMOPTICS_START_STRING);
	oapiWriteScenario_int(scn, "POWERED", Powered);
	oapiWriteScenario_int(scn, "OPTICSMANUALMOVEMENT", OpticsManualMovement);
	papiWriteScenario_double(scn, "OPTICSSHAFT", SextShaft); //Keep it named OPTICSSHAFT for backwards compatibility
	papiWriteScenario_double(scn, "SEXTTRUNION", SextTrunion);
	papiWriteScenario_double(scn, "TELESHAFT", TeleShaft);
	papiWriteScenario_double(scn, "TELETRUNION", TeleTrunion);
	papiWriteScenario_bool(scn, "OPTICSCOVERED", OpticsCovered); 
	oapiWriteLine(scn, CMOPTICS_END_STRING);
}

void CMOptics::LoadState(FILEHANDLE scn) {

	char *line;

	while (oapiReadScenario_nextline (scn, line)) {
		if (!strnicmp(line, CMOPTICS_END_STRING, sizeof(CMOPTICS_END_STRING)))
			return;
		else if (!strnicmp (line, "POWERED", 7)) {
			sscanf (line+7, "%d", &Powered);
		}
		else if (!strnicmp (line, "OPTICSMANUALMOVEMENT", 20)) {
			sscanf (line+20, "%d", &OpticsManualMovement);
		}
		else if (!strnicmp (line, "OPTICSSHAFT", 11)) {
			sscanf (line+11, "%lf", &SextShaft);
		}
		else if (!strnicmp (line, "SEXTTRUNION", 11)) {
			sscanf (line+11, "%lf", &SextTrunion);
		}
		else if (!strnicmp(line, "TELESHAFT", 9)) {
			sscanf(line + 9, "%lf", &TeleShaft);
		}
		else if (!strnicmp (line, "TELETRUNION", 11)) {
			sscanf (line+11, "%lf", &TeleTrunion);
		}
		papiReadScenario_bool(line, "OPTICSCOVERED", OpticsCovered); 
	}
}
