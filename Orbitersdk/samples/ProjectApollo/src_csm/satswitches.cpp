/***************************************************************************
  This file is part of Project Apollo - NASSP
  Copyright 2004-2005

  ORBITER vessel module: Saturn-specific switches

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
#include <stdio.h>
#include <math.h>
#include "soundlib.h"

#include "resource.h"

#include "nasspdefs.h"
#include "nasspsound.h"

#include "toggleswitch.h"
#include "apolloguidance.h"
#include "csmcomputer.h"
#include "ioChannels.h"

#include "saturn.h"

#include "CM_VC_Resource.h"

extern GDIParams g_Param;

void SaturnToggleSwitch::Init(int xp, int yp, int w, int h, SURFHANDLE surf, SURFHANDLE bsurf, SwitchRow &row, Saturn *s, int xoffset, int yoffset)
{
	ToggleSwitch::Init(xp, yp, w, h, surf, bsurf, row, xoffset, yoffset);
	sat = s;
}

bool XLunarSwitch::SwitchTo(int newState, bool dontspring)
{
	if (ToggleSwitch::SwitchTo(newState,dontspring)) {
		// Do nothing, handling is done in the IU
		return true;
	}
	return false;
}

void SaturnThreePosSwitch::Init(int xp, int yp, int w, int h, SURFHANDLE surf, SURFHANDLE bsurf, SwitchRow &row, Saturn *s)
{
	ThreePosSwitch::Init(xp, yp, w, h, surf, bsurf, row);
	sat = s;
}


SaturnRCSValveTalkback::SaturnRCSValveTalkback()
{
	valve = 0;
}


void SaturnRCSValveTalkback::Init(int xp, int yp, int w, int h, SURFHANDLE surf, SwitchRow &row, RCSValve *v, bool failopen)
{
	IndicatorSwitch::Init(xp, yp, w, h, surf, row, failopen);
	valve = v;	
}

int SaturnRCSValveTalkback::GetState()
{
	if (valve && SRC && (SRC->Voltage() > SP_MIN_DCVOLTAGE))
		state = valve->IsOpen() ? 1 : 0;
	else 
		// SM RCS helium and prim. propellant talkbacks fail open
		state = (failOpen ? 1 : 0);

	return state;
}


void SaturnH2PressureMeter::Init(int i, SURFHANDLE surf, SwitchRow &row, Saturn *s)
{
	MeterSwitch::Init(row);
	Index = i;
	NeedleSurface = surf;
	Sat = s;
}

double SaturnH2PressureMeter::QueryValue()
{
	TankPressures press;
	Sat->GetTankPressures(press);

	if (Index == 1) 
		return press.H2Tank1PressurePSI;
	else
		return press.H2Tank2PressurePSI;
}

void SaturnH2PressureMeter::CalculateNeedleState()
{
	state = 130 - (int)(GetDisplayValue() / 400.0 * 104.0);
}

void SaturnH2PressureMeter::DoDrawSwitch(SURFHANDLE drawSurface)
{
	if (Index == 1) 
		oapiBlt(drawSurface, NeedleSurface, 0, state, 0, 0, 10, 10, SURF_PREDEF_CK);
	else
		oapiBlt(drawSurface, NeedleSurface, 0, state, 10, 0, 10, 10, SURF_PREDEF_CK);
}

void SaturnO2PressureMeter::Init(int i, SURFHANDLE surf, SwitchRow &row, Saturn *s, ToggleSwitch *o2PressIndSwitch)
{
	MeterSwitch::Init(row);
	Index = i;
	NeedleSurface = surf;
	Sat = s;
	O2PressIndSwitch = o2PressIndSwitch;
}

double SaturnO2PressureMeter::QueryValue()
{
	TankPressures press;
	Sat->GetTankPressures(press);

	if (Index == 1)
		if (O2PressIndSwitch->IsUp())
			return press.O2Tank1PressurePSI;
		else
			return Sat->O2SurgeTankPressSensor.Voltage()*200.0 + 50.0;
	else
		return press.O2Tank2PressurePSI;
}

void SaturnO2PressureMeter::CalculateNeedleState()
{
	double v = GetDisplayValue();

	if (v < 100.0)
		state = 130;
	else if (v <= 500.0)
		state = 130 - (int)((v - 100.0) * 0.065);
	else if (v <= 850.0)
		state = 104 - (int)((v - 500.0) * 0.07714);
	else if (v <= 900.0)
		state = 77 - (int)((v - 850.0) * 0.38);
	else if (v <= 950.0)
		state = 58 - (int)((v - 900.0) * 0.42);
	else if (v <= 1050.0)
		state = 37 - (int)((v - 950.0) * 0.13);
	else
		state = 24;
}

void SaturnO2PressureMeter::DoDrawSwitch(SURFHANDLE drawSurface)
{
	if (Index == 1) 
		DoDrawSwitch(drawSurface, NeedleSurface, 0);
	else
		DoDrawSwitch(drawSurface, NeedleSurface, 10);
}

void SaturnO2PressureMeter::DoDrawSwitch(SURFHANDLE surf, SURFHANDLE needle, int xNeedle)
{
	oapiBlt(surf, needle, 0, state, xNeedle, 0, 10, 10, SURF_PREDEF_CK);
}

void SaturnCryoQuantityMeter::Init(char *sub, int i, SURFHANDLE surf, SwitchRow &row, Saturn *s)
{
	MeterSwitch::Init(row);
	Substance = sub;
	Index = i;
	NeedleSurface = surf;
	Sat = s;
}

double SaturnCryoQuantityMeter::QueryValue()
{
	TankQuantities q;
	Sat->GetTankQuantities(q);

	if (!strcmp("H2", Substance)) {
		if (Index == 1) 
			return q.H2Tank1Quantity;
		else
			return q.H2Tank2Quantity;
	} else {
		if (Index == 1) 
			return q.O2Tank1Quantity;
		else
			return q.O2Tank2Quantity;
	}
}

void SaturnCryoQuantityMeter::DoDrawSwitch(SURFHANDLE drawSurface)
{
	if (Index == 1)
		oapiBlt(drawSurface, NeedleSurface, 0, state, 0, 0, 10, 10, SURF_PREDEF_CK);
	else
		oapiBlt(drawSurface, NeedleSurface, 0, state, 10, 0, 10, 10, SURF_PREDEF_CK);
}

bool SaturnCryoQuantityMeter::DrawSwitch2(int ID, SURFHANDLE DrawSurface, bool FlashOn)
{
	double v = GetDisplayValue();

	//
	// Apollo 13 O2 tank 2 quantity display failed offscale high around 46:45.
	//
	if (Sat->GetApolloNo() == 1301 && !strcmp("H2", Substance) && Index == 2)
	{
#define O2FAILURETIME	(46.0 * 3600.0 + 45.0 * 60.0)

		if (Sat->GetMissionTime() >= (O2FAILURETIME + 5.0)) {
			v = 1.05;
		}
		else if (Sat->GetMissionTime() >= O2FAILURETIME) {
			v += (1.05 - value) * ((Sat->GetMissionTime() - O2FAILURETIME) / 5.0);
		}
	}

	state = 130 - (int)(v * 104.0);

	if (state != DisplayState)
	{
		oapiBltPanelAreaBackground(ID, DrawSurface);
		DoDrawSwitch(DrawSurface);
		DisplayState = state;
		return true;
	}
	return false;
}

RCSQuantityMeter::RCSQuantityMeter()
{
	source = 0;
	SMRCSIndSwitch = 0;
	NeedleSurface = 0;
}

void RCSQuantityMeter::Init(SURFHANDLE surf, SwitchRow &row, PropellantRotationalSwitch *s, ToggleSwitch *indswitch)
{
	MeterSwitch::Init(row);
	source = s;
	SMRCSIndSwitch = indswitch;
	NeedleSurface = surf;
}

double RCSQuantityMeter::QueryValue()
{
	if (!source) return 0.0;

	// only SM data are available/displayed
	SMRCSPropellantSource *ps = source->GetSMSource();
	if (!ps) return 0;

	if (SMRCSIndSwitch->IsDown()) {		
		return ps->GetPropellantQuantityToDisplay();
	}
	return ps->GetHeliumTempF() / 100.;
}

void RCSQuantityMeter::CalculateNeedleState()
{
	state = 108 - (int)(GetDisplayValue() * 104.0);
}

void RCSQuantityMeter::DoDrawSwitch(SURFHANDLE drawSurface)
{
	oapiBlt(drawSurface, NeedleSurface,  0, state, 10, 0, 10, 10, SURF_PREDEF_CK);
}

RCSFuelPressMeter::RCSFuelPressMeter()
{
	source = 0;
	NeedleSurface = 0;
}

void RCSFuelPressMeter::Init(SURFHANDLE surf, SwitchRow &row, PropellantRotationalSwitch *s)
{
	MeterSwitch::Init(row);
	source = s;
	NeedleSurface = surf;
}

double RCSFuelPressMeter::QueryValue()
{
	if (!source) return 0;

	SMRCSPropellantSource *ps = source->GetSMSource();
	if (ps) return ps->GetPropellantPressurePSI();

	CMRCSPropellantSource *cmps = source->GetCMSource();
	if (cmps) return cmps->GetPropellantPressurePSI();

	return 0;
}

void RCSFuelPressMeter::CalculateNeedleState()
{
	state = 108 - (int)(GetDisplayValue() / 400.0 * 104.0);
}

void RCSFuelPressMeter::DoDrawSwitch(SURFHANDLE drawSurface)
{
	oapiBlt(drawSurface, NeedleSurface, 0, state, 0, 0, 10, 10, SURF_PREDEF_CK);
}

RCSHeliumPressMeter::RCSHeliumPressMeter()
{
	source = 0;
	NeedleSurface = 0;
}

void RCSHeliumPressMeter::Init(SURFHANDLE surf, SwitchRow &row, PropellantRotationalSwitch *s)
{
	MeterSwitch::Init(row);
	source = s;
	NeedleSurface = surf;
}

double RCSHeliumPressMeter::QueryValue()
{
	if (!source) return 0;

	SMRCSPropellantSource *ps = source->GetSMSource();
	if (ps) return ps->GetHeliumPressurePSI();

	CMRCSPropellantSource *cmps = source->GetCMSource();
	if (cmps) return cmps->GetHeliumPressurePSI();

	return 0;
}

void RCSHeliumPressMeter::CalculateNeedleState()
{
	state = 108 - (int)(GetDisplayValue() / 5000.0 * 104.0);
}

void RCSHeliumPressMeter::DoDrawSwitch(SURFHANDLE drawSurface)
{
	oapiBlt(drawSurface, NeedleSurface, 0, state, 10, 0, 10, 10, SURF_PREDEF_CK);
}

RCSTempMeter::RCSTempMeter()
{
	source = 0;
	NeedleSurface = 0;
}

void RCSTempMeter::Init(SURFHANDLE surf, SwitchRow &row, PropellantRotationalSwitch *s)
{
	MeterSwitch::Init(row);
	source = s;
	NeedleSurface = surf;
}

double RCSTempMeter::QueryValue()
{
	if (!source) return 0;

	SMRCSPropellantSource *ps = source->GetSMSource();
	if (ps) return ps->GetPackageTempF();

	CMRCSPropellantSource *cmps = source->GetCMSource();
	if (cmps) return cmps->GetHeliumTempF();

	return 0;
}

void RCSTempMeter::CalculateNeedleState()
{
	state = 108 - (int)(GetDisplayValue() / 300.0 * 104.0);
}

void RCSTempMeter::DoDrawSwitch(SURFHANDLE drawSurface)
{
	oapiBlt(drawSurface, NeedleSurface, 0, state, 0, 0, 10, 10, SURF_PREDEF_CK);
}

PropellantSource::PropellantSource(PROPELLANT_HANDLE &h) : source_prop(h)
{
	our_vessel = 0;
}

PROPELLANT_HANDLE PropellantSource::Handle()
{
	return source_prop;
}

double PropellantSource::Quantity()
{
	if (source_prop && our_vessel) {
		return our_vessel->GetPropellantMass(source_prop) / our_vessel->GetPropellantMaxMass(source_prop);
	}

	return 0.0;
}


PropellantRotationalSwitch::PropellantRotationalSwitch()
{
	int i;

	for (i = 0; i < 7; i++) {
		CMSources[i] = 0;
		SMSources[i] = 0;
	}
}

void PropellantRotationalSwitch::SetCMSource(int num, CMRCSPropellantSource *s)
{
	if (num >= 0 && num < 7)
		CMSources[num] = s; 
}

void PropellantRotationalSwitch::SetSMSource(int num, SMRCSPropellantSource *s)
{
	if (num >= 0 && num < 7)
		SMSources[num] = s; 
}

CMRCSPropellantSource *PropellantRotationalSwitch::GetCMSource()
{
	return CMSources[GetState()];
}

SMRCSPropellantSource *PropellantRotationalSwitch::GetSMSource()
{
	return SMSources[GetState()];
}


void SaturnFuelCellMeter::Init(SURFHANDLE surf, SwitchRow &row, Saturn *s, RotationalSwitch *fuelCellIndicatorsSwitch)
{
	MeterSwitch::Init(row);
	NeedleSurface = surf;
	Sat = s;
	FuelCellIndicatorsSwitch = fuelCellIndicatorsSwitch;
}


double SaturnFuelCellH2FlowMeter::QueryValue()
{
	FuelCellStatus fc;
	Sat->GetFuelCellStatus(FuelCellIndicatorsSwitch->GetState() + 1, fc);

	return fc.H2FlowLBH; 
}

void SaturnFuelCellH2FlowMeter::CalculateNeedleState()
{
	double v = GetDisplayValue();

	if (v < 0.05)
		state = 111 - (int)(v / 0.05 * 21.0);
	else if (v < 0.15)
		state = 90 - (int)((v - 0.05) / 0.1 * 65.0);
	else
		state = 25 - (int)((v - 0.15) / 0.05 * 21.0);
}

void SaturnFuelCellH2FlowMeter::DoDrawSwitch(SURFHANDLE drawSurface)
{
	oapiBlt(drawSurface, NeedleSurface, 0, state, 0, 0, 10, 10, SURF_PREDEF_CK);
}

double SaturnFuelCellO2FlowMeter::QueryValue()
{
	FuelCellStatus fc;
	Sat->GetFuelCellStatus(FuelCellIndicatorsSwitch->GetState() + 1, fc);

	return fc.O2FlowLBH; 
}

void SaturnFuelCellO2FlowMeter::DoDrawSwitch(SURFHANDLE drawSurface)
{
	oapiBlt(drawSurface, NeedleSurface, 0, state, 10, 0, 10, 10, SURF_PREDEF_CK);
}

void SaturnFuelCellO2FlowMeter::CalculateNeedleState()
{
	double v = GetDisplayValue();

	if (v < 0.4)
		state = 111 - (int)(v / 0.4 * 21.0);
	else if (v < 1.2)
		state = 90 - (int)((v - 0.4) / 0.8 * 65.0);
	else
		state = 25 - (int)((v - 1.2) / 0.4 * 21.0);
}

double SaturnFuelCellTempMeter::QueryValue()
{
	return (Sat->GetSCE()->GetVoltage(2, FuelCellIndicatorsSwitch->GetState() + 6)*94.0 + 80.0);
}

void SaturnFuelCellTempMeter::CalculateNeedleState()
{
	double v = GetDisplayValue();
	if (v < 400.0)
		state = 109 - (int)((v - 100.0) / 300.0 * 53.0);
	else if (v < 500.0)
		state = 56 - (int)((v - 400.0) / 100.0 * 40.0);
	else
		state = 16 - (int)((v - 500.0) / 50.0 * 12.0);
}

void SaturnFuelCellTempMeter::DoDrawSwitch(SURFHANDLE drawSurface)
{
	oapiBlt(drawSurface, NeedleSurface, 0, state, 0, 0, 10, 10, SURF_PREDEF_CK);
}

double SaturnFuelCellCondenserTempMeter::QueryValue()
{
	return (Sat->GetSCE()->GetVoltage(2, FuelCellIndicatorsSwitch->GetState() + 3)*21.0 + 145.0);
}

void SaturnFuelCellCondenserTempMeter::DoDrawSwitch(SURFHANDLE drawSurface)
{
	oapiBlt(drawSurface, NeedleSurface, 0, state, 10, 0, 10, 10, SURF_PREDEF_CK);
}

void SaturnFuelCellCondenserTempMeter::CalculateNeedleState()
{
	double v = GetDisplayValue();
	state = 109 - (int)((v - 150.0) / 100.0 * 103.0);
}

void SaturnCabinMeter::Init(SURFHANDLE surf, SwitchRow &row, Saturn *s)
{
	MeterSwitch::Init(row);
	NeedleSurface = surf;
	Sat = s;
}


double SaturnSuitTempMeter::QueryValue()
{
	return Sat->SuitTempSensor.Voltage()*15.0 + 20.0;
}

void SaturnSuitTempMeter::CalculateNeedleState()
{
	double v = GetDisplayValue();
	state = 110 - (int)((v - 20.0) / 75.0 * 104.0);
}

void SaturnSuitTempMeter::DoDrawSwitch(SURFHANDLE drawSurface)
{
	oapiBlt(drawSurface, NeedleSurface, 0, state, 0, 0, 10, 10, SURF_PREDEF_CK);
}

double SaturnCabinTempMeter::QueryValue()
{
	return Sat->CabinTempSensor.Voltage()*17.0 + 40.0;
}

void SaturnCabinTempMeter::CalculateNeedleState()
{
	double v = GetDisplayValue();
	state = 110 - (int)((v - 40.0) / 80.0 * 104.0);
}

void SaturnCabinTempMeter::DoDrawSwitch(SURFHANDLE drawSurface)
{
	oapiBlt(drawSurface, NeedleSurface, 0, state, 10, 0, 10, 10, SURF_PREDEF_CK);
}

double SaturnSuitPressMeter::QueryValue()
{
	return Sat->SuitPressSensor.Voltage()*3.4;
}

void SaturnSuitPressMeter::CalculateNeedleState()
{
	double v = GetDisplayValue();

	if (v < 6.0)
		state = 108 - (int)(v / 6.0 * 55.0);
	else
		state = 53 - (int)((v - 6.0) / 10.0 * 45.0);
}

void SaturnSuitPressMeter::DoDrawSwitch(SURFHANDLE drawSurface)
{
	oapiBlt(drawSurface, NeedleSurface, 0, state, 0, 0, 10, 10, SURF_PREDEF_CK);
}

void SaturnSuitPressMeter::OnPostStep(double SimT, double DeltaT, double MJD) {

	double v = GetDisplayValue();

	if (v < 6.0) {
		OurVessel->SetAnimation(anim_switch, v / 11.0);
	}
	else {
		OurVessel->SetAnimation(anim_switch, (v / 22.0) + (3.0 / 11.0));
	}
}

double SaturnCabinPressMeter::QueryValue()
{
	return Sat->CabinPressSensor.Voltage()*3.4;
}

void SaturnCabinPressMeter::CalculateNeedleState()
{
	double v = GetDisplayValue();

	if (v < 6.0)
		state = 108 - (int)(v / 6.0 * 55.0);
	else
		state = 53 - (int)((v - 6.0) / 10.0 * 45.0);
}

void SaturnCabinPressMeter::DoDrawSwitch(SURFHANDLE drawSurface)
{
	oapiBlt(drawSurface, NeedleSurface, 0, state, 10, 0, 10, 10, SURF_PREDEF_CK);
}

void SaturnCabinPressMeter::OnPostStep(double SimT, double DeltaT, double MJD) {

	double v = GetDisplayValue();

	if (v < 6.0) {
		OurVessel->SetAnimation(anim_switch, v / 11.0);
	}
	else {
		OurVessel->SetAnimation(anim_switch, (v / 22.0) + (3.0 / 11.0));
	}
}

double SaturnPartPressCO2Meter::QueryValue()
{
	return pow(Sat->CO2PartPressSensor.Voltage(), 2)*30.0 / 25.0;
}

void SaturnPartPressCO2Meter::CalculateNeedleState()
{
	double v = GetDisplayValue();

	if (v < 10.0)
		state = 109 - (int)(v / 10.0 * 55.0);
	else if (v < 15.0)
		state = 54 - (int)((v - 10.0) / 5.0 * 19.0);
	else if (v < 20.0)
		state = 35 - (int)((v - 15.0) / 5.0 * 15.0);
	else
		state = 20 - (int)((v - 20.0) / 10.0 * 14.0);
}

void SaturnPartPressCO2Meter::DoDrawSwitch(SURFHANDLE drawSurface)
{
	oapiBlt(drawSurface, NeedleSurface, 0, state, 10, 0, 10, 10, SURF_PREDEF_CK);
}

void SaturnRoundMeter::Init(HPEN p0, HPEN p1, SwitchRow &row, Saturn *s)
{
	RoundMeter::Init(p0, p1, row);
	Sat = s;
}

double SaturnSuitComprDeltaPMeter::QueryValue()
{
	return Sat->SuitCompressorDeltaPSensor.Voltage()*1.0 / 5.0;
}

void SaturnSuitComprDeltaPMeter::DoDrawSwitch(double v, SURFHANDLE drawSurface)
{
	v = (v - .5) / .5 * 60.0;
	DrawNeedle(drawSurface, 45, 22, 20.0, (180.0 - v) * RAD);
}


double SaturnLeftO2FlowMeter::QueryValue()
{
	return Sat->ECSO2FlowO2SupplyManifoldSensor.Voltage()*0.16 + 0.2;
}

void SaturnLeftO2FlowMeter::DoDrawSwitch(double v, SURFHANDLE drawSurface)
{
	v = (v - .6) / .4 * 60.0;	
	DrawNeedle(drawSurface, 0, 22, 20.0, v * RAD);
}


double SaturnSuitCabinDeltaPMeter::QueryValue()
{
	return Sat->SuitCabinDeltaPressSensor.Voltage()*2.0 - 5.0;
}

void SaturnSuitCabinDeltaPMeter::DoDrawSwitch(double v, SURFHANDLE drawSurface)
{
	v = (v / 5.0) * 60.0;
	DrawNeedle(drawSurface, 0, 22, 20.0, v * RAD);
	oapiBlt(drawSurface, FrameSurface, 0, 13, 0, 0, 46, 18, SURF_PREDEF_CK);
}


double SaturnRightO2FlowMeter::QueryValue()
{
	return Sat->ECSO2FlowO2SupplyManifoldSensor.Voltage()*0.16 + 0.2;
}

void SaturnRightO2FlowMeter::DoDrawSwitch(double v, SURFHANDLE drawSurface)
{
	v = (v - .6) / .4 * 60.0;
	DrawNeedle(drawSurface, 45, 22, 20.0, (180.0 - v) * RAD);
	oapiBlt(drawSurface, FrameSurface, 0, 13, 0, 0, 46, 18, SURF_PREDEF_CK);
}


void SaturnEcsRadTempInletMeter::Init(HPEN p0, HPEN p1, SwitchRow &row, Saturn *s, RotationalSwitch *ecsindicatorsswitch)
{
	SaturnRoundMeter::Init(p0, p1, row, s);
	ECSIndicatorsSwitch = ecsindicatorsswitch;
}

double SaturnEcsRadTempInletMeter::QueryValue()
{
	if (ECSIndicatorsSwitch->GetState() == 0) {
		return Sat->PriRadInTempSensor.Voltage()*13.0 + 55.0;
	} else {
		return Sat->SecRadInTempSensor.Voltage()*13.0 + 55.0;
	}
}

void SaturnEcsRadTempInletMeter::DoDrawSwitch(double v, SURFHANDLE drawSurface)
{
	v = (v - 90.0) / 30.0 * 60.0;	
	DrawNeedle(drawSurface, 0, 22, 20.0, v * RAD);
}


double SaturnEcsRadTempPrimOutletMeter::QueryValue()
{
	return Sat->ECSRadOutTempSensor.Voltage()*30.0 - 50.0;
}

void SaturnEcsRadTempPrimOutletMeter::DoDrawSwitch(double v, SURFHANDLE drawSurface)
{
	v = (v - 25.0) / 75.0 * 60.0;	
	DrawNeedle(drawSurface, 45, 22, 20.0, (180.0 - v) * RAD);
}


double SaturnEcsRadTempSecOutletMeter::QueryValue()
{
	return Sat->SecRadOutTempSensor.Voltage()*8.0 + 30.0;
}

void SaturnEcsRadTempSecOutletMeter::DoDrawSwitch(double v, SURFHANDLE drawSurface)
{
	v = (v - 50.0) / 20.0 * 60.0;	
	DrawNeedle(drawSurface, 0, 22, 20.0, v * RAD);
}


void SaturnGlyEvapTempOutletMeter::Init(HPEN p0, HPEN p1, SwitchRow &row, Saturn *s, RotationalSwitch *ecsindicatorsswitch)
{
	SaturnRoundMeter::Init(p0, p1, row, s);
	ECSIndicatorsSwitch = ecsindicatorsswitch;
}

double SaturnGlyEvapTempOutletMeter::QueryValue()
{
	if (ECSIndicatorsSwitch->GetState() == 0) {
		return Sat->GlyEvapOutTempSensor.Voltage()*10.0 + 25.0;
	} else {
		return Sat->SecEvapOutLiqTempSensor.Voltage()*10.0 + 25.0;
	}
}

void SaturnGlyEvapTempOutletMeter::DoDrawSwitch(double v, SURFHANDLE drawSurface)
{
	v = (v - 50.0) / 20.0 * 60.0;	
	DrawNeedle(drawSurface, 45, 22, 20.0, (180.0 - v) * RAD);
}


void SaturnGlyEvapSteamPressMeter::Init(HPEN p0, HPEN p1, SwitchRow &row, Saturn *s, RotationalSwitch *ecsindicatorsswitch)
{
	SaturnRoundMeter::Init(p0, p1, row, s);
	ECSIndicatorsSwitch = ecsindicatorsswitch;
}

double SaturnGlyEvapSteamPressMeter::QueryValue()
{
	if (ECSIndicatorsSwitch->GetState() == 0) {
		return Sat->GlyEvapBackPressSensor.Voltage()*0.04 + 0.05; 
	} else {
		return Sat->SecEvapOutSteamPressSensor.Voltage()*0.04 + 0.05;
	}
}

void SaturnGlyEvapSteamPressMeter::DoDrawSwitch(double v, SURFHANDLE drawSurface)
{
	v = (v - 0.15) / 0.1 * 60.0;	
	DrawNeedle(drawSurface, 0, 22, 20.0, v * RAD);
}


void SaturnGlycolDischPressMeter::Init(HPEN p0, HPEN p1, SwitchRow &row, Saturn *s, RotationalSwitch *ecsindicatorsswitch)
{
	SaturnRoundMeter::Init(p0, p1, row, s);
	ECSIndicatorsSwitch = ecsindicatorsswitch;
}

double SaturnGlycolDischPressMeter::QueryValue()
{
	if (ECSIndicatorsSwitch->GetState() == 0) {
		return Sat->GlycolPumpOutPressSensor.Voltage()*12.0;
	} else {
		return Sat->SecGlyPumpOutPressSensor.Voltage()*12.0;
	}
}

void SaturnGlycolDischPressMeter::DoDrawSwitch(double v, SURFHANDLE drawSurface)
{
	v = (v - 30.0) / 30.0 * 60.0;	
	DrawNeedle(drawSurface, 45, 22, 20.0, (180.0 - v) * RAD);
}


void SaturnAccumQuantityMeter::Init(HPEN p0, HPEN p1, SwitchRow &row, Saturn *s, RotationalSwitch *ecsindicatorsswitch)
{
	SaturnRoundMeter::Init(p0, p1, row, s);
	ECSIndicatorsSwitch = ecsindicatorsswitch;
}

double SaturnAccumQuantityMeter::QueryValue()
{
	if (ECSIndicatorsSwitch->GetState() == 0) {
		return Sat->GlycolAccumQtySensor.Voltage()*1.0 / 5.0;
	} else {
		return Sat->SecGlycolAccumQtySensor.Voltage()*1.0 / 5.0;
	}
}

void SaturnAccumQuantityMeter::DoDrawSwitch(double v, SURFHANDLE drawSurface)
{
	v = (v - 0.5) / 0.5 * 60.0;	
	DrawNeedle(drawSurface, 0, 22, 20.0, v * RAD);
}


void SaturnH2oQuantityMeter::Init(HPEN p0, HPEN p1, SwitchRow &row, Saturn *s, ToggleSwitch *h2oqtyindswitch, PowerSource *pwr)
{
	SaturnRoundMeter::Init(p0, p1, row, s);
	H2oQtyIndSwitch = h2oqtyindswitch;
	WireTo(pwr);
}

double SaturnH2oQuantityMeter::QueryValue()
{
	if (H2oQtyIndSwitch->IsUp())
		return Sat->PotH2OQtySensor.Voltage()*1.0 / 5.0;
	else
		return Sat->WasteH2OQtySensor.Voltage()*1.0 / 5.0;
}

void SaturnH2oQuantityMeter::DoDrawSwitch(double v, SURFHANDLE drawSurface)
{
	v = (v - 0.5) / 0.5 * 60.0;	
	DrawNeedle(drawSurface, 45, 22, 20.0, (180.0 - v) * RAD);
}


double SaturnAccelGMeter::QueryValue()
{
	return Sat->GetAccelG();
}

void SaturnAccelGMeter::DoDrawSwitch(double v, SURFHANDLE drawSurface)
{
	v = (-v * 180.0 / 12.0) + 180.0;
	DrawNeedle (drawSurface, 40, 40, 35.0, v * RAD);
}


void DirectO2RotationalSwitch::Init(int xp, int yp, int w, int h, SURFHANDLE surf, SURFHANDLE bsurf, SwitchRow &row, h_Pipe *p)
{
	RotationalSwitch::Init(xp, yp, w, h, surf, bsurf, row);
	Pipe = p;
}

/*bool DirectO2RotationalSwitch::CheckMouseClick(int event, int mx, int my)
{
	if (RotationalSwitch::CheckMouseClick(event, mx, my)) {
		CheckValve();
		return true;
	}
	return false;
}*/

bool DirectO2RotationalSwitch::SwitchTo(int newValue)
{
	if (RotationalSwitch::SwitchTo(newValue)) {
		CheckValve();
		return true;
	}
	return false;
}
/*
void DirectO2RotationalSwitch::CheckValve()
{
	if (GetState() == 6) {
		Pipe->in->h_open = SP_VALVE_CLOSE;
		Pipe->flowMax = 0;
	
	} else if (GetState() == 5) {
		Pipe->in->h_open = SP_VALVE_OPEN;
		Pipe->flowMax = 6.0 / LBH;		//0.1 lb/min

	} else if (GetState() == 4) {
		Pipe->in->h_open = SP_VALVE_OPEN;
		Pipe->flowMax = 12.0 / LBH;		//0.2 lb/min

	} else if (GetState() == 3) {
		Pipe->in->h_open = SP_VALVE_OPEN;
		Pipe->flowMax = 18.6 / LBH;		//0.31 lb/min

	} else if (GetState() == 2) {
		Pipe->in->h_open = SP_VALVE_OPEN;
		Pipe->flowMax = 24.6 / LBH;		//0.41 lb/min

	} else if (GetState() == 1) {
		Pipe->in->h_open = SP_VALVE_OPEN;
		Pipe->flowMax = 31.8 / LBH;		//0.53 lb/min

	} else if (GetState() == 0) {
		Pipe->in->h_open = SP_VALVE_OPEN;
		Pipe->flowMax = 40.2 / LBH;		//0.67 lb/min
	}
}
*/

void DirectO2RotationalSwitch::CheckValve()

{
	if (GetState() == 6) {
		Pipe->in->h_open = SP_VALVE_CLOSE;
		Pipe->flowMax = 0;

	}
	else if (GetState() == 5) {
		Pipe->in->h_open = SP_VALVE_OPEN;
		Pipe->flowMax = 0.42 / LBH;  //0.007 lb/min

	}
	else if (GetState() == 4) {
		Pipe->in->h_open = SP_VALVE_OPEN;
		Pipe->flowMax = 0.78 / LBH;  //0.013 lb/min

	}
	else if (GetState() == 3) {
		Pipe->in->h_open = SP_VALVE_OPEN;
		Pipe->flowMax = 1.56 / LBH;  //0.026 lb/min

	}
	else if (GetState() == 2) {
		Pipe->in->h_open = SP_VALVE_OPEN;
		Pipe->flowMax = 24.6 / LBH;		//0.41 lb/min

	}
	else if (GetState() == 1) {
		Pipe->in->h_open = SP_VALVE_OPEN;
		Pipe->flowMax = 31.8 / LBH;		//0.53 lb/min

	}
	else if (GetState() == 0) {
		Pipe->in->h_open = SP_VALVE_OPEN;
		Pipe->flowMax = 40.2 / LBH;		//0.67 lb/min
	}
}


void SaturnEcsGlycolPumpsSwitch::Init(int xp, int yp, int w, int h, SURFHANDLE surf, SURFHANDLE bsurf, SwitchRow &row, Pump *p,
									  CircuitBrakerSwitch* ac1a, CircuitBrakerSwitch* ac1b, CircuitBrakerSwitch* ac1c,
									  CircuitBrakerSwitch* ac2a, CircuitBrakerSwitch* ac2b, CircuitBrakerSwitch* ac2c)
{
	RotationalSwitch::Init(xp, yp, w, h, surf, bsurf, row);
	GlycolPump = p;
	ACBus1.WireToBuses(ac1a, ac1b, ac1c);
	ACBus2.WireToBuses(ac2a, ac2b, ac2c);
	CheckPump();
}

/*bool SaturnEcsGlycolPumpsSwitch::CheckMouseClick(int event, int mx, int my)
{
	if (RotationalSwitch::CheckMouseClick(event, mx, my)) {
		CheckPump();
		return true;
	}
	return false;
}*/

bool SaturnEcsGlycolPumpsSwitch::SwitchTo(int newValue)
{
	if (RotationalSwitch::SwitchTo(newValue)) {
		CheckPump();
		return true;
	}
	return false;
}

void SaturnEcsGlycolPumpsSwitch::LoadState(char *line)
{
	RotationalSwitch::LoadState(line);
	CheckPump();
}

void SaturnEcsGlycolPumpsSwitch::CheckPump()
{
	if (GetState() == 2) 
		GlycolPump->WireTo(NULL);
	else if (GetState() == 1 || GetState() == 3) 
		GlycolPump->WireTo(&ACBus1);
	else
		GlycolPump->WireTo(&ACBus2);
}


void SaturnSuitCompressorSwitch::Init(int xp, int yp, int w, int h, SURFHANDLE surf, SURFHANDLE bsurf, SwitchRow &row,
									  CircuitBrakerSwitch* ac1a, CircuitBrakerSwitch* ac1b, CircuitBrakerSwitch* ac1c,
									  CircuitBrakerSwitch* ac2a, CircuitBrakerSwitch* ac2b, CircuitBrakerSwitch* ac2c)
{
	ACBus1.WireToBuses(ac1a, ac1b, ac1c);
	ACBus2.WireToBuses(ac2a, ac2b, ac2c);
	ThreeSourceSwitch::Init(xp, yp, w, h, surf, bsurf, row, &ACBus1, NULL, &ACBus2);
}

void DCBusIndicatorSwitch::Init(int xp, int yp, int w, int h, SURFHANDLE surf, SwitchRow &row, DCBusController *d, int fc)
{
	IndicatorSwitch::Init(xp, yp, w, h, surf, row);
	dcbus = d;
	fuelcell = fc;
}

int DCBusIndicatorSwitch::GetState()
{
	if (dcbus->IsFuelCellConnected(fuelcell) || !dcbus->IsBusContPowered(fuelcell))
		return 1;
	else
		return 0;
}


SaturnDCAmpMeter::SaturnDCAmpMeter(double minVal, double maxVal, double vMin, double vMax) :
	ElectricMeter(minVal, maxVal, vMin, vMax)
{
}

void SaturnDCAmpMeter::Init(HPEN p0, HPEN p1, SwitchRow &row, Saturn *s, PowerStateRotationalSwitch *dcindicatorswitch)
{
	ElectricMeter::Init(p0, p1, row, dcindicatorswitch);
	DCIndicatorSwitch = dcindicatorswitch;
}

double SaturnDCAmpMeter::QueryValue()
{
	// Battery Charger
	if (DCIndicatorSwitch->GetState() == 7)
		return DCIndicatorSwitch->Current() * 20.0;

	return DCIndicatorSwitch->Current();
}

void BMAGPowerRotationalSwitch::Init(int xp, int yp, int w, int h, SURFHANDLE surf, SURFHANDLE bsurf, SwitchRow &row, BMAG *Unit)
{
	RotationalSwitch::Init(xp, yp, w, h, surf, bsurf, row);
	bmag = Unit;
	
	CheckBMAGPowerState();
}

void BMAGPowerRotationalSwitch::CheckBMAGPowerState()
{
	switch (GetState()) {
	case 0: // OFF
		bmag->SetPower(false, false);
		break;
	case 1: // WARM UP
		bmag->SetPower(true, false);
		break;
	case 2: // ON
		bmag->SetPower(true, true);
		break;
	}
}

/*bool BMAGPowerRotationalSwitch::CheckMouseClick(int event, int mx, int my)
{
	if (RotationalSwitch::CheckMouseClick(event, mx, my)) {		
		CheckBMAGPowerState();
		return true;
	}

	return false;
}*/

bool BMAGPowerRotationalSwitch::SwitchTo(int newValue)
{
	if (RotationalSwitch::SwitchTo(newValue)) {
		CheckBMAGPowerState();
		return true;
	}

	return false;
}

void BMAGPowerRotationalSwitch::LoadState(char *line)
{
	RotationalSwitch::LoadState(line);
	CheckBMAGPowerState();
}


void SaturnSPSPercentMeter::Init(SURFHANDLE blackFontSurf, SURFHANDLE whiteFontSurf, SwitchRow &row, Saturn *s)
{
	MeterSwitch::Init(row);
	BlackFontSurface = blackFontSurf;
	WhiteFontSurface = whiteFontSurf;
	Sat = s;
}

void SaturnSPSPercentMeter::InitVC(SURFHANDLE blackFontSurf, SURFHANDLE whiteFontSurf)
{
	BlackFontSurfacevc = blackFontSurf;
	WhiteFontSurfacevc = whiteFontSurf;
}

void SaturnSPSPercentMeter::DoDrawSwitch(double v, SURFHANDLE drawSurface)
{
	int percent = (int) (v * 1000.0);

	// What should the panel display with full tanks? Looks like 99.9 is the maximum.
	if (percent > 999) {
		percent = 999;
	}

	int digit1 = percent / 100;
	percent -= (digit1 * 100);

	int digit2 = percent / 10;
	int digit3 = percent - (digit2 * 10);

	oapiBlt(drawSurface, BlackFontSurface, 0, 0, 10 * digit1, 0, 10, 12);
	oapiBlt(drawSurface, BlackFontSurface, 13, 0, 10 * digit2, 0, 10, 12);
	oapiBlt(drawSurface, WhiteFontSurface, 26, 0, 11 * digit3, 0, 11, 12);
}

void SaturnSPSPercentMeter::DrawSwitchVC(int id, int event, SURFHANDLE drawSurface)
{
	double v = GetDisplayValue();
	int percent = (int)(v * 1000.0);

	// What should the panel display with full tanks? Looks like 99.9 is the maximum.
	if (percent > 999) {
		percent = 999;
	}

	int digit1 = percent / 100;
	percent -= (digit1 * 100);

	int digit2 = percent / 10;
	int digit3 = percent - (digit2 * 10);

	oapiBlt(drawSurface, BlackFontSurfacevc, 0, 0, 10 * digit1, 0, 10, 12);
	oapiBlt(drawSurface, BlackFontSurfacevc, 13, 0, 10 * digit2, 0, 10, 12);
	oapiBlt(drawSurface, WhiteFontSurfacevc, 26, 0, 11 * digit3, 0, 11, 12);
}

double SaturnSPSOxidPercentMeter::QueryValue()
{
	return Sat->GetSPSPropellant()->GetOxidPercent(); 
}


double SaturnSPSFuelPercentMeter::QueryValue()
{
	return Sat->GetSPSPropellant()->GetFuelPercent(); 
}


double SaturnSPSOxidUnbalMeter::QueryValue()
{
	return Sat->GetSPSPropellant()->GetOxidUnbalanceLB();
}

void SaturnSPSOxidUnbalMeter::DoDrawSwitch(double v, SURFHANDLE drawSurface)
{
	v = v / 450.0 * 90.0;
	DrawNeedle(drawSurface, 30, 31, 28.0, (180.0 - v) * RAD);
}


void SaturnSPSPropellantPressMeter::Init(SURFHANDLE surf, SwitchRow &row, Saturn *s, bool fuel)
{
	MeterSwitch::Init(row);
	NeedleSurface = surf;
	Sat = s;
	Fuel = fuel;
}

double SaturnSPSPropellantPressMeter::QueryValue()
{
	// Fuel and oxidizer have the same pressure for now.
	return Sat->GetSPSPropellant()->GetPropellantPressurePSI();
}

void SaturnSPSPropellantPressMeter::CalculateNeedleState()
{
	double v = GetDisplayValue();

	state = 109 - (int)(v / 250.0 * 103.0);
}

void SaturnSPSPropellantPressMeter::DoDrawSwitch(SURFHANDLE drawSurface)
{
	oapiBlt(drawSurface, NeedleSurface, 0, state, 0, 0, 10, 10, SURF_PREDEF_CK);	
}

void SaturnSPSTempMeter::Init(SURFHANDLE surf, SwitchRow &row, Saturn *s)
{
	MeterSwitch::Init(row);
	NeedleSurface = surf;
	Sat = s;
}

double SaturnSPSTempMeter::QueryValue()
{
	return Sat->GetSPSPropellant()->GetPropellantLineTempF();
}

void SaturnSPSTempMeter::CalculateNeedleState()
{
	state = 109 - (int)(GetDisplayValue() / 200.0 * 103.0);
}

void SaturnSPSTempMeter::DoDrawSwitch(SURFHANDLE drawSurface)
{
	oapiBlt(drawSurface, NeedleSurface, 0, state, 0, 0, 10, 10, SURF_PREDEF_CK);
}

void SaturnSPSHeliumNitrogenPressMeter::Init(SURFHANDLE surf, SwitchRow &row, Saturn *s, ThreePosSwitch *spspressindswitch)
{
	MeterSwitch::Init(row);
	NeedleSurface = surf;
	Sat = s;
	SPSPressIndSwitch = spspressindswitch;
}

double SaturnSPSHeliumNitrogenPressMeter::QueryValue()
{
    if (SPSPressIndSwitch->IsUp()) {
        WireTo(&Sat->InstrumentationPowerFeeder); // InstrumentationPowerFeeder until proper source found
        return Sat->GetSPSPropellant()->GetHeliumPressurePSI();
    } else if (SPSPressIndSwitch->IsCenter()) {
        WireTo(&Sat->Panel276CB3);
        return Sat->GetSPSEngine()->GetNitrogenPressureAPSI();
    } else {
        WireTo(&Sat->Panel276CB4);
        return Sat->GetSPSEngine()->GetNitrogenPressureBPSI();
    }
}

void SaturnSPSHeliumNitrogenPressMeter::CalculateNeedleState()
{
	state = 109 - (int)(GetDisplayValue() / 5000.0 * 103.0);
}

void SaturnSPSHeliumNitrogenPressMeter::DoDrawSwitch(SURFHANDLE drawSurface)
{
	oapiBlt(drawSurface, NeedleSurface, 0, state, 10, 0, 10, 10, SURF_PREDEF_CK);
}

void SaturnLVSPSPcMeter::Init(HPEN p0, HPEN p1, SwitchRow &row, Saturn *s, ToggleSwitch *lvspspcindicatorswitch, SURFHANDLE frameSurface)
{
	SaturnRoundMeter::Init(p0, p1, row, s);
	LVSPSPcIndicatorSwitch = lvspspcindicatorswitch;
	FrameSurface = frameSurface;
}

double SaturnLVSPSPcMeter::QueryValue()
{
	//
	// Note: on a real Saturn, it shows a percentage of pressure difference measured by 
	// the Q-ball of the LES. For now it shows AoA x 10
	//

	if (LVSPSPcIndicatorSwitch->IsDown()) {
		return Sat->GetSPSEngine()->GetChamberPressurePSI();

	} else {
		if (Sat->stage < CSM_LEM_STAGE) {
			return fabs((10.0 / RAD) * Sat->qball.GetAOA());
		} else {
			return 0;
		}
	}
}

void SaturnLVSPSPcMeter::DoDrawSwitch(double v, SURFHANDLE drawSurface)
{
	v = (155.0 - v) / 160.0 * 270.0;	
	DrawNeedle(drawSurface, 48, 45, 20.0, (v - 45.0) * RAD);
	oapiBlt(drawSurface, FrameSurface, 0, 0, 0, 0, 95, 91, SURF_PREDEF_CK);
}

SaturnSystemTestAttenuator::SaturnSystemTestAttenuator(char *i_name, double minIn, double maxIn, double minOut, double maxOut):
	VoltageAttenuator(i_name, minIn, maxIn, minOut, maxOut)
{
}

void SaturnSystemTestAttenuator::Init(Saturn* s, RotationalSwitch *leftsystemtestrotaryswitch, RotationalSwitch *rightsystemtestrotaryswitch, e_object *Instrum)
{
	Sat = s;
	LeftSystemTestRotarySwitch = leftsystemtestrotaryswitch;
	RightSystemTestRotarySwitch = rightsystemtestrotaryswitch;

	WireTo(Instrum);
}

double SaturnSystemTestAttenuator::GetValue()
{
	unsigned char val = NULL;
	int left = LeftSystemTestRotarySwitch->GetState(); //0 = Off, 1 = 1 and so on
	int right = RightSystemTestRotarySwitch->GetState();// 0 = A, 1 = B, 2 = C, 3 = D

	switch(left)
	{
	case 1:
		switch (right)
		{
		case 0:	//FC1 N2 REG PRESS
			val = Sat->pcm.measure(11, TLM_A, 29);
			break;
		case 1:	//FC2 N2 REG PRESS
			val = Sat->pcm.measure(11, TLM_A, 30);
			break;
		case 2:	//FC3 N2 REG PRESS
			val = Sat->pcm.measure(11, TLM_A, 35);
			break;
		case 3:	//FC1 O2 REG PRESS
			val = Sat->pcm.measure(11, TLM_A, 67);
			break;
		}
		break;
	case 2:
		switch (right)
		{
		case 0:	//FC2 O2 REG PRESS
			val = Sat->pcm.measure(11, TLM_A, 68);
			break;
		case 1: //FC3 O2 REG PRESS
			val = Sat->pcm.measure(11, TLM_A, 69);
			break;
		case 2:	//FC1 H2 REG PRESS
			val = Sat->pcm.measure(11, TLM_A, 70);
			break;
		case 3:	//FC2 H2 REG PRESS
			val = Sat->pcm.measure(11, TLM_A, 71);
			break;
		}
		break;
	case 3:
		switch (right)
		{
		case 0:	//FC3 H2 REG PRESS
			val = Sat->pcm.measure(11, TLM_A, 72);
			break;
		case 1: //FC 1 RAD OUT TEMP
			val = Sat->pcm.measure(10, TLM_A, 126);
			break;
		case 2: // FC 2 RAD OUT TEMP
			val = Sat->pcm.measure(10, TLM_A, 129);
			break;
		case 3: // FC 3 RAD OUT TEMP
			val = Sat->pcm.measure(10, TLM_A, 132);
			break;
		}
		break;
	case 4:
		switch (right)
		{
		case 1:	//BAT RLY BUS VOLT
			return Sat->sce.GetVoltage(0, 4)*256.0 / 5.0;	//Temporary scaling
		case 3:	//CSM TO LM CURRENT
			return Sat->sce.GetVoltage(1, 7)*256.0 / 5.0;	//Temporary scaling
		}
		break;
	case 5:
		switch (right)
		{
		case 0:	//SPS OX LINE TEMP
			val = Sat->pcm.measure(10, TLM_A, 120);
			break;
		case 2:	//TEMP JET 24 -P ENG INJECTOR SYS 2
			val = Sat->pcm.scale_data(Sat->CMRCS2.GetInjectorTempF(0), -50, 50);	//Scaled -50F = 0V & 50F = 5V
			break;
		case 3:	//TEMP JET 25 +Y ENG INJECTOR SYS 2
			val = Sat->pcm.scale_data(Sat->CMRCS2.GetInjectorTempF(1), -50, 50);	//Scaled -50F = 0V & 50F = 5V
			break;
		}
		break;
	case 6:
		switch (right)
		{
		case 0:	//TEMP JET 12 CCW ENG INJECTOR SYS 1
			val = Sat->pcm.scale_data(Sat->CMRCS1.GetInjectorTempF(2), -50, 50);	//Scaled -50F = 0V & 50F = 5V
			break;
		case 1:	//TEMP JET 14 -P ENG INJECTOR SYS 1
			val = Sat->pcm.scale_data(Sat->CMRCS1.GetInjectorTempF(0), -50, 50);	//Scaled -50F = 0V & 50F = 5V
			break;
		case 2:	//TEMP JET 16 -Y ENG INJECTOR SYS 1
			val = Sat->pcm.scale_data(Sat->CMRCS1.GetInjectorTempF(1), -50, 50);	//Scaled -50F = 0V & 50F = 5V
			break;
		case 3:	//TEMP JET 21 CW ENG INJECTOR SYS 2
			val = Sat->pcm.scale_data(Sat->CMRCS2.GetInjectorTempF(2), -50, 50);	//Scaled -50F = 0V & 50F = 5V
			break;
		}
		break;
	case 8:
		switch (right)
		{
		case 0:	
			val = Sat->RRTsystem.GetScaledRFPower();
			break;
		case 1:	
			val = Sat->RRTsystem.GetScaledAGCPower();
			break;
		case 2:	
			val = Sat->RRTsystem.GetScaledFreqLock();
			break;
		}
	}

	return (double)val;
}

void SaturnGPFPIMeter::Init(SURFHANDLE surf, SwitchRow &row, Saturn *s, int sys)
{
	MeterSwitch::Init(row);
	NeedleSurface = surf;
	Sat = s;
	system = sys;
}

double SaturnGPFPIMeter::AdjustForPower(double val) 
{ 
	if (ACSource && DCSource) {
		if (ACSource->Voltage() > SP_MIN_ACVOLTAGE && DCSource->Voltage() > SP_MIN_DCVOLTAGE)
			return val;
	}
	return 0; 
}

void SaturnGPFPIMeter::DoDrawSwitch(SURFHANDLE drawSurface)
{
	oapiBlt(drawSurface, NeedleSurface, 0, state, 10, 1, 7, 8, SURF_PREDEF_CK);
	oapiBlt(drawSurface, NeedleSurface, 12, state,  3, 1, 7, 8, SURF_PREDEF_CK);
}

void SaturnGPFPIMeter::CalculateNeedleState()
{
	state = 93 - (int)GetDisplayValue();
}

void SaturnGPFPIMeter::OnPostStep(double SimT, double DeltaT, double MJD)
{
	double v = ((GetDisplayValue() - minValue) * 1.04) / (maxValue - minValue);

	Sat->SetAnimation(anim_switch, v);
}

double SaturnGPFPIPitchMeter::QueryValue()
{
	return Sat->GetEDA()->GetGPFPIPitch(system);
}


double SaturnGPFPIYawMeter::QueryValue()
{
	return Sat->GetEDA()->GetGPFPIYaw(system);
}


void FDAIPowerRotationalSwitch::Init(int xp, int yp, int w, int h, SURFHANDLE surf, SURFHANDLE bsurf, SwitchRow &row, FDAI *F1, FDAI *F2,
    					 		     e_object *dc1, e_object *dc2, e_object *ac1, e_object *ac2,
									 SaturnGPFPIMeter *gpfpiPitch1, SaturnGPFPIMeter *gpfpiPitch2, SaturnGPFPIMeter *gpfpiYaw1, SaturnGPFPIMeter *gpfpiYaw2, EDA *ed)
{
	RotationalSwitch::Init(xp, yp, w, h, surf, bsurf, row);
	FDAI1 = F1;
	FDAI2 = F2;

	DCSource1 = dc1;
	DCSource2 = dc2;
	ACSource1 = ac1;
	ACSource2 = ac2;

	GPFPIPitch1 =gpfpiPitch1;
	GPFPIPitch2 =gpfpiPitch2;
	GPFPIYaw1 =gpfpiYaw1;
	GPFPIYaw2 =gpfpiYaw2;

	eda = ed;

	CheckFDAIPowerState();
}

//
// Wire up the FDAIs to the appropriate power source based on switch position. We wire them to this
// switch, and higher level code can wire the switch to a suitable power source.
//

void FDAIPowerRotationalSwitch::CheckFDAIPowerState()
{
	switch (GetState()) {
	case 0:
		FDAI1->WireTo(NULL, NULL);
		FDAI2->WireTo(NULL, NULL);

		GPFPIPitch1->WireTo(NULL, NULL);
		GPFPIPitch2->WireTo(NULL, NULL);
		GPFPIYaw1->WireTo(NULL, NULL);
		GPFPIYaw2->WireTo(NULL, NULL);

		eda->WireTo(NULL, NULL, NULL, NULL);

		break;

	case 1:
		FDAI1->WireTo(DCSource1, ACSource1);
		FDAI2->WireTo(NULL, NULL);

		GPFPIPitch1->WireTo(DCSource1, ACSource1);
		GPFPIPitch2->WireTo(NULL, NULL);
		GPFPIYaw1->WireTo(DCSource1, ACSource1);
		GPFPIYaw2->WireTo(NULL, NULL);

		eda->WireTo(ACSource1, NULL, DCSource1, NULL);
		break;

	case 2:
		FDAI1->WireTo(NULL, NULL);
		FDAI2->WireTo(DCSource2, ACSource2);

		GPFPIPitch1->WireTo(NULL, NULL);
		GPFPIPitch2->WireTo(DCSource2, ACSource2);
		GPFPIYaw1->WireTo(NULL, NULL);
		GPFPIYaw2->WireTo(DCSource2, ACSource2);

		eda->WireTo(NULL, ACSource2, NULL, DCSource2);
		break;

	case 3:
		FDAI1->WireTo(DCSource1, ACSource1);
		FDAI2->WireTo(DCSource2, ACSource2);

		GPFPIPitch1->WireTo(DCSource1, ACSource1);
		GPFPIPitch2->WireTo(DCSource2, ACSource2);
		GPFPIYaw1->WireTo(DCSource1, ACSource1);
		GPFPIYaw2->WireTo(DCSource2, ACSource2);

		eda->WireTo(ACSource1, ACSource2, DCSource1, DCSource2);
		break;
	}
}

/*bool FDAIPowerRotationalSwitch::CheckMouseClick(int event, int mx, int my)
{
	if (RotationalSwitch::CheckMouseClick(event, mx, my)) {
		CheckFDAIPowerState();
		return true;
	}

	return false;
}*/

bool FDAIPowerRotationalSwitch::SwitchTo(int newValue)
{
	if (RotationalSwitch::SwitchTo(newValue)) {
		CheckFDAIPowerState();
		return true;
	}

	return false;
}

void FDAIPowerRotationalSwitch::LoadState(char *line)
{
	RotationalSwitch::LoadState(line);
	CheckFDAIPowerState();
}


//
// CMACInverterSwitch allows you to connect the CM AC inverters to the CM AC busses.
//

void CMACInverterSwitch::Init(int xp, int yp, int w, int h, SURFHANDLE surf, SURFHANDLE bsurf, SwitchRow &row,int bus,int inv, Saturn *ship)
{
	ToggleSwitch::Init(xp, yp, w, h, surf, bsurf, row);
	acbus = bus;
	acinv = inv;
	sat = ship;
	UpdateSourceState();
}

/*bool CMACInverterSwitch::CheckMouseClick(int event, int mx, int my)
{
	if (ToggleSwitch::CheckMouseClick(event, mx, my))
	{
		UpdateSourceState();
		return true;
	}

	return false;
}*/

bool CMACInverterSwitch::SwitchTo(int newState, bool dontspring)
{
	if (ToggleSwitch::SwitchTo(newState,dontspring)) {
		UpdateSourceState();
		return true;
	}
	return false;
}

void CMACInverterSwitch::UpdateSourceState()
{	 
	if (IsUp()) {
		switch(acbus){
		case 1: // AC1
			switch(acinv){
			case 1:
				// Inverter 3 beats inverter 1
				if(sat->AcBus1Switch3.GetState() == TOGGLESWITCH_UP){
					return; // LOCKOUT
				}
				sat->ACBus1PhaseA.WireTo(&sat->Inverter1->PhaseA);
				sat->ACBus1PhaseB.WireTo(&sat->Inverter1->PhaseB);
				sat->ACBus1PhaseC.WireTo(&sat->Inverter1->PhaseC);
				break;
			case 2:
				// Inverter 1 beats inverter 2
				if(sat->AcBus1Switch1.GetState() == TOGGLESWITCH_UP){
					return; // LOCKOUT
				}
				sat->ACBus1PhaseA.WireTo(&sat->Inverter2->PhaseA);
				sat->ACBus1PhaseB.WireTo(&sat->Inverter2->PhaseB);
				sat->ACBus1PhaseC.WireTo(&sat->Inverter2->PhaseC);
				break;
			case 3:
				// Inverter 2 beats 3
				if(sat->AcBus1Switch2.GetState() == TOGGLESWITCH_UP){
					return; // LOCKOUT
				}
				sat->ACBus1PhaseA.WireTo(&sat->Inverter3->PhaseA);
				sat->ACBus1PhaseB.WireTo(&sat->Inverter3->PhaseB);
				sat->ACBus1PhaseC.WireTo(&sat->Inverter3->PhaseC);
				break;
			}
			break;
		case 2: // AC2
			switch(acinv){
			case 1:
				if(sat->AcBus2Switch3.GetState() == TOGGLESWITCH_UP){
					return; // LOCKOUT
				}
				sat->ACBus2PhaseA.WireTo(&sat->Inverter1->PhaseA);
				sat->ACBus2PhaseB.WireTo(&sat->Inverter1->PhaseB);
				sat->ACBus2PhaseC.WireTo(&sat->Inverter1->PhaseC);
				break;
			case 2:
				if(sat->AcBus2Switch1.GetState() == TOGGLESWITCH_UP){
					return; // LOCKOUT
				}
				sat->ACBus2PhaseA.WireTo(&sat->Inverter2->PhaseA);
				sat->ACBus2PhaseB.WireTo(&sat->Inverter2->PhaseB);
				sat->ACBus2PhaseC.WireTo(&sat->Inverter2->PhaseC);
				break;
			case 3:
				if(sat->AcBus2Switch2.GetState() == TOGGLESWITCH_UP){
					return; // LOCKOUT
				}
				sat->ACBus2PhaseA.WireTo(&sat->Inverter3->PhaseA);
				sat->ACBus2PhaseB.WireTo(&sat->Inverter3->PhaseB);
				sat->ACBus2PhaseC.WireTo(&sat->Inverter3->PhaseC);
				break;
			}
			break;
		}
	}
	else if (IsDown()) {
		// SHUTTING DOWN CASE
		switch(acbus){
		case 1: // AC1
			switch(acinv){
			case 1:
				if(sat->ACBus1PhaseA.SRC != &sat->Inverter1->PhaseA){ return; } // LOCKOUT
				sat->ACBus1PhaseA.WireTo(NULL);
				sat->ACBus1PhaseB.WireTo(NULL);
				sat->ACBus1PhaseC.WireTo(NULL);
				// Check inv 2
				if(sat->AcBus1Switch2.GetState() == TOGGLESWITCH_UP){ sat->AcBus1Switch2.UpdateSourceState(); }
				break;
			case 2:
				if(sat->ACBus1PhaseA.SRC != &sat->Inverter2->PhaseA){ return; } // LOCKOUT
				sat->ACBus1PhaseA.WireTo(NULL);
				sat->ACBus1PhaseB.WireTo(NULL);
				sat->ACBus1PhaseC.WireTo(NULL);
				if(sat->AcBus1Switch3.GetState() == TOGGLESWITCH_UP){ sat->AcBus1Switch3.UpdateSourceState(); }
				break;
			case 3:
				if(sat->ACBus1PhaseA.SRC != &sat->Inverter3->PhaseA){ return; } // LOCKOUT
				sat->ACBus1PhaseA.WireTo(NULL);
				sat->ACBus1PhaseB.WireTo(NULL);
				sat->ACBus1PhaseC.WireTo(NULL);
				if(sat->AcBus1Switch1.GetState() == TOGGLESWITCH_UP){ sat->AcBus1Switch1.UpdateSourceState(); }
				break;
			}
			break;
		case 2: // AC2
			switch(acinv){
			case 1:
				if(sat->ACBus2PhaseA.SRC != &sat->Inverter1->PhaseA){ return; } // LOCKOUT
				sat->ACBus2PhaseA.WireTo(NULL);
				sat->ACBus2PhaseB.WireTo(NULL);
				sat->ACBus2PhaseC.WireTo(NULL);
				if(sat->AcBus2Switch2.GetState() == TOGGLESWITCH_UP){ sat->AcBus2Switch2.UpdateSourceState(); }
				break;
			case 2:
				if(sat->ACBus2PhaseA.SRC != &sat->Inverter2->PhaseA){ return; } // LOCKOUT
				sat->ACBus2PhaseA.WireTo(NULL);
				sat->ACBus2PhaseB.WireTo(NULL);
				sat->ACBus2PhaseC.WireTo(NULL);
				if(sat->AcBus2Switch3.GetState() == TOGGLESWITCH_UP){ sat->AcBus2Switch3.UpdateSourceState(); }
				break;
			case 3:
				if(sat->ACBus2PhaseA.SRC != &sat->Inverter3->PhaseA){ return; } // LOCKOUT
				sat->ACBus2PhaseA.WireTo(NULL);
				sat->ACBus2PhaseB.WireTo(NULL);
				sat->ACBus2PhaseC.WireTo(NULL);
				if(sat->AcBus2Switch1.GetState() == TOGGLESWITCH_UP){ sat->AcBus2Switch1.UpdateSourceState(); }
				break;
			}
			break;
		}		
	}
}

void CMACInverterSwitch::LoadState(char *line)
{
	ToggleSwitch::LoadState(line);
	UpdateSourceState();
}


/*bool SaturnSCContSwitch::CheckMouseClick(int event, int mx, int my) 
{
	if (SaturnToggleSwitch::CheckMouseClick(event, mx, my)) {
		SetSCControl(sat);
		return true;
	}
	return false;
}*/

bool SaturnSCContSwitch::SwitchTo(int newState, bool dontspring)
{
	if (SaturnToggleSwitch::SwitchTo(newState,dontspring)) {
		SetSCControl(sat);
		return true;
	}
	return false;
}


void THCRotarySwitch::Init(int xp, int yp, int w, int h, SURFHANDLE surf, SURFHANDLE bsurf, SwitchRow &row, Saturn *s)
{
	RotationalSwitch::Init(xp, yp, w, h, surf, bsurf, row);
	sat = s;
}

/*bool THCRotarySwitch::CheckMouseClick(int event, int mx, int my) 
{
	if (RotationalSwitch::CheckMouseClick(event, mx, my)) {
		SetSCControl(sat);
		return true;
	}
	return false;
}*/

bool THCRotarySwitch::SwitchTo(int newState)
{
	if (RotationalSwitch::SwitchTo(newState)) {
		SetSCControl(sat);
		return true;
	}
	return false;
}

void SaturnSCControlSetter::SetSCControl(Saturn *sat)
{
	if (sat->SCContSwitch.IsUp() && !sat->THCRotary.IsClockwise()) {
		sat->agc.SetInputChannelBit(031, GNAutopilotControl, true);
	} else {
		sat->agc.SetInputChannelBit(031, GNAutopilotControl, false);
	}
}

void SaturnEMSDvDisplay::Init(SURFHANDLE digits, SwitchRow &row, Saturn *s)
{
	MeterSwitch::Init(row);
	Digits = digits;
	Sat = s;
}

void SaturnEMSDvDisplay::DoDrawSwitch(double v, SURFHANDLE drawSurface)
{
	if (Voltage() < SP_MIN_DCVOLTAGE || Sat->ems.IsOff() || !Sat->ems.IsDisplayPowered()) return;

	const int DigitWidth = 17;
	const int DigitHeight = 19;

	if (v < 0) {	// Draw minus sign
		oapiBlt(drawSurface, Digits, 0, 0, 10 * DigitWidth, 0, DigitWidth, DigitHeight);
	}

	int i, Curdigit;
	char buffer[100];
	sprintf(buffer, "%7.1f", fabs(v));
	for (i = 0; i < 7; i++) {
		if (buffer[i] >= '0' && buffer[i] <= '9') {
			Curdigit = buffer[i] - '0';
			oapiBlt(drawSurface, Digits, (i == 6 ? -2 : 8) + DigitWidth * i, 0, DigitWidth * Curdigit, 0, DigitWidth, DigitHeight);	// Offset final (6th) digit
		} else if (buffer[i] == '.') {
			if (!Sat->ems.IsDecimalPointBlanked())
			{
				oapiBlt(drawSurface, Digits, 8 + DigitWidth * i, 0, 12 * DigitWidth, 0, 4, DigitHeight);	// Draw decimal point
			}
		}
	}
}

void SaturnEMSDvDisplay::DoDrawSwitchVC(SURFHANDLE surf, double v, SURFHANDLE drawSurface)
{
	if (Voltage() < SP_MIN_DCVOLTAGE || Sat->ems.IsOff() || !Sat->ems.IsDisplayPowered()) return;

	const int DigitWidth = 17;
	const int DigitHeight = 19;

	if (v < 0) {	// Draw minus sign
		oapiBlt(surf, drawSurface, 0, 0, 10 * DigitWidth, 0, DigitWidth, DigitHeight);
	}

	int i, Curdigit;
	char buffer[100];
	sprintf(buffer, "%7.1f", fabs(v));
	for (i = 0; i < 7; i++) {
		if (buffer[i] >= '0' && buffer[i] <= '9') {
			Curdigit = buffer[i] - '0';
			oapiBlt(surf, drawSurface, (i == 6 ? -2 : 8) + DigitWidth * i, 0, DigitWidth * Curdigit, 0, DigitWidth, DigitHeight);	// Offset final (6th) digit
		}
		else if (buffer[i] == '.') {
			if (!Sat->ems.IsDecimalPointBlanked())
			{
				oapiBlt(surf, drawSurface, 8 + DigitWidth * i, 0, 12 * DigitWidth, 0, 4, DigitHeight);	// Draw decimal point
			}
		}
	}
}

double SaturnEMSDvDisplay::QueryValue() {
	return Sat->ems.GetdVRangeCounter();
}

int SaturnEMSDvDisplay::GetState() {
	return (int) (Sat->ems.GetdVRangeCounter() * 10.);
}

void SaturnEMSDvDisplay::SetState(int value) {
	Sat->ems.dVRangeCounter = value / 10.;
}


void SaturnEventTimer::Init(SwitchRow &row, Saturn *s) {
	MeterSwitch::Init(row);
	Sat = s;
}

int SaturnEventTimer::GetState() {
	return (int) (Sat->EventTimerDisplay.GetTime());
}

void SaturnEventTimer::SetState(int value) {
	Sat->EventTimerDisplay.SetTime(value);
}


void SaturnEMSScrollDisplay::Init(SwitchRow &row, Saturn *s) {
	MeterSwitch::Init(row);
	Sat = s;
}

int SaturnEMSScrollDisplay::GetState() {
	return (int) (Sat->ems.ScrollPosition);
}

void SaturnEMSScrollDisplay::SetState(int value) {
	Sat->ems.ScrollPosition = value;
}


void SaturnPanel382Cover::Init(SwitchRow &row, Saturn *s) {
	MeterSwitch::Init(row);
	Sat = s;
}

int SaturnPanel382Cover::GetState() {
	return Sat->panel382Enabled;
}

void SaturnPanel382Cover::SetState(int value) {
	Sat->panel382Enabled = value;
}


void SaturnPanel600::Init(SwitchRow &row, Saturn *s) {
	MeterSwitch::Init(row);
	Sat = s;
}

int SaturnPanel600::GetState() {
	return Sat->hatchPanel600EnabledLeft;
}

void SaturnPanel600::SetState(int value)
{
	if (value == 0) value = -1;

	Sat->hatchPanel600EnabledLeft = value;
	Sat->hatchPanel600EnabledRight = value;
}


void SaturnPanelOrdeal::Init(SwitchRow &row, Saturn *s) {
	MeterSwitch::Init(row);
	Sat = s;
}

int SaturnPanelOrdeal::GetState() {
	return Sat->ordealEnabled;
}

void SaturnPanelOrdeal::SetState(int value)
{
	if (value == 0) value = -1;

	Sat->ordealEnabled = value;
}


void SaturnASCPSwitch::Init(SwitchRow &row, Saturn *s, int axis) {
	MeterSwitch::Init(row);
	Sat = s;
	Axis = axis;
}

int SaturnASCPSwitch::GetState() {
	return (int) (Sat->ascp.output.data[Axis] * 10.);
}

void SaturnASCPSwitch::SetState(int value) {
	Sat->ascp.output.data[Axis] = value / 10.;
}

SaturnEMSDvSetSwitch::SaturnEMSDvSetSwitch(Sound &clicksound) : ClickSound(clicksound)
{
	sat = NULL;
	position = 0;
	anim_emsdvsetswitch = -1;
	grp = 0;
	dvswitchrot = NULL;
}

SaturnEMSDvSetSwitch::~SaturnEMSDvSetSwitch()
{
	if (dvswitchrot) delete dvswitchrot;
}

bool SaturnEMSDvSetSwitch::CheckMouseClick(int event, int mx, int my)
{
	int oldPos = position;
	switch(event) {
		case PANEL_MOUSE_LBPRESSED:
			if (my < 44)
				position = 1;
			else
				position = 3;			
			break;

		case PANEL_MOUSE_RBPRESSED:
			if (my < 44)
				position = 2;
			else
				position = 4;
			break;

		case PANEL_MOUSE_LBUP:
		case PANEL_MOUSE_RBUP:
			position = 0;
			break;
	}
	if (position && position != oldPos)
		ClickSound.play();
	return true;
}

bool SaturnEMSDvSetSwitch::CheckMouseClickVC(int event, VECTOR3 &p) {

	int oldPos = position;
	switch (event) {
	case PANEL_MOUSE_LBPRESSED:
		if (p.y < 0.5)
			position = 1;
		else
			position = 3;
		break;

	case PANEL_MOUSE_RBPRESSED:
		if (p.y < 0.5)
			position = 2;
		else
			position = 4;
		break;

	case PANEL_MOUSE_LBUP:
	case PANEL_MOUSE_RBUP:
		position = 0;
		break;
	}
	if (position && position != oldPos)
		ClickSound.play();
	return true;
}

void SaturnEMSDvSetSwitch::SetReference(const VECTOR3& ref)
{
	reference = ref;
}

void SaturnEMSDvSetSwitch::DefineMeshGroup(UINT _grp)
{
	grp = _grp;
}

const VECTOR3& SaturnEMSDvSetSwitch::GetReference() const
{
	return reference;
}

void SaturnEMSDvSetSwitch::DefineVCAnimations(UINT vc_idx)
{
	dvswitchrot = new MGROUP_ROTATE(vc_idx, &grp, 1, GetReference(), _V(1, 0, 0), (float)(RAD * 10));
	anim_emsdvsetswitch = sat->CreateAnimation(0.5);
	sat->AddAnimationComponent(anim_emsdvsetswitch, 0.0f, 1.0f, dvswitchrot);
}

void SaturnEMSDvSetSwitch::DrawSwitchVC(int id, int event, SURFHANDLE surf)
{
	if (anim_emsdvsetswitch != -1) {
		switch ((int)GetPosition()) {
		case 1:
			sat->SetAnimation(anim_emsdvsetswitch, 1.0);
			break;
		case 2:
			sat->SetAnimation(anim_emsdvsetswitch, 0.75);
			break;
		case 3:
			sat->SetAnimation(anim_emsdvsetswitch, 0.0);
			break;
		case 4:
			sat->SetAnimation(anim_emsdvsetswitch, 0.25);
			break;
		default:
			sat->SetAnimation(anim_emsdvsetswitch, 0.5);
			break;
		}
	}
}

bool SaturnCabinPressureReliefLever::CheckMouseClickVC(int event, VECTOR3 &p) {
	int mx = (int)(p.x * (x + width));
	int my = (int)(p.y * (y + height));

	return CheckMouseClick(event, mx, my);
}

void SaturnGuardedCabinPressureReliefLever::InitGuard(SURFHANDLE surf, SoundLib *soundlib)
{	
	guardSurface = surf;

	if (!guardClick.isValid())
		soundlib->LoadSound(guardClick, GUARD_SOUND, INTERNAL_ONLY);
}

void SaturnGuardedCabinPressureReliefLever::DrawSwitch(SURFHANDLE drawSurface)
{
	oapiBlt(drawSurface, guardSurface, 0, 0, guardState * 152, 0, 152, 79, SURF_PREDEF_CK);
	ThumbwheelSwitch::DrawSwitch(drawSurface);
}

void SaturnGuardedCabinPressureReliefLever::DrawSwitchVC(int id, int event, SURFHANDLE surf) {

	ThumbwheelSwitch::DrawSwitchVC(id, event, surf);

	if (guardState > 0) {
		OurVessel->SetAnimation(guardAnim, 1.0);
	} else {
		OurVessel->SetAnimation(guardAnim, 0.0);
	}
}

bool SaturnGuardedCabinPressureReliefLever::CheckMouseClick(int event, int mx, int my)
{
	if (event & PANEL_MOUSE_RBDOWN) {
		if (mx <= 152 &&  my <= 79) {			
			if (guardState) {
				guardState = 0;
			} else {
				guardState = 1;
			}
			guardClick.play();
			return true;

		} else {
			return false;
		}
	} else {
		bool r = ThumbwheelSwitch::CheckMouseClick(event, mx, my);
		if (state == 3 && guardState == 0) {
			state = 2;
			return false;
		} else {
			return r;
		}
	}
}

bool SaturnGuardedCabinPressureReliefLever::SwitchTo(int newState)
{
	if (ThumbwheelSwitch::SwitchTo(newState))
	{
		if(state == 3 && guardState == 0)
		{
			state = 2;
			return false;
		}
		else
			return true;
	}
	return false;
}

void SaturnGuardedCabinPressureReliefLever::SetState(int value)
{
	if (ThumbwheelSwitch::SwitchTo(value))
	{
		if (state == 3 && guardState == 0)
		{
			guardState = 1;
		}
	}
}

void SaturnGuardedCabinPressureReliefLever::Guard()
{
	if (guardState) {
		guardState = 0;

		if (state == 3)
		{
			state = 2;
		}
	}
}

void SaturnGuardedCabinPressureReliefLever::SaveState(FILEHANDLE scn)
{
	char buffer[100];

	sprintf(buffer, "%i %i", state, guardState); 
	oapiWriteScenario_string(scn, name, buffer);
}

void SaturnGuardedCabinPressureReliefLever::LoadState(char *line)
{
	char buffer[100];
	int st, gst;

	sscanf(line, "%s %i %i", buffer, &st, &gst); 
	if (!strnicmp(buffer, name, strlen(name))) {
		state = st;
		guardState = gst;
	}
}

void SaturnGuardedCabinPressureReliefLever::DefineVCAnimations(UINT vc_idx)
{
	ThumbwheelSwitch::DefineVCAnimations(vc_idx);

	ANIMATIONCOMPONENT_HANDLE ach_guardAnim;
	const VECTOR3 Cab_Press_Rel_Handle_LockLocation = { -1.1553, 0.6961, -0.2298 };
	static UINT	meshgroup_guard = { VC_GRP_Cab_Press_Rel_Handle_Lock };
	static MGROUP_ROTATE mgt_guardAnim(vc_idx, &meshgroup_guard, 1, Cab_Press_Rel_Handle_LockLocation, _V(0, 0, -1), (float)(37.0*RAD));
	guardAnim = OurVessel->CreateAnimation(0.0);
	ach_guardAnim = OurVessel->AddAnimationComponent(guardAnim, 0.0f, 1.0f, &mgt_guardAnim);
	OurVessel->SetAnimation(guardAnim, 1.0);
}

void OpticsHandcontrollerSwitch::Init(int xp, int yp, int w, int h, SURFHANDLE surf, SURFHANDLE bsurf, SwitchRow &row, Saturn *s)
{
	sat = s;
	HandcontrollerSwitch::Init(xp, yp, w, h, surf, bsurf, row);
}

bool OpticsHandcontrollerSwitch::CheckMouseClick(int event, int mx, int my)
{
	if (HandcontrollerSwitch::CheckMouseClick(event, mx, my)) {
		sat->optics.OpticsManualMovement &= 0xF0; 
		if (state == 1) {
			// Optics Right
			sat->optics.OpticsManualMovement |= 0x08; 
		} else if (state == 2) {
			// Optics Left
			sat->optics.OpticsManualMovement |= 0x04;
		} else if (state == 3) {
			// Optics Down
			sat->optics.OpticsManualMovement |= 0x02; 
		} else if (state == 4) {
			// Optics Up
			sat->optics.OpticsManualMovement |= 0x01; 
		}
		return true;
	}
	return false;
}


void MinImpulseHandcontrollerSwitch::Init(int xp, int yp, int w, int h, SURFHANDLE surf, SURFHANDLE bsurf, SwitchRow &row, Saturn *s)
{
	sat = s;
	HandcontrollerSwitch::Init(xp, yp, w, h, surf, bsurf, row);
}

bool MinImpulseHandcontrollerSwitch::CheckMouseClick(int event, int mx, int my)
{
	if (HandcontrollerSwitch::CheckMouseClick(event, mx, my)) {
		unsigned int c = sat->agc.GetInputChannel(032);
		c &= 077700;
		if (state == 1) {
			// roll right
			c |= 020;			
		} else if (state == 2) {
			// roll left
			c |= 040;			
		} else if (state == 3) {
			// pitch down
			c |= 02;
		} else if (state == 4) {
			// pitch up
			c |= 01;
		} else if (state == 5) {
			// yaw left
			c |= 010;			
		} else if (state == 6) {
			// yaw right
			c |= 04;			
		}		
		sat->agc.SetInputChannel(032, c);
		return true;
	}
	return false;
}

void DSEIndicatorSwitch::Init(int xp, int yp, int w, int h, SURFHANDLE surf, SwitchRow &row, DSE *d, bool failopen)
{
	dse = d;
	IndicatorSwitch::Init(xp, yp, w, h, surf, row, failopen);
}

int DSEIndicatorSwitch::GetState()
{
	return dse->TapeMotion() ? 1 : 0;
}


//
// "Special offset" of 130 px to avoid overlapping with the O2DemandRegulatorRotary
//

void SuitTestSwitch::DrawSwitch(SURFHANDLE drawSurface)
{
	if (position) {
		// Find best bitmap for position angle
		int index = -1;
		for (int i = 0; i < RotationalSwitchBitmapCount; i++) {
			if (index == -1) {
				index = i;
			} else if (AngleDiff(bitmaps[i].angle, position->GetAngle()) < AngleDiff(bitmaps[index].angle, position->GetAngle())) {
				index = i;
			}
		}
		oapiBlt(drawSurface, switchSurface, x, y, (bitmaps[index].xOffset * width) + 130, bitmaps[index].yOffset * height, width - 130, height, SURF_PREDEF_CK);
	}
}

void SuitTestSwitch::DrawFlash(SURFHANDLE DrawSurface)
{
	if (!visible)
		return;

	if (switchBorder)
		oapiBlt(DrawSurface, switchBorder, x, y, 0, 0, width - 130, height, SURF_PREDEF_CK);
}

bool SuitTestSwitch::CheckMouseClick(int event, int mx, int my) 
{	
	return RotationalSwitch::CheckMouseClick(event, mx + 130, my);
}


double SaturnOxygenRepressPressMeter::QueryValue()
{
	AtmosStatus atm;
	Sat->GetAtmosStatus(atm);

	return atm.O2RepressPressurePSI;
}

void SaturnOxygenRepressPressMeter::DoDrawSwitch(double v, SURFHANDLE drawSurface)
{
	v = 115.0 - v / 1200.0 * 50.0 ;	
	DrawNeedle(drawSurface, 55, 440, 60.0, v * RAD);
	oapiBlt(drawSurface, FrameSurface, 0, 413, 0, 110, 90, 30, SURF_PREDEF_CK);
}

// LM power switch
void CSMLMPowerSwitch::Init(int xp, int yp, int w, int h, SURFHANDLE surf, SURFHANDLE bsurf, SwitchRow &row, Saturn *s){
	sat = s;
	ThreePosSwitch::Init(xp, yp, w, h, surf, bsurf, row);
}

void CSMLMPowerSwitch::LoadState(char *line)
{
	ToggleSwitch::LoadState(line);

	switch (state)
	{
	case THREEPOSSWITCH_UP:
		sat->CSMToLEMPowerDrain.Enable();
		sat->CSMToLEMPowerDrain.WireTo(&sat->LMUmbilicalFeeder);
		break;
	default:
		sat->CSMToLEMPowerDrain.Disable();
		sat->CSMToLEMPowerDrain.WireTo(NULL);
		break;
	}

}

bool CSMLMPowerSwitch::SwitchTo(int newState, bool dontspring)
{
	if (SaturnThreePosSwitch::SwitchTo(newState, dontspring)) {
		switch (state) {
		case THREEPOSSWITCH_UP:
			// Connect the bus
			sat->CSMToLEMPowerDrain.Enable();
			sat->CSMToLEMPowerDrain.WireTo(&sat->LMUmbilicalFeeder);
			break;
		case THREEPOSSWITCH_CENTER:
		case THREEPOSSWITCH_DOWN:
			// Ensure disconnected
			sat->CSMToLEMPowerDrain.Disable();
			sat->CSMToLEMPowerDrain.WireTo(NULL);
			break;
		}
		return true;
	}
	return false;
}

double SaturnHighGainAntennaPitchMeter::QueryValue(){
	return Sat->hga.GetResolvedPitch();
}

void SaturnHighGainAntennaPitchMeter::DoDrawSwitch(double v, SURFHANDLE drawSurface){
	// Gauge runs from -110 to 110 degrees. Sign matches.
	v = v * 1.222222;
	DrawNeedle(drawSurface, 30, 25, 20.0, v * RAD);
}

double SaturnHighGainAntennaStrengthMeter::QueryValue(){
	return Sat->usb.rcvr_agc_voltage;
}

void SaturnHighGainAntennaStrengthMeter::DoDrawSwitch(double v, SURFHANDLE drawSurface){
	// Gauge runs from 180 to 0. Sign reversed.
	v = 180-(v*1.8);
	DrawNeedle(drawSurface, 107, 25, 20.0, v * RAD);
}

double SaturnHighGainAntennaYawMeter::QueryValue(){
	return Sat->hga.GetResolvedYaw();
}

void SaturnHighGainAntennaYawMeter::DoDrawSwitch(double v, SURFHANDLE drawSurface){
	// Gauge runs from 220 to -40. Sign reversed.
	v = 220-(v*0.722222);
	DrawNeedle(drawSurface, 185, 25, 20.0, v * RAD);
}

double SaturnLMDPGauge::QueryValue(){
	if (Sat->LMTunnelVentValve.GetState() == 2)
	{
		AtmosStatus atm;
		Sat->GetAtmosStatus(atm);

		return atm.CabinPressurePSI - atm.TunnelPressurePSI;
	}

	return -1.0;
}

void SaturnLMDPGauge::DoDrawSwitch(double v, SURFHANDLE drawSurface){
	// Gauge runs from 122.5 to 57.5, sign reversed.
	v = 122.5-((v+1)*13);	
	DrawNeedle(drawSurface, 43, 77, 20.0, v * RAD);
}

void SaturnLMDPGauge::DrawNeedle (SURFHANDLE surf, int x, int y, double rad, double angle){
	// Needle function by Rob Conley from Mercury code
	// This one needs a longer and offset needle

	double dx = rad * cos(angle), dy = rad * sin(angle);
	HGDIOBJ oldObj;

	HDC hDC = oapiGetDC (surf);
	oldObj = SelectObject (hDC, Pen1);
	MoveToEx (hDC, x + (int)(2*dx+0.5), y - (int)(2*dy+0.5), 0); 
	LineTo (hDC, x + (int)(3*dx+0.5), y - (int)(3*dy+0.5));
	SelectObject (hDC, oldObj);
	oapiReleaseDC (surf, hDC);
}

// Right Docking Target Switch
bool DockingTargetSwitch::SwitchTo(int newState, bool dontspring)
{
	if (SaturnThreePosSwitch::SwitchTo(newState, dontspring)) {
		switch (state) {
		case THREEPOSSWITCH_UP:  // BRIGHT
			sat->CMdocktgt = true;
			break;
		case THREEPOSSWITCH_CENTER:  // DIM
			sat->CMdocktgt = true;
			break;
		case THREEPOSSWITCH_DOWN:  // OFF
			sat->CMdocktgt = false;
			break;
		}
		sat->SetCMdocktgtMesh();
		//sprintf(oapiDebugString(), "Flag %d, Index number %d", sat->CMdocktgt, sat->cmdocktgtidx);
		return true;
	}
	return false;
}

SaturnLiftoffNoAutoAbortSwitch::SaturnLiftoffNoAutoAbortSwitch()
{
	secs = NULL;
}

void SaturnLiftoffNoAutoAbortSwitch::Init(int xp, int yp, int w, int h, SURFHANDLE surf, SURFHANDLE bsurf, SwitchRow &row, SECS *s,
	int xoffset, int yoffset)
{
	GuardedPushSwitch::Init(xp, yp, w, h, surf, bsurf, row, xoffset, yoffset);

	secs = s;
}

void SaturnLiftoffNoAutoAbortSwitch::DoDrawSwitch(SURFHANDLE drawSurface)
{
	if (secs->LiftoffLightPower()) {
		if (!secs->NoAutoAbortLightPower())
			SetOffset(78, 81);
		else
			SetOffset(234, 81);
	}
	else {
		SetOffset(0, 81);
	}

	GuardedPushSwitch::DoDrawSwitch(drawSurface);
}

void SaturnLiftoffNoAutoAbortSwitch::RepaintSwitchVC(SURFHANDLE drawSurface, SURFHANDLE switchsurfacevc)
{
	int ofs = 4;
	if (secs->LiftoffLightPower()) {
		if (!secs->NoAutoAbortLightPower())
			oapiBlt(drawSurface, switchsurfacevc, 0 + ofs - 1, 0 + ofs, 117 + ofs, 1 + ofs, width - ofs, height - ofs, SURF_PREDEF_CK);
		else
			oapiBlt(drawSurface, switchsurfacevc, 0 + ofs - 1, 0 + ofs, 273 + ofs, 1 + ofs, width - ofs, height - ofs, SURF_PREDEF_CK);
	}
	else {
		oapiBlt(drawSurface, switchsurfacevc, 0 + ofs - 1, 0 + ofs, 39 + ofs, 1 + ofs, width - ofs, height - ofs, SURF_PREDEF_CK);
	}
}

void SaturnPanel181::Register(PanelSwitchScenarioHandler *PSH)
{
	SMSector1Cryo3ACPowerSwitch.Register(*PSH, "SMSector1Cryo3ACPowerSwitch", TOGGLESWITCH_UP);
	SMSector1SMACPowerSwitch.Register(*PSH, "SMSector1SMACPowerSwitch", TOGGLESWITCH_UP);
	SMSector1AC2ASystemBraker.Register(*PSH, "SMSector1AC2ASystemBraker", 1);
	SMSector1AC2BSystemBraker.Register(*PSH, "SMSector1AC2BSystemBraker", 1);
	SMSector1AC2CSystemBraker.Register(*PSH, "SMSector1AC2CSystemBraker", 1);
	SMSector1DoorJettisonSwitch.Register(*PSH, "SMSector1DoorJettisonSwitch", TOGGLESWITCH_DOWN, false, SPRINGLOADEDSWITCH_DOWN);
	SMSector1LogicPower1Switch.Register(*PSH, "SMSector1LogicPower1Switch", THREEPOSSWITCH_CENTER);
	SMSector1LogicPower2Switch.Register(*PSH, "SMSector1LogicPower2Switch", THREEPOSSWITCH_CENTER);
	SMSector1LogicPowerMNABraker.Register(*PSH, "SMSector1LogicPowerMNABraker", 0);
	SMSector1LogicPowerMNBBraker.Register(*PSH, "SMSector1LogicPowerMNBBraker", 0);
}

void SaturnPanel277::Register(PanelSwitchScenarioHandler *PSH)
{
	SPSPressIndFuelSwitch.Register(*PSH, "SPSPressIndFuelSwitch", TOGGLESWITCH_UP);
	SPSPressIndFuelSwitch.SetSideways(1);
	SPSPressIndOxidSwitch.Register(*PSH, "SPSPressIndOxidSwitch", TOGGLESWITCH_UP);
	SPSPressIndOxidSwitch.SetSideways(1);
	CSMLMFinalSepABatABraker.Register(*PSH, "CSMLMFinalSepABatABraker", 1);
	CSMLMFinalSepBBatBBraker.Register(*PSH, "CSMLMFinalSepBBatBBraker", 1);
}

void SaturnPanel278J::Register(PanelSwitchScenarioHandler *PSH)
{
	ExperimentCovers1Switch.Register(*PSH, "ExperimentCovers1Switch", THREEPOSSWITCH_CENTER);
	ExperimentCovers1Switch.SetSideways(2);
	ExperimentCovers2Switch.Register(*PSH, "ExperimentCovers2Switch", THREEPOSSWITCH_CENTER);
	ExperimentCovers2Switch.SetSideways(2);
	SMPowerSourceSwitch.Register(*PSH, "SMPowerSourceSwitch", THREEPOSSWITCH_DOWN, false);
	SMPowerSourceSwitch.SetSideways(2);
	SMPowerSourceSwitch.SetGuardResetsState(false);
	O2Tank3IsolSwitch.Register(*PSH, "O2Tank3IsolSwitch", THREEPOSSWITCH_CENTER);
	O2Tank3IsolSwitch.SetSideways(2);
	ExperimentCovers1Indicator.Register(*PSH, "ExperimentCovers1Indicator", false);
	ExperimentCovers2Indicator.Register(*PSH, "ExperimentCovers2Indicator", false);
	O2Tank3IsolIndicator.Register(*PSH, "O2Tank3IsolIndicator", false);
	ExperimentCoversDeployBraker.Register(*PSH, "ExperimentCoversDeployBraker", 1);
}

bool LeftCOASPowerSwitch::SwitchTo(int newState, bool dontspring)
{
	if (SaturnToggleSwitch::SwitchTo(newState, dontspring)) {

		if (state == TOGGLESWITCH_UP) {
			sat->COASreticlevisible = true;
		} else {
			sat->COASreticlevisible = false;
		}
		sat->SetCOASMesh();
		return true;
	}
	return false;
}

void SaturnAltimeter::Init(SURFHANDLE surf1, SURFHANDLE surf2, Saturn *s) {

	surface1 = surf1;
	surface2 = surf2;
	Sat = s;
};

void SaturnAltimeter::DrawNeedle(HDC hDC, int x, int y, double rad, double angle, HPEN pen0, HPEN pen1)
{
	//
    //Needle function by Rob Conley from Mercury code
    //
	double dx = rad * cos(angle), dy = rad * sin(angle);
	HGDIOBJ oldObj;

	oldObj = SelectObject(hDC, pen1);
	MoveToEx(hDC, x, y, 0); LineTo(hDC, x + (int)(0.85*dx + 0.5), y - (int)(0.85*dy + 0.5));
	SelectObject(hDC, oldObj);
	oldObj = SelectObject(hDC, pen0);
	MoveToEx(hDC, x, y, 0); LineTo(hDC, x + (int)(dx + 0.5), y - (int)(dy + 0.5));
	SelectObject(hDC, oldObj);
}

//
// Altimeter Needle function by Rob Conley from Mercury code, Heavily modified to have non linear gauge range... :):)
//

void SaturnAltimeter::RedrawPanel_Alt(SURFHANDLE surf)
{
	double alpha;
	double range;
	double press;

	press = Sat->GetAtmPressure();
	alpha = Sat->GetAltitude();
	alpha = alpha / 0.3048;

#define ALTIMETER_X_CENTER	68
#define ALTIMETER_Y_CENTER	69
#define ALTIMETER_RADIUS	55.0

	//sprintf(oapiDebugString(), "altitude %f", alpha);
	if (alpha > 55000 || press < 1000.0) alpha = 55000;

	if (alpha < 4001) {
		range = 120 * RAD;
		range = range / 4000;
		alpha = 4000 - alpha;
		HDC hDC = oapiGetDC(surf);
		DrawNeedle(hDC, ALTIMETER_X_CENTER, ALTIMETER_Y_CENTER, ALTIMETER_RADIUS, (alpha*range) + 150 * RAD, g_Param.pen[1], g_Param.pen[4]);//(alpha * range)
		oapiReleaseDC(surf, hDC);
	}
	else if (alpha > 4001 && alpha < 6001) {
		range = 35 * RAD;
		range = range / 2000;
		alpha = 2000 - alpha;
		HDC hDC = oapiGetDC(surf);
		DrawNeedle(hDC, ALTIMETER_X_CENTER, ALTIMETER_Y_CENTER, ALTIMETER_RADIUS, (alpha*range) + 185 * RAD, g_Param.pen[1], g_Param.pen[4]);//(alpha * range)
		oapiReleaseDC(surf, hDC);
	}
	else if (alpha > 6001 && alpha < 8001) {
		range = 25 * RAD;
		range = range / 2000;
		alpha = 2000 - alpha;
		HDC hDC = oapiGetDC(surf);
		DrawNeedle(hDC, ALTIMETER_X_CENTER, ALTIMETER_Y_CENTER, ALTIMETER_RADIUS, (alpha*range) + 165 * RAD, g_Param.pen[1], g_Param.pen[4]);//(alpha * range)
		oapiReleaseDC(surf, hDC);
	}
	else if (alpha > 8001 && alpha < 10001) {
		range = 30 * RAD;
		range = range / 2000;
		alpha = 2000 - alpha;
		HDC hDC = oapiGetDC(surf);
		DrawNeedle(hDC, ALTIMETER_X_CENTER, ALTIMETER_Y_CENTER, ALTIMETER_RADIUS, (alpha*range) + 180 * RAD, g_Param.pen[1], g_Param.pen[4]);//(alpha * range)
		oapiReleaseDC(surf, hDC);
	}
	else if (alpha > 10001 && alpha < 20001) {
		range = 45 * RAD;
		range = range / 10000;
		alpha = 10000 - alpha;
		HDC hDC = oapiGetDC(surf);
		DrawNeedle(hDC, ALTIMETER_X_CENTER, ALTIMETER_Y_CENTER, ALTIMETER_RADIUS, (alpha*range) + 60 * RAD, g_Param.pen[1], g_Param.pen[4]);//(alpha * range)
		oapiReleaseDC(surf, hDC);
	}
	else if (alpha > 20001 && alpha < 40001) {
		range = 65 * RAD;
		range = range / 20000;
		alpha = 20000 - alpha;
		HDC hDC = oapiGetDC(surf);
		DrawNeedle(hDC, ALTIMETER_X_CENTER, ALTIMETER_Y_CENTER, ALTIMETER_RADIUS, (alpha*range) + 15 * RAD, g_Param.pen[1], g_Param.pen[4]);//(alpha * range)
		oapiReleaseDC(surf, hDC);
	}
	else {
		range = 20 * RAD;
		range = range / 10000;
		alpha = 10000 - alpha;
		HDC hDC = oapiGetDC(surf);
		DrawNeedle(hDC, ALTIMETER_X_CENTER, ALTIMETER_Y_CENTER, ALTIMETER_RADIUS, (alpha*range) + 10 * RAD, g_Param.pen[1], g_Param.pen[4]);//(alpha * range)
		oapiReleaseDC(surf, hDC);
	}
	oapiBlt(surf, surface1, 0, 0, 0, 0, 137, 137, SURF_PREDEF_CK);
}

void SaturnAltimeter::RedrawPanel_Alt2(SURFHANDLE surf)
{
	double alpha;
	double range;

	alpha = Sat->GetAltitude();
	alpha = alpha / 0.305;

#define ALTIMETER2_X_CENTER	80
#define ALTIMETER2_Y_CENTER	80
#define ALTIMETER2_RADIUS	70.0

	//sprintf(oapiDebugString(), "altitude %f", alpha);
	if (alpha > 50000) alpha = 50000;

	if (alpha < 4001) {
		range = 120 * RAD;
		range = range / 4000;
		alpha = 4000 - alpha;
		HDC hDC = oapiGetDC(surf);
		DrawNeedle(hDC, ALTIMETER2_X_CENTER, ALTIMETER2_Y_CENTER, ALTIMETER2_RADIUS, (alpha*range) + 150 * RAD, g_Param.pen[1], g_Param.pen[4]);//(alpha * range)
		oapiReleaseDC(surf, hDC);
	}
	else if (alpha > 4001 && alpha < 6001) {
		range = 35 * RAD;
		range = range / 2000;
		alpha = 2000 - alpha;
		HDC hDC = oapiGetDC(surf);
		DrawNeedle(hDC, ALTIMETER2_X_CENTER, ALTIMETER2_Y_CENTER, ALTIMETER2_RADIUS, (alpha*range) + 185 * RAD, g_Param.pen[1], g_Param.pen[4]);//(alpha * range)
		oapiReleaseDC(surf, hDC);
	}
	else if (alpha > 6001 && alpha < 8001) {
		range = 25 * RAD;
		range = range / 2000;
		alpha = 2000 - alpha;
		HDC hDC = oapiGetDC(surf);
		DrawNeedle(hDC, ALTIMETER2_X_CENTER, ALTIMETER2_Y_CENTER, ALTIMETER2_RADIUS, (alpha*range) + 165 * RAD, g_Param.pen[1], g_Param.pen[4]);//(alpha * range)
		oapiReleaseDC(surf, hDC);
	}
	else if (alpha > 8001 && alpha < 10001) {
		range = 20 * RAD;
		range = range / 2000;
		alpha = 2000 - alpha;
		HDC hDC = oapiGetDC(surf);
		DrawNeedle(hDC, ALTIMETER2_X_CENTER, ALTIMETER2_Y_CENTER, ALTIMETER2_RADIUS, (alpha*range) + 150 * RAD, g_Param.pen[1], g_Param.pen[4]);//(alpha * range)
		oapiReleaseDC(surf, hDC);
	}
	else if (alpha > 10001 && alpha < 20001) {
		range = 55 * RAD;
		range = range / 10000;
		alpha = 10000 - alpha;
		HDC hDC = oapiGetDC(surf);
		DrawNeedle(hDC, ALTIMETER2_X_CENTER, ALTIMETER2_Y_CENTER, ALTIMETER2_RADIUS, (alpha*range) + 70 * RAD, g_Param.pen[1], g_Param.pen[4]);//(alpha * range)
		oapiReleaseDC(surf, hDC);
	}
	else if (alpha > 20001 && alpha < 40001) {
		range = 65 * RAD;
		range = range / 20000;
		alpha = 20000 - alpha;
		HDC hDC = oapiGetDC(surf);
		DrawNeedle(hDC, ALTIMETER2_X_CENTER, ALTIMETER2_Y_CENTER, ALTIMETER2_RADIUS, (alpha*range) + 15 * RAD, g_Param.pen[1], g_Param.pen[4]);//(alpha * range)
		oapiReleaseDC(surf, hDC);
	}
	else {
		range = 20 * RAD;
		range = range / 10000;
		alpha = 10000 - alpha;
		HDC hDC = oapiGetDC(surf);
		DrawNeedle(hDC, ALTIMETER2_X_CENTER, ALTIMETER2_Y_CENTER, ALTIMETER2_RADIUS, (alpha*range) + 10 * RAD, g_Param.pen[1], g_Param.pen[4]);//(alpha * range)
		oapiReleaseDC(surf, hDC);
	}
	oapiBlt(surf, surface2, 0, 0, 0, 0, 161, 161, SURF_PREDEF_CK);
}

void SaturnAltimeter::DrawSwitchVC(int id, int event, SURFHANDLE surf) {

	double alpha;
	double range;
	double press;

	double value = 0.0;

	press = Sat->GetAtmPressure();
	alpha = Sat->GetAltitude();
	alpha = alpha / 0.3048;

	//sprintf(oapiDebugString(), "altitude %f", alpha);
	if (alpha > 55000 || press < 1000.0) alpha = 55000;

	if (alpha < 4001) {
		range = 120 * RAD;
		range = range / 4000;
		alpha = 4000 - alpha;
		value = (alpha*range) + 150 * RAD; //(alpha * range)
	}
	else if (alpha > 4001 && alpha < 6001) {
		range = 35 * RAD;
		range = range / 2000;
		alpha = 2000 - alpha;
		value = (alpha*range) + 185 * RAD; //(alpha * range)
	}
	else if (alpha > 6001 && alpha < 8001) {
		range = 25 * RAD;
		range = range / 2000;
		alpha = 2000 - alpha;
		value = (alpha*range) + 165 * RAD; //(alpha * range)
	}
	else if (alpha > 8001 && alpha < 10001) {
		range = 30 * RAD;
		range = range / 2000;
		alpha = 2000 - alpha;
		value = (alpha*range) + 180 * RAD; //(alpha * range)
	}
	else if (alpha > 10001 && alpha < 20001) {
		range = 45 * RAD;
		range = range / 10000;
		alpha = 10000 - alpha;
		value = (alpha*range) + 60 * RAD; //(alpha * range)
	}
	else if (alpha > 20001 && alpha < 40001) {
		range = 65 * RAD;
		range = range / 20000;
		alpha = 20000 - alpha;
		value = (alpha*range) + 15 * RAD; //(alpha * range)
	}
	else {
		range = 20 * RAD;
		range = range / 10000;
		alpha = 10000 - alpha;
		value = (alpha*range) + 10 * RAD; //(alpha * range)
	}
	value = value + 90 * RAD;
	value = (value * DEG) / 360;
	if (value < 0) value = 0;
	if (value > 1) value = 1;
	Sat->SetAnimation(animNeedle, value);
	//sprintf(oapiDebugString(), "Alt %lf", Sat->GetAltitude() / 0.3048);
}

void SaturnAltimeter::DefineVCAnimations(UINT vc_idx)
{
	ANIMATIONCOMPONENT_HANDLE ach_needleAnim;
	const VECTOR3 Needle_AltimeterLocation = { -0.5245, 0.9144, 0.4308 };
	const VECTOR3 Needle_AltimeterAxis = { 0.0, -0.316903546411375, 0.948457770420958 };
	static UINT	meshgroup_needle = { VC_GRP_Needle_Altimeter };
	static MGROUP_ROTATE mgt_needleAnim(vc_idx, &meshgroup_needle, 1, Needle_AltimeterLocation, Needle_AltimeterAxis, (float)(360.0*RAD));
	animNeedle = Sat->CreateAnimation(0.0);
	ach_needleAnim = Sat->AddAnimationComponent(animNeedle, 0.0f, 1.0f, &mgt_needleAnim);
	Sat->SetAnimation(animNeedle, 0.0);
}

CSMPanel::CSMPanel()
{

}

void CSMPanel::Init(Saturn *v, PanelSwitches &panel, e_object *p)
{
	SwitchRow::Init(-1, panel, p);
	sat = v;
}

void CSMPanel1::RegisterPanelAreas()
{
	//TBD: Master Alarm (S1)
	RegisterPanelArea(&sat->CMCAttSwitch, _R(343, 635, 377, 664), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S2
	RegisterPanelArea(&sat->FDAIScaleSwitch, _R(265, 742, 299, 771), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S3
	RegisterPanelArea(&sat->FDAISelectSwitch, _R(308, 742, 342, 771), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S4
	RegisterPanelArea(&sat->FDAISourceSwitch, _R(407, 742, 441, 771), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S5
	RegisterPanelArea(&sat->FDAIAttSetSwitch, _R(450, 742, 484, 771), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S6

	RegisterPanelArea(&sat->ManualAttRollSwitch, _R(190, 838, 224, 867), PANEL_MOUSE_DOWN); //S7
	RegisterPanelArea(&sat->ManualAttPitchSwitch, _R(233, 838, 267, 867), PANEL_MOUSE_DOWN); //S8
	RegisterPanelArea(&sat->ManualAttYawSwitch, _R(276, 838, 310, 867), PANEL_MOUSE_DOWN); //S9
	RegisterPanelArea(&sat->LimitCycleSwitch, _R(319, 838, 353, 867), PANEL_MOUSE_DOWN); //S10
	RegisterPanelArea(&sat->AttDeadbandSwitch, _R(362, 838, 396, 867), PANEL_MOUSE_DOWN); //S11
	RegisterPanelArea(&sat->AttRateSwitch, _R(405, 838, 439, 867), PANEL_MOUSE_DOWN); //S12
	RegisterPanelArea(&sat->TransContrSwitch, _R(448, 838, 482, 867), PANEL_MOUSE_DOWN); //S66

	RegisterPanelArea(&sat->RotPowerNormal1Switch, _R(104, 948, 138, 977), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S64
	RegisterPanelArea(&sat->RotPowerNormal2Switch, _R(163, 948, 197, 977), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S65
	RegisterPanelArea(&sat->RotPowerDirect1Switch, _R(222, 948, 256, 977), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S13
	RegisterPanelArea(&sat->RotPowerDirect2Switch, _R(290, 948, 324, 977), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S67

	RegisterPanelArea(&sat->BMAGRollSwitch, _R(125, 1036, 159, 1065), PANEL_MOUSE_DOWN); //S20
	RegisterPanelArea(&sat->BMAGPitchSwitch, _R(174, 1036, 208, 1065), PANEL_MOUSE_DOWN); //S21
	RegisterPanelArea(&sat->BMAGYawSwitch, _R(224, 1036, 258, 1065), PANEL_MOUSE_DOWN); //S22

	RegisterPanelArea(&sat->SCContSwitch, _R(383, 948, 417, 977), PANEL_MOUSE_DOWN); //S18
	RegisterPanelArea(&sat->CMCModeSwitch, _R(426, 948, 460, 977), PANEL_MOUSE_DOWN); //S19

	RegisterPanelArea(&sat->EventTimerUpDownSwitch, _R(787, 1260, 821, 1289), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S55
	RegisterPanelArea(&sat->EventTimerContSwitch, _R(830, 1260, 864, 1289), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S56
	RegisterPanelArea(&sat->EventTimerMinutesSwitch, _R(873, 1260, 907, 1289), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S57
	RegisterPanelArea(&sat->EventTimerSecondsSwitch, _R(916, 1260, 950, 1289), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S58

	RegisterPanelArea(&sat->SCSTvcPitchSwitch, _R(380, 1173, 414, 1202), PANEL_MOUSE_DOWN); //S38
	RegisterPanelArea(&sat->SCSTvcYawSwitch, _R(423, 1173, 457, 1202), PANEL_MOUSE_DOWN); //S39

	RegisterPanelArea(&sat->Pitch1Switch, _R(472, 1173, 506, 1202), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S40
	RegisterPanelArea(&sat->Pitch2Switch, _R(521, 1173, 555, 1202), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S41
	RegisterPanelArea(&sat->Yaw1Switch, _R(570, 1173, 604, 1202), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S42
	RegisterPanelArea(&sat->Yaw2Switch, _R(620, 1173, 654, 1202), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S43

	RegisterPanelArea(&sat->IMUGuardedCageSwitch, _R(289, 1237, 325, 1306), PANEL_MOUSE_DOWN); //S49

	RegisterPanelArea(&sat->EMSRollSwitch, _R(336, 1260, 370, 1289), PANEL_MOUSE_DOWN); //S50
	RegisterPanelArea(&sat->GSwitch, _R(379, 1260, 413, 1289), PANEL_MOUSE_DOWN); //S51

	RegisterPanelArea(&sat->LVSPSPcIndicatorSwitch, _R(422, 1260, 456, 1289), PANEL_MOUSE_DOWN); //S51
	RegisterPanelArea(&sat->LVFuelTankPressIndicatorSwitch, _R(465, 1260, 499, 1289), PANEL_MOUSE_DOWN); //S52

	RegisterPanelArea(&sat->TVCGimbalDrivePitchSwitch, _R(508, 1260, 542, 1289), PANEL_MOUSE_DOWN); //S27
	RegisterPanelArea(&sat->TVCGimbalDriveYawSwitch, _R(551, 1260, 585, 1289), PANEL_MOUSE_DOWN); //S28

	RegisterPanelArea(&sat->dVThrust1Switch, _R(442, 1046, 478, 1115), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S26
	RegisterPanelArea(&sat->dVThrust2Switch, _R(536, 1046, 572, 1115), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S59

	RegisterPanelArea(&sat->GDCAlignButton, _R(290, 1169, 329, 1207), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S37

	RegisterPanelArea(&sat->DirectUllageButton, _R(370, 1037, 409, 1075), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S24
	RegisterPanelArea(&sat->ThrustOnButton, _R(370, 1090, 409, 1128), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S25

	RegisterPanelArea(&sat->THCRotary, _R(0, 1251, 72, 1360), PANEL_MOUSE_DOWN);

	RegisterPanelArea(&sat->CGSwitch, _R(702, 1173, 736, 1202), PANEL_MOUSE_DOWN); //S54
	RegisterPanelArea(&sat->ELSLogicSwitch, _R(745, 1150, 781, 1219), PANEL_MOUSE_DOWN); //S44
	RegisterPanelArea(&sat->ELSAutoSwitch, _R(790, 1173, 824, 1202), PANEL_MOUSE_DOWN); //S63
	RegisterPanelArea(&sat->CMRCSLogicSwitch, _R(833, 1173, 867, 1202), PANEL_MOUSE_DOWN); //S46
	RegisterPanelArea(&sat->CMPropDumpSwitch, _R(876, 1150, 912, 1219), PANEL_MOUSE_DOWN); //S47
	RegisterPanelArea(&sat->CMPropPurgeSwitch, _R(921, 1150, 957, 1219), PANEL_MOUSE_DOWN); //S48

	RegisterPanelArea(&sat->EMSFunctionSwitch, _R(595, 280, 685, 370), PANEL_MOUSE_DOWN);
	RegisterPanelArea(&sat->EMSModeSwitch, _R(593, 402, 628, 432), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP);
	RegisterPanelArea(&sat->GTASwitch, _R(904, 291, 959, 402), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP);

	RegisterPanelArea(&sat->SPSswitch, _R(299, 1051, 337, 1103), PANEL_MOUSE_DOWN); //S23

	RegisterPanelArea(&sat->GPFPIPitch1Meter, _R(644, 927, 663, 1032), PANEL_MOUSE_IGNORE);
	RegisterPanelArea(&sat->GPFPIPitch2Meter, _R(682, 927, 701, 1032), PANEL_MOUSE_IGNORE);
	RegisterPanelArea(&sat->GPFPIYaw1Meter, _R(720, 927, 739, 1032), PANEL_MOUSE_IGNORE);
	RegisterPanelArea(&sat->GPFPIYaw2Meter, _R(758, 927, 777, 1032), PANEL_MOUSE_IGNORE);
}

void CSMPanel2::RegisterPanelAreasLeft(int offset)
{
	RegisterPanelArea(&sat->EDSSwitch,				_R(1087 + offset, 958, 1121 + offset, 987),		PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S67
	RegisterPanelArea(&sat->CsmLmFinalSep1Switch,	_R(1129 + offset, 935, 1165 + offset, 1004),	PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S109
	RegisterPanelArea(&sat->CsmLmFinalSep2Switch,	_R(1173 + offset, 935, 1209 + offset, 1004),	PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S112
	RegisterPanelArea(&sat->CmSmSep1Switch,			_R(1217 + offset, 935, 1253 + offset, 1004),	PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S110
	RegisterPanelArea(&sat->CmSmSep2Switch,			_R(1261 + offset, 935, 1297 + offset, 1004),	PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S111
	if (!sat->SkylabCM)
	{
		RegisterPanelArea(&sat->SIVBPayloadSepSwitch, _R(1305 + offset, 935, 1341 + offset, 1004), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S108
	}

	RegisterPanelArea(&sat->SMRCSHelium1ASwitch, _R(1585 + offset, 430, 1619 + offset, 459), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S24
	RegisterPanelArea(&sat->SMRCSHelium1BSwitch, _R(1628 + offset, 430, 1662 + offset, 459), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S25
	RegisterPanelArea(&sat->SMRCSHelium1CSwitch, _R(1671 + offset, 430, 1705 + offset, 459), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S26
	RegisterPanelArea(&sat->SMRCSHelium1DSwitch, _R(1714 + offset, 430, 1748 + offset, 459), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S27

	RegisterPanelArea(&sat->CMUplinkSwitch, _R(1411 + offset, 587, 1445 + offset, 616), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S21
	if (!sat->SkylabCM)
	{
		RegisterPanelArea(&sat->IUUplinkSwitch, _R(1454 + offset, 587, 1488 + offset, 616), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S115
	}
	RegisterPanelArea(&sat->CMRCSPressSwitch, _R(1497 + offset, 564, 1533 + offset, 633), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S31
	RegisterPanelArea(&sat->SMRCSIndSwitch, _R(1542 + offset, 587, 1576 + offset, 616), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S98
	RegisterPanelArea(&sat->SMRCSHelium2ASwitch, _R(1585 + offset, 587, 1619 + offset, 616), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S34
	RegisterPanelArea(&sat->SMRCSHelium2BSwitch, _R(1628 + offset, 587, 1662 + offset, 616), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S35
	RegisterPanelArea(&sat->SMRCSHelium2CSwitch, _R(1671 + offset, 587, 1705 + offset, 616), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S36
	RegisterPanelArea(&sat->SMRCSHelium2DSwitch, _R(1714 + offset, 587, 1748 + offset, 616), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S37

	RegisterPanelArea(&sat->LVGuidanceSwitch, _R(1043 + offset, 1138, 1079 + offset, 1207), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S69

	if (!sat->SkylabCM)
	{
		RegisterPanelArea(&sat->SIISIVBSepSwitch, _R(1090 + offset, 1138, 1126 + offset, 1207), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S68
		RegisterPanelArea(&sat->TLIEnableSwitch, _R(1139 + offset, 1161, 1173 + offset, 1190), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S70
	}

	RegisterPanelArea(&sat->MainReleaseSwitch, _R(1042 + offset, 1228, 1078 + offset, 1297), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S71

	RegisterPanelArea(&sat->SMRCSHeaterASwitch, _R(1413 + offset, 718, 1447 + offset, 747), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S92
	RegisterPanelArea(&sat->SMRCSHeaterBSwitch, _R(1456 + offset, 718, 1490 + offset, 747), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S94
	RegisterPanelArea(&sat->SMRCSHeaterCSwitch, _R(1499 + offset, 718, 1533 + offset, 747), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S93
	RegisterPanelArea(&sat->SMRCSHeaterDSwitch, _R(1542 + offset, 718, 1576 + offset, 747), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S95
	RegisterPanelArea(&sat->SMRCSProp1ASwitch, _R(1585 + offset, 718, 1619 + offset, 747), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S47
	RegisterPanelArea(&sat->SMRCSProp1BSwitch, _R(1628 + offset, 718, 1662 + offset, 747), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S48
	RegisterPanelArea(&sat->SMRCSProp1CSwitch, _R(1671 + offset, 718, 1705 + offset, 747), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S49
	RegisterPanelArea(&sat->SMRCSProp1DSwitch, _R(1714 + offset, 718, 1748 + offset, 747), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S50

	RegisterPanelArea(&sat->RCSCMDSwitch, _R(1413 + offset, 848, 1447 + offset, 877), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S43
	RegisterPanelArea(&sat->RCSTrnfrSwitch, _R(1456 + offset, 848, 1490 + offset, 877), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S44
	RegisterPanelArea(&sat->CMRCSProp1Switch, _R(1499 + offset, 848, 1533 + offset, 877), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S45
	RegisterPanelArea(&sat->CMRCSProp2Switch, _R(1542 + offset, 848, 1576 + offset, 877), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S46
	RegisterPanelArea(&sat->SMRCSProp2ASwitch, _R(1585 + offset, 848, 1619 + offset, 877), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S116
	RegisterPanelArea(&sat->SMRCSProp2BSwitch, _R(1628 + offset, 848, 1662 + offset, 877), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S117
	RegisterPanelArea(&sat->SMRCSProp2CSwitch, _R(1671 + offset, 848, 1705 + offset, 877), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S118
	RegisterPanelArea(&sat->SMRCSProp2DSwitch, _R(1714 + offset, 848, 1748 + offset, 877), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S119

	RegisterPanelArea(&sat->PropDumpAutoSwitch, _R(1042 + offset, 1050, 1076 + offset, 1079), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S63
	RegisterPanelArea(&sat->TwoEngineOutAutoSwitch, _R(1091 + offset, 1050, 1125 + offset, 1079), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S64
	RegisterPanelArea(&sat->LVRateAutoSwitch, _R(1152 + offset, 1050, 1186 + offset, 1079), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S65
	RegisterPanelArea(&sat->TowerJett1Switch, _R(1210 + offset, 1027, 1246 + offset, 1096), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S66
	RegisterPanelArea(&sat->TowerJett2Switch, _R(1258 + offset, 1027, 1294 + offset, 1096), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S96

	RegisterPanelArea(&sat->DockingProbeExtdRelSwitch, _R(1388 + offset, 256, 1424 + offset, 325), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S1
	RegisterPanelArea(&sat->DockingProbeRetractPrimSwitch, _R(1432 + offset, 279, 1466 + offset, 308), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S2
	RegisterPanelArea(&sat->DockingProbeRetractSecSwitch, _R(1475 + offset, 279, 1509 + offset, 308), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S3

	RegisterPanelArea(&sat->RunEVALightSwitch, _R(1518 + offset, 279, 1552 + offset, 308), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S5
	RegisterPanelArea(&sat->RndzLightSwitch, _R(1575 + offset, 279, 1609 + offset, 308), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S114
	RegisterPanelArea(&sat->TunnelLightSwitch, _R(1629 + offset, 279, 1663 + offset, 308), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S6
	RegisterPanelArea(&sat->LMPowerSwitch, _R(1692 + offset, 279, 1726 + offset, 308), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S82

	RegisterPanelArea(&sat->RCSTempMeter, _R(1389 + offset, 385, 1399 + offset, 511), PANEL_MOUSE_IGNORE); //M5
	RegisterPanelArea(&sat->RCSHeliumPressMeter, _R(1444 + offset, 385, 1454 + offset, 511), PANEL_MOUSE_IGNORE); //M5
	RegisterPanelArea(&sat->RCSFuelPressMeter, _R(1480 + offset, 385, 1490 + offset, 511), PANEL_MOUSE_IGNORE); //M5
	RegisterPanelArea(&sat->RCSQuantityMeter, _R(1535 + offset, 385, 1545 + offset, 511), PANEL_MOUSE_IGNORE); //M5

	RegisterPanelArea(&sat->SMRCSHelium1ATalkback, _R(1591 + offset, 368, 1614 + offset, 391), PANEL_MOUSE_IGNORE); //DS2
	RegisterPanelArea(&sat->SMRCSHelium1BTalkback, _R(1633 + offset, 368, 1656 + offset, 391), PANEL_MOUSE_IGNORE); //DS3
	RegisterPanelArea(&sat->SMRCSHelium1CTalkback, _R(1676 + offset, 368, 1699 + offset, 391), PANEL_MOUSE_IGNORE); //DS4
	RegisterPanelArea(&sat->SMRCSHelium1DTalkback, _R(1718 + offset, 368, 1741 + offset, 391), PANEL_MOUSE_IGNORE); //DS5

	RegisterPanelArea(&sat->SMRCSHelium2ATalkback, _R(1591 + offset, 526, 1614 + offset, 549), PANEL_MOUSE_IGNORE); //DS8
	RegisterPanelArea(&sat->SMRCSHelium2BTalkback, _R(1633 + offset, 526, 1656 + offset, 549), PANEL_MOUSE_IGNORE); //DS9
	RegisterPanelArea(&sat->SMRCSHelium2CTalkback, _R(1676 + offset, 526, 1699 + offset, 549), PANEL_MOUSE_IGNORE); //DS10
	RegisterPanelArea(&sat->SMRCSHelium2DTalkback, _R(1718 + offset, 526, 1741 + offset, 549), PANEL_MOUSE_IGNORE); //DS11

	RegisterPanelArea(&sat->SMRCSProp1ATalkback, _R(1591 + offset, 659, 1614 + offset, 682), PANEL_MOUSE_IGNORE); //DS14
	RegisterPanelArea(&sat->SMRCSProp1BTalkback, _R(1633 + offset, 659, 1656 + offset, 682), PANEL_MOUSE_IGNORE); //DS15
	RegisterPanelArea(&sat->SMRCSProp1CTalkback, _R(1676 + offset, 659, 1699 + offset, 682), PANEL_MOUSE_IGNORE); //DS16
	RegisterPanelArea(&sat->SMRCSProp1DTalkback, _R(1718 + offset, 659, 1741 + offset, 682), PANEL_MOUSE_IGNORE); //DS17

	RegisterPanelArea(&sat->CMRCSProp1Talkback, _R(1503 + offset, 792, 1526 + offset, 815), PANEL_MOUSE_IGNORE); //DS12
	RegisterPanelArea(&sat->CMRCSProp2Talkback, _R(1545 + offset, 792, 1568 + offset, 815), PANEL_MOUSE_IGNORE); //DS13
	RegisterPanelArea(&sat->SMRCSProp2ATalkback, _R(1588 + offset, 792, 1611 + offset, 815), PANEL_MOUSE_IGNORE); //DS25
	RegisterPanelArea(&sat->SMRCSProp2BTalkback, _R(1631 + offset, 792, 1654 + offset, 815), PANEL_MOUSE_IGNORE); //DS24
	RegisterPanelArea(&sat->SMRCSProp2CTalkback, _R(1674 + offset, 792, 1697 + offset, 815), PANEL_MOUSE_IGNORE); //DS27
	RegisterPanelArea(&sat->SMRCSProp2DTalkback, _R(1716 + offset, 792, 1739 + offset, 815), PANEL_MOUSE_IGNORE); //DS26

	RegisterPanelArea(&sat->DockingProbeAIndicator, _R(1396 + offset, 179, 1419 + offset, 202), PANEL_MOUSE_IGNORE); //DS1
	RegisterPanelArea(&sat->DockingProbeBIndicator, _R(1396 + offset, 206, 1419 + offset, 229), PANEL_MOUSE_IGNORE); //DS21
}

void CSMPanel2::RegisterPanelAreasRight(int offset)
{
	RegisterPanelArea(&sat->CautionWarningModeSwitch, _R(1915 + offset, 400, 1949 + offset, 434), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S7
	RegisterPanelArea(&sat->CautionWarningCMCSMSwitch, _R(1963 + offset, 400, 1997 + offset, 434), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S8
	RegisterPanelArea(&sat->CautionWarningPowerSwitch, _R(2012 + offset, 400, 2046 + offset, 434), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S9
	RegisterPanelArea(&sat->CautionWarningLightTestSwitch, _R(2055 + offset, 400, 2089 + offset, 434), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S79
	RegisterPanelArea(&sat->MissionTimerSwitch, _R(2098 + offset, 400, 2132 + offset, 434), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S100

	RegisterPanelArea(&sat->MissionTimerHoursSwitch, _R(2019 + offset, 299, 2053 + offset, 328), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S101
	RegisterPanelArea(&sat->MissionTimerMinutesSwitch, _R(2062 + offset, 299, 2096 + offset, 328), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S102
	RegisterPanelArea(&sat->MissionTimerSecondsSwitch, _R(2105 + offset, 299, 2139 + offset, 328), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S103

	RegisterPanelArea(&sat->ECSIndicatorsSwitch, _R(1785 + offset, 582, 1875 + offset, 672), PANEL_MOUSE_DOWN); //S89

	RegisterPanelArea(&sat->EcsRadiatorsFlowContAutoSwitch, _R(1796 + offset, 743, 1830 + offset, 772), PANEL_MOUSE_DOWN); //S84
	RegisterPanelArea(&sat->EcsRadiatorsFlowContPwrSwitch,	_R(1846 + offset, 743, 1880 + offset, 772), PANEL_MOUSE_DOWN); //S85
	RegisterPanelArea(&sat->EcsRadiatorsManSelSwitch,		_R(1896 + offset, 743, 1930 + offset, 772), PANEL_MOUSE_DOWN); //S86
	RegisterPanelArea(&sat->EcsRadiatorsHeaterPrimSwitch,	_R(1946 + offset, 743, 1980 + offset, 772), PANEL_MOUSE_DOWN); //S87
	RegisterPanelArea(&sat->EcsRadiatorsHeaterSecSwitch,	_R(1989 + offset, 743, 2023 + offset, 772), PANEL_MOUSE_DOWN); //S88

	RegisterPanelArea(&sat->PotH2oHtrSwitch,					_R(1787 + offset, 848, 1821 + offset, 877), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S42
	RegisterPanelArea(&sat->SuitCircuitH2oAccumAutoSwitch,		_R(1830 + offset, 848, 1864 + offset, 877), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S51
	RegisterPanelArea(&sat->SuitCircuitH2oAccumOnSwitch,		_R(1873 + offset, 848, 1907 + offset, 877), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S52
	RegisterPanelArea(&sat->SuitCircuitHeatExchSwitch,			_R(1916 + offset, 848, 1950 + offset, 877), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S57
	RegisterPanelArea(&sat->SecCoolantLoopEvapSwitch,			_R(1959 + offset, 848, 1993 + offset, 877), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S53
	RegisterPanelArea(&sat->SecCoolantLoopPumpSwitch,			_R(2008 + offset, 848, 2042 + offset, 877), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S54
	RegisterPanelArea(&sat->H2oQtyIndSwitch,					_R(2057 + offset, 848, 2091 + offset, 877), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S56
	RegisterPanelArea(&sat->GlycolEvapTempInSwitch,				_R(2100 + offset, 848, 2134 + offset, 877), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S61
	RegisterPanelArea(&sat->GlycolEvapSteamPressAutoManSwitch,	_R(2151 + offset, 848, 2185 + offset, 877), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S59
	RegisterPanelArea(&sat->GlycolEvapSteamPressIncrDecrSwitch, _R(2198 + offset, 848, 2232 + offset, 877), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S60
	RegisterPanelArea(&sat->GlycolEvapH2oFlowSwitch,			_R(2243 + offset, 848, 2277 + offset, 877), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S58
	RegisterPanelArea(&sat->CabinTempAutoManSwitch,				_R(2293 + offset, 848, 2327 + offset, 877), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S62

	RegisterPanelArea(&sat->HighGainAntennaPitchPositionSwitch, _R(2268 + offset, 1016, 2358 + offset, 1116), PANEL_MOUSE_DOWN);
	RegisterPanelArea(&sat->HighGainAntennaYawPositionSwitch, _R(2398 + offset, 1016, 2488 + offset, 1116), PANEL_MOUSE_DOWN);

	RegisterPanelArea(&sat->CabinTempAutoControlSwitch, _R(2441 + offset, 843, 2458 + offset, 879), PANEL_MOUSE_DOWN); //R1

	RegisterPanelArea(&sat->PostLDGVentValveLever, _R(2062 + offset, 76, 2112 + offset, 234), PANEL_MOUSE_DOWN);

	RegisterPanelArea(&sat->GHATrackSwitch, _R(2185 + offset, 943, 2219 + offset, 972), PANEL_MOUSE_DOWN); //S77
	RegisterPanelArea(&sat->GHABeamSwitch, _R(2228 + offset, 943, 2262 + offset, 972), PANEL_MOUSE_DOWN); //S90

	RegisterPanelArea(&sat->GHAPowerSwitch, _R(2381 + offset, 1157, 2415 + offset, 1186), PANEL_MOUSE_DOWN); //S76
	RegisterPanelArea(&sat->GHAServoElecSwitch, _R(2424 + offset, 1157, 2458 + offset, 1186), PANEL_MOUSE_DOWN); //S83

	RegisterPanelArea(&sat->H2Pressure1Meter, _R(2173 + offset, 295, 2183 + offset, 439), PANEL_MOUSE_IGNORE); //M5
	RegisterPanelArea(&sat->H2Pressure2Meter, _R(2226 + offset, 295, 2236 + offset, 439), PANEL_MOUSE_IGNORE); //M5
	RegisterPanelArea(&sat->O2Pressure1Meter, _R(2259 + offset, 295, 2269 + offset, 439), PANEL_MOUSE_IGNORE); //M6
	RegisterPanelArea(&sat->O2Pressure2Meter, _R(2312 + offset, 295, 2322 + offset, 439), PANEL_MOUSE_IGNORE); //M6

	RegisterPanelArea(&sat->H2Quantity1Meter, _R(2345 + offset, 295, 2355 + offset, 439), PANEL_MOUSE_IGNORE); //M7
	RegisterPanelArea(&sat->H2Quantity2Meter, _R(2398 + offset, 295, 2408 + offset, 439), PANEL_MOUSE_IGNORE); //M7
	RegisterPanelArea(&sat->O2Quantity1Meter, _R(2431 + offset, 295, 2441 + offset, 439), PANEL_MOUSE_IGNORE); //M8
	RegisterPanelArea(&sat->O2Quantity2Meter, _R(2484 + offset, 295, 2494 + offset, 439), PANEL_MOUSE_IGNORE); //M8

	RegisterPanelArea(&sat->CabinFan1Switch, _R(1912 + offset, 490, 1946 + offset, 519), PANEL_MOUSE_DOWN); //S29
	RegisterPanelArea(&sat->CabinFan2Switch, _R(1971 + offset, 490, 2005 + offset, 519), PANEL_MOUSE_DOWN); //S30
	RegisterPanelArea(&sat->H2Heater1Switch, _R(2026 + offset, 490, 2060 + offset, 519), PANEL_MOUSE_DOWN); //S12
	RegisterPanelArea(&sat->H2Heater2Switch, _R(2069 + offset, 490, 2103 + offset, 519), PANEL_MOUSE_DOWN); //S13
	RegisterPanelArea(&sat->O2Heater1Switch, _R(2112 + offset, 490, 2146 + offset, 519), PANEL_MOUSE_DOWN); //S14
	RegisterPanelArea(&sat->O2Heater2Switch, _R(2162 + offset, 490, 2196 + offset, 519), PANEL_MOUSE_DOWN); //S15
	RegisterPanelArea(&sat->O2PressIndSwitch, _R(2205 + offset, 490, 2239 + offset, 519), PANEL_MOUSE_DOWN); //S16
	RegisterPanelArea(&sat->H2Fan1Switch, _R(2261 + offset, 490, 2295 + offset, 519), PANEL_MOUSE_DOWN); //S17
	RegisterPanelArea(&sat->H2Fan2Switch, _R(2325 + offset, 490, 2359 + offset, 519), PANEL_MOUSE_DOWN); //S18
	RegisterPanelArea(&sat->O2Fan1Switch, _R(2390 + offset, 490, 2424 + offset, 519), PANEL_MOUSE_DOWN); //S19
	RegisterPanelArea(&sat->O2Fan2Switch, _R(2453 + offset, 490, 2487 + offset, 519), PANEL_MOUSE_DOWN); //S20

	RegisterPanelArea(&sat->SuitTempMeter, _R(2279 + offset, 593, 2289 + offset, 717), PANEL_MOUSE_IGNORE); //M13
	RegisterPanelArea(&sat->CabinTempMeter, _R(2331 + offset, 593, 2341 + offset, 717), PANEL_MOUSE_IGNORE); //M13
	RegisterPanelArea(&sat->SuitPressMeter, _R(2379 + offset, 593, 2389 + offset, 717), PANEL_MOUSE_IGNORE); //M14
	RegisterPanelArea(&sat->CabinPressMeter, _R(2431 + offset, 593, 2441 + offset, 717), PANEL_MOUSE_IGNORE); //M14
	RegisterPanelArea(&sat->PartPressCO2Meter, _R(2493 + offset, 593, 2503 + offset, 717), PANEL_MOUSE_IGNORE); //M15

	RegisterPanelArea(&sat->EcsRadiatorIndicator, _R(1799 + offset, 683, 1822 + offset, 706), PANEL_MOUSE_IGNORE); //DS22

	RegisterPanelArea(&sat->RCSIndicatorsSwitch, _R(1785 + offset, 445, 1875 + offset, 535), PANEL_MOUSE_DOWN); //S28
}

void CSMPanel3::RegisterPanelAreas(int offset)
{
	RegisterPanelArea(&sat->FuelCellRadiators1Switch, _R(2816 + offset, 607, 2850 + offset, 637), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S2
	RegisterPanelArea(&sat->FuelCellRadiators2Switch, _R(2859 + offset, 607, 2893 + offset, 637), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S3
	RegisterPanelArea(&sat->FuelCellRadiators3Switch, _R(2902 + offset, 607, 2936 + offset, 637), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S4

	RegisterPanelArea(&sat->FuelCellIndicatorsSwitch, _R(3027 + offset, 627, 3117 + offset, 717), PANEL_MOUSE_DOWN); //S10

	RegisterPanelArea(&sat->FuelCellHeater1Switch, _R(2817 + offset, 695, 2851 + offset, 725), PANEL_MOUSE_DOWN); //S6
	RegisterPanelArea(&sat->FuelCellHeater2Switch, _R(2860 + offset, 695, 2894 + offset, 725), PANEL_MOUSE_DOWN); //S7
	RegisterPanelArea(&sat->FuelCellHeater3Switch, _R(2903 + offset, 695, 2937 + offset, 725), PANEL_MOUSE_DOWN); //S8

	RegisterPanelArea(&sat->FuelCellPurge1Switch, _R(2815 + offset, 817, 2849 + offset, 846), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S11
	RegisterPanelArea(&sat->FuelCellPurge2Switch, _R(2858 + offset, 817, 2892 + offset, 846), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S12
	RegisterPanelArea(&sat->FuelCellPurge3Switch, _R(2901 + offset, 817, 2935 + offset, 846), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S13

	RegisterPanelArea(&sat->MainBusASwitch1, _R(2944 + offset, 817, 2978 + offset, 846), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S14
	RegisterPanelArea(&sat->MainBusASwitch2, _R(2994 + offset, 817, 3028 + offset, 846), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S15
	RegisterPanelArea(&sat->MainBusASwitch3, _R(3045 + offset, 817, 3079 + offset, 846), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S16
	RegisterPanelArea(&sat->MainBusAResetSwitch, _R(3088 + offset, 817, 3122 + offset, 846), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S17

	RegisterPanelArea(&sat->FuelCellReactants1Switch, _R(2800 + offset, 955, 2834 + offset, 984), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S20
	RegisterPanelArea(&sat->FuelCellReactants2Switch, _R(2843 + offset, 955, 2877 + offset, 984), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S21
	RegisterPanelArea(&sat->FuelCellReactants3Switch, _R(2886 + offset, 955, 2920 + offset, 984), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S22

	RegisterPanelArea(&sat->MainBusBSwitch1, _R(2929 + offset, 955, 2963 + offset, 984), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S23
	RegisterPanelArea(&sat->MainBusBSwitch2, _R(2985 + offset, 955, 3019 + offset, 984), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S23
	RegisterPanelArea(&sat->MainBusBSwitch3, _R(3041 + offset, 955, 3075 + offset, 984), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S25
	RegisterPanelArea(&sat->MainBusBResetSwitch, _R(3097 + offset, 955, 3131 + offset, 984), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S26

	RegisterPanelArea(&sat->FCReacsValvesSwitch, _R(2593 + offset, 1251, 2627 + offset, 1280), PANEL_MOUSE_DOWN); //S76
	RegisterPanelArea(&sat->H2PurgeLineSwitch, _R(2636 + offset, 1251, 2670 + offset, 1280), PANEL_MOUSE_DOWN); //S77

	RegisterPanelArea(&sat->SBandNormalXPDRSwitch, _R(2593 + offset, 1050, 2627 + offset, 1079), PANEL_MOUSE_DOWN); //S29
	RegisterPanelArea(&sat->SBandNormalPwrAmpl1Switch, _R(2636 + offset, 1050, 2670 + offset, 1079), PANEL_MOUSE_DOWN); //S31
	RegisterPanelArea(&sat->SBandNormalPwrAmpl2Switch, _R(2679 + offset, 1050, 2713 + offset, 1079), PANEL_MOUSE_DOWN); //S32
	RegisterPanelArea(&sat->SBandNormalMode1Switch, _R(2738 + offset, 1050, 2772 + offset, 1079), PANEL_MOUSE_DOWN); //S33
	RegisterPanelArea(&sat->SBandNormalMode2Switch, _R(2781 + offset, 1050, 2815 + offset, 1079), PANEL_MOUSE_DOWN); //S34
	RegisterPanelArea(&sat->SBandNormalMode3Switch, _R(2824 + offset, 1050, 2858 + offset, 1079), PANEL_MOUSE_DOWN); //S35

	RegisterPanelArea(&sat->VHFAntennaRotarySwitch, _R(2593 + offset, 184, 2683 + offset, 274), PANEL_MOUSE_DOWN); //S1

	RegisterPanelArea(&sat->SPSTestSwitch, _R(2714 + offset, 711, 2748 + offset, 740), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP);
	RegisterPanelArea(&sat->SPSOxidFlowValveSwitch, _R(2640 + offset, 798, 2674 + offset, 827), PANEL_MOUSE_DOWN);
	RegisterPanelArea(&sat->SPSOxidFlowValveSelectorSwitch, _R(2683 + offset, 798, 2717 + offset, 827), PANEL_MOUSE_DOWN);
	RegisterPanelArea(&sat->SPSPugModeSwitch, _R(2726 + offset, 798, 2760 + offset, 827), PANEL_MOUSE_DOWN);

	RegisterPanelArea(&sat->SPSHeliumValveASwitch, _R(2593 + offset, 955, 2627 + offset, 984), PANEL_MOUSE_DOWN); //S18
	RegisterPanelArea(&sat->SPSHeliumValveBSwitch, _R(2636 + offset, 955, 2670 + offset, 984), PANEL_MOUSE_DOWN); //S19
	RegisterPanelArea(&sat->SPSLineHTRSSwitch, _R(2679 + offset, 955, 2713 + offset, 984), PANEL_MOUSE_DOWN); //S75
	RegisterPanelArea(&sat->SPSPressIndSwitch, _R(2757 + offset, 955, 2791 + offset, 984), PANEL_MOUSE_DOWN); //S69

	RegisterPanelArea(&sat->DCIndicatorsRotary, _R(3180 + offset, 896, 3270 + offset, 986), PANEL_MOUSE_DOWN); //S27
	RegisterPanelArea(&sat->BatteryChargeRotary, _R(3336 + offset, 896, 3426 + offset, 986), PANEL_MOUSE_DOWN); //S28
	RegisterPanelArea(&sat->ACIndicatorRotary, _R(3386 + offset, 1205, 3476 + offset, 1295), PANEL_MOUSE_DOWN); //S68

	RegisterPanelArea(&sat->SBandAuxSwitch1, _R(2877 + offset, 1050, 2911 + offset, 1079), PANEL_MOUSE_DOWN); //S36
	RegisterPanelArea(&sat->SBandAuxSwitch2, _R(2930 + offset, 1050, 2964 + offset, 1079), PANEL_MOUSE_DOWN); //S37

	RegisterPanelArea(&sat->UPTLMSwitch1, _R(2983 + offset, 1050, 3017 + offset, 1079), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S38
	RegisterPanelArea(&sat->UPTLMSwitch2, _R(3036 + offset, 1050, 3070 + offset, 1079), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S39

	RegisterPanelArea(&sat->SBandAntennaSwitch1, _R(2593 + offset, 1155, 2627 + offset, 1184), PANEL_MOUSE_DOWN); //S41
	RegisterPanelArea(&sat->SBandAntennaSwitch2, _R(2636 + offset, 1155, 2670 + offset, 1184), PANEL_MOUSE_DOWN); //S40

	RegisterPanelArea(&sat->SquelchAThumbwheel, _R(2745 + offset, 1149, 2762 + offset, 1185), PANEL_MOUSE_DOWN); //R1
	RegisterPanelArea(&sat->SquelchBThumbwheel, _R(2745 + offset, 1247, 2762 + offset, 1283), PANEL_MOUSE_DOWN); //R2

	RegisterPanelArea(&sat->VHFAMASwitch, _R(2838 + offset, 1152, 2872 + offset, 1181), PANEL_MOUSE_DOWN); //S43
	RegisterPanelArea(&sat->VHFAMBSwitch, _R(2889 + offset, 1152, 2923 + offset, 1181), PANEL_MOUSE_DOWN); //S44
	RegisterPanelArea(&sat->RCVOnlySwitch, _R(2940 + offset, 1152, 2974 + offset, 1181), PANEL_MOUSE_DOWN); //S71
	RegisterPanelArea(&sat->VHFBeaconSwitch, _R(2983 + offset, 1152, 3017 + offset, 1181), PANEL_MOUSE_DOWN); //S45
	RegisterPanelArea(&sat->VHFRangingSwitch, _R(3026 + offset, 1152, 3060 + offset, 1181), PANEL_MOUSE_DOWN); //S78

	RegisterPanelArea(&sat->TapeRecorderPCMSwitch, _R(2838 + offset, 1250, 2872 + offset, 1279), PANEL_MOUSE_DOWN); //S49
	RegisterPanelArea(&sat->TapeRecorderRecordSwitch, _R(2881 + offset, 1250, 2915 + offset, 1279), PANEL_MOUSE_DOWN); //S50
	RegisterPanelArea(&sat->TapeRecorderForwardSwitch, _R(2924 + offset, 1250, 2958 + offset, 1279), PANEL_MOUSE_DOWN); //S51

	RegisterPanelArea(&sat->SCESwitch, _R(2967 + offset, 1250, 3001 + offset, 1279), PANEL_MOUSE_DOWN); //S52
	RegisterPanelArea(&sat->PMPSwitch, _R(3010 + offset, 1250, 3044 + offset, 1279), PANEL_MOUSE_DOWN); //S54
	RegisterPanelArea(&sat->PCMBitRateSwitch, _R(3053 + offset, 1250, 3130 + offset, 1279), PANEL_MOUSE_DOWN); //S54

	RegisterPanelArea(&sat->MnA1Switch, _R(3182 + offset, 1050, 3216 + offset, 1079), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S57
	RegisterPanelArea(&sat->MnB2Switch, _R(3245 + offset, 1050, 3279 + offset, 1079), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S58
	RegisterPanelArea(&sat->MnA3Switch, _R(3308 + offset, 1050, 3342 + offset, 1079), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S59
	RegisterPanelArea(&sat->AcBus1Switch1, _R(3182 + offset, 1151, 3216 + offset, 1180), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S60
	RegisterPanelArea(&sat->AcBus1Switch2, _R(3225 + offset, 1151, 3259 + offset, 1180), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S61
	RegisterPanelArea(&sat->AcBus1Switch3, _R(3268 + offset, 1151, 3302 + offset, 1180), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S62
	RegisterPanelArea(&sat->AcBus1ResetSwitch, _R(3311 + offset, 1151, 3345 + offset, 1180), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S63
	RegisterPanelArea(&sat->AcBus2Switch1, _R(3182 + offset, 1250, 3216 + offset, 1279), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S64
	RegisterPanelArea(&sat->AcBus2Switch2, _R(3225 + offset, 1250, 3259 + offset, 1279), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S65
	RegisterPanelArea(&sat->AcBus2Switch3, _R(3268 + offset, 1250, 3302 + offset, 1279), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S66
	RegisterPanelArea(&sat->AcBus2ResetSwitch, _R(3311 + offset, 1250, 3345 + offset, 1279), PANEL_MOUSE_DOWN | PANEL_MOUSE_UP); //S67

	RegisterPanelArea(&sat->SPSTempMeter, _R(2583 + offset, 319, 2593 + offset, 443), PANEL_MOUSE_IGNORE); //M1
	RegisterPanelArea(&sat->SPSHeliumNitrogenPressMeter, _R(2636 + offset, 319, 2646 + offset, 443), PANEL_MOUSE_IGNORE); //M1
	RegisterPanelArea(&sat->SPSFuelPressMeter, _R(2669 + offset, 319, 2679 + offset, 443), PANEL_MOUSE_IGNORE); //M2
	RegisterPanelArea(&sat->SPSOxidPressMeter, _R(2722 + offset, 319, 2732 + offset, 443), PANEL_MOUSE_IGNORE); //M2

	RegisterPanelArea(&sat->FuelCellH2FlowMeter, _R(2763 + offset, 319, 2773 + offset, 443), PANEL_MOUSE_IGNORE); //M3
	RegisterPanelArea(&sat->FuelCellO2FlowMeter, _R(2816 + offset, 319, 2826 + offset, 443), PANEL_MOUSE_IGNORE); //M3
	RegisterPanelArea(&sat->FuelCellTempMeter, _R(2849 + offset, 319, 2859 + offset, 443), PANEL_MOUSE_IGNORE); //M4
	RegisterPanelArea(&sat->FuelCellCondenserTempMeter, _R(2902 + offset, 319, 2912 + offset, 443), PANEL_MOUSE_IGNORE); //M4

	RegisterPanelArea(&sat->PwrAmplTB, _R(3119 + offset, 1042, 3142 + offset, 1065), PANEL_MOUSE_IGNORE); //DS21
	RegisterPanelArea(&sat->DseTapeTB, _R(3119 + offset, 1092, 3142 + offset, 1115), PANEL_MOUSE_IGNORE); //DS22

	RegisterPanelArea(&sat->FuelCellPhIndicator, _R(2822 + offset, 490, 2845 + offset, 513), PANEL_MOUSE_IGNORE); //DS1
	RegisterPanelArea(&sat->FuelCellRadTempIndicator, _R(2996 + offset, 490, 3019 + offset, 513), PANEL_MOUSE_IGNORE); //DS5

	RegisterPanelArea(&sat->FuelCellRadiators1Indicator, _R(2822 + offset, 539, 2845 + offset, 562), PANEL_MOUSE_IGNORE); //DS6
	RegisterPanelArea(&sat->FuelCellRadiators2Indicator, _R(2865 + offset, 539, 2888 + offset, 562), PANEL_MOUSE_IGNORE); //DS7
	RegisterPanelArea(&sat->FuelCellRadiators3Indicator, _R(2908 + offset, 539, 2931 + offset, 562), PANEL_MOUSE_IGNORE); //DS8

	RegisterPanelArea(&sat->FuelCellReactants1Indicator, _R(2823 + offset, 893, 2846 + offset, 917), PANEL_MOUSE_IGNORE); //DS15
	RegisterPanelArea(&sat->FuelCellReactants2Indicator, _R(2866 + offset, 893, 2889 + offset, 917), PANEL_MOUSE_IGNORE); //DS16
	RegisterPanelArea(&sat->FuelCellReactants3Indicator, _R(2909 + offset, 893, 2932 + offset, 917), PANEL_MOUSE_IGNORE); //DS17
	RegisterPanelArea(&sat->MainBusBIndicator1, _R(2952 + offset, 893, 2975 + offset, 917), PANEL_MOUSE_IGNORE); //DS18
	RegisterPanelArea(&sat->MainBusBIndicator2, _R(2995 + offset, 893, 3018 + offset, 917), PANEL_MOUSE_IGNORE); //DS19
	RegisterPanelArea(&sat->MainBusBIndicator3, _R(3038 + offset, 893, 3061 + offset, 917), PANEL_MOUSE_IGNORE); //DS20

	RegisterPanelArea(&sat->MainBusAIndicator1, _R(2953 + offset, 758, 2976 + offset, 781), PANEL_MOUSE_IGNORE); //DS10
	RegisterPanelArea(&sat->MainBusAIndicator2, _R(2996 + offset, 758, 3019 + offset, 781), PANEL_MOUSE_IGNORE); //DS11
	RegisterPanelArea(&sat->MainBusAIndicator3, _R(3039 + offset, 758, 3062 + offset, 781), PANEL_MOUSE_IGNORE); //DS12

	RegisterPanelArea(&sat->SPSInjectorValve1Indicator, _R(2596 + offset, 523, 2630 + offset, 557), PANEL_MOUSE_IGNORE); //M5
	RegisterPanelArea(&sat->SPSInjectorValve2Indicator, _R(2644 + offset, 523, 2678 + offset, 557), PANEL_MOUSE_IGNORE); //M6
	RegisterPanelArea(&sat->SPSInjectorValve3Indicator, _R(2692 + offset, 523, 2726 + offset, 557), PANEL_MOUSE_IGNORE); //M7
	RegisterPanelArea(&sat->SPSInjectorValve4Indicator, _R(2740 + offset, 523, 2774 + offset, 557), PANEL_MOUSE_IGNORE); //M8

	RegisterPanelArea(&sat->SPSOxidFlowValveMaxIndicator, _R(2605 + offset, 792, 2627 + offset, 804), PANEL_MOUSE_IGNORE);
	RegisterPanelArea(&sat->SPSOxidFlowValveMinIndicator, _R(2605 + offset, 822, 2627 + offset, 834), PANEL_MOUSE_IGNORE);

	RegisterPanelArea(&sat->SPSHeliumValveAIndicator, _R(2598 + offset, 896, 2621 + offset, 919), PANEL_MOUSE_IGNORE); //DS13
	RegisterPanelArea(&sat->SPSHeliumValveBIndicator, _R(2641 + offset, 896, 2664 + offset, 919), PANEL_MOUSE_IGNORE); //DS14
}