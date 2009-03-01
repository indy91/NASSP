/***************************************************************************
  This file is part of Project Apollo - NASSP
  Copyright 2004-2007

  Generic Saturn class for AGC interface. As much code as possible will be shared
  here between the SaturnV and Saturn1B.

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

  **************************** Revision History ****************************
  *	$Log$
  *	Revision 1.286  2008/12/07 18:35:21  movieman523
  *	Very basics of DSE telemetry recording: the play/record switch works but nothing else does!
  *	
  *	Also don't try to create a panel if we can't load the bitmap.
  *	
  *	Revision 1.285  2008/10/16 07:12:11  movieman523
  *	Telemetry changes.
  *	
  *	Revision 1.284  2008/10/16 02:26:17  movieman523
  *	Some more telemetry.
  *	
  *	Revision 1.283  2008/10/16 00:48:45  movieman523
  *	More telemetry: some of these are dummy values for now.
  *	
  *	Revision 1.282  2008/10/15 06:00:41  movieman523
  *	Added more telemetry data.
  *	
  *	Revision 1.281  2008/07/13 17:47:13  tschachim
  *	Rearranged realism levels, merged Standard and Quickstart Mode.
  *	
  *	Revision 1.280  2008/06/15 15:22:48  tschachim
  *	Bugfixes switch names and borders
  *	
  *	Revision 1.279  2008/06/01 11:46:58  flydba
  *	New circuit breaker bitmaps added.
  *	
  *	Revision 1.278  2008/05/24 17:22:42  tschachim
  *	Added switch borders.
  *	
  *	Revision 1.277  2008/04/11 12:19:15  tschachim
  *	New SM and CM RCS.
  *	Improved abort handling.
  *	Fixed BasicExcel for VC6, reduced VS2005 warnings, bugfixes.
  *	
  *	Revision 1.276  2008/01/23 01:40:09  lassombra
  *	Implemented timestep functions and event management
  *	
  *	Events for Saturns are now fully implemented
  *	
  *	Removed all hardcoded checklists from Saturns.
  *	
  *	Automatic Checklists are coded into an excel file.
  *	
  *	Added function to get the name of the active checklist.
  *	
  *	ChecklistController is now 100% ready for Saturn.
  *	
  *	Revision 1.275  2008/01/22 05:22:27  movieman523
  *	Added port number to docking probe.
  *	
  *	Revision 1.274  2008/01/22 02:55:08  movieman523
  *	Moved DockConnectors/UndockConnectors into the base class. We now dock/undock all registered connectors on that port.
  *	
  *	Revision 1.273  2008/01/18 05:57:23  movieman523
  *	Moved SIVB creation code into generic Saturn function, and made ASTP sort of start to work.
  *	
  *	Revision 1.272  2008/01/17 01:46:27  movieman523
  *	Renamed LEMName to PayloadName and replaced LEMN with PAYN in the scenario file; reading LEMN is still supported for backward compatibility.
  *	
  *	Revision 1.271  2008/01/16 05:52:07  movieman523
  *	Removed all dockstate code.
  *	
  *	Revision 1.270  2008/01/16 04:14:24  movieman523
  *	Rewrote docking probe separation code and moved the CSM_LEM code into a single function in the Saturn class.
  *	
  *	Revision 1.269  2008/01/14 01:17:07  movieman523
  *	Numerous changes to move payload creation from the CSM to SIVB.
  *	
  *	Revision 1.268  2008/01/11 05:24:12  movieman523
  *	Added LEM fuel masses; currently they're passed to the LEM but it ignores them.
  *	
  *	Revision 1.267  2008/01/09 15:00:20  lassombra
  *	Added support for checklistController to save/load state.
  *	
  *	Added support for new scenario options LEMCHECK <lem checklist file and LEMCHECKAUTO <whether the lem should automatically execute checklists.
  *	
  *	Will document new options on the wiki
  *	
  *	Revision 1.266  2007/12/31 05:05:37  lassombra
  *	Added ChecklistController class as a member of Saturn.
  *	
  *	Updated ChecklistController class to have, near as I can see, all the required data.
  *	
  *	Waiting on updates to the connector in order to be able to procede with complete implementation.
  *	
  *	Revision 1.265  2007/12/29 08:46:39  flydba
  *	Panels 226, 250, 251 and 252 now work.
  *	
  *	Revision 1.264  2007/12/28 05:02:01  flydba
  *	Panel 225 now works.
  *	
  *	Revision 1.263  2007/12/27 07:07:53  flydba
  *	Panels 10, 12, 276, 278 and the part of 229 on the right hand side panel now work.
  *	
  *	Revision 1.262  2007/12/26 04:21:18  flydba
  *	Panel 300, 301, 302, 303 and 305 now works.
  *	
  *	Revision 1.261  2007/12/25 06:16:02  jasonims
  *	Implemented LES covers on CM windows....  covers need to be redone though and pixels tweaked.
  *	
  *	Revision 1.260  2007/12/21 01:00:22  movieman523
  *	Really basic Checklist MFD based on Project Apollo MFD, along with the various support functions required to make it work.
  *	
  *	Revision 1.259  2007/12/19 02:54:26  lassombra
  *	Added function to move debug strings to the MFD.
  *	
  *	MFD can display, freeze, or clear the debug strings.
  *	
  *	Use (Saturn *)saturn->debugString() just as you would use oapiDebugString.
  *	
  *	oapiDebugString can be used to output important alerts to users still, but should remain clean otherwise.
  *	
  *	Also, redirected all the debug statements from the Telecom to the mfd.
  *	
  *	Revision 1.258  2007/12/05 23:07:50  movieman523
  *	Revised to allow SLA panel rotaton to be specified up to 150 degrees. Also start of new connector-equipped vessel code which was mixed up with the rest!
  *	
  *	Revision 1.257  2007/12/05 19:23:29  jasonims
  *	EMS Implementation Step 4 - jasonims :   RSI is set up to rotate, but no actual controlling of it is done.
  *	
  *	Revision 1.256  2007/12/04 20:26:35  tschachim
  *	IMFD5 communication including a new TLI for the S-IVB IU.
  *	Additional CSM panels.
  *	
  *	Revision 1.255  2007/12/02 07:13:39  movieman523
  *	Updates for Apollo 5 and unmanned Saturn 1b missions.
  *	
  *	Revision 1.254  2007/11/29 21:53:20  movieman523
  *	Generising the Volt meters.
  *	
  *	Revision 1.253  2007/11/29 04:56:09  movieman523
  *	Made the System Test meter work (though currently it's connected to the rotary switch, which isn't connected to anything, so just displays 0V).
  *	
  *	Revision 1.252  2007/11/29 01:42:02  flydba
  *	New rotaries added for the left side wall panels (and a new altimeter for the left rendezvous window).
  *	
  *	Revision 1.251  2007/11/26 17:59:06  movieman523
  *	Assorted tidying up of state variable structures.
  *	
  *	Revision 1.250  2007/11/25 09:07:24  jasonims
  *	EMS Implementation Step 2 - jasonims :   EMS Scroll can slew, and some functionality set up for EMS.
  *	
  *	Revision 1.249  2007/11/25 06:55:42  movieman523
  *	Tidied up surface ID code, moving the enum from a shared include file to specific versions for the Saturn and LEM classes.
  *	
  *	Revision 1.248  2007/11/17 02:44:48  lassombra
  *	Added constants for split lower panel id's.  Lower Center is added with same ID as full-size Lower.  Config switch decides which to display.
  *	
  *	Added integer GNSplit: represents the config option for the split panels.
  *	
  *	Added methods for initializing the split lower panel (and the full one, just calls all methods with 0 in offset).
  *	
  *	Revision 1.247  2007/11/16 06:40:45  lassombra
  *	Reverted to 1.244 (Didn't intend to commit, hit commit instead of unedit)
  *	
  *	Revision 1.244  2007/10/18 00:23:22  movieman523
  *	Primarily doxygen changes; minimal functional change.
  *	
  *	Revision 1.243  2007/08/13 16:06:18  tschachim
  *	Moved bitmaps to subdirectory.
  *	New VAGC mission time pad load handling.
  *	New telescope and sextant panels.
  *	Fixed CSM/LV separation speed.
  *	
  *	Revision 1.242  2007/07/27 19:57:28  jasonims
  *	Created MCC master class and split individual functions into sub-classes.  Initial work on CapCom routines.
  *	
  *	Revision 1.241  2007/07/17 14:33:09  tschachim
  *	Added entry and post landing stuff.
  *	
  *	Revision 1.240  2007/06/23 21:20:37  dseagrav
  *	LVDC++ Update: Now with Pre-IGM guidance
  *	
  *	Revision 1.239  2007/06/08 20:08:29  tschachim
  *	Kill apex cover vessel.
  *	
  *	Revision 1.238  2007/06/06 15:02:17  tschachim
  *	OrbiterSound 3.5 support, various fixes and improvements.
  *	
  *	Revision 1.237  2007/04/29 12:36:43  tschachim
  *	Panel bugfixes.
  *	
  *	Revision 1.236  2007/04/25 18:48:11  tschachim
  *	EMS dV functions.
  *	
  *	Revision 1.235  2007/03/24 03:19:39  flydba
  *	LEB and new side panels added.
  *	
  *	Revision 1.234  2007/02/18 01:35:29  dseagrav
  *	MCC / LVDC++ CHECKPOINT COMMIT. No user-visible functionality added. lvimu.cpp/h and mcc.cpp/h added.
  *	
  *	Revision 1.233  2007/02/06 18:30:17  tschachim
  *	Bugfixes docking probe, CSM/LM separation. The ASTP stuff still needs fixing though.
  *	
  *	Revision 1.232  2007/02/02 13:55:45  tschachim
  *	CSM RHC/THC Z axis autodetection.
  *	
  *	Revision 1.231  2007/01/22 15:48:16  tschachim
  *	SPS Thrust Vector Control, RHC power supply, THC clockwise switch, bugfixes.
  *	
  *	Revision 1.230  2007/01/14 13:02:43  dseagrav
  *	CM AC bus feed reworked. Inverter efficiency now varies, AC busses are 3-phase all the way to the inverter, inverter switching logic implemented to match the CM motor-switch lockouts. Original AC bus feeds deleted. Inverter overload detection enabled and correct.
  *	
  *	Revision 1.229  2007/01/06 23:08:32  dseagrav
  *	More telecom stuff. A lot of the S-band signal path exists now, albeit just to consume electricity.
  *	
  *	Revision 1.228  2007/01/06 07:34:35  dseagrav
  *	FLIGHT bus added, uptelemetry now draws power, UPTLM switches on MDC now operate
  *	
  *	Revision 1.227  2007/01/06 04:44:49  dseagrav
  *	Corrected CREW ALARM command behavior, PCM downtelemetry generator now draws power
  *	
  *	Revision 1.226  2006/12/26 12:58:47  dseagrav
  *	CMC C/W lamp on restart and altered restart to compensate.
  *	
  *	Revision 1.225  2006/12/23 17:01:28  movieman523
  *	Added multiple rocket-cams. I'm not sure how to rotate the view, so it's pretty much a 'you get what you get' at the moment.
  *	
  *	Revision 1.224  2006/12/19 15:56:09  tschachim
  *	ECS test stuff, bugfixes.
  *	
  *	Revision 1.223  2006/12/10 00:47:27  dseagrav
  *	Optics code moved to class, now draws power, most switches work, manual-resolved mode not implemented
  *	
  *	Revision 1.222  2006/12/07 18:52:43  tschachim
  *	New LC34, Bugfixes.
  *	
  *	Revision 1.221  2006/11/30 03:34:25  dseagrav
  *	Added basic manual optics controls
  *	
  *	Revision 1.220  2006/11/29 03:01:17  dseagrav
  *	Cause "Zero Optics" mode to zero optics.
  *	
  *	Revision 1.219  2006/11/24 22:42:44  dseagrav
  *	Enable changing bits in AGC channel 33, enable LEB optics switch, enable tracker switch as optics status debug switch.
  *	
  *	Revision 1.218  2006/11/13 14:47:30  tschachim
  *	New SPS engine.
  *	New ProjectApolloConfigurator.
  *	Fixed and changed camera and FOV handling.
  *	
  *	Revision 1.217  2006/10/26 18:48:50  movieman523
  *	Fixed up CM RCS 1 and 2 warning lights to make the 'C&WS Operational Check' work.
  *	
  *	Revision 1.216  2006/09/23 22:34:40  jasonims
  *	New J-2 Engine textures...
  *	
  *	Revision 1.215  2006/08/25 05:16:51  jasonims
  *	Passive Optics-orbiter interface is commited.  SextTrunion, TeleTrunion, and OpticsShaft are values that need to be updated in order to produce a visual change of view.
  *	
  *	Revision 1.214  2006/08/12 14:14:18  movieman523
  *	Renamed EVA and LEVA classes, and added ApexCoverAttached flag to Saturn.
  *	
  *	Revision 1.213  2006/08/11 21:16:22  movieman523
  *	Dummy ELS implementation.
  *	
  *	Revision 1.212  2006/08/11 19:34:47  movieman523
  *	Added code to take the docking probe with the LES on a post-abort jettison.
  *	
  *	Revision 1.211  2006/08/09 00:46:03  flydba
  *	Panel 101 added.
  *	
  *	Revision 1.210  2006/08/08 20:23:50  jasonims
  *	More Optics stuff and changed the Aperture settings for interior views.
  *	
  *	Revision 1.209  2006/07/31 15:58:31  jasonims
  *	*** empty log message ***
  *	
  *	Revision 1.208  2006/07/31 00:05:59  jasonims
  *	Set up Optics panels...
  *	
  *	Revision 1.207  2006/07/28 02:06:58  movieman523
  *	Now need to hard dock to get the connectors connected.
  *	
  *	Revision 1.206  2006/07/27 23:24:11  tschachim
  *	The Saturn 1b now has the Saturn V IGM autopilot.
  *	
  *	Revision 1.205  2006/07/27 22:38:57  movieman523
  *	Added CSM to LEM power connector.
  *	
  *	Revision 1.204  2006/07/27 20:40:06  movieman523
  *	We can now draw power from the SIVb in the Apollo to Venus scenario.
  *	
  *	Revision 1.203  2006/07/21 23:04:35  movieman523
  *	Added Saturn 1b engine lights on panel and beginnings of electrical connector work (couldn't disentangle the changes). Be sure to get the config file for the SIVb as well.
  *	
  *	Revision 1.202  2006/07/16 17:43:42  flydba
  *	Switches and rotary on panel 13 (ORDEAL) now work.
  *	
  *	Revision 1.201  2006/07/09 16:09:38  movieman523
  *	Added Prog 59 for SIVb venting.
  *	
  *	Revision 1.200  2006/07/09 00:07:07  movieman523
  *	Initial tidy-up of connector code.
  *	
  **************************************************************************/

#if !defined(_PA_SATURN_H)
#define _PA_SATURN_H

//
// I hate nested includes, but this is much easier than adding them to all the files
// which need them.
//

#include "PanelSDK/PanelSDK.h"

#include "connector.h"
#include "csmconnector.h"
#include "cautionwarning.h"
#include "csmcautionwarning.h"
#include "missiontimer.h"
#include "FDAI.h"
#include "iu.h"
#include "satswitches.h"
#include "powersource.h"
#include "dockingprobe.h"
#include "pyro.h"
#include "secs.h"
#include "scs.h"
#include "csm_telecom.h"
#include "sps.h"
#include "mcc.h"
#include "ecs.h"
#include "csmrcs.h"
#include "checklistController.h"
#include "payload.h"

#define DIRECTINPUT_VERSION 0x0800
#include "dinput.h"


//
// IMFD5 communication support
//

#include "IMFD/IMFD_Client.h"


#define RCS_SM_QUAD_A		0
#define RCS_SM_QUAD_B		1
#define RCS_SM_QUAD_C		2
#define RCS_SM_QUAD_D		3

#define RCS_CM_RING_1		4
#define RCS_CM_RING_2		5

///
/// \brief O2/H2 tank status.
/// \ingroup InternalInterface
///
typedef struct {
	double O2Tank1PressurePSI;
	double O2Tank2PressurePSI;
	double H2Tank1PressurePSI;
	double H2Tank2PressurePSI;
	double O2SurgeTankPressurePSI;
} TankPressures;

///
/// \brief O2/H2 tank quantities.
/// \ingroup InternalInterface
///
typedef struct {
	double O2Tank1Quantity;
	double O2Tank1QuantityKg;
	double O2Tank2Quantity;
	double O2Tank2QuantityKg;
	double H2Tank1Quantity;
	double H2Tank2Quantity;
} TankQuantities;

///
/// \brief Launch Vehicle tank quantities.
/// \ingroup InternalInterface
///
typedef struct {
	double SICQuantity;
	double SIIQuantity;
	double SIVBOxQuantity;
	double SIVBFuelQuantity;
	double SICFuelMass;
	double SIIFuelMass;
	double S4BFuelMass;
	double S4BOxMass;
} LVTankQuantities;

///
/// \brief Cabin atmosphere status.
/// \ingroup InternalInterface
///
typedef struct {
	double SuitTempK;
	double SuitTempF;
	double CabinTempK;
	double CabinTempF;
	double CabinPressureMMHG;
	double SuitPressureMMHG;
	double SuitReturnPressureMMHG;
	double CabinPressurePSI;
	double SuitPressurePSI;
	double SuitReturnPressurePSI;
	double SuitCO2MMHG;
	double CabinCO2MMHG;
	double CabinRegulatorFlowLBH;
	double O2DemandFlowLBH;
	double DirectO2FlowLBH;
	double SuitTestFlowLBH;
	double CabinRepressFlowLBH;
	double EmergencyCabinRegulatorFlowLBH;
} AtmosStatus;

///
/// \brief Displayed atmosphere status.
/// \ingroup InternalInterface
///
typedef struct {
	double DisplayedO2FlowLBH;
	double DisplayedSuitComprDeltaPressurePSI;
	double DisplayedEcsRadTempPrimOutletMeterTemperatureF;
} DisplayedAtmosStatus;

///
/// \brief Primary ECS cooling status.
/// \ingroup InternalInterface
///
typedef struct {
	double RadiatorInletPressurePSI;
	double RadiatorInletTempF;
	double RadiatorOutletTempF;
	double EvaporatorOutletTempF;
	double EvaporatorSteamPressurePSI;
	double AccumulatorQuantityPercent;
} PrimECSCoolingStatus;

///
/// \brief Secondary ECS cooling status.
/// \ingroup InternalInterface
///
typedef struct {
	double RadiatorInletPressurePSI;
	double RadiatorInletTempF;
	double RadiatorOutletTempF;
	double EvaporatorOutletTempF;
	double EvaporatorSteamPressurePSI;
	double AccumulatorQuantityPercent;
} SecECSCoolingStatus;

///
/// \brief ECS water status.
/// \ingroup InternalInterface
///
typedef struct {
	double PotableH2oTankQuantityPercent;
	double WasteH2oTankQuantityPercent;
} ECSWaterStatus;

///
/// \brief General ECS status.
/// \ingroup InternalInterface
///
typedef struct {
	int crewNumber;
	int crewStatus;
	double PrimECSHeating;
	double PrimECSTestHeating;
	double SecECSHeating;
	double SecECSTestHeating;
} ECSStatus;

///
/// \brief Main bus status.
/// \ingroup InternalInterface
///
typedef struct {
	double MainBusAVoltage;
	double MainBusBVoltage;
	bool Enabled_DC_A_CWS;
	bool Enabled_DC_B_CWS;
	bool Reset_DC_A_CWS;
	bool Reset_DC_B_CWS;
	bool Fc_Disconnected;
} MainBusStatus;

///
/// \brief Battery bus status.
/// \ingroup InternalInterface
///
typedef struct {
	double BatBusAVoltage;
	double BatBusACurrent;
	double BatBusBVoltage;
	double BatBusBCurrent;
} BatteryBusStatus;

///
/// \brief Battery status.
/// \ingroup InternalInterface
///
typedef struct {
	double BatteryAVoltage;
	double BatteryACurrent;
	double BatteryBVoltage;
	double BatteryBCurrent;
	double BatteryCVoltage;
	double BatteryCCurrent;
	double BatteryChargerCurrent;
} BatteryStatus;

///
/// \brief Apollo Guidance Computer warning status.
/// \ingroup InternalInterface
///
typedef struct {
	bool CMCWarning;
	bool ISSWarning;
	bool TestAlarms;
} AGCWarningStatus;

///
/// \brief Pyro status.
/// \ingroup InternalInterface
///
typedef struct {
	double BusAVoltage;
	double BusBVoltage;
} PyroStatus;

///
/// \brief SECS status.
/// \ingroup InternalInterface
///
typedef struct {
	double BusAVoltage;
	double BusBVoltage;
} SECSStatus;

///
/// \brief RCS status
/// \ingroup InternalInterface
///
typedef struct {
	double PackageTempF;
	double HeliumTempF;
	double HeliumPressurePSI;
	double PropellantPressurePSI;
} RCSStatus;

///
/// \brief SPS status.
/// \ingroup InternalInterface
///
typedef struct {
	double chamberPressurePSI;
} SPSStatus;

///
/// \brief Generic Saturn launch vehicle class.
/// \ingroup Saturns
///
class Saturn: public ProjectApolloConnectorVessel, public PanelSwitchListener {

public:

	//
	// First define some structures and values specific to the Saturn class. This ensures we don't
	// have problems with using the same names for different purposes in other classes.
	//
	
	///
	/// This enum gives IDs for the surface bitmaps. We don't use #defines because we want
	/// to automatically calculate the maximum number of bitmaps.
	///
	/// \ingroup Saturns
	///
	enum SurfaceID
	{
		//
		// First value in the enum must be set to one. Entry zero is not
		// used.
		//
		SRF_INDICATOR							=	 1,
		SRF_NEEDLE,
		SRF_DIGITAL,
		SRF_SWITCHUP,
		SRF_SWITCHLEVER,
		SRF_SWITCHGUARDS,
		SRF_ABORT,
		SRF_LV_ENG,
		SRF_ALTIMETER,
		SRF_ALTIMETER2,
		SRF_THRUSTMETER,
		SRF_SEQUENCERSWITCHES,
		SRF_LMTWOPOSLEVER,
		SRF_MASTERALARM_BRIGHT,
		SRF_DSKY,
		SRF_THREEPOSSWITCH,
		SRF_MFDFRAME,
		SRF_MFDPOWER,
		SRF_ROTATIONALSWITCH,
		SRF_SUITCABINDELTAPMETER,
		SRF_THREEPOSSWITCH305,
		SRF_DSKYDISP,
		SRF_FDAI,
		SRF_FDAIROLL,
		SRF_CWSLIGHTS,
		SRF_EVENT_TIMER_DIGITS,
		SRF_DSKYKEY,
		SRF_ECSINDICATOR,
		SRF_SWITCHUPSMALL,
		SRF_CMMFDFRAME,
		SRF_COAS,
		SRF_THUMBWHEEL_SMALLFONTS,
		SRF_CIRCUITBRAKER,
		SRF_CIRCUITBRAKER_YELLOW,
		SRF_THREEPOSSWITCH20,
		SRF_THUMBWHEEL_SMALLFONTS_DIAGONAL,
		SRF_THREEPOSSWITCH30,
		SRF_SWITCH20,
		SRF_SWITCH30,
		SRF_SWITCH20LEFT,
		SRF_THREEPOSSWITCH20LEFT,
		SRF_GUARDEDSWITCH20,
		SRF_SWITCHGUARDPANEL15,
		SRF_THUMBWHEEL_SMALLFONTS_DIAGONAL_LEFT,
		SRF_THREEPOSSWITCH30LEFT,
		SRF_SWITCH30LEFT,
		SRF_THREEPOSSWITCH305LEFT,
		SRF_SWITCH305LEFT,
		SRF_FDAIPOWERROTARY,
		SRF_DIRECTO2ROTARY,
		SRF_ECSGLYCOLPUMPROTARY,
		SRF_GTACOVER,
		SRF_DCVOLTS,
		SRF_DCVOLTS_PANEL101,
		SRF_ACVOLTS,
		SRF_DCAMPS,
		SRF_POSTLDGVENTVLVLEVER,
		SRF_SPSMAXINDICATOR,
		SRF_ECSROTARY,
		SRF_CSM_MNPNL_WDW_LES,
		SRF_CSM_RNDZ_WDW_LES,
		SRF_CSM_RIGHT_WDW_LES,
		SRF_CSM_LEFT_WDW_LES,
		SRF_GLYCOLLEVER,
		SRF_FDAIOFFFLAG,
		SRF_FDAINEEDLES,
		SRF_THUMBWHEEL_LARGEFONTS,
		SRF_SPS_FONT_WHITE,
		SRF_SPS_FONT_BLACK,
		SRF_BORDER_31x31,
		SRF_BORDER_34x29,
		SRF_BORDER_34x61,
		SRF_BORDER_55x111,
		SRF_BORDER_46x75,
		SRF_BORDER_39x38,
		SRF_BORDER_92x40,
		SRF_BORDER_34x33,
		SRF_BORDER_29x29,
		SRF_BORDER_34x31,
		SRF_BORDER_50x158,
		SRF_BORDER_38x52,
		SRF_BORDER_34x34,
		SRF_BORDER_90x90,
		SRF_BORDER_84x84,
		SRF_BORDER_70x70,
		SRF_BORDER_23x20,
		SRF_BORDER_78x78,
		SRF_BORDER_32x160,
		SRF_BORDER_72x72,
		SRF_BORDER_75x64,
		SRF_BORDER_34x39,
		SRF_BORDER_58x58,
		SRF_BORDER_160x32,
		SRF_BORDER_57x57,
		SRF_BORDER_47x47,
		SRF_BORDER_48x48,
		SRF_BORDER_65x65,
		SRF_BORDER_87x111,
		SRF_BORDER_23x23,
		SRF_BORDER_118x118,
		SRF_BORDER_38x38,
		SRF_BORDER_116x116,
		SRF_THUMBWHEEL_SMALL,
		SRF_THUMBWHEEL_LARGEFONTSINV,
		SRF_SWLEVERTHREEPOS,
		SRF_ORDEAL_ROTARY,
		SRF_LV_ENG_S1B,
		SRF_SPSMININDICATOR,
		SRF_SPS_INJ_VLV,
		SRF_SM_RCS_MODE,
		SRF_THUMBWHEEL_GPI_PITCH,
		SRF_THUMBWHEEL_GPI_YAW,
		SRF_THC,
		SRF_EMS_LIGHTS,
		SRF_SUITRETURN_LEVER,
		SRF_CABINRELIEFUPPERLEVER,
		SRF_CABINRELIEFLOWERLEVER,
		SRF_CABINRELIEFGUARDLEVER,
		SRF_OPTICS_HANDCONTROLLER,
		SRF_MARK_BUTTONS,
		SRF_THREEPOSSWITCHSMALL,
		SRF_OPTICS_DSKY,
		SRF_MINIMPULSE_HANDCONTROLLER,
		SRF_EMS_SCROLL_LEO,
		SRF_EMS_SCROLL_BORDER,
		SRF_EMS_RSI_BKGRND,
		SRF_EMSDVSETSWITCH,
		SRF_OXYGEN_SURGE_TANK_VALVE,
		SRF_GLYCOL_TO_RADIATORS_KNOB,
		SRF_ACCUM_ROTARY,
		SRF_GLYCOL_ROTARY,
		SRF_TANK_VALVE,
		SRF_PRESS_RELIEF_VALVE,
		SRF_CABIN_REPRESS_VALVE,
		SRF_SELECTOR_INLET_ROTARY,
		SRF_SELECTOR_OUTLET_ROTARY,
		SRF_EMERGENCY_PRESS_ROTARY,
		SRF_SUIT_FLOW_CONTROL_LEVER,
		SRF_CSM_SEC_CABIN_TEMP_VALVE, 
		SRF_CSM_FOOT_PREP_WATER_LEVER,
		SRF_CSM_LM_TUNNEL_VENT_VALVE,
		SRF_CSM_WASTE_MGMT_ROTARY,
		SRF_BORDER_45x36,
		SRF_BORDER_17x36,                         
		SRF_BORDER_33x43,                        
		SRF_BORDER_36x17,                         
		SRF_BORDER_150x80,                      
		SRF_BORDER_200x80,                   
		SRF_BORDER_72x109,        
		SRF_CSM_DEMAND_REG_ROTARY,
		SRF_CSM_SUIT_TEST_LEVER,
		SRF_CSM_GEAR_BOX_ROTARY,
		SRF_CSM_PUMP_HANDLE_ROTARY,
		SRF_CSM_VENT_VALVE_HANDLE,
		SRF_CSM_PUMP_HANDLE_ROTARY_OPEN,
		SRF_BORDER_200x300,

		//
		// NSURF MUST BE THE LAST ENTRY HERE. PUT ANY NEW SURFACE IDS ABOVE THIS LINE
		//
		nsurf	///< nsurf gives the count of surfaces for the array size calculation.
	};

	//
	// Random failure flags, copied into unions and extracted as ints (or vice-versa).
	//

	///
	/// \ingroup FailFlags
	/// \brief Landing failure flags.
	///
	union LandingFailures {
		struct {
			unsigned Init:1;		///< Flags have been initialised.
			unsigned CoverFail:1;	///< Apex cover will fail to deploy automatically.
			unsigned DrogueFail:1;	///< Drogue will fail to deploy automatically.
			unsigned MainFail:1;	///< Main chutes will fail to deploy automatically.
		};
		int word;					///< Word holds the flags from the bitfield in one 32-bit value for scenarios.

		LandingFailures() { word = 0; };
	};

	///
	/// \ingroup FailFlags
	/// \brief Launch failure flags.
	///
	union LaunchFailures {
		struct {
			unsigned Init:1;					///< Flags have been initialised.
			unsigned EarlySICenterCutoff:1;		///< The first stage center engine will shut down early.
			unsigned EarlySIICenterCutoff:1;	///< The second stage center engine will shut down early.
			unsigned LETAutoJetFail:1;			///< The LES auto jettison will fail.
			unsigned LESJetMotorFail:1;			///< The LET jettison motor will fail.
			unsigned SIIAutoSepFail:1;			///< Stage two will fail to seperate automatically from stage one.
		};
		int word;								///< Word holds the flags from the bitfield in one 32-bit value for scenarios.

		LaunchFailures() { word = 0; };
	};

	///
	/// \ingroup FailFlags
	/// \brief Flags specifying which control panel switches will fail.
	///
	/// \ingroup InternalInterface
	///
	union SwitchFailures {
		struct {
			unsigned Init:1;				///< Flags have been initialised.
			unsigned TowerJett1Fail:1;		///< TWR JETT switch 1 will fail.
			unsigned TowerJett2Fail:1;		///< TWR JETT switch 2 will fail.
			unsigned SMJett1Fail:1;			///< SM JETT switch 1 will fail.
			unsigned SMJett2Fail:1;			///< SM JETT switch 2 will fail
		};
		int word;							///< Word holds the flags from the bitfield in one 32-bit value for scenarios.

		SwitchFailures() { word = 0; };
	};

	///
	/// \ingroup ScenarioState
	/// \brief CSM light display state.
	///
	 union LightState {
		struct {
			unsigned Engind0:1;
			unsigned Engind1:1;
			unsigned Engind2:1;
			unsigned Engind3:1;
			unsigned Engind4:1;
			unsigned Engind5:1;
			unsigned LVGuidLight:1;
			unsigned Launchind0:1;
			unsigned Launchind1:1;
			unsigned Launchind2:1;
			unsigned Launchind3:1;
			unsigned Launchind4:1;
			unsigned Launchind5:1;
			unsigned Launchind6:1;
			unsigned Launchind7:1;
			unsigned Engind6:1;
			unsigned Engind7:1;
			unsigned Engind8:1;
			unsigned LVRateLight:1;
		};
		unsigned long word;

		LightState() { word = 0; };
	};

	///
	/// \ingroup ScenarioState
	/// \brief State which is only required through the launch process.
	///
	union LaunchState {
		struct {
			unsigned autopilot:1;
			unsigned TLIEnabled:1;
		};
		unsigned long word;

		LaunchState() { word = 0; };
	};

	///
	/// \ingroup ScenarioState
	/// \brief State which is only required for Apollo 13
	///
	/// This structure holds the flags which are used for the Apollo 13 simulation. 
	///
	union A13State {
		struct {
			unsigned ApolloExploded:1;	///< Has the SM exploded yet?
			unsigned CryoStir:1;		///< Have the crew been asked to do a cryo stir?
			unsigned KranzPlayed:1;		///< Has the Kranz audio been played yet?
		};
		unsigned long word;				///< Used to return all the flags as one 32-bit word for the scenario file.

		A13State() { word = 0; };
	};

	///
	/// \ingroup ScenarioState
	/// \brief Which parts are still attached?
	///
	union AttachState {
		struct {
			unsigned InterstageAttached:1;	///< Is the interstage attached?
			unsigned LESAttached:1;			///< Is the LES attached?
			unsigned HasProbe:1;			///< Does the CM have a docking probe?
			unsigned ApexCoverAttached:1;	///< Is the apex cover attached?
			unsigned ChutesAttached:1;		///< Are the chutes attached?
			unsigned CSMAttached:1;			///< Is there a CSM?
			unsigned NosecapAttached:1;		///< Is there an Apollo 5-style nosecap?
		};
		unsigned long word;

		AttachState() { word = 0; };
	};

	///
	/// \ingroup ScenarioState
	/// \brief State related to the SLA panels.
	///
	union SLAState {
		struct {
			unsigned SLARotationLimit:8;	///< Maximum rotation limit in degrees when opening.
			unsigned SLAWillSeparate:1;		///< Will the panels separate when they reach the limit?
		};
		unsigned long word;

		SLAState() { word = 0; };
	};

	///
	/// \ingroup ScenarioState
	/// \brief Main state flags.
	///
	union MainState {
		struct {
			unsigned MissionTimerRunning:1;			///< Is the Mission timer running?
			unsigned SIISepState:1;					///< State of the SII Sep light.
			unsigned TLIBurnDone:1;					///< Have we done our TLI burn?
			unsigned Scorrec:1;						///< Have we played the course correction sound?
			unsigned Burned:1;						///< Has the CM been burned by re-entry heating?
			unsigned unused:1;						///< Unused bit for backwards compatibility. Can be used for other things.
			unsigned ABORT_IND:1;					///< State of the abort light.
			unsigned unused_2:1;					///< Unused bit for backwards compatibility. Can be used for other things.
			unsigned SplashdownPlayed:1;			///< Have we played the splashdown sound?
			unsigned unused_3:1;					///< Unused bit for backwards compatibility. Can be used for other things.
			unsigned PayloadDataTransfer:1;			///< Have we transfered setup data to the SIVB for the payload?
			unsigned PostSplashdownPlayed:1;		///< Have we played the post-splashdown sound?
			unsigned IGMEnabled:1;					///< Is the IGM guidance enabled?
			unsigned TLISoundsLoaded:1;				///< Have we loaded the TLI sounds?
			unsigned MissionTimerEnabled:1;			///< Is the Mission Timer enabled?
			unsigned EventTimerEnabled:1;			///< Is the Event Timer enabled?
			unsigned EventTimerRunning:1;			///< Is the Event Timer running?
			unsigned EventTimerCountUp:2;			///< Is the Event Timer counting up?
			unsigned SkylabSM:1;					///< Is this a Skylab Service Module?
			unsigned SkylabCM:1;					///< Is this a Skylab Command Module?
			unsigned S1bPanel:1;					///< Is this a Command Module with a Saturn 1b panel?
			unsigned NoHGA:1;						///< Do we have a High-Gain Antenna?
			unsigned viewpos:5;						///< Position of the virtual cockpit viewpoint.
		};
		unsigned long word;

		MainState() { word = 0; };
	};

	//
	// Now the actual code and data for the class.
	//

	//This simply equips us with a reference to either the mfd debug string, 
	//or in the extreme case, the orbiter debug string
	char *(__cdecl *debugString)();
	bool debugConnected;
	///
	/// \brief Standard constructor with the usual Orbiter parameters.
	///
	Saturn(OBJHANDLE hObj, int fmodel);

	///
	/// \brief Destructor.
	///
	virtual ~Saturn();

	///
	/// Turn the autopilot on or off.
	/// \brief Set the autopilot state.
	/// \param ap Autopilot on or off?
	///
	void SetAutopilot(bool ap) { autopilot = ap; };

	///
	/// Check whether the autopilot is enabled.
	/// \brief Get the autopilot state.
	/// \return Is the autopilot enabled?
	///
	bool GetAutopilot() { return autopilot; };

	///
	/// Turn on the SII Seperation light on the control panel.
	/// \brief Set the SII Seperation light.
	///
	void SetSIISep() { SIISepState = true; };

	///
	/// Turn off the SII Seperation light on the control panel.
	/// \brief Clear the SII Seperation light.
	///
	void ClearSIISep() { SIISepState = false; };

	///
	/// Turn on an engine indicator light on the control panel.
	/// \brief Turn on engine light
	/// \param i Specify the engine light number to turn on (1-8).
	///
	void SetEngineIndicator(int i);

	///
	/// Turn off an engine indicator light on the control panel.
	/// \brief Turn off engine light
	/// \param i Specify the engine light number to turn off (1-8).
	///
	void ClearEngineIndicator(int i);

	///
	/// Call this function to jettison the LET and BPC from the Saturn. This should work during any stage,
	/// and will handle any tidying up required. If called with no LET attached, it will just return.
	/// \brief Jettison the LET and BPC.
	/// \param UseMain Specifies whether to use the main abort motor or the jettison motor.
	/// \param AbortJettison If we're jettisoning during an abort, the BPC will take the docking probe with it.
	///
	void JettisonLET(bool UseMain = false, bool AbortJettison = false);

	///
	/// This function can be used during the countdown to update the MissionTime. Since we launch when
	/// MissionTime reaches zero, setting MissionTime to (-t) tells the code when to launch.
	/// \brief Update the launch time.
	/// \param t Specifies the time in seconds to wait before launch.
	///
	void UpdateLaunchTime(double t);	

	///
	/// Set up the default mesh for the virtual cockpit.
	///
	void DefaultCMMeshSetup();

	///
	/// Turn a Service Module RCS thruster on or off.
	/// \brief SM RCS thruster control.
	/// \param Quad SM RCS thruster quad.
	/// \param Thruster Thruster number in quad.
	/// \param Active Thruster on or off?
	///
	virtual void SetRCSState(int Quad, int Thruster, bool Active);

	///
	/// Turn a Command Module RCS thruster on or off.
	/// \brief CM RCS thruster control.
	/// \param Thruster Thruster number to control.
	/// \param Active Thruster on or off?
	///
	virtual void SetCMRCSState(int Thruster, bool Active);

	bool GetCMRCSStateCommanded(THRUSTER_HANDLE th);


	//
	// RHC/THC 
	//

	// DirectInput stuff
	/// Handle to DLL instance
	HINSTANCE dllhandle;
	/// pointer to DirectInput class itself
	LPDIRECTINPUT8 dx8ppv;
	/// Joysticks-Enabled flag / counter - Zero if we aren't using DirectInput, nonzero is the number of joysticks we have.
	int js_enabled;
	/// Pointers to DirectInput joystick devices
	LPDIRECTINPUTDEVICE8 dx8_joystick[2]; ///< One for THC, one for RHC, ignore extras
	DIDEVCAPS			 dx8_jscaps[2];   ///< Joystick capabilities
	DIJOYSTATE2			 dx8_jstate[2];   ///< Joystick state
	HRESULT				 dx8_failure;     ///< DX failure reason
	int rhc_id;							  ///< Joystick # for the RHC
	int rhc_rot_id;						  ///< ID of ROTATOR axis to use for RHC Z-axis
	int rhc_sld_id;                       ///< ID of SLIDER axis to use for RHC Z-axis
	int rhc_rzx_id;                       ///< Flag to use native Z-axis as RHC Z-axis
	int rhc_pov_id;						  ///< ID of the cooliehat a.k.a. POV
	int thc_id;                           ///< Joystick # for the THC
	int thc_rot_id;						  ///< ID of ROTATOR axis to use for THC Z-axis
	int thc_sld_id;                       ///< ID of SLIDER axis to use for THC Z-axis
	int thc_rzx_id;                       ///< Flag to use native Z-axis as THC Z-axis	
	int thc_pov_id;						  ///< ID of the cooliehat a.k.a. POV
	int rhc_debug;						  ///< Flags to print debugging messages.
	int thc_debug;
	bool rhc_auto;						  ///< RHC Z-axis auto detection
	bool thc_auto;						  ///< THC Z-axis auto detection
	bool rhc_thctoggle;					  ///< Enable RHC/THC toggle
	int rhc_thctoggle_id;				  ///< RHC button id for RHC/THC toggle
	bool rhc_thctoggle_pressed;			  ///< Button pressed flag				  
	int js_current;

	//
	// General functions that handle calls from Orbiter.
	//

	///
	/// \brief Orbiter dock state function.
	///
	void clbkDockEvent(int dock, OBJHANDLE connected);
	bool clbkLoadGenericCockpit ();
	bool clbkPanelMouseEvent(int id, int event, int mx, int my);
	bool clbkPanelRedrawEvent(int id, int event, SURFHANDLE surf);
	void clbkMFDMode (int mfd, int mode);

	///
	/// \brief Orbiter state saving function.
	/// \param scn Scenario file to save to.
	///
	void clbkSaveState (FILEHANDLE scn);

	///
	/// \brief Orbiter timestep function.
	/// \param simt Current simulation time, in seconds since Orbiter was started.
	/// \param simdt Time in seconds since last timestep.
	/// \param mjd Current MJD.
	///
	void clbkPostStep (double simt, double simdt, double mjd);

	///
	/// \brief Orbiter timestep function.
	/// \param simt Current simulation time, in seconds since Orbiter was started.
	/// \param simdt Time in seconds since last timestep.
	/// \param mjd Current MJD.
	///
	void clbkPreStep(double simt, double simdt, double mjd);
	bool clbkLoadPanel (int id);
	int clbkConsumeDirectKey(char *keystate);
	int clbkConsumeBufferedKey(DWORD key, bool down, char *kstate);
	bool clbkLoadVC (int id);
	bool clbkVCMouseEvent (int id, int event, VECTOR3 &p);
	bool clbkVCRedrawEvent (int id, int event, SURFHANDLE surf);
	void clbkPostCreation();

	///
	/// This function performs all actions required to update the spacecraft state as time
	/// passes. This is a pure virtual function implemented by the derived classes.
	/// \brief Process time steps from Orbiter.
	/// \param simt Current MET in seconds.
	/// \param simdt Time in seconds since last timestep.
	///
	virtual void Timestep(double simt, double simdt, double mjd) = 0;

	///
	/// \brief Initialise a virtual cockpit view.
	///
	void InitVC (int vc);
	bool RegisterVC ();

	void PanelSwitchToggled(ToggleSwitch *s);
	void PanelIndicatorSwitchStateRequested(IndicatorSwitch *s); 
	void PanelRotationalSwitchChanged(RotationalSwitch *s);
	void PanelThumbwheelSwitchChanged(ThumbwheelSwitch *s);
	void PanelRefreshHatch();

	// Called by Crawler/ML
	virtual void LaunchVehicleRolloutEnd() {};	// after arrival on launch pad
	virtual void LaunchVehicleBuild() {};		// build/unbuild during assembly
	virtual void LaunchVehicleUnbuild() {};
	
	int GetBuildStatus() { return buildstatus; }

	///
	/// \brief Get the current stage number
	/// \return Stage number as specified in nassdefs.h
	///
	int GetStage() { return stage; };

	///
	/// \brief Get the systems state
	/// \return Systems state.
	///
	int GetSystemsState() { return systemsState; };


	///
	/// \brief Get the Apollo mission number
	/// \return Apollo mission number.
	///
	int GetApolloNo() { return ApolloNo; };

	///
	/// \brief Get the current mission elapsed time
	/// \return Current time
	///
	double GetMissionTime() { return MissionTime; };

	///
	/// Since we can now run with either the Virtual AGC emulator or the C++ AGC, this function
	/// allows you to check which we're using.
	/// \brief Are we running a Virtual AGC?
	/// \return True for Virtual AGC, false for C++ AGC.
	///
	bool IsVirtualAGC() { return agc.IsVirtualAGC(); };

	///
	/// \brief Triggers Virtual AGC core dump
	///
	virtual void VirtualAGCCoreDump() { agc.VirtualAGCCoreDump("ProjectApollo CMC.core"); }

	///
	/// Get a pointer to the Saturn Instrument Unit, which controls the autopilot prior to SIVb/CSM
	/// seperation.
	/// \brief Access the Saturn IU.
	/// \return Pointer to IU object.
	///
	IU *GetIU() { return &iu; };

	///
	/// \brief Get settings for the Saturn payload.
	///
	void GetPayloadSettings(PayloadSettings &p);

	SPSPropellantSource *GetSPSPropellant() { return &SPSPropellant; };
	SPSEngine *GetSPSEngine() { return &SPSEngine; };

	///
	/// \brief Accessor to get checklistController
	///
	ChecklistController *GetChecklistControl();


	//
	// CWS functions.
	//

	///
	/// Get information on the atmosphere status in the CSM.
	/// \brief Get atmosphere status.
	/// \param atm Atmosphere information structure, updated by the call.
	///
	void GetAtmosStatus(AtmosStatus &atm);
	void GetDisplayedAtmosStatus(DisplayedAtmosStatus &atm);
	void GetTankPressures(TankPressures &press);
	void GetTankQuantities(TankQuantities &q);
	void SetO2TankQuantities(double q);

	///
	/// Get information on the status of a fuel cell in the CSM.
	/// \brief Get fuel cell status.
	/// \param index Index of fuel cell to query (1-3)
	/// \param fc Fuel cell information structure, updated by the call.
	///
	void GetFuelCellStatus(int index, FuelCellStatus &fc);
	void GetPrimECSCoolingStatus(PrimECSCoolingStatus &pcs);
	void GetSecECSCoolingStatus(SecECSCoolingStatus &scs);
	void GetECSWaterStatus(ECSWaterStatus &ws);
	void GetMainBusStatus(MainBusStatus &ms);
	void GetBatteryBusStatus( BatteryBusStatus &bs );
	void GetACBusStatus(ACBusStatus &as, int busno);
	void GetBatteryStatus( BatteryStatus &bs );
	void GetSPSStatus( SPSStatus &ss );
	void GetSECSStatus( SECSStatus &ss );
	void GetPyroStatus( PyroStatus &ps );
	void DisconnectInverter(bool disc, int busno);
	void GetAGCWarningStatus(AGCWarningStatus &aws);
	void GetRCSStatus(int index, RCSStatus &rs);
	double GetAccelG() { return aZAcc / G; };
	virtual void GetECSStatus(ECSStatus &ecs);
	virtual void SetCrewNumber(int number);
	virtual void SetPrimECSTestHeaterPowerW(double power);
	virtual void SetSecECSTestHeaterPowerW(double power);

	///
	/// Get information on launch vehicle propellant tank quantities.
	/// \brief Get LV fuel tank status.
	/// \param LVq LV fuel tank information structure, updated by the call.
	///
	void GetLVTankQuantities(LVTankQuantities &LVq);

	///
	/// Enable or disable generic Service Module systems based on current state.
	/// \brief Check SM systems state.
	///
	void CheckSMSystemsState();

	///
	/// Check whether the pyros are armed.
	/// \brief Are pyros armed?
	/// \return True if armed, false if not.
	///
	bool PyrosArmed();

	///
	/// If the scenario specified AUTOSLOW and time acceleration is enabled, slow it
	/// down to 1.0.
	/// \brief Disable time acceleration if desired.
	///
	void SlowIfDesired();

	void ActivateS4RCS();
	void DeactivateS4RCS();

	///
	/// \brief Enable or disable the J2 engine on the SIVb.
	/// \param Enable Engine on or off.
	///
	void EnableDisableJ2(bool Enable);

	///
	/// \brief Set thrust level of the SIVb J2 engine.
	/// \param thrust Thrust level 0.0 - 1.0.
	///
	void SetJ2ThrustLevel(double thrust);

	///
	/// \brief Get thrust level of the SIVb J2 engine.
	/// \return Thrust level 0.0 - 1.0.
	///
	double GetJ2ThrustLevel();

	///
	/// \brief Set thrust level of the SIVb APS engines.
	/// \param thrust Thrust level 0.0 - 1.0.
	///
	void SetAPSThrustLevel(double thrust);

	///
	/// \brief Get propellant mass in the SIVb stage.
	/// \return Propellant mass in kg.
	///
	double GetSIVbPropellantMass();

	///
	/// \brief Set propellant mass in the SIVb stage.
	/// \param mass Propellant mass in kg.
	///
	void SetSIVbPropellantMass(double mass);

	///
	/// \brief Get the state of the SII/SIVb Sep switch.
	/// \return Switch state.
	///
	int GetSIISIVbSepSwitchState();

	///
	/// \brief Get the state of the TLI Enable switch.
	/// \return Switch state.
	///
	int GetTLIEnableSwitchState();

	///
	/// \brief Load sounds required for TLI burn.
	///
	void LoadTLISounds();

	///
	/// \brief Clear TLI sounds.
	///
	void ClearTLISounds();

	///
	/// \brief Play or stop countdown sound.
	/// \param StartStop True to start, false to stop.
	///
	void PlayCountSound(bool StartStop);

	///
	/// \brief Play or stop SECO sound.
	/// \param StartStop True to start, false to stop.
	///
	void PlaySecoSound(bool StartStop);

	///
	/// \brief Play or stop seperation sound.
	/// \param StartStop True to start, false to stop.
	///
	void PlaySepsSound(bool StartStop);

	///
	/// \brief Play or stop TLI sound.
	/// \param StartStop True to start, false to stop.
	///
	void PlayTLISound(bool StartStop);

	///
	/// \brief Play or stop TLI start sound.
	/// \param StartStop True to start, false to stop.
	///
	void PlayTLIStartSound(bool StartStop);

	///
	/// \brief We've hard docked, so check connections.
	///
	void HaveHardDocked(int port);

	///
	/// \brief We've extended the docking probe, so disconnect connections.
	///
	void Undocking(int port);

	///
	/// \brief Set docking probe mesh
	///
	void SetDockingProbeMesh();

	///
	/// \brief Set side hatch mesh
	///
	void SetSideHatchMesh();
	
	///
	/// \brief Set crew mesh
	///
	void SetCrewMesh();

	///
	/// Check whether the Launch Escape Tower is attached.
	/// \brief Is the LET still attached?
	/// \return True if attached, false if not.
	///
	bool LETAttached();

	///
	/// \brief Returns the IMFD communication client for ProjectApolloMFD
	///
	virtual IMFD_Client *GetIMFDClient() { return &IMFD_Client; }; 

	///
	/// \brief TLI event management
	///
	void TLI_Begun();

protected:

	///
	/// PanelSDK functions as a interface between the
	/// actual System & Panel SDK and VESSEL class
	///
	/// Note that this must be defined early in the file, so it will be initialised
	/// before any other classes which rely on it at creation. Don't move it further
	/// down without good reason, or you're likely to crash!
	///
	/// \brief Panel SDK library.
	///
    PanelSDK Panelsdk;

	// FILE *PanelsdkLogFile;

	void InitSwitches();

	///
	/// Set all engine indicator lights on the control panel.
	/// \brief Set engine indicator lights.
	///
	void SetEngineIndicators();

	///
	/// Clear all engine indicator lights on the control panel.
	/// \brief Clear engine indicator lights.
	///
	void ClearEngineIndicators();

	///
	/// Turn on the liftoff light on the control panel.
	/// \brief Set the liftoff light.
	///
	void SetLiftoffLight();

	///
	/// Turn off the liftoff light on the control panel.
	/// \brief Clear the liftoff light.
	///
	void ClearLiftoffLight();

	///
	/// Turn the control panel LES Motor light on or off. If SequencerSwitchLightingDisabled, the light is never lit,
	/// as the real panel didn't have a light for this button.
	/// \brief Set or clear the LES Motor light.
	/// \param lit True to turn light on, false to turn light off.
	///
	void SetLESMotorLight(bool lit);

	///
	/// Turn the control panel Canard Deploy light on or off. If SequencerSwitchLightingDisabled, the light is never lit,
	/// as the real panel didn't have a light for this button.
	/// \brief Set or clear the Canard Deploy light.
	/// \param lit True to turn light on, false to turn light off.
	///
	void SetCanardDeployLight(bool lit);

	///
	/// Turn the control panel CSM/LV Sep light on or off. If SequencerSwitchLightingDisabled, the light is never lit,
	/// as the real panel didn't have a light for this button.
	/// \brief Set or clear the CSM/LV Sep light.
	/// \param lit True to turn light on, false to turn light off.
	///
	void SetCSMLVSepLight(bool lit);

	///
	/// Turn the control panel Apex Cover light on or off. If SequencerSwitchLightingDisabled, the light is never lit,
	/// as the real panel didn't have a light for this button.
	/// \brief Set or clear the Apex Cover Sep light.
	/// \param lit True to turn light on, false to turn light off.
	///
	void SetApexCoverLight(bool lit);

	///
	/// Turn the control panel Drogue Deploy light on or off. If SequencerSwitchLightingDisabled, the light is never lit,
	/// as the real panel didn't have a light for this button.
	/// \brief Set or clear the Drogue Deploy light.
	/// \param lit True to turn light on, false to turn light off.
	///
	void SetDrogueDeployLight(bool lit);

	///
	/// Turn the control panel Main Deploy light on or off. If SequencerSwitchLightingDisabled, the light is never lit,
	/// as the real panel didn't have a light for this button.
	/// \brief Set or clear the Main Deploy light.
	/// \param lit True to turn light on, false to turn light off.
	///
	void SetMainDeployLight(bool lit);

	void SetCmRcsHeDumpSwitch(bool lit);

	///
	/// Turn on the LV Guidance warning light on the control panel to indicate an autopilot
	/// failure.
	/// \brief Set the LV Guidance light.
	///
	void SetLVGuidLight();

	///
	/// Turn off the LV Guidance warning light on the control panel.
	/// \brief Clear the LV Guidance light.
	///
	void ClearLVGuidLight();

	///
	/// Turn on the LV Rate warning light on the control panel, indicating that the spacecraft
	/// is turning too fast.
	/// \brief Set the LV Rate light.
	///
	void SetLVRateLight();

	///
	/// Turn on the LV Rate warning light on the control panel.
	/// \brief Set the LV Rate light.
	///
	void ClearLVRateLight();

	///
	/// Check whether the Earth Landing System is active.
	/// \brief Is ELS Active?
	/// \return True if active, false if not.
	///
	bool ELSActive();

	///
	/// Check whether the Earth Landing System is in AUTO mode.
	/// \brief Is ELS in AUTO?
	/// \return True if AUTO, false if not.
	///
	bool ELSAuto();

	///
	/// Check whether the Saturn vehicle has a CSM. Some, like Apollo 5, flew without a CSM for
	// LEM testing.
	/// \brief Do we have a CSM?
	/// \return True if CSM, false if not.
	///
	bool SaturnHasCSM();

	void JettisonDockingProbe();

	//
	// State that needs to be saved.
	//

	char StagesString[256];

	///
	/// Autopilot flag. True if autopilot enabled.
	/// \brief Autopilot flag.
	///
	bool autopilot;

	///
	/// SII Sep light. True if light should be lit.
	/// \brief SII Sep light.
	///
	bool SIISepState;

	///
	/// Abort light. True if ABORT light is lit.
	/// \brief Abort light.
	///
	bool ABORT_IND;

	///
	/// Interstage attached flag. True if interstage is attached.
	/// \brief Interstage flag.
	///
	bool InterstageAttached;

	///
	/// LESAttached flag. True if the LES is attached.
	/// \brief LES flag.
	///
	bool LESAttached;

	///
	/// True if the docking probe is attached.
	/// \brief Docking probe flag.
	///
	bool HasProbe;

	///
	/// True if the parachutes are attached.
	/// \brief Parachutes attached flag.
	///
	bool ChutesAttached;

	///
	/// True if the apex cover is attached.
	/// \brief Apex cover flag.
	///
	bool ApexCoverAttached;

	///
	/// True if there is a CSM.
	/// \brief Nosecap attached flag.
	///
	bool CSMAttached;

	///
	/// True if there is an Apollo 5-style nosecap in place of a CSM.
	/// \brief Nosecap attached flag.
	///
	bool NosecapAttached;

	///
	/// Gives the angle to which the SLA panels will rotate; some of the Skylab missions
	/// planned to retain the SLA panels but rotate them around 150 degrees.
	/// \brief SLA panel rotation angle.
	///
	int SLARotationLimit;

	///
	/// True if the SLA panels will separate from the SIVB.
	/// \brief SLA panels separation flag.
	///
	bool SLAWillSeparate;

	bool DeleteLaunchSite;

	int buildstatus;

	//
	// Current mission time and mission times for stage events.
	//

	///
	/// The current Mission Elapsed Time. This is the main variable used for timing
	/// automated events during the mission, giving the time in seconds from launch
	/// (negative for the pre-launch countdown).
	/// \brief Mission Elapsed Time.
	///
	double MissionTime;

	///
	/// The time in seconds of the next automated event to occur in the mission. This 
	/// is a generic value used by the autopilot and staging code.
	/// \brief Time of next event.
	///
	double NextMissionEventTime;

	///
	/// The time in seconds of the previous automated event that occur in the mission. This 
	/// is a generic value used by the autopilot and staging code.
	/// \brief Time of last event.
	///
	double LastMissionEventTime;

	///
	/// The time in seconds of the next check for destroying old stages. We destroy them when
	/// they get too far away or too close to the ground as there's no point keeping them
	/// hanging around when we'll never see them again.
	/// \brief Time to next check for stage destruction.
	///
	double NextDestroyCheckTime;

	///
	/// The time in seconds when the next failure will occur.
	/// \brief Time of next system failure.
	///
	double NextFailureTime;

	///
	/// Mission Timer display on control panel.
	/// \brief Mission Timer display.
	///
	MissionTimer MissionTimerDisplay;

	///
	/// Event Timer display on control panel.
	/// \brief Event Timer display.
	///
	EventTimer EventTimerDisplay;
	
	//
	// Center engine shutdown times for first and
	// second stage.
	//

	double FirstStageCentreShutdownTime;
	double SecondStageCentreShutdownTime;

	//
	// Interstage and LES jettison time.
	//

	double InterstageSepTime;
	double LESJettisonTime;

	//
	// Mixture-ratio shift time for second stage.
	//

	double SecondStagePUShiftTime;

	//
	// Stage shutdown times.
	//

	double FirstStageShutdownTime;
	double SecondStageShutdownTime;

	//
	// Iterative Guidance Mode start time, when we stop following the pitch program and start aiming for
	// the correct orbit.
	//

	double IGMStartTime;
	bool IGMEnabled;

	///
	/// Flag for use of low-res meshes to reduce graphics lag.
	/// \brief Using low-res meshes?
	///
	bool LowRes;

	//
	// Mesh handles.
	//

	MESHHANDLE hStage1Mesh;
	MESHHANDLE hStage2Mesh;
	MESHHANDLE hStage3Mesh;
	MESHHANDLE hInterstageMesh;
	MESHHANDLE hStageSLA1Mesh;
	MESHHANDLE hStageSLA2Mesh;
	MESHHANDLE hStageSLA3Mesh;
	MESHHANDLE hStageSLA4Mesh;

	UINT SPSidx;

	bool LEM_DISPLAY;

	///
	/// The Saturn vehicle number. Saturn 1b will usually be in the 200s, and Saturn V in the 500s.
	/// \brief Saturn number.
	///
	int VehicleNo;

	///
	/// Apollo mission number. Some events are triggered based on the mission, so for fictional
	/// missions this should be set to a value below 7 or above 18.
	/// \brief Apollo mission number.
	///
	int ApolloNo;

	///
	/// Flag to determine whether we leave the OrbiterSound default ATC enabled. Some people like
	/// it, so if this flag is true we allow them to use it.
	/// \brief Orbitersound ATC enabled.
	///
	bool UseATC;

	///
	/// Realism is a value from 0 to 10, determining how realistic the systems simulation will be.
	/// Zero is basically 'arcade mode' whereas 10 is as realistic as we can make it.
	/// \brief Simulation realism level.
	///
	int Realism;

	///
	/// Flag to indicate that we've told the user to make a course correction.
	/// \brief Course correction flag.
	///
	bool Scorrec;

	///
	/// Flag to indicate that the SM has exploded on the Apollo 13 mission.
	/// \brief SM exploded flag.
	///
	bool ApolloExploded;

	///
	/// Flag to indicate that the CM has been 'burnt' by re-entry.
	/// \brief CM burnt flag.
	///
	bool Burned;

	bool CryoStir;
	double TCPO;

	//
	// Failures.
	//

	LandingFailures LandFail;
	LaunchFailures LaunchFail;
	SwitchFailures SwitchFail;

	//
	// Pitch table.
	//

	#define PITCH_TABLE_SIZE	16

	///
	/// Table of Mission Times for pitch changes.
	/// \brief Pitch change times.
	///
	double met[PITCH_TABLE_SIZE];

	///
	/// Table of pitch values at the specified mission times. The correct pitch will be
	/// interpolated between these values.
	/// \brief Pitch table values.
	///
	double cpitch[PITCH_TABLE_SIZE];

	//
	// *** LVDC++ ITEMS ***
	//
	bool use_lvdc; // LVDC use flag
	LVIMU lvimu;   // ST-124-M3 IMU (LV version)
	LVRG lvrg;	   // LV rate gyro package

	//
	// Ground Systems
	//
	MCC	 mcc;
	MC_GroundTrack  groundtrack;
	MC_CapCom  capcom;

	//
	// ChecklistController
	//
	ChecklistController checkControl;
	SaturnEvents eventControl;

	//
	// Switches
	//

	int coasEnabled;
	int opticsDskyEnabled;

	///
	/// \brief Right-hand FDAI.
	///
	FDAI fdaiRight;

	///
	/// \brief Left-hand FDAI.
	///
	FDAI fdaiLeft;
	int fdaiDisabled;
	int fdaiSmooth;

	HBITMAP hBmpFDAIRollIndicator;

	SwitchRow MasterAlarmSwitchRow;
	MasterAlarmSwitch MasterAlarmSwitch; 

	// EMS
	SwitchRow EMSFunctionSwitchRow;
	RotationalSwitch EMSFunctionSwitch;

	SwitchRow EMSModeRow;
	ThreePosSwitch EMSModeSwitch;

	SwitchRow GTASwitchRow;
	GuardedToggleSwitch GTASwitch;

	SaturnEMSDvSetSwitch EMSDvSetSwitch;

	SwitchRow EMSDvDisplayRow;
	SaturnEMSDvDisplay EMSDvDisplay;

	SwitchRow IMUCageSwitchRow;
	SwitchRow CautionWarningRow;
	SwitchRow MissionTimerSwitchesRow;
	SwitchRow SPSRow;

	SwitchRow AccelGMeterRow;
	SaturnAccelGMeter AccelGMeter;

	SwitchRow THCRotaryRow;
	THCRotarySwitch THCRotary;

	GuardedPushSwitch LiftoffNoAutoAbortSwitch;
	LESMotorFireSwitch LesMotorFireSwitch;
	GuardedPushSwitch CanardDeploySwitch;
	GuardedPushSwitch CsmLvSepSwitch;
	GuardedPushSwitch ApexCoverJettSwitch;
	GuardedPushSwitch DrogueDeploySwitch;
	GuardedPushSwitch MainDeploySwitch;
	GuardedPushSwitch CmRcsHeDumpSwitch;

	ToggleSwitch	    EDSSwitch;				
	GuardedToggleSwitch CsmLmFinalSep1Switch;
	GuardedToggleSwitch CsmLmFinalSep2Switch;
	GuardedToggleSwitch CmSmSep1Switch;
	GuardedToggleSwitch CmSmSep2Switch;
	GuardedToggleSwitch SIVBPayloadSepSwitch;

	ToggleSwitch   CabinFan1Switch;
	ToggleSwitch   CabinFan2Switch;
	ThreePosSwitch H2Heater1Switch;
	ThreePosSwitch H2Heater2Switch;
	ThreePosSwitch O2Heater1Switch;
	ThreePosSwitch O2Heater2Switch;	
	ToggleSwitch   O2PressIndSwitch;	
	ThreePosSwitch H2Fan1Switch; 
	ThreePosSwitch H2Fan2Switch; 
	ThreePosSwitch O2Fan1Switch; 
	ThreePosSwitch O2Fan2Switch; 

	IndicatorSwitch FuelCellPhIndicator;
	IndicatorSwitch FuelCellRadTempIndicator;

	IndicatorSwitch FuelCellRadiators1Indicator;
	IndicatorSwitch FuelCellRadiators2Indicator;
	IndicatorSwitch FuelCellRadiators3Indicator;
	ThreePosSwitch FuelCellRadiators1Switch;
	ThreePosSwitch FuelCellRadiators2Switch;
	ThreePosSwitch FuelCellRadiators3Switch;

	RotationalSwitch FuelCellIndicatorsSwitch;

	ToggleSwitch FuelCellHeater1Switch;
	ToggleSwitch FuelCellHeater2Switch;
	ToggleSwitch FuelCellHeater3Switch;

	ThreePosSwitch FuelCellPurge1Switch;
	ThreePosSwitch FuelCellPurge2Switch;
	ThreePosSwitch FuelCellPurge3Switch;

	IndicatorSwitch FuelCellReactants1Indicator;
	IndicatorSwitch FuelCellReactants2Indicator;
	IndicatorSwitch FuelCellReactants3Indicator;
	ThreePosSwitch FuelCellReactants1Switch;
	ThreePosSwitch FuelCellReactants2Switch;
	ThreePosSwitch FuelCellReactants3Switch;

	ToggleSwitch FCReacsValvesSwitch;
	ToggleSwitch H2PurgeLineSwitch;

	ThreeSourceSwitch FuelCellPumps1Switch;
	ThreeSourceSwitch FuelCellPumps2Switch;
	ThreeSourceSwitch FuelCellPumps3Switch;

	SaturnSuitCompressorSwitch SuitCompressor1Switch;
	SaturnSuitCompressorSwitch SuitCompressor2Switch;

	TimerControlSwitch MissionTimerSwitch;
	CWSModeSwitch CautionWarningModeSwitch;
	CWSSourceSwitch CautionWarningCMCSMSwitch;
	CWSPowerSwitch CautionWarningPowerSwitch;
	CWSLightTestSwitch CautionWarningLightTestSwitch;

	TimerUpdateSwitch MissionTimerHoursSwitch;
	TimerUpdateSwitch MissionTimerMinutesSwitch;
	TimerUpdateSwitch MissionTimerSecondsSwitch;

	IMUCageSwitch IMUGuardedCageSwitch;

	SwitchRow SMRCSHelium1Row;
	ThreePosSwitch SMRCSHelium1ASwitch;
	ThreePosSwitch SMRCSHelium1BSwitch;
	ThreePosSwitch SMRCSHelium1CSwitch;
	ThreePosSwitch SMRCSHelium1DSwitch;

	SwitchRow SMRCSHelium1TalkbackRow;
	SaturnRCSValveTalkback SMRCSHelium1ATalkback;
	SaturnRCSValveTalkback SMRCSHelium1BTalkback;
	SaturnRCSValveTalkback SMRCSHelium1CTalkback;
	SaturnRCSValveTalkback SMRCSHelium1DTalkback;

	SwitchRow SMRCSHelium2Row;
	ThreePosSwitch SMRCSHelium2ASwitch;
	ThreePosSwitch SMRCSHelium2BSwitch;
	ThreePosSwitch SMRCSHelium2CSwitch;
	ThreePosSwitch SMRCSHelium2DSwitch;

	SwitchRow SMRCSHelium2TalkbackRow;
	SaturnRCSValveTalkback SMRCSHelium2ATalkback;
	SaturnRCSValveTalkback SMRCSHelium2BTalkback;
	SaturnRCSValveTalkback SMRCSHelium2CTalkback;
	SaturnRCSValveTalkback SMRCSHelium2DTalkback;

	AGCIOSwitch CMUplinkSwitch;
	ToggleSwitch IUUplinkSwitch;

	GuardedToggleSwitch CMRCSPressSwitch;
	ToggleSwitch SMRCSIndSwitch;

	SwitchRow RCSGaugeRow;
	RCSQuantityMeter RCSQuantityMeter;
	RCSFuelPressMeter RCSFuelPressMeter;
	RCSHeliumPressMeter RCSHeliumPressMeter;
	RCSTempMeter RCSTempMeter;

	SwitchRow SMRCSProp1Row;
	ThreePosSwitch SMRCSProp1ASwitch;
	ThreePosSwitch SMRCSProp1BSwitch;
	ThreePosSwitch SMRCSProp1CSwitch;
	ThreePosSwitch SMRCSProp1DSwitch;

	ThreeOutputSwitch SMRCSHeaterASwitch;
	ThreeOutputSwitch SMRCSHeaterBSwitch;
	ThreeOutputSwitch SMRCSHeaterCSwitch;
	ThreeOutputSwitch SMRCSHeaterDSwitch;

	SwitchRow SMRCSProp1TalkbackRow;
	SaturnRCSValveTalkback SMRCSProp1ATalkback;
	SaturnRCSValveTalkback SMRCSProp1BTalkback;
	SaturnRCSValveTalkback SMRCSProp1CTalkback;
	SaturnRCSValveTalkback SMRCSProp1DTalkback;

	SwitchRow SMRCSProp2Row;
	ThreePosSwitch SMRCSProp2ASwitch;
	ThreePosSwitch SMRCSProp2BSwitch;
	ThreePosSwitch SMRCSProp2CSwitch;
	ThreePosSwitch SMRCSProp2DSwitch;

	ThreePosSwitch RCSCMDSwitch;
	ThreePosSwitch RCSTrnfrSwitch;

	ThreePosSwitch CMRCSProp1Switch;
	ThreePosSwitch CMRCSProp2Switch;

	SwitchRow SMRCSProp2TalkbackRow;
	SaturnRCSValveTalkback CMRCSProp1Talkback;
	SaturnRCSValveTalkback CMRCSProp2Talkback;

	SaturnRCSValveTalkback SMRCSProp2ATalkback;
	SaturnRCSValveTalkback SMRCSProp2BTalkback;
	SaturnRCSValveTalkback SMRCSProp2CTalkback;
	SaturnRCSValveTalkback SMRCSProp2DTalkback;

	SwitchRow RCSIndicatorsSwitchRow;
	PropellantRotationalSwitch RCSIndicatorsSwitch;

	SwitchRow ECSIndicatorsSwitchRow;
	RotationalSwitch ECSIndicatorsSwitch;

	SwitchRow AttitudeControlRow;
	ThreePosSwitch ManualAttRollSwitch;
	ThreePosSwitch ManualAttPitchSwitch;
	ThreePosSwitch ManualAttYawSwitch;
	ToggleSwitch LimitCycleSwitch;
	ToggleSwitch AttDeadbandSwitch;
	ToggleSwitch AttRateSwitch;
	ToggleSwitch TransContrSwitch;

	SwitchRow BMAGRow;
	ThreePosSwitch BMAGRollSwitch;
	ThreePosSwitch BMAGPitchSwitch;
	ThreePosSwitch BMAGYawSwitch;

	SwitchRow SCContCMCModeSwitchesRow;
	SaturnSCContSwitch SCContSwitch;
	CMCModeHoldFreeSwitch CMCModeSwitch;

	SwitchRow SCSTvcSwitchesRow;
	ThreePosSwitch SCSTvcPitchSwitch;
	ThreePosSwitch SCSTvcYawSwitch;

	SwitchRow SPSGimbalMotorsRow;
	ThreePosSwitch Pitch1Switch;
	ThreePosSwitch Pitch2Switch;
	ThreePosSwitch Yaw1Switch;
	ThreePosSwitch Yaw2Switch;

	SwitchRow EntrySwitchesRow;
	ToggleSwitch EMSRollSwitch;
	ToggleSwitch GSwitch;

	SwitchRow LVSPSIndSwitchesRow;
	ToggleSwitch LVSPSPcIndicatorSwitch;
	ToggleSwitch LVFuelTankPressIndicatorSwitch;

	SwitchRow TVCGimbalDriveSwitchesRow;
	ThreePosSwitch TVCGimbalDrivePitchSwitch;
	ThreePosSwitch TVCGimbalDriveYawSwitch;

	SwitchRow CSMLightSwitchesRow;
	ToggleSwitch RunEVALightSwitch;
	ThreePosSwitch RndzLightSwitch;
	ToggleSwitch TunnelLightSwitch;

	SwitchRow LMPowerSwitchRow;
	ThreePosSwitch LMPowerSwitch;

	SwitchRow PostLDGVentValveLeverRow;
	CircuitBrakerSwitch PostLDGVentValveLever;

	SwitchRow GDCAlignButtonRow;
	PushSwitch GDCAlignButton;
	
	//
	// Communication switches (s-band, vhf etc.)
	//

	SwitchRow TelecomTBRow;
	IndicatorSwitch PwrAmplTB;
	DSEIndicatorSwitch DseTapeTB;

	ThreePosSwitch SBandNormalXPDRSwitch;
	ThreePosSwitch SBandNormalPwrAmpl1Switch;
	ThreePosSwitch SBandNormalPwrAmpl2Switch;
	ThreePosSwitch SBandNormalMode1Switch;
	ThreePosSwitch SBandNormalMode2Switch;
	ToggleSwitch SBandNormalMode3Switch;

	SwitchRow SBandAuxSwitchesRow;
	ThreePosSwitch SBandAuxSwitch1;
	ThreePosSwitch SBandAuxSwitch2;

	SwitchRow UPTLMSwitchesRow;
	ToggleSwitch   UPTLMSwitch1;
	ThreePosSwitch UPTLMSwitch2;

	SwitchRow SBandAntennaSwitchesRow;
	ThreePosSwitch SBandAntennaSwitch1;
	ThreePosSwitch SBandAntennaSwitch2;

	SwitchRow VHFAmThumbwheelsRow;
	ThumbwheelSwitch SquelchAThumbwheel;
	ThumbwheelSwitch SquelchBThumbwheel;

	SwitchRow VHFSwitchesRow;
	ThreePosSwitch VHFAMASwitch;
	ThreePosSwitch VHFAMBSwitch;
	ThreePosSwitch RCVOnlySwitch;
	ToggleSwitch VHFBeaconSwitch;
	ToggleSwitch VHFRangingSwitch;

	SwitchRow TapeRecorderSwitchesRow;
	ToggleSwitch TapeRecorder1Switch;
	DSEPlayRecordSwitch TapeRecorder2Switch;
	ThreePosSwitch TapeRecorder3Switch;

	SwitchRow PowerSwitchesRow;
	ThreePosSwitch SCESwitch;
	ThreePosSwitch PMPSwitch;

	SwitchRow PCMBitRateSwitchRow;
	ToggleSwitch PCMBitRateSwitch;
	ThreePosSwitch DummySwitch;
	
	//
	// VHF antenna rotary
	//
	
	SwitchRow VHFAntennaRotaryRow;
	RotationalSwitch VHFAntennaRotarySwitch;

	//
	// SPS switches (panel 3)
	//

	SwitchRow SPSInjectorValveIndicatorsRow;
	IndicatorSwitch SPSInjectorValve1Indicator;
	IndicatorSwitch SPSInjectorValve2Indicator;
	IndicatorSwitch SPSInjectorValve3Indicator;
	IndicatorSwitch SPSInjectorValve4Indicator;

	SwitchRow SPSTestSwitchRow;
	ThreePosSwitch SPSTestSwitch;

	SwitchRow SPSOxidFlowValveSwitchesRow;
	ThreePosSwitch SPSOxidFlowValveSwitch;
	ToggleSwitch SPSOxidFlowValveSelectorSwitch;

	SwitchRow SPSOxidFlowValveIndicatorsRow;
	IndicatorSwitch SPSOxidFlowValveMaxIndicator;
	IndicatorSwitch SPSOxidFlowValveMinIndicator;

	SwitchRow SPSPugModeSwitchRow;
	ThreePosSwitch SPSPugModeSwitch;

	SwitchRow SPSHeliumValveIndicatorsRow;
	IndicatorSwitch SPSHeliumValveAIndicator;
	IndicatorSwitch SPSHeliumValveBIndicator;

	SwitchRow SPSSwitchesRow;
	ThreePosSwitch SPSHeliumValveASwitch;
	ThreePosSwitch SPSHeliumValveBSwitch;
	ThreePosSwitch SPSLineHTRSSwitch;
	ThreePosSwitch SPSPressIndSwitch;

	//
	// Electricals switches & indicators
	//

	SwitchRow DCIndicatorsRotaryRow;
	PowerStateRotationalSwitch DCIndicatorsRotary;

	SwitchRow BatteryChargeRotaryRow;
	RotationalSwitch BatteryChargeRotary;

	SwitchRow ACIndicatorRotaryRow;
	PowerStateRotationalSwitch ACIndicatorRotary;

	SwitchRow ACInverterSwitchesRow;
	TwoSourceSwitch MnA1Switch;
	TwoSourceSwitch MnB2Switch;
	ThreeSourceSwitch MnA3Switch;
	CMACInverterSwitch AcBus1Switch1;
	CMACInverterSwitch AcBus1Switch2;
	CMACInverterSwitch AcBus1Switch3;
	ThreeSourceSwitch AcBus1ResetSwitch;
	CMACInverterSwitch AcBus2Switch1;
	CMACInverterSwitch AcBus2Switch2;
	CMACInverterSwitch AcBus2Switch3;
	ThreeSourceSwitch AcBus2ResetSwitch;

	DCBusIndicatorSwitch MainBusBIndicator1;
	DCBusIndicatorSwitch MainBusBIndicator2;
	DCBusIndicatorSwitch MainBusBIndicator3;
	DCBusIndicatorSwitch MainBusAIndicator1;
	DCBusIndicatorSwitch MainBusAIndicator2;
	DCBusIndicatorSwitch MainBusAIndicator3;
	SaturnFuelCellConnectSwitch MainBusBSwitch1;
	SaturnFuelCellConnectSwitch MainBusBSwitch2;
	SaturnFuelCellConnectSwitch MainBusBSwitch3;
	ThreeSourceSwitch MainBusBResetSwitch;
	SaturnFuelCellConnectSwitch MainBusASwitch1;
	SaturnFuelCellConnectSwitch MainBusASwitch2;
	SaturnFuelCellConnectSwitch MainBusASwitch3;
	ThreeSourceSwitch MainBusAResetSwitch;

	//
	// Electrical meters
	//

	SwitchRow ACVoltMeterRow;
	ACVoltMeter CSMACVoltMeter;

	SwitchRow DCVoltMeterRow;
	DCVoltMeter CSMDCVoltMeter;

	SwitchRow DCAmpMeterRow;
	SaturnDCAmpMeter DCAmpMeter;

	SwitchRow SystemTestMeterRow;
	DCVoltMeter SystemTestVoltMeter;

	//
	// FDAI control switches.
	//

	SwitchRow FDAISwitchesRow;
	ThreePosSwitch FDAIScaleSwitch;
	ThreePosSwitch FDAISourceSwitch;
	ThreePosSwitch FDAISelectSwitch;
	ToggleSwitch FDAIAttSetSwitch;

	//
	// CMC Att: IMU is normal state, GDC does nothing.
	//

	SwitchRow CMCAttRow;
	ToggleSwitch CMCAttSwitch;

	//
	// Launch vehicle switches.
	//

	SwitchRow LVRow;
	GuardedToggleSwitch LVGuidanceSwitch;
	GuardedToggleSwitch SIISIVBSepSwitch;
	XLunarSwitch TLIEnableSwitch;

	//
	// ELS and CSM propellant switches.
	//

	SwitchRow ELSRow;
	AGCIOSwitch CGSwitch;
	GuardedToggleSwitch ELSLogicSwitch;
	ToggleSwitch ELSAutoSwitch;
	ToggleSwitch CMRCSLogicSwitch;
	GuardedToggleSwitch CMPropDumpSwitch;
	GuardedToggleSwitch CMPropPurgeSwitch;

	//
	// Event Timer switches.
	//

	SwitchRow EventTimerRow;
	// The FSCM switches were removed in the real CSM
	// ToggleSwitch FCSMSPSASwitch;
	// ToggleSwitch FCSMSPSBSwitch;
	EventTimerResetSwitch EventTimerUpDownSwitch;
	EventTimerControlSwitch EventTimerControlSwitch;
	TimerUpdateSwitch EventTimerMinutesSwitch;
	TimerUpdateSwitch EventTimerSecondsSwitch;

	//
	// Main chute release switch.
	//
	// Currently this does nothing, as the parachutes automatically release from the CM
	// after landing.
	//

	SwitchRow MainReleaseRow;
	GuardedToggleSwitch MainReleaseSwitch;

	//
	// Abort switches.
	//

	SwitchRow AbortRow;
	ToggleSwitch PropDumpAutoSwitch;
	ToggleSwitch TwoEngineOutAutoSwitch;
	ToggleSwitch LVRateAutoSwitch;
	GuardedThreePosSwitch TowerJett1Switch;
	GuardedThreePosSwitch TowerJett2Switch;

	///
	/// \brief Rotational Controller power switches.
	///
	SwitchRow RotContrPowerRow;
	ThreePosSwitch RotPowerNormal1Switch;
	ThreePosSwitch RotPowerNormal2Switch;
	ThreePosSwitch RotPowerDirect1Switch;
	ThreePosSwitch RotPowerDirect2Switch;

	///
	/// \brief dV Thrust switches.
	///
	SwitchRow dvThrustRow;
	GuardedTwoSourceSwitch dVThrust1Switch;
	GuardedTwoSourceSwitch dVThrust2Switch;

	///
	/// \brief SPS Switch.
	///
	ToggleSwitch SPSswitch;

	SwitchRow SPSGimbalPitchThumbwheelRow;
	ThumbwheelSwitch SPSGimbalPitchThumbwheel;
	SwitchRow SPSGimbalYawThumbwheelRow;
	ThumbwheelSwitch SPSGimbalYawThumbwheel;

	SwitchRow DirectUllageThrustOnRow;
	PushSwitch DirectUllageButton;
	PushSwitch ThrustOnButton;

	///
	/// \brief SPS meters.
	///
	SwitchRow SPSOxidPercentRow;
	SaturnSPSOxidPercentMeter SPSOxidPercentMeter;

	SwitchRow SPSFuelPercentRow;
	SaturnSPSFuelPercentMeter SPSFuelPercentMeter;

	SwitchRow SPSOxidUnbalMeterRow;
	SaturnSPSOxidUnbalMeter SPSOxidUnbalMeter;

	SwitchRow SPSMetersRow;
	SaturnSPSTempMeter SPSTempMeter;
	SaturnSPSHeliumNitrogenPressMeter SPSHeliumNitrogenPressMeter;
	SaturnSPSPropellantPressMeter SPSFuelPressMeter;
	SaturnSPSPropellantPressMeter SPSOxidPressMeter;

	SwitchRow LVSPSPcMeterRow;
	SaturnLVSPSPcMeter LVSPSPcMeter;

	SwitchRow GPFPIMeterRow;
	SaturnGPFPIPitchMeter GPFPIPitch1Meter;
	SaturnGPFPIPitchMeter GPFPIPitch2Meter;
	SaturnGPFPIYawMeter GPFPIYaw1Meter;
	SaturnGPFPIYawMeter GPFPIYaw2Meter;

	///
	/// \brief Cryo tank meters
	///
	SwitchRow CryoTankMetersRow;
	SaturnH2PressureMeter H2Pressure1Meter;
	SaturnH2PressureMeter H2Pressure2Meter;
	SaturnO2PressureMeter O2Pressure1Meter;
	SaturnO2PressureMeter O2Pressure2Meter;
	SaturnCryoQuantityMeter H2Quantity1Meter;
	SaturnCryoQuantityMeter H2Quantity2Meter;
	SaturnCryoQuantityMeter O2Quantity1Meter;
	SaturnCryoQuantityMeter O2Quantity2Meter;

	///
	/// \brief Fuel cell meters
	///
	SwitchRow FuelCellMetersRow;
	SaturnFuelCellH2FlowMeter FuelCellH2FlowMeter;
	SaturnFuelCellO2FlowMeter FuelCellO2FlowMeter;
	SaturnFuelCellTempMeter FuelCellTempMeter;
	SaturnFuelCellCondenserTempMeter FuelCellCondenserTempMeter;

	///
	/// \brief Cabin meters
	///
	SwitchRow CabinMetersRow;
	SaturnSuitTempMeter SuitTempMeter;
	SaturnCabinTempMeter CabinTempMeter;
	SaturnSuitPressMeter SuitPressMeter; 
	SaturnCabinPressMeter CabinPressMeter;
	SaturnPartPressCO2Meter PartPressCO2Meter; 

	//
	// Suit compressor delta pressure meter
	//
	SwitchRow SuitComprDeltaPMeterRow;
	SaturnSuitComprDeltaPMeter SuitComprDeltaPMeter;
	SaturnLeftO2FlowMeter LeftO2FlowMeter;

	//
	// Suit cabin delta pressure meter
	//
	SwitchRow SuitCabinDeltaPMeterRow;
	SaturnSuitCabinDeltaPMeter SuitCabinDeltaPMeter;
	SaturnRightO2FlowMeter RightO2FlowMeter;

	//
	// ECS radiator/evaporator temperature/pressure meters
	//
	SwitchRow EcsRadTempMetersRow;
	SaturnEcsRadTempInletMeter EcsRadTempInletMeter;
	SaturnEcsRadTempPrimOutletMeter EcsRadTempPrimOutletMeter;

	SwitchRow EcsEvapTempMetersRow;
	SaturnEcsRadTempSecOutletMeter EcsRadTempSecOutletMeter;
	SaturnGlyEvapTempOutletMeter GlyEvapTempOutletMeter;

	SwitchRow EcsPressMetersRow;
	SaturnGlyEvapSteamPressMeter GlyEvapSteamPressMeter;
	SaturnGlycolDischPressMeter GlycolDischPressMeter;

	SwitchRow EcsQuantityMetersRow;
	SaturnAccumQuantityMeter AccumQuantityMeter;
	SaturnH2oQuantityMeter H2oQuantityMeter;

	//
	// ECS radiator switches
	//
	SwitchRow EcsRadiatorIndicatorRow;
	IndicatorSwitch EcsRadiatorIndicator;

	SwitchRow EcsRadiatorSwitchesRow;
	ThreePosSwitch EcsRadiatorsFlowContAutoSwitch;
	ThreePosSwitch EcsRadiatorsFlowContPwrSwitch;
	ThreePosSwitch EcsRadiatorsManSelSwitch;
	ThreePosSwitch EcsRadiatorsHeaterPrimSwitch;
	ToggleSwitch EcsRadiatorsHeaterSecSwitch;

	//
	// ECS switches
	//
	SwitchRow EcsSwitchesRow;
	ThreePosSwitch PotH2oHtrSwitch;
	ThreePosSwitch SuitCircuitH2oAccumAutoSwitch;
	ThreePosSwitch SuitCircuitH2oAccumOnSwitch;
	ThreePosSwitch SuitCircuitHeatExchSwitch;
	ThreePosSwitch SecCoolantLoopEvapSwitch;
	ThreeSourceSwitch SecCoolantLoopPumpSwitch;
	ToggleSwitch H2oQtyIndSwitch;
	ToggleSwitch GlycolEvapTempInSwitch;
	ToggleSwitch GlycolEvapSteamPressAutoManSwitch;
	ThreePosSwitch GlycolEvapSteamPressIncrDecrSwitch;
	ThreePosSwitch GlycolEvapH2oFlowSwitch;
	ToggleSwitch CabinTempAutoManSwitch;

	SwitchRow CabinTempAutoControlSwitchRow;
	ThumbwheelSwitch CabinTempAutoControlSwitch;

	SwitchRow EcsGlycolPumpsSwitchRow;
	SaturnEcsGlycolPumpsSwitch EcsGlycolPumpsSwitch;

	//
	// High gain antenna
	//
	
	SwitchRow HighGainAntennaUpperSwitchesRow;
	ThreePosSwitch GHATrackSwitch;
	ThreePosSwitch GHABeamSwitch;

	SwitchRow HighGainAntennaLowerSwitchesRow;
	ThreePosSwitch GHAPowerSwitch;
	ToggleSwitch GHAServoElecSwitch;
	
	SwitchRow HighGainAntennaPitchPositionSwitchRow;
	RotationalSwitch HighGainAntennaPitchPositionSwitch;

	SwitchRow HighGainAntennaYawPositionSwitchRow;
	RotationalSwitch HighGainAntennaYawPositionSwitch;

	///
	/// \brief Docking probe switches
	///
	SwitchRow DockingProbeSwitchesRow;
	GuardedThreePosSwitch DockingProbeExtdRelSwitch;
	ThreePosSwitch DockingProbeRetractPrimSwitch;
	ThreePosSwitch DockingProbeRetractSecSwitch;

	SwitchRow DockingProbeIndicatorsRow;
	IndicatorSwitch DockingProbeAIndicator;
	IndicatorSwitch DockingProbeBIndicator;

	//
	// Orbiter switches
	//

	SwitchRow OrbiterAttitudeToggleRow;
	AttitudeToggle OrbiterAttitudeToggle;

	/////////////////////////////
	// Panel 5 circuit brakers //
	/////////////////////////////
	
	// EPS sensor signal circuit brakers

	SwitchRow EpsSensorSignalDcCircuitBrakersRow;
	CircuitBrakerSwitch EpsSensorSignalDcMnaCircuitBraker;
	CircuitBrakerSwitch EpsSensorSignalDcMnbCircuitBraker;
	
	SwitchRow EpsSensorSignalAcCircuitBrakersRow;
	CircuitBrakerSwitch EpsSensorSignalAc1CircuitBraker;
	CircuitBrakerSwitch EpsSensorSignalAc2CircuitBraker;

	// C/W circuit brakers

	SwitchRow CWCircuitBrakersRow;
	CircuitBrakerSwitch CWMnaCircuitBraker;
	CircuitBrakerSwitch CWMnbCircuitBraker;

	// LEM PWR circuit brakers
	
	SwitchRow LMPWRCircuitBrakersRow;
	CircuitBrakerSwitch MnbLMPWR1CircuitBraker;
	CircuitBrakerSwitch MnbLMPWR2CircuitBraker;

	// Inverter control circuit brakers
	
	SwitchRow InverterControlCircuitBrakersRow;
	CircuitBrakerSwitch InverterControl1CircuitBraker;
	CircuitBrakerSwitch InverterControl2CircuitBraker;
	CircuitBrakerSwitch InverterControl3CircuitBraker;

	// EPS sensor unit circuit brakers
	
	SwitchRow EPSSensorUnitCircuitBrakersRow;
	CircuitBrakerSwitch EPSSensorUnitDcBusACircuitBraker;
	CircuitBrakerSwitch EPSSensorUnitDcBusBCircuitBraker;
	CircuitBrakerSwitch EPSSensorUnitAcBus1CircuitBraker;
	CircuitBrakerSwitch EPSSensorUnitAcBus2CircuitBraker;

	// Battery relay bus circuit brakers

	SwitchRow BATRLYBusCircuitBrakersRow;
	CircuitBrakerSwitch BATRLYBusBatACircuitBraker;
	CircuitBrakerSwitch BATRLYBusBatBCircuitBraker;

	// ECS radiators circuit brakers

	SwitchRow ECSRadiatorsCircuitBrakersRow;
	CircuitBrakerSwitch ControllerAc1CircuitBraker;
	CircuitBrakerSwitch ControllerAc2CircuitBraker;
	CircuitBrakerSwitch CONTHTRSMnACircuitBraker;
	CircuitBrakerSwitch CONTHTRSMnBCircuitBraker;
	CircuitBrakerSwitch HTRSOVLDBatACircuitBraker;
	CircuitBrakerSwitch HTRSOVLDBatBCircuitBraker;

	// Battery charger circuit brakers

	SwitchRow BatteryChargerCircuitBrakersRow;
	CircuitBrakerSwitch BatteryChargerBatACircuitBraker;
	CircuitBrakerSwitch BatteryChargerBatBCircuitBraker;
	CircuitBrakerSwitch BatteryChargerMnACircuitBraker;
	CircuitBrakerSwitch BatteryChargerMnBCircuitBraker;
	CircuitBrakerSwitch BatteryChargerAcPwrCircuitBraker;

	// Instrument lighting circuit brakers

	SwitchRow InstrumentLightingCircuitBrakersRow;
	CircuitBrakerSwitch InstrumentLightingESSMnACircuitBraker;
	CircuitBrakerSwitch InstrumentLightingESSMnBCircuitBraker;
	CircuitBrakerSwitch InstrumentLightingNonESSCircuitBraker;
	CircuitBrakerSwitch InstrumentLightingSCIEquipSEP1CircuitBraker;
	CircuitBrakerSwitch InstrumentLightingSCIEquipSEP2CircuitBraker;
	CircuitBrakerSwitch InstrumentLightingSCIEquipHatchCircuitBraker;
	
	// ECS circuit brakers

	SwitchRow ECSCircuitBrakersRow;
	CircuitBrakerSwitch ECSPOTH2OHTRMnACircuitBraker;
	CircuitBrakerSwitch ECSPOTH2OHTRMnBCircuitBraker;
	CircuitBrakerSwitch ECSH2OAccumMnACircuitBraker;
	CircuitBrakerSwitch ECSH2OAccumMnBCircuitBraker;
	CircuitBrakerSwitch ECSTransducerWastePOTH2OMnACircuitBraker;
	CircuitBrakerSwitch ECSTransducerWastePOTH2OMnBCircuitBraker;
	CircuitBrakerSwitch ECSTransducerPressGroup1MnACircuitBraker;
	CircuitBrakerSwitch ECSTransducerPressGroup1MnBCircuitBraker;
	CircuitBrakerSwitch ECSTransducerPressGroup2MnACircuitBraker;
	CircuitBrakerSwitch ECSTransducerPressGroup2MnBCircuitBraker;
	CircuitBrakerSwitch ECSTransducerTempMnACircuitBraker;
	CircuitBrakerSwitch ECSTransducerTempMnBCircuitBraker;

	// ECS circuit brakers lower row 

	SwitchRow ECSLowerRowCircuitBrakersRow;
	CircuitBrakerSwitch ECSSecCoolLoopAc1CircuitBraker;
	CircuitBrakerSwitch ECSSecCoolLoopAc2CircuitBraker;
	CircuitBrakerSwitch ECSSecCoolLoopRADHTRMnACircuitBraker;
	CircuitBrakerSwitch ECSSecCoolLoopXducersMnACircuitBraker;
	CircuitBrakerSwitch ECSSecCoolLoopXducersMnBCircuitBraker;
	CircuitBrakerSwitch ECSWasteH2OUrineDumpHTRMnACircuitBraker;
	CircuitBrakerSwitch ECSWasteH2OUrineDumpHTRMnBCircuitBraker;
	CircuitBrakerSwitch ECSCabinFanAC1ACircuitBraker;
	CircuitBrakerSwitch ECSCabinFanAC1BCircuitBraker;
	CircuitBrakerSwitch ECSCabinFanAC1CCircuitBraker;
	CircuitBrakerSwitch ECSCabinFanAC2ACircuitBraker;
	CircuitBrakerSwitch ECSCabinFanAC2BCircuitBraker;
	CircuitBrakerSwitch ECSCabinFanAC2CCircuitBraker;

	// Guidance/navigation circuit brakers 

	SwitchRow GNCircuitBrakersRow;
	CircuitBrakerSwitch GNPowerAc1CircuitBraker;
	CircuitBrakerSwitch GNPowerAc2CircuitBraker;
	CircuitBrakerSwitch GNIMUMnACircuitBraker;
	CircuitBrakerSwitch GNIMUMnBCircuitBraker;
	CircuitBrakerSwitch GNIMUHTRMnACircuitBraker;
	CircuitBrakerSwitch GNIMUHTRMnBCircuitBraker;
	CircuitBrakerSwitch GNComputerMnACircuitBraker;
	CircuitBrakerSwitch GNComputerMnBCircuitBraker;
	CircuitBrakerSwitch GNOpticsMnACircuitBraker;
	CircuitBrakerSwitch GNOpticsMnBCircuitBraker;

	/////////////////////////////
	// Panel 4 circuit brakers //
	/////////////////////////////
	
	// Suit compressors circuit brakers 

	SwitchRow SuitCompressorsAc1ACircuitBrakerRow;
	CircuitBrakerSwitch SuitCompressorsAc1ACircuitBraker;

	SwitchRow SuitCompressorsAc1BCircuitBrakerRow;
	CircuitBrakerSwitch SuitCompressorsAc1BCircuitBraker;

	SwitchRow SuitCompressorsAc1CCircuitBrakerRow;
	CircuitBrakerSwitch SuitCompressorsAc1CCircuitBraker;

	SwitchRow SuitCompressorsAc2ACircuitBrakerRow;
	CircuitBrakerSwitch SuitCompressorsAc2ACircuitBraker;

	SwitchRow SuitCompressorsAc2BCircuitBrakerRow;
	CircuitBrakerSwitch SuitCompressorsAc2BCircuitBraker;

	SwitchRow SuitCompressorsAc2CCircuitBrakerRow;
	CircuitBrakerSwitch SuitCompressorsAc2CCircuitBraker;

	// ECS glycol pumps circuit brakers 

	SwitchRow ECSGlycolPumpsAc1ACircuitBrakerRow;
	CircuitBrakerSwitch ECSGlycolPumpsAc1ACircuitBraker;

	SwitchRow ECSGlycolPumpsAc1BCircuitBrakerRow;
	CircuitBrakerSwitch ECSGlycolPumpsAc1BCircuitBraker;

	SwitchRow ECSGlycolPumpsAc1CCircuitBrakerRow;
	CircuitBrakerSwitch ECSGlycolPumpsAc1CCircuitBraker;

	SwitchRow ECSGlycolPumpsAc2ACircuitBrakerRow;
	CircuitBrakerSwitch ECSGlycolPumpsAc2ACircuitBraker;

	SwitchRow ECSGlycolPumpsAc2BCircuitBrakerRow;
	CircuitBrakerSwitch ECSGlycolPumpsAc2BCircuitBraker;

	SwitchRow ECSGlycolPumpsAc2CCircuitBrakerRow;
	CircuitBrakerSwitch ECSGlycolPumpsAc2CCircuitBraker;

	/////////////////////////
	// Panel 6 thumbwheels //
	/////////////////////////

	SwitchRow ModeIntercomVOXSensThumbwheelSwitchRow;
	ThumbwheelSwitch ModeIntercomVOXSensThumbwheelSwitch;

	SwitchRow PowerMasterVolumeThumbwheelSwitchRow;
	VolumeThumbwheelSwitch PowerMasterVolumeThumbwheelSwitch;

	SwitchRow PadCommVolumeThumbwheelSwitchRow;
	ThumbwheelSwitch PadCommVolumeThumbwheelSwitch;

	SwitchRow IntercomVolumeThumbwheelSwitchRow;
	ThumbwheelSwitch IntercomVolumeThumbwheelSwitch;

	SwitchRow SBandVolumeThumbwheelSwitchRow;
	ThumbwheelSwitch SBandVolumeThumbwheelSwitch;

	SwitchRow VHFAMVolumeThumbwheelSwitchRow;
	ThumbwheelSwitch VHFAMVolumeThumbwheelSwitch;

	//////////////////////
	// Panel 6 switches //
	//////////////////////

	SwitchRow RightModeIntercomSwitchRow;
	ThreePosSwitch RightModeIntercomSwitch;

	SwitchRow RightAudioPowerSwitchRow;
	ThreePosSwitch RightAudioPowerSwitch;

	SwitchRow RightPadCommSwitchRow;
	ThreePosSwitch RightPadCommSwitch;

	SwitchRow RightIntercomSwitchRow;
	ThreePosSwitch RightIntercomSwitch;

	SwitchRow RightSBandSwitchRow;
	ThreePosSwitch RightSBandSwitch;

	SwitchRow RightVHFAMSwitchRow;
	ThreePosSwitch RightVHFAMSwitch;
	
	SwitchRow AudioControlSwitchRow;
	ToggleSwitch AudioControlSwitch;

	SwitchRow SuidPowerSwitchRow;
	ToggleSwitch SuidPowerSwitch;

	///////////////////////
	// Panel 16 switches //
	///////////////////////

	SwitchRow RightCOASPowerSwitchRow;
	ToggleSwitch RightCOASPowerSwitch;

	SwitchRow RightUtilityPowerSwitchRow;
	ToggleSwitch RightUtilityPowerSwitch;

	SwitchRow RightDockingTargetSwitchRow;
	ThreePosSwitch RightDockingTargetSwitch;

	//////////////////////
	// Panel 5 switches //
	//////////////////////

	SwitchRow GNPowerSwitchRow;
	ThreePosSwitch GNPowerSwitch;
	
	SwitchRow MainBusTieSwitchesRow;
	ThreePosSwitch MainBusTieBatAcSwitch;
	ThreePosSwitch MainBusTieBatBcSwitch;

	SwitchRow BatCHGRSwitchRow;
	ThreeSourceSwitch BatCHGRSwitch;

	SwitchRow NonessBusSwitchRow;
	ThreeSourceSwitch NonessBusSwitch;

	SwitchRow InteriorLightsFloodSwitchesRow;
	ToggleSwitch InteriorLightsFloodDimSwitch;
	ToggleSwitch InteriorLightsFloodFixedSwitch;

	//////////////////////
	// Panel 5 rotaries //
	//////////////////////
	
	SwitchRow RightInteriorLightRotariesRow;
	RotationalSwitch RightIntegralRotarySwitch;
	RotationalSwitch RightFloodRotarySwitch;

	//////////////////////
	// Panel 4 switches //
	//////////////////////

	SwitchRow SPSGaugingSwitchRow;
	ThreeSourceSwitch SPSGaugingSwitch;

	SwitchRow TelcomSwitchesRow;
	ThreeSourceSwitch TelcomGroup1Switch;
	ThreeSourceSwitch TelcomGroup2Switch;

	///////////////////////////////
	// Panel 275 circuit brakers //
	///////////////////////////////

	SwitchRow Panel275CircuitBrakersRow;
	CircuitBrakerSwitch InverterPower3MainBCircuitBraker;
	CircuitBrakerSwitch InverterPower3MainACircuitBraker;
	CircuitBrakerSwitch InverterPower2MainBCircuitBraker;
	CircuitBrakerSwitch InverterPower1MainACircuitBraker;
	CircuitBrakerSwitch FlightPostLandingMainBCircuitBraker;
	CircuitBrakerSwitch FlightPostLandingMainACircuitBraker;
	CircuitBrakerSwitch FlightPostLandingBatCCircuitBraker;
	CircuitBrakerSwitch FlightPostLandingBatBusBCircuitBraker;
	CircuitBrakerSwitch FlightPostLandingBatBusACircuitBraker;
	CircuitBrakerSwitch MainBBatBusBCircuitBraker;
	CircuitBrakerSwitch MainBBatCCircuitBraker;
	CircuitBrakerSwitch MainABatCCircuitBraker;
	CircuitBrakerSwitch MainABatBusACircuitBraker;

	////////////////////////
	// Right window cover //
	////////////////////////

	//SwitchRow RightWindowCoverRow;
	//ToggleSwitch RightWindowCoverSwitch;

	/////////////////////////////
	// G&N lower equipment bay //
	/////////////////////////////

	SwitchRow ModeSwitchRow;
	CMCOpticsModeSwitch ModeSwitch;

	SwitchRow ControllerSpeedSwitchRow;
	ThreePosSwitch ControllerSpeedSwitch;

	SwitchRow ControllerCouplingSwitchRow;
	ToggleSwitch ControllerCouplingSwitch;

	SwitchRow ControllerSwitchesRow;
	ThreePosSwitch ControllerTrackerSwitch;
	ThreePosSwitch ControllerTelescopeTrunnionSwitch;

	SwitchRow ConditionLampsSwitchRow;
	ThreePosSwitch ConditionLampsSwitch;

	SwitchRow UPTLMSwitchRow;
	ToggleSwitch UPTLMSwitch;

	SwitchRow OpticsHandcontrollerSwitchRow;
	OpticsHandcontrollerSwitch OpticsHandcontrollerSwitch;

	SwitchRow OpticsMarkButtonRow;
	PushSwitch OpticsMarkButton;

	SwitchRow OpticsMarkRejectButtonRow;
	PushSwitch OpticsMarkRejectButton;

	SwitchRow MinImpulseHandcontrollerSwitchRow;
	MinImpulseHandcontrollerSwitch MinImpulseHandcontrollerSwitch;


	///////////////
	// Panel 100 //
	///////////////

	SwitchRow Panel100SwitchesRow;
	ToggleSwitch UtilityPowerSwitch;
	ToggleSwitch Panel100FloodDimSwitch;	
	ToggleSwitch Panel100FloodFixedSwitch;
	ToggleSwitch GNPowerOpticsSwitch;
	GuardedToggleSwitch GNPowerIMUSwitch;
	ThreePosSwitch Panel100RNDZXPDRSwitch;

	SwitchRow Panel100LightingRoatariesRow;
	RotationalSwitch Panel100NumericRotarySwitch;
	RotationalSwitch Panel100FloodRotarySwitch;
	RotationalSwitch Panel100IntegralRotarySwitch;
	
	///////////////
	// Panel 101 //
	///////////////

	SwitchRow SystemTestRotariesRow;
	RotationalSwitch LeftSystemTestRotarySwitch;
	RotationalSwitch RightSystemTestRotarySwitch;

	SwitchRow RNDZXPDRSwitchRow;
	ToggleSwitch RNDZXPDRSwitch;

	SwitchRow Panel101LowerSwitchRow;
	ToggleSwitch CMRCSHTRSSwitch;
	ThreePosSwitch WasteH2ODumpSwitch;
	ThreePosSwitch UrineDumpSwitch;

	///////////////
	// Panel 163 //
	///////////////

	SwitchRow SCIUtilPowerSwitchRow;
	ToggleSwitch SCIUtilPowerSwitch;

	///////////////////////
	// Panel 15 switches //
	///////////////////////

	SwitchRow LeftCOASPowerSwitchRow;
	ToggleSwitch LeftCOASPowerSwitch;

	SwitchRow LeftUtilityPowerSwitchRow;
	ToggleSwitch LeftUtilityPowerSwitch;

	SwitchRow PostLandingBCNLTSwitchRow;
	ThreePosSwitch PostLandingBCNLTSwitch;

	SwitchRow PostLandingDYEMarkerSwitchRow;
	GuardedToggleSwitch PostLandingDYEMarkerSwitch;

	SwitchRow PostLandingVentSwitchRow;
	ThreePosSwitch PostLandingVentSwitch;

	/////////////////////////
	// Panel 9 thumbwheels //
	/////////////////////////

	SwitchRow LeftModeIntercomVOXSensThumbwheelSwitchRow;
	ThumbwheelSwitch LeftModeIntercomVOXSensThumbwheelSwitch;

	SwitchRow LeftPowerMasterVolumeThumbwheelSwitchRow;
	VolumeThumbwheelSwitch LeftPowerMasterVolumeThumbwheelSwitch;

	SwitchRow LeftPadCommVolumeThumbwheelSwitchRow;
	ThumbwheelSwitch LeftPadCommVolumeThumbwheelSwitch;

	SwitchRow LeftIntercomVolumeThumbwheelSwitchRow;
	ThumbwheelSwitch LeftIntercomVolumeThumbwheelSwitch;

	SwitchRow LeftSBandVolumeThumbwheelSwitchRow;
	ThumbwheelSwitch LeftSBandVolumeThumbwheelSwitch;

	SwitchRow LeftVHFAMVolumeThumbwheelSwitchRow;
	ThumbwheelSwitch LeftVHFAMVolumeThumbwheelSwitch;

	//////////////////////
	// Panel 9 switches //
	//////////////////////

	SwitchRow LeftModeIntercomSwitchRow;
	ThreePosSwitch LeftModeIntercomSwitch;

	SwitchRow LeftAudioPowerSwitchRow;
	ThreePosSwitch LeftAudioPowerSwitch;

	SwitchRow LeftPadCommSwitchRow;
	ThreePosSwitch LeftPadCommSwitch;

	SwitchRow LeftIntercomSwitchRow;
	ThreePosSwitch LeftIntercomSwitch;

	SwitchRow LeftSBandSwitchRow;
	ThreePosSwitch LeftSBandSwitch;

	SwitchRow LeftVHFAMSwitchRow;
	ThreePosSwitch LeftVHFAMSwitch;

	SwitchRow LeftAudioControlSwitchRow;
	ThreePosSwitch LeftAudioControlSwitch;

	SwitchRow LeftSuitPowerSwitchRow;
	ToggleSwitch LeftSuitPowerSwitch;

	SwitchRow VHFRNGSwitchRow;
	ToggleSwitch VHFRNGSwitch;

	SwitchRow FloatBagSwitchRow;
	ThreePosSwitch FloatBagSwitch1;
	ThreePosSwitch FloatBagSwitch2;
	ThreePosSwitch FloatBagSwitch3;

	SwitchRow SeqEventsContSystemSwitchesRow;
	ToggleSwitch SECSLogic1Switch;
	ToggleSwitch SECSLogic2Switch;
	ToggleSwitch PyroArmASwitch;
	ToggleSwitch PyroArmBSwitch;

	//////////////////////////////
	// Panel 8 circuit breakers //
	//////////////////////////////

	SwitchRow StabContSystemCircuitBrakerRow;
	CircuitBrakerSwitch StabContSystemTVCAc1CircuitBraker;
	CircuitBrakerSwitch StabContSystemAc1CircuitBraker;
	CircuitBrakerSwitch StabContSystemAc2CircuitBraker;

	SwitchRow StabilizationControlSystemCircuitBrakerRow;
	CircuitBrakerSwitch ECATVCAc2CircuitBraker;
	CircuitBrakerSwitch DirectUllMnACircuitBraker;
	CircuitBrakerSwitch DirectUllMnBCircuitBraker;
	CircuitBrakerSwitch ContrDirectMnA1CircuitBraker;
	CircuitBrakerSwitch ContrDirectMnB1CircuitBraker;
	CircuitBrakerSwitch ContrDirectMnA2CircuitBraker;
	CircuitBrakerSwitch ContrDirectMnB2CircuitBraker;
	CircuitBrakerSwitch ACRollMnACircuitBraker;
	CircuitBrakerSwitch ACRollMnBCircuitBraker;
	CircuitBrakerSwitch BDRollMnACircuitBraker;
	CircuitBrakerSwitch BDRollMnBCircuitBraker;
	CircuitBrakerSwitch PitchMnACircuitBraker;
	CircuitBrakerSwitch PitchMnBCircuitBraker;
	CircuitBrakerSwitch YawMnACircuitBraker;
	CircuitBrakerSwitch YawMnBCircuitBraker;

	SwitchRow StabilizationControlSystem2CircuitBrakerRow;
	CircuitBrakerSwitch OrdealAc2CircuitBraker;
	CircuitBrakerSwitch OrdealMnBCircuitBraker;
	CircuitBrakerSwitch ContrAutoMnACircuitBraker;
	CircuitBrakerSwitch ContrAutoMnBCircuitBraker;
	CircuitBrakerSwitch LogicBus12MnACircuitBraker;
	CircuitBrakerSwitch LogicBus34MnACircuitBraker;
	CircuitBrakerSwitch LogicBus14MnBCircuitBraker;
	CircuitBrakerSwitch LogicBus23MnBCircuitBraker;
	CircuitBrakerSwitch SystemMnACircuitBraker;
	CircuitBrakerSwitch SystemMnBCircuitBraker;

	SwitchRow ReactionControlSystemCircuitBrakerRow;
	CircuitBrakerSwitch CMHeater1MnACircuitBraker;
	CircuitBrakerSwitch CMHeater2MnBCircuitBraker;
	CircuitBrakerSwitch SMHeatersAMnBCircuitBraker;
	CircuitBrakerSwitch SMHeatersCMnBCircuitBraker;
	CircuitBrakerSwitch SMHeatersBMnACircuitBraker;
	CircuitBrakerSwitch SMHeatersDMnACircuitBraker;
	CircuitBrakerSwitch PrplntIsolMnACircuitBraker;
	CircuitBrakerSwitch PrplntIsolMnBCircuitBraker;
	CircuitBrakerSwitch RCSLogicMnACircuitBraker;
	CircuitBrakerSwitch RCSLogicMnBCircuitBraker;
	CircuitBrakerSwitch EMSMnACircuitBraker;
	CircuitBrakerSwitch EMSMnBCircuitBraker;
	CircuitBrakerSwitch DockProbeMnACircuitBraker;
	CircuitBrakerSwitch DockProbeMnBCircuitBraker;

	SwitchRow ServicePropulsionSysCircuitBrakerRow;
	CircuitBrakerSwitch GaugingMnACircuitBraker;
	CircuitBrakerSwitch GaugingMnBCircuitBraker;
	CircuitBrakerSwitch GaugingAc1CircuitBraker;
	CircuitBrakerSwitch GaugingAc2CircuitBraker;
	CircuitBrakerSwitch HeValveMnACircuitBraker;
	CircuitBrakerSwitch HeValveMnBCircuitBraker;
	CircuitBrakerSwitch PitchBatACircuitBraker;
	CircuitBrakerSwitch PitchBatBCircuitBraker;
	CircuitBrakerSwitch YawBatACircuitBraker;
	CircuitBrakerSwitch YawBatBCircuitBraker;
	CircuitBrakerSwitch PilotValveMnACircuitBraker;
	CircuitBrakerSwitch PilotValveMnBCircuitBraker;

	SwitchRow FloatBagCircuitBrakerRow;
	CircuitBrakerSwitch FloatBag1BatACircuitBraker;
	CircuitBrakerSwitch FloatBag2BatBCircuitBraker;
	CircuitBrakerSwitch FloatBag3FLTPLCircuitBraker;

	SwitchRow SeqEventsContSysCircuitBrakerRow;
	CircuitBrakerSwitch SECSLogicBatACircuitBraker;
	CircuitBrakerSwitch SECSLogicBatBCircuitBraker;
	CircuitBrakerSwitch SECSArmBatACircuitBraker;
	CircuitBrakerSwitch SECSArmBatBCircuitBraker;

	SwitchRow EDSCircuitBrakerRow;
	CircuitBrakerSwitch EDS1BatACircuitBraker;
	CircuitBrakerSwitch EDS2BatBCircuitBraker;
	CircuitBrakerSwitch EDS3BatCCircuitBraker;

	SwitchRow ELSCircuitBrakerRow;
	CircuitBrakerSwitch ELSBatACircuitBraker;
	CircuitBrakerSwitch ELSBatBCircuitBraker;

	SwitchRow PLVentCircuitBrakerRow;
	CircuitBrakerSwitch FLTPLCircuitBraker;

	//////////////////////
	// Panel 8 switches //
	//////////////////////
	
	SwitchRow AutoRCSSelectSwitchesRow;
	ThreeSourceSwitch AcRollA1Switch;
	ThreeSourceSwitch AcRollC1Switch;
	ThreeSourceSwitch AcRollA2Switch;
	ThreeSourceSwitch AcRollC2Switch;
	ThreeSourceSwitch BdRollB1Switch;
	ThreeSourceSwitch BdRollD1Switch;
	ThreeSourceSwitch BdRollB2Switch;
	ThreeSourceSwitch BdRollD2Switch;
	ThreeSourceSwitch PitchA3Switch;
	ThreeSourceSwitch PitchC3Switch;
	ThreeSourceSwitch PitchA4Switch;
	ThreeSourceSwitch PitchC4Switch;
	ThreeSourceSwitch YawB3Switch;
	ThreeSourceSwitch YawD3Switch;
	ThreeSourceSwitch YawB4Switch;
	ThreeSourceSwitch YawD4Switch;

	SwitchRow FloodDimSwitchRow;
	ToggleSwitch FloodDimSwitch;

	SwitchRow FloodFixedSwitchRow;
	ThreePosSwitch FloodFixedSwitch;

	//////////////////////
	// Panel 7 switches //
	//////////////////////

	SwitchRow EDSPowerSwitchRow;
	ToggleSwitch EDSPowerSwitch;

	SwitchRow TVCServoPowerSwitchesRow;
	ThreePosSwitch TVCServoPower1Switch;
	ThreePosSwitch TVCServoPower2Switch;

	SwitchRow LogicPowerSwitchRow;
	ToggleSwitch LogicPowerSwitch;

	SwitchRow SIGCondDriverBiasPowerSwitchesRow;
	ThreeSourceSwitch SIGCondDriverBiasPower1Switch;
	ThreeSourceSwitch SIGCondDriverBiasPower2Switch;

	//////////////////////
	// Panel 7 rotaries //
	//////////////////////

	SwitchRow LeftInteriorLightRotariesRow;
	RotationalSwitch NumericRotarySwitch;
	RotationalSwitch FloodRotarySwitch;
	RotationalSwitch IntegralRotarySwitch;

	SwitchRow FDAIPowerRotaryRow;
	FDAIPowerRotationalSwitch FDAIPowerRotarySwitch;

	SwitchRow SCSElectronicsPowerRotaryRow;
	RotationalSwitch SCSElectronicsPowerRotarySwitch;

	SwitchRow BMAGPowerRotary1Row;
	BMAGPowerRotationalSwitch BMAGPowerRotary1Switch;

	SwitchRow BMAGPowerRotary2Row;
	BMAGPowerRotationalSwitch BMAGPowerRotary2Switch;

	SwitchRow DirectO2RotaryRow;
	DirectO2RotationalSwitch DirectO2RotarySwitch;

	/////////////////
	// Panel 10/12 //
	/////////////////

	SwitchRow LeftAudioSwitchesRow;
	ThreePosSwitch ModeIntercomSwitch;
	ThreePosSwitch PadComSwitch;
	ThreePosSwitch SBandSwitch;

	SwitchRow LeftAudioThumbwheelsRow;
	ThumbwheelSwitch LeftAudioVOXSensThumbwheel;
	ThumbwheelSwitch LeftAudioPadComVolumeThumbwheel;
	ThumbwheelSwitch LeftAudioSBandVolumeThumbwheel;

	SwitchRow CenterAudioSwitchesRow;
	ToggleSwitch CenterSuitPowerSwitch;
	ToggleSwitch CenterAudioControlSwitch;

	SwitchRow RightAudioThumbwheelsRow;
	ThumbwheelSwitch RightAudioMasterVolumeThumbwheel;
	ThumbwheelSwitch RightAudioIntercomVolumeThumbwheel;
	ThumbwheelSwitch RightAudioVHFAMVolumeThumbwheel;

	SwitchRow RightAudioSwitchesRow;
	ThreePosSwitch PowerAudioSwitch;
	ThreePosSwitch IntercomSwitch;
	ThreePosSwitch VHFAMSwitch;

	SwitchRow LMTunnelVentValveRow;
	RotationalSwitch LMTunnelVentValve;

	///////////////////
	// Panel 225/226 //
	///////////////////
	
	SwitchRow Panel225CircuitBreakersRow;
	CircuitBrakerSwitch PCMTLMGroup1CB;
	CircuitBrakerSwitch PCMTLMGroup2CB;
	CircuitBrakerSwitch FLTBusMNACB;
	CircuitBrakerSwitch FLTBusMNBCB;
	CircuitBrakerSwitch PMPPowerPrimCB;
	CircuitBrakerSwitch PMPPowerAuxCB;
	CircuitBrakerSwitch VHFStationAudioLCB;
	CircuitBrakerSwitch VHFStationAudioCTRCB;
	CircuitBrakerSwitch VHFStationAudioRCB;
	CircuitBrakerSwitch UDLCB;
	CircuitBrakerSwitch HGAFLTBus1CB;
	CircuitBrakerSwitch HGAGroup2CB;
	CircuitBrakerSwitch SBandFMXMTRFLTBusCB;
	CircuitBrakerSwitch SBandFMXMTRGroup1CB;
	CircuitBrakerSwitch CentralTimingEquipMNACB;
	CircuitBrakerSwitch CentralTimingEquipMNBCB;
	CircuitBrakerSwitch RNDZXPNDRFLTBusCB;
	CircuitBrakerSwitch SIGCondrFLTBusCB;
	CircuitBrakerSwitch SBandPWRAmpl1FLTBusCB;
	CircuitBrakerSwitch SBandPWRAmpl1Group1CB;
	CircuitBrakerSwitch SBandPWRAmpl2FLTBusCB;
	CircuitBrakerSwitch SBandPWRAmpl2Group1CB;

	SwitchRow Panel226CircuitBreakersRow;
	CircuitBrakerSwitch FuelCell1PumpsACCB;
	CircuitBrakerSwitch FuelCell1ReacsCB;
	CircuitBrakerSwitch FuelCell1BusContCB;
	CircuitBrakerSwitch FuelCell1PrugeCB;
	CircuitBrakerSwitch FuelCell1RadCB;
	CircuitBrakerSwitch CryogenicH2HTR1CB;
	CircuitBrakerSwitch CryogenicH2HTR2CB;
	CircuitBrakerSwitch FuelCell2PumpsACCB;
	CircuitBrakerSwitch FuelCell2ReacsCB;
	CircuitBrakerSwitch FuelCell2BusContCB;
	CircuitBrakerSwitch FuelCell2PrugeCB;
	CircuitBrakerSwitch FuelCell2RadCB;
	CircuitBrakerSwitch CryogenicO2HTR1CB;
	CircuitBrakerSwitch CryogenicO2HTR2CB;
	CircuitBrakerSwitch FuelCell3PumpsACCB;
	CircuitBrakerSwitch FuelCell3ReacsCB;
	CircuitBrakerSwitch FuelCell3BusContCB;
	CircuitBrakerSwitch FuelCell3PrugeCB;
	CircuitBrakerSwitch FuelCell3RadCB;
	CircuitBrakerSwitch CryogenicQTYAmpl1CB;
	CircuitBrakerSwitch CryogenicQTYAmpl2CB;
	CircuitBrakerSwitch CryogenicFanMotorsAC1ACB;
	CircuitBrakerSwitch CryogenicFanMotorsAC1BCB;
	CircuitBrakerSwitch CryogenicFanMotorsAC1CCB;
	CircuitBrakerSwitch CryogenicFanMotorsAC2ACB;
	CircuitBrakerSwitch CryogenicFanMotorsAC2BCB;
	CircuitBrakerSwitch CryogenicFanMotorsAC2CCB;
	CircuitBrakerSwitch LightingRndzMNACB;
	CircuitBrakerSwitch LightingRndzMNBCB;
	CircuitBrakerSwitch LightingFloodMNACB;
	CircuitBrakerSwitch LightingFloodMNBCB;
	CircuitBrakerSwitch LightingFloodFLTPLCB;
	CircuitBrakerSwitch LightingNumIntLEBCB;
	CircuitBrakerSwitch LightingNumIntLMDCCB;
	CircuitBrakerSwitch LightingNumIntRMDCCB;
	CircuitBrakerSwitch RunEVATRGTAC1CB;
	CircuitBrakerSwitch RunEVATRGTAC2CB;
	
	///////////////
	// Panel 227 //
	///////////////

	//SwitchRow SCIInstSwitchRow;
	//ToggleSwitch SCIInstSwitch;

	////////////////////////////////////////
	// Panel 229 - Right Instrument Panel //
	////////////////////////////////////////

	SwitchRow Panel229CircuitBreakersRow;
	CircuitBrakerSwitch UtilityCB1;
	CircuitBrakerSwitch UtilityCB2;
	CircuitBrakerSwitch EPSBatBusACircuitBraker;
	CircuitBrakerSwitch EPSBatBusBCircuitBraker;

	///////////////////////
	// Panel 250/251/252 //
	///////////////////////

	SwitchRow Panel250CircuitBreakersRow;
	CircuitBrakerSwitch BatBusAToPyroBusTieCircuitBraker;
	CircuitBrakerSwitch PyroASeqACircuitBraker;
	CircuitBrakerSwitch BatBusBToPyroBusTieCircuitBraker;
	CircuitBrakerSwitch PyroBSeqBCircuitBraker;
	CircuitBrakerSwitch BatAPWRCircuitBraker;
	CircuitBrakerSwitch BatBPWRCircuitBraker;
	CircuitBrakerSwitch BatCPWRCircuitBraker;
	CircuitBrakerSwitch BatCtoBatBusACircuitBraker;
	CircuitBrakerSwitch BatCtoBatBusBCircuitBraker;
	CircuitBrakerSwitch BatCCHRGCircuitBraker;

	SwitchRow WasteMGMTOvbdDrainDumpRotaryRow;
	RotationalSwitch WasteMGMTOvbdDrainDumpRotary;

	SwitchRow WasteMGMTBatteryVentRotaryRow;
	RotationalSwitch WasteMGMTBatteryVentRotary;

	SwitchRow WasteMGMTStoageVentRotaryRow;
	RotationalSwitch WasteMGMTStoageVentRotary;
	
	///////////////////
	// Panel 276/278 //
	///////////////////

	SwitchRow Panel276CBRow;
	CircuitBrakerSwitch Panel276CB1;
	CircuitBrakerSwitch Panel276CB2;
	CircuitBrakerSwitch Panel276CB3;
	CircuitBrakerSwitch Panel276CB4;
	
	SwitchRow Panel278CBRow;
	CircuitBrakerSwitch UprightingSystemCompressor1CircuitBraker;
	CircuitBrakerSwitch UprightingSystemCompressor2CircuitBraker;
	CircuitBrakerSwitch SIVBLMSepPyroACircuitBraker;
	CircuitBrakerSwitch SIVBLMSepPyroBCircuitBraker;
	
	///////////////////////////////
	// Panel 300/301/302/303/305 //
	///////////////////////////////

	SwitchRow SuitCircuitFlow300SwitchRow;
	ThreePosSwitch SuitCircuitFlow300Switch;

	SwitchRow SuitCircuitFlow301SwitchRow;
	ThreePosSwitch SuitCircuitFlow301Switch;

	SwitchRow SuitCircuitFlow302SwitchRow;
	ThreePosSwitch SuitCircuitFlow302Switch;

	SwitchRow SecondaryCabinTempValveRow;
	RotationalSwitch SecondaryCabinTempValve;

	SwitchRow FoodPreparationWaterLeversRow;
	RotationalSwitch FoodPreparationWaterHotLever;
	RotationalSwitch FoodPreparationWaterColdLever;
	
	////////////////////////
	// Panel 325/326 etc. //
	////////////////////////

	SwitchRow GlycolToRadiatorsLeverRow; 	
	CircuitBrakerSwitch GlycolToRadiatorsLever;

	SwitchRow CabinPressureReliefLever1Row;
	ThumbwheelSwitch CabinPressureReliefLever1;

	SwitchRow CabinPressureReliefLever2Row;
	SaturnCabinPressureReliefLever CabinPressureReliefLever2;

	SwitchRow GlycolReservoirRotariesRow;
	RotationalSwitch GlycolReservoirInletRotary;
	RotationalSwitch GlycolReservoirBypassRotary;
	RotationalSwitch GlycolReservoirOutletRotary;

	SwitchRow OxygenRotariesRow;
	RotationalSwitch OxygenSurgeTankRotary;
	RotationalSwitch OxygenSMSupplyRotary;
	RotationalSwitch OxygenRepressPackageRotary;

	///////////////
	// Panel 380 //
	///////////////


	SwitchRow SuitCircuitReturnValveLeverRow;
	CircuitBrakerSwitch SuitCircuitReturnValveLever;

	SwitchRow O2DemandRegulatorRotaryRow;
	RotationalSwitch O2DemandRegulatorRotary;

	SwitchRow SuitTestRotaryRow;
	SuitTestSwitch SuitTestRotary;


	///////////////////////////
	// Panel 375/377/378/379 //
	///////////////////////////

	SwitchRow OxygenSurgeTankValveRotaryRow;
	RotationalSwitch OxygenSurgeTankValveRotary;

	SwitchRow GlycolToRadiatorsRotaryRow;
	RotationalSwitch GlycolToRadiatorsRotary;

	SwitchRow AccumRotaryRow;
	RotationalSwitch AccumRotary;

	SwitchRow GlycolRotaryRow;
	RotationalSwitch GlycolRotary;

	///////////////
	// Panel 351 //
	///////////////

	SwitchRow CabinRepressValveRotaryRow;
	RotationalSwitch CabinRepressValveRotary;
	
	SwitchRow WaterGlycolTanksRotariesRow;
	RotationalSwitch SelectorInletValveRotary;
	RotationalSwitch SelectorOutletValveRotary;

	SwitchRow EmergencyCabinPressureRotaryRow;
	RotationalSwitch EmergencyCabinPressureRotary;

	///////////////
	// Panel 352 //
	///////////////

	SwitchRow WaterControlPanelRow;
	RotationalSwitch PressureReliefRotary;
	RotationalSwitch WasteTankInletRotary;
	RotationalSwitch PotableTankInletRotary;
	RotationalSwitch WasteTankServicingRotary;

	///////////////////////
	// Panel 13 switches //
	///////////////////////

	SwitchRow ORDEALSwitchesRow;
	ToggleSwitch ORDEALFDAI1Switch;
	ToggleSwitch ORDEALFDAI2Switch;
	ThreePosSwitch ORDEALEarthSwitch;
	ThreePosSwitch ORDEALLightingSwitch; 
	ToggleSwitch ORDEALModeSwitch;	
	ThreePosSwitch ORDEALSlewSwitch;

	SwitchRow ORDEALRotaryRow;
	RotationalSwitch ORDEALAltSetRotary;

	///////////////////////
	// Hatch             //
	///////////////////////

	SwitchRow HatchGearBoxSelectorRow;
	RotationalSwitch HatchGearBoxSelector;

	SwitchRow HatchActuatorHandleSelectorRow;
	RotationalSwitch HatchActuatorHandleSelector;

	SwitchRow HatchVentValveRotaryRow;
	RotationalSwitch HatchVentValveRotary;

	SwitchRow HatchActuatorHandleSelectorOpenRow;
	RotationalSwitch HatchActuatorHandleSelectorOpen;

	SwitchRow HatchToggleRow;
	PushSwitch HatchToggle;


	///
	/// Stage is the main stage of the flight.
	/// \brief Flight stage.
	///
	int stage;

	///
	/// StageState is the current state within the flight stage.
	/// \brief State within flight stage.
	///
	int StageState;

	///
	/// Which type of Saturn are we simulating? e.g. Saturn 1b, Saturn V, INT-20.
	/// \brief Saturn rocket type.
	///
	int SaturnType;

	#define SATSYSTEMS_NONE				  0
	#define SATSYSTEMS_PRELAUNCH		100
	#define SATSYSTEMS_CREWINGRESS_1	200
	#define SATSYSTEMS_CREWINGRESS_2	210
	#define SATSYSTEMS_CABINCLOSEOUT	300
	#define SATSYSTEMS_GSECONNECTED_1	400
	#define SATSYSTEMS_GSECONNECTED_2	410
	#define SATSYSTEMS_READYTOLAUNCH    500
	#define SATSYSTEMS_CABINVENTING		600
	#define SATSYSTEMS_FLIGHT			700
	#define SATSYSTEMS_LANDING			800

	int systemsState;
	bool firstSystemsTimeStepDone;
	double lastSystemsMissionTime;

	//
	// Stage masses: should really be saved, but probably aren't at the
	// moment.
	//

	///
	/// Command module total mass in kg.
	/// \brief CM total mass.
	///
	double CM_Mass;

	///
	/// Command module empty mass in kg.
	/// \brief CM empty mass.
	///
	double CM_EmptyMass;

	///
	/// Command module fuel mass in kg.
	/// \brief CM fuel mass.
	///
	double CM_FuelMass;

	///
	/// Service module total mass in kg.
	/// \brief SM total mass.
	///
	double SM_Mass;

	///
	/// Service module empty mass in kg.
	/// \brief SM empty mass.
	///
	double SM_EmptyMass;

	///
	/// Service module fuel mass in kg.
	/// \brief SM fuel mass.
	///
	double SM_FuelMass;

	///
	/// SIVb payload mass in kg.
	/// \brief SIVb payload mass.
	///
	double S4PL_Mass;

	double Abort_Mass;
	double Interstage_Mass;
	double S4B_Mass;
	double S4B_EmptyMass;
	double S4B_FuelMass;
	double SII_Mass;
	double SII_EmptyMass;
	double SII_FuelMass;
	double SI_Mass;
	double SI_EmptyMass;
	double SI_FuelMass;

	bool SI_MassLoaded;
	bool SII_MassLoaded;
	bool S4B_MassLoaded;

	bool S1_ThrustLoaded;
	bool S2_ThrustLoaded;
	bool S3_ThrustLoaded;

	//
	// Calculated total stage mass.
	//

	double Stage1Mass;
	double Stage2Mass;
	double Stage3Mass;

	//
	// Engine numbers.
	//

	///
	/// \brief Number of engines on stage one.
	///
	int SI_EngineNum;

	///
	/// \brief Number of engines on stage two.
	///
	int SII_EngineNum;

	///
	/// \brief Number of engines on stage three.
	///
	int SIII_EngineNum;

	///
	/// Engine indicator lights on control panel: true for lit. Five used on Saturn V, eight on
	/// Saturn 1b.
	/// \brief Engine indicator lights.
	///
	bool ENGIND[9];
	bool LAUNCHIND[8];
	bool LVGuidLight;
	bool LVRateLight;

	//
	// And state that doesn't need to be saved.
	//

	double aVAcc;
	double aVSpeed;
	double aHAcc;
	double aZAcc;

	///
	/// Mesh offset for BPC and LET.
	/// \brief Tower mesh offset.
	///
	double TowerOffset;

	///
	/// Mesh offset for SIVb.
	/// \brief SIVb mesh offset.
	///
	double S4Offset;

	double actualVEL;
	double actualALT;
	double actualFUEL;

	#define LASTVELOCITYCOUNT 50
	VECTOR3 LastVelocity[LASTVELOCITYCOUNT];
	double LastVerticalVelocity[LASTVELOCITYCOUNT];
	double LastSimt[LASTVELOCITYCOUNT];
	int LastVelocityFilled;

	double ThrustAdjust;
	double MixtureRatio;

	bool KEY1;
	bool KEY2;
	bool KEY3;
	bool KEY4;
	bool KEY5;
	bool KEY6;
	bool KEY7;
	bool KEY8;
	bool KEY9;

	//
	// State variables that must be initialised but not saved.
	//

	///
	/// \brief Flag for first timestep.
	///
	bool FirstTimestep;

	///
	/// \brief Flag for first call to generic timestep function.
	///
	bool GenericFirstTimestep;

	int CurrentTimestep;
	int LongestTimestep;
	double LongestTimestepLength;
	VECTOR3 normal;

	PanelSwitches MainPanel;
	PanelSwitchScenarioHandler PSH;

	SwitchRow SequencerSwitchesRow;
	SwitchRow SeparationSwitchesRow;
	SwitchRow CryoTankSwitchesRow;
	SwitchRow FuelCellPhRadTempIndicatorsRow;
	SwitchRow FuelCellRadiatorsIndicatorsRow;
	SwitchRow FuelCellRadiatorsSwitchesRow;
	SwitchRow FuelCellIndicatorsSwitchRow;
	SwitchRow FuelCellHeatersSwitchesRow;
	SwitchRow FuelCellPurgeSwitchesRow;
	SwitchRow FuelCellReactantsIndicatorsRow;
	SwitchRow FuelCellReactantsSwitchesRow;
	SwitchRow FuelCellLatchSwitchesRow;
	SwitchRow FuelCellPumpsSwitchesRow;
	SwitchRow SuitCompressorSwitchesRow;
	SwitchRow MainBusAIndicatorsRow;

	SwitchRow SBandNormalSwitchesRow;

	///////////////////////////////////////////////////////
	// Internal systems devices.						 //
	///////////////////////////////////////////////////////

	// SCS components
	BMAG bmag1;
	BMAG bmag2;
	GDC  gdc;
	ASCP ascp;
	EDA  eda;
	RJEC rjec;
	ECA  eca;
	// Telecom equipment
	DSE  dataRecorder;
	PCM  pcm;
	PMP	 pmp;
	USB  usb;
	EMS  ems;

	// CM Optics
	CMOptics optics;

	// Fuel cells	
	FCell *FuelCells[3];
	Boiler *FuelCellHeaters[3];
	Cooling *FuelCellCooling[3];

	// O2 tanks.
	h_Tank *O2Tanks[2];
	Boiler *O2TanksHeaters[2];
	Boiler *O2TanksFans[2];

	// H2 tanks
	h_Tank *H2Tanks[2];
	Boiler *H2TanksHeaters[2];
	Boiler *H2TanksFans[2];

	// Main bus A and B.
	DCbus *MainBusA;
	DCbus *MainBusB;

	// Non-essential buses... oddly, these are for non-essential systems.
	DCbus NonEssBus1;
	DCbus NonEssBus2;

	DCBusController MainBusAController;
	DCBusController MainBusBController;

	// Flight Bus.
	// This gets fed via diodes from MNA and MNB through 20-amp circuit breakers on panel 225.
	DCbus FlightBus;
	PowerMerge FlightBusFeeder;

	// Flight/Post Landing Bus.
	// This gets fed via various circuit breakers on panel 275.
	DCbus FlightPostLandingBus;
	NWayPowerMerge FlightPostLandingBusFeeder;

	// Inverters.
	ACInverter *Inverter1;
	ACInverter *Inverter2;
	ACInverter *Inverter3;

	// AC bus 1 and 2, which are three-phase.
	ACbus ACBus1PhaseA;
	ACbus ACBus1PhaseB;
	ACbus ACBus1PhaseC;

	ACbus ACBus2PhaseA;
	ACbus ACBus2PhaseB;
	ACbus ACBus2PhaseC;

	ThreeWayPowerMerge ACBus1;
	ThreeWayPowerMerge ACBus2;

	Battery *EntryBatteryA;
	Battery *EntryBatteryB;
	Battery *EntryBatteryC;

	Battery *PyroBatteryA;
	Battery *PyroBatteryB;

	PowerMerge BatteryBusA;
	PowerMerge BatteryBusB;

	PowerMerge BatteryRelayBus;

	BatteryCharger BatteryCharger;

	PowerMerge PyroBusAFeeder;
	PowerMerge PyroBusBFeeder;
	DCbus PyroBusA;
	DCbus PyroBusB;

	DCbus SECSLogicBusA;
	DCbus SECSLogicBusB;

	PowerMerge SwitchPower;
	PowerMerge GaugePower;

	// ECS
	h_HeatExchanger *PrimCabinHeatExchanger;
	h_HeatExchanger *SecCabinHeatExchanger;
	h_HeatExchanger *PrimEcsRadiatorExchanger1;
	h_HeatExchanger *PrimEcsRadiatorExchanger2;
	h_HeatExchanger *SecEcsRadiatorExchanger1;
	h_HeatExchanger *SecEcsRadiatorExchanger2;
	Boiler *CabinHeater;
	Boiler *PrimECSTestHeater;
	Boiler *SecECSTestHeater;
	AtmRegen *SuitCompressor1;
	AtmRegen *SuitCompressor2;
	h_crew *Crew;
	CabinPressureRegulator CabinPressureRegulator;
	O2DemandRegulator O2DemandRegulator;
	CabinPressureReliefValve CabinPressureReliefValve1;
	CabinPressureReliefValve CabinPressureReliefValve2;
	SuitCircuitReturnValve SuitCircuitReturnValve;
	O2SMSupply O2SMSupply;
	CrewStatus CrewStatus;
	SaturnSideHatch SideHatch;

	// RHC/THC 
	PowerMerge RHCNormalPower;
	PowerMerge RHCDirect1Power;
	PowerMerge RHCDirect2Power;

	// CSM has two DSKYs: one is in the main panel, the other is below. For true realism we should support
	// both.
	DSKY dsky;
	DSKY dsky2;
	CSMcomputer agc;	
	IMU imu;
	IU iu;
	CSMCautionWarningSystem cws;
	DockingProbe dockingprobe;
	SECS secs;
	ELS els;

	Pyro CMSMPyros;
	Pyro CMDockingRingPyros;
	Pyro CSMLVPyros;
	PowerMerge CMSMPyrosFeeder;
	PowerMerge CMDockingRingPyrosFeeder;
	PowerMerge CSMLVPyrosFeeder;


	//
	// LM PAD
	//

	int LMPadCount;
	unsigned int *LMPad;

	//
	// Do we have a crew, or is this an unmanned flight?
	//

	bool Crewed;

	//
	// Unmanned flight parameters.
	//

	bool AutoSlow;

	int SIVBPayload;

	//
	// CSM seperation info for unmanned flights.
	//

	bool CSMSepSet;
	double CSMSepTime;
	bool CMSepSet;
	double CMSepTime;

	//
	// Payload deploy time for unmanned flights with no CSM.
	//

	bool PayloadDeploySet;
	double PayloadDeployTime;

	//
	// SIVB burn info for unmanned flights.
	//

	double SIVBBurnStart;
	double SIVBApogee;

	//
	// CSM burn information in unmanned flights.
	//

	bool CSMBurn;
	double CSMBurnStart;
	double CSMApogee;
	double CSMPerigee;

	//
	// Pre-entry CSM acceleration and end time for unmanned flights.
	//

	bool CSMAccelSet;
	double CSMAccelTime;
	double CSMAccelEnd;
	double CSMAccelPitch;

	//
	// Quickstart Mode settings
	//

	bool ChecklistAutoDisabled;
	bool OrbiterAttitudeDisabled;
	bool SequencerSwitchLightingDisabled;

	//
	// Misc. settings
	//

	bool TLICapableBooster;
	bool TLISoundsLoaded;
	bool SkylabSM;
	bool NoHGA;
	bool SkylabCM;
	bool S1bPanel;
	bool bAbort;
	bool bRecovery;

	#define SATVIEW_LEFTSEAT		0
	#define SATVIEW_RIGHTSEAT		1
	#define SATVIEW_CENTERSEAT		2
	#define SATVIEW_LEFTDOCK		3
	#define SATVIEW_RIGHTDOCK		4
	#define SATVIEW_GNPANEL			5
	#define SATVIEW_ENG1			10
	#define SATVIEW_ENG2			11
	#define SATVIEW_ENG3			12
	#define SATVIEW_ENG4			13
	#define SATVIEW_ENG5			14
	#define SATVIEW_ENG6			15

	unsigned int	viewpos;

	// Mesh indexes
	int probeidx;
	int probeextidx;
	int crewidx;
	int cmpidx;
	int sidehatchidx;
	int sidehatchopenidx;
	int sidehatchburnedidx;
	int sidehatchburnedopenidx;

	bool ASTPMission;

	double DockAngle;
	double SeparationSpeed;

	double AtempP;
	double AtempY;
	double AtempR;

	ELEMENTS elemSaturn1B;
	double refSaturn1B;
	ELEMENTS elemPREV;
	double refPREV;
	double AltitudePREV;

	double 	Offset1st;

	bool StopRot;
	bool PayloadDataTransfer;

	//
	// Panels
	//

#define SATPANEL_MAIN				0 // Both have Orbiter's 
#define SATPANEL_MAIN_LEFT		    0 // default panel id 0
#define SATPANEL_GN					1
#define SATPANEL_GN_CENTER			1 // Should never have both full Lower and Center Lower available.
#define SATPANEL_LEFT				2
#define SATPANEL_RIGHT				3
#define SATPANEL_LEFT_RNDZ_WINDOW	4
#define SATPANEL_RIGHT_RNDZ_WINDOW	5
#define SATPANEL_HATCH_WINDOW		6
#define SATPANEL_CABIN_PRESS_PANEL	7
#define SATPANEL_TELESCOPE			8
#define SATPANEL_SEXTANT			9
#define SATPANEL_MAIN_MIDDLE		10
#define SATPANEL_MAIN_RIGHT			11
#define SATPANEL_GN_LEFT			12
#define SATPANEL_GN_RIGHT			13
#define SATPANEL_LOWER_LEFT			14
#define SATPANEL_LOWER_MAIN			15
#define SATPANEL_RIGHT_CB			16

	int  PanelId;
	int MainPanelSplit;
	int GNSplit;
	bool InVC;
	bool InPanel;
	bool CheckPanelIdInTimestep;
	bool RefreshPanelIdInTimestep;
	bool FovFixed;
	int FovExternal;
	double FovSave;
	double FovSaveExternal;

	//
	// Virtual cockpit
	//

	bool VCRegistered;
	VECTOR3 VCCameraOffset;
	VECTOR3 VCMeshOffset;

	bool KranzPlayed;
	bool PostSplashdownPlayed;
	bool SplashdownPlayed;

	OBJHANDLE hEVA;

	SoundLib soundlib;

	TimedSoundManager timedSounds;

	//
	// Surfaces.
	//

	SURFHANDLE srf[nsurf];  // handles for panel bitmaps.
	SURFHANDLE SMExhaustTex;
	SURFHANDLE CMTex;
	SURFHANDLE J2Tex;
	SURFHANDLE SIVBRCSTex;

	/// \todo SURFHANDLEs for VC

	//
	// Vessel handles.
	//

	OBJHANDLE hLMV;
	OBJHANDLE hstg1;
	OBJHANDLE hstg2;
	OBJHANDLE hintstg;
	OBJHANDLE hesc1;
	OBJHANDLE hPROBE;
	OBJHANDLE hs4bM;
	OBJHANDLE hs4b1;
	OBJHANDLE hs4b2;
	OBJHANDLE hs4b3;
	OBJHANDLE hs4b4;
	OBJHANDLE habort;
	OBJHANDLE hMaster;
	OBJHANDLE hSMJet;
	OBJHANDLE hVAB;
	OBJHANDLE hCrawler;
	OBJHANDLE hML;
	OBJHANDLE hMSS;
	OBJHANDLE hApex;
	OBJHANDLE hDrogueChute;
	OBJHANDLE hMainChute;

	//
	// ISP and thrust values, which vary depending on vehicle number.
	//

	double ISP_FIRST_SL;
	double ISP_FIRST_VAC;
	double ISP_SECOND_SL;//300*G;
	double ISP_SECOND_VAC;//421*G;
	double ISP_THIRD_VAC;//421*G;
	double ISP_LET_VAC;
	double ISP_LET_SL;

	double THRUST_FIRST_VAC;
	double THRUST_SECOND_VAC;//115200*G;
	double THRUST_THIRD_VAC;
	double THRUST_VAC_LET;

	//
	// Generic functions shared between SaturnV and Saturn1B
	//

	void AddRCSJets(double TRANZ,double MaxThrust);
	void AddRCS_S4B();
	void SetRecovery();
	void InitPanel(int panel);
	void SetSwitches(int panel);
	void AddLeftMainPanelAreas();
	void AddLeftMiddleMainPanelAreas(int offset);
	void AddRightMiddleMainPanelAreas(int offset);
	void AddRightMainPanelAreas(int offset);
	void AddLeftLowerPanelAreas();
	void AddLeftCenterLowerPanelAreas(int offset);
	void AddCenterLowerPanelAreas(int offset);
	void AddRightCenterLowerPanelAreas(int offset);
	void AddRightLowerPanelAreas(int offset);
	void ReleaseSurfaces();
	void KillDist(OBJHANDLE &hvessel, double kill_dist = 5000.0);
	void KillAlt(OBJHANDLE &hvessel,double altVS);
	void RedrawPanel_Alt (SURFHANDLE surf);
	void RedrawPanel_Alt2 (SURFHANDLE surf);
	void RedrawPanel_MFDButton (SURFHANDLE surf, int mfd, int side, int xoffset, int yoffset, int ydist);
	void CryoTankHeaterSwitchToggled(ToggleSwitch *s, int *pump);
	void FuelCellHeaterSwitchToggled(ToggleSwitch *s, int *pump);
	void FuelCellPurgeSwitchToggled(ToggleSwitch *s, int *start);
	void FuelCellReactantsSwitchToggled(ToggleSwitch *s, int *start);
	void CabinTempAutoSwitchToggled();
	void MousePanel_MFDButton(int mfd, int event, int mx, int my);
	double SetPitchApo();
	void SetStage(int s);
	void initSaturn();
	void SwitchClick();
	void ProbeSound();
	void CabinFanSound();
	void StopCabinFanSound();
	void CabinFansSystemTimestep();
	void ButtonClick();
	void GuardClick();
	void SetView();
	void SetView(double offset);
	void SetView(bool update_direction);
	void SetView(double offset, bool update_direction);
	void MasterAlarm();
	void StopMasterAlarm();
	void GenericTimestep(double simt, double simdt, double mjd);
	void SystemsInit();
	void SystemsTimestep(double simt, double simdt, double mjd);
	void SystemsInternalTimestep(double simdt);
	void JoystickTimestep();
	void SetSIVBThrusters(bool active);
	void LimitSetThrusterDir (THRUSTER_HANDLE th, const VECTOR3 &dir);
	void AttitudeLaunchSIVB();
	void LinearGuidance(VECTOR3 &target, double &pitch, double &yaw);
	virtual void AutoPilot(double autoT) = 0;

	void RenderS1bEngineLight(bool EngineOn, SURFHANDLE dest, SURFHANDLE src, int xoffs, int yoffs);

	void ClearPropellants();
	void ClearThrusters();
	virtual void SeparateStage (int stage) = 0;
	virtual void ConfigureStageMeshes(int stage_state) = 0;
	virtual void ConfigureStageEngines(int stage_state) = 0;
	virtual void CreateStageOne() = 0;

	void StageOrbitSIVB(double simt, double simdt);
	void StageSix(double simt);
	void JostleViewpoint(double amount);
	double CalculateApogeeTime();
	void UpdatePayloadMass();
	double GetCPitch(double t);
	double GetJ2ISP(double ratio);
	void StartAbort();
	void GetPayloadName(char *s);
	void GetApolloName(char *s);
	void AddSM(double offet, bool showSPS);

	//
	// Systems functions.
	//

	bool AutopilotActive();
	bool CabinFansActive();
	bool CabinFan1Active();
	bool CabinFan2Active();
	void ActivateCSMRCS();
	void DeactivateCSMRCS();
	void ActivateCMRCS();
	void DeactivateCMRCS();
	void FuelCellCoolingBypass(int fuelcell, bool bypassed);
	bool FuelCellCoolingBypassed(int fuelcell);
	void SetRandomFailures();
	void SetPipeMaxFlow(char *pipe, double flow);
	virtual void ActivatePrelaunchVenting() = 0;
	virtual void DeactivatePrelaunchVenting() = 0;

	//
	// Save/Load support functions.
	//

	int GetMainState();
	void SetMainState(int s);
	int GetAttachState();
	void SetAttachState(int s);
	int GetLaunchState();
	void SetLaunchState(int s);
	int GetSLAState();
	void SetSLAState(int s);

	///
	/// Get the Apollo 13 state flags as an int.
	/// \return 32-bit int representing the state flags.
	///
	int GetA13State();

	///
	/// Set the Apollo 13 state flags based on an int value.
	/// \param s This 32-bit int has the packed Apollo 13 flags.
	///
	void SetA13State(int s);
	int GetLightState();
	void SetLightState(int s);
	void GenericLoadStateSetup();

	virtual void SetVehicleStats() = 0;
	virtual void CalculateStageMass () = 0;
	virtual void SaveVehicleStats(FILEHANDLE scn) = 0;

	void GetScenarioState (FILEHANDLE scn, void *status);
	bool ProcessConfigFileLine (FILEHANDLE scn, char *line);

	void ClearPanelSDKPointers();

	//
	// Mission stage functions.
	//

	void DoLaunch(double simt);
	void LaunchCountdown(double simt);
	void StageSeven(double simt);
	void StageEight(double simt);
	void SetChuteStage1();
	void SetChuteStage2();
	void SetChuteStage3();
	void SetChuteStage4();
	void SetSplashStage();
	void SetCSMStage();
	void CreateSIVBStage(char *config, VESSELSTATUS &vs1, bool SaturnVStage);
	void SetReentryStage();
	void SetReentryMeshes();
	void AddRCS_CM(double MaxThrust, double offset = 0.0, bool createThrusterGroups = true);
	void GenericTimestepStage(double simt, double simdt);
	bool CheckForLaunchShutdown();
	void SetGenericStageState();
	void DestroyStages(double simt);
	void SIVBBoiloff();
	void LookForSIVb();
	void LookForLEM();
	void FireSeperationThrusters(THRUSTER_HANDLE *pth);
	void LoadDefaultSounds();
	void RCSSoundTimestep();

	//
	// Sounds
	//

	Sound Sclick;
	Sound Bclick;
	Sound Gclick;
	Sound ThumbClick;
	Sound SMasterAlarm;
	Sound LaunchS;
	Sound Scount;
	Sound SplashS;
	Sound Swater;
	Sound PostSplashdownS;
	Sound TowerJS;
	Sound StageS;
	Sound Sctdw;
	Sound SeparationS;
	Sound SMJetS;
	Sound S4CutS;
	Sound SShutS;
	Sound S2ShutS;
	Sound SepS;
	Sound CrashBumpS;
	Sound Psound;
	Sound CabinFans;
	Sound SwindowS;
	Sound SKranz;
	Sound SExploded;
	Sound SApollo13;
	Sound SSMSepExploded;
	Sound SPUShiftS;
	Sound SDockingCapture;
	Sound SDockingLatch;
	Sound SDockingExtend;
	Sound SUndock;
	Sound CabincloseoutS;
	Sound STLI;
	Sound STLIStart;
	Sound SecoSound;
	Sound PostLandingVentSound;
	Sound CrewDeadSound;
	Sound RCSFireSound;
	Sound RCSSustainSound;
	Sound SCorrection;

	///
	/// Drogue deployment message.
	///
	/// \brief 'Drogues' sound.
	///
	Sound DrogueS;

	//
	// General engine resources.
	//

	//
	// Propellant resources.
	//

	//
	// Main fuel for stage 1, 2, and 3.
	//

	PROPELLANT_HANDLE ph_1st, ph_2nd, ph_3rd;

	//
	// SM RCS
	//

	PROPELLANT_HANDLE ph_rcs0, ph_rcs1, ph_rcs2, ph_rcs3;

	//
	// CM RCS
	//

	PROPELLANT_HANDLE ph_rcs_cm_1, ph_rcs_cm_2;

	//
	// Fake propellant for fake thrusters spewing out junk at stage
	// seperation.
	//

	PROPELLANT_HANDLE ph_sep, ph_sep2;

	//
	// SPS and LET.
	//

	PROPELLANT_HANDLE ph_sps, ph_let;

	//
	// Ullage rockets for stage 1, 2 and 3.
	//

	PROPELLANT_HANDLE ph_ullage1, ph_ullage2, ph_ullage3;

	///
	/// Fake propellant for O2 vents.
	///
	PROPELLANT_HANDLE ph_o2_vent;

	//
	// Thruster group handles. We have a lot of these :).
	//

	THGROUP_HANDLE thg_main, thg_ull, thg_ver, thg_let;
	THGROUP_HANDLE thg_retro1, thg_retro2, thg_aps;

	THRUSTER_HANDLE th_main[5], th_ull[8], th_ver[3];                       // handles for orbiter main engines
	THRUSTER_HANDLE th_let[4];
	THRUSTER_HANDLE th_att_rot[24], th_att_lin[24];              
	THRUSTER_HANDLE	th_aps[3];
	THRUSTER_HANDLE	th_sep[8], th_sep2[8];
	THRUSTER_HANDLE th_rcs_a[8], th_rcs_b[8], th_rcs_c[8], th_rcs_d[8];		// SM RCS quads. Entry zero is not used, to match Apollo numbering
	THRUSTER_HANDLE th_att_cm[12], th_att_cm_sys1[6], th_att_cm_sys2[6];    // CM RCS  
	THRUSTER_HANDLE th_o2_vent;
	bool th_att_cm_commanded[12];

	PSTREAM_HANDLE dyemarker;
	PSTREAM_HANDLE cmrcsdump[12];
	PSTREAM_HANDLE prelaunchvent[3];
	PSTREAM_HANDLE stagingvent[8];
	PSTREAM_HANDLE contrail[8];
	double contrailLevel;

	CMRCSPropellantSource CMRCS1;
	CMRCSPropellantSource CMRCS2;

	SMRCSPropellantSource SMQuadARCS;
	SMRCSPropellantSource SMQuadBRCS;
	SMRCSPropellantSource SMQuadCRCS;
	SMRCSPropellantSource SMQuadDRCS;

	SPSEngine SPSEngine;
	SPSPropellantSource SPSPropellant;
	Boiler *SPSPropellantLineHeaterA;
	Boiler *SPSPropellantLineHeaterB;

	//
	// LEM data.
	//

	double LMLandingLatitude;
	double LMLandingLongitude;
	double LMLandingAltitude;
	char   LMLandingBase[256];
	double LMLandingMJD;		// MJD of lunar landing
	double LMDescentFuelMassKg;	///< Mass of fuel in descent stage of LEM.
	double LMAscentFuelMassKg;	///< Mass of fuel in ascent stage of LEM.

	//
	// Earth landing data.
	//

	double SplashdownLatitude;
	double SplashdownLongitude;
	double EntryInterfaceMJD;			// MJD of normal return entry interface

	//
	// Mission TLI and LOI parameters
	//

	double TransLunarInjectionMJD;			// MJD of TLI burn
	double LunarOrbitInsertionMJD;			// MJD of first LOI burn
	double FreeReturnPericynthionMJD;		// MJD of free return pericynthion
	double FreeReturnPericynthionAltitude;	// Altitude of free return pericynthion
	double TransLunarInjectionOffsetLon;	// TLI target offset
	double TransLunarInjectionOffsetLat;	// TLI target offset
	double TransLunarInjectionOffsetRad;	// TLI target offset

	//
	// Random motion.
	//

	double ViewOffsetx;
	double ViewOffsety;
	double ViewOffsetz;

	//
	// Save the last view offset.
	//

	double CurrentViewOffset;

	///
	/// Time of last timestep call.
	///
	double LastTimestep;

	//
	// Panel flash.
	//

	double NextFlashUpdate;
	bool PanelFlashOn;

	///
	/// Audio language.
	///
	char AudioLanguage[64];

	///
	/// SIVB payload name
	///
	char PayloadName[64];

	///
	/// LEM checklist file
	///
	char LEMCheck[100];

	bool LEMCheckAuto;

	//
	// Connectors.
	//

	///
	/// \brief MFD to panel connector.
	///
	PanelConnector MFDToPanelConnector;

	///
	/// \brief Connector from CSM to IU.
	///
	CSMToIUConnector iuCommandConnector;
	SaturnToIUCommandConnector sivbCommandConnector;

	MultiConnector CSMToSIVBConnector;
	MultiConnector CSMToLEMConnector;

	///
	/// \brief SIVb control connector for venting.
	///
	CSMToSIVBControlConnector sivbControlConnector;

	PowerSourceConnectorObject SIVBToCSMPowerSource;
	Connector SIVBToCSMPowerConnector;

	PowerDrainConnectorObject CSMToLEMPowerDrain;
	PowerDrainConnector CSMToLEMPowerConnector;

	//
	// PanelSDK pointers.
	//

	double *pCO2Level;
	double *pCabinCO2Level;
	double *pCabinPressure;
	double *pCabinTemp;
	double *pSuitTemp;
	double *pSuitPressure;
	double *pSuitReturnPressure;
	double *pCabinRegulatorFlow;
	double *pO2DemandFlow;
	double *pDirectO2Flow;
	double *pSuitTestFlow;
	double *pCabinRepressFlow;
	double *pEmergencyCabinRegulatorFlow;
	double *pO2Tank1Press;
	double *pO2Tank2Press;
	double *pH2Tank1Press;
	double *pH2Tank2Press;
	double *pO2SurgeTankPress;
	double *pO2Tank1Quantity;
	double *pO2Tank2Quantity;
	double *pH2Tank1Quantity;
	double *pH2Tank2Quantity;
	double *pFCH2Flow[4];
	double *pFCO2Flow[4];
	double *pFCTemp[4];
	double *pFCCondenserTemp[4];
	double *pFCCoolingTemp[4];
	double *pPrimECSRadiatorInletPressure;
	double *pPrimECSRadiatorInletTemp;
	double *pPrimECSRadiatorOutletTemp;
	double *pPrimECSEvaporatorOutletTemp;
	double *pPrimECSEvaporatorSteamPressure;
	double *pPrimECSAccumulatorQuantity;
	double *pSecECSRadiatorInletPressure;
	double *pSecECSRadiatorInletTemp;
	double *pSecECSRadiatorOutletTemp;
	double *pSecECSEvaporatorOutletTemp;
	double *pSecECSEvaporatorSteamPressure;
	double *pSecECSAccumulatorQuantity;
	double *pPotableH2oTankQuantity;
	double *pWasteH2oTankQuantity;

	// InitSaturn is called twice, but some things must run only once
	bool InitSaturnCalled;

	int LMPadLoadCount;
	int LMPadValueCount;

	//
	// IMFD5 communication support
	//

	IMFD_Client IMFD_Client; 

	//
	// Friend Class List for systems objects 
	//

	friend class GDC;
	friend class BMAG;
	friend class ASCP;
	friend class EDA;
	friend class RJEC;
	friend class ECA;
	friend class CSMcomputer; // I want this to be able to see the GDC	
	friend class PCM;         // Otherwise reading telemetry is a pain
	friend class PMP;
	friend class USB;
	friend class EMS;
	friend class SPSPropellantSource;
	friend class SPSEngine;
	friend class SPSGimbalActuator;
	friend class CMOptics;
	friend class CSMCautionWarningSystem;
	friend class CMACInverterSwitch;
	friend class SaturnSCControlSetter;
	friend class SaturnEMSDvDisplay;
	friend class SECS;
	friend class ELS;
	friend class CrewStatus;
	friend class OpticsHandcontrollerSwitch;
	friend class MinImpulseHandcontrollerSwitch;
	friend class CMRCSPropellantSource;
	friend class DockingProbe;

	double o2fcinletFlowBuffer;
};

extern void BaseInit();
extern void SaturnInitMeshes();
extern void StageTransform(VESSEL *vessel, VESSELSTATUS *vs, VECTOR3 ofs, VECTOR3 vel);

const double STG2O = 8;
const double SMVO = 0.0;
const double CREWO = 0.0;

extern MESHHANDLE hSM;
extern MESHHANDLE hCM;
extern MESHHANDLE hCMInt;
extern MESHHANDLE hCMVC;
extern MESHHANDLE hFHC;
extern MESHHANDLE hFHO;
extern MESHHANDLE hCMP;
extern MESHHANDLE hCREW;
extern MESHHANDLE hSMhga;
extern MESHHANDLE hprobe;
extern MESHHANDLE hprobeext;
extern MESHHANDLE hsat5tower;
extern MESHHANDLE hFHO2;

extern void SetupgParam(HINSTANCE hModule);
extern void DeletegParam();

#endif // _PA_SATURN_H