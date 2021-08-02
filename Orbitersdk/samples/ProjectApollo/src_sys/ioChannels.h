/***************************************************************************
  This file is part of Project Apollo - NASSP
  Copyright 2004-2005

  I/O channel definitions for AGC.

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
#include <bitset>
///
/// AGC output channel ten, used to control the DSKY displays. Output values sent to
/// this channel can have numerous different results based on differnt combinations
/// of the a, b, c and d values.
///
/// \ingroup AGCIO
/// \brief AGC output channel 10.
///

typedef union {
	struct {
		unsigned d:5;						///< Typically second display digit.
		unsigned c:5;						///< Typically first display digit.
		unsigned b:1;						///< Typically positive or negative sign.
		unsigned a:4;						///< Indicates the kind of request.
	} Bits;
	unsigned int Value;
} ChannelValue10;


typedef std::bitset<16> ChannelValue;

///
/// AGC output channel ten, used to control the DSKY lights.
///
/// \ingroup AGCIO
/// \brief AGC output channel 11.
///


//IN0
enum ChannelValue4_Bits
{
	BlockUplink = 5,
	InhibitUpsync,
	GNAttitudeControlMode = 10,
	GNDVMode,
	GNEntryMode,
	OpticsMark = 14
};

//IN2
enum ChannelValue6_Bits
{
	LiftOff = 4,
	GuidanceReferenceRelease,
	UllageThrust,
	SIVBSeperateAbort,
	CMSMSeperate,
	IMUCDUFail,
	PIPAFail,
	IMUFail,
	SCSDVMode,
	GNMonitorMode,
	ParityFail
};

//IN3
enum ChannelValue7_Bits
{
	ZeroEncoderMode = 0,
	CoarseAlign,
	ManualCDU,
	FineAlign,
	AttitudeControl,
	TransferSwitch,
	EntryMode,
	TrackerOn = 9,
	StarPresence,
	ZeroOptics = 11,
	CMCControl = 13,
	ORofC1C33
};

//OUT1
enum ChannelValue11_Bits
{
	ProgramAlarm = 0,
	ComputerActivity,
	KeyRelease,
	TelemetryAlarm,
	CheckFail,
	RUPT2TrapReset = 6,
	IDWord = 8,
	BlockEndPulse,
	EngineOn = 12
};

//OUT2
enum ChannelValue12_Bits
{
	RadarC = 0,
	RadarB,
	RadarA,
	Thrust,
	OptY,
	OptX,
	OptPlus,
	OptMinus,
	IMUZ,
	IMUY,
	IMUX,
	IMUGyro,
	IMUCDU,
	IMUPlus,
	IMUMinus
};

enum ChannelValue163_Bits {

	Ch163DSKYWarn = 0,				///< Turn on the CMC/LGC light.
	Ch163LightTemp = 3,				///< Turn on the Temperature light.
	Ch163LightKbRel,				///< Turn on the Keyboard Release light.
	Ch163FlashVerbNoun,				///< Flash the Verb and Noun displays.			
	Ch163LightOprErr,				///< Light the Operator Error light
	Ch163LightRestart,				///< Light the Restart light
	Ch163LightStandby,				///< Light the Standby light
	Ch163ELOff,						///< Switch off EL panel power

};

///
/// \ingroup AGCIO
/// \brief AGC input channel 177.
///
typedef union {
	struct {
		unsigned GyroPulses:11;
		unsigned GyroEnable:1;
		unsigned GyroSelectB:1;
		unsigned GyroSelectA:1;
		unsigned GyroSign:1;
	} Bits;
	unsigned int Value;
} ChannelValue177;

//
// For now we'll also put AGC addresses in here.
//

///
/// \brief Get AGC bank number from AGC flat address.
///
#define AGC_BANK(n) ((n) / 256)

///
/// \brief Get offset into bank from AGC flat address.
///
#define AGC_ADDR(n) ((n) & 0xff)
