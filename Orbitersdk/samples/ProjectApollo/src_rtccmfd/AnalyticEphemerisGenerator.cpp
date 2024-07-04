/****************************************************************************
This file is part of Project Apollo - NASSP

Analytic Ephemeris Generator

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

#include "AnalyticEphemerisGenerator.h"
#include "rtcc.h"

AEGDataBlock::AEGDataBlock()
{
	InputOutputInd = 11;
	ENTRY = 0;
	TIMA = 0;
	HarmonicsInd = 1;
	ICSUBD = 0.0;
	VehArea = 0.0;
	Item7 = 0.0;
	Item8 = 0.0;
	Item9 = 0.0;
	Item10 = 0.0;

	for (int i = 0; i < 6; i++)
	{
		coe_osc.data[i] = coe_mean.data[i] = 0.0;
	}

	TS = 0.0;
	l_dot = 0.0;
	g_dot = 0.0;
	h_dot = 0.0;
	TE = 0.0;
	f = 0.0;
	U = 0.0;
	R = 0.0;
}

CELEMENTS GeneralAEG::KeplerToEquinoctial(CELEMENTS kep) const
{
	CELEMENTS aeq;

	aeq.a = kep.a;
	aeq.e = kep.e*sin(kep.g + kep.h);
	aeq.i = kep.e*cos(kep.g + kep.h);
	aeq.h = sin(kep.i / 2.0)*sin(kep.h);
	aeq.g = sin(kep.i / 2.0)*cos(kep.h);
	aeq.l = kep.h + kep.g + kep.l;
	if (aeq.l >= PI2)
	{
		aeq.l -= PI2;
	}

	return aeq;
}

CELEMENTS GeneralAEG::EquinoctialToKepler(CELEMENTS aeq) const
{
	CELEMENTS kep;

	kep.a = aeq.a;
	kep.e = sqrt(aeq.e*aeq.e + aeq.i*aeq.i);
	if ((aeq.h*aeq.h + aeq.g*aeq.g) <= 1.0)
		kep.i = acos2(1.0 - 2.0*(aeq.h*aeq.h + aeq.g*aeq.g));
	if ((aeq.h*aeq.h + aeq.g*aeq.g) > 1.0)
		kep.i = acos2(1.0 - 2.0*1.0);

	kep.h = atan2(aeq.h, aeq.g);
	while (kep.h < 0)
	{
		kep.h += PI2;
	}
	kep.g = atan2(aeq.e, aeq.i) - kep.h;
	while (kep.g < 0)
	{
		kep.g += PI2;
	}
	kep.l = aeq.l - atan2(aeq.e, aeq.i);
	while (kep.l >= PI2)
	{
		kep.l -= PI2;
	}
	while (kep.l < 0)
	{
		kep.l += PI2;
	}

	return kep;
}

CELEMENTS GeneralAEG::LyddaneOsculatingToMean(CELEMENTS arr_osc) const
{
	CELEMENTS arr_mean, arr_osc2, arr_mean2;
	CELEMENTS aeq, aeq2, aeq_mean, aeq_mean2;
	int j;
	bool stop, pseudostate = false;

	if (arr_osc.i > 175.0*RAD)
	{
		arr_osc.i = PI - arr_osc.i;
		arr_osc.h = -arr_osc.h + PI2;
		pseudostate = true;
	}

	aeq = KeplerToEquinoctial(arr_osc);
	aeq_mean = aeq;
	stop = true;
	j = 0;
	do
	{
		arr_mean = EquinoctialToKepler(aeq_mean);
		arr_osc2 = LyddaneMeanToOsculating(arr_mean);
		aeq2 = KeplerToEquinoctial(arr_osc2);
		aeq_mean2 = aeq_mean - (aeq2 - aeq);
		arr_mean2 = EquinoctialToKepler(aeq_mean2);

		//Map to 0 to 2PI
		while (arr_mean2.l < 0)
		{
			arr_mean2.l += PI2;
		}
		while (arr_mean2.l >= PI2)
		{
			arr_mean2.l -= PI2;
		}
		while (arr_mean2.g < 0)
		{
			arr_mean2.g += PI2;
		}
		while (arr_mean2.g >= PI2)
		{
			arr_mean2.g -= PI2;
		}
		if (arr_mean2.e < 1e-6)
		{
			arr_mean2.e = 1e-6;
		}

		if (abs(arr_mean2.a - arr_mean.a) > 0.1)
		{
			stop = 0;
		}
		else if (abs(arr_mean2.e - arr_mean.e) > 0.0001)
		{
			stop = 0;
		}
		else if (abs(arr_mean2.i - arr_mean.i) > 0.0001)
		{
			stop = 0;
		}
		else if (abs(arr_mean2.l - arr_mean.l) > 0.0001)
		{
			stop = 0;
		}
		else if (abs(arr_mean2.g - arr_mean.g) > 0.0001)
		{
			stop = 0;
		}
		else if (abs(arr_mean2.h - arr_mean.h) > 0.0001)
		{
			stop = 0;
		}
		aeq_mean = aeq_mean2;
		if (stop == 1)
		{
			break;
		}
		stop = 1;
		j = j + 1;
	} while (j < 25);

	if (pseudostate != 0)
	{
		arr_mean2.i = PI - arr_mean2.i;
		arr_mean2.h = -arr_mean2.h + PI2;
	}

	return arr_mean2;
}

void GeneralAEG::BrouwerSecularRates(CELEMENTS coe_osc, CELEMENTS coe_mean, int body, double &l_dot, double &g_dot, double &h_dot) const
{
	double mu, n0, eccdp2, cn, cn2, theta, theta2, theta3, theta4, k2, k4, gm2, gm4, gmp2, gmp4, J2, J3, J4, R_e;
	double esing, ecosg, L, u, f, sin_lat, R, ainv;

	if (body == BODY_EARTH)
	{
		mu = OrbMech::mu_Earth;
		J2 = OrbMech::J2_Earth;
		J3 = OrbMech::J3_Earth;
		J4 = OrbMech::J4_Earth;
		R_e = OrbMech::R_Earth;
	}
	else
	{
		mu = OrbMech::mu_Moon;
		J2 = OrbMech::J2_Moon;
		J3 = OrbMech::J3_Moon;
		J4 = 0;
		R_e = OrbMech::R_Moon;
	}

	n0 = sqrt(mu / pow(coe_mean.a, 3));
	eccdp2 = coe_mean.e * coe_mean.e;
	cn2 = 1.0 - eccdp2;
	cn = sqrt(cn2);
	theta = cos(coe_mean.i);
	theta2 = theta * theta;
	theta3 = theta2 * theta;
	theta4 = theta2 * theta2;
	k2 = J2 * pow(R_e, 2) / 2.0;
	k4 = -3.0 * J4*pow(R_e, 4) / 8.0;
	gm2 = k2 / pow(coe_mean.a, 2);
	gm4 = k4 / pow(coe_mean.a, 4);
	gmp2 = gm2 / pow(cn2, 2);
	gmp4 = gm4 / pow(cn2, 4);

	esing = coe_osc.e*sin(coe_osc.g);
	ecosg = coe_osc.e*cos(coe_osc.g);
	L = coe_osc.l + coe_osc.g;
	u = L + (2.0*ecosg*sin(L) - 2.0*esing*cos(L))*(1.0 + 5.0 / 4.0*ecosg*cos(L) + esing * sin(L));
	f = u - coe_osc.g;
	sin_lat = sin(u)*sin(coe_osc.i);
	R = coe_osc.a*(1.0 - coe_osc.e*coe_osc.e) / (1.0 + coe_osc.e*cos(f));
	ainv = 1.0 / coe_osc.a + J2 * pow(R_e, 2) / pow(R, 3)*(1.0 - 3.0*pow(sin_lat, 2)) + J3 * pow(R_e, 3) / pow(R, 4)*(3.0*sin_lat - 5.0*pow(sin_lat, 3)) -
		J4 * pow(R_e, 4) / (4.0*pow(R, 5))*(3.0 - 30.0*pow(sin_lat, 2) + 35.0*pow(sin_lat, 4));
	l_dot = sqrt(mu*pow(ainv, 3));

	//l_dot = n0 + n0 * cn*(gmp2*(3.0 / 2.0*(3.0*theta2 - 1.0) + 3.0 / 32.0*gmp2*(25.0*cn2 + 16.0*cn - 15.0 + (30.0 - 96.0*cn - 90.0*cn2)*theta2
	//	+ (105.0 + 144.0*cn + 25.0*cn2)*theta4)) + 15.0 / 16.0*gmp4*eccdp2*(3.0 - 30.0*theta2 + 35.0*theta4));
	g_dot = n0 * (gmp2*(3.0 / 2.0*(5.0*theta2 - 1.0) + 3.0 / 32.0*gmp2*(25.0*cn2 + 24.0*cn - 35.0
		+ (90.0 - 192.0*cn - 126.0*cn2)*theta2 + (385.0 + 360.0*cn + 45.0*cn2)*theta4))
		+ 5.0 / 16.0*gmp4*(21.0 - 9.0*cn2 + (126.0*cn2 - 270.0)*theta2 + (385.0 - 189.0*cn2)*theta4));
	h_dot = n0 * (gmp2*(3.0 / 8.0*gmp2*((9.0*cn2 + 12.0*cn - 5.0)*theta - (35.0 + 36.0*cn + 5.0*cn2)*theta3) - 3.0*theta)
		+ 5.0 / 4.0*gmp4*theta*(5.0 - 3.0*cn2)*(3.0 - 7.0*theta2));
}

PMMAEG::PMMAEG(RTCC *r) : RTCCModule(r)
{

}

void PMMAEG::CALL(AEGHeader &header, AEGDataBlock &in, AEGDataBlock &out)
{
	AEGDataBlock tempblock;

	header.ErrorInd = 0;

	if (abs(in.TE - in.TS) > 96.0*3600.0)
	{
		goto NewPMMAEG_V846;
	}
	if (in.coe_osc.a<0.4*OrbMech::R_Earth || in.coe_osc.a>9.0*OrbMech::R_Earth)
	{
		goto NewPMMAEG_V846;
	}
	if (in.coe_osc.e<0.0 || in.coe_osc.e>0.85)
	{
		goto NewPMMAEG_V846;
	}
	if (in.coe_osc.i<0.18 || in.coe_osc.i>1.05)
	{
		goto NewPMMAEG_V846;
	}
	if (in.coe_osc.l<0.0 || in.coe_osc.l>PI2)
	{
		goto NewPMMAEG_V846;
	}
	if (in.coe_osc.g<0.0 || in.coe_osc.g>PI2)
	{
		goto NewPMMAEG_V846;
	}
	if (in.coe_osc.h<0.0 || in.coe_osc.h>PI2)
	{
		goto NewPMMAEG_V846;
	}

	if (in.TE == in.TS && in.ENTRY != 0 && in.TIMA == 0)
	{
		//Input time equals output time, we have initialized elements and time option. Nothing to do
		CurrentBlock = in;
		goto NewPMMAEG_V1030;
	}

	if (in.TIMA >= 4)
	{
		//Save a, e, i, u, t, h, r, t_f from previous block for phase lag routine
		tempblock = CurrentBlock;
	}

	//Uninitialized
	if (in.ENTRY == 0)
	{
		in.coe_mean = LyddaneOsculatingToMean(in.coe_osc);

		BrouwerSecularRates(in.coe_osc, in.coe_mean, BODY_EARTH, in.l_dot, in.g_dot, in.h_dot);

		in.f = OrbMech::MeanToTrueAnomaly(in.coe_osc.l, in.coe_osc.e);
		in.U = in.f + in.coe_osc.g;
		if (in.U >= PI2)
		{
			in.U -= PI2;
		}
		in.R = in.coe_osc.a*(1.0 - in.coe_osc.e*in.coe_osc.e) / (1.0 + in.coe_osc.e*cos(in.coe_osc.g)*cos(in.U) + in.coe_osc.e*sin(in.coe_osc.g)*sin(in.U));
		in.ENTRY = 1;
	}

	//Initial values for final state
	CurrentBlock = in;

	double dt, theta_R;
	bool firstpass = true;

	if (in.TIMA == 0 || in.TIMA >= 4)
	{
		if (in.TIMA == 0)
		{
			dt = in.TE - in.TS;
			if (dt == 0.0)
			{
				goto NewPMMAEG_V2000;
			}
		}
		else
		{
			dt = tempblock.TE - in.TS;
		}
	NewPMMAEG_V1000:
		if (MeanElementsUpdateRoutine(in, dt))
		{
			//Drag altitude error
			header.ErrorInd = -2;
			return;
		}

		CurrentBlock.TE = CurrentBlock.TS = in.TS + dt;
		CurrentBlock.coe_osc = LyddaneMeanToOsculating(CurrentBlock.coe_mean);
	}
	else
	{
		CurrentBlock.coe_osc = in.coe_osc;

		double L_D, DX_L, DH, X_L, X_L_dot, ddt;
		int LINE, COUNT;

		if (in.TIMA != 3)
		{
			L_D = in.Item8;
		}
		else
		{
			L_D = in.U;
		}
		DX_L = 1.0;
		DH = true;
		dt = 0.0;
		LINE = 0;
		COUNT = 24;

		do
		{
			//Mean anomaly
			if (in.TIMA == 1)
			{
				X_L = CurrentBlock.coe_osc.l;
				X_L_dot = CurrentBlock.l_dot;
			}
			//Argument of latitude
			else if (in.TIMA == 2)
			{
				double u = OrbMech::MeanToTrueAnomaly(CurrentBlock.coe_osc.l, CurrentBlock.coe_osc.e) + CurrentBlock.coe_osc.g;
				u = fmod(u, PI2);
				if (u < 0)
					u += PI2;

				X_L = u;
				X_L_dot = CurrentBlock.l_dot + CurrentBlock.g_dot;
			}
			//Maneuver line
			else
			{
				double u = OrbMech::MeanToTrueAnomaly(CurrentBlock.coe_osc.l, CurrentBlock.coe_osc.e) + CurrentBlock.coe_osc.g;
				u = fmod(u, PI2);
				if (u < 0)
					u += PI2;

				X_L = u;
				X_L_dot = CurrentBlock.l_dot + CurrentBlock.g_dot;
				LINE = 2;
			}

			if (DH)
			{
				double DN_apo = in.Item10 * PI2;
				ddt = DN_apo / CurrentBlock.l_dot;
				DH = false;

				if (LINE != 0)
				{
					L_D = L_D + CurrentBlock.g_dot * ddt + DN_apo;
					while (L_D < 0) L_D += PI2;
					while (L_D >= PI2) L_D -= PI2;
				}
				else
				{
					ddt += (L_D - X_L) / X_L_dot;
				}
			}
			else
			{
				DX_L = L_D - X_L;
				if (abs(DX_L) - PI >= 0)
				{
					if (DX_L > 0)
					{
						DX_L -= PI2;
					}
					else
					{
						DX_L += PI2;
					}
				}
				ddt = DX_L / X_L_dot;
				if (LINE != 0)
				{
					L_D = L_D + ddt * CurrentBlock.g_dot;
				}
			}

			dt += ddt;

			if (MeanElementsUpdateRoutine(in, dt))
			{
				//Drag altitude error
				header.ErrorInd = -2;
				return;
			}

			CurrentBlock.coe_osc = LyddaneMeanToOsculating(CurrentBlock.coe_mean);

			COUNT--;

		} while (abs(DX_L) > 2e-4 && COUNT > 0);

		if (COUNT == 0)
		{
			header.ErrorInd = -3;
		}

		CurrentBlock.TE = CurrentBlock.TS = in.TS + dt;
	}

NewPMMAEG_V2000:
	CurrentBlock.f = OrbMech::MeanToTrueAnomaly(CurrentBlock.coe_osc.l, CurrentBlock.coe_osc.e);
	CurrentBlock.U = CurrentBlock.f + CurrentBlock.coe_osc.g;
	if (CurrentBlock.U >= PI2)
	{
		CurrentBlock.U -= PI2;
	}
	CurrentBlock.R = CurrentBlock.coe_osc.a*(1.0 - CurrentBlock.coe_osc.e*CurrentBlock.coe_osc.e) / (1.0 + CurrentBlock.coe_osc.e*cos(CurrentBlock.coe_osc.g)*cos(CurrentBlock.U) + CurrentBlock.coe_osc.e*sin(CurrentBlock.coe_osc.g)*sin(CurrentBlock.U));

	if (in.TIMA >= 4)
	{
		theta_R = CurrentBlock.U - tempblock.U - 2.0*atan(tan((CurrentBlock.coe_osc.h - tempblock.coe_osc.h) / 2.0)*(sin(0.5*(CurrentBlock.coe_osc.i + tempblock.coe_osc.i - PI)) / sin(0.5*(CurrentBlock.coe_osc.i - tempblock.coe_osc.i + PI))));
		if (theta_R < -PI)
		{
			theta_R += PI2;
		}
		else if (theta_R >= PI)
		{
			theta_R -= PI2;
		}
		if (in.TIMA == 4)
		{
			CurrentBlock.Item10 = theta_R;
		}
		else
		{
			if (firstpass)
			{
				CurrentBlock.Item10 = theta_R;
				firstpass = false;
			}
		}
	}

	if (in.TIMA >= 5)
	{
		CurrentBlock.Item8 = CurrentBlock.R - tempblock.R;
		CurrentBlock.Item9 = CurrentBlock.TE - tempblock.TE;
		dt += -theta_R / (CurrentBlock.l_dot + CurrentBlock.g_dot);
		if (abs(theta_R) > 0.00005)
		{
			goto NewPMMAEG_V1000;
		}
	}

NewPMMAEG_V1030:
	//Move output into area supplied by the calling program (already done)
	out = CurrentBlock;
NewPMMAEG_V305:
	return;
NewPMMAEG_V846:
	header.ErrorInd = -1;
	goto NewPMMAEG_V305;
}

CELEMENTS PMMAEG::LyddaneMeanToOsculating(CELEMENTS arr) const
{
	CELEMENTS out;
	double ae, am, Em, fm, J2, J3, J4, R_e, cn, cn2, theta, theta2, theta4, eccdp2, sinI, sinI2, cosI2, adr, adr2, adr3, a;
	double sinta, costa, costa2, sn2gta, cs2gta, sinGD, cosGD, sin2gd, cs2gd, sn3fgd, snf2gd, csf2gd, cs3fgd;
	double k2, k3, k4, gamma2, gamma3, gamma4, gamma2_apo, gamma3_apo, gamma4_apo, g3dg2, g4dg2;
	double A1_apo, A1, A2_apo, A2, A6, A7, A10, A11, A12, A13, A14, A15, A16, A17, A18, A20, A21, A25, A26;
	double B1, B2, B4, B5, B7, B8, B10, B11, B13, B14;
	double delta1e, de, edl, di, sin_im2_dh, lagaha, lgh, sinMADP, cosMADP, sinraandp, cosraandp;
	bool pseudostate = false;

	J2 = OrbMech::J2_Earth;
	J3 = OrbMech::J3_Earth;
	J4 = OrbMech::J4_Earth;
	R_e = OrbMech::R_Earth;

	if (arr.i > 175.0*RAD)
	{
		arr.i = PI - arr.i;
		arr.h = -arr.h;
		pseudostate = true;
	}

	ae = 1.0;
	am = arr.a / R_e;
	eccdp2 = arr.e*arr.e;
	cn2 = 1.0 - eccdp2;
	cn = sqrt(cn2);
	theta = cos(arr.i);
	theta2 = theta * theta;
	theta4 = theta2 * theta2;
	k2 = J2 * pow(ae, 2) / 2.0;
	k3 = -J3 * pow(ae, 3);
	k4 = -3.0 * J4*pow(ae, 4) / 8.0;
	gamma2 = k2 / pow(am, 2);
	gamma3 = k3 / pow(am, 3);
	gamma4 = k4 / pow(am, 4);
	gamma2_apo = gamma2 / pow(cn, 4);
	gamma3_apo = gamma3 / pow(cn, 6);
	gamma4_apo = gamma4 / pow(cn, 8);

	g3dg2 = gamma3_apo / gamma2_apo;
	g4dg2 = gamma4_apo / gamma2_apo;

	Em = OrbMech::kepler_E(arr.e, arr.l, 1e-12);
	fm = atan2(sqrt(1.0 - eccdp2)*sin(Em), cos(Em) - arr.e);
	if (fm < 0)
	{
		fm = fm + PI2;
	}
	adr = 1.0 / (1.0 - arr.e*cos(Em));
	adr2 = adr * adr;
	adr3 = adr2 * adr;

	sinI = sin(arr.i);
	sinI2 = sin(arr.i / 2.0);
	cosI2 = cos(arr.i / 2.0);
	sinta = sin(fm);
	costa = cos(fm);
	costa2 = costa * costa;
	sn2gta = sin(2.0*arr.g + 2.0*fm);
	cs2gta = cos(2.0*arr.g + 2.0*fm);
	snf2gd = sin(2.0 * arr.g + fm);
	csf2gd = cos(2.0 * arr.g + fm);
	sinGD = sin(arr.g);
	cosGD = cos(arr.g);
	sin2gd = sin(2.0 * arr.g);
	cs2gd = cos(2.0 * arr.g);
	sn3fgd = sin(3.0*fm + 2.0*arr.g);
	cs3fgd = cos(3.0*fm + 2.0*arr.g);
	sinMADP = sin(arr.l);
	cosMADP = cos(arr.l);
	sinraandp = sin(arr.h);
	cosraandp = cos(arr.h);

	A1_apo = 1.0 / (1.0 - 5.0 * theta2);
	A1 = 1.0 / 8.0 * gamma2_apo*cn2 * (1.0 - 11.0 * theta2 - 40.0 * theta4 * A1_apo);
	A2_apo = 3.0 * theta2 + 8.0 * theta4 * A1_apo;
	A2 = 5.0 / 12.0 * g4dg2 * cn2 * (1.0 - A2_apo);
	A6 = 1.0 / 4.0 * g3dg2;
	A7 = A6 * cn2 * sinI;
	A10 = 2.0 + eccdp2;
	A11 = 3.0 * eccdp2 + 2.0;
	A12 = A11 * theta2;
	A13 = (5.0 * eccdp2 + 2.0)*theta4 * A1_apo;
	A14 = eccdp2 * theta4*theta2 * pow(A1_apo, 2);
	A15 = theta2 * A1_apo;
	A16 = pow(A15, 2);
	A17 = arr.e * sinI;
	A18 = A17 / (1.0 + cn);
	A20 = arr.e * theta;
	A21 = arr.e * A20;
	A25 = 16.0 * A15 + 40.0 * A16 + 3.0;
	A26 = 1.0 / 8.0 * A21*(11.0 + 200.0 * A16 + 80.0 * A15);

	B1 = cn * (A1 - A2) - (1.0 / 16.0 * (A10 - 400.0 * A14 - 40.0 * A13 - 11.0 * A12) + 1.0 / 8.0 * A21*(11.0 + 200.0 * A16 + 80.0 * A15))*gamma2_apo
		+ 5.0 / 24.0 * (-80.0 * A14 - 8.0 * A13 - 3.0 * A12 + 2.0 * A25*A21 + A10)*g4dg2;
	B2 = A6 * A18*(2.0 + cn - eccdp2) + A20 * tan(arr.i / 2.0)*(A6);
	B4 = cn * arr.e*(A1 - A2);
	B5 = cn * A7;
	B7 = cn2 * A17*A1_apo*(1.0 / 8.0 * gamma2_apo*(1.0 - 15.0 * theta2) - 5.0 / 12.0 * g4dg2 * (1.0 - 7.0 * theta2));
	B8 = cn2 * A6;
	B10 = sinI * (5.0 / 12.0 * g4dg2 * A21*A25 - A26 * gamma2_apo);
	B11 = A21 * A6;
	B13 = arr.e * (A1 - A2);
	B14 = A7;

	a = am * (1.0 + gamma2 * ((3.0 * theta2 - 1.0)*arr.e / (cn2*cn2*cn2)*(arr.e*cn + arr.e / (1.0 + cn) + costa * (3.0 + 3.0 * arr.e*costa
		+ eccdp2 * costa2)) + 3.0 * (1.0 - theta2)*adr3 * cs2gta));

	delta1e = B13 * cs2gd + B14 * sinGD;
	de = delta1e - cn2 / 2.0 * (gamma2_apo*(1.0 - theta2)*(3.0 * csf2gd + cs3fgd)
		- 3.0 * gamma2 * 1.0 / (cn2*cn2*cn2)*(1.0 - theta2)*cs2gta*(3.0 * arr.e*costa2
			+ 3.0 * costa + eccdp2 * costa*costa2 + arr.e)
		- gamma2 * 1.0 / (cn2*cn2*cn2)*(3.0 * theta2 - 1.0)*(arr.e*cn + arr.e / (1.0 + cn) + 3.0 * arr.e*costa2 + 3.0 * costa + eccdp2 * costa * costa2));
	edl = B4 * sin2gd - B5 * cosGD
		- 1.0 / 4.0 * cn*cn2 * gamma2_apo*(2.0 * (3.0 * theta2 - 1.0)*(cn2 * adr2 + adr + 1.0)*sinta
			+ 3.0 * (1.0 - theta2)*((-cn2 * adr2 - adr + 1.0)*snf2gd
				+ (cn2 * adr2 + adr + 1.0 / 3.0)*sn3fgd));
	out.e = sqrt(pow(arr.e + de, 2) + pow(edl, 2));

	di = 1.0 / 2.0 * theta*gamma2_apo*sinI*(arr.e*cs3fgd + 3.0 * (arr.e*csf2gd + cs2gta))
		- A20 / cn2 * (B7*cs2gd + B8 * sinGD);
	sin_im2_dh = 1.0 / (2.0 * cosI2)*(B10*sin2gd + B11 * cosGD
		- 1.0 / 2.0 * gamma2_apo*theta*sinI*(6.0 * (arr.e*sinta - arr.l + fm)
			- 3.0 * (sn2gta + arr.e * snf2gd) - arr.e * sn3fgd));
	out.i = 2.0 * asin(sqrt(pow(sin_im2_dh, 2) + pow(1.0 / 2.0 * di*cosI2 + sinI2, 2)));

	if (out.e <= 1.0e-11)
	{
		out.l = 0;
	}
	else
	{
		out.l = atan2(edl*cosMADP + (arr.e + de)*sinMADP, (arr.e + de)*cosMADP - edl * sinMADP);
		if (out.l < 0)
		{
			out.l = out.l + PI2;
		}
	}

	if (out.i == 0.0)
	{
		out.h = 0;
	}
	else
	{
		out.h = atan2(sin_im2_dh*cosraandp + sinraandp * (1.0 / 2.0 * di*cosI2 + sinI2), cosraandp*(1.0 / 2.0 * di*cosI2 + sinI2) - sin_im2_dh * sinraandp);
		if (out.h < 0)
		{
			out.h = out.h + PI2;
		}
	}

	lagaha = arr.l + arr.g + arr.h + B1 * sin2gd + B2 * cosGD;
	lgh = lagaha + (1.0 / 4.0 * (cn2 / (cn + 1.0))*arr.e*gamma2_apo*(3.0 * (1.0 - theta2)*(sn3fgd
		*(1.0 / 3.0 + adr2 * cn2 + adr) + snf2gd * (1.0 - adr2 * cn2 - adr))
		+ 2.0 * sinta*(3.0 * theta2 - 1.0)*(1.0 + adr2 * cn2 + adr)))
		+ 3.0 / 2.0 * gamma2_apo*((5.0 * theta2 - 2.0 * theta - 1.0)*(arr.e*sinta + fm - arr.l)) + (3.0 + 2.0 * theta - 5.0 * theta2)
		*(1.0 / 4.0 * gamma2_apo*(arr.e*sn3fgd + 3.0 * (sn2gta + arr.e * snf2gd)));

	out.g = lgh - out.l - out.h;
	while (out.g >= PI2)
	{
		out.g -= PI2;
	}
	while (out.g < 0)
	{
		out.g += PI2;
	}

	if (pseudostate)
	{
		out.i = PI - out.i;
		out.h = -out.h;

		if (out.h < 0)
		{
			out.h = out.h + PI2;
		}
	}

	out.a = a * R_e;
	return out;

	//Lyddane-Cohen improvement on the SMA
	/*double a1 = a;
	double f = OrbMech::MeanToTrueAnomaly(out.l, out.e);
	double ar = (1.0 + out.e*cos(f)) / (1.0 - out.e*out.e);
	double psi = (-1.0 + 3.0*pow(cos(out.i), 2))*(pow(ar, 3) - pow(sqrt(1.0 - out.e*out.e), 3)) + 3.0*(1.0 - pow(cos(out.i), 2))*pow(ar, 3)*cos(2.0*out.g + 2.0*f);
	double da = 1.0 / pow(a1, 3)*(k2*psi*(a1*a1 - k2 * psi + 3.0 / 2.0*(k2 / (cn*cn2))*(1.0 - 3.0*theta2))
		- k2 * k2 / (16.0*pow(cn, 7))*(15.0*cn2*(1.0 - 18.0 / 5.0*theta2 + theta4)
			+ 12.0*cn*(1.0 - 6.0*theta2 + 9.0*theta4) - 15.0*(1.0 - 2.0*theta2 - 7.0*theta4)
			+ 6.0*eccdp2*(1.0 - 16.0*theta2 + 15.0*theta4)*cos(2.0*arr.g))
		+ 9.0*k2*k2 / (2.0*pow(cn, 7))*(1.0 - 6.0*theta2 + 5.0*theta4)*(cos(2.0*arr.g + 2.0*fm)
			+ arr.e*cos(2.0*arr.g + fm) + 1.0 / 3.0*arr.e*cos(2.0*arr.g + 3.0*fm)));

	out.a = (am + da)*R_Earth;
	double a1r = a1 * R_Earth;
	return out;*/
}

bool PMMAEG::MeanElementsUpdateRoutine(AEGDataBlock &in, double dt)
{
	//Apply drag corrections, if desired
	if (in.ICSUBD != 0.0 && in.Item7 > 0.0)
	{
		CELEMENTS coe_mean_drag;
		double l_dot_drag;

		if (DRAG(CurrentBlock.coe_mean, in.ICSUBD*in.VehArea / in.Item7, CurrentBlock.l_dot, CurrentBlock.g_dot, dt, coe_mean_drag, l_dot_drag))
		{
			return true;
		}

		//Update elements
		CurrentBlock.coe_mean = coe_mean_drag;
		CurrentBlock.l_dot = l_dot_drag;
	}
	CurrentBlock.coe_mean.l = CurrentBlock.l_dot*dt + in.coe_mean.l;
	CurrentBlock.coe_mean.g = CurrentBlock.g_dot*dt + in.coe_mean.g;
	CurrentBlock.coe_mean.h = CurrentBlock.h_dot*dt + in.coe_mean.h;

	OrbMech::normalizeAngle(CurrentBlock.coe_mean.l);
	OrbMech::normalizeAngle(CurrentBlock.coe_mean.g);
	OrbMech::normalizeAngle(CurrentBlock.coe_mean.h);
	return false;
}

bool PMMAEG::DRAG(CELEMENTS coe_mean0, double KAm, double l_dot, double g_dot, double ddt, CELEMENTS &coe_mean0_apo, double &l_dot_apo) const
{
	//KAm = KFactor*Area/mass

	const double ddt_max = 450.0;
	double THETA, BETA, C_D_apo, f_t, u_t, a_j, e_j, g_t, l_t, l_dot_0, dl_D, da_D, de_D, dg_D, K_D, dl_1D, dl_2D, ddddt, p, dr_SP, R;
	double ej2, B, C, D, E, ER2, E_R, G, H, r_F, alt, V, V_rel, rho, SPOS, da_a_dt, D2, P, e_dot_0, e_gdot_0, l_dot2, da;
	bool stop;

	//Prevent division by zero
	coe_mean0.e = max(1.e-6, coe_mean0.e);

	//Save input as output
	coe_mean0_apo = coe_mean0;
	l_dot_apo = l_dot;

	//Initial settings
	stop = false;
	THETA = cos(coe_mean0.i);
	BETA = sin(coe_mean0.i);
	C_D_apo = -pRTCC->SystemParameters.MCADRG*KAm;

	//Calculate initial true (mean) anomaly
	f_t = OrbMech::MeanToTrueAnomaly(coe_mean0.l, coe_mean0.e);
	//Calculate argument of latitude
	u_t = f_t + coe_mean0.g;
	//Initial elements
	a_j = coe_mean0.a;
	e_j = coe_mean0.e;
	g_t = coe_mean0.g;
	l_t = coe_mean0.l;
	l_dot_0 = l_dot;

	//Set initial corrections to zero
	dl_D = 0.0;
	da_D = 0;
	de_D = 0;
	dg_D = 0;
	K_D = 0;
	dl_1D = 0;
	dl_2D = 0;

	do
	{
		//Determine step size
		if (K_D + ddt_max > ddt)
		{
			ddddt = ddt - K_D;
			stop = true;
		}
		else
		{
			ddddt = ddt_max;
		}

		//Intermediate quantities
		ej2 = e_j * e_j;
		B = cos(f_t);
		C = sin(f_t);
		E = e_j * B;
		D = sqrt(1.0 + ej2 + 2.0 * E);
		ER2 = 1.0 - ej2;
		E_R = sqrt(ER2);
		H = 1.0 + E;
		G = H + ej2;

		//TBD: Don't use mean elements for radius calculation
		p = a_j * (1.0 - ej2);
		dr_SP = -OrbMech::J2_Earth * pow(OrbMech::R_Earth, 2) / (4.0 * p)*((3.0 * THETA*THETA - 1)*(2.0 * E_R / H + E / (1.0 + E_R) + 1.0) - BETA * BETA*cos(2.0 * u_t));
		R = p / (1.0 + E);
		R = R + dr_SP;

		//Low altitude check
		if (R <= OrbMech::R_Earth + 50.0*1852.0)
		{
			return true;
		}

		//TBD: Take ellipsoid into account
		//sin_phi = BETA * sin(u_t);
		//r_F = sqrt(pow(b_E, 2) / (1.0 - (1.0 - pow(b_E, 2) / a_E / a_E)*(1.0 - pow(sin_phi, 2))));
		r_F = OrbMech::R_Earth;
		alt = R - r_F;

		//Inertial velocity(TBD: don't use mean a)
		V = sqrt(OrbMech::mu_Earth*(2.0 / R - 1.0 / a_j));
		//Assumption of small inclination, real RTCC does it as well
		V_rel = V - OrbMech::w_Earth * R;

		//Get densitiy
		pRTCC->GLFDEN(alt, rho, SPOS);

		//Semi major axis decay rate
		da_a_dt = rho * C_D_apo*pow(V_rel, 2) * D / (l_dot*a_j*E_R);
		//More intermediate variables
		D2 = D * D;
		P = da_a_dt * ER2 / D2;
		e_dot_0 = P * (B + e_j);
		e_gdot_0 = P * C;
		//Change in mean anomaly due to apsidal precession(?)
		l_dot2 = e_gdot_0 * D2 / (H*(H + E_R * G))*(1.0 + ej2 * (1.0 + B * B + e_j * (2.0 * B + e_j)));
		//Change in mean semi major axis over the timestep
		da = da_a_dt * ddddt*a_j;
		//Adjust semi major axis
		a_j = a_j + da;
		//Adjust eccentricity
		e_j = e_j + e_dot_0 * ddddt;
		//Change in argument of latitude due to apsidal precession(?)
		u_t = u_t + l_dot2 * ddddt;
		//Change in argument of perigee due to apsidal precession
		g_t = g_t + e_gdot_0 * ddddt / coe_mean0.e;
		//Change in mean motion due to drag
		dl_D = dl_D - 3.0 / 4.0 * l_dot*da / a_j;
		//Adjusted mean motion
		l_dot = l_dot_0 + dl_D;
		//Keep track of total change in semi major axis
		da_D = da_D + da_a_dt * ddddt*coe_mean0.a; //TBD: Why a_mean and not a_j?
		//Keep track of total change in eccentricity
		de_D = de_D + e_dot_0 * ddddt;
		//Keep track of change in argument of perigee
		dg_D = dg_D + e_gdot_0 * ddddt / coe_mean0.e;
		//Keep track of changes in mean anomaly
		dl_1D = dl_1D - 3.0 / 2.0 * l_dot*ddddt*da_a_dt*(ddt - K_D - 0.5*ddddt);
		dl_2D = dl_2D + ddddt * (l_dot2 - e_gdot_0 / coe_mean0.e);

		//Apply elements if we don't stop
		if (stop == false)
		{
			u_t = u_t + (l_dot + g_dot)*ddddt + 2.0 * e_j*sin(l_dot2*ddddt);
			f_t = u_t - g_t;
			g_t = g_t + g_dot * ddddt;
			K_D = K_D + ddddt;
		}

	} while (stop == false);

	//Output elements
	coe_mean0_apo.a = coe_mean0.a + da_D;
	coe_mean0_apo.e = coe_mean0.e + de_D;
	coe_mean0_apo.g = coe_mean0.g + dg_D;
	OrbMech::normalizeAngle(coe_mean0_apo.g, true);
	coe_mean0_apo.l = coe_mean0.l + dl_1D + dl_2D;
	OrbMech::normalizeAngle(coe_mean0_apo.l);

	return false;
}

PMMLAEG::PMMLAEG(RTCC *r) : RTCCModule(r)
{

}

void PMMLAEG::CALL(AEGHeader &header, AEGDataBlock &in, AEGDataBlock &out)
{
	AEGDataBlock tempblock;
	CELEMENTS coe_osc0, coe_osc1, coe_mean1;
	MATRIX3 Rot;
	VECTOR3 P, W;

	header.ErrorInd = 0;

	if (in.coe_osc.a < 0.2*OrbMech::R_Earth || in.coe_osc.a > 5.0*OrbMech::R_Earth)
	{
		goto NewPMMLAEG_V846;
	}
	if (in.coe_osc.e < 0.0 || in.coe_osc.e > 0.3)
	{
		goto NewPMMLAEG_V846;
	}
	if (in.coe_osc.i < 0 || in.coe_osc.i > PI)
	{
		goto NewPMMLAEG_V846;
	}
	if (in.coe_osc.l < 0.0 || in.coe_osc.l >= PI2)
	{
		goto NewPMMLAEG_V846;
	}
	if (in.coe_osc.g < 0.0 || in.coe_osc.g >= PI2)
	{
		goto NewPMMLAEG_V846;
	}
	if (in.coe_osc.h < 0.0 || in.coe_osc.h >= PI2)
	{
		goto NewPMMLAEG_V846;
	}

	if (in.TE == in.TS && in.ENTRY != 0 && in.TIMA == 0)
	{
		CurrentBlock = in;
		//Input time equals output time, we have initialized elements and time option. Nothing to do
		goto NewPMMLAEG_V1030;
	}

	if (in.TIMA >= 4)
	{
		//Save a, e, i, u, t, h, r, t_f from previous block for phase lag routine
		tempblock = CurrentBlock;
	}

	CurrentBlock = in;

	//Matrix to rotate to selenographic inertial
	if (pRTCC->PLEFEM(5, in.TS / 3600.0, 0, NULL, NULL, NULL, &Rot))
	{
		goto NewPMMLAEG_V846;
	}

	//Uninitialized
	if (in.ENTRY == 0)
	{
		//Selenocentric to selenographic
		coe_osc0 = in.coe_osc;
		pRTCC->PIVECT(in.coe_osc.i, in.coe_osc.g, in.coe_osc.h, P, W);
		P = mul(Rot, P);
		W = mul(Rot, W);
		pRTCC->PIVECT(P, W, coe_osc0.i, coe_osc0.g, coe_osc0.h);

		//Osculating to mean
		in.coe_mean = LyddaneOsculatingToMean(coe_osc0);

		BrouwerSecularRates(in.coe_osc, in.coe_mean, BODY_MOON, in.l_dot, in.g_dot, in.h_dot);
		CurrentBlock.l_dot = in.l_dot;
		CurrentBlock.g_dot = in.g_dot;
		CurrentBlock.h_dot = in.h_dot;

		in.f = OrbMech::MeanToTrueAnomaly(in.coe_osc.l, in.coe_osc.e);
		in.U = in.f + in.coe_osc.g;
		if (in.U >= PI2)
		{
			in.U -= PI2;
		}
		in.R = in.coe_osc.a*(1.0 - in.coe_osc.e*in.coe_osc.e) / (1.0 + in.coe_osc.e*cos(in.coe_osc.g)*cos(in.U) + in.coe_osc.e*sin(in.coe_osc.g)*sin(in.U));
		in.ENTRY = 1;
	}
	else
	{
		CurrentBlock.l_dot = in.l_dot;
		CurrentBlock.g_dot = in.g_dot;
		CurrentBlock.h_dot = in.h_dot;
	}

	coe_mean1 = in.coe_mean;

	double dt, theta_R;
	bool firstpass = true;

	if (in.TIMA == 0 || in.TIMA >= 4)
	{
		if (in.TIMA == 0)
		{
			dt = in.TE - in.TS;
		}
		else
		{
			dt = tempblock.TE - in.TS;
		}
	NewPMMLAEG_V1000:
		coe_mean1.l = CurrentBlock.l_dot*dt + in.coe_mean.l;
		coe_mean1.g = CurrentBlock.g_dot*dt + in.coe_mean.g;
		coe_mean1.h = CurrentBlock.h_dot*dt + in.coe_mean.h;

		OrbMech::normalizeAngle(coe_mean1.l);
		OrbMech::normalizeAngle(coe_mean1.g);
		OrbMech::normalizeAngle(coe_mean1.h);

		CurrentBlock.TE = CurrentBlock.TS = in.TS + dt;
		coe_osc1 = LyddaneMeanToOsculating(coe_mean1);

		//Selenographic to selenocentric
		pRTCC->PIVECT(coe_osc1.i, coe_osc1.g, coe_osc1.h, P, W);
		P = tmul(Rot, P);
		W = tmul(Rot, W);
		pRTCC->PIVECT(P, W, coe_osc1.i, coe_osc1.g, coe_osc1.h);
	}
	else
	{
		coe_osc1 = in.coe_osc;

		double L_D, DX_L, DH, X_L, X_L_dot, ddt;
		int LINE, COUNT;

		if (in.TIMA != 3)
		{
			L_D = in.Item8;
		}
		else
		{
			L_D = in.U;
		}
		DX_L = 1.0;
		DH = true;
		dt = 0.0;
		LINE = 0;
		COUNT = 24;

		do
		{
			//Mean anomaly
			if (in.TIMA == 1)
			{
				X_L = coe_osc1.l;
				X_L_dot = CurrentBlock.l_dot;
			}
			//Argument of latitude
			else if (in.TIMA == 2)
			{
				double u = OrbMech::MeanToTrueAnomaly(coe_osc1.l, coe_osc1.e) + coe_osc1.g;
				u = fmod(u, PI2);
				if (u < 0)
					u += PI2;

				X_L = u;
				X_L_dot = CurrentBlock.l_dot + CurrentBlock.g_dot;
			}
			//Maneuver line
			else
			{
				double u = OrbMech::MeanToTrueAnomaly(coe_osc1.l, coe_osc1.e) + coe_osc1.g;
				u = fmod(u, PI2);
				if (u < 0)
					u += PI2;

				X_L = u;
				X_L_dot = CurrentBlock.l_dot + CurrentBlock.g_dot;
				LINE = 2;
			}

			if (DH)
			{
				double DN_apo = in.Item10 * PI2;
				ddt = DN_apo / CurrentBlock.l_dot;
				DH = false;

				if (LINE != 0)
				{
					L_D = L_D + CurrentBlock.g_dot * ddt + DN_apo;
					while (L_D < 0) L_D += PI2;
					while (L_D >= PI2) L_D -= PI2;
				}
				else
				{
					ddt += (L_D - X_L) / X_L_dot;
				}
			}
			else
			{
				DX_L = L_D - X_L;
				if (abs(DX_L) - PI >= 0)
				{
					if (DX_L > 0)
					{
						DX_L -= PI2;
					}
					else
					{
						DX_L += PI2;
					}
				}
				ddt = DX_L / X_L_dot;
				if (LINE != 0)
				{
					L_D = L_D + ddt * CurrentBlock.g_dot;
				}
			}

			dt += ddt;
			coe_mean1.l = CurrentBlock.l_dot*dt + in.coe_mean.l;
			coe_mean1.g = CurrentBlock.g_dot*dt + in.coe_mean.g;
			coe_mean1.h = CurrentBlock.h_dot*dt + in.coe_mean.h;

			OrbMech::normalizeAngle(coe_mean1.l);
			OrbMech::normalizeAngle(coe_mean1.g);
			OrbMech::normalizeAngle(coe_mean1.h);

			coe_osc1 = LyddaneMeanToOsculating(coe_mean1);

			//Selenographic to selenocentric
			pRTCC->PIVECT(coe_osc1.i, coe_osc1.g, coe_osc1.h, P, W);
			P = tmul(Rot, P);
			W = tmul(Rot, W);
			pRTCC->PIVECT(P, W, coe_osc1.i, coe_osc1.g, coe_osc1.h);

			COUNT--;

		} while (abs(DX_L) > 2e-4 && COUNT > 0);

		if (COUNT == 0)
		{
			header.ErrorInd = -3;
		}

		CurrentBlock.TE = CurrentBlock.TS = in.TS + dt;
	}

	CurrentBlock.coe_osc = coe_osc1;
	CurrentBlock.f = OrbMech::MeanToTrueAnomaly(CurrentBlock.coe_osc.l, CurrentBlock.coe_osc.e);
	CurrentBlock.U = CurrentBlock.f + CurrentBlock.coe_osc.g;
	if (CurrentBlock.U >= PI2)
	{
		CurrentBlock.U -= PI2;
	}
	CurrentBlock.R = CurrentBlock.coe_osc.a*(1.0 - CurrentBlock.coe_osc.e*CurrentBlock.coe_osc.e) / (1.0 + CurrentBlock.coe_osc.e*cos(CurrentBlock.coe_osc.g)*cos(CurrentBlock.U) + CurrentBlock.coe_osc.e*sin(CurrentBlock.coe_osc.g)*sin(CurrentBlock.U));

	if (in.TIMA >= 4)
	{
		theta_R = CurrentBlock.U - tempblock.U - 2.0*atan(tan((CurrentBlock.coe_osc.h - tempblock.coe_osc.h) / 2.0)*(sin(0.5*(CurrentBlock.coe_osc.i + tempblock.coe_osc.i - PI)) / sin(0.5*(CurrentBlock.coe_osc.i - tempblock.coe_osc.i + PI))));
		if (theta_R < -PI)
		{
			theta_R += PI2;
		}
		else if (theta_R >= PI)
		{
			theta_R -= PI2;
		}
		if (in.TIMA == 4)
		{
			CurrentBlock.Item10 = theta_R;
		}
		else
		{
			if (firstpass)
			{
				CurrentBlock.Item10 = theta_R;
				firstpass = false;
			}
		}
	}

	if (in.TIMA >= 5)
	{
		CurrentBlock.Item8 = CurrentBlock.R - tempblock.R;
		CurrentBlock.Item9 = CurrentBlock.TE - tempblock.TE;
		dt += -theta_R / (CurrentBlock.l_dot + CurrentBlock.g_dot);
		if (abs(theta_R) > 0.00005)
		{
			goto NewPMMLAEG_V1000;
		}
	}

NewPMMLAEG_V1030:
	//Move output into area supplied by the calling program
	out.ENTRY = 0;
	out.Item7 = CurrentBlock.Item7;
	out.Item8 = CurrentBlock.Item8;
	out.Item9 = CurrentBlock.Item9;
	out.Item10 = CurrentBlock.Item10;
	out.coe_osc = CurrentBlock.coe_osc;
	out.f = CurrentBlock.f;
	out.U = CurrentBlock.U;
	out.R = CurrentBlock.R;
	out.TS = CurrentBlock.TS;
	out.TE = CurrentBlock.TE;
NewPMMLAEG_V305:
	return;
NewPMMLAEG_V846:
	header.ErrorInd = -1;
	goto NewPMMLAEG_V305;
}

CELEMENTS PMMLAEG::LyddaneMeanToOsculating(CELEMENTS arr) const
{
	CELEMENTS out;
	double ae, am, Em, fm, J2, J3, R_e, cn, cn2, theta, theta2, theta4, eccdp2, sinI, sinI2, cosI2, adr, adr2, adr3, a;
	double sinta, costa, costa2, sn2gta, cs2gta, sinGD, cosGD, sin2gd, cs2gd, sn3fgd, snf2gd, csf2gd, cs3fgd;
	double k2, k3, gamma2, gamma3, gamma2_apo, gamma3_apo, g3dg2;
	double A1_apo, A1, A2_apo, A6, A7, A10, A11, A12, A13, A14, A15, A16, A17, A18, A20, A21, A25, A26;
	double B1, B2, B4, B5, B7, B8, B10, B11, B13, B14;
	double delta1e, de, edl, di, sin_im2_dh, lagaha, lgh, sinMADP, cosMADP, sinraandp, cosraandp;
	bool pseudostate = false;

	J2 = OrbMech::J2_Moon;
	J3 = OrbMech::J3_Moon;
	R_e = OrbMech::R_Moon;

	if (arr.i > 175.0*RAD)
	{
		arr.i = PI - arr.i;
		arr.h = -arr.h;
		pseudostate = true;
	}

	ae = 1.0;
	am = arr.a / R_e;
	eccdp2 = arr.e*arr.e;
	cn2 = 1.0 - eccdp2;
	cn = sqrt(cn2);
	theta = cos(arr.i);
	theta2 = theta * theta;
	theta4 = theta2 * theta2;
	k2 = J2 * pow(ae, 2) / 2.0;
	k3 = -J3 * pow(ae, 3);
	gamma2 = k2 / pow(am, 2);
	gamma3 = k3 / pow(am, 3);
	gamma2_apo = gamma2 / pow(cn, 4);
	gamma3_apo = gamma3 / pow(cn, 6);

	g3dg2 = gamma3_apo / gamma2_apo;

	Em = OrbMech::kepler_E(arr.e, arr.l, 1e-12);
	fm = atan2(sqrt(1.0 - eccdp2)*sin(Em), cos(Em) - arr.e);
	if (fm < 0)
	{
		fm = fm + PI2;
	}
	adr = 1.0 / (1.0 - arr.e*cos(Em));
	adr2 = adr * adr;
	adr3 = adr2 * adr;

	sinI = sin(arr.i);
	sinI2 = sin(arr.i / 2.0);
	cosI2 = cos(arr.i / 2.0);
	sinta = sin(fm);
	costa = cos(fm);
	costa2 = costa * costa;
	sn2gta = sin(2.0*arr.g + 2.0*fm);
	cs2gta = cos(2.0*arr.g + 2.0*fm);
	snf2gd = sin(2.0 * arr.g + fm);
	csf2gd = cos(2.0 * arr.g + fm);
	sinGD = sin(arr.g);
	cosGD = cos(arr.g);
	sin2gd = sin(2.0 * arr.g);
	cs2gd = cos(2.0 * arr.g);
	sn3fgd = sin(3.0*fm + 2.0*arr.g);
	cs3fgd = cos(3.0*fm + 2.0*arr.g);
	sinMADP = sin(arr.l);
	cosMADP = cos(arr.l);
	sinraandp = sin(arr.h);
	cosraandp = cos(arr.h);

	A1_apo = 1.0 / (1.0 - 5.0 * theta2);
	A1 = 1.0 / 8.0 * gamma2_apo*cn2 * (1.0 - 11.0 * theta2 - 40.0 * theta4 * A1_apo);
	A2_apo = 3.0 * theta2 + 8.0 * theta4 * A1_apo;
	A6 = 1.0 / 4.0 * g3dg2;
	A7 = A6 * cn2 * sinI;
	A10 = 2.0 + eccdp2;
	A11 = 3.0 * eccdp2 + 2.0;
	A12 = A11 * theta2;
	A13 = (5.0 * eccdp2 + 2.0)*theta4 * A1_apo;
	A14 = eccdp2 * theta4*theta2 * pow(A1_apo, 2);
	A15 = theta2 * A1_apo;
	A16 = pow(A15, 2);
	A17 = arr.e * sinI;
	A18 = A17 / (1.0 + cn);
	A20 = arr.e * theta;
	A21 = arr.e * A20;
	A25 = 16.0 * A15 + 40.0 * A16 + 3.0;
	A26 = 1.0 / 8.0 * A21*(11.0 + 200.0 * A16 + 80.0 * A15);

	B1 = cn * A1 - (1.0 / 16.0 * (A10 - 400.0 * A14 - 40.0 * A13 - 11.0 * A12) + 1.0 / 8.0 * A21*(11.0 + 200.0 * A16 + 80.0 * A15))*gamma2_apo;
	B2 = A6 * A18*(2.0 + cn - eccdp2) + A20 * tan(arr.i / 2.0)*(A6);
	B4 = cn * arr.e*A1;
	B5 = cn * A7;
	B7 = cn2 * A17*A1_apo*(1.0 / 8.0 * gamma2_apo*(1.0 - 15.0 * theta2));
	B8 = cn2 * A6;
	B10 = sinI * (-A26 * gamma2_apo);
	B11 = A21 * A6;
	B13 = arr.e * A1;
	B14 = A7;

	a = am * (1.0 + gamma2 * ((3.0 * theta2 - 1.0)*arr.e / (cn2*cn2*cn2)*(arr.e*cn + arr.e / (1.0 + cn) + costa * (3.0 + 3.0 * arr.e*costa
		+ eccdp2 * costa2)) + 3.0 * (1.0 - theta2)*adr3 * cs2gta));

	delta1e = B13 * cs2gd + B14 * sinGD;
	de = delta1e - cn2 / 2.0 * (gamma2_apo*(1.0 - theta2)*(3.0 * csf2gd + cs3fgd)
		- 3.0 * gamma2 * 1.0 / (cn2*cn2*cn2)*(1.0 - theta2)*cs2gta*(3.0 * arr.e*costa2
			+ 3.0 * costa + eccdp2 * costa*costa2 + arr.e)
		- gamma2 * 1.0 / (cn2*cn2*cn2)*(3.0 * theta2 - 1.0)*(arr.e*cn + arr.e / (1.0 + cn) + 3.0 * arr.e*costa2 + 3.0 * costa + eccdp2 * costa * costa2));
	edl = B4 * sin2gd - B5 * cosGD
		- 1.0 / 4.0 * cn*cn2 * gamma2_apo*(2.0 * (3.0 * theta2 - 1.0)*(cn2 * adr2 + adr + 1.0)*sinta
			+ 3.0 * (1.0 - theta2)*((-cn2 * adr2 - adr + 1.0)*snf2gd
				+ (cn2 * adr2 + adr + 1.0 / 3.0)*sn3fgd));
	out.e = sqrt(pow(arr.e + de, 2) + pow(edl, 2));

	di = 1.0 / 2.0 * theta*gamma2_apo*sinI*(arr.e*cs3fgd + 3.0 * (arr.e*csf2gd + cs2gta))
		- A20 / cn2 * (B7*cs2gd + B8 * sinGD);
	sin_im2_dh = 1.0 / (2.0 * cosI2)*(B10*sin2gd + B11 * cosGD
		- 1.0 / 2.0 * gamma2_apo*theta*sinI*(6.0 * (arr.e*sinta - arr.l + fm)
			- 3.0 * (sn2gta + arr.e * snf2gd) - arr.e * sn3fgd));
	out.i = 2.0 * asin(sqrt(pow(sin_im2_dh, 2) + pow(1.0 / 2.0 * di*cosI2 + sinI2, 2)));

	if (out.e <= 1.0e-11)
	{
		out.l = 0;
	}
	else
	{
		out.l = atan2(edl*cosMADP + (arr.e + de)*sinMADP, (arr.e + de)*cosMADP - edl * sinMADP);
		if (out.l < 0)
		{
			out.l = out.l + PI2;
		}
	}

	if (out.i == 0.0)
	{
		out.h = 0;
	}
	else
	{
		out.h = atan2(sin_im2_dh*cosraandp + sinraandp * (1.0 / 2.0 * di*cosI2 + sinI2), cosraandp*(1.0 / 2.0 * di*cosI2 + sinI2) - sin_im2_dh * sinraandp);
		if (out.h < 0)
		{
			out.h = out.h + PI2;
		}
	}

	lagaha = arr.l + arr.g + arr.h + B1 * sin2gd + B2 * cosGD;
	lgh = lagaha + (1.0 / 4.0 * (cn2 / (cn + 1.0))*arr.e*gamma2_apo*(3.0 * (1.0 - theta2)*(sn3fgd
		*(1.0 / 3.0 + adr2 * cn2 + adr) + snf2gd * (1.0 - adr2 * cn2 - adr))
		+ 2.0 * sinta*(3.0 * theta2 - 1.0)*(1.0 + adr2 * cn2 + adr)))
		+ 3.0 / 2.0 * gamma2_apo*((5.0 * theta2 - 2.0 * theta - 1.0)*(arr.e*sinta + fm - arr.l)) + (3.0 + 2.0 * theta - 5.0 * theta2)
		*(1.0 / 4.0 * gamma2_apo*(arr.e*sn3fgd + 3.0 * (sn2gta + arr.e * snf2gd)));

	out.g = lgh - out.l - out.h;
	while (out.g >= PI2)
	{
		out.g -= PI2;
	}
	while (out.g < 0)
	{
		out.g += PI2;
	}

	if (pseudostate)
	{
		out.i = PI - out.i;
		out.h = -out.h;

		if (out.h < 0)
		{
			out.h = out.h + PI2;
		}
	}

	out.a = a * R_e;
	return out;
}