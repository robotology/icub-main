#include <math.h>

const double PI=3.14159265;
const double TOLERANCE = 0.0001;



void computeTendonsLength(double &d1, double &d2, double &d3, double Roll, double Pitch)
{

	double L        = 4.0;      // length of the spring
    double L_2      = 2.0;      // length of rigid part of the spring
    double L_cables = 5.2;      // distance between tendons
	double R_capstan= 1.0;		// radius of the capstan
	double Spring_R = 1.5/2;		// radius of the spring
	double s1, s2, s3;
	double s4;
	double t0;

    Roll = Roll * PI / 180;
    Pitch = Pitch * PI / 180;

	if ((fabs(Roll) < TOLERANCE) && (fabs(Pitch) < TOLERANCE))
        {
            d1 = 0.0;
            d2 = 0.0;
            d3 = 0.0;
        }
	else
        {
			s1 = pow(-(-cos(Pitch)*L_cables*sqrt(1.0-pow(cos(Roll),2.0)*pow(cos(Pitch),2.0))*acos(cos(Roll)*cos(Pitch))-2.0*sin(Pitch)*L-2.0*sin(Pitch)*Spring_R*acos(cos(Roll)*cos(Pitch))+2.0*sin(Pitch)*cos(Roll)*cos(Pitch)*L+2.0*sin(Pitch)*cos(Roll)*cos(Pitch)*Spring_R*acos(cos(Roll)*cos(Pitch)))/sqrt(1.0-pow(cos(Roll 
			),2.0)*pow(cos(Pitch),2.0))/acos(cos(Roll)*cos(Pitch))/2.0-L_cables/2.0,2.0);      s3 = pow((3.0*sin(Roll)*sin(Pitch)*L_cables*sqrt(1.0-pow(cos(Roll),2.0)*pow(cos(Pitch),2.0))*acos(cos(Roll)*cos(Pitch))+cos(Roll)*L_cables*sqrt(3.0)*sqrt(1.0-pow(cos(Roll),2.0)*pow(cos(Pitch),2.0))*acos(cos(Roll)*cos(Pitch))-6.0*sin(Roll)*cos(Pitch)*L-6.0*sin(Roll)*cos(Pitch)*Spring_R*acos(cos(Roll)*cos(Pitch 
			))+6.0*sin(Roll)*pow(cos(Pitch),2.0)*cos(Roll)*L+6.0*sin(Roll)*pow(cos(Pitch 
			),2.0)*cos(Roll)*Spring_R*acos(cos(Roll)*cos(Pitch)))/sqrt(1.0-pow(cos(Roll 
			),2.0)*pow(cos(Pitch),2.0))/acos(cos(Roll)*cos(Pitch))/6.0-L_cables*sqrt(3.0)/6.0,2.0);      s4 = pow(-3.0*cos(Roll)*sin(Pitch)*L_cables*acos(cos(Roll)*cos(Pitch))+sin(Roll)*L_cables*sqrt(3.0)*acos(cos(Roll)*cos(Pitch))+6.0*sqrt(1.0-pow(cos(Roll 
			),2.0)*pow(cos(Pitch),2.0))*L+6.0*sqrt(1.0-pow(cos(Roll),2.0)*pow(cos(Pitch 
			),2.0))*Spring_R*acos(cos(Roll)*cos(Pitch)),2.0)/pow(acos(cos(Roll)*cos(Pitch 
			)),2.0)/36.0;      s2 = s3+s4;      t0 = s1+s2; 
			
			d1 = sqrt(t0)-L;
			d1 = -d1 / R_capstan * (180.0/PI);

			s1 = pow(-(cos(Pitch)*L_cables*sqrt(1.0-pow(cos(Roll),2.0)*pow(cos(Pitch),2.0))*acos(cos(Roll)*cos(Pitch))-2.0*sin(Pitch)*L-2.0*sin(Pitch)*Spring_R*acos(cos(Roll)*cos(Pitch))+2.0*sin(Pitch)*cos(Roll)*cos(Pitch)*L+2.0*sin(Pitch)*cos(Roll)*cos(Pitch)*Spring_R*acos(cos(Roll)*cos(Pitch)))/sqrt(1.0-pow(cos(Roll),2.0)*pow(cos(Pitch),2.0))/acos(cos(Roll)*cos(Pitch))/2.0+L_cables/2.0,2.0);      s3 = pow((-3.0*sin(Roll)*sin(Pitch)*L_cables*sqrt(1.0-pow(cos(Roll),2.0)*pow(cos(Pitch),2.0))*acos(cos(Roll)*cos(Pitch))+cos(Roll)*L_cables*sqrt(3.0)*sqrt(1.0-pow(cos(Roll),2.0)*pow(cos(Pitch),2.0))*acos(cos(Roll)*cos(Pitch))-6.0*sin(Roll)*cos(Pitch)*L-6.0*sin(Roll)*cos(Pitch)*Spring_R*acos(cos(Roll)*cos(Pitch 
			))+6.0*sin(Roll)*pow(cos(Pitch),2.0)*cos(Roll)*L+6.0*sin(Roll)*pow(cos(Pitch 
			),2.0)*cos(Roll)*Spring_R*acos(cos(Roll)*cos(Pitch)))/sqrt(1.0-pow(cos(Roll 
			),2.0)*pow(cos(Pitch),2.0))/acos(cos(Roll)*cos(Pitch))/6.0-L_cables*sqrt(3.0)/6.0,2.0);      s4 = pow(3.0*cos(Roll)*sin(Pitch)*L_cables*acos(cos(Roll)*cos(Pitch))+sin(Roll)*L_cables*sqrt(3.0)*acos(cos(Roll)*cos(Pitch))+6.0*sqrt(1.0-pow(cos(Roll),2.0)*pow(cos(Pitch),2.0))*L+6.0*sqrt(1.0-pow(cos(Roll),2.0)*pow(cos(Pitch),2.0))*Spring_R*acos(cos(Roll)*cos(Pitch)),2.0)/pow(acos(cos(Roll)*cos(Pitch)),2.0)/36.0;      s2 = s3+s4;      t0 = s1+s2; 

			d2 = sqrt(t0)-L; 
			d2 = -d2 / R_capstan * (180.0/PI);

			s1 = pow(sin(Pitch),2.0)*pow(-L-Spring_R*acos(cos(Roll)*cos(Pitch))+cos(Roll 
			)*cos(Pitch)*L+cos(Roll)*cos(Pitch)*Spring_R*acos(cos(Roll)*cos(Pitch)),2.0)/(1.0-pow(cos(Roll),2.0)*pow(cos(Pitch),2.0))/pow(acos(cos(Roll)*cos(Pitch)),2.0);      s2 = pow((-cos(Roll)*L_cables*sqrt(3.0)*sqrt(1.0-pow(cos(Roll),2.0)*pow(cos(Pitch),2.0))*acos(cos(Roll)*cos(Pitch))-3.0*sin(Roll)*cos(Pitch)*L-3.0*sin(Roll 
			)*cos(Pitch)*Spring_R*acos(cos(Roll)*cos(Pitch))+3.0*sin(Roll)*pow(cos(Pitch 
			),2.0)*cos(Roll)*L+3.0*sin(Roll)*pow(cos(Pitch),2.0)*cos(Roll)*Spring_R*acos(cos(Roll)*cos(Pitch)))/sqrt(1.0-pow(cos(Roll),2.0)*pow(cos(Pitch),2.0))/acos(cos(Roll)*cos(Pitch))/3.0+L_cables*sqrt(3.0)/3.0,2.0)+pow(-sin(Roll)*L_cables 
			*sqrt(3.0)*acos(cos(Roll)*cos(Pitch))+3.0*sqrt(1.0-pow(cos(Roll),2.0)*pow(cos(Pitch),2.0))*L+3.0*sqrt(1.0-pow(cos(Roll),2.0)*pow(cos(Pitch),2.0))*Spring_R 
			*acos(cos(Roll)*cos(Pitch)),2.0)/pow(acos(cos(Roll)*cos(Pitch)),2.0)/9.0;      t0 = s1+s2; 

			d3 = sqrt(t0)-L;
			d3 = -d3 / R_capstan * (180.0/PI);
        }
}


void computeModifiedPitchRoll(double Yaw, double Roll, double Pitch, double &Roll_hat, double &Pitch_hat)
{

    Roll = Roll * PI / 180;
    Pitch = Pitch * PI / 180;
	Yaw = Yaw * PI / 180;

	double x, x_hat;
	double y, y_hat;
	double z, z_hat;



	x = -sin(Pitch) * cos(Roll);
	y = -sin(Roll);
	z = cos(Pitch) * cos(Roll);

	x_hat = x*cos(Yaw) + y*sin(Yaw);
	y_hat = -x*sin(Yaw) + y*cos(Yaw); 
	z_hat = z;

	Pitch_hat = atan2(-x_hat, z_hat);
	Roll_hat = atan2(-y_hat, z_hat/cos(Pitch_hat));

	Pitch_hat = Pitch_hat *180/PI;
	Roll_hat = Roll_hat *180/PI;

}

void computeOriginalPitchRoll(double Yaw, double &Roll, double &Pitch, double Roll_hat, double Pitch_hat)
{

    Roll_hat = Roll_hat * PI / 180;
    Pitch_hat = Pitch_hat * PI / 180;
	Yaw = Yaw * PI / 180;

	double x, x_hat;
	double y, y_hat;
	double z, z_hat;



	x_hat = -sin(Pitch_hat) * cos(Roll_hat);
	y_hat = -sin(Roll_hat);
	z_hat = cos(Pitch_hat) * cos(Roll_hat);

	x = x_hat*cos(Yaw) + y_hat*sin(Yaw);
	y = -x_hat*sin(Yaw) + y_hat*cos(Yaw); 
	z = z_hat;

	Pitch = atan2(-x, z);
	Roll = atan2(-y, z/cos(Pitch));

	Pitch = Pitch *180/PI;
	Roll = Roll *180/PI;

}



/* old version...


void computeTendonsLength(double &d1, double &d2, double &d3, double Roll, double Pitch)
{

	double L        = 4.0;      // length of the spring
    double L_2      = 2.0;      // length of rigid part of the spring
    double L_cables = 5.2;      // distance between tendons
	double R_capstan= 1.0;		// radius of the capstan
	double s1, s2, s3;
	double s4, s5, s6;
	double t0;


    Roll = Roll * PI / 180;
    Pitch = Pitch * PI / 180;

	if ((fabs(Roll) < TOLERANCE) && (fabs(Pitch) < TOLERANCE))
	{
		d1 = 0.0;
		d2 = 0.0;
		d3 = 0.0;
	}
	else
	{
		s2 = 1.0/6.0;      s5 = 36.0*pow((-6.0*sin(Pitch)*pow(cos(Roll),2.0)*L*cos(Pitch)+3.0*cos(Pitch 
			)*L_cables*sqrt(-pow(cos(Pitch),2.0)*pow(cos(Roll),2.0)+1.0)*acos(cos(Pitch 
			)*cos(Roll))+sin(Pitch)*sin(Roll)*L_cables*sqrt(3.0)*sqrt(-pow(cos(Pitch),2.0)*pow(cos(Roll),2.0)+1.0)*acos(cos(Pitch)*cos(Roll))+6.0*sin(Pitch)*cos(Roll 
			)*L)/sqrt(-pow(cos(Pitch),2.0)*pow(cos(Roll),2.0)+1.0)/acos(cos(Pitch)*cos(Roll))/6.0-L_cables/2.0,2.0);      s6 = 36.0*pow((cos(Roll)*L_cables*sqrt(3.0)*sqrt(-pow(cos(Pitch),2.0)*pow(cos(Roll),2.0)+1.0)*acos(cos(Pitch)*cos(Roll))-6.0*sin(Roll)*L+6.0*sin(Roll)*L*cos(Pitch)*cos(Roll))/sqrt(-pow(cos(Pitch),2.0)*pow(cos(Roll),2.0)+1.0)/acos(cos(Pitch)*cos(Roll))/6.0-L_cables*sqrt(3.0)/6.0,2.0)+pow(3.0*sin(Pitch)*L_cables 
			*acos(cos(Pitch)*cos(Roll))-cos(Pitch)*sin(Roll)*L_cables*sqrt(3.0)*acos(cos(Pitch)*cos(Roll))-6.0*L*sqrt(-pow(cos(Pitch),2.0)*pow(cos(Roll),2.0)+1.0),2.0)/pow(acos(cos(Pitch)*cos(Roll)),2.0);      s4 = s5+s6;      s3 = sqrt(s4);      s1 = s2*s3;      s2 = -sqrt(L*L);      t0 = s1+s2; 

		d1 = t0;

		d1 = -d1 / R_capstan * (180.0/PI);


		s2 = 1.0/6.0;      s5 = 36.0*pow((-6.0*sin(Pitch)*pow(cos(Roll),2.0)*L*cos(Pitch)-3.0*cos(Pitch 
			)*L_cables*sqrt(-pow(cos(Pitch),2.0)*pow(cos(Roll),2.0)+1.0)*acos(cos(Pitch 
			)*cos(Roll))+sin(Pitch)*sin(Roll)*L_cables*sqrt(3.0)*sqrt(-pow(cos(Pitch),2.0)*pow(cos(Roll),2.0)+1.0)*acos(cos(Pitch)*cos(Roll))+6.0*sin(Pitch)*cos(Roll 
			)*L)/sqrt(-pow(cos(Pitch),2.0)*pow(cos(Roll),2.0)+1.0)/acos(cos(Pitch)*cos(Roll))/6.0+L_cables/2.0,2.0);      s6 = 36.0*pow((cos(Roll)*L_cables*sqrt(3.0)*sqrt(-pow(cos(Pitch),2.0)*pow(cos(Roll),2.0)+1.0)*acos(cos(Pitch)*cos(Roll))-6.0*sin(Roll)*L+6.0*sin(Roll)*L*cos(Pitch)*cos(Roll))/sqrt(-pow(cos(Pitch),2.0)*pow(cos(Roll),2.0)+1.0)/acos(cos(Pitch)*cos(Roll))/6.0-L_cables*sqrt(3.0)/6.0,2.0)+pow(3.0*sin(Pitch)*L_cables 
			*acos(cos(Pitch)*cos(Roll))+cos(Pitch)*sin(Roll)*L_cables*sqrt(3.0)*acos(cos(Pitch)*cos(Roll))+6.0*L*sqrt(-pow(cos(Pitch),2.0)*pow(cos(Roll),2.0)+1.0),2.0)/pow(acos(cos(Pitch)*cos(Roll)),2.0);      s4 = s5+s6;      s3 = sqrt(s4);      s1 = s2*s3;      s2 = -sqrt(L*L);      t0 = s1+s2; 

		d2 = t0; 
		d2 = -d2 / R_capstan * (180.0/PI);

		t0 = 1/(sqrt(cos(Pitch)*cos(Roll)+1.0))*sqrt(3.0)*sqrt(2.0)*sqrt(-3.0/pow(acos(cos(Pitch)*cos(Roll)),2.0)*L*L*pow(cos(Pitch),2.0)*pow(cos(Roll),2.0)-cos(Pitch)*L_cables*L_cables*pow(cos(Roll),2.0)+cos(Pitch)*cos(Roll)*L_cables*L_cables 
			-cos(Roll)*L_cables*L_cables-1/acos(cos(Pitch)*cos(Roll))*sqrt(-pow(cos(Pitch),2.0)*pow(cos(Roll),2.0)+1.0)*cos(Pitch)*sin(Roll)*L_cables*sqrt(3.0)*L-1/acos(cos(Pitch)*cos(Roll))*sin(Roll)*L*L_cables*sqrt(3.0)*sqrt(-pow(cos(Pitch 
			),2.0)*pow(cos(Roll),2.0)+1.0)+L_cables*L_cables+3.0/pow(acos(cos(Pitch)*cos(Roll)),2.0)*L*L)/3.0-sqrt(L*L); 

		d3 = t0;
		d3 = -d3 / R_capstan * (180.0/PI);
	}
}

void computeModifiedPitchRoll(double Yaw, double Roll, double Pitch, double &Roll_hat, double &Pitch_hat)
{

    Roll = Roll * PI / 180;
    Pitch = Pitch * PI / 180;
	Yaw = Yaw * PI / 180;

	double x, x_hat;
	double y, y_hat;
	double z, z_hat;



	x = -sin(Pitch) * cos(Roll);
	y = -sin(Roll);
	z = cos(Pitch) * cos(Roll);

	x_hat = x*cos(Yaw) + y*sin(Yaw);
	y_hat = -x*sin(Yaw) + y*cos(Yaw); 
	z_hat = z;

	Pitch_hat = atan2(-x_hat, z_hat);
	Roll_hat = atan2(-y_hat, z_hat/cos(Pitch_hat));

	Pitch_hat = Pitch_hat *180/PI;
	Roll_hat = Roll_hat *180/PI;

}

  */