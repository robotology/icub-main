// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
* Authors: Valentina Gaggero
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*
*/

#include "measuresConverter.h"



measureConvFactors::measureConvFactors(int numofjoints)
{
    angleToEncoder   = new double [numofjoints];
    dutycycleToPWM   = new double [numofjoints];
    ampsToSensor     = new double [numofjoints];
    newtonsToSensor  = new double [numofjoints];
}

measureConvFactors::~measureConvFactors()
{
    if (angleToEncoder)   delete [] angleToEncoder;
    if (dutycycleToPWM)   delete [] dutycycleToPWM;
    if (ampsToSensor)     delete [] ampsToSensor;
    if (newtonsToSensor)  delete [] newtonsToSensor;

    angleToEncoder = nullptr;
    dutycycleToPWM = nullptr;
    ampsToSensor = nullptr;
    newtonsToSensor  = nullptr;
}


torqueControlConvFactors::torqueControlConvFactors(int numofjoints)
{
    angleToEncoder   = new double [numofjoints];
    newtonsToSensor  = new double [numofjoints];
    this->numofjoints = numofjoints;
}

bool torqueControlConvFactors::init (const double *angleToEncoderFactors, const double *newtonsToSensorFactors)
{
    if(angleToEncoderFactors == nullptr || newtonsToSensorFactors==nullptr)
        return false;
    memcpy(angleToEncoder, angleToEncoderFactors, sizeof(double)*numofjoints);
    memcpy(newtonsToSensor, newtonsToSensorFactors, sizeof(double)*numofjoints);
    return true;
}

torqueControlConvFactors::~torqueControlConvFactors()
{
    if (angleToEncoder)   delete [] angleToEncoder;
    if (newtonsToSensor)  delete [] newtonsToSensor;

    angleToEncoder = nullptr;
    newtonsToSensor  = nullptr;
}

double torqueControlConvFactors::get_N2S(int joint)
{
    return newtonsToSensor[joint];
}

double torqueControlConvFactors::get_A2E(int joint)
{
    return angleToEncoder[joint];
}






measuresConverter::measuresConverter( int n, const int *aMap, torqueControlConvFactors &trqCrtlFact, measureConvFactors &measFact):
ControlBoardHelper(n, aMap, measFact.angleToEncoder, nullptr, measFact.newtonsToSensor, measFact.ampsToSensor, nullptr , measFact.dutycycleToPWM)
{
    trqCrtlFactors = new torqueControlConvFactors(n);
    trqCrtlFactors->init(trqCrtlFact.angleToEncoder, trqCrtlFact.newtonsToSensor);
}

measuresConverter::~measuresConverter()
{
    if(trqCrtlFactors) delete trqCrtlFactors;
}

double measuresConverter::getAngleToEncoder(int j)
{
    return angleToEncoders[j];
}

void measuresConverter::convertTrqPid_N2S(int j, yarp::dev::Pid &pid)
{
    //Here i need to use the conversion factors related to the torque control that could be different from the other torque conversion
    double trq_factor_N2S = trqCrtlFactors->get_N2S(j);

    pid.kp = pid.kp / trq_factor_N2S ;  //[PWM/Nm] ==> [PWM/microNm]
    pid.ki = pid.ki / trq_factor_N2S;   //[PWM/Nm] ==> [PWM/microNm]
    pid.kd = pid.kd / trq_factor_N2S;   //[PWM/Nm] ==> [PWM/microNm]
    pid.stiction_up_val   = pid.stiction_up_val * trq_factor_N2S;   //[Nm] ==> [microNm]
    pid.stiction_down_val = pid.stiction_down_val * trq_factor_N2S; //[Nm] ==> [microNm]
}


void measuresConverter::convertTrqPid_S2N(int j, yarp::dev::Pid &pid)
{
    //Here i need to use the conversion factors related to the torque control that could be different from the other torque conversion
    double trq_factor_N2S = trqCrtlFactors->get_N2S(j);

    pid.kp = pid.kp * trq_factor_N2S; //[PWM/microNm] ==> [PWM/Nm]
    pid.ki = pid.ki * trq_factor_N2S; //[PWM/microNm] ==> [PWM/Nm]
    pid.kd = pid.kd * trq_factor_N2S; //[PWM/microNm] ==> [PWM/Nm]
    pid.stiction_up_val   = pid.stiction_up_val   / trq_factor_N2S; //[microNm] ==> [Nm]
    pid.stiction_down_val = pid.stiction_down_val / trq_factor_N2S; //[microNm] ==> [Nm]
}


void measuresConverter::convertPosPid_A2E(int j, yarp::dev::Pid &pid)
{
    pid.kp = pid.kp / getAngleToEncoder(j);  //[PWM/deg] ==> [PWM/icubdegrees]
    pid.ki = pid.ki / getAngleToEncoder(j);  //[PWM/deg] ==> [PWM/icubdegrees]
    pid.kd = pid.kd / getAngleToEncoder(j);  //[PWM/deg] ==> [PWM/icubdegrees]
}

void measuresConverter::convertPosPid_E2A(int j, yarp::dev::Pid &pid)
{
    pid.kp = pid.kp * getAngleToEncoder(j); //[PWM/icubdegrees] ==> [PWM/deg]
    pid.ki = pid.ki * getAngleToEncoder(j); //[PWM/icubdegrees] ==> [PWM/deg]
    pid.kd = pid.kd * getAngleToEncoder(j); //[PWM/icubdegrees] ==> [PWM/deg]
}

double measuresConverter::convertTrqMotorBemfParam_MachineUnitsToMetric(int j, double bemf)
{
    //Here i need to use the conversion factors related to the torque control that could be different from the other torque conversion
//     ("convertTrqMotorBemfParam_MachineUnitsToMetric(%d)\n", j);
//     if(trqCrtlFactors==nullptr)
//     {
//         printf("   - ptr is null!!\n");
//         return (1.4);
//     }
//     printf("   - N2S %f \n", trqCrtlFactors->get_N2S(j));
//     printf("   - A2E %f \n", trqCrtlFactors->get_A2E(j));


    return ( bemf / trqCrtlFactors->get_N2S(j) *  trqCrtlFactors->get_A2E(j));
}

double measuresConverter::convertTrqMotorKtaufParam_MachineUnitsToMetric(int j, double ktau)
{
    //Here i need to use the conversion factors related to the torque control that could be different from the other torque conversion
//     printf("convertTrqMotorKtaufParam_MachineUnitsToMetric(%d)\n", j);
//     if(trqCrtlFactors==nullptr)
//     {
//         printf("   - ptr is null!!\n");
//         return (1.5);
//     }
//     printf("   - N2S %f \n", trqCrtlFactors->get_N2S(j));
//     printf("   - A2E %f \n", trqCrtlFactors->get_A2E(j));

    return ( ktau * trqCrtlFactors->get_N2S(j));
}

double measuresConverter::convertTrqMotorBemfParam_MetricToMachineUnits(int j, double bemf)
{
    //Here i need to use the conversion factors related to the torque control that could be different from the other torque conversion
//     printf("convertTrqMotorBemfParam_MetricToMachineUnits(%d)\n", j);
//     if(trqCrtlFactors==nullptr)
//     {
//         printf("   - ptr is null!!\n");
//         return (1.2);
//     }
//     printf("   - N2S %f \n", trqCrtlFactors->get_N2S(j));
//     printf("   - A2E %f \n", trqCrtlFactors->get_A2E(j));
//
    return ( bemf * trqCrtlFactors->get_N2S(j) /  trqCrtlFactors->get_A2E(j));
}

double measuresConverter::convertTrqMotorKtaufParam_MetricToMachineUnits(int j, double ktau)
{
    //Here i need to use the conversion factors related to the torque control that could be different from the other torque conversion
//     printf("convertTrqMotorKtaufParam_MetricToMachineUnits(%d)\n", j);
//     if(trqCrtlFactors==nullptr)
//     {
//         printf("   - ptr is null!!\n");
//         return (1.3);
//     }
//     printf("   - N2S %f \n", trqCrtlFactors->get_N2S(j));
//     printf("   - A2E %f \n", trqCrtlFactors->get_A2E(j));


    return ( ktau / trqCrtlFactors->get_N2S(j));
}
