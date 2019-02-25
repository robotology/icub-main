// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/* Copyright (C) 2012  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Valentina Gaggero
 * email: valentina.gaggero@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

//here there is only a name space that contains all utils funtion of embObjMotionControl

namespace yarp {
    namespace dev  {
       namespace eomc

{
    enum { MAX_SHORT = 32767, MIN_SHORT = -32768, MAX_INT = 0x7fffffff, MIN_INT = 0x80000000,  MAX_U32 = 0xffffffff, MIN_U32 = 0x00, MAX_U16 = 0xffff, MIN_U16 = 0x0000};
    enum { CAN_SKIP_ADDR = 0x80 };
    
    // saturation check and rounding for 16 bit unsigned integer
    int U_16(double x)
    {
        if (x <= double(MIN_U16) )
            return MIN_U16;
        else
            if (x >= double(MAX_U16))
                return MAX_U16;
            else
                return int(x + .5);
    }
    
    // saturation check and rounding for 16 bit signed integer
    short S_16(double x)
    {
        if (x <= double(-(MAX_SHORT))-1)
            return MIN_SHORT;
        else
            if (x >= double(MAX_SHORT))
                return MAX_SHORT;
            else
                if  (x>0)
                    return short(x + .5);
                else
                    return short(x - .5);
    }
    
    // saturation check and rounding for 32 bit unsigned integer
    int U_32(double x) 
    {
        if (x <= double(MIN_U32) )
            return MIN_U32;
        else
            if (x >= double(MAX_U32))
                return MAX_U32;
            else
                return int(x + .5);
    }
    
    // saturation check and rounding for 32 bit signed integer
    int S_32(double x) 
    {
        if (x <= double(-(MAX_INT))-1.0)
            return MIN_INT;
        else
            if (x >= double(MAX_INT))
                return MAX_INT;
            else
                if  (x>0)
                    return int(x + .5);
                else
                    return int(x - .5);
    }
    
    bool EncoderType_iCub2eo(const string* in, uint8_t *out)
    {
        if (*in == "NONE")
        {
            *out = 0;
            return true;
        }
        else if (*in == "AEA")
        {
            *out = 1;
            return true;
        }
        else if (*in == "ROIE")
        {
            *out = 2;
            return true;
        }
        else if (*in == "HALL_ADC")
        {
            *out = 3;
            return true;
        }
        else if (*in == "MAIS")
        {
            *out = 4;
            return true;
        }
        else if (*in == "OPTICAL_QUAD")
        {
            *out = 5;
            return true;
        }
        else if (*in == "HALL_MOTOR_SENS")
        {
            *out = 6;
            return true;
        }
        *out = 0;
        return false;
    }
    
    bool EncoderType_eo2iCub(const uint8_t *in, string* out)
    {
        if (*in == 0)
        {
            *out = "NONE";
            return true;
        }
        else if (*in == 1)
        {
            *out = "AEA";
            return true;
        }
        else if (*in == 2)
        {
            *out = "ROIE";
            return true;
        }
        else if (*in == 3)
        {
            *out = "HALL_ADC";
            return true;
        }
        else if (*in == 4)
        {
            *out = "MAIS";
            return true;
        }
        else if (*in == 5)
        {
            *out = "OPTICAL_QUAD";
            return true;
        }
        else if (*in == 6)
        {
            *out = "HALL_MOTOR_SENS";
            return true;
        }
        else if(*in == 9)
        {
            *out = "AMO";
            return true;
        }
        *out = "ERROR";
        return false;
    }
    
    void copyPid_iCub2eo(const Pid *in, eOmc_PID_t *out)
    {
        memset(out, 0, sizeof(eOmc_PID_t));     // marco.accame: it is good thing to clear the out struct before copying. this prevent future members of struct not yet managed to be dirty.
        out->kp = (float) (in->kp);
        out->ki = (float) (in->ki);
        out->kd = (float) (in->kd);
        out->limitonintegral = (float)(in->max_int);
        out->limitonoutput = (float)(in->max_output);
        out->offset = (float) (in->offset);
        out->scale = (int8_t) (in->scale);
        out->kff = (float) (in->kff);
        out->stiction_down_val = (float)(in->stiction_down_val);
        out->stiction_up_val = (float)(in->stiction_up_val);
    }
    
    void copyPid_eo2iCub(eOmc_PID_t *in, Pid *out)
    {
        // marco.accame: in here i dont clear the out class because there is not a clear() method
        out->kp = (double) in->kp;
        out->ki = (double) in->ki;
        out->kd = (double) in->kd;
        out->max_int = (double) in->limitonintegral;
        out->max_output = (double) in->limitonoutput;
        out->offset = (double) in->offset;
        out->scale = (double) in->scale;
        out->setStictionValues(in->stiction_up_val, in->stiction_down_val);
        out->setKff(in->kff);
    }
    
    
    bool controlModeCommandConvert_yarp2embObj(int vocabMode, eOenum08_t &embOut)
    {
        bool ret = true;
        
        switch(vocabMode)
        {
            case VOCAB_CM_IDLE:
                embOut = eomc_controlmode_cmd_idle;
                break;
                
            case VOCAB_CM_POSITION:
                embOut = eomc_controlmode_cmd_position;
                break;
                
            case VOCAB_CM_POSITION_DIRECT:
                embOut = eomc_controlmode_cmd_direct;
                break;
                
            case VOCAB_CM_VELOCITY:
                embOut = eomc_controlmode_cmd_velocity;
                break;
                
            case VOCAB_CM_MIXED:
                embOut = eomc_controlmode_cmd_mixed;
                break;
                
            case VOCAB_CM_TORQUE:
                embOut = eomc_controlmode_cmd_torque;
                break;
                
            case VOCAB_CM_IMPEDANCE_POS:
                embOut = eomc_controlmode_cmd_impedance_pos;
                break;
                
            case VOCAB_CM_IMPEDANCE_VEL:
                embOut = eomc_controlmode_cmd_impedance_vel;
                break;
                
            case VOCAB_CM_PWM:
                embOut = eomc_controlmode_cmd_openloop;
                break;
                
            case VOCAB_CM_CURRENT:
                embOut = eomc_controlmode_cmd_current;
                break;
                
            case VOCAB_CM_FORCE_IDLE:
                embOut = eomc_controlmode_cmd_force_idle;
                break;
                
            default:
                ret = false;
                break;
        }
        return ret;
    }
    
    int controlModeStatusConvert_embObj2yarp(eOenum08_t embObjMode)
    {
        int vocabOut;
        switch(embObjMode)
        {
            case eomc_controlmode_idle:
                vocabOut = VOCAB_CM_IDLE;
                break;
                
            case eomc_controlmode_position:
                vocabOut = VOCAB_CM_POSITION;
                break;
                
            case eomc_controlmode_velocity:
                vocabOut = VOCAB_CM_VELOCITY;
                break;
                
            case eomc_controlmode_direct:
                vocabOut = VOCAB_CM_POSITION_DIRECT;
                break;
                
            case eomc_controlmode_mixed:
            case eomc_controlmode_velocity_pos:      // they are the same, this will probably removed in the future
                vocabOut = VOCAB_CM_MIXED;
                break;
                
            case eomc_controlmode_torque:
                vocabOut = VOCAB_CM_TORQUE;
                break;
                
            case eomc_controlmode_calib:
                vocabOut = VOCAB_CM_CALIBRATING;
                break;
                
            case eomc_controlmode_impedance_pos:
                vocabOut = VOCAB_CM_IMPEDANCE_POS;
                break;
                
            case eomc_controlmode_impedance_vel:
                vocabOut = VOCAB_CM_IMPEDANCE_VEL;
                break;
                
            case eomc_controlmode_openloop:
                vocabOut = VOCAB_CM_PWM;
                break;
                
            case eomc_controlmode_current:
                vocabOut = VOCAB_CM_CURRENT;
                break;
                
            case eomc_controlmode_hwFault:
                vocabOut = VOCAB_CM_HW_FAULT;
                break;
                
            case eomc_controlmode_notConfigured:
                vocabOut = VOCAB_CM_NOT_CONFIGURED;
                break;
                
            case eomc_controlmode_configured:
                vocabOut = VOCAB_CM_CONFIGURED;
                break;
                
            default:
                printf("embObj to yarp unknown controlmode %d\n", embObjMode);
                vocabOut = VOCAB_CM_UNKNOWN;
                break;
        }
        return vocabOut;
    }
    
    bool controlModeStatusConvert_yarp2embObj(int vocabMode, eOmc_controlmode_t &embOut)
    {
        yError() << "controlModeStatusConvert_yarp2embObj" << " is not yet implemented for embObjMotionControl";
        return false;
    }
    
    int controlModeCommandConvert_embObj2yarp(eOmc_controlmode_command_t embObjMode)
    {
        yError() << "embObjMotionControl::controlModeCommandConvert_embObj2yarp" << " is not yet implemented for embObjMotionControl";
        return 0;
    
    }
    /*
    eOmc_pidoutputtype_t pidOutputTypeConver_eomc2fw(PidAlgorithmType_t controlLaw)
    {
        switch(controlLaw)
        {
            case PidAlgo_simple:
                return(eomc_pidoutputtype_pwm);
                
            case PIdAlgo_velocityInnerLoop:
                return(eomc_pidoutputtype_vel);
                
            case PidAlgo_currentInnerLoop:
                return(eomc_pidoutputtype_iqq);
            default:
            {
                yError() << "pidOutputTypeConver_eomc2fw: unknown pid output type" ;
                return(eomc_pidoutputtype_unknown);
            }
        }
    }
    */
    
    bool interactionModeCommandConvert_yarp2embObj(int vocabMode, eOenum08_t &embOut)
    {
        bool ret = true;
        
        switch(vocabMode)
        {
            case VOCAB_IM_STIFF:
                embOut = eOmc_interactionmode_stiff;
                break;
                
            case VOCAB_IM_COMPLIANT:
                embOut = eOmc_interactionmode_compliant;
                break;
                
            default:
                ret = false;
                break;
        }
        return ret;
    }
    
    
    bool interactionModeStatusConvert_embObj2yarp(eOenum08_t embObjMode, int &vocabOut)
    {
        bool ret = true;
        switch(embObjMode)
        {
            case eOmc_interactionmode_stiff:
                vocabOut = VOCAB_IM_STIFF;
                break;
                
            case eOmc_interactionmode_compliant:
                vocabOut = VOCAB_IM_COMPLIANT;
                break;
                
            default:
                vocabOut = 666; //VOCAB_CM_UNKNOWN;
                yError() << "Received an unknown interactionMode from the EMS boards with value " << embObjMode;
                //        ret = false;
                break;
        }
        return ret;
    }
    
    
    
    }
    }}