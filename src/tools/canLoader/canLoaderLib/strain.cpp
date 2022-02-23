/*
 * Copyright (C) 2018 iCub Facility - Istituto Italiano di Tecnologia
 * Author:  Marco Accame
 * email:   marco.accame@iit.it
 * website: www.robotcub.org
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


// --------------------------------------------------------------------------------------------------------------------
// - public interface
// --------------------------------------------------------------------------------------------------------------------

#include "strain.h"



// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include <cstring>
#include <vector>
#include <fstream>

using namespace std;



// --------------------------------------------------------------------------------------------------------------------
// - pimpl: private implementation (see scott meyers: item 22 of effective modern c++, item 31 of effective c++
// --------------------------------------------------------------------------------------------------------------------

// so let me introduce to you the one and only .... pimpl


#define IMPORT_CODE_FROM_EMBOT_HW_PGA308

struct strain::amplifier::PGA308::Impl
{

    // marco.accame on 27jul18: i decide to use code already developed (and very well tested ....) in:
    // namespace embot { namespace hw { namespace PGA308 {
    // i encapluslate it under a different scope (this Impl) and I remove dependency from can protocol
    // by changing: embot::app::canprotocol::analog::polling::PGA308cfg1
    // with: strain::amplifier::PGA308::Registers
    // teh two structs have the same bitmap layout. but Registers also have methods to import from and export to a can packet

#if defined(IMPORT_CODE_FROM_EMBOT_HW_PGA308)

    struct ZDACregister
    {
        std::uint16_t value;
        static const std::uint16_t Default = 0x8000;

        // ZDAC Register: Zero DAC Register (Fine Offset Adjust)
        // bit:         15      14      13      12      11      10      09      08      07      06      05      04      03      02      01      00
        // content:     ZD15    ZD14    ZD13    ZD12    ZD11    ZD10    ZD9     ZD8     ZD7     ZD6     ZD5     ZD4     ZD3     ZD2     ZD1     ZD0
        // teh value x contained in the register is mapped into: +VREF/2 – (x/65536) (VREF)
        // the possible ranges of the offset is [+VREF*0.5, -VREF*0.4999847]

        ZDACregister() : value(0) {}

        ZDACregister(std::uint16_t v) : value(v) {}

        void reset() { value = 0; }

        void set(std::uint16_t zdac) { value = zdac; }

        void setDefault() { value = ZDACregister::Default; }
    };

    struct GDACregister
    {
        std::uint16_t value;
        static const std::uint16_t Default = 0x4000;

        // GDAC Register: Gain DAC Register
        // bit:         15      14      13      12      11      10      09      08      07      06      05      04      03      02      01      00
        // content:     GD15    GD14    GD13    GD12    GD11    GD10    GD9     GD8     GD7     GD6     GD5     GD4     GD3     GD2     GD1     GD0
        // the value x contained in the register is mapped into: 0.333333333 + x * (1.000000000 – 0.333333333) / 65536 = 0.333333333 + x * 1.0172526 * 10–5
        // value of register x is: (DesiredGain – 0.333333333) / (1.0172526 x 10–5)
        // the range of gain is [0.333333333, 0.999989824] and GDACregister::Default is 0.499999999

        GDACregister() : value(0) {}

        GDACregister(std::uint16_t v) : value(v) {}

        void reset() { value = 0; }

        void set(std::uint16_t gdac) { value = gdac; }

        void setDefault() { value = GDACregister::Default; }
    };

    struct CFG0register
    {
        std::uint16_t value;

        static const std::uint8_t DefaultGO = 0x06;             // output gain = 6.0 (0x06) OR 2.0 (0x00)
        static const std::uint8_t DefaultGO_g_32_96 = 0x06;     // output gain = 6.0 (0x06)
        static const std::uint8_t DefaultGO_g_16_80 = 0x02;     // output gain = 3.0 (0x02)
        static const std::uint8_t DefaultGO_g_11_75 = 0x00;     // output gain = 3.0 (0x00)
        static const std::uint8_t DefaultMUX = 0x00;    // VIN1= VINPositive, VIN2= VINNegative
        static const std::uint8_t DefaultGI = 0x04;     // input gain = 16
        static const std::uint8_t DefaultOS = 0x20;     // coarse offset  // 0x25 0x8A

        // CFG0 Output Amplifier Gain Select, Front-End PGA Mux & Gain Select, Coarse Offset Adjust on Front-End PGA
        // bit:         15      14      13      12      11      10      09      08      07      06      05      04      03      02      01      00
        // content:     GO2     GO1     GO0     GI4     GI3     GI2     GI1     GI0     OS7     OS6     OS5     OS4     OS3     OS2     OS1     OS0

        CFG0register() : value(0) {}

        CFG0register(std::uint8_t GO, std::uint8_t MUX, std::uint8_t GI, std::uint8_t OS)
        {
            value = 0;
            setGO(GO);
            setMUX(MUX);
            setGI(GI);
            setOS(OS);
        }

        CFG0register(std::uint16_t v) : value(v) {}

        void reset() { value = 0; }

        void setDefault() { value = 0; setGO(CFG0register::DefaultGO); setMUX(CFG0register::DefaultMUX); setGI(CFG0register::DefaultGI); setOS(CFG0register::DefaultOS); }

        // Output Amplifier Gain Select: [GO2-GO0] -> from 000b to 111b: 2.0, 2.4, 3, 3.6, 4.0, 4.5, 6.0 disable-internal-feedback
        void setGO(std::uint8_t GO) { value |= (static_cast<std::uint8_t>(GO&0x07) << 13); }
        std::uint8_t getGO() const { return ((value >> 13) & 0x07); }

        // Front-End PGA Mux Select: [GI4] -> 0 is [VIN1= VINN, VIN2= VINP], 1 is [VIN1= VINP, VIN2= VINN]
        void setMUX(std::uint8_t MUX) { value |= (static_cast<std::uint8_t>(MUX&0x01) << 12); }
        std::uint8_t getMUX() const { return ((value >> 12) & 0x01); }

        // Front-End PGA Gain Select: [GI3-GI0] -> from 0000b to 1101b: 4 6 8 12 16 32 64 100 200 400 480 600 800 960 1200 1600
        void setGI(std::uint8_t GI) { value |= (static_cast<std::uint8_t>(GI&0x0f) << 8); }
        std::uint8_t getGI() const { return ((value >> 8) & 0x0f); }

        // Coarse Offset Adjust on Front-End PGA: [OS7-OS0] where OS7 is sign (0 is +) and [OS6-OS0] is value. valid range is only x = [-100, +100]
        // which maps into (x/128)(Vref)(0.0256) for a global offset range of [-0.02*V_REF, +0.02*V_REF]. V_REF is in register CFG2.COSVR[1:0]
        void setOS(std::uint8_t OS) { value |= (static_cast<std::uint8_t>(OS&0xff)); }
        std::uint8_t getOS() const { return ((value) & 0xff); }

    };


    struct CFG1register
    {
        std::uint16_t value;
        static const std::uint16_t Default = 0x0000;

        // CFG1 Register: Configuration Register 1
        // bit:         15      14      13      12      11      10      09      08      07      06      05      04      03      02      01      00
        // content:     FLT-REF FLT-IPU OU-CFG  FLT-SEL CMP-SEL EXT-EN  INT-EN  EXT-POL INT-POL OU-EN   HL2     HL1     HL0     LL2     LL1     LL0

        CFG1register() { value = 0; }

        CFG1register(std::uint16_t v) : value(v) {}

        void reset() { value = 0; }

        void set(std::uint16_t cfg1) { value = cfg1; }

        void setDefault() { value = Default; }
    };

    struct CFG2register
    {
        std::uint16_t value;
        static const std::uint16_t Default = 0x0400;

        // CFG2 Register: Configuration Register 2
        // bit:         15      14      13      12      11      10      09      08      07      06      05      04      03      02      01      00
        // content:     OWD     OWD-OFF DIS-OUT NOW     COSVR1  COSVR0  RESERVD DOUTSEL DOUT    SD      RESERVD RESERVD RESERVD RESERVD RESERVD RESERVD
        // COSVR:       00 -> ; 01-> ; 10 -> ; 11 -> ;
        // with COSVR = 01b, as in our Default ... we have:
        // - Coarse Offset Range = [2.4, 3.6] V
        // - Coarse Offset Resolution = (1/128)(VREF)(0.0427)
        // - Coarse Offset Range = (±100mV)(VREF/3)
        CFG2register() { value = 0; }

        CFG2register(std::uint16_t v) : value(v) {}

        void reset() { value = 0; }

        void set(std::uint16_t cfg2) { value = cfg2; }

        void setDefault() { value = Default; }
    };


    struct SFTCregister
    {
        std::uint16_t value;
        static const std::uint16_t Default = 0x0050;    // where ... there is the SOFTWARE LOCK MODE (Runs from RAM) and the PGA308 operates from data written into the RAM registers from the user

        // SFTC Register: Software Control
        // bit:         15      14      13      12      11      10      09      08      07      06      05      04      03      02      01      00
        // content:     RESERVD RESERVD RESERVD RESERVD RESERVD RESERVD RESERVD CHKSFLG OW-DLY  SW-L2   SW-L1   SW-L0   RFB0    XP5     XP4     XP3
        // brief description:
        // CHKSFLG: Register Checksum Bit (Read-only)               1 = register checksum correct
        // OW-DLY:  One-Wire Delay Bit                              1 = 8-bit delay from transmit to receive during One-Wire reads, 0 = 1-bit delay from transmit to receive during One-Wire reads
        // SWL[2:0] Software Lock Mode Control                      101b = SOFTWARE LOCK (Runs from RAM), etc....
        // XP[5:3] OTP Bank Selection for Software Lock Mode        If XP[5:3] = '000', then the PGA308 operates from data written into the RAM registers from the user.
        SFTCregister() : value(0) {}

        SFTCregister(std::uint16_t v) : value(v) {}

        void reset() { value = 0; }

        void set(std::uint16_t sftc) { value = sftc; }

        void setDefault() { value = SFTCregister::Default; }
    };




    struct TransferFunctionConfig
    {

        static const std::uint16_t VREF = 8192;    // the allowed values are in range [0, VREF).  it must be constant ...

        std::uint16_t       GD;                     // gain DAC. values are [0.333333333, 0.999989824]
        std::uint8_t        GI          : 4;        // front end gain. from 0000b to 1101b: {4 6 8 12 16 32 64 100 200 400 480 600 800 960 1200 1600}
        std::uint8_t        muxsign     : 1;        // the sign: 0 is +, 1 is -
        std::uint8_t        GO          : 3;        // output gain. from 000b to 111b: {2.0, 2.4, 3, 3.6, 4.0, 4.5, 6.0, disable-internal-feedback}
        std::uint8_t        Vcoarseoffset;          // Vcoarseoffset is stored as sign+value and must be in range [-100, +100].
        std::uint16_t       Vzerodac;

        enum class Parameter { GD = 0, GI = 1, muxsign = 2, GO = 3, Vcoarseoffset = 4, Vzerodac = 5 };

        // the formulas are:
        //
        // from Vin to Vout
        // Vout = ((muxsign*Vin + Vcoarseoffset)*GI + Vzerodac)*GD*GO
        // Vout = alpha * Vin + beta
        // alpha = muxsign*GI*GD*GO
        // beta = (Vcoarseoffset*GI + Vzerodac)*GD*GO
        //
        // we use only three variables x, y, z and we keep the other fixed to default ...
        // x -> content of register GDAC (.GD)
        // y -> content of register CFG0.OS (.Vcoarseoffset)
        // z -> content of register ZDAC (.Vzerodac)
        // we thus have:
        // alpha(x) = valueOf(GI) * valueOf(GO) * ((1/3) + ((2/3)/64k) * x)
        // beta(x, y, z) = alpha(x) * ((1/128)*VREF*COSVR*y + (1/valueOf(GI))*VREF*(1/64k)(32k - z))
        // for some operations we need to represent Vout as a function only of y and z, hence
        // Vout = a * y + b * z + c
        // a = (d/dy) Vout = (d/dy) beta = alpha(x)*(1/128)*VREF*COSVR
        // b = (d/dz) Vout = (d/dz) beta = - alpha(x)*(1/valueOf(GI))*VREF*(1/64k)
        // c = Vout - a*y - b*z


        TransferFunctionConfig() : GD(0), GI(0), muxsign(0), GO(0), Vcoarseoffset(0), Vzerodac(0) {}

        void load(const Registers &regs)
        {
            GD = regs.GD;
            GI = regs.GI;
            muxsign = regs.S;
            GO = regs.GO;
            Vcoarseoffset = regs.Voffsetcoarse;
            Vzerodac = regs.Vzerodac;
        }

        void get(Registers &regs)
        {
            regs.GD = GD;
            regs.GI = GI;
            regs.S = muxsign;
            regs.GO = GO;
            regs.Voffsetcoarse = Vcoarseoffset;
            regs.Vzerodac = Vzerodac;
        }

        void setDefault()
        {
            GD = GDACregister::Default;
            GI = CFG0register::DefaultGI;
            muxsign = CFG0register::DefaultMUX;
            GO = CFG0register::DefaultGO;
            Vcoarseoffset = CFG0register::DefaultOS;
            Vzerodac = ZDACregister::Default;
        }

        // from internal memory to the values of the registers to be written into the amplifier
        void obtain(CFG0register &cfg0, ZDACregister &zdac, GDACregister &gdac) const
        {
            cfg0.reset(); cfg0.setGO(GO); cfg0.setMUX(muxsign); cfg0.setGI(GI); cfg0.setOS(Vcoarseoffset);
            zdac.reset(); zdac.value = Vzerodac;
            gdac.reset(); gdac.value = GD;
        }

        // from the registers read from the amplifier to internal memory.
        void assign(const CFG0register &cfg0, const ZDACregister &zdac, const GDACregister &gdac)
        {
            GO = cfg0.getGO();
            muxsign = cfg0.getMUX();
            GI = cfg0.getGI();
            Vcoarseoffset = cfg0.getOS();
            Vzerodac = zdac.value;
            GD = gdac.value;
        }

        void load(const CFG0register &cfg0)
        {
            GO = cfg0.getGO();
            muxsign = cfg0.getMUX();
            GI = cfg0.getGI();
            Vcoarseoffset = cfg0.getOS();
        }

        void load(const ZDACregister &zdac)
        {
            Vzerodac = zdac.value;
        }

        void load(const GDACregister &gdac)
        {
            GD = gdac.value;
        }

        float valueOfGI()
        {
            static const std::uint16_t mapGI2val[16] = {4, 6, 8, 12, 16, 32, 64, 100, 200, 400, 480, 600, 800, 960, 1200, 1600};
            return static_cast<float>(mapGI2val[GI]);
        }

        float valueOfGO()
        {
            static const float mapGO2val[8] = {2.0f, 2.4f, 3.0f, 3.6f, 4.0f, 4.5f, 6.0f, 1.0f};
            return mapGO2val[GO];
        }

        float valueOfGD()
        {
            //return 0.33333f + (static_cast<float>(GD) * 0.66666f) / 65536.0f;
            //return (1.0f + static_cast<float>(GD)/32768.0f)/3.0f;
            return valueOfGD(GD);
        }

        float valueOfGD(std::uint16_t x)
        {
            //return 0.33333f + (static_cast<float>(GD) * 0.66666f) / 65536.0f;
            return (1.0f + static_cast<float>(x)/32768.0f)/3.0f;
        }

        float valueOfCOR()
        {
            //static const float mapCOSVR2val[4] = {0.064f, 0.0427f, 0.0320f, 0.0256f};
            //std::uint8_t cosvr2 = 1;
            //return static_cast<float>(VREF)*(1.0f/128.0f)*mapCOSVR2val[cosvr2];
            return 2.7328f;
        }

        float regvco2value(std::uint8_t co)
        {   // regvco is inside [-100, +100] in sign-value format
            std::uint8_t v = co & 0x7f;
            if(v > 100) { v = 100; }
            std::uint8_t negative = co >> 7;
            return (1 == negative) ? -v : +v;
        }

        std::uint8_t value2regvco(float v)
        {   // regvco is inside [-100, +100] in sign-value format
            std:: uint8_t r = 0;
            if(v > 0)
            {
                v = std::floor(v + 0.5f);
                if(v > +100.0f) { v = +100.0f; }
                r = static_cast<std::uint8_t>(v);
            }
            else
            {
                v = -v;
                v = std::floor(v + 0.5f);
                if(v > +100.0f) { v = +100.0f; }
                r = static_cast<std::uint8_t>(v) | 0x80;
            }

            return r;
        }

        float valueOfCoarseOffset()
        {
            return valueOfCOR()*regvco2value(Vcoarseoffset);
        }

        float valueOfFineOffset()
        {
            return static_cast<float>(VREF)*(1.0f/65536.0f)*(32768.0f - static_cast<float>(Vzerodac));
            //return static_cast<float>(VREF)*(0.0000152587890625f)*(32768.0f - static_cast<float>(Vzerodac));
        }

        float alpha()
        {
            //float v = valueOfGD()*valueOfGO()*valueOfGI();
            //return (0 == muxsign) ? v : -v;
            return alpha(GD);
        }

        float alpha(std::uint16_t x)
        {
            float v = valueOfGD(x)*valueOfGO()*valueOfGI();
            return (0 == muxsign) ? v : -v;
        }

        float beta()
        {
            float gi = valueOfGI();
            return alpha()*(valueOfCoarseOffset() + valueOfFineOffset()/gi);
        }

        void computeOffsetParams(const std::uint16_t vout, float &a, float &b, float &c)
        {
            // a = (d/dy) Vout = (d/dy) beta = alpha(x)*(1/128)*VREF*COSVR = alpha(x) * COR
            // b = (d/dz) Vout = (d/dz) beta = - alpha(x)*(1/valueOf(GI))*VREF*(1/64k)
            // c = Vout - a*y - b*z
            a = alpha() * valueOfCOR();
            b = - (alpha()*static_cast<float>(VREF)/valueOfGI())/65536.0f; // valueOfGI() is always != 0
            c = static_cast<float>(vout) - a*regvco2value(Vcoarseoffset) - b*static_cast<float>(Vzerodac);
        }

        bool alignVOUT(const std::uint16_t vout, const std::uint16_t target, std::uint8_t &Y, std::uint16_t &Z)
        {
            float a, b, c;
            computeOffsetParams(vout, a, b, c);
            float y = (target - c - b*static_cast<float>(Vzerodac))/a;  // a is always != 0

            // round and limit y ... which must be integer and inside [-100, +100] and
            std::uint8_t tmp = value2regvco(y);
            y = regvco2value(tmp);

            // must apply new y to ...
            float z = (target - c - a*y)/b;
            if((z > 65535.0f) || (z < 0.0f))
            {
                return false;
            }
            Y = value2regvco(y);
            Z = static_cast<std::uint16_t>(std::floor(z + 0.5f));
            Vcoarseoffset = Y;
            Vzerodac = Z;

            return true;
        }

        bool setalpha(float a)
        {
            float xgd = 98304.0f * (a/(valueOfGI()*valueOfGO()) - (1.0f/3.0f));
            std::int32_t x = static_cast<std::int32_t>(std::floor(xgd + 0.5f));
            if((x >= 65536) || (x < 0))
            {
                return false;
            }
            GD = static_cast<std::uint16_t>(x);
            return true;
        }

        bool setbeta(float b)
        {
            float yco = b / (alpha() * valueOfCOR());
            std::uint8_t y = value2regvco(yco);
            // now we compute the fine adjustment
            float zco = ( regvco2value(y) * valueOfCOR() + 0.5f*static_cast<float>(VREF)/valueOfGI() - b/alpha() ) / ( (static_cast<float>(VREF)/65536.0f)/valueOfGI() );
            std::int32_t z = static_cast<std::int32_t>(std::floor(zco + 0.5f));
            if((z >= 65536) || (z < 0))
            {
                return false;
            }
            Vcoarseoffset = y;
            Vzerodac = static_cast<std::uint16_t>(z);

            return true;
        }

    };

    TransferFunctionConfig tsf;

#endif // #if defined(IMPORT_CODE_FROM_EMBOT_HW_PGA308)


    // data in here;
    Impl()
    {

    }
    ~Impl()
    {

    }


    bool load(const DiscreteGain g)
    {
        //LUCA
        // offset all equal to 32k-1
        static const uint8_t _cfgmap[static_cast<uint8_t>(DiscreteGain::maxnumberof)][6] =
        {
            {0x00, 0x40, 0x46, 0x1f, 0xb1, 0x7f},   // gain = 102
            {0x00, 0x40, 0x46, 0x1f, 0xb1, 0x7f},   // gain = 90
            {0x00, 0x40, 0x46, 0x1f, 0xb1, 0x7f},   // gain = 78
            {0x00, 0x40, 0x46, 0x1f, 0xb1, 0x7f},   // gain = 56
            {0x00, 0x40, 0x46, 0x1f, 0xb1, 0x7f},   // gain = 48
            {0x00, 0x10, 0x46, 0x2a, 0x80, 0x80},   // gain = 36
            {0x00, 0x40, 0x42, 0x3e, 0x62, 0x7f},   // gain = 24 [or 0x26 instead of 0x42]
            {0x00, 0x20, 0x42, 0x4b, 0x15, 0x80},   // gain = 20 [or 0x26 instead of 0x42]
            {0x00, 0x00, 0x42, 0x5e, 0x72, 0x80},   // gain = 16 [or 0x26 instead of 0x42]
            {0x00, 0xC0, 0x02, 0x64, 0xf6, 0x6e},   // gain = 10 [or 0x10 instead of 0x02]
            {0x00, 0x80, 0x02, 0x64, 0x29, 0x62},   // gain = 08 [or 0x10 instead of 0x02]
            {0x00, 0x40, 0x02, 0x64, 0xd4, 0x4c},   // gain = 06 [or 0x10 instead of 0x02]
            {0x00, 0x40, 0x00, 0x64, 0x29, 0x22}    // gain = 04
        };

        bool ret = true;
        switch(g)
        {
            case DiscreteGain::none:
            case DiscreteGain::maxnumberof:
            {
                ret = false;
            } break;

            default:
            {
                uint8_t index = static_cast<uint8_t>(g);
                Registers cfg;
                cfg.load(_cfgmap[index], 6);
                tsf.load(cfg);
            } break;

        }

        return ret;
    }

    bool load_step1(const DiscreteGain g)
    {
        //LUCA
        // offset are not meaningful yet. we just need zdac to be 0x8000 and cfg0.os =  ... boh: 0x20
        static const uint8_t _cfgmap[static_cast<uint8_t>(DiscreteGain::maxnumberof)][6] =
        {
            {0x00, 0x40, 0x46, 0x1f, 0xb1, 0x7f},   // gain = 102
            {0x00, 0x40, 0x46, 0x1f, 0xb1, 0x7f},   // gain = 90
            {0x00, 0x40, 0x46, 0x1f, 0xb1, 0x7f},   // gain = 78
            {0x00, 0x40, 0x46, 0x1f, 0xb1, 0x7f},   // gain = 56            
            {0x00, 0x40, 0x46, 0x20, 0x00, 0x80},   // gain = 48
            {0x00, 0x10, 0x46, 0x20, 0x00, 0x80},   // gain = 36
            {0x00, 0x40, 0x42, 0x20, 0x00, 0x80},   // gain = 24 [or 0x26 instead of 0x42]
            {0x00, 0x20, 0x42, 0x20, 0x00, 0x80},   // gain = 20 [or 0x26 instead of 0x42]
            {0x00, 0x00, 0x42, 0x20, 0x00, 0x80},   // gain = 16 [or 0x26 instead of 0x42]
            {0x00, 0xC0, 0x02, 0x20, 0x00, 0x80},   // gain = 10 [or 0x10 instead of 0x02]
            {0x00, 0x80, 0x02, 0x20, 0x00, 0x80},   // gain = 08 [or 0x10 instead of 0x02]
            {0x00, 0x40, 0x02, 0x20, 0x00, 0x80},   // gain = 06 [or 0x10 instead of 0x02]
            {0x00, 0x40, 0x00, 0x20, 0x00, 0x80}    // gain = 04
        };

        bool ret = true;
        switch(g)
        {
            case DiscreteGain::none:
            case DiscreteGain::maxnumberof:
            {
                ret = false;
            } break;

            default:
            {
                uint8_t index = static_cast<uint8_t>(g);
                Registers cfg;
                cfg.load(_cfgmap[index], 6);
                tsf.load(cfg);
            } break;

        }

        return ret;
    }

    bool load(const DiscreteGain g, const Offset offset)
    {
        // there are some compinations of g+offset which cannot reached.
        // if g is 6 and o8k > 7780 (ofsfet =
        // if g is 4 and o8k > 5100 (offset =
        if(((g == DiscreteGain::val06) && (offset > (62240))) || ((g == DiscreteGain::val04) && (offset > (40800))))
        {
            return false;
        }

        // step1: we load the gain, so that we surely have some registers which are already ok.
        //        also we wnat zdac to be 0x8000
        if(false == load_step1(g))
        {
            return false;
        }

        bool ret = true;

        float o8k = offset / 8;

        // step2: we keep the values of the gain(s), and if ZDAC = 0x8000 we can compute cfg0.os in a simple mode:
        // cfg0.os = o8k/(alpha()*valueOfCOR())
        float cfg0os = o8k/(tsf.alpha()*tsf.valueOfCOR());

        // however, we must be sure that what we load is in range [-100, =100] and is in format sign-value.
        int8_t Y = tsf.value2regvco(cfg0os);
        // ok, we assign it.
        tsf.Vcoarseoffset = Y;

        // but now we must compute the zdac with formula:

        // zdac = (64*1024)/(VREF) * ( (VREF/2) - (o8k - alpha() * valueOfCoarseOffset())/(g0*gd) )

        float z = ((64.0f*1024.0f)/tsf.VREF) * ( (tsf.VREF/2) - (o8k - tsf.alpha()*tsf.valueOfCoarseOffset())/(tsf.valueOfGO()*tsf.valueOfGD()) );

        if((z > 65535.0f) || (z < 0.0f))
        {
            return false;
        }

        uint16_t Z = static_cast<std::uint16_t>(std::floor(z + 0.5f));
        tsf.Vzerodac = Z;

        return true;
    }

    bool get(Gain &gain, Offset &offset)
    {
        // this funtion extends / uses the struct TransferFunctionConfig
        gain = tsf.alpha();
        int32_t tmp = static_cast<int32_t>(round(8.0f*tsf.beta()));
        if((tmp >= 0) && (tmp < 65536))
        {
            offset = static_cast<uint16_t>(tmp);
            return true;
        }
        return false;
    }


};


// --------------------------------------------------------------------------------------------------------------------
// - all the rest
// --------------------------------------------------------------------------------------------------------------------




namespace strain { namespace amplifier {

    //LUCA
    static const float mapofgains[static_cast<uint8_t>(DiscreteGain::maxnumberof)] =
    {
        102,90,78,56,48, 36, 24, 20, 16, 10, 8, 6, 4
    };

    Gain convert(const DiscreteGain dg)
    {
        uint8_t index = static_cast<uint8_t>(dg);
        if(index >= static_cast<uint8_t>(DiscreteGain::maxnumberof))
        {
            return 0;
        }
        return mapofgains[index];
    }

    bool convert(const Gain g, DiscreteGain &dg)
    {
        bool ret = false;
        dg = DiscreteGain::none;

        // if g one of the allowed? i check by iteration
        for(int i=0; i<static_cast<uint8_t>(DiscreteGain::maxnumberof); i++)
        {
            if(g == mapofgains[i])
            {
                dg = static_cast<DiscreteGain>(i);
                ret = true;
                break;
            }
        }

        return ret;
    }

    bool convert(const WideParams &wp, DiscreteParams &dp)
    {
        DiscreteGain dg = DiscreteGain::none;
        if(false == convert(wp.g, dg))
        {
            return false;
        }
        dp.dg = dg;
        dp.o = wp.o;

        return true;
    }


    const std::uint8_t PGA308::Registers::defval[PGA308::Registers::sizeofregisters] = {0x00, 0x40, 0x46, 0x1f, 0xb1, 0x7f}; // gain = 48, offset = midrangeOffset

    bool PGA308::Registers::load(const void *data, const size_t size)
    {
        if((nullptr == data) || (size != sizeofregisters))
        {
            GD = GI = S = GO = Voffsetcoarse = Vzerodac = 0;
            return false;
        }
        const uint16_t *u16 = reinterpret_cast<const uint16_t*>(data);
        const uint8_t *u08 = reinterpret_cast<const uint8_t*>(data);
        GD = u16[0]; Vzerodac = u16[2];
        GI = (u08[2] >> 4) & 0x0f;
        S  = (u08[2] >> 3) & 0x01;
        GO = (u08[2]) & 0x07;
        Voffsetcoarse = u08[3];
        return true;
    }

    bool PGA308::Registers::fill(void *data, size_t &size)
    {
        if(nullptr == data)
        {
            return false;
        }
        uint16_t *u16 = reinterpret_cast<uint16_t*>(data);
        uint8_t *u08 = reinterpret_cast<uint8_t*>(data);
        u16[0] = GD; u16[2] = Vzerodac;
        u08[2] = 0;
        u08[2] = ((static_cast<uint8_t>(GI) & 0x0f) << 4) | ((static_cast<uint8_t>(S) & 0x01) << 3) | ((static_cast<uint8_t>(GO) & 0x07));
        u08[3] = Voffsetcoarse;
        size = sizeofregisters;
        return true;
    }


    //        struct basicRegs
    //        {
    //            std::uint16_t       GD;                 // it is a gain register
    //            std::uint8_t        GI          : 4;    // it is a gain register
    //            std::uint8_t        S           : 1;    // it is a sign of gain register (keep it always 0)
    //            std::uint8_t        GO          : 3;    // it is a gain register
    //            std::uint8_t        Voffsetcoarse;      // it is an offset register
    //            std::uint16_t       Vzerodac;           // it is an offset register

    //            basicRegs() : GD(0), GI(0), S(0), GO(0), Voffsetcoarse(0), Vzerodac(0) {}
    //            basicRegs(void *data, size_t s) { load(data, s); }
    //            void load(const void *data, const size_t s);    // import from a can frame
    //            bool fill(void *data, size_t &s);               // export to a can frame
    //        };

//    void PGA308::basicRegs::load(const void *data, const size_t s)
//    {
//        if(nullptr == data) { GD = GI = S = GO = Voffsetcoarse = Vzerodac = 0; return; }
//        const uint16_t *u16 = reinterpret_cast<const uint16_t*>(data);
//        const uint8_t *u08 = reinterpret_cast<const uint8_t*>(data);
//        GD = u16[0]; Vzerodac = u16[2];
//        GI = (u08[2] >> 4) & 0x0f;
//        S  = (u08[2] >> 3) & 0x01;
//        GO = (u08[2]) & 0x07;
//        Voffsetcoarse = u08[3];
//    }

//    bool PGA308::basicRegs::fill(void *data, size_t &s)
//    {
//        if(nullptr == data)
//        {
//            return false;
//        }
//        uint16_t *u16 = reinterpret_cast<uint16_t*>(data);
//        uint8_t *u08 = reinterpret_cast<uint8_t*>(data);
//        u16[0] = GD; u16[2] = Vzerodac;
//        u08[2] = 0;
//        u08[2] = ((static_cast<uint8_t>(GI) & 0x0f) << 4) | ((static_cast<uint8_t>(S) & 0x01) << 3) | ((static_cast<uint8_t>(GO) & 0x07));
//        u08[3] = Voffsetcoarse;
//        s = 6;
//        return true;
//    }



    PGA308::PGA308()
    : pImpl(new Impl)
    {

    }

    PGA308::~PGA308()
    {
        delete pImpl;
    }

    bool PGA308::import(const Registers &regs, WideParams *wideparams)
    {
       load(regs);

       if(nullptr != wideparams)
       {
           get(wideparams->g, wideparams->o);
       }

       return true;
    }

    bool PGA308::import(const DiscreteParams &params, Registers *regs)
    {
        load(params);

        if(nullptr != regs)
        {
            get(*regs);
        }

        return true;
    }



    bool PGA308::load(const Registers &regs)
    {
        pImpl->tsf.load(regs);
        return true;
    }

    // attempts to load into the PGA308 a pair discretegain-offset, which produce a given Registers.
    bool PGA308::load(const DiscreteGain g, const Offset offset)
    {
        if(midrangeOffset == offset)
        {
            return pImpl->load(g);
        }

        return pImpl->load(g, offset);
    }

    bool PGA308::load(const DiscreteParams &discreteparams)
    {
        return load(discreteparams.dg, discreteparams.o);
    }

    // retrieve the pair gain-offset which is effectively in use after a load operation
    bool PGA308::get(Gain &gain, Offset &offset)
    {
        return pImpl->get(gain, offset);
    }

    bool PGA308::get(WideParams &wideparams)
    {
        return get(wideparams.g, wideparams.o);
    }

//    bool PGA308::get(Params &params)
//    {
//        return get(params.g, params.o);
//    }

    // retrieve the registers effectively in use.
    bool PGA308::get(Registers &regs)
    {
        pImpl->tsf.get(regs);
        return true;
    }



}} // namespace strain { namespace amplifier {


    
// --------------------------------------------------------------------------------------------------------------------
// - test code
// --------------------------------------------------------------------------------------------------------------------

namespace strain { namespace amplifier {

    // example of use.
    // we must have an array of 6 PGA308 objects
    // in the widget of the firmwareupdater:strainwindow we retrieve for each channel of the strain2 two values: a gain and an offset.
    // we must have a

    struct Example_strain2_ampl_regs_t
    {
       uint8_t data[6];
       Example_strain2_ampl_regs_t() { data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = 0; }
       void load(void *mem) { memmove(data, mem, sizeof(data)); }
       void * memory() { return data; }
       size_t size() { return 6; }
    };


    void testIT(std::string &output)
    {

        const bool step1 = true;

        const bool step2 = true;

        // we use service object pga just to convert from and to.
        PGA308 pga;

        Example_strain2_ampl_regs_t regstoTX;

        if(true == step1)
        {
            // in here we get two values of gain+offset (e.g., from the gui), we load int the pga and we retrieve the registers to send to the board
            WideParams wp;
            DiscreteParams dp;
            PGA308::Registers regs;


            wp.load(4.0, 32*1024);
            if(true == convert(wp, dp))
            {
                // load dp and ...
                //PGA308::Registers regs;
                pga.import(dp, &regs);
                // use the regs for instance by sending to a strain2_ampl_regs_t which can be initted from a void *
                //Example_strain2_ampl_regs_t regstoTX;
                size_t s;
                regs.fill(regstoTX.memory(), s);
                // now call
                //cDownloader::strain_set_amplifier_regs(bus, id, regstoTX, strain_regset_inuse);
            }

            //wp.load(48.0, 32*1024);
            wp.load(48.0, 32531);
            if(true == convert(wp, dp))
            {
                // load dp and ...
                //PGA308::Registers regs;
                pga.import(dp, &regs);
                // use the regs for instance by sending to a strain2_ampl_regs_t which can be initted from a void *
                //Example_strain2_ampl_regs_t regstoTX;
                size_t s;
                regs.fill(regstoTX.memory(), s);
                // now call
                //cDownloader::strain_set_amplifier_regs(bus, id, regstoTX, strain_regset_inuse);
            }
            output += "step1: ";
            output += "done!";

        }

        Example_strain2_ampl_regs_t RXregs;

        if(true == step2)
        {
            // in here we retrieve a register. and we compute the associated gain-offset

            RXregs = regstoTX;

            //cDownloader::strain_get_amplifier_regs(bus, id, RXregs, strain_regset_inuse);

            PGA308::Registers regs;
            regs.load(RXregs.memory(), RXregs.size());

            // now regs has the content of what we have received from can bus
            WideParams wparams;
            pga.import(regs, &wparams);
            // now we see if the wparams.g is one of the allowed
            DiscreteParams dparams;
            if(true == convert(wparams, dparams))
            {
                // use dparams
                Gain gg = convert(dparams.dg);
                Offset oo = dparams.o;
            }
        }

    }

    void exampleofuse(void)
    {

#if 1

        // we use service object pga just to convert from and to.
        PGA308 pga;

        // in here we get two values of gain+offset (e.g., from the gui), we load int the pga and we retrieve the registers to send to the board
        WideParams wp;
        wp.load(48.0, 32*1024);
        DiscreteParams dp;
        if(true == convert(wp, dp))
        {
            // load dp and ...
            PGA308::Registers regs;
            pga.import(dp, &regs);
            // use the regs for instance by sending to a strain2_ampl_regs_t which can be initted from a void *
            Example_strain2_ampl_regs_t regstoTX;
            size_t s;
            regs.fill(regstoTX.memory(), s);
            // now call
            //cDownloader::strain_set_amplifier_regs(bus, id, regstoTX, strain_regset_inuse);
        }

        // in here we retrieve a register. and we compute the associated gain-offset

        Example_strain2_ampl_regs_t RXregs;

        //cDownloader::strain_get_amplifier_regs(bus, id, RXregs, strain_regset_inuse);

        PGA308::Registers regs;
        regs.load(RXregs.memory(), RXregs.size());

        // now regs has the content of what we have received from can bus
        WideParams wparams;
        pga.import(regs, &wparams);
        // now we see if the wparams.g is one of the allowed
        DiscreteParams dparams;
        if(true == convert(wparams, dparams))
        {
            // use dparams
            Gain gg = convert(dp.dg);
            Offset oo = dp.o;
        }



#else
        // we have one for each channel
        PGA308 thePGAs[6];

        // channel 0:
        DiscreteParams discreteparams;
        // get the value from teh widget
        discreteparams.load(DiscreteGain::val10, 32000);
        // load them into the object
        thePGAs[0].load(discreteparams);
        // now i want to send the packet to the board. i retrieve teh registers .
        PGA308::Registers regs;
        thePGAs[0].get(regs);
        // and now i transform them into a payload for can tx
        uint8_t payload[8] = {0};
        regs.fill(payload);
        // and now i use payload to tx.

        // now i ask the registers to the board. i receive a payload
        uint8_t rxpayload[8] = {0}; // ok, not zero but ...
        regs.load(rxpayload);
        // and i want to fill the widget with the real values
        PGA308::Registers rxregs;
        rxregs.load(rxpayload);
        float gg =0;
        uint16_t oo =0;
        thePGAs[0].load(rxregs);
        thePGAs[0].get(gg, oo);
        // print gg, oo
#endif

    }


}} // namespace strain { namespace amplifier



namespace strain { namespace dsp { namespace fsc {

    FSC convert(const double v, bool &saturated)
    {
        saturated = false;
        FSC ret = static_cast<FSC>(v);

        if(v>fsc::max)
        {
           saturated = true;
           ret = fsc::max;
        }

        if(v<fsc::min)
        {
           saturated = true;
           ret = fsc::min;
        }

        return ret;
    }

    double convert(const FSC v)
    {
        return static_cast<double>(v);
    }


    bool convert(const std::vector<double> &in, std::vector<FSC> &out)
    {
        bool sat = false;
        bool ret = true;
        out.resize(0);
        for(int i=0; i<in.size(); i++)
        {
            out.push_back(strain::dsp::fsc::convert(in[i], sat));
            if(sat)
            {
                ret = false;
            }
        }
        return ret;
    }

    bool convert(const std::vector<FSC> &in, std::vector<double> &out)
    {
        bool ret = true;
        out.resize(0);
        for(int i=0; i<in.size(); i++)
        {
            out.push_back(strain::dsp::fsc::convert(in[i]));
        }
        return ret;
    }


} } } // namespace strain { namespace dsp { namespace fsc {



namespace strain { namespace dsp { namespace q15 {

    Q15 convert(const double v, bool &saturated)
    {
        Q15result rr = static_cast<Q15result>(v * 32768.0);    // note: 32768.0 = 2^15
        return saturate(rr, saturated);
    }

    Q15 U16toQ15(const std::uint16_t valU16)
    {
        return valU16-0x8000;
    }

    bool convert(const std::vector<double> &in, std::vector<Q15> &out)
    {
        bool sat = false;
        bool ret = true;
        out.resize(0);
        for(int i=0; i<in.size(); i++)
        {
            out.push_back(convert(in[i], sat));
            if(sat)
            {
                ret = false;
            }
        }
        return ret;
    }

    bool convert(const std::vector<Q15> &in, std::vector<double> &out)
    {
        bool ret = true;
        out.resize(0);
        for(int i=0; i<in.size(); i++)
        {
            out.push_back(convert(in[i]));
        }
        return ret;
    }

    std::uint16_t Q15toU16(const Q15 valQ15)
    {
        return valQ15+0x8000;
    }

    double convert(const Q15 v)
    {
        return static_cast<double>(v) / 32768.0;  // note: 32768.0 = 2^15
    }

    Q15 opposite(const Q15 v)
    {
        if(negOne == v)
        {
            return posOneNearly;
        }
        return -v;
    }

    Q15 saturate(const Q15result x, bool &saturated)
    {
        if (x > 0x7FFF)
        {
            saturated = true;
            return 0x7FFF;
        }
        else if (x < -0x8000)
        {
            saturated = true;
            return -0x8000;
        }

        saturated = false;
        return static_cast<Q15>(x);
    }

    Q15 add(const Q15 a, const Q15 b)
    {
        bool saturated = false;
        return add(a, b, saturated);
    }

    Q15 add(const Q15 a, const Q15 b, bool &saturated)
    {
        Q15result tmp = static_cast<Q15result>(a) + static_cast<Q15result>(b);

        return saturate(tmp, saturated);
    }

    Q15 sub(Q15 &a, Q15 &b)
    {
        return a-b;
    }

    Q15 mul(const Q15 a, const Q15 b, bool &saturated)
    {
        Q15result tmp = static_cast<Q15result>(a) * static_cast<Q15result>(b);

        // Rounding; mid values are rounded up
        tmp += (1 << (15 - 1));
        // Correct by dividing by base
        tmp >>= 15;
        // and saturate result
        return saturate(tmp, saturated);
    }

    Q15 mul(const Q15 a, const Q15 b)
    {
        bool saturated = false;
        return mul(a, b, saturated);
    }

    Q15 div(const Q15 a, const Q15 b, bool &saturated)
    {
        Q15result n1 = static_cast<Q15result>(a);
        Q15result n2 = static_cast<Q15result>(b);
        Q15result rr = 0;
        std::uint8_t neg = 0;

        if(0 == n2)
        {
            saturated = true;
            return 0;
        }

        if(n1 < 0)
        {
            neg = !neg;
            n1 = -n1;
        }

        if(n2 < 0)
        {
            neg = !neg;
            n2 = -n2;
        }

        rr = (n1 << 15) / n2;

        if(0 != neg)
        {
            rr = - rr;
        }

        return saturate(rr, saturated);
    }

    bool add(const matrix &m1, const matrix &m2, matrix &res, bool &saturated)
    {
        if((m1.ncols != m2.ncols) || (m1.ncols != res.ncols))
        {
            return false;
        }
        if((m1.nrows != m2.nrows) || (m1.nrows != res.nrows))
        {
            return false;
        }

        saturated =  false;

        // ok, we do action
        for(int r=0; r<res.nrows; r++)
        {
            for(int c=0; c<res.ncols; c++)
            {
                bool sat1 = false;
                res.data[r*res.ncols+c] = add(m1.data[r*m1.ncols+c], m2.data[r*m2.ncols+c], sat1);

                if(sat1)
                {
                    saturated = true;
                }
            }
        }

        return true;
    }

    bool multiply(const matrix &m1, const matrix &m2, matrix &res, bool &saturated)
    {
        if(m1.ncols != m2.nrows)
        {
            return false;
        }

        if(m1.nrows != res.nrows)
        {
            return false;
        }

        if(m2.ncols != res.ncols)
        {
            return false;
        }

        saturated =  false;
        std::uint32_t nsat = 0;

        // ok, we do action
        for(int r=0; r<res.nrows; r++)
        {
            for(int c=0; c<res.ncols; c++)
            {
                Q15 v = 0;

                const Q15 *row_wise = &m1.data[r*m1.ncols]; // navigo con +1
                const Q15 *col_wise = &m2.data[c];          // navigo con +(m2.ncols)
                for(int i=0; i<m1.ncols; i++)
                {
                    bool sat1 = false;
                    bool sat2 = false;
                    Q15 x = mul(*row_wise, *col_wise, sat1);
                    v = add(v, x, sat2);
                    if(sat1 || sat2)
                    {
                        nsat ++;
                        saturated = true;
                    }
                    row_wise += 1;
                    col_wise += m2.ncols;
                }

                res.data[c + r*res.ncols] = v;
            }
        }

        return true;
    }

} } } // namespace strain { namespace dsp { namespace q15 {

namespace strain { namespace regulation {

    bool read(const std::string fname, FullRegulation &reg)
    {
        bool ret = false;
#if 0
        // to be done later on

        reg.clear();

        if(fname.empty())
        {
//            yError("File not found!\n");
//            appendLogMsg("File not found!");
            return false;
        }

//        if (selected_id <1 || selected_id >= 15){
//            yError("Invalid board address!\n");
//            appendLogMsg("Invalid board address!");
//            return false;
//        }

        const char * filename = fname.c_str();
        int file_version=0;
        std::fstream filestr;
        filestr.open (filename, fstream::in);
        if (!filestr.is_open()){
//            yError("Error opening calibration file!\n");
//            appendLogMsg("Error opening calibration file!");
            return false;
        }

        int i=0;
        char buffer[256];

        //file version
        filestr.getline (buffer,256);
        filestr.getline (buffer,256);
        sscanf (buffer,"%d",&file_version);


        if((4 != file_version) && (3 != file_version))
        {
//            yError("Wrong file. Calibration version not supported for strain2: %d\n", file_version);
//            appendLogMsg("Wrong file. Calibration version not supported for strain2");
            return false;
        }

        reg.version = static_cast<Version>(file_version);

        if(Version::three == reg.version)
        {
            // Board type:
            filestr.getline (buffer,256);
            filestr.getline (buffer,256);
            if(0 != strcmp(buffer, "strain2"))
            {
//                yError("Wrong file. Board type not supported: %s\n", buffer);
//                appendLogMsg("Wrong file. Board type not supported");
                return false;
            }

            reg.board = Board::strain2;

            // Serial number:
            char serial_no[256] = {0};
            filestr.getline (buffer,256);
            filestr.getline (buffer,256);
            snprintf(serial_no, sizeof(serial_no), "%s", buffer);
            //core->getDownloader()->strain_set_serial_number(bus,id, serial_no);

            reg.serial = std::string(serial_no);

            // there is only one regulation set

            Set set;
            set.clear();

            // Amplifier registers:
            filestr.getline (buffer,256);
            for (i=0;i<6; i++)
            {
                filestr.getline (buffer,256);
//                yDebug() << buffer;
                unsigned int t08[6] = {0};
                sscanf  (buffer,"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x", &t08[0], &t08[1], &t08[2], &t08[3], &t08[4], &t08[5]);
                for(int j=0; j<6; j++)
                {
                    set.analog.amplregs[i][j] = t08[j];
                }

//                core->getDownloader()->strain_set_amplifier_regs(core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, i, amp_registers[i], regset);

                // downloader.strain_set_offset (downloader.board_list[selected].bus, downloader.board_list[selected].pid, i, offset[i]);
                //core->getDownloader()->strain_set_offset (bus,id, i, offset[i]);
                //printf("0X%02x, 0X%02x, 0X%02x, 0X%02x, 0X%02x,0X%02x", amp_registers[i].data[0], amp_registers[i].data[1], amp_registers[i].data[2], amp_registers[i].data[3], amp_registers[i].data[4], amp_registers[i].data[5]);
                //fflush(stdout);
//                drv_sleep(10);
            }



            //calibration matrix
            filestr.getline (buffer,256);
            for (i=0;i<36; i++){
                unsigned int tmp = 0;
                filestr.getline (buffer,256);
                sscanf (buffer,"%x", &tmp);
                set.digital.matrix[i] = static_cast<strain::dsp::Q15>(tmp);
//                core->getDownloader()->strain_set_matrix_rc(bus,id, ri, ci, calib_matrix[index][ri][ci], regset);
            }



            //matrix gain
            filestr.getline (buffer,256);
            filestr.getline (buffer,256);
            int cc=0;
            sscanf (buffer,"%d",&cc);
//            core->getDownloader()->strain_set_matrix_gain(bus,id, cc, regset);

            //tare
            filestr.getline (buffer,256);
            for (i=0;i<6; i++){
                filestr.getline (buffer,256);
                int tt = 0;
                sscanf  (buffer,"%d", &tt);
                set.digital.tare[i] = static_cast<std::uint16_t>(tt);
//                core->getDownloader()->strain_set_calib_bias(bus,id, i, calib_bias[i], regset);
            }

            //full scale values
            filestr.getline (buffer,256);
            for (i=0;i<6; i++){
                filestr.getline (buffer,256);
                int fs = 0;
                sscanf  (buffer,"%d", &fs);
                set.digital.fullscale[i] = static_cast<strain::dsp::FSC>(fs);
//                core->getDownloader()->strain_set_full_scale(bus,id, i, full_scale_const[index][i], regset);
            }

            reg.sets.push_back(set);

        }

        reg.set2useatbootstrap = 1;


        filestr.close();
        filestr.clear();

#endif

        return ret;
    }

} } // namespace strain { namespace regulation


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

