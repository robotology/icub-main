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

// - include guard ----------------------------------------------------------------------------------------------------

#ifndef _STRAIN_H_
#define _STRAIN_H_

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace strain {

    enum class Board { strain1 = 1, strain2 = 2 };

    const std::uint8_t numberofchannels = 6;

}   // namespace strain {

namespace strain { namespace dsp {

    using FSC = std::uint16_t;
    using Q15 = std::int16_t;
    using Q15result = std::int32_t;

} }  // namespace strain { namespace dsp {

namespace strain { namespace dsp { namespace fsc {

    const FSC max = 64*1024-1;
    const FSC min = 0;

    FSC convert(const double v, bool &saturated);
    double convert(const FSC v);

    bool convert(const std::vector<double> &in, std::vector<FSC> &out);
    bool convert(const std::vector<FSC> &in, std::vector<double> &out);

} } }  // namespace strain { namespace dsp { namespace fsc {

namespace strain { namespace dsp { namespace q15 {

    const Q15 negOne = 0x8000;
    const Q15 negOneNearly = 0x8001;    // -1+2^(-15) = -0.999969482421875
    const Q15 negOneHalf = 0xC000;      // -1+2^(-1) = -1.00+0.50 = -0.50
    const Q15 negOneFourth = 0xE000;    // -1+2^(-1)+2^(-2) = -1.00+0.50+0.25 = -0.25
    const Q15 negOneEigth = 0xF000;     // -1+2^(-1)+2^(-2)+2^(-3) = -1.00+0.50+0.25+0.125 = -0.125
    const Q15 negEPSILON = 0xFFFF;      // -1+sum_(i=-1,..,-15)(2^i) = -0.000030517578125
    const Q15 zero = 0;
    const Q15 posEPSILON = 0x0001;      // 2^(-15) = 0.000030517578125
    const Q15 posOneHalf = 0x4000;      // 2^(-1) = 0.5
    const Q15 posOneFourth = 0x2000;    // 2^(-2) = 0.25
    const Q15 posOneEigth = 0x4000;     // 2^(-3) = 0.125
    const Q15 posOneNearly = 0x7FFF;    // sum_(i=-1,..,-15)(2^i) = 1-2^(-15) = 0.999969482421875

    Q15 convert(const double v, bool &saturated);
    double convert(const Q15 v);

    bool convert(const std::vector<double> &in, std::vector<Q15> &out);
    bool convert(const std::vector<Q15> &in, std::vector<double> &out);

    Q15 U16toQ15(const std::uint16_t valU16); // transforms a value in range [0, 64k-1] into a Q15
    std::uint16_t Q15toU16(const Q15 valQ15); // transforms a Q15 into range [0, 64k-1]

    Q15 opposite(const Q15 v);  // opposite of negOne is posOneNearly
    Q15 saturate(const Q15result x, bool &saturated);
    Q15 add(const Q15 a, const Q15 b);
    Q15 add(const Q15 a, const Q15 b, bool &saturated);
    Q15 sub(const Q15 a, const Q15 b);
    Q15 mul(const Q15 a, const Q15 b);
    Q15 mul(const Q15 a, const Q15 b, bool &saturated);
    Q15 div(const Q15 a, const Q15 b, bool &saturated);


    struct matrix
    {
        std::uint8_t    nrows;
        std::uint8_t    ncols;
        Q15*            data;   // organised by row

        void load(std::uint8_t r, std::uint8_t c, Q15* d) { nrows = r; ncols = c; data = d; }
        matrix() { load(0, 0, nullptr); }
        matrix(std::uint8_t r, std::uint8_t c, Q15* d) { load(r, c, d); }
        Q15 get(std::uint8_t r, std::uint8_t c) { if((r<nrows) && (c<ncols) && (nullptr != data)) { return data[c + r*ncols]; } else { return 0; } }
        void set(std::uint8_t r, std::uint8_t c, Q15 v) { if((r<nrows) && (c<ncols) && (nullptr != data)) { data[c + r*ncols] = v; }  }
        void clear() {  if(nullptr != data) { std::memset(data, 0, sizeof(Q15)*ncols*nrows); }  }
        void diagonal(Q15 v) { clear(); if(nullptr != data) { std::uint8_t min = (ncols<nrows) ? (ncols) : (nrows); for(int i=0; i<min; i++) data[i+i*ncols] = v; } }
        void fill(Q15 v) { clear(); if(nullptr != data) { for(int r=0; r<nrows; r++) for(int c=0; c<nrows; c++) data[c+r*ncols] = v; } }
    };

    bool multiply(const matrix &m1, const matrix &m2, matrix &res, bool &saturated);
    bool add(const matrix &m1, const matrix &m2, matrix &res, bool &saturated);

} } }  // namespace strain { namespace dsp { namespace q15 {



namespace strain { namespace amplifier {

    // according to formula: Vout = gain * Vin + offset
    // gain is a float, and offset is a positive integer in range [0, 64k)
    using Gain = float;
    using Offset = std::uint16_t;

    // but gain cannot be any real number because it is given by a limited combination of registers.
    // the exact definition of the possible values is possible but very complicated, hence we decide
    // to simplify and allow only a limited number of possible values whcih have shown to be useful.
    // they are the DiscreteGain
    enum class DiscreteGain { val48 = 0, val36 = 1, val24 = 2, val20 = 3, val16 = 4, val10 = 5, val08 = 6, val06 = 7,  val04 = 8, none = 32, maxnumberof = 9 };

    // with this we convert from the enum to the real value (should you use it in some debug message)
    Gain convert(DiscreteGain dg);

    // with this we convert a float gain into a discrete one. but only if the float gain is exactly equal.
    // as an example g = 48.0f will be succesfully converted to dg = DiscreteGain::val48 and teh funtion will return true,
    // but g = 47.0f will cause funtion to return false and dg = DiscreteGain::none
    bool convert(const Gain g, DiscreteGain &dg);

    // we have some special offsets: the minimum, the midrange, the maximum
    const Offset minimumOffset = 0;
    const Offset midrangeOffset = 32*1024-1;
    const Offset maximumOffset = 64*1024-1;

    // we group in here the discrete gain + offset
    struct DiscreteParams
    {
        DiscreteGain    dg;
        Offset          o;
        DiscreteParams() : dg(DiscreteGain::none), o(minimumOffset) {}
        void load(DiscreteGain _dg, Offset _o) { dg = _dg; o = _o; }
    };

    // we group in here the float gain + offset
    struct WideParams
    {
        Gain            g;
        Offset          o;
        WideParams() : g(1.0f), o(minimumOffset) {}
        void load(Gain _g, Offset _o) { g = _g; o = _o; }
        void load(DiscreteGain _dg, Offset _o) { g = convert(_dg); o = _o; }
        void load(const DiscreteParams &_dp) { g = convert(_dp.dg); o = _dp.o; }
    };

    // and in here we can convert from wide to discrete. it returns true only if conversion of Gain to DiscreteGain is possible
    bool convert(const WideParams &wp, DiscreteParams &dp);

    // in here we define a virtual interface for the registers
    class IFregs
    {
    public:
        virtual ~IFregs() {}
        virtual bool load(const void *data, const size_t size) = 0;     // import from a stream (e.g., a can frame)
        virtual bool fill(void *data, size_t &size) = 0;                 // export to a stream (e.g., a can frame)
        virtual std::uint8_t size() = 0;
    };

    // and now we have a class which manages a particular amplifier: the PGA308.
    // transformations between discrete params of the amplifier in the form of DiscreteParams
    // towards the registers of the PGA308. And also the transformation from the registers towards full range Gain+Offset
    class PGA308
    {
    public:

        class Registers: public IFregs
        {
        public:
            std::uint16_t       GD;                 // it is a gain register
            std::uint8_t        GI          : 4;    // it is a gain register
            std::uint8_t        S           : 1;    // it is a sign of gain register (keep it always 0)
            std::uint8_t        GO          : 3;    // it is a gain register
            std::uint8_t        Voffsetcoarse;      // it is an offset register
            std::uint16_t       Vzerodac;           // it is an offset register

            static const std:: uint8_t sizeofregisters = 6;

            static const std::uint8_t defval[sizeofregisters];// = {0x00, 0x40, 0x46, 0x1f, 0xb1, 0x7f}; // gain = 48, offset = midrangeOffset

            Registers() : GD(0), GI(0), S(0), GO(0), Voffsetcoarse(0), Vzerodac(0) {}
            Registers(void *data, size_t size) { load(data, size); }
            virtual bool load(const void *data, const size_t size);
            virtual bool fill(void *data, size_t &size);
            virtual std::uint8_t size() { return sizeofregisters; }
        };


        PGA308();
        ~PGA308();

        // usage: there are two possible modes:
        // 1. import registers and retrieve gain+offset into WideParams
        // 2. load discrete-gain+ofsfet and retrieve regs into Registers.

        bool import(const Registers &regs, WideParams *wideparams = nullptr);
        bool get(WideParams &wideparams);

        bool import(const DiscreteParams &discreteparams, Registers *regs = nullptr);
        bool get(Registers &regs);



    protected:

        // in here we put a ... protected interface

        // loads into the PGA308 the basic registers which define its behaviour in terms of gain-offset
        // that will cause the PGA to have values of gain and offset
        bool load(const Registers &regs);

        // attempts to load into the PGA308 a pair discretegain-offset, which produce a given Registers.
        // the attempt can cause sligthly different values
        bool load(const DiscreteGain g, const Offset offset = midrangeOffset);
        bool load(const DiscreteParams &discreteparams);

        // retrieve the pair gain-offset which is effectively in use after a load operation
        bool get(Gain &gain, Offset &offset);


        bool get(DiscreteParams &discreteparams);



    private:
        struct Impl;
        Impl *pImpl;
    };


    void testIT(std::string &output);


}} // namespace strain { namespace amplifier



class cDownloader;

namespace strain { namespace regulation {


    const std::uint8_t maxSets = 3;

    enum class Version { two = 2, three = 3, four = 4 };

    struct Analog1
    {   // for strain1
        std::uint8_t offset[strain::numberofchannels];
        Analog1() { clear(); }
        void clear() { std::memset(offset, 0, sizeof(offset)); }
    };

    struct Analog
    {   // for strain2
        std::uint8_t amplregs[strain::numberofchannels][strain::amplifier::PGA308::Registers::sizeofregisters];
        Analog() { clear(); }
        void clear() {
            for(int i=0; i<strain::numberofchannels; i++)
            {
                std::memmove(amplregs[i], strain::amplifier::PGA308::Registers::defval, sizeof(strain::amplifier::PGA308::Registers::sizeofregisters));
            }
        }
    };



    struct Digital
    {
        std::uint16_t tare[strain::numberofchannels];
        strain::dsp::Q15 matrix[strain::numberofchannels*strain::numberofchannels];
        strain::dsp::FSC fullscale[strain::numberofchannels];
        Digital() { clear(); }
        void clear() {
            std::memset(tare, 0, sizeof(tare));
            std::memset(fullscale, strain::dsp::fsc::max, sizeof(fullscale));
            strain::dsp::q15::matrix mat(strain::numberofchannels, strain::numberofchannels, matrix);
            mat.diagonal(strain::dsp::q15::posOneNearly);
        }
    };

    struct Set1
    {   // for strain1
       Analog1  analog1;
       Digital  digital;
    };

    struct Set
    {   // for strain2
        Analog   analog;
        Digital  digital;
        Set() { clear(); }
        void clear() { analog.clear(); digital.clear(); }
    };



    struct FullRegulation1
    {
        Version             version;
        strain::Board       board;
        std::string         serial;
        Set1                set1;
    };

    struct FullRegulation
    {
        Version             version;
        strain::Board       board;
        std::string         serial;
        std::uint8_t        set2useatbootstrap; // 1, 2 or 3
        std::vector<Set>    sets;
        FullRegulation() { clear(); }
        void clear() { version = Version::four; board = Board::strain2; serial = "SN666"; set2useatbootstrap = 1; sets.resize(0); }
    };

    // it just reads from file and fills FullRegulation for use outside of this module
    bool read(const std::string filename, FullRegulation &reg);

    // it just reads from file and fills FullRegulation for use outside of this module
    bool write(const std::string filename, const FullRegulation &reg);
    bool apply(cDownloader *down, const FullRegulation &reg);

} }   // namespace strain { namespace regulation {





#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




