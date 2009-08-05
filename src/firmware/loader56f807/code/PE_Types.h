#ifndef __PE_Types_H
#define __PE_Types_H

#define  FALSE  0                      /* Boolean value FALSE. FALSE is defined always as a zero value. */
#define  TRUE   1                      /* Boolean value TRUE. TRUE is defined always as a non zero value. */

#ifndef NULL
  #define  NULL   0
#endif

/* PE types definition */
typedef unsigned char bool;
typedef unsigned char byte;
typedef unsigned int word;
typedef unsigned long dword;
typedef unsigned long dlong[2];

/* SDK types definition */
typedef char           Word8;
typedef unsigned char  UWord8;
typedef short          Word16;
typedef unsigned short UWord16;
typedef long           Word32;
typedef unsigned long  UWord32;
typedef char           Int8;
typedef unsigned char  UInt8;
typedef int            Int16;
typedef unsigned int   UInt16;
typedef long           Int32;
typedef unsigned long  UInt32;
typedef __typeof__(sizeof(0)) ssize_t;

/**************************************************/
/* PE register access macros                      */
/**************************************************/
#define setRegBit(reg, bit)                                     (reg |= reg##_##bit##_##MASK)
#define clrRegBit(reg, bit)                                     (reg &= ~reg##_##bit##_##MASK)
#define getRegBit(reg, bit)                                     (reg & reg##_##bit##_##MASK)
#define setReg(reg, val)                                        (reg = (word)(val))
#define getReg(reg)                                             reg
#define setRegBits(reg, mask)                                   (reg |= (word)(mask))
#define getRegBits(reg, mask)                                   (reg & (word)(mask))
#define clrRegBits(reg, mask)                                   reg &= (word)(~(mask))
#define setRegBitGroup(reg, bits, val)                          reg = (word)((reg & ~reg##_##bits##_##MASK) | ((val) << reg##_##bits##_##BITNUM))
#define getRegBitGroup(reg, bits)                               ((reg & reg##_##bits##_##MASK) >> reg##_##bits##_##BITNUM)
#define setRegMask(reg, maskAnd, maskOr)                        reg = (word)((getReg(reg) & ~(maskAnd)) | (maskOr))
#define setRegBitVal(reg, bit, val)                             ((val) == 0 ? (reg &= ~reg##_##bit##_##MASK) : (reg |= reg##_##bit##_##MASK))
#define changeRegBits(reg, mask)                                reg ^= (mask)
#define changeRegBit(reg, bit)                                  reg ^= reg##_##bit##_##MASK

/**************************************************/
/* Uniform multiplatform peripheral access macros */
/**************************************************/
#define setReg16Bit(RegName, BitName)                           (RegName |= RegName##_##BitName##_##MASK)
#define clrReg16Bit(RegName, BitName)                           (RegName &= ~RegName##_##BitName##_##MASK)
#define invertReg16Bit(RegName, BitName)                        (RegName ^= RegName##_##BitName##_##MASK)
#define testReg16Bit(RegName, BitName)                          (RegName & RegName##_##BitName##_##MASK)

/* Whole peripheral register access macros */
#define setReg16(RegName, val)                                   (RegName = (word)(val))
#define getReg16(RegName)                                        RegName

/* Bits peripheral register access macros */
#define testReg16Bits(RegName, GetMask)                          (RegName & (GetMask))
#define clrReg16Bits(RegName, ClrMask)                           (RegName &= ~(word)(ClrMask))
#define setReg16Bits(RegName, SetMask)                           (RegName |= (word)(SetMask))
#define invertReg16Bits(RegName, InvMask)                        (RegName ^= (word)(InvMask))
#define clrSetReg16Bits(RegName, ClrMask, SetMask)               (RegName = (RegName & (~(word)(ClrMask))) | (word)(SetMask))
#define seqClrSetReg16Bits(RegName, BitsMask, BitsVal)           ( RegName &= ~(~(word)(BitsVal) & (word)(BitsMask)),\
                                                                 RegName |= (word)(BitsVal) & (word)(BitsMask) )
#define seqSetClrReg16Bits(RegName, BitsMask, BitsVal)           ( RegName |= (word)(BitsVal) & (word)(BitsMask),\
                                                                 RegName &= ~(~(word)(BitsVal) & (word)(BitsMask)) )
#define seqResetSetReg16Bits(RegName, BitsMask, BitsVal)         ( RegName &= ~(word)(BitsMask),\
                                                                 RegName |= (word)(BitsVal) & (word)(BitsMask) )
#define clrReg16BitsByOne(RegName, ClrMask, BitsMask)            (RegName &= (word)(ClrMask) & (word)(BitsMask))

/* Bit group peripheral register access macros */
#define testReg16BitGroup(RegName, GroupName)                    (RegName & RegName##_##GroupName##_##MASK)
#define getReg16BitGroupVal(RegName, GroupName)                  (RegName & RegName##_##GroupName##_##MASK) >> RegName##_##GroupName##_##BITNUM
#define setReg16BitGroupVal(RegName, GroupName, GroupVal)        (RegName = (RegName & ~RegName##_##GroupName##_##MASK) | ((GroupVal) << RegName##_##GroupName##_##BITNUM))
#define seqClrSetReg16BitGroupVal(RegName,GroupName,GroupVal)    ( RegName &= ~(~((GroupVal) << RegName##_##GroupName##_##BITNUM) & RegName##_##GroupName##_##MASK),\
                                                                 RegName |= ((GroupVal) << RegName##_##GroupName##_##BITNUM) & RegName##_##GroupName##_##MASK )
#define seqSetClrReg16BitGroupVal(RegName,GroupName,GroupVal)    ( RegName |= ((GroupVal) << RegName##_##GroupName##_##BITNUM) & RegName##_##GroupName##_##MASK,\
                                                                 RegName &= ~(~((GroupVal) << RegName##_##GroupName##_##BITNUM) & RegName##_##GroupName##_##MASK) )
#define seqResetSetReg16BitGroupVal(RegName,GroupName,GroupVal)  ( RegName &= ~RegName##_##GroupName##_##MASK,\
                                                                 RegName |= ((GroupVal) << RegName##_##GroupName##_##BITNUM) & RegName##_##GroupName##_##MASK )



#define in16(var,l,h)  var = ((word)(l)) | (((word)(h)) << 8)
#define out16(l,h,val) { l = (byte)(val); h = (byte)((val) >> 8); }

#define output(P, V) setReg(P,V)
#define input(P) getReg(P)

#define __EI0() { asm(bfset  #0x0100,SR); asm(bfclr  #0x0200,SR); } /* Enable interrupts of level 0,1 */
#define __EI(level)  __EI0()           /* Enable interrupts of level 0,1 */
#define __DI() { asm(bfset  #0x0300,SR); } /* Disable interrupts, only level 1 is allowed */

#define SaveStatusReg()     { asm(move SR,SR_reg); asm(bfset  #0x0300,SR); asm(nop); asm(nop); } /* Save status register and disable interrupts */
#define RestoreStatusReg()  { asm(move SR_reg,SR); asm(nop); asm(nop); } /* Restore status register  */
#define EnterCritical()     { if (++SR_lock == 1) SaveStatusReg(); } /* Save status register and disable interrupts */
#define ExitCritical()      { if (--SR_lock == 0) RestoreStatusReg(); } /* Restore status register  */

typedef struct {          /* Black&White Image  */
  word width;             /* Image width  */
  word height;            /* Image height */
  byte *pixmap;           /* Image pixel bitmap */
  dword size;             /* Image size   */
  char *name;             /* Image name   */
} TIMAGE;
typedef TIMAGE* PIMAGE ; /* Pointer to image */

/* 16-bit register (big endian) */
typedef union {
   word w;
   struct {
     byte high,low;
   } b;
} TWREG;

typedef union
{
        struct
        {
          UWord16 LSBpart;
          Word16 MSBpart;
        } RegParts;

        Word32 Reg32bit;

} decoder_uReg32bit;

typedef struct
{
        union { Word16 PositionDifferenceHoldReg;
                Word16 posdh; };
        union { Word16 RevolutionHoldReg;
                Word16 revh; };
        union { decoder_uReg32bit PositionHoldReg;
                Word32 posh; };

}decoder_sState;

typedef struct
{
        UWord16 EncPulses;
        UWord16 RevolutionScale;

        Int16   scaleDiffPosCoef;
        UInt16  scalePosCoef;
        Int16   normDiffPosCoef;
        Int16   normPosCoef;
}decoder_sEncScale;

typedef struct
{
        UWord16 Index    :1;
        UWord16 PhaseB   :1;
        UWord16 PhaseA   :1;
        UWord16 Reserved :13;
}decoder_sEncSignals;

typedef union{
        decoder_sEncSignals  EncSignals;
        UWord16 Value;
} decoder_uEncSignals;

/******************************************************************************
*
* This Motor Control section contains generally useful and generic
* types that are used throughout the domain of motor control.
*
******************************************************************************/
/* Fractional data types for portability */
typedef short          Frac16;
typedef long           Frac32;

typedef enum
{
        mcPhaseA,
        mcPhaseB,
        mcPhaseC
} mc_ePhaseType;

typedef struct
{
        Frac16 PhaseA;
        Frac16 PhaseB;
        Frac16 PhaseC;
} mc_s3PhaseSystem;

/* general types, primary used in FOC */

typedef struct
{
        Frac16 alpha;
        Frac16 beta;
} mc_sPhase;

typedef struct
{
        Frac16 sine;
        Frac16 cosine;
} mc_sAngle;

typedef struct
{
        Frac16 d_axis;
        Frac16 q_axis;
} mc_sDQsystem;

typedef struct
{
        Frac16 psi_Rd;
        Frac16 omega_field;
        Frac16 i_Sd;
        Frac16 i_Sq;
} mc_sDQEstabl;

typedef UWord16 mc_tPWMSignalMask;    /*  pwm_tSignalMask contains six control bits
                                          representing six PWM signals, shown below.
                                          The bits can be combined in a numerical value
                                          that represents the union of the appropriate
                                          bits.  For example, the value 0x15 indicates
                                          that PWM signals 0, 2, and 4 are set.
                                      */

#define MC_PWM_SIGNAL_0       0x0001
#define MC_PWM_SIGNAL_1       0x0002
#define MC_PWM_SIGNAL_2       0x0004
#define MC_PWM_SIGNAL_3       0x0008
#define MC_PWM_SIGNAL_4       0x0010
#define MC_PWM_SIGNAL_5       0x0020
#define MC_PWM_NO_SIGNALS     0x0000     /* No (none) PWM signals */
#define MC_PWM_ALL_SIGNALS   (MC_PWM_SIGNAL_0 | \
                                                                        MC_PWM_SIGNAL_1 | \
                                                                        MC_PWM_SIGNAL_2 | \
                                                                        MC_PWM_SIGNAL_3 | \
                                                                        MC_PWM_SIGNAL_4 | \
                                                                        MC_PWM_SIGNAL_5)

/* general types, primary used in PI, PID and other controllers */

typedef struct
{
   Word16 ProportionalGain;
   Word16 ProportionalGainScale;
   Word16 IntegralGain;
   Word16 IntegralGainScale;
   Word16 DerivativeGain;
   Word16 DerivativeGainScale;
   Word16 PositivePIDLimit;
   Word16 NegativePIDLimit;
   Word16 IntegralPortionK_1;
   Word16 InputErrorK_1;
}mc_sPIDparams;

typedef struct
{
   Word16 ProportionalGain;
   Word16 ProportionalGainScale;
   Word16 IntegralGain;
   Word16 IntegralGainScale;
   Word16 PositivePILimit;
   Word16 NegativePILimit;
   Word16 IntegralPortionK_1;
}mc_sPIparams;

#endif /* __PE_Types_H */

