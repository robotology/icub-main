



/*  metrowerks sample code  */



// phase locked loop registers for M56803-7
// PRECS bit is reserved (0) for 803,805,807. 
// 801 it can be 0=Relaxation Oscillator and  1 for external 

#define CLKGEN_BASE 0x13A0 // different from 803/805

#define PLLCR CLKGEN_BASE + 0x0000  // pll control register
#define PLLDB CLKGEN_BASE + 0x0001  // pll divide-by register
#define PLLSR CLKGEN_BASE + 0x0002  // pll status register


#define TESTR  CLKGEN_BASE + 0x0003  // pll test register
#define CLKOSR CLKGEN_BASE + 0x0004  // pll select register
#define ISOCTL CLKGEN_BASE + 0x0005  // pll internal oscillator control register



// PLL setup #defines for 56803, 56805, 56807

#define wait_lock 0x4000


#define pllcr_init 0x0081  // 0000000010000001b   
// notes:
//   0000  PLL Interrupt Enable 1 (PLLIE1[1:0]) bits 15-14
//     00  PLL Interrupt Enable 0 (PLLIE0[1:0]) bits 13-12
//     00  Loss of Clock Interrupt Enable (LOCIE) bit 11
//    xxx  Reserved Bits bits 10-8
//      1  Lock Detector On (LCKON) bit 7  (lock set on)
//      0  Charge Pump Tri-state (CHPMPTRI) bit 6
//      x  Reserved Bit bit 5
//      0  PLL Power Down (PLLPD) bit 4
//      x  Reserved Bit bit-3
//      x  Prescaler Clock Select (PRECS) bit 2  (reserved as zero for 803-7)
//     01  ZCLOCK Source (ZSRC[1:0]) bits 1-0 (prescaler chosen -- core clock)


#define plldb_init 0xF013 // 1111000000010011b
// notes:
//   1111  Loss of Reference Timer Period (LORTP[3:0]) bits 15-12 (set to max)
//     00  PLL Clock Out Divide (PLLCOD[1:0]) bits 11-10
//     00  PLL Clock In Divide (PLLCID[1:0]) bits 9-8
//         Reserved Bits bit 7
// 010011  PLL Divide-By (PLLDB[6:0]) bits 6-0  (set to max)
   

#define pllcr_proceed 0x0082  // 0000000010000010b
// notes:
//   0000  PLL Interrupt Enable 1 (PLLIE1[1:0]) bits 15-14
//     00  PLL Interrupt Enable 0 (PLLIE0[1:0]) bits 13-12
//     00  Loss of Clock Interrupt Enable (LOCIE) bit 11
//    xxx  Reserved Bits bits 10-8
//      1  Lock Detector On (LCKON) bit 7  (detect lock set on)
//      0  Charge Pump Tri-state (CHPMPTRI) bit 6
//      x  Reserved Bit bit 5
//      0  PLL Power Down (PLLPD) bit 4
//      x  Reserved Bit bit-3
//      x  Prescaler Clock Select (PRECS) bit 2  (reserved as zero for 803-7)
//     10  ZCLOCK Source (ZSRC[1:0]) bits 1-0 (postscaler chosen -- the PLL clock)



#define pllsr_init 0x0040  // 0000000001000000b
// notes:
//      0  PLL Loss of Lock Interrupt 1 (LOLI1) bit 15
//      0  PLL Loss of Lock Interrupt 0 (LOLI0) bit 14
//      0  Loss of Clock (LOCI) bit 13
//     xx  Reserved Bits bits 12-7
//      1  Loss of Lock 1 (LCK1) bit 6
//      0  Loss of Lock 0 (LCK0) bit 5
//      0  PLL Power Down (PLLPDN) bit 4
//      x  Reserved Bit bit 3
//      0  Prescaler Clock Status Source Register (PRECSS) bit 2
//     00  ZCLOCK Source (ZSRC[1:0]) bits 1-0


