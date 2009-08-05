#include "Cpu.h"


extern void _EntryPoint(void);         // Startup routine //


#pragma define_section interrupt_vectorsboot "interrupt_vectorsboot.text"  RX
#pragma section interrupt_vectorsboot begin
static asm void _vectboot(void) {
  JMP  _EntryPoint                     // Reset vector (Used) //
  JMP  _EntryPoint                     // COP reset vector (Used) //
}
#pragma section interrupt_vectorsboot end

#pragma define_section interrupt_vectors "interrupt_vectors.text"  RX
#pragma section interrupt_vectors begin
static asm void _vect(void) {
  JMP  _EntryPoint                     // Interrupt no. 0 (Used)   - ivINT_Reset //
  JMP  _EntryPoint                     // Interrupt no. 1 (Used)   - ivINT_COPReset  //
  JSR  Cpu_Interrupt                   // Interrupt no. 2 (Unused) - ivINT_Reserved1  //
  JSR  Cpu_Interrupt                   // Interrupt no. 3 (Unused) - ivINT_Illegal_Instruction  //
  JSR  Cpu_Interrupt                   // Interrupt no. 4 (Unused) - ivINT_SWI  //
  JSR  Cpu_Interrupt                   // Interrupt no. 5 (Unused) - ivINT_HWStackOverflow  //
  JSR  Cpu_Interrupt                   // Interrupt no. 6 (Unused) - ivINT_OnCETrap  //
  JSR  Cpu_Interrupt                   // Interrupt no. 7 (Unused) - ivINT_Reserved2  //
  JSR  Cpu_Interrupt                   // Interrupt no. 8 (Unused) - ivINT_IRQA  //
  JSR  Cpu_Interrupt                   // Interrupt no. 9 (Unused) - ivINT_IRQB  //
  JSR  Cpu_Interrupt                   // Interrupt no. 10 (Unused) - ivINT_Reserved3  //
  JSR  Cpu_Interrupt                   // Interrupt no. 11 (Unused) - ivINT_BFlash  //
  JSR  Cpu_Interrupt                   // Interrupt no. 12 (Unused) - ivINT_PFlash  //
  JSR  Cpu_Interrupt                   // Interrupt no. 13 (Unused) - ivINT_DFlash  //
  JSR  Cpu_Interrupt                   // Interrupt no. 14 (Unused) - ivINT_MSCAN_TxReady  //
  JSR  Cpu_Interrupt                   // Interrupt no. 15 (Unused) - ivINT_MSCAN_RxFull  //
  JSR  Cpu_Interrupt                   // Interrupt no. 16 (Unused) - ivINT_MSCAN_Error  //
  JSR  Cpu_Interrupt                   // Interrupt no. 17 (Unused) - ivINT_MSCAN_Wakeup  //
  JSR  Cpu_Interrupt                   // Interrupt no. 18 (Unused) - ivINT_PFlash2  //
  JSR  Cpu_Interrupt                   // Interrupt no. 19 (Unused) - ivINT_GPIO_E  //
  JSR  Cpu_Interrupt                   // Interrupt no. 20 (Unused) - ivINT_GPIO_D  //
  JSR  Cpu_Interrupt                   // Interrupt no. 21 (Unused) - ivINT_Reserved4  //
  JSR  Cpu_Interrupt                   // Interrupt no. 22 (Unused) - ivINT_GPIO_B  //
  JSR  Cpu_Interrupt                   // Interrupt no. 23 (Unused) - ivINT_GPIO_A  //
  JSR  Cpu_Interrupt                   // Interrupt no. 24 (Unused) - ivINT_SPI_TxEmpty  //
  JSR  Cpu_Interrupt                   // Interrupt no. 25 (Unused) - ivINT_SPI_RxFull  //
  JSR  Cpu_Interrupt                   // Interrupt no. 26 (Unused) - ivINT_DEC1_Home_Watchdog  //
  JSR  Cpu_Interrupt                   // Interrupt no. 27 (Unused) - ivINT_DEC1_Index  //
  JSR  Cpu_Interrupt                   // Interrupt no. 28 (Unused) - ivINT_DEC0_Home_Watchdog  //
  JSR  Cpu_Interrupt                   // Interrupt no. 29 (Unused) - ivINT_DEC0_Index  //
  JSR  Cpu_Interrupt                   // Interrupt no. 30 (Unused) - ivINT_TMRD0  //
  JSR  Cpu_Interrupt                   // Interrupt no. 31 (Unused) - ivINT_TMRD1  //
  JSR  Cpu_Interrupt                   // Interrupt no. 32 (Unused) - ivINT_TMRD2  //
  JSR  Cpu_Interrupt                   // Interrupt no. 33 (Unused) - ivINT_TMRD3  //
  JSR  Cpu_Interrupt                   // Interrupt no. 34 (Unused) - ivINT_TMRC0  //
  JSR  Cpu_Interrupt                   // Interrupt no. 35 (Unused) - ivINT_TMRC1  //
  JSR  Cpu_Interrupt                   // Interrupt no. 36 (Unused) - ivINT_TMRC2  //
  JSR  Cpu_Interrupt                   // Interrupt no. 37 (Unused) - ivINT_TMRC3  //
  JSR  Cpu_Interrupt                   // Interrupt no. 38 (Unused) - ivINT_TMRB0  //
  JSR  Cpu_Interrupt                   // Interrupt no. 39 (Unused) - ivINT_TMRB1  //
  JSR  Cpu_Interrupt                   // Interrupt no. 40 (Unused) - ivINT_TMRB2  //
  JSR  Cpu_Interrupt                   // Interrupt no. 41 (Unused) - ivINT_TMRB3  //
  JSR  Cpu_Interrupt                   // Interrupt no. 42 (Unused) - ivINT_TMRA0  //
  JSR  Cpu_Interrupt                   // Interrupt no. 43 (Unused) - ivINT_TMRA1  //
  JSR  Cpu_Interrupt                   // Interrupt no. 44 (Unused) - ivINT_TMRA2  //
  JSR  Cpu_Interrupt                   // Interrupt no. 45 (Unused) - ivINT_TMRA3  //
  JSR  Cpu_Interrupt                   // Interrupt no. 46 (Unused) - ivINT_SCI1_TxComplete  //
  JSR  Cpu_Interrupt                   // Interrupt no. 47 (Unused) - ivINT_SCI1_TxReady  //
  JSR  Cpu_Interrupt                   // Interrupt no. 48 (Unused) - ivINT_SCI1_RxError  //
  JSR  Cpu_Interrupt                   // Interrupt no. 49 (Unused) - ivINT_SCI1_RxFull  //
  JSR  Cpu_Interrupt                   // Interrupt no. 50 (Unused) - ivINT_SCI0_TxComplete  //
  JSR  Cpu_Interrupt                   // Interrupt no. 51 (Unused) - ivINT_SCI0_TxReady  //
  JSR  Cpu_Interrupt                   // Interrupt no. 52 (Unused) - ivINT_SCI0_RxError  //
  JSR  Cpu_Interrupt                   // Interrupt no. 53 (Unused) - ivINT_SCI0_RxFull  //
  JSR  Cpu_Interrupt                   // Interrupt no. 54 (Unused) - ivINT_ADCB_Complete  //
  JSR  Cpu_Interrupt                   // Interrupt no. 55 (Unused) - ivINT_ADCA_Complete  //
  JSR  Cpu_Interrupt                   // Interrupt no. 56 (Unused) - ivINT_ADCB_ZC_LE  //
  JSR  Cpu_Interrupt                   // Interrupt no. 57 (Unused) - ivINT_ADCA_ZC_LE  //
  JSR  Cpu_Interrupt                   // Interrupt no. 58 (Unused) - ivINT_PWMB_Reload  //
  JSR  Cpu_Interrupt                   // Interrupt no. 59 (Unused) - ivINT_PWMA_Reload  //
  JSR  Cpu_Interrupt                   // Interrupt no. 60 (Unused) - ivINT_PWMB_Fault  //
  JSR  Cpu_Interrupt                   // Interrupt no. 61 (Unused) - ivINT_PWMA_Fault  //
  JSR  Cpu_Interrupt                   // Interrupt no. 62 (Unused) - ivINT_PLL  //
  JSR  Cpu_Interrupt                   // Interrupt no. 63 (Unused) - ivINT_LowVoltage  //
}
#pragma section interrupt_vectors end
