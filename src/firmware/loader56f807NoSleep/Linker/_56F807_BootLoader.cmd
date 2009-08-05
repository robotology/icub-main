
MEMORY {
        # I/O registers area for on-chip peripherals
        .x_Peripherals (RW)   : ORIGIN = 0x1000, LENGTH = 0
        .x_CoreRegs    (RW)   : ORIGIN = 0xFF80, LENGTH = 0

        # List of all sections specified in the "Build options" tab
        #Internal vector boot area.
        .p_Interruptsboot  (RWX) : ORIGIN = 0x0000F800, LENGTH = 0x0004
        .p_Interrupts  (RWX) : ORIGIN = 0x00000000, LENGTH = 0x00000080
        .p_Code  (RWX) : ORIGIN = 0x0000F804, LENGTH = 0x000007FC
        .x_Data  (RW) : ORIGIN = 0x00000040, LENGTH = 0x00000FC0
        .x_CWRegisters  (RW) : ORIGIN = 0x00000030, LENGTH = 0x00000010


        # p_flash_ROM_data mirrors x_internal_RAM, mapping to origin and length
        # the "X" flag in "RX" tells the debugger flash p-memory.
        # the p-memory flash is directed to the address determined by AT
        # in the data_in_p_flash_ROM section definition

        .p_flash_ROM_data  (RX) : ORIGIN = 0x00000040, LENGTH = 0x00000FC0

        #Other memory segments
        .x_internal_ROM  (RW)  : ORIGIN = 0x00002000, LENGTH = 0x1000
        .x_internal_ROM2 (R)   : ORIGIN = 0x00003000, LENGTH = 0x1000
        .p_internal_RAM  (RWX) : ORIGIN = 0x0000F000, LENGTH = 0x0800


}

KEEP_SECTION { interrupt_vectorsboot.text }
KEEP_SECTION { interrupt_vectors.text }

SECTIONS {

        .interrupt_vectorsboot :
        {
          F_vector_addr = .;
          # interrupt vectors boot area
          * (interrupt_vectorsboot.text)
        } > .p_Interruptsboot

        .interrupt_vectors :
        {
          # interrupt vectors
          * (interrupt_vectors.text)
        } > .p_Interrupts

        .ApplicationCode :
        {

              F_Pcode_start_addr = .;

              # .text sections
              * (.text)
              * (rtlib.text)
              * (startup.text)
              * (fp_engine.text)
              * (user.text)

              F_Pcode_end_addr = .;

              # save address where for the data start in pROM
              . = ALIGN(2);
              __pROM_data_start = .;

        } > .p_Code

        # AT sets the download address
        # the download stashes the data just after the program code
        # as determined by our saved pROM data start variable

        .data_in_p_flash_ROM : AT(__pROM_data_start)
        {
              # the data sections flashed to pROM
              # save data start so we can copy data later to xRAM

              __xRAM_data_start = .;

              # .data sections
              * (strings.data)
              
              * (fp_state.data)
              * (rtlib.data)
              * (.data.char)        # used if "Emit Separate Char Data Section" enabled
              * (.data)

              # save data end and calculate data block size
              __xRAM_data_end = .;
              __data_size = __xRAM_data_end - __xRAM_data_start;

        } > .p_flash_ROM_data    # this section is designated as p-memory
                                 # with X flag in the memory map
                                 # the start address and length map to
                                 # actual internal xRAM
                                 
		.x_internal_ROM2_data :
		{
			__xROM2_data_start =.;
			F_data_ROM2_addr = __xROM2_data_start;
		} > .x_internal_ROM2
                                 

        .ApplicationData :
        {
              # save space for the pROM data copy
              . = __xRAM_data_start + __data_size;

              # .bss sections
              * (rtlib.bss.lo)
              * (rtlib.bss)
              . = ALIGN(4);
              F_Xbss_start_addr = .;
              _START_BSS = .;
              * (.bss.char)         # used if "Emit Separate Char Data Section" enabled
              * (.bss)
              _END_BSS   = .;
              F_Xbss_length = _END_BSS - _START_BSS;

              /* Setup the HEAP address */
              . = ALIGN(4);
              _HEAP_ADDR = .;
              _HEAP_SIZE = 0x00000100;
              _HEAP_END = _HEAP_ADDR + _HEAP_SIZE;
              . = _HEAP_END;

              /* SETUP the STACK address */
              _min_stack_size = 0x00000200;
              _stack_addr = _HEAP_END;
              _stack_end  = _stack_addr + _min_stack_size;
              . = _stack_end;

              /* EXPORT HEAP and STACK runtime to libraries */
              F_heap_addr   = _HEAP_ADDR;
              F_heap_end    = _HEAP_END;
              F_Lstack_addr = _HEAP_END;
              F_StackAddr = _HEAP_END;
              F_StackEndAddr = _stack_end - 1;

              # runtime code __init_sections uses these globals:

              F_Ldata_size     = __data_size;
              F_Ldata_RAM_addr = __xRAM_data_start;
              F_Ldata_ROM_addr = __pROM_data_start;

              F_xROM_to_xRAM   = 0x0000;
              F_pROM_to_xRAM   = 0x0001;

              F_start_bss   = _START_BSS;
              F_end_bss     = _END_BSS;

        } > .x_Data

        # peripheral registers
        FArchIO = ADDR(.x_Peripherals); /* Peripheral registers */

        # peripheral registers
        FCoreIO = ADDR(.x_CoreRegs);   /* Core registers */
        
		FarchStart = 0x0080;
		
}
