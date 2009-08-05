
  

# ----------------------------------------------------

# Metrowerks sample code

# linker command file for DSP56807EVM
# using 
#     external pRAM
#     external xRAM
#     internal xRAM (for compiler regs)
#         mode 3
#          EXT 0

# ----------------------------------------------------



# see end of file for additional notes
# additional reference: Motorola docs 



# for this LCF:
# interrupt vectors --> external pRAM starting at zero
#      program code --> external pRAM
#         constants --> external xRAM
#      dynamic data --> external xRAM 

# stack size is set to 0x1000 for external RAM LCF


# requirements: Mode 3 and EX=0
# note -- there is a mode OB but any Reset or COP Reset 
#         resets the memory map back to Mode 0A.


# DSP56807EVM board settings
#    ON --> jumper JG7 (mode 0 upon exit from reset)
#    ON --> jumper JG8 (enable external board SRAM)


# CodeWarrior debugger Target option settings
#   OFF --> "Use Hardware Breakpoints" 
#    ON --> "Debugger sets OMR at Launch" option

# note: with above option on, CW debugger sets OMR as
# OMR:
#     0 --> EX bit (stay in Debug processing state)
#     1 --> MA bit  
#     1 --> MB bit






# 56807
# mode 3 (development)
# EX = 0

MEMORY 
{
    .p_interrupts_ext_RAM (RWX) : ORIGIN = 0x0000, LENGTH = 0x0080 
    .p_external_RAM       (RWX) : ORIGIN = 0x0080, LENGTH = 0x0000
    .x_compiler_regs_iRAM (RW)  : ORIGIN = 0x0030, LENGTH = 0x0010 	
    .x_internal_RAM       (RW)  : ORIGIN = 0x0040, LENGTH = 0x0FC0
    .x_peripherals        (RW)  : ORIGIN = 0x1000, LENGTH = 0x0800
#   .x_reserved                 : ORIGIN = 0x1800, LENGTH = 0x0800
    .x_flash_ROM          (R)   : ORIGIN = 0x2000, LENGTH = 0x2000
    .x_external_RAM       (RW)  : ORIGIN = 0x4000, LENGTH = 0xBF80
    .x_core_regs          (RW)  : ORIGIN = 0xFF80, LENGTH = 0x0000
}





# we ensure the interrupt vector section is not deadstripped here

KEEP_SECTION{ interrupt_vectors.text }




# place all executing code & data in external memory

SECTIONS 
{

	.interrupt_vectors_for_p_ram :
	{
	    # from 56807_vector_pROM.asm
	    * (interrupt_vectors.text) 
	   
	} > .p_interrupts_ext_RAM
	
		 
		 
	.executing_code :
	{
		# .text sections
		
		* (.text)
		* (rtlib.text)
		* (fp_engine.text)
		* (startup.text)	
		* (user.text)	
	} > .p_external_RAM



	.data :
	{
		# .data sections
		
		* (.const.data)		
		* (fp_state.data)
		* (rtlib.data)
		* (.data)

					
		
		# .bss sections
		
	  	* (rtlib.bss.lo)
	  	
	  	__bss_start = .;
	  	
		* (.bss)
		
		__bss_end   = .;
		__bss_size = __bss_end - __bss_start;



		# setup the heap address
		
		__heap_addr = .;
		__heap_size = 0x1000; # larger heap for hostIO
		__heap_end = __heap_addr + __heap_size; 
		
		. = __heap_end;



		# setup the stack address 
		
		_min_stack_size = 0x0200;
		__stack_addr = __heap_end;
		__stack_end  = __stack_addr + _min_stack_size;
		. = __stack_end;
		
		
		
		# set global vars now
		
		# MSL uses these globals:		
		F_heap_addr  = __heap_addr;
		F_heap_end   = __heap_end;
		F_stack_addr = __stack_addr;
		
		
		# stationery init code globals
		
		F_bss_size      = __bss_size;
		F_bss_addr      = __bss_start;

		# next not used in this LCF 
		# we define anyway so init code will link 
		# these can be removed with removal of rom-to-ram
		# copy code in init file
	
		F_data_size     = 0x0000;
		F_data_RAM_addr = 0x0000;
		F_data_ROM_addr = 0x0000;
		
        F_rom_to_ram    = 0x0000; # zero is no rom-to-ram copy
	
	} > .x_external_RAM	
}




# -------------------------------------------------------
# additional notes:


# about the reserved sections
# for this external RAM only LCF:

# p_interrupts_RAM -- reserved in external pRAM
# memory space reserved for interrupt vectors
# interrupt vectors must start at address zero
# interrupt vector space size is 0x80

# x_compiler_regs_iRAM -- reserved in internal xRAM
# The compiler uses page 0 address locations 0x30-0x40 
# as register variables. See the Target manual for more info.



# notes:
# program memory (p memory)
# (RWX) read/write/execute for pRAM
# (RX) read/execute for flashed pROM

# data memory (X memory)
# (RW) read/write for xRAM
# (R)  read for data flashed xROM

# LENGTH = next start address - previous
# LENGTH = 0x0000 means use all remaing memory



# revision history
# 011020 R4.1 a.h. first version
# 030220 R5.1 a.h. improved comments
