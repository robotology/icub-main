
 

# ----------------------------------------------------

# Metrowerks sample code

# linker command file for DSP56807
# using 
#        flash pROM
#        flash xROM (const and dynamic data)
#     internal xRAM (dynamic data copied from xROM)

# ----------------------------------------------------



# see end of file for additional notes
# additional reference: Motorola docs 



# memory use for this LCF: 
# interrupt vectors --> flash pROM starting at zero
#      program code --> flash pROM
#         constants --> flash xROM
#      dynamic data --> flash xROM (copied to RAM with init) 



# requirements: Mode 0A and EX=0
# note -- there is a mode OB but any Reset or COP Reset 
#         resets the memory map back to Mode 0A.



# DSP56807EVM board settings
#    ON --> jumper JG7 (mode 0 upon exit from reset)
#   OFF --> jumper JG8 (enable external board SRAM)

# note: with this LCF memory config, internal memory 
#       will be used regardless of JG8 setting;
#       however, you can turn SRAM off to validate
#       use of internal RAM.



# CodeWarrior debugger Target option settings
#    ON --> "Use Hardware Breakpoints" 
#   OFF --> "Debugger sets OMR at Launch" option

# note: since debugger doesn't set OMR, init code's setting is:
# OMR:
#     0 --> EX bit (stay in Debug processing state)
#     0 --> MA bit  
#     0 --> MB bit





# DSP56807
# mode 0A
# EX = 0

MEMORY 
{
#   .p_boot_flash_1       (RX)  : ORIGIN = 0x0000, LENGTH = 0x0004 
    .p_interrupts_ROM     (RX)  : ORIGIN = 0x0004, LENGTH = 0x007C 
    .p_flash_ROM          (RX)  : ORIGIN = 0x0080, LENGTH = 0xEF80
    .p_internal_RAM       (RWX) : ORIGIN = 0xF000, LENGTH = 0x0800
#   .p_boot_flash_2       (RX)  : ORIGIN = 0xF800, LENGTH = 0x0000 
    .x_compiler_regs_iRAM (RW)  : ORIGIN = 0x0030, LENGTH = 0x0010 	
    .x_internal_RAM       (RW)  : ORIGIN = 0x0040, LENGTH = 0x0FC0
    .x_peripherals        (RW)  : ORIGIN = 0x1000, LENGTH = 0x0800
    .x_flash_ROM          (R)   : ORIGIN = 0x2000, LENGTH = 0x1000
    .x_flash_ROM2		  (R)	: ORIGIN = 0x3000, LENGTH = 0x1000
    .x_external_RAM       (RW)  : ORIGIN = 0x4000, LENGTH = 0xBF80
    .x_core_regs          (RW)  : ORIGIN = 0xFF80, LENGTH = 0x0000
}



# we ensure the interrupt vector sections are not deadstripped here

#KEEP_SECTION{ interrupt_vectors.text, interrupt_vectors_mirror.text }
KEEP_SECTION{ interrupt_vectors.text}



# About ROM-to-RAM copying at init time:

# In embedded programming,it is common for a portion of 
# a program resident in ROM to be copied into RAM at runtime.
# For starters,program variables cannot be accessed until 
# they are copied to RAM. 

# To indicate data or code that is meant to be copied 
# from ROM to RAM,the data or code is given two addresses.

# One address is its resident location in ROM (defined by 
# the linker command file).The other is its intended
# location in RAM (defined in C code where we will 
# do the actual copying).

# To create a section with the resident location in ROM 
# and an intended location in RAM,you define the two addresses 
# in the linker command file.

# Use the MEMORY segment to specify the intended RAM location,
# and the AT(address)parameter to specify the resident ROM address.


# we have defined the MEMORY segment x_internal_RAM above
# we set the .data section with AT to resident ROM address 



SECTIONS 
{
	.interrupt_vectors :
	{
	    # from 56807_vector_pROM.asm
	    * (interrupt_vectors.text)  
	   
	} > .p_interrupts_ROM



    # hawk mirrors these back to P memory boot_flash_1

#	.interrupt_vectors_mirror :
#	{
#	    # from 56807_vector_pROM.asm
#	    * (interrupt_vectors_mirror.text)  
#	   
#	} > .p_boot_flash_2

	.executing_code :
	{
		# Serial bootloader configuration section
		WRITEH(0xE9C8);         # JSR 0x0084 instruction opcode. Application code starts at the address
		WRITEH(0x0084);         # 0x0084 when serial bootloader support is enabled. JSR insctruction have
		                       	# to be placed at address 0x0080.
		WRITEH(0);              # Dummy word
		# Bootloader start delay in seconds
		WRITEH(5);              # Bootloader start delay config word at address 0x0083. Possible values 0-255.
		
		# .text sections
		
		* (.text)
		* (rtlib.text)
		* (fp_engine.text)
		* (startup.text)	
		* (user.text)	
	} > .p_flash_ROM
	
	.x_flash_ROM_data :
	{
	    
		* (.rodata)	 # initialized constants stay in flash xROM
		
		# save address for ROM data we will copy to RAM
		__xROM_data_start =.; 
		      
	} > .x_flash_ROM


	.x_flash_ROM2_data :
	{
		* (.flashdata)
		__xROM2_data_start =.;
		F_data_ROM2_addr = __xROM2_data_start;
	} > .x_flash_ROM2
	
	
# the LCF command AT sets optional parameter 
# that specifies the  address of the section. 
# The default (if not specified)is to make 
# the load address the same as the relocation address.
# Here, we want the load address to be in ROM
# and the relocation address in RAM.


	.data : AT(__xROM_data_start) # load address is in ROM
	{                             # starting after constant data
	    # data sections
	    
	    # values inside this section represent relocated data
	    __xRAM_data_start =.; 
	    
		# note: 
		# for troubleshooting or other uses, you can directly
		# write data using the LCF and then use the memory window
		# to check results
		
		# WRITEH 0x0050;
        
	    
        * (.data)
	    * (fp_state.data)
		* (rtlib.data)
					
		__xRAM_data_end = .;
		__data_size = __xRAM_data_end - __xRAM_data_start;
			
		
		
		# .bss sections
		
	  	* (rtlib.bss.lo)
	  	
	  	__bss_start = .;
	  	
		* (.bss)
		
		__bss_end   = .;
		__bss_size = __bss_end - __bss_start;



		# setup the heap address

		__heap_addr = .;
		__heap_size = 0x0100;
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
		
		
		# stationery init code uses these globals:
		F_data_size     = __data_size;
		F_data_RAM_addr = __xRAM_data_start;
		F_data_ROM_addr = __xROM_data_start;
		F_bss_size      = __bss_size;
		F_bss_addr      = __bss_start;
		
        F_rom_to_ram    =  0x0001; # non-zero is true
	
		
	} > .x_internal_RAM   # relocation address -- 
	                      # intended final destination in RAM

	# boot application start address
    FbootStart = 0xF804;                    
}





# -------------------------------------------------------
# additional notes:


# about the reserved sections
# for this internal RAM and flash ROM LCF:

# p_interrupts_ROM -- reserved in pROM
# memory space reserved for interrupt vectors
# interrupt vectors must start at address zero
# interrupt vector space size is 0x80

# x_compiler_regs_iRAM -- reserved in internal xRAM
# The compiler uses page 0 address locations 0x30-0x40 
# as register variables. See the Target manual for more info.

# other memory sections are per chip




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

