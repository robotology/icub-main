/*	xmmx.h

	MultiMedia eXtensions GCC interface library for IA32.

	To use this library, simply include this header file
	and compile with GCC.  You MUST have inlining enabled
	in order for mmx_ok() to work; this can be done by
	simply using -O on the GCC command line.

	Compiling with -DMMX_TRACE will cause detailed trace
	output to be sent to stderr for each mmx operation.
	This adds lots of code, and obviously slows execution to
	a crawl, but can be very useful for debugging.

	THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY
	EXPRESS OR IMPLIED WARRANTIES, INCLUDING, WITHOUT
	LIMITATION, THE IMPLIED WARRANTIES OF MERCHANTABILITY
	AND FITNESS FOR ANY PARTICULAR PURPOSE.

	1997-99 by H. Dietz and R. Fisher

	Extended from mmx.h to use 128 bit numbers by Niklas Pettersson
 Notes:
	It appears that the latest gas has the pand problem fixed, therefore
	  I'll undefine BROKEN_PAND by default.
*/

#ifndef _XMMX_H
#define _XMMX_H



/*	The type of an value that fits in an XMMX register
	(note that long long constant values MUST be suffixed
	 by LL and unsigned long long values by ULL, lest
	 they be truncated by the compiler)
*/
typedef	union {
	long long		q[2];	/* Quadword (64-bit) value */
	unsigned long long	uq[2];	/* Unsigned Quadword */
	int			d[4];	/* 4 Doubleword (32-bit) values */
	unsigned int		ud[4];	/* 4 Unsigned Doubleword */
	short			w[8];	/* 8 Word (16-bit) values */
	unsigned short		uw[8];	/* 8 Unsigned Word */
	char			b[16];	/* 16 Byte (8-bit) values */
	unsigned char		ub[16];	/* 16 Unsigned Byte */
	float			s[4];	/* Single-precision (32-bit) value */
} __attribute__ ((aligned (16))) xmmx_t;	/* On an 8-byte (64-bit) boundary */


/*	Function to test if multimedia instructions are supported...
*/
inline extern int
xmmx_support(void)
{
	/* Returns 1 if MMX instructions are supported,
	   3 if Cyrix MMX and Extended MMX instructions are supported
	   5 if AMD MMX and 3DNow! instructions are supported
	   0 if hardware does not support any of these
	*/
	register int rval = 0;

	__asm__ __volatile__ (
		/* See if CPUID instruction is supported ... */
		/* ... Get copies of EFLAGS into eax and ecx */
		"pushf\n\t"
		"popl %%eax\n\t"
		"movl %%eax, %%ecx\n\t"

		/* ... Toggle the ID bit in one copy and store */
		/*     to the EFLAGS reg */
		"xorl $0x200000, %%eax\n\t"
		"push %%eax\n\t"
		"popf\n\t"

		/* ... Get the (hopefully modified) EFLAGS */
		"pushf\n\t"
		"popl %%eax\n\t"

		/* ... Compare and test result */
		"xorl %%eax, %%ecx\n\t"
		"testl $0x200000, %%ecx\n\t"
		"jz NotSupported1\n\t"		/* CPUID not supported */


		/* Get standard CPUID information, and
		       go to a specific vendor section */
		"movl $0, %%eax\n\t"
		"cpuid\n\t"

		/* Check for Intel */
		"cmpl $0x756e6547, %%ebx\n\t"
		"jne TryAMD\n\t"
		"cmpl $0x49656e69, %%edx\n\t"
		"jne TryAMD\n\t"
		"cmpl $0x6c65746e, %%ecx\n"
		"jne TryAMD\n\t"
		"jmp Intel\n\t"

		/* Check for AMD */
		"\nTryAMD:\n\t"
		"cmpl $0x68747541, %%ebx\n\t"
		"jne TryCyrix\n\t"
		"cmpl $0x69746e65, %%edx\n\t"
		"jne TryCyrix\n\t"
		"cmpl $0x444d4163, %%ecx\n"
		"jne TryCyrix\n\t"
		"jmp AMD\n\t"

		/* Check for Cyrix */
		"\nTryCyrix:\n\t"
		"cmpl $0x69727943, %%ebx\n\t"
		"jne NotSupported2\n\t"
		"cmpl $0x736e4978, %%edx\n\t"
		"jne NotSupported3\n\t"
		"cmpl $0x64616574, %%ecx\n\t"
		"jne NotSupported4\n\t"
		/* Drop through to Cyrix... */


		/* Cyrix Section */
		/* See if extended CPUID level 80000001 is supported */
		/* The value of CPUID/80000001 for the 6x86MX is undefined
		   according to the Cyrix CPU Detection Guide (Preliminary
		   Rev. 1.01 table 1), so we'll check the value of eax for
		   CPUID/0 to see if standard CPUID level 2 is supported.
		   According to the table, the only CPU which supports level
		   2 is also the only one which supports extended CPUID levels.
		*/
		"cmpl $0x2, %%eax\n\t"
		"jne MMXtest\n\t"	/* Use standard CPUID instead */

		/* Extended CPUID supported (in theory), so get extended
		   features */
		"movl $0x80000001, %%eax\n\t"
		"cpuid\n\t"
		"testl $0x00800000, %%eax\n\t"	/* Test for MMX */
		"jz NotSupported5\n\t"		/* MMX not supported */
		"testl $0x01000000, %%eax\n\t"	/* Test for Ext'd MMX */
		"jnz EMMXSupported\n\t"
		"movl $1, %0:\n\n\t"		/* MMX Supported */
		"jmp Return\n\n"
		"EMMXSupported:\n\t"
		"movl $3, %0:\n\n\t"		/* EMMX and MMX Supported */
		"jmp Return\n\t"


		/* AMD Section */
		"AMD:\n\t"

		/* See if extended CPUID is supported */
		"movl $0x80000000, %%eax\n\t"
		"cpuid\n\t"
		"cmpl $0x80000000, %%eax\n\t"
		"jl MMXtest\n\t"	/* Use standard CPUID instead */

		/* Extended CPUID supported, so get extended features */
		"movl $0x80000001, %%eax\n\t"
		"cpuid\n\t"
		"testl $0x00800000, %%edx\n\t"	/* Test for MMX */
		"jz NotSupported6\n\t"		/* MMX not supported */
		"testl $0x80000000, %%edx\n\t"	/* Test for 3DNow! */
		"jnz ThreeDNowSupported\n\t"
		"movl $1, %0:\n\n\t"		/* MMX Supported */
		"jmp Return\n\n"
		"ThreeDNowSupported:\n\t"
		"movl $5, %0:\n\n\t"		/* 3DNow! and MMX Supported */
		"jmp Return\n\t"


		/* Intel Section */
		"Intel:\n\t"

		/* Check for MMX */
		"MMXtest:\n\t"
		"movl $1, %%eax\n\t"
		"cpuid\n\t"
		"testl $0x00800000, %%edx\n\t"	/* Test for MMX */
		"jz NotSupported7\n\t"		/* MMX Not supported */
		"movl $1, %0:\n\n\t"		/* MMX Supported */
		"jmp Return\n\t"

		/* Nothing supported */
		"\nNotSupported1:\n\t"
		"#movl $101, %0:\n\n\t"
		"\nNotSupported2:\n\t"
		"#movl $102, %0:\n\n\t"
		"\nNotSupported3:\n\t"
		"#movl $103, %0:\n\n\t"
		"\nNotSupported4:\n\t"
		"#movl $104, %0:\n\n\t"
		"\nNotSupported5:\n\t"
		"#movl $105, %0:\n\n\t"
		"\nNotSupported6:\n\t"
		"#movl $106, %0:\n\n\t"
		"\nNotSupported7:\n\t"
		"#movl $107, %0:\n\n\t"
		"movl $0, %0:\n\n\t"

		"Return:\n\t"
		: "=a" (rval)
		: /* no input */
		: "eax", "ebx", "ecx", "edx"
	);

	/* Return */
	return(rval);
}

/*	Function to test if mmx instructions are supported...
*/
inline extern int
xmmx_ok(void)
{
	/* Returns 1 if MMX instructions are supported, 0 otherwise */
	return ( mmx_support() & 0x1 );
}


/*	Helper functions for the instruction macros that follow...
	(note that memory-to-register, m2r, instructions are nearly
	 as efficient as register-to-register, r2r, instructions;
	 however, memory-to-memory instructions are really simulated
	 as a convenience, and are only 1/3 as efficient)
*/
#ifdef	MMX_TRACE

/*	Include the stuff for printing a trace to stderr...
*/

#include <stdio.h>

#define	xmmx_i2r(op, imm, reg) \
	{ \
		xmmx_t mmx_trace; \
		mmx_trace.uq[0] = (imm);mmx_trace.uq[1] = 0; \
		fprintf(stderr, #op "_i2r(" #imm "=0x%08x%08x%08x%08x,\t", \
			mmx_trace.d[3], mmx_trace.d[2],mmx_trace.d[1], mmx_trace.d[0]); \
		__asm__ __volatile__ ("movdqa %%" #reg ", %0" \
				      : "=m" (mmx_trace) \
				      : /* nothing */ ); \
		fprintf(stderr, #reg "=0x%08x%08x%08x%08x) =>\t", \
			mmx_trace.d[3], mmx_trace.d[2],mmx_trace.d[1], mmx_trace.d[0]); \
		__asm__ __volatile__ (#op " %0, %%" #reg \
				      : /* nothing */ \
				      : "X" (imm)); \
		__asm__ __volatile__ ("movdqa %%" #reg ", %0" \
				      : "=m" (mmx_trace) \
				      : /* nothing */ ); \
		fprintf(stderr, #reg "=0x%08x%08x%08x%08x\n", \
			mmx_trace.d[3], mmx_trace.d[2],mmx_trace.d[1], mmx_trace.d[0]); \
	}

#define	xmmx_m2r(op, mem, reg) \
	{ \
		xmmx_t mmx_trace; \
		mmx_trace = (mem); \
		fprintf(stderr, #op "_m2r(" #mem "=0x%08x%08x%08x%08x,\t", \
			mmx_trace.d[3], mmx_trace.d[2],mmx_trace.d[1], mmx_trace.d[0]); \
		__asm__ __volatile__ ("movdqa %%" #reg ", %0" \
				      : "=m" (mmx_trace) \
				      : /* nothing */ ); \
		fprintf(stderr, #reg "=0x%08x%08x%08x%08x) =>\t", \
			mmx_trace.d[3], mmx_trace.d[2],mmx_trace.d[1], mmx_trace.d[0]); \
		__asm__ __volatile__ (#op " %0, %%" #reg \
				      : /* nothing */ \
				      : "m" (mem)); \
		__asm__ __volatile__ ("movdqa %%" #reg ", %0" \
				      : "=m" (mmx_trace) \
				      : /* nothing */ ); \
		fprintf(stderr, #reg "=0x%08x%08x%08x%08x\n", \
			mmx_trace.d[3], mmx_trace.d[2],mmx_trace.d[1], mmx_trace.d[0]); \
	}

#define	xmmx_r2m(op, reg, mem) \
	{ \
		xmmx_t mmx_trace; \
		__asm__ __volatile__ ("movdqa %%" #reg ", %0" \
				      : "=m" (mmx_trace) \
				      : /* nothing */ ); \
		fprintf(stderr, #op "_r2m(" #reg "=0x%08x%08x%08x%08x,\t", \
			mmx_trace.d[3], mmx_trace.d[2],mmx_trace.d[1], mmx_trace.d[0]); \
		mmx_trace = (mem); \
		fprintf(stderr, #mem "=0x%08x%08x%08x%08x) =>\t", \
			mmx_trace.d[3], mmx_trace.d[2],mmx_trace.d[1], mmx_trace.d[0]); \
		__asm__ __volatile__ (#op " %%" #reg ", %0" \
				      : "=m" (mem) \
				      : /* nothing */ ); \
		mmx_trace = (mem); \
		fprintf(stderr, #mem "=0x%08x%08x%08x%08x\n", \
			mmx_trace.d[3], mmx_trace.d[2],mmx_trace.d[1], mmx_trace.d[0]); \
	}

#define	xmmx_r2r(op, regs, regd) \
	{ \
		xmmx_t mmx_trace; \
		__asm__ __volatile__ ("movdqa %%" #regs ", %0" \
				      : "=m" (mmx_trace) \
				      : /* nothing */ ); \
		fprintf(stderr, #op "_r2r(" #regs "=0x%08x%08x%08x%08x,\t", \
			mmx_trace.d[3], mmx_trace.d[2],mmx_trace.d[1], mmx_trace.d[0]); \
		__asm__ __volatile__ ("movdqa %%" #regd ", %0" \
				      : "=m" (mmx_trace) \
				      : /* nothing */ ); \
		fprintf(stderr, #regd "=0x%08x%08x%08x%08x) =>\t", \
			mmx_trace.d[3], mmx_trace.d[2],mmx_trace.d[1], mmx_trace.d[0]); \
		__asm__ __volatile__ (#op " %" #regs ", %" #regd); \
		__asm__ __volatile__ ("movdqa %%" #regd ", %0" \
				      : "=m" (mmx_trace) \
				      : /* nothing */ ); \
		fprintf(stderr, #regd "=0x%08x%08x%08x%08x\n", \
			mmx_trace.d[3], mmx_trace.d[2],mmx_trace.d[1], mmx_trace.d[0]); \
	}

#define	xmmx_m2m(op, mems, memd) \
	{ \
		xmmx_t mmx_trace; \
		mmx_trace = (mems); \
		fprintf(stderr, #op "_m2m(" #mems "=0x%08x%08x%08x%08x, ", \
			mmx_trace.d[3], mmx_trace.d[2],mmx_trace.d[1], mmx_trace.d[0]); \
		mmx_trace = (memd); \
		fprintf(stderr, #memd "=0x%08x%08x%08x%08x) => ", \
			mmx_trace.d[3], mmx_trace.d[2],mmx_trace.d[1], mmx_trace.d[0]); \
		__asm__ __volatile__ ("movdqa %0, %%mm0\n\t" \
				      #op " %1, %%mm0\n\t" \
				      "movdqa %%mm0, %0" \
				      : "=m" (memd) \
				      : "m" (mems)); \
		mmx_trace = (memd); \
		fprintf(stderr, #memd "=0x%08x%08x%08x%08x\n", \
			mmx_trace.d[3], mmx_trace.d[2],mmx_trace.d[1], mmx_trace.d[0]); \
	}

#else

/*	These macros are a lot simpler without the tracing...
*/

#define	xmmx_i2r(op, imm, reg) \
	__asm__ __volatile__ (#op " %0, %%" #reg \
			      : /* nothing */ \
			      : "X" (imm) )

#define	xmmx_m2r(op, mem, reg) \
	__asm__ __volatile__ (#op " %0, %%" #reg \
			      : /* nothing */ \
			      : "m" (mem))

#define	xmmx_r2m(op, reg, mem) \
	__asm__ __volatile__ (#op " %%" #reg ", %0" \
			      : "=m" (mem) \
			      : /* nothing */ )

#define	xmmx_r2r(op, regs, regd) \
	__asm__ __volatile__ (#op " %" #regs ", %" #regd)

#define	xmmx_m2m(op, mems, memd) \
	__asm__ __volatile__ ("movq %0, %%mm0\n\t" \
			      #op " %1, %%mm0\n\t" \
			      "movq %%mm0, %0" \
			      : "=m" (memd) \
			      : "m" (mems))

#endif


/*	1x128 MOVe Quadword
	(this is both a load and a store...
	 in fact, it is the only way to store)
*/
#define	xmovq_m2r(var, reg)	xmmx_m2r(movdqa, var, reg)
#define	xmovq_r2m(reg, var)	xmmx_r2m(movdqa, reg, var)
#define	xmovq_r2r(regs, regd)	xmmx_r2r(movdqa, regs, regd)
#define	xmovq(vars, vard) \
	__asm__ __volatile__ ("movdqa %1, %%mm0\n\t" \
			      "movdqa %%mm0, %0" \
			      : "=m" (vard) \
			      : "m" (vars))


/*	1x32 MOVe Doubleword
	(like movq, this is both load and store...
	 but is most useful for moving things between
	 mmx registers and ordinary registers)
*/
#define	xmovd_m2r(var, reg)	xmmx_m2r(movd, var, reg)
#define	xmovd_r2m(reg, var)	xmmx_r2m(movd, reg, var)
#define	xmovd_r2r(regs, regd)	xmmx_r2r(movd, regs, regd)
#define	xmovd(vars, vard) \
	__asm__ __volatile__ ("movd %1, %%mm0\n\t" \
			      "movd %%mm0, %0" \
			      : "=m" (vard) \
			      : "m" (vars))


/*	4x32, 8x16, and 16x8 Parallel ADDs
*/
#define	xpaddd_m2r(var, reg)	xmmx_m2r(paddd, var, reg)
#define	xpaddd_r2r(regs, regd)	xmmx_r2r(paddd, regs, regd)
#define	xpaddd(vars, vard)	xmmx_m2m(paddd, vars, vard)

#define	xpaddw_m2r(var, reg)	xmmx_m2r(paddw, var, reg)
#define	xpaddw_r2r(regs, regd)	xmmx_r2r(paddw, regs, regd)
#define	xpaddw(vars, vard)	xmmx_m2m(paddw, vars, vard)

#define	xpaddb_m2r(var, reg)	xmmx_m2r(paddb, var, reg)
#define	xpaddb_r2r(regs, regd)	xmmx_r2r(paddb, regs, regd)
#define	xpaddb(vars, vard)	xmmx_m2m(paddb, vars, vard)


/*	8x16 and 16x8 Parallel ADDs using Saturation arithmetic
*/
#define	xpaddsw_m2r(var, reg)	xmmx_m2r(paddsw, var, reg)
#define	xpaddsw_r2r(regs, regd)	xmmx_r2r(paddsw, regs, regd)
#define	xpaddsw(vars, vard)	xmmx_m2m(paddsw, vars, vard)

#define	xpaddsb_m2r(var, reg)	xmmx_m2r(paddsb, var, reg)
#define	xpaddsb_r2r(regs, regd)	xmmx_r2r(paddsb, regs, regd)
#define	xpaddsb(vars, vard)	xmmx_m2m(paddsb, vars, vard)


/*	8x16 and 16x8 Parallel ADDs using Unsigned Saturation arithmetic
*/
#define	xpaddusw_m2r(var, reg)		xmmx_m2r(paddusw, var, reg)
#define	xpaddusw_r2r(regs, regd)	xmmx_r2r(paddusw, regs, regd)
#define	xpaddusw(vars, vard)		xmmx_m2m(paddusw, vars, vard)

#define	xpaddusb_m2r(var, reg)		xmmx_m2r(paddusb, var, reg)
#define	xpaddusb_r2r(regs, regd)	xmmx_r2r(paddusb, regs, regd)
#define	xpaddusb(vars, vard)		xmmx_m2m(paddusb, vars, vard)


/*	4x32, 8x16, and 16x8 Parallel SUBs
*/
#define	xpsubd_m2r(var, reg)	xmmx_m2r(psubd, var, reg)
#define	xpsubd_r2r(regs, regd)	xmmx_r2r(psubd, regs, regd)
#define	xpsubd(vars, vard)	xmmx_m2m(psubd, vars, vard)

#define	xpsubw_m2r(var, reg)	xmmx_m2r(psubw, var, reg)
#define	xpsubw_r2r(regs, regd)	xmmx_r2r(psubw, regs, regd)
#define	xpsubw(vars, vard)	xmmx_m2m(psubw, vars, vard)

#define	xpsubb_m2r(var, reg)	xmmx_m2r(psubb, var, reg)
#define	xpsubb_r2r(regs, regd)	xmmx_r2r(psubb, regs, regd)
#define	xpsubb(vars, vard)	xmmx_m2m(psubb, vars, vard)


/*	8x16 and 16x8 Parallel SUBs using Saturation arithmetic
*/
#define	xpsubsw_m2r(var, reg)	xmmx_m2r(psubsw, var, reg)
#define	xpsubsw_r2r(regs, regd)	xmmx_r2r(psubsw, regs, regd)
#define	xpsubsw(vars, vard)	xmmx_m2m(psubsw, vars, vard)

#define	xpsubsb_m2r(var, reg)	xmmx_m2r(psubsb, var, reg)
#define	xpsubsb_r2r(regs, regd)	xmmx_r2r(psubsb, regs, regd)
#define	xpsubsb(vars, vard)	xmmx_m2m(psubsb, vars, vard)


/*	8x16 and 16x8 Parallel SUBs using Unsigned Saturation arithmetic
*/
#define	xpsubusw_m2r(var, reg)		xmmx_m2r(psubusw, var, reg)
#define	xpsubusw_r2r(regs, regd)	xmmx_r2r(psubusw, regs, regd)
#define	xpsubusw(vars, vard)		xmmx_m2m(psubusw, vars, vard)

#define	xpsubusb_m2r(var, reg)		xmmx_m2r(psubusb, var, reg)
#define	xpsubusb_r2r(regs, regd)	xmmx_r2r(psubusb, regs, regd)
#define	xpsubusb(vars, vard)		xmmx_m2m(psubusb, vars, vard)


/*	8x16 Parallel MULs giving Low 8x16 portions of results
*/
#define	xpmullw_m2r(var, reg)	xmmx_m2r(pmullw, var, reg)
#define	xpmullw_r2r(regs, regd)	xmmx_r2r(pmullw, regs, regd)
#define	xpmullw(vars, vard)	xmmx_m2m(pmullw, vars, vard)


/*	8x16 Parallel MULs giving High 8x16 portions of results
*/
#define	xpmulhw_m2r(var, reg)	xmmx_m2r(pmulhw, var, reg)
#define	xpmulhw_r2r(regs, regd)	xmmx_r2r(pmulhw, regs, regd)
#define	xpmulhw(vars, vard)	xmmx_m2m(pmulhw, vars, vard)


/*	8x16->4x32 Parallel Mul-ADD
	(muls like pmullw, then adds adjacent 16-bit fields
	 in the multiply result to make the final 4x32 result)
*/
#define	xpmaddwd_m2r(var, reg)		xmmx_m2r(pmaddwd, var, reg)
#define	xpmaddwd_r2r(regs, regd)	xmmx_r2r(pmaddwd, regs, regd)
#define	xpmaddwd(vars, vard)		xmmx_m2m(pmaddwd, vars, vard)


/*	1x128 bitwise AND
*/
#define	xpand_m2r(var, reg)	xmmx_m2r(pand, var, reg)
#define	xpand_r2r(regs, regd)	xmmx_r2r(pand, regs, regd)
#define	xpand(vars, vard)	xmmx_m2m(pand, vars, vard)


/*	1x128 bitwise AND with Not the destination
*/
#define	xpandn_m2r(var, reg)	xmmx_m2r(pandn, var, reg)
#define	xpandn_r2r(regs, regd)	xmmx_r2r(pandn, regs, regd)
#define	xpandn(vars, vard)	xmmx_m2m(pandn, vars, vard)


/*	1x128 bitwise OR
*/
#define	xpor_m2r(var, reg)	xmmx_m2r(por, var, reg)
#define	xpor_r2r(regs, regd)	xmmx_r2r(por, regs, regd)
#define	xpor(vars, vard)	xmmx_m2m(por, vars, vard)


/*	1x128 bitwise eXclusive OR
*/
#define	xpxor_m2r(var, reg)	xmmx_m2r(pxor, var, reg)
#define	xpxor_r2r(regs, regd)	xmmx_r2r(pxor, regs, regd)
#define	xpxor(vars, vard)	xmmx_m2m(pxor, vars, vard)


/*	4x32, 8x16, and 16x8 Parallel CoMPare for EQuality
	(resulting fields are either 0 or -1)
*/
#define	xpcmpeqd_m2r(var, reg)		xmmx_m2r(pcmpeqd, var, reg)
#define	xpcmpeqd_r2r(regs, regd)	xmmx_r2r(pcmpeqd, regs, regd)
#define	xpcmpeqd(vars, vard)		xmmx_m2m(pcmpeqd, vars, vard)

#define	xpcmpeqw_m2r(var, reg)		xmmx_m2r(pcmpeqw, var, reg)
#define	xpcmpeqw_r2r(regs, regd)	xmmx_r2r(pcmpeqw, regs, regd)
#define	xpcmpeqw(vars, vard)		xmmx_m2m(pcmpeqw, vars, vard)

#define	xpcmpeqb_m2r(var, reg)		xmmx_m2r(pcmpeqb, var, reg)
#define	xpcmpeqb_r2r(regs, regd)	xmmx_r2r(pcmpeqb, regs, regd)
#define	xpcmpeqb(vars, vard)		xmmx_m2m(pcmpeqb, vars, vard)


/*	4x32, 8x16, and 16x8 Parallel CoMPare for Greater Than
	(resulting fields are either 0 or -1)
*/
#define	xpcmpgtd_m2r(var, reg)		xmmx_m2r(pcmpgtd, var, reg)
#define	xpcmpgtd_r2r(regs, regd)	mmx_r2r(pcmpgtd, regs, regd)
#define	xpcmpgtd(vars, vard)		xmmx_m2m(pcmpgtd, vars, vard)

#define	xpcmpgtw_m2r(var, reg)		xmmx_m2r(pcmpgtw, var, reg)
#define	xpcmpgtw_r2r(regs, regd)	xmmx_r2r(pcmpgtw, regs, regd)
#define	xpcmpgtw(vars, vard)		xmmx_m2m(pcmpgtw, vars, vard)

#define	xpcmpgtb_m2r(var, reg)		xmmx_m2r(pcmpgtb, var, reg)
#define	xpcmpgtb_r2r(regs, regd)	xmmx_r2r(pcmpgtb, regs, regd)
#define	xpcmpgtb(vars, vard)		xmmx_m2m(pcmpgtb, vars, vard)


/*	1x128, 4x32, and 8x16 Parallel Shift Left Logical
*/
#define	xpslldq_i2r(imm, reg)	xmmx_i2r(pslldq, imm, reg)
#define	xpslldq_m2r(var, reg)	xmmx_m2r(pslldq, var, reg)
#define	xpslldq_r2r(regs, regd)	xmmx_r2r(pslldq, regs, regd)
#define	xpslldq(vars, vard)	xmmx_m2m(pslldq, vars, vard)

#define	xpsllq_i2r(imm, reg)	xmmx_i2r(psllq, imm, reg)
#define	xpsllq_m2r(var, reg)	xmmx_m2r(psllq, var, reg)
#define	xpsllq_r2r(regs, regd)	xmmx_r2r(psllq, regs, regd)
#define	xpsllq(vars, vard)	xmmx_m2m(psllq, vars, vard)

#define	xpslld_i2r(imm, reg)	xmmx_i2r(pslld, imm, reg)
#define	xpslld_m2r(var, reg)	xmmx_m2r(pslld, var, reg)
#define	xpslld_r2r(regs, regd)	xmmx_r2r(pslld, regs, regd)
#define	xpslld(vars, vard)	xmmx_m2m(pslld, vars, vard)

#define	xpsllw_i2r(imm, reg)	xmmx_i2r(psllw, imm, reg)
#define	xpsllw_m2r(var, reg)	xmmx_m2r(psllw, var, reg)
#define	xpsllw_r2r(regs, regd)	xmmx_r2r(psllw, regs, regd)
#define	xpsllw(vars, vard)	xmmx_m2m(psllw, vars, vard)


/*	1x128, 4x32, and 8x16 Parallel Shift Right Logical
*/
#define	xpsrldq_i2r(imm, reg)	xmmx_i2r(psrldq, imm, reg)
#define	xpsrldq_m2r(var, reg)	xmmx_m2r(psrldq, var, reg)
#define	xpsrldq_r2r(regs, regd)	xmmx_r2r(psrldq, regs, regd)
#define	xpsrldq(vars, vard)	xmmx_m2m(psrldq, vars, vard)

#define	xpsrlq_i2r(imm, reg)	xmmx_i2r(psrlq, imm, reg)
#define	xpsrlq_m2r(var, reg)	xmmx_m2r(psrlq, var, reg)
#define	xpsrlq_r2r(regs, regd)	xmmx_r2r(psrlq, regs, regd)
#define	xpsrlq(vars, vard)	xmmx_m2m(psrlq, vars, vard)

#define	xpsrld_i2r(imm, reg)	xmmx_i2r(psrld, imm, reg)
#define	xpsrld_m2r(var, reg)	xmmx_m2r(psrld, var, reg)
#define	xpsrld_r2r(regs, regd)	xmmx_r2r(psrld, regs, regd)
#define	xpsrld(vars, vard)	xmmx_m2m(psrld, vars, vard)

#define	xpsrlw_i2r(imm, reg)	xmmx_i2r(psrlw, imm, reg)
#define	xpsrlw_m2r(var, reg)	xmmx_m2r(psrlw, var, reg)
#define	xpsrlw_r2r(regs, regd)	xmmx_r2r(psrlw, regs, regd)
#define	xpsrlw(vars, vard)	xmmx_m2m(psrlw, vars, vard)


/*	4x32 and 8x16 Parallel Shift Right Arithmetic
*/
#define	xpsrad_i2r(imm, reg)	xmmx_i2r(psrad, imm, reg)
#define	xpsrad_m2r(var, reg)	xmmx_m2r(psrad, var, reg)
#define	xpsrad_r2r(regs, regd)	xmmx_r2r(psrad, regs, regd)
#define	xpsrad(vars, vard)	xmmx_m2m(psrad, vars, vard)

#define	xpsraw_i2r(imm, reg)	xmmx_i2r(psraw, imm, reg)
#define	xpsraw_m2r(var, reg)	xmmx_m2r(psraw, var, reg)
#define	xpsraw_r2r(regs, regd)	xmmx_r2r(psraw, regs, regd)
#define	xpsraw(vars, vard)	xmmx_m2m(psraw, vars, vard)


/*	4x32->8x16 and 8x16->16x8 PACK and Signed Saturate
	(packs source and dest fields into dest in that order)
*/
#define	xpackssdw_m2r(var, reg)	xmmx_m2r(packssdw, var, reg)
#define	xpackssdw_r2r(regs, regd) xmmx_r2r(packssdw, regs, regd)
#define	xpackssdw(vars, vard)	xmmx_m2m(packssdw, vars, vard)

#define	xpacksswb_m2r(var, reg)	xmmx_m2r(packsswb, var, reg)
#define	xpacksswb_r2r(regs, regd) xmmx_r2r(packsswb, regs, regd)
#define	xpacksswb(vars, vard)	xmmx_m2m(packsswb, vars, vard)


/*	8x16->16x8 PACK and Unsigned Saturate
	(packs source and dest fields into dest in that order)
*/
#define	xpackuswb_m2r(var, reg)	xmmx_m2r(packuswb, var, reg)
#define	xpackuswb_r2r(regs, regd) xmmx_r2r(packuswb, regs, regd)
#define	xpackuswb(vars, vard)	xmmx_m2m(packuswb, vars, vard)


/*	2x32->1x64, 4x16->2x32, and 8x8->4x16 UNPaCK Low
	(interleaves low half of dest with low half of source
	 as padding in each result field)
*/
#define	xpunpckldq_m2r(var, reg)	xmmx_m2r(punpckldq, var, reg)
#define	xpunpckldq_r2r(regs, regd) xmmx_r2r(punpckldq, regs, regd)
#define	xpunpckldq(vars, vard)	xmmx_m2m(punpckldq, vars, vard)

#define	xpunpcklwd_m2r(var, reg)	xmmx_m2r(punpcklwd, var, reg)
#define	xpunpcklwd_r2r(regs, regd) xmmx_r2r(punpcklwd, regs, regd)
#define	xpunpcklwd(vars, vard)	xmmx_m2m(punpcklwd, vars, vard)

#define	xpunpcklbw_m2r(var, reg)	xmmx_m2r(punpcklbw, var, reg)
#define	xpunpcklbw_r2r(regs, regd) xmmx_r2r(punpcklbw, regs, regd)
#define	xpunpcklbw(vars, vard)	xmmx_m2m(punpcklbw, vars, vard)


/*	2x32->1x64, 4x16->2x32, and 8x8->4x16 UNPaCK High
	(interleaves high half of dest with high half of source
	 as padding in each result field)
*/
#define	xpunpckhdq_m2r(var, reg)	xmmx_m2r(punpckhdq, var, reg)
#define	xpunpckhdq_r2r(regs, regd) xmmx_r2r(punpckhdq, regs, regd)
#define	xpunpckhdq(vars, vard)	xmmx_m2m(punpckhdq, vars, vard)

#define	xpunpckhwd_m2r(var, reg)	xmmx_m2r(punpckhwd, var, reg)
#define	xpunpckhwd_r2r(regs, regd) xmmx_r2r(punpckhwd, regs, regd)
#define	xpunpckhwd(vars, vard)	xmmx_m2m(punpckhwd, vars, vard)

#define	xpunpckhbw_m2r(var, reg)	xmmx_m2r(punpckhbw, var, reg)
#define	xpunpckhbw_r2r(regs, regd) xmmx_r2r(punpckhbw, regs, regd)
#define	xpunpckhbw(vars, vard)	xmmx_m2m(punpckhbw, vars, vard)


/*	Empty MMx State
	(used to clean-up when going from mmx to float use
	 of the registers that are shared by both; note that
	 there is no float-to-mmx operation needed, because
	 only the float tag word info is corruptible)
*/
#ifdef	MMX_TRACE

#define	emms() \
	{ \
		fprintf(stderr, "emms()\n"); \
		__asm__ __volatile__ ("emms"); \
	}

#else

#define	emms()			__asm__ __volatile__ ("emms")

#endif

#endif

