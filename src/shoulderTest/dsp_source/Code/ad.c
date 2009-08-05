/*
 * ad.c
 *	reading from the AD converter.
 */

#include "ad.h"
#include "controller.h"

#define IDLE            0              /* IDLE state           */
#define MEASURE         1              /* MESURE state         */
#define CONTINUOUS      2              /* CONTINUOUS state     */
#define SINGLE          3              /* SINGLE state         */

static bool OutFlgA;                    /* Measurement finish flag */
volatile byte ad_ModeFlgA;	           /* Current state of device */
static bool OutFlgB;                    /* Measurement finish flag */
volatile byte ad_ModeFlgB;	           /* Current state of device */

word _sample_A[3];					   /* 3 channels */
word _sample_B[3];

/**
 * on interrupt flags the output ready bit.
 */
#pragma interrupt 
void AD_interruptCCA(void)
{
	_sample_A[0] = (getReg(ADCA_ADRSLT0));
	_sample_A[1] = (getReg(ADCA_ADRSLT1));
	_sample_A[2] = (getReg(ADCA_ADRSLT2));

	setReg(ADCA_ADSTAT, 0x0800);            /* Clear EOSI flag */
	
	OutFlgA = TRUE;                       	/* Measured values are available */
	if (!(getRegBit(ADCA_ADCR1, SMODE2))) 
	{
		/* Not running in trigger mode? */
		ad_ModeFlgA = IDLE;                	/* Set the bean to the idle mode */
	}
}

/**
 * on interrupt flags the output ready bit.
 */
#pragma interrupt 
void AD_interruptCCB(void)
{
	_sample_B[0] = (getReg(ADCB_ADRSLT0));
	_sample_B[1] = (getReg(ADCB_ADRSLT1));
	_sample_B[2] = (getReg(ADCB_ADRSLT2));

	setReg(ADCB_ADSTAT, 0x0800);            /* Clear EOSI flag */
	
	OutFlgB = TRUE;                       	/* Measured values are available */
	if (!(getRegBit(ADCB_ADCR1, SMODE2))) 
	{
		/* Not running in trigger mode? */
		ad_ModeFlgB = IDLE;                	/* Set the bean to the idle mode */
	}
}

/**
 * starts the acquisition.
 */
static void HWEnDiA(void)
{
	if (ad_ModeFlgA)  /* Launch measurement? */
	{
		OutFlgA = FALSE;                    	  /* Measured values are available */
		/* Trigger mode? */
		if (getRegBit (ADCA_ADCR1, SMODE2)) 
		{ 
			setRegBit (ADCA_ADCR1, SYNC);     /* Use sync input to initiate a conversion */
			clrRegBit (ADCA_ADCR1, STOP);     /* Normal operation mode */
		}
		else 
		{
			/* Set normal operation mode and sync input disabled */
			clrRegBits (ADCA_ADCR1, ADCA_ADCR1_SYNC_MASK | ADCA_ADCR1_STOP_MASK); 
			setRegBit (ADCA_ADCR1, START);    /* Launching of conversion */
		}
	}
	else 
	{
		setRegBit (ADCA_ADCR1, STOP);         /* Stop command issued */
	}
}

/**
 * starts the acquisition.
 */
static void HWEnDiB(void)
{
	if (ad_ModeFlgB)  /* Launch measurement? */
	{
		OutFlgB = FALSE;                    	  /* Measured values are available */
		/* Trigger mode? */
		if (getRegBit (ADCB_ADCR1, SMODE2)) 
		{ 
			setRegBit (ADCB_ADCR1, SYNC);     /* Use sync input to initiate a conversion */
			clrRegBit (ADCB_ADCR1, STOP);     /* Normal operation mode */
		}
		else 
		{
			/* Set normal operation mode and sync input disabled */
			clrRegBits (ADCB_ADCR1, ADCB_ADCR1_SYNC_MASK | ADCB_ADCR1_STOP_MASK); 
			setRegBit (ADCB_ADCR1, START);    /* Launching of conversion */
		}
	}
	else 
	{
		setRegBit (ADCB_ADCR1, STOP);         /* Stop command issued */
	}
}

/*
 * This method performs one measurement on all channels that
 *  are set in the bean inspector. (Note: If the <number of
 *  conversions> is more than one the conversion of A/D
 *  channels is performed specified number of times.)
 *
 * @param wait waits for result to be ready.
 * @return ERR_OK after a successful sampling, ERR_BUSY if the device is
 * already running a conversion.
 */
byte AD_measureA(bool wait)
{
	if (ad_ModeFlgA != IDLE)
		return ERR_BUSY;
		
	/* sequential once mode */
	clrRegBits (ADCA_ADCR1, 0x00);
	
	ad_ModeFlgA = MEASURE;
	
	HWEnDiA();
	
	/* wait on the interrupt */
	if (wait)
		while (ad_ModeFlgA == MEASURE) {}
	
	return ERR_OK;
}

/*
 * enables triggered sequential mode synchronous with the
 * PWM generation signal.
 */
byte AD_enableIntTriggerA(void)
{
	if (ad_ModeFlgA != IDLE)             /* Is the device in running mode? */
		return ERR_BUSY;
		
	/// starts sampling in triggered sequential mode
	/// synchro with PWM generation.
	setRegBits (ADCA_ADCR1, 0x04);
	clrRegBits (ADCA_ADCR1, 0x03);
	
	ad_ModeFlgA = MEASURE;               /* Set state of device to the measure mode */
	
	HWEnDiA();
	return ERR_OK;
}

/*
 * enables triggered sequential mode synchronous with the
 * PWM generation signal.
 */
byte AD_enableIntTriggerB(void)
{
	if (ad_ModeFlgB != IDLE)             /* Is the device in running mode? */
		return ERR_BUSY;
		
	/// starts sampling in triggered sequential mode
	/// synchro with PWM generation.
	setRegBits (ADCB_ADCR1, 0x04);
	clrRegBits (ADCB_ADCR1, 0x03);
	
	ad_ModeFlgB = MEASURE;               /* Set state of device to the measure mode */
	
	HWEnDiB();
	return ERR_OK;
}

/*
 * stops the acquisition, disables interrupts.
 * use init to start acquisition again.
 */
byte AD_stopAcquisitionA(void)
{
	setRegBit (ADCA_ADCR1, STOP);         /* Stop command issued */
	return ERR_OK;
}

/**
 * gets the sampled values if available.
 * @param values is a pointer to an array of three elements.
 * @return ERR_OK if values are available, ERR_NOTAVAIL otherwise.
 */
byte AD_getValue16A (word *values)
{
	ADA_DI;
	values[0] = _sample_A[0];
	values[1] = _sample_A[1];
	values[2] = _sample_A[2];
	ADA_EI;
	
	return ERR_OK;
}

/**
 * gets the sampled values if available.
 * @param values is a pointer to an array of three elements.
 * @return ERR_OK if values are available, ERR_NOTAVAIL otherwise.
 */
byte AD_getValue16B (word *values)
{
	ADB_DI;
	values[0] = _sample_B[0];
	values[1] = _sample_B[1];
	values[2] = _sample_B[2];
	ADB_EI;
	
	return ERR_OK;
}

/**
 * gets the sampled values if available.
 * @param i is the index of the channel to read 0-2
 * @param values is a pointer to a 16 bit value.
 * @return ERR_OK if values are available, ERR_NOTAVAIL otherwise.
 */
byte AD_getChannel16A(byte i, word *value)
{
	if (i < 0 || i > 2)
		return ERR_RANGE;
	ADA_DI;
	*value = _sample_A[i];
	ADA_EI;
}

/**
 * gets the sampled values if available.
 * @param i is the index of the channel to read 0-2
 * @param values is a pointer to a 16 bit value.
 * @return ERR_OK if values are available, ERR_NOTAVAIL otherwise.
 */
byte AD_getChannel16B(byte i, word *value)
{
	if (i < 0 || i > 2)
		return ERR_RANGE;
	ADB_DI;
	*value = _sample_B[i];
	ADB_EI;
}

/**
 * initializes the AD conversion module.
 */
void AD_init (void)
{
	OutFlgA = FALSE;                      /* No measured value */
	ad_ModeFlgA = IDLE;                  /* Device isn't running */

	setReg(ADCA_ADCR1, 0x4800);           /* Set control register 1 */
	setReg(ADCA_ADOFS0, 0);               /* Set offset reg. 0 */
	setReg(ADCA_ADOFS1, 0);               /* Set offset reg. 1 */
	setReg(ADCA_ADOFS2, 0);               /* Set offset reg. 2 */
	setReg(ADCA_ADHLMT0, 0x7ff8);         /* Set high limit reg. 0 */
	setReg(ADCA_ADHLMT1, 0x7ff8);         /* Set high limit reg. 1 */
	setReg(ADCA_ADHLMT2, 0x7ff8);         /* Set high limit reg. 2 */
	setReg(ADCA_ADLLMT0, 0);              /* Set low limit reg. 0 */
	setReg(ADCA_ADLLMT1, 0);              /* Set low limit reg. 1 */
	setReg(ADCA_ADLLMT2, 0);              /* Set low limit reg. 2 */
	setReg(ADCA_ADZCSTAT, 0xff);          /* Clear zero crossing status flags */
	setReg(ADCA_ADLSTAT, 0xffff);         /* Clear high and low limit status */
	setReg(ADCA_ADSTAT, 0x800);           /* Clear EOSI flag */
	setReg(ADCA_ADSDIS, 0xf8);            /* Enable/disable of samples */
	setReg(ADCA_ADLST1, 0x105);           /* Set ADC channel list reg. */
	setReg(ADCA_ADZCC, 0);                /* Set zero crossing control reg. */
	setReg(ADCA_ADCR2, 0xf);              /* Set prescaler */

	_sample_A[0] = 0;
	_sample_A[1] = 0;
	_sample_A[2] = 0;
	
	HWEnDiA();

	OutFlgB = FALSE;                      /* No measured value */
	ad_ModeFlgB = IDLE;                  /* Device isn't running */

	setReg(ADCB_ADCR1, 0x4800);           /* Set control register 1 */
	setReg(ADCB_ADOFS0, 0);               /* Set offset reg. 0 */
	setReg(ADCB_ADOFS1, 0);               /* Set offset reg. 1 */
	setReg(ADCB_ADOFS2, 0);               /* Set offset reg. 2 */
	setReg(ADCB_ADHLMT0, 0x7ff8);         /* Set high limit reg. 0 */
	setReg(ADCB_ADHLMT1, 0x7ff8);         /* Set high limit reg. 1 */
	setReg(ADCB_ADHLMT2, 0x7ff8);         /* Set high limit reg. 2 */
	setReg(ADCB_ADLLMT0, 0);              /* Set low limit reg. 0 */
	setReg(ADCB_ADLLMT1, 0);              /* Set low limit reg. 1 */
	setReg(ADCB_ADLLMT2, 0);              /* Set low limit reg. 2 */
	setReg(ADCB_ADZCSTAT, 0xff);          /* Clear zero crossing status flags */
	setReg(ADCB_ADLSTAT, 0xffff);         /* Clear high and low limit status */
	setReg(ADCB_ADSTAT, 0x800);           /* Clear EOSI flag */
	setReg(ADCB_ADSDIS, 0xf8);            /* Enable/disable of samples */
	setReg(ADCB_ADLST1, 0x105);           /* Set ADC channel list reg. */
	setReg(ADCB_ADZCC, 0);                /* Set zero crossing control reg. */
	setReg(ADCB_ADCR2, 0xf);              /* Set prescaler */

	_sample_B[0] = 0;
	_sample_B[1] = 0;
	_sample_B[2] = 0;
	
	HWEnDiB();
}

