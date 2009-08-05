%Genova 01 Ottobre 2005
%Edited by Francesco Nori
%
%This file describe some crucial points in measuring the
%current trhough the transistors. 


- 	The configuration of the transistors is such that currrent is always positive.
	Therefore, _current is an unsigned word.

-	Since the current is always positive, also the tension is always positive.
	Tehrefore all analog tension are set "single ended" so that the formula to
	be used for conversions is:
			ValueRead/32760 * VREF = SingleEndedVoltage
	ValueRead is stored in ADRSLT1.

-	Given the SingleEndedVoltage, the current is given by:
			Current [A] * 0,025 [V/A] * 11 = SingleEndedVoltage [V]
	or equivalently:
			Current = 40/11 * SingleEndedVoltage
	And expressed in milli ampere we have:
			Current [mA] 	= 40000/11 * SingleEndedVoltage
					= 40000/11 * ValueRead/32760 * VREF
					= 0.11 * ValueRead * VREF

-	VREF equals 3.3 [V] and therefore we conclude:
			Current [mA] 	= 0.3663 * ValueRead
			