// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

//
// $Id: main.cpp,v 1.9 2009/07/21 12:19:39 iron Exp $
//  @author pasa
//

/**
 * @ingroup icub_module
 *
 * \defgroup icub_canbus_collector canbus_collector
 *
 * Collects broadcast messages from a CAN bus connection and 
 * dumps them to file.
 *
 * \author Giorgio Metta
 *
 */


#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>

#include <string>
/* memset example */
#include <string.h>



using namespace yarp::dev;
using namespace yarp::os;

//YARPEsdDaqDeviceDriver touch; LATER: required to read from the ESD-DA acquisition card.

PolyDriver head;
IAmplifierControl *head_amp;
IPidControl *head_pid;
IEncoders *head_enc;

PolyDriver arm;
IAmplifierControl *arm_amp;  
IPidControl *arm_pid;
IEncoders *arm_enc;

/*
 * Alloc, assert and initialize mem.
 */
template <class T>
inline T* allocAndCheck(int size)
{
    T* t = new T[size];
    memset(t, 0, sizeof(T) * size);
    return t;
}

/*
 * Check and free mem.
 */
template <class T>
inline void checkAndDestroy(T* &p) {
    if (p!=0) {
        delete [] p;
        p = 0;
    }
}

/*
 * usage:
 *  PROGRAMNAME --robot ROBOTNAME --part [head|arm|all] --filename OUTPUT_FILENAME
 *              --steps # of steps --period PERIOD [ms] --verbose
 *
 */
int main(int argc, char *argv[]) {

    Network::init();

    Property options;
    options.fromCommand(argc, argv);

	std::string filename;
    Value& robotname = options.find("robot");
    
	if (!options.check("part"))
		options.put("part", "all");
	
	Value& instantiate = options.find("part");

    std::string tmp=instantiate.asString().c_str();
    if (tmp==std::string("head") || 
        tmp==std::string("all")) {
		
		///////////// head
		Property p;
        filename.clear();
        filename += robotname.asString().c_str();
        filename += "_head.ini";
		printf("Head working with config file %s\n", filename.c_str());
		p.fromConfigFile(filename.c_str());

		p.put("device", "esdsniffer");
		if (options.check("verbose"))
			p.put("verbose", 1);

		// create a device with the loaded options.
		head.open(p);
		if (!head.isValid()) {
			printf("Device not available.  Here are the known devices:\n");
			printf("%s", Drivers::factory().toString().c_str());
			return -1;
		}
	}

	if (tmp=="arm" ||
		tmp=="all") {

		///////////// arm
		Property parm;
        filename.clear();
        filename += robotname.asString().c_str();
        filename += "_arm.ini";
		printf("Arm working with config file %s\n", filename.c_str());
		parm.fromConfigFile(filename.c_str());

		parm.put("device", "esdsniffer");
		if (options.check("verbose"))
			parm.put("verbose", 1);

		// create a device for the arm 
		arm.open(parm);
        if (!arm.isValid()) {  
			printf("Device not available.  Here are the known devices:\n");
			printf("%s", Drivers::factory().toString().c_str());
			return -1;
        } 
	}

	if (tmp=="right_arm" ||
		tmp=="all") {

		///////////// arm
		Property parm;
        filename.clear();
        filename += robotname.asString().c_str();
        filename += "_arm_right.ini";
		printf("Arm working with config file %s\n", filename.c_str());
		parm.fromConfigFile(filename.c_str());

		parm.put("device", "esdsniffer");
		if (options.check("verbose"))
			parm.put("verbose", 1);

		// create a device for the arm 
		arm.open(parm);
        if (!arm.isValid()) {  
			printf("Device not available.  Here are the known devices:\n");
			printf("%s", Drivers::factory().toString().c_str());
			return -1;
        } 
	}
    
    bool ok = true;
	if (head.isValid())	{
		ok &= head.view(head_amp);
        ok &= head.view(head_pid);
		ok &= head.view(head_enc);
	}
    
	if (arm.isValid()) {
		ok &= arm.view(arm_amp);
		ok &= arm.view(arm_pid);
		ok &= arm.view(arm_enc);
	}

    if (!ok) {
        printf("One or more interfaces are not available for the given driver\n");

        head_amp = 0;
        head_pid = 0;
        head_enc = 0;
        if (head.isValid()) head.close();

		arm_amp = 0;
		arm_pid = 0;
		arm_enc = 0;
        if (arm.isValid()) arm.close();

		return -1;
    }

    Time::turboBoost();

    int max_steps = 50;
    int sampling_period = 10; /// ms.
    bool verbose = false;

    Value *val;
    if (options.check("filename", val)) {
        filename.clear();
        filename += val->asString().c_str();
    }
    else {
        filename.clear();
        filename += "dummy.txt";
    }


    if (options.check("steps", val)) {
        max_steps = val->asInt();
    }

    if (options.check("period", val)) {
        sampling_period = val->asInt();
    }

    if (options.check("verbose", val)) {
        verbose = val->asInt();
    }

    printf("Running with:\n");
    printf("filename: %s\n", filename.c_str());
    printf("number of steps: %d\n", max_steps);
    printf("period: %d\n", sampling_period);
    printf("verbose: %d\n", verbose);
	printf ("The robot is initialized\n");

	double **_headjointstore = 0;
    double **_headcurrentstore = 0;
    double **_headerrorstore = 0;
	double **_armjointstore = 0;
    double **_armcurrentstore = 0;
    double **_armerrorstore = 0;

    int j;
    int head_jnts = 0;
    if (head.isValid()) {
        _headjointstore = allocAndCheck<double *>(max_steps);
        _headcurrentstore = allocAndCheck<double *>(max_steps);
        _headerrorstore = allocAndCheck<double *>(max_steps);

        head_enc->getAxes(&head_jnts);
        _headjointstore[0] = allocAndCheck<double>(head_jnts*max_steps);
        _headcurrentstore[0] = allocAndCheck<double>(head_jnts*max_steps);
        _headerrorstore[0] = allocAndCheck<double>(head_jnts*max_steps);

        for (j = 1; j < max_steps; j++) {
            _headjointstore[j] = _headjointstore[j-1]+head_jnts;
            _headcurrentstore[j] = _headcurrentstore[j-1]+head_jnts;
            _headerrorstore[j] = _headerrorstore[j-1]+head_jnts;
        }
    }

    int arm_jnts = 0;
    if (arm.isValid()) {
        _armjointstore = allocAndCheck<double *>(max_steps);
        _armcurrentstore = allocAndCheck<double *>(max_steps);
        _armerrorstore = allocAndCheck<double *>(max_steps);

        arm_enc->getAxes(&arm_jnts);
        _armjointstore[0] = allocAndCheck<double>(arm_jnts*max_steps);
        _armcurrentstore[0] = allocAndCheck<double>(arm_jnts*max_steps);
        _armerrorstore[0] = allocAndCheck<double>(arm_jnts*max_steps);

        for (j = 1; j < max_steps; j++)
        {
            _armjointstore[j] = _armjointstore[j-1]+arm_jnts;
            _armcurrentstore[j] = _armcurrentstore[j-1]+arm_jnts;
            _armerrorstore[j] = _armerrorstore[j-1]+arm_jnts;
        }
    }

	printf ("Running!\n");

    FILE *fp = fopen (filename.c_str(), "w");
    if (fp == NULL) {
        printf ("Can't open filename %s\n", filename.c_str());

        if (head.isValid()) {
		    head.close();
            if (_headjointstore) {
                checkAndDestroy<double>(_headjointstore[0]);
                checkAndDestroy<double *>(_headjointstore);
            }

            if (_headcurrentstore) {
                checkAndDestroy<double>(_headcurrentstore[0]);
                checkAndDestroy<double *>(_headcurrentstore);
            }

            if (_headerrorstore) {
                checkAndDestroy<double>(_headerrorstore[0]);
                checkAndDestroy<double *>(_headerrorstore);
            }
	    }

        if (arm.isValid()) {
		    arm.close();
		    if (_armjointstore) {
                checkAndDestroy<double>(_armjointstore[0]);
                checkAndDestroy<double *>(_armjointstore);
            }

            if (_armcurrentstore) {
                checkAndDestroy<double>(_armcurrentstore[0]);
                checkAndDestroy<double *>(_armcurrentstore);
            }

            if (_armerrorstore) {
                checkAndDestroy<double>(_armerrorstore[0]);
                checkAndDestroy<double *>(_armerrorstore);
            }
	    }

        return -1;
    }

    /*
     * write to file here, check code for format.
     */
    int i;
    for (i = 0; i < head_jnts; i++)
        fprintf (fp, "p-head%d ", i);

    for (i = 0; i < head_jnts; i++)
        fprintf (fp, "i-head%d ", i);

    for (i = 0; i < head_jnts; i++)
        fprintf (fp, "u-head%d ", i);

    for (i = 0; i < arm_jnts; i++)
        fprintf (fp, "p-arm%d ", i);

    for (i = 0; i < arm_jnts; i++)
        fprintf (fp, "i-arm%d ", i);

    for (i = 0; i < arm_jnts; i++)
        fprintf (fp, "u-arm%d ", i);

    // fake entries.
    for (i = 0; i < 32; i++)
        fprintf (fp, "analog%d ", i);

    fprintf (fp, "\n");

    double before = Time::now(), now = 0.0;
    double beginning = before;

	int cycle;
    for (cycle = 0; cycle < max_steps; cycle++)
	{
		bool ok;
        if (!(cycle % 100)) {
            printf ("cycle; %d\n", cycle);
        }

        if (head.isValid()) {
            ok = head_enc->getEncoders(_headjointstore[cycle]);
		    if (!ok)
			    printf ("troubles reading head joint pos\n");

            ok = head_amp->getCurrents(_headcurrentstore[cycle]);
		    if (!ok)
			    printf ("troubles reading head joint torques\n");

            ok = head_pid->getOutputs(_headerrorstore[cycle]);
		    if (!ok)
			    printf ("troubles reading head joint errors\n");
        }

        if (arm.isValid()) {
            ok = arm_enc->getEncoders(_armjointstore[cycle]);
		    if (!ok)
			    printf ("troubles reading arm joint pos\n");

            ok = arm_amp->getCurrents(_armcurrentstore[cycle]);
		    if (!ok)
			    printf ("troubles reading arm torques\n");

            ok = arm_pid->getOutputs(_armerrorstore[cycle]);
		    if (!ok)
			    printf ("troubles reading arm errors\n");
        }

		// wait.
		now = Time::now();
        if ((now - before)*1000 < sampling_period) {
            double k = double(sampling_period)/1000.0-(now-before);
			Time::delay(k);
            before = now + k;
		}
        else {
            printf("%d: Thread can't poll fast enough (time: %f)\n", cycle, now-before);
            before = now;
		}
	}

    // save data to file.
    for (cycle = 0; cycle < max_steps; cycle++) {
        for (i = 0; i < head_jnts; i++) {
			fprintf (fp, "%.2f ", _headjointstore[cycle][i]);
		}

        for (i = 0; i < head_jnts; i++) {
			fprintf (fp, "%.2f ", _headcurrentstore[cycle][i]);
		}

        for (i = 0; i < head_jnts; i++) {
			fprintf (fp, "%.2f ", _headerrorstore[cycle][i]);
		}

		for (i = 0; i < arm_jnts; i++) {
			fprintf (fp, "%.2f ", _armjointstore[cycle][i]);
		}

		for (i = 0; i < arm_jnts; i++) {
			fprintf (fp, "%.2f ", _armcurrentstore[cycle][i]);
		}

		for (i = 0; i < arm_jnts; i++) {
			fprintf (fp, "%.2f ", _armerrorstore[cycle][i]);
		}

        for (i = 0; i < 32; i++)
			fprintf (fp, "%.2f ", double(0.0));

        fprintf (fp, "\n");
    }

    if (fp != NULL)
        fclose (fp);

    if (head.isValid())
    {
	    head.close();
        if (_headjointstore) {
            checkAndDestroy<double>(_headjointstore[0]);
            checkAndDestroy<double *>(_headjointstore);
        }

        if (_headcurrentstore) {
            checkAndDestroy<double>(_headcurrentstore[0]);
            checkAndDestroy<double *>(_headcurrentstore);
        }

        if (_headerrorstore) {
            checkAndDestroy<double>(_headerrorstore[0]);
            checkAndDestroy<double *>(_headerrorstore);
        }
    }

    if (arm.isValid())
    {
	    arm.close();
	    if (_armjointstore) {
            checkAndDestroy<double>(_armjointstore[0]);
            checkAndDestroy<double *>(_armjointstore);
        }

        if (_armcurrentstore) {
            checkAndDestroy<double>(_armcurrentstore[0]);
            checkAndDestroy<double *>(_armcurrentstore);
        }

        if (_armerrorstore) {
            checkAndDestroy<double>(_armerrorstore[0]);
            checkAndDestroy<double *>(_armerrorstore);
        }
    }

    Network::fini();
	return 0;
}
