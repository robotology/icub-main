// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// main.cpp : a learner application
//


// written by BREMEN @ LIRALAB, 2006
// modified to make it a more standard yarp module by paulfitz, 2007


// -------------------------------------------------------
// preliminaries
// -------------------------------------------------------

//#include <conio.h>
#include "libsvmLearningMachine.h"
#include "lMCommands.h"

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Property.h>
#include <yarp/os/Network.h>
#include <yarp/os/all.h>
#include <yarp/os/Module.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;


namespace iCub {
    namespace contrib {
        class BremenLearner;
        class Learner;
        template <class T> class LearnerModule;
    }
}

using namespace iCub::contrib;



// port basename and network name
std::string portName = "learner";
std::string netName = "default";

#define learner (*(pLearner))

void _itoa_(int val, char *str, int base) {
    if (base!=10) {
        printf("Please count your fingers.\n");
    }
    sprintf(str,"%d",val);
}

// what kind of machines? (default: plain)
namespace typeOfMachine {
	enum {
		plain = 0,
		uniform = 1,
		feedback = 2
	};
};

// ---------- prototypes

class libsvmLearner {
public:
    libsvmLearner(int typeOfMachine, unsigned int codomainSize, libsvmLearningMachine::params& params, real *tolerance, real threshold)
        : _machineType(typeOfMachine), _codomainSize(codomainSize), _params(params), tolerance(tolerance), threshold(threshold)
    {
        threshold = 0.5;
		// create learning machines according to cmd line
        lmAlloc(_machine, _codomainSize);
        lmAlloc(_norm, _codomainSize);
		switch ( _machineType ) {
		case typeOfMachine::plain:
			// create normaliser
	        { foreach(_codomainSize,i) {
				_norm[i] = new msNormaliser(_params._domainSize+1);
				libsvmLearningMachine::params tmpParams(params);
				char tmp[32]; _itoa_(i,tmp,10); tmpParams._name = _params._name + "." + tmp;
				_machine[i] = new libsvmLearningMachine(_norm[i], tmpParams);
			} }
            break;
		case typeOfMachine::uniform:
	        { foreach(_codomainSize,i) {
				_norm[i] = new msNormaliser(_params._domainSize+1);
				libsvmUniLearningMachine::params tmpParams(params);
				char tmp[32]; _itoa_(i,tmp,10); tmpParams._name = _params._name + "." + tmp;
				_machine[i] = new libsvmUniLearningMachine(_norm[i], tmpParams, tolerance);
			} }
            break;
		case typeOfMachine::feedback:
	        { foreach(_codomainSize,i) {
				_norm[i] = new msNormaliser(_params._domainSize+1);
				libsvmFBLearningMachine::params tmpParams(params);
				char tmp[32]; _itoa_(i,tmp,10); tmpParams._name = _params._name + "." + tmp;
				_machine[i] = new libsvmFBLearningMachine(_norm[i], tmpParams, threshold);
			} }
            break;
		}
	
        // allocate space for the predicted vector
        lmAlloc(_predicted, _codomainSize);
    }
	~libsvmLearner() {
		delete[] _predicted;
		delete[] _norm;
		delete[] _machine;
	}

	// viewing counters
	unsigned int getDomainSize( void ) const { return _params._domainSize; }
	unsigned int getCapacity( void ) const {
		unsigned int totalCapacity = 0;
		{ foreach(_codomainSize,i) totalCapacity += _machine[i]->getCapacity(); }
		return totalCapacity;
	}
	unsigned int getCount( void ) const {
		unsigned int totalCount = 0;
		{ foreach(_codomainSize,i) totalCount += _machine[i]->getCount(); }
		return totalCount;
	}
	// resetting the machine
    void reset( void ) {
        { foreach(_codomainSize,i) { _machine[i]->reset(); } }
    }
    // loading and saving status
    void save( void ) {
        { foreach(_codomainSize,i) { _machine[i]->save(); } }
    }
    bool load( void ) {
        bool res = true;
        { foreach(_codomainSize,i) { 
            res &= _machine[i]->load();
        } }
        return res;
    }
    void setC( const double C ) {
        { foreach(_codomainSize,i) { _machine[i]->setC(C); } }
    }
    bool addExample( const real x[], const real y[] ) {
        bool res = true;
        { foreach(_codomainSize,i) { 
            res &= _machine[i]->addExample(x,y[i]);
        } }
        return res;
    }
    void train( void ) {
        { foreach(_codomainSize,i) { _machine[i]->train(); } }
    }
    void predict( const real x[] ) {
        { foreach(_codomainSize,i) { _predicted[i] = _machine[i]->predict(x); } }
    }
	void filterSVs( void) {
        { foreach(_codomainSize,i) { _machine[i]->filterSVs(); } }
	}
    real* _predicted;

private:
	// what kind of machines do we use?
	int _machineType;
	// normalisers and machines
	Normaliser** _norm;
    libsvmLearningMachine** _machine;
	// size of the codomain
    unsigned int _codomainSize;
	// machines parameters (all the same)
    libsvmLearningMachine::params _params;


    // in case it is a uniform machine, need array of tolerances
    real* tolerance;
    // in case it is a feedback machine, need error threshold
    real threshold;

};


/**
 *
 * A minimal learner interface for regression.
 * Learners that meet this interface can, if desired, be made
 * into an executable module by with iCub::contrib::LearnerModule
 *
 * \see iCub::contrib::LearnerModule
 *
 * \author Paul Fizpatrick
 *
 */
class iCub::contrib::Learner : public IConfig {
public:

    /**
     * Provide the learner with an example of the desired mapping.
     * @param input a sample input
     * @param desiredOutput the corresponding output
     */
    virtual void train(const Vector& input, const Vector& desiredOutput) = 0;

    /**
     *
     * Ask the learner to predict the output for a given input.
     * @param input the input
     * @return the expected output
     *
     */
    virtual Vector test(const Vector& input) = 0;

    /**
     *
     * Forget everything and start over.
     *
     */
    virtual void clear() = 0;
};


/**
 *
 * A wrapper class for Bremen's SVM learning stuff.
 *
 * \author Francesco Orabona, Paul Fizpatrick
 *
 */
class iCub::contrib::BremenLearner : public Learner {
private:
    // domain and codomain size and number of examples
    int domainSize;
    int codomainSize;
    int capacity;

    int machineType;
    // are we doing classification or regression?
    bool classification;
    // do we want non-SVs filtering?
    bool filter;
    // do we want to load previouslt saved models and data, on startup?
    bool load;
  
    int samples;
  
    // vectors and arrays of reals to interact with the machine
    Vector example, prediction;
    real* x, * y;

    // in case it is a uniform machine, need array of tolerances
    real* tolerance;
    // in case it is a feedback machine, need error threshold
    real threshold;

    libsvmLearner * pLearner;

public:
    BremenLearner() {
        machineType = typeOfMachine::plain;
        classification = false;
        filter = false;
        load = false;
        samples = 0;
        pLearner = NULL;
    }

    /**
     *
     * Minimal open method.  Just makes a basic learner that might
     * not work too well but is useful for getting started using 
     * the class.
     *
     * @param domainSize the size of input vectors
     * @param codomainSize the size of output vectors
     * @param exampleCount I don't actually know what this is really
     *
     */
    bool open(int domainSize, int codomainSize, int exampleCount = 0) {
        Property p;
        p.put("dom",domainSize);
        p.put("cod",codomainSize);
        p.put("ex",exampleCount?exampleCount:256);
        return open(p);
    }

    bool open(Searchable& config);

    void clear() {
        learner.reset();
    }

    void setC(double newC) {
        learner.setC(newC);
        learner.train();
        learner.save();
    }

    void train(const Vector& input, const Vector& desiredOutput) {
        if (input.size() != domainSize ||
            desiredOutput.size() != codomainSize ) {
            printf("example is wrong size\n");
            return;
        }
        printf("Got an example\n");
        // a full example: add it
        { foreach(domainSize,i) x[i] = input[i]; }
        { foreach(codomainSize,i) y[i] = desiredOutput[i]; }
        if ( learner.addExample(x, y) ) {
            cout << learner.getCount() << "/" << learner.getCapacity() << "     \r";
            // every now and then, train the machines and save
            if ( learner.getCount() % 5 > 2 ) {
                learner.train();
                if ( filter ) {
                    learner.filterSVs();
                }
                learner.save();
            }
        }
    }

    Vector test(const Vector& input) {
        if ( input.size() != domainSize ) {
            printf("testing with wrong size\n");
            return Vector();
        }
        printf("Should make a prediction\n");
        // this is just a sample: then predict and send
        { foreach(domainSize,i) x[i] = input[i]; }
        learner.predict(x);
        { foreach(codomainSize,i) prediction[i] = learner._predicted[i]; }
        return prediction;
    }

    int getDomainSize() {
        return domainSize;
    }

    int getCodomainSize() {
        return codomainSize;
    }

    bool close() {
        if (pLearner!=NULL) {
            delete pLearner;
            pLearner = NULL;
            
            cout << "Bailing out." << endl;
            if ( machineType == typeOfMachine::uniform ) {
                delete[] tolerance;
            }
            delete[] x;
            delete[] y;
        }
        
        return true;
    }


    virtual bool configure(Searchable& config) {
        if (config.check("c")) {
            setC(config.check("c",Value(1), 
                              "mysterious c value").asDouble());
            return true;
        }
        return false;
    }
};



bool BremenLearner::open(Searchable& opt) {

	Value *val;
	if (opt.check("help")) {
		cout << "BremenLearner parameters: " << endl;
		cout << "  --dom <domain size> --cod <codomain size>" << endl;
		cout << "  --ex <number of examples>" << endl;
		cout << "  [--f] [--u <tolerance values>]" << endl;
		cout << "  [--cl] [--filter] [--load]" << endl;
		return false;
	}
	
	if (opt.check("dom",val)) {
		domainSize = val->asInt();
	} else {
		cout << "FATAL ERROR: must specify domain size." << endl;
		exit(-1);
	}
	
	if (opt.check("cod",val)) {
		codomainSize = val->asInt();
	} else {
		cout << "FATAL ERROR: must specify domain size." << endl;
		exit(-1);
	}

	// number of examples
	if (opt.check("ex",val)) {
		capacity = val->asInt();
	} else {
		cout << "FATAL ERROR: must specify number of examples." << endl;
		exit(-1);
	}

	// uniform machine required?
	if (opt.check("u")) {
		if ( machineType != typeOfMachine::plain ) {
			// can't have both types of machine...
			cout << "FATAL ERROR: must specify one type only of machine." << endl;
			exit(-1);
		}
		machineType = typeOfMachine::uniform;
		lmAlloc(tolerance, domainSize);

        Bottle bot = opt.findGroup("u").tail();

        /*
		unsigned int index;
		{ foreach(argc,i) {
			String argument(argv[i]);
			if ( argument == "--u" ) { index = i; break; }
		} }
		// must be followed by domainSize reals...
		if ( argc-(int)index < domainSize ) {
			cout << "FATAL ERROR: must specify " << domainSize << " tolerances after --u." << endl;
			exit(-1);
		}
        */
        if (bot.size()!=domainSize) {
			cout << "FATAL ERROR: must specify " << domainSize << " tolerances after --u." << endl;
            return false;
        }
        for (int i=0; i<domainSize; i++) {
            tolerance[i] = bot.get(i).asDouble();
        }

        /*
		{ foreach_s(index+1,index+1+domainSize,i) 
              if ( sscanf(argv[i], "%lf", &tolerance[i-index-1]) != 1 ) {
                  cout << "FATAL ERROR: invalid tolerance value." << endl;
                  exit(-1);
              }
		}
        */
	}

	// feedback machine required?
	if (opt.check("f",val)) {
		if ( machineType != typeOfMachine::plain ) {
			// can't have both types of machine...
			cout << "FATAL ERROR: must specify one type only of machine." << endl;
			exit(-1);
		}
		machineType = typeOfMachine::feedback;
        /*
		unsigned int index;
		{ foreach(argc,i) {
			String argument(argv[i]);
			if ( argument == "--f" ) { index = i; break; }
		} }
		// must be followed by a real
		if ( argc-index <= 1 ) {
			cout << "FATAL ERROR: must specify threshold after --f." << endl;
			exit(-1);
		}
		if ( sscanf(argv[index+1], "%lf", &threshold) != 1 ) {
			cout << "FATAL ERROR: invalid threshold value." << endl;
			exit(-1);
		}
        */

        Bottle bot = opt.findGroup("f").tail();
		if ( bot.size() != 1 ) {
			cout << "FATAL ERROR: must specify threshold after --f." << endl;
            return false;
		}
        threshold = bot.get(0).asDouble();
	}

	// classification required?
	if (opt.check("cl")) {
		classification = true;
	}

	// filter out non-SVs?
	if (opt.check("filter")) {
		filter = true;
	}

	// load data and models on startup?
	if (opt.check("load")) {
		load = true;
	}

	// show parameters
	cout << "CMDLINE: port basename is " << portName.c_str() << "." << endl;
	cout << "CMDLINE: network name is " << netName.c_str() << "." << endl;
	cout << "CMDLINE: domain dim. " << domainSize << ", codomain dim. " << codomainSize << ", " << capacity << " examples." << endl;
	switch ( machineType ) {
	case typeOfMachine::plain:
		cout << "CMDLINE: plain machine selected." << endl;
		break;
	case typeOfMachine::uniform:
		cout << "CMDLINE: uniform machine selected. Tolerances are ";
		{ foreach(domainSize,i) cout << tolerance[i] << " "; }
		cout << endl;
		break;
	case typeOfMachine::feedback:
		cout << "CMDLINE: feedback machine selected. Threshold is " << threshold << endl;
		break;
	}
	if ( classification ) {
		cout << "CMDLINE: doing classification."<<endl;
	} else {
		cout << "CMDLINE: doing regression."<<endl;
	}
	if ( filter ) {
		cout << "CMDLINE: non-SVs filtering: ON."<<endl;
	} else {
		cout << "CMDLINE: non-SVs filtering: OFF."<<endl;
	}
	if ( ! load ) {
		cout << "CMDLINE: this machine WON'T load/save any models/data on startup."<<endl;
	}

	// create the parameters
    libsvmLearningMachine::params params;
    params._capacity = capacity;
    params._domainSize = domainSize;
	string name(portName.c_str());
    params._name = name;
    if ( classification ) {
		params._svmparams.svm_type = C_SVC;
		params._svmparams.kernel_type = RBF;
		params._svmparams.degree = 3;
		params._svmparams.gamma = 1/(real)params._domainSize;
		params._svmparams.coef0 = 0;
		params._svmparams.nu = 0.3;
		params._svmparams.cache_size = 10;
		params._svmparams.C = 1;
		params._svmparams.eps = 1e-3;
		params._svmparams.p = 0.1;
		params._svmparams.shrinking = 1;
		params._svmparams.probability = 0;
		params._svmparams.nr_weight = 0;
		params._svmparams.weight_label = 0;
		params._svmparams.weight = 0;
	} else {
		params._svmparams.svm_type = EPSILON_SVR;
		params._svmparams.kernel_type = RBF;
		params._svmparams.degree = 3;
		params._svmparams.gamma = 1/(real)params._domainSize;
		params._svmparams.coef0 = 0;
		params._svmparams.nu = 0.3;
		params._svmparams.cache_size = 10;
		params._svmparams.C = 30;
		params._svmparams.eps = 1e-3;
		params._svmparams.p = 0.1;
		params._svmparams.shrinking = 1;
		params._svmparams.probability = 0;
		params._svmparams.nr_weight = 0;
		params._svmparams.weight_label = 0;
		params._svmparams.weight = 0;
	}
    params._filter = filter;
    
	// create learner (pool of learning machines)
    pLearner = 
        new libsvmLearner(machineType,codomainSize,params,tolerance,
                          threshold);

	// if required, try and load previously saved model and data
	if ( load ) {
		learner.load();
	}

	prediction.resize(codomainSize);
	lmAlloc(x,domainSize);
	lmAlloc(y,codomainSize);

	// ------------ main command parsing loop

	/*
      if ( learner.getCount() == 0 ) {
	  ofstream outFile("E:\\yarp2\\zgarbage\\sample.dat");
      }
      ofstream outFile("E:\\yarp2\\zgarbage\\sample.dat",ios::app);
	*/
    //	outFile.precision(2);

	//bool goOn = true;

	return true;
}





/**
 * A module for doing learning.
 * T can be any iCub::contrib::Learner
 *
 * \see iCub::contrib::Learner
 * \see iCub::contrib::BremenLearner
 *
 * \author Francesco Orabona, Paul Fizpatrick
 *
 */
template <class T>
class iCub::contrib::LearnerModule : public Module {
private:
    // data ports
    BufferedPort<yarp::sig::Vector> data_out;
    BufferedPort<yarp::sig::Vector> data_in;
    // cmd ports. must be NO_BUFFERS, otherwise sequentiality is lost
    BufferedPort<Bottle> cmd_out;
    BufferedPort<Bottle> cmd_in;
    //YARPOutputPortOf<YVector> data_out (YARPOutputPort::DEFAULT_OUTPUTS, YARP_TCP);
    //YARPInputPortOf<YVector> data_in (YARPInputPort::DEFAULT_BUFFERS, YARP_TCP);
    //YARPOutputPortOf<int> cmd_out (YARPOutputPort::DEFAULT_OUTPUTS, YARP_TCP);
    //YARPInputPortOf<int> cmd_in (YARPInputPort::NO_BUFFERS, YARP_TCP);

    T bLearner;

	void registerPort(Port& port, std::string& name, std::string &net);
    void registerAllPorts();
    void unregisterAllPorts();
public:
    LearnerModule() {
    }

    bool open(Searchable& opt);
    bool close();

    bool updateModule();

    bool interruptModule() {
        cmd_in.interrupt();
        data_in.interrupt();
	return true;
    }

    bool respond(const Bottle& cmd, Bottle& reply);

};


// -------------------------------------------------------
// a class enforcing multiple-output learning machines
// -------------------------------------------------------


// -------------------------------------------------------
// helper procedures
// -------------------------------------------------------

template <class T>
void LearnerModule<T>::registerPort(Port& port, std::string& name, std::string& net)
{

	if ( port.open(name.c_str()) != true ) {
		cout << "FATAL ERROR: could not register port " << name.c_str() << " on network " << net.c_str() << endl;
		exit(-1);
	}

}

template <class T>
void LearnerModule<T>::registerAllPorts()
{

	std::string fullPortName;

	fullPortName = "/" + portName + "/o:vec"; data_out.open(fullPortName.c_str());
	fullPortName = "/" + portName + "/i:vec"; data_in.open(fullPortName.c_str());
	fullPortName = "/" + portName + "/o:bot"; cmd_out.open(fullPortName.c_str());
	fullPortName = "/" + portName + "/i:bot"; cmd_in.open(fullPortName.c_str());

}

template <class T>
void LearnerModule<T>::unregisterAllPorts( void )
{

	cmd_in.close();
	cmd_out.close();
	data_in.close();
	data_out.close();

}

template <class T>
bool LearnerModule<T>::open(Searchable& opt)
{
	Value *val;
	if (opt.check("help")||!(opt.check("dom"))) {
		cout << "USAGE: learner"  << endl;
		cout << "  --dom <domain size> --cod <codomain size>" << endl;
		cout << "  --ex <number of examples>" << endl;
		cout << "  [--port <port basename>] [--net <network name>]" << endl;
		cout << "  [--f] [--u <tolerance values>]" << endl;
		cout << "  [--cl] [--filter] [--load]" << endl;
		return false;
	}
	
	if (opt.check("port",val)) {
		portName = val->asString().c_str();
	}
	
	if (opt.check("net",val)) {
		portName = val->asString().c_str();
	}

    bool ok = bLearner.open(opt);
    if (!ok) { return false; }

	// show parameters
	cout << "CMDLINE: port basename is " << portName.c_str() << "." << endl;
	cout << "CMDLINE: network name is " << netName.c_str() << "." << endl;

	attach(cmd_in);

	// register ports
	registerAllPorts();

    return true;
}



template <class T>
bool LearnerModule<T>::respond(const Bottle& cmd, Bottle& reply) {

    const Bottle *bot1=&cmd;
    Bottle& bot2=reply;

    bool success = true;

    // support older integer-based protocol
    switch ( bot1->get(0).asVocab() ) {
        // YARP now takes care of quit messages
        //case lMCommand::Quit: // QUIT
        //bot2.addInt(lMCommand::Ok);
        //cmd_out.write(true);
        //cout << "Got QUIT command. Bailing out." << endl;
        //goOn = false;
        //break;
    case VOCAB4('h','e','l','p'):
        bot2.addVocab(Vocab::encode("help"));
        bot2.addString("[clr]");
        bot2.addString("[c] $c");
        bot2.addString("[set] $property $value");
        break;
    case VOCAB3('c','l','r'):
    case lMCommand::Reset: // RESET
        cout << "Got RESET command. Resetting machine." << endl;
        bLearner.clear();
        bot2.addInt(lMCommand::Ok);
        break;

    case VOCAB3('s','e','t'):
        //case 'c': // CHANGE C
        {
            ConstString name = bot1->get(1).toString();
            double val = bot1->get(2).asDouble();
            Property p;
            p.put(name,val);
            bool ok = bLearner.configure(p);
            if (ok) {
                bot2.addInt(lMCommand::Ok);
            } else {
                cout << "Unrecognised parameter" << endl;
                success = false;
                bot2.addInt(lMCommand::Failed);
            }
        } 
        break;

    case VOCAB1('c'):
        //case 'c': // CHANGE C
        if (bot1->get(1).isDouble()||bot1->get(1).isInt()) {
            double newC;
            newC = bot1->get(1).asDouble();
            /*
            bLearner.setC(newC);
            */
            Property p;
            p.put("c",newC);
            bool ok = bLearner.configure(p);
            if (ok) {
                bot2.addInt(lMCommand::Ok);
            } else {
                cout << "Unrecognised parameter" << endl;
                success = false;
                bot2.addInt(lMCommand::Failed);
            }
        } else {
            bot2.addInt(lMCommand::Failed);
        }
        break;

    default: // unrecognised port command
        cout << "Unrecognised command from the network " << bot1->get(0).asInt() << "." << endl;
        success = false;
        bot2.addInt(lMCommand::Failed);
        break;
    }

    // unnecessary, but for compatibility for way this program was originally
    // written...
    cmd_out.prepare() = bot2;
    cmd_out.write();

    return success;
}


// -------------------------------------------------------
// main routine
// -------------------------------------------------------

template <class T>
bool LearnerModule<T>::updateModule() {

    // ----- read an example or a sample to be predicted
    if ( data_in.read(true)!=NULL ) {
        Vector example = *data_in.lastRead();
        if ( example.size() == bLearner.getDomainSize() ) {
            data_out.prepare() = bLearner.test(example);
            data_out.write();
        } else if ( example.size() == bLearner.getDomainSize()+
                    bLearner.getCodomainSize() ) {
            Vector xx(bLearner.getDomainSize());
            Vector yy(bLearner.getCodomainSize());
            for (int i=0; i<xx.size(); i++) {
                xx(i) = example(i);
            }
            for (int j=0; j<yy.size(); j++) {
                yy(j) = example(j+bLearner.getDomainSize());
            }
            bLearner.train(xx,yy);
        } else {
            cout << "ERROR: got sample/example of the wrong size." << endl;
            return false;
        }
        return true;
    }

    return true;
}


template <class T>
bool LearnerModule<T>::close() {
    unregisterAllPorts();
    return bLearner.close();
}


int main (int argc, char* argv[]) {
    Network yarp;
    LearnerModule<BremenLearner> module;
    return module.runModule(argc,argv);
}


