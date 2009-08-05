/* LibLSSVM - Library for Least Squares Support Vector Machine Regression
 *
 * 2006-2007 Arjan Gijsberts, arjan@liralab.it
 *
 * LSSVM Predict executable
 *
 * Todo:
 * - Implement
 */


#include <iostream>
#include <string>

#include "iCub/LSSVM.h"
#include "iCub/MercerKernels.h"
#include "iCub/Problem.h"


using namespace std;
using namespace iCub::contrib::liblssvm;

class PredictConfig {
  public:
    PredictConfig() {
      this->model_filename = "";
      this->test_filename = "";
      this->problem_type = 2;
      this->xml = false;
      this->verbose = false;
      this->problem = new Problem();
      // if regression, avg. sq. error, otherwise classification error
      this->error = 0.0;
    }

    ~PredictConfig() {
      delete(problem);
      delete(machine);
    }

    double error;

    string model_filename;
    string test_filename;
    int problem_type;
    bool xml;
    bool verbose;

    Problem* problem;
    LSSVM* machine;
    vector<double> pred_y;
};


void exit_with_help() {
  printf(
  "Usage: lssvm-predict [options] test_set_file model_file\n"
  "-o : verbose output (default off)\n"
  "-p type : set problem type (default 2)\n"
  "    0 -- Enforce classification\n"
  "    1 -- Enforce regression\n"
  "    2 -- Autosensing\n"
  "    3 -- Read from model\n"
  "-x : set xml output (default off)\n"
  );
  exit(1);
}

void read_params(int argc, char** argv, PredictConfig &config) {
  int i;

  for(i=1;i<argc;i++) {
    if(argv[i][0] != '-') break;
    if(++i >= argc - 1)
      exit_with_help();
    switch(argv[i-1][1]) {
      case 'x':
        config.xml = true;
        i--;
        break;
      case 'p':
        config.problem_type = atoi(argv[i]);
        break;
      case 'o':
        config.verbose = true;
        i--;
        break;
      default:
        fprintf(stderr,"unknown option\n");
        exit_with_help();
    }
  }

  // determine filenames
  if(i >= argc - 1)
    exit_with_help();

  config.test_filename = argv[i];
  config.model_filename = argv[i+1];
}

void print_config(PredictConfig &config) {
  cout << "# LSSVM Predict" << endl;
  cout << "#" << endl;
  cout << "# - Settings -" << endl;
  cout << "# Test Filename: " << config.test_filename << endl;
  cout << "# Model Output Filename: " << config.model_filename << endl;
  cout << "# Kernel: " << config.machine->getKernel()->toString() << endl;
  cout << "# Machine: " << config.machine->toString() << endl;
  cout << "#" << endl;
  cout << "# - Testing -" << endl;
  cout << "# Problem Type: ";
  cout << ((config.machine->getClassification()) ? "classification" : "regression") << endl;
  cout << "# Number of Samples: " << config.problem->x.size() << "/" << config.problem->y.size() << endl;
  cout << "# Number of Features: " << config.problem->x[0].size() << endl;
  cout << "# Prediction Timing: " << config.machine->getPredictionTime() << endl;
  cout << "# Prediction Error: " << config.error << endl;

  if(config.verbose) {
    cout << "#" << endl;
    cout << "#   id     predicted y          real y" << endl;
    for(unsigned int i = 0; i < config.pred_y.size(); i++) {
      cout.width(6);
      cout << i + 1;
      cout.width(16);
      cout << config.pred_y[i];
      cout.width(16);
      cout << config.problem->y[i];
      cout << endl;
    }
  }
}

void print_config_xml(PredictConfig &config) {
  cout << "<?xml version=\"1.0\" encoding=\"iso-8859-1\"?>" << endl;
  cout << "<lssvm>" << endl;
  cout << "  <model description=\"Model Filename\">" << config.model_filename << "</model>" << endl;
  cout << "  <testset description=\"Testset Filename\" size=\"" << config.problem->x.size() << "\"";
  cout << " dimensionality=\"" << config.problem->x[0].size() << "\">" << config.test_filename;
  cout << "</testset>" << endl;
  cout << "  <test type=\"";
  cout << ((config.machine->getClassification()) ? "classification" : "regression") << "\">" << endl;
  cout << "    <run>" << endl;
  cout << "      <tradeoff description=\"Tradeoff Parameter C\">";
  cout << config.machine->getC() << "</tradeoff>" << endl;
  cout << "      <kernel description=\"Kernel Function Specifier\">";
  cout << config.machine->getKernel()->toString() << "</kernel>" << endl;
  cout << "      <error description=\"Prediction Error\">";
  cout << config.error << "</error>" << endl;
  cout << "      <timing>" << endl;
  cout << "        <prediction description=\"Prediction Timing\">";
  cout << config.machine->getPredictionTime() << "</prediction>" << endl;
  cout << "      </timing>" << endl;
  cout << "    </run>" << endl;
  cout << "  </test>" << endl;
  cout << "</lssvm>" << endl;
}


void predict(PredictConfig &config) {
  config.error = config.machine->predict(config.problem->x, config.problem->y);
  if(config.verbose) {
    config.pred_y = config.machine->predict(config.problem->x);
  }
}

void create_machine(PredictConfig &config) {
  config.machine = &LSSVM::loadFromFile(config.model_filename);

  switch(config.problem_type) {
    case 0: // classification
      config.machine->setClassification(true);
      break;
    case 1: // regression
      config.machine->setClassification(false);
      break;
    case 3: // from model
      break;
    case 2: // auto sensing
    default:
      config.machine->setClassification(config.problem->classification);
      break;
  }
}


int main(int argc, char** argv) {
  registerKernels();
  PredictConfig* config = new PredictConfig();
  try {
    // read params from arguments into config
    read_params(argc, argv, *config);
    // load problem from sparse file
    config->problem->readSparseFile(config->test_filename);
    // load model from file
    create_machine(*config);
    // perform prediction
    predict(*config);
    // print results
    if(config->xml) {
      print_config_xml(*config);
    } else {
      print_config(*config);
    }

    delete(config);
  } catch(string message) {
    std::cerr << std::endl << "Exception caught: " << message << std::endl;
    // delete config
    delete(config);
    return -1;
  }
  return 0;
}




