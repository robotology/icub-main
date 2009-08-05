/* LibLSSVM - Library for Least Squares Support Vector Machine Regression
 *
 * 2006-2007 Arjan Gijsberts, arjan@liralab.it
 *
 * LSSVM Train executable
 *
 * Mostly adapted code from LibSVM (svm-train.c) (by now: literally mostly adapted :) )
 *
 * Todo:
 * - Implement grid search and evolutionary search (here?)
 */


#include <iostream>
#include <string>

#include "iCub/LSSVM.h"
#include "iCub/MercerKernels.h"
#include "iCub/Problem.h"

using namespace std;
using namespace iCub::contrib::liblssvm;

class TrainConfig {
  public:
    TrainConfig() {
      // default kernel with id=0 with default settings
      this->kernel_str = "0";
      this->machine_str = "0";
      this->input_filename = "";
      this->model_filename = "";
      this->machine_c = 1;
      this->problem_type = 2;
      this->xml = false;
      this->verify_loo = false;
      this->problem = new Problem();
      this->k_fold = 0;
      this->k_fold_error = 0.0;
    }

    ~TrainConfig() {
      delete(problem);
      delete(machine);
    }

    string kernel_str;
    string machine_str;

    string input_filename;
    string model_filename;
    double machine_c;
    int problem_type;
    int k_fold;
    double k_fold_error;
    bool xml;
    bool verify_loo;

    Problem* problem;
    LSSVM* machine;
};



void exit_with_help() {
  printf(
  "Usage: lssvm-train [options] training_set_file [model_file]\n"
  "options:\n"
  "-k type[:param1[,param2]] : set type of kernel function (default 0)\n"
  "    0 -- polynomial: (u'*v + const_val2)^degree1\n"
  "    1 -- radial basis function: exp(-gamma*|u-v|^2)\n"
  "    2 -- gaussian radial basis function: exp(-|u-v|^2/2sigma^2)\n"
  "    3 -- boolean function: (1+gamma)^(u'*v)\n"
  "    4 -- sigmoid: tanh(gamma1*u'*v + const_val2)\n"
  "-t type[:param1] : set type of LSSVM (default 0)\n"
  "    0 -- Standard LSSVM using Cholesky decomposition\n"
  "    1 -- Reference LSSVM using standard inversion\n"
  "    2 -- Partial LSSVM using random subset with probability parameter\n"
  "-c tradeoff : set the parameter C (default 1)\n"
  "-p type : set problem type (default 2)\n"
  "    0 -- Enforce classification\n"
  "    1 -- Enforce regression\n"
  "    2 -- Autosensing\n"
  "-v k : perform k-fold cross validation (default off)\n"
  "-x : set xml output (default off)\n"
  );
  exit(1);
}

void read_params(int argc, char** argv, TrainConfig &config) {
  int i;

  for(i=1;i<argc;i++) {
    if(argv[i][0] != '-') break;
    if(++i>=argc)
      exit_with_help();
    switch(argv[i-1][1]) {
      case 'k':
        config.kernel_str = argv[i];
        break;
      case 't':
        config.machine_str = argv[i];
        break;
      case 'c':
        config.machine_c = atof(argv[i]);
        break;
      case 'p':
        config.problem_type = atoi(argv[i]);
        break;
      case 'v':
        config.k_fold = atoi(argv[i]);
        break;
      case 'x':
        config.xml = true;
        i--;
        break;
      case 'l':
        config.verify_loo = true;
        i--;
        break;
      default:
        fprintf(stderr,"unknown option\n");
        exit_with_help();
    }
  }

  // determine filenames
  if(i>=argc)
    exit_with_help();

  config.input_filename = argv[i];

  if(i<argc-1) {
    config.model_filename = argv[i+1];
  } else {
    config.model_filename = config.input_filename + ".model";
  }
}


void print_config(TrainConfig &config) {
  cout << "# LSSVM Train" << endl;
  cout << "#" << endl;
  cout << "# - Settings -" << endl;
  cout << "# Input Filename: " << config.input_filename << endl;
  cout << "# Model Output Filename: " << config.model_filename << endl;
  cout << "# Machine: " << config.machine->toString() << endl;
  cout << "# Kernel: " << config.machine->getKernel()->toString() << endl;
  cout << "#" << endl;
  cout << "# - Training -" << endl;
  cout << "# Problem Type: ";
  cout << ((config.machine->getClassification()) ? "classification" : "regression") << endl;
  cout << "# Number of Samples: " << config.problem->x.size() << "/" << config.problem->y.size() << endl;
  cout << "# Number of Features: " << config.problem->x[0].size() << endl;
  cout << "# Kernel Timing: " << config.machine->getKernelTime() << endl;
  cout << "# Inversion Timing: " << config.machine->getInversionTime() << endl;
  cout << "# Timing: " << config.machine->getTrainingTime() << endl;
  cout << "# Leave One Out Error: " << config.machine->getLOO() << endl;
  if(config.k_fold > 0) {
    cout << "# " << config.k_fold << "-fold Cross Validation Error: " << config.k_fold_error << endl;
  }
  if(config.verify_loo) {
    cout << "# Real LOO: " << config.machine->getRealLOO(config.problem->y) << endl;
  }
}

void print_config_xml(TrainConfig &config) {
  cout << "<?xml version=\"1.0\" encoding=\"iso-8859-1\"?>" << endl;
  cout << "<lssvm>" << endl;
  cout << "  <model description=\"Model Filename\">" << config.model_filename << "</model>" << endl;
  cout << "  <trainingset description=\"Trainingset Filename\" size=\"" << config.problem->x.size() << "\"";
  cout << " dimensionality=\"" << config.problem->x[0].size() << "\">" << config.input_filename;
  cout << "</trainingset>" << endl;
  cout << "  <training type=\"";
  cout << ((config.machine->getClassification()) ? "classification" : "regression") << "\">" << endl;
  cout << "    <run>" << endl;
  cout << "      <machine description=\"Machine Configuration\">";
  cout << config.machine->toString() << "</machine>" << endl;
  cout << "      <kernel description=\"Kernel Function Specifier\">";
  cout << config.machine->getKernel()->toString() << "</kernel>" << endl;
  cout << "      <error description=\"Leave One Out error\">";
  cout << config.machine->getLOO() << "</error>" << endl;
  if(config.k_fold > 0) {
    cout << "      <error description=\"" << config.k_fold << "-fold CV error\">";
    cout << config.k_fold_error << "</error>" << endl;
  }
  if(config.verify_loo) {
    cout << "      <error description=\"Real Leave One Out error\">";
    cout << config.machine->getRealLOO(config.problem->y) << "</error>" << endl;
  }
  cout << "      <timing>" << endl;
  cout << "        <matrix_creation description=\"Kernel Matrix Creation Timing\">";
  cout << config.machine->getKernelTime() << "</matrix_creation>" << endl;
  cout << "        <matrix_inversion description=\"Kernel Matrix Inversion Timing\">";
  cout << config.machine->getInversionTime() << "</matrix_inversion>" << endl;
  cout << "        <total description=\"Total Learn Timing\">";
  cout << config.machine->getTrainingTime() << "</total>" << endl;
  cout << "      </timing>" << endl;
  cout << "    </run>" << endl;
  cout << "  </training>" << endl;
  cout << "</lssvm>" << endl;
}

void create_machine(TrainConfig &config) {
  char machine_cnf[10];
  int machine_type;
  sscanf(config.machine_str.c_str(), "%d:%s", &machine_type, machine_cnf);
  switch(machine_type) {
    case 1: // reference
      config.machine = new ReferenceLSSVM(config.machine_c);
      break;
    case 2: // partial
      double prob;
      prob = 0.0;
      sscanf(machine_cnf, "%lf", &prob);
      if(prob != 0.0) {
        config.machine = new PartialLSSVM(config.machine_c, prob);
      } else {
        config.machine = new PartialLSSVM(config.machine_c);
      }
      break;
    //case 3:
      //break;
    case 0: // cholesky
    default: // just in case...
      config.machine = new LSSVM(config.machine_c);
      break;
  }
  switch(config.problem_type) {
    case 0: // classification
      config.machine->setClassification(true);
      break;
    case 1: // regression
      config.machine->setClassification(false);
      break;
    case 2: // auto sensing
    default:
      config.machine->setClassification(config.problem->classification);
      break;
  }
}

void train_machine(TrainConfig &config) {
  create_machine(config);
  config.machine->setKernel(Kernel::fromString(config.kernel_str));
  if(config.k_fold == 0) {
    config.machine->train(config.problem->x, config.problem->y);
    config.machine->saveToFile(config.model_filename);
  } else {
    config.k_fold_error = config.machine->getKFoldCV(config.problem->x, config.problem->y, config.k_fold);
  }
}


int main(int argc, char** argv) {
  registerKernels();
  TrainConfig* config = new TrainConfig();
  try {
    // read params from arguments into config
    read_params(argc, argv, *config);
    // load problem from sparse file
    config->problem->readSparseFile(config->input_filename);
    // train the machine based on the config and problem
    train_machine(*config);
    // print config and results
    if(config->xml) {
      print_config_xml(*config);
    } else {
      print_config(*config);
    }
    //cout << "Real LOO: " << config->machine->getRealLOO(config->problem->y) << endl;
    // delete config
    delete(config);
  } catch(string message) {
    std::cerr << std::endl << "Exception caught: " << message << std::endl;
    // delete config
    delete(config);
    return -1;
  }

  return 0;
}


