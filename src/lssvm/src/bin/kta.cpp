/* LibLSSVM - Library for Least Squares Support Vector Machine Regression
 *
 * 2006-2007 Arjan Gijsberts, arjan@liralab.it
 *
 * Kernel Target Alignment executable
 *
 * Todo:
 *
 */


#include <iostream>
#include <string>

#include "iCub/KernelAlignment.h"
#include "iCub/MercerKernels.h"
#include "iCub/Problem.h"

using namespace std;
using namespace iCub::contrib::liblssvm;

class AlignmentConfig {
  public:
    AlignmentConfig() {
      // default kernel with id=0 with default settings
      this->kernel_str = "0";
      this->input_filename = "";
      this->machine_c = 1;
      this->xml = false;
      this->problem_type = 2;
      this->problem = new Problem();
      this->ka = new KernelAlignment();
      this->m_pp = false;
    }

    ~AlignmentConfig() {
      delete(problem);
      delete(ka);
    }

    string kernel_str;
    double machine_c;
    int problem_type;

    string input_filename;
    bool xml;
    bool m_pp;

    Problem* problem;
    KernelAlignment* ka;
};



void exit_with_help() {
  printf(
  "Usage: lssvm-train [options] training_set_file\n"
  "options:\n"
  "-k type[:param1[,param2]] : set type of kernel function (default 0)\n"
  "    0 -- polynomial: (u'*v + const_val2)^degree1\n"
  "    1 -- radial basis function: exp(-gamma*|u-v|^2)\n"
  "    2 -- gaussian radial basis function: exp(-|u-v|^2/2sigma^2)\n"
  "    3 -- boolean function: (1+gamma)^(u'*v)\n"
  "    4 -- sigmoid: tanh(gamma1*u'*v + const_val2)\n"
  "-c tradeoff : set the parameter C (default 1)\n"
  "-p type : set problem type (default 2)\n"
  "    0 -- Enforce classification\n"
  "    1 -- Enforce regression\n"
  "    2 -- Autosensing\n"
  "-m : set matrix preprocessing (default off)\n"
  "-x : set xml output (default off)\n"
  );
  exit(1);
}

void read_params(int argc, char** argv, AlignmentConfig &config) {
  int i;

  for(i=1;i<argc;i++) {
    if(argv[i][0] != '-') break;
    if(++i>=argc)
      exit_with_help();
    switch(argv[i-1][1]) {
      case 'k':
        config.kernel_str = argv[i];
        break;
      case 'c':
        config.machine_c = atof(argv[i]);
        break;
      case 'm':
        config.m_pp = true;
        i--;
        break;
      case 'x':
        config.xml = true;
        i--;
        break;
      case 'p':
        config.problem_type = atoi(argv[i]);
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
}


void print_config(AlignmentConfig &config) {
  cout << "# Kernel Target Alignment" << endl;
  cout << "#" << endl;
  cout << "# - Settings -" << endl;
  cout << "# Input filename: " << config.input_filename << endl;
  cout << "# Kernel: " << config.ka->getKernel().toString() << endl;
  cout << "# Machine C: " << config.ka->getC() << endl;
  cout << "# Matrix Preprocessing: " << config.ka->getPreprocessMatrix() << endl;
  cout << "#" << endl;
  cout << "# - Training -" << endl;
  cout << "# Problem Type: ";
  cout << ((config.problem->classification) ? "classification" : "regression") << endl;
  cout << "# Number of samples: " << config.problem->x.size() << "/" << config.problem->y.size() << endl;
  cout << "# Alignment Timing: " << config.ka->getAlignmentTime() << endl;
  cout << "# Kernel Target Alignment: " << config.ka->getAlignment() << endl;
}

void print_config_xml(AlignmentConfig &config) {
  cout << "<?xml version=\"1.0\" encoding=\"iso-8859-1\"?>" << endl;
  cout << "<kernel_alignment>" << endl;
  cout << "  <trainingset description=\"Trainingset Filename\" size=\"" << config.problem->x.size() << "\"";
  cout << " dimensionality=\"" << config.problem->x[0].size() << "\">" << config.input_filename;
  cout << "</trainingset>" << endl;
  cout << "  <kernel description=\"Kernel Function Specifier\">";
  cout << config.ka->getKernel().toString() << "</kernel>" << endl;
  cout << "  <alignment description=\"Kernel Target Alignment\">";
  cout << config.ka->getAlignment() << "</alignment>" << endl;
  cout << "  <timing>" << endl;
  cout << "    <calculation description=\"Kernel Alignment Calculation Timing\">";
  cout << config.ka->getAlignmentTime() << "</calculation>" << endl;
  cout << "  </timing>" << endl;
  cout << "</kernel_alignment>" << endl;
}


void train_machine(AlignmentConfig &config) {
  config.ka = new KernelAlignment(config.machine_c);
  config.ka->setKernel(*Kernel::fromString(config.kernel_str));
  // set matrix preprocessing if enabled
  config.ka->setPreprocessMatrix(config.m_pp);
  // preprocess labels for regression
  if(!config.problem->classification) {
    config.ka->preprocessLabels(config.problem->y);
  }
  config.ka->calculate(config.problem->x, config.problem->y);
}


int main(int argc, char** argv) {
  registerKernels();
  AlignmentConfig* config = new AlignmentConfig();
  try {
    // read params from arguments into config
    read_params(argc, argv, *config);
    // load problem from sparse file
    config->problem->readSparseFile(config->input_filename);
    // train the kernel alignment based on the config and problem
    train_machine(*config);
    // print config and results
    if(config->xml) {
      print_config_xml(*config);
    } else {
      print_config(*config);
    }
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

