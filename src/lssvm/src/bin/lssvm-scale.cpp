/* LibLSSVM - Library for Least Squares Support Vector Machine Regression
 *
 * 2006-2007 Arjan Gijsberts, arjan@liralab.it
 *
 * LSSVM Scale file
 *
 * Note:
 * - Nothing more than svm-scale.c from LibSVM extended with target scaling to 
 *   mean = 0 and standard deviation 1.
 *
 * Todo: 
 */
/*
	scale attributes to [lower,upper]
	usage: scale [-l lower] [-u upper] [-y y_lower y_upper] -d
	    [-s filename] [-r filename] filename
*/

#include "iCub/Problem.h"
#include <string>

using namespace std;
using namespace iCub::contrib::liblssvm;

void exit_with_help() {
  printf(
  "Usage: lssvm-scale [options] input_file\n"
  "options:\n"
  "-y min max : boundaries for the labels\n"
  "-f : scale features to mean 0 and unit standard deviation\n"
  "-l : scale labels to mean 0 and unit standard deviation\n"
  "-s : only print statistics about the data set\n"
  );
  exit(1);
}


void process_params(int argc, char** argv) {
  int i;
  bool norm_labels = false;
  bool norm_features = false;
  bool print_stats = false;
  double upper = 0.0;
  double lower = 0.0;

  string input_filename;
  string output_filename;

  Problem* problem = new Problem();

  for(i=1;i<argc;i++) {
    if(argv[i][0] != '-') break;
    if(++i>=argc)
      exit_with_help();
    switch(argv[i-1][1]) {
      case 'y':
        lower = atof(argv[i]);
        ++i;
        upper = atof(argv[i]);
        break;
      case 'l':
        norm_labels = true;
        i--;
        break;
      case 'f':
        norm_features = true;
        i--;
        break;
      case 's':
        print_stats = true;
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

  input_filename = argv[i];

  //cout << "Reading file..." << endl;
  problem->readSparseFile(input_filename);
  //cout << "done!" << endl;
  if(print_stats) {
    cout << "# Statistics" << endl;
    cout << "# File: " << input_filename << endl;
    cout << "# Number of Samples: " << problem->x.size() << endl;
    cout << "# Number of Features: " << problem->x[0].size() << endl;
    cout << "# Problem Type: " << (problem->classification ? "classification" : "regression") << endl;
    if(problem->classification) {
      int cnt_positive = 0;
      int cnt_negative = 0;
      for(int i = 0; i < problem->y.size(); i++) {
        if(problem->y[i] == -1) cnt_negative++;
        if(problem->y[i] == 1) cnt_positive++;
      }
      double pct_positive = (double(cnt_positive) / (cnt_positive + cnt_negative)) * 100;
      double pct_negative = (double(cnt_negative) / (cnt_positive + cnt_negative)) * 100;
      cout << "# Balance: " << cnt_positive << "/" << cnt_negative << " ";
      cout << "(" << pct_positive << "/" << pct_negative << ")" << endl;
    } else {
      if(norm_labels) {
        std::pair<double, double> labels = problem->normalizeLabels();
        cout << "# Label Scaling: mu=" << labels.first << ", sigma=" << labels.second << endl;
      }
    }
  } else {
    if(norm_labels) {
      //cout << "Normalizing labels...";
      problem->normalizeLabels();
      //cout << "done!" << endl;
    } else if (upper > lower) {
      //cout << "Normalizing features...";
      problem->scaleLabels(lower, upper);
      //cout << "done!" << endl;
    }
  
    if(norm_features) {
      //cout << "Normalizing features...";
      problem->normalizeFeatures();
      //cout << "done!" << endl;
    }
  
    problem->writeSparseFile(cout);
  }
  delete(problem);
}

int main(int argc, char** argv) {
  try {
    process_params(argc, argv);
  } catch(string message) {
    std::cerr << std::endl << "Exception caught: " << message << std::endl;
    return -1;
  }

  return 0;
}

