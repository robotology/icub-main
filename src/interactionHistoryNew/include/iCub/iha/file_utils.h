#ifndef __IHA_FILE_UTILS_H__
#define __IHA_FILE_UTILS_H__

#include <stdio.h>
#include <fstream>
#include <limits>
#include <ace/OS.h>

namespace iCub {
	namespace iha {
		bool fileExists(const char * file);
		bool dirExists(const char * dir);
		bool prepareDirectoryNew(const char * base, char * newdir);
		static bool openOutputFile(const char * of);
		static void writeToOuputFile(const char * s);
		static void closeOutputFile();
		static bool openInputFile(char * inf);
		static bool readFromInputFile(char * buf, int buf_sz);
		static void closeInputFile();
	}
}

ofstream oFP;
ifstream iFP;

bool iCub::iha::fileExists(const char * file) {
	ACE_stat fb;
	if (ACE_OS::stat( file, &fb) < 0)
		return false;
	else 
		return true;
}

bool iCub::iha::dirExists(const char * dir) {
	bool exists=false;
	ACE_DIR * ace_dir = ACE_OS::opendir( dir );
	if ( ace_dir != NULL ) {
		exists=true;
		ACE_OS::closedir( ace_dir );
	}
	return exists;			
}


bool iCub::iha::prepareDirectoryNew(const char * base, char * newdir) {
	int dirNum=0;
	char dirNumS[30];
	char dataDir[100];

	sprintf(dataDir, "%s0000", base);
	while (fileExists(dataDir)) {
		dirNum++;
		sprintf(dirNumS,"0000%d",dirNum);
		int start = strlen(dirNumS) - 4;
		
		sprintf(dataDir, "%s%s", base,dirNumS+start);
	}
	
	fprintf(stderr,"Creating dir: %s\n",dataDir);
	
	if (ACE_OS::mkdir(dataDir)>=0) {
		strcpy (newdir, dataDir);	
		return true;
	}
	
	return false;
}

static bool iCub::iha::openOutputFile(const char * of) {
	// open the data output file
	oFP.open( of, ios::app);

	if (oFP.fail()) {
		fprintf(stderr,"Error: opening file %s\n",of);
		return false;
	}

	return true;
}

static void iCub::iha::writeToOuputFile(const char * s) {
	oFP << s;
	oFP.flush();
}

static void iCub::iha::closeOutputFile() {
	oFP.flush();
	oFP.close();
}

static bool iCub::iha::openInputFile(char * inf) {
	// open the data output file
	iFP.open( inf, ios::in);
	if (iFP.fail()) {
		fprintf(stderr,"Error: opening file %s\n",inf);
		return false;
	}

	return true;
}
static bool iCub::iha::readFromInputFile(char * buf, int buf_sz) {
	if (iFP.good() && !iFP.eof()) {
		iFP.getline(buf, buf_sz);
	} 
	if (iFP.good()) {
		return true;
	} else {
		return false;
	}
}

static void iCub::iha::closeInputFile() {
	iFP.close();
}

#endif
