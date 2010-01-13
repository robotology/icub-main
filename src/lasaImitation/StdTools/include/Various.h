#ifndef __VARIOUS_H__
#define __VARIOUS_H__

#include <string>
#include <vector>
using namespace std;

string Int01ToString(int i);
string Int02ToString(int i);
string Int03ToString(int i);
string Int04ToString(int i);

string IntToString(int i);
string FloatToString(float f);
string DoubleToString(double d);
string BoolToString(bool b);

vector<string> Tokenize(string params);
vector<string> Tokenize(string params, string delim, string exDelim);
string Serialize(vector<string>, int start=0, int cnt=-1);

string RemoveSpaces(string s);

string GetPathFromFilename(string fname);
int    GetConsecutiveFileCount(const char * dir, const char *nameTag, int maxS=20);

#ifdef WIN32          // Linux Wrap for Win32
#define usleep(X)     Sleep((X) / 1000)
#define sleep(X)      Sleep((X) * 1000)

#define	MSG_DONTWAIT	0
#define socklen_t     int FAR


#else                 // Windows warp for linux

long int GetTickCount();

#endif

vector<string> ScanDir(string dirName);
vector<string> AutoCompletion(vector<string> & choices, string & target);

#endif
