
#include <string>
#include <list>
#include <vector>

extern std::string sequenceToPattern(std::string seq);

extern std::string extendSequence(std::string seq, 
				  std::string pat, 
				  int ct=1);

extern std::vector<int> extendSequence(const std::vector<int>& seq, 
				       int ct=1);
