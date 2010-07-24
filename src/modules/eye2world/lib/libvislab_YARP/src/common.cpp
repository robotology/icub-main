/*******************************************************************************
 * Copyright (C) 2009 Christian Wressnegger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *******************************************************************************/

#include "vislab/yarp/util/common.h"

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::os;

using namespace vislab::util;

namespace vislab {
namespace yarp {
namespace util {

void addMultilineString(Bottle& b, const ConstString& str) {
	vector<string> v;
	explode(str.c_str(), "\n", v);
	for (unsigned int i = 0; i < v.size(); i++) {
		b.addString(v[i].c_str());
	}
}

void parseListOfDoubles(const ConstString& in, Vector& out) {
	vector<double> v;
	parseListOf<double> (in, v);
	if (in.length() <= 0) {
		out.clear();
	} else {
		out.resize(v.size());
		for (int i = 0; i < out.size(); i++) {
			out[i] = v[i];
		}
	}
}

void stripRFProperties(Property& p) {
	string strs[] = { "from", "style", "/", "default_capability", "capability_directory" };
	for (int i = 0; i < 5; i++) {
		p.unput(strs[i].c_str());
	}
}

bool replaceDoubleSlash(ConstString &str) { // ô_O weird
	bool modified = false;
	string sz = "";
	sz += str;
	string::size_type loc;
	while ((loc = sz.find("//", 0)) != string::npos) {
		sz.erase(loc + 1, 1);
		str = sz.c_str();
		modified = true;
	}
	return modified;
}

void printVector(const double* const v, size_t len, ostream& s) {
	for (size_t i = 0; i < len; i++) {
		s << v[i] << " ";
	}
	s << endl;
}

void printVector(const Vector &v, ostream& s) {
	printVector(v.data(), v.size(), s);
}

void printMatrix(const Matrix &m, ostream& s) {
	for (int r = 0; r < m.rows(); r++) {
		printVector(m.getRow(r), s);
	}
	cout << endl;
}

void readVector(const Bottle& in, double* out, const int valueSize) {
	if (!in.isNull()) {
		if (in.size() - 1 != valueSize) {
			cout << "Warning: The expected size of `" << in.get(0).toString() << "` is " << valueSize
					<< ", but it was " << in.size() - 1 << endl;
		}
		for (int j = 0; j < in.size() && j < valueSize; j++) {
			out[j] = in.get(j + 1).asDouble();
		}
	}
}

void readVector(const Bottle& in, Vector& out, const int valueSize) {
	int size = valueSize <= 0 ? in.size() - 1 : valueSize;
	out.resize(size);
	readVector(in, out.data(), size);
}

void readVectors(Bottle& in, map<const string, Vector>& output, const string values[],
		const int numValues, const int valueSize) {
	for (int i = 0; i < numValues; i++) {
		Bottle b = in.findGroup(values[i].c_str());
		readVector(b, output[values[i]], valueSize);
	}
}

void readMatrices(const Bottle& in, map<const string, Matrix>& output, bool isSubProperty) {
	map<const string, vector<Vector> > m;
	// TODO: multimap

	int startIdx = (isSubProperty ? 1 : 0);
	if (in.size() > startIdx) {
		// 0: Group title
		for (int i = startIdx; i < in.size(); i++) {
			Bottle line(in.get(i).toString());
			if (line.size() > 1) {
				int valueSize = line.size() - 1;
				string key = line.get(0).asString().c_str();
				//	transform(key.begin(), key.end(), key.begin(), ::tolower);

				Vector v(valueSize);
				readVector(line, v.data(), valueSize);
				m[key].push_back(v);
			}
		}
	}

	// This was done, since Matrices and Vectors are not supposed to grow in size.
	map<const string, vector<Vector> >::const_iterator itr;
	for (itr = m.begin(); itr != m.end(); ++itr) {
		vector<Vector> v = itr->second;
		// define size
		int valueSize = v[0].size();
		output[itr->first] = Matrix(itr->second.size(), valueSize);
		// populate new Matrix
		for (size_t i = 0; i < v.size(); i++) {
			// TODO: memcpy(m[i], v[i].data(), valueSize);
			for (int j = 0; j < v[i].length(); j++) {
				output[itr->first][i][j] = v[i][j];
			}
		}
	}
}

void readMotionSequence(const Bottle& in, MotionSequence& output) {
	Bottle seq = in;
	Bottle& b = seq.findGroup("DIMENSIONS");
	int numPoses = b.find("numberOfPoses").asInt();
	int numJoints = b.find("numberOfJoints").asInt();
	if (numJoints <= 0 || numPoses <= 0) {
		// cout << "Warning: " <<  endl;
	} else {
		output.setNumJoints(numJoints);
		Vector v(numJoints);

		for (int i = 0; i < numPoses; i++) {
			ostringstream ss;
			ss << "POSITION" << i;
			Bottle& position = seq.findGroup(ss.str().c_str());

			Motion m(numJoints);

			readVector(position.findGroup("jointPositions"), v, numJoints);
			m.setPosition(v);
			readVector(position.findGroup("jointVelocities"), v, numJoints);
			m.setVelocity(v);
			m.setTiming(position.find("timing").asDouble());

			output.addMotion(m);
		}
	}
}

void readMotionSequences(const Bottle& in, map<const string, MotionSequence>& output) {
	int startIdx = 0;
	for (int i = startIdx; i < in.size(); i++) {
		Bottle line(in.get(i).toString());
		if (line.size() > 1) {
			string key = line.get(0).asString().c_str();
			MotionSequence& sequence = output[key];
			readMotionSequence(line, sequence);
			if (sequence.length() <= 0) {
				output.erase(key);
			}
		}
	}
}

}
}
}
