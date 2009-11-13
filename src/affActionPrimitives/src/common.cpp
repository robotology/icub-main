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

#include <iCub/common.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::os;


void readVector(const Bottle& in, double* out, const int valueSize)
{
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

void readVector(const Bottle& in, Vector& out, const int valueSize)
{
	int size = valueSize <= 0 ? in.size() - 1 : valueSize;

	Vector tmp=out;
	out.resize(size);
	for (int i=0; i<out.length() && i<tmp.length(); i++)
		out[i]=tmp[i];

	readVector(in, out.data(), size);
}

void readVectors(Bottle& in, map<const string, Vector>& output, const string values[],
		const int numValues, const int valueSize)
{
	for (int i = 0; i < numValues; i++) {
		Bottle b = in.findGroup(values[i].c_str());
		readVector(b, output[values[i]], valueSize);
	}
}

void readMatrices(const Bottle& in, map<const string, Matrix>& output, bool isSubProperty)
{
	int startIdx = (isSubProperty ? 1 : 0);
	if (in.size() > startIdx) {
		// 0: Group title
		for (int i = startIdx; i < in.size(); i++) {
			Bottle line(in.get(i).toString());
			if (line.size() > 1) {
				int valueSize = line.size() - 1;
				string key = line.get(0).asString().c_str();
				//			transform(key.begin(), key.end(), key.begin(), ::tolower);
				if (output.find(key) == output.end()) {
					output.insert(std::make_pair<const string, Matrix>(key, Matrix(1, valueSize)));
				} else {
					Matrix& m = output[key];
					m.resize(m.rows() + 1, max(m.cols(), valueSize));
				}
				Matrix& m = output[key];
				readVector(line, m[m.rows() - 1], valueSize);
			}
		}
	}
}


