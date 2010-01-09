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

#include "vislab/yarp/util/Option.h"

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;

namespace vislab {
namespace yarp {
namespace util {

const Option::Type* Option::SIMPLE = new Option::Type();
const Option::Number* Option::NUMERIC = new Option::Number();
const Option::Boolean* Option::BOOLEAN = new Option::Boolean();
const Option::OnOff* Option::ON_OFF = new Option::OnOff();
const Option::Vector* Option::VECTOR = new Option::Vector();

const Value Option::ON = Value("on");
const Value Option::OFF = Value("off");
const Value Option::TRUE_ = Value("true");
const Value Option::FALSE_ = Value("false");

const Value DEFAULT_VALUE = Value("");

Option::List::List(const vector<Value>& l) :
	options(l) {
	if (l.size() < 2) {
		throw invalid_argument("At least 2 options have to be provided");
	}
}

Option::List::~List() {
}

bool Option::List::validate(const Value& v, ostream* const output) const {
	for (size_t i = 0; i < options.size(); i++) {
		if (options[i].toString() == v.toString()) {
			return true;
		}
	}
	if (output != NULL) {
		*output << "has to be one of: " << toString() << endl;
	}
	return false;
}

ConstString Option::List::toString() const {
	ostringstream oss;
	oss << options[0].toString();
	for (size_t i = 1; i < options.size(); i++) {
		oss << ", " << options[i].toString();
	}
	return oss.str().c_str();
}

Option::Boolean::Boolean(ConstString trueValue, ConstString falseValue) {

	vector<Value> v;
	v.push_back(Value(trueValue));
	v.push_back(Value(falseValue));

	o = new List(v);
}

Option::Boolean::~Boolean() {
	delete o;
}

bool Option::Boolean::validate(const ::yarp::os::Value& v, ostream* const output) const {
	return o->validate(v, output);
}

ConstString Option::Boolean::toString() const {
	return o->toString();
}

bool Option::Number::validate(const Value& v, ostream* const output) const {
	if (!v.isDouble() && !v.isInt()) {
		if (output != NULL) {
			*output << "has to be a numerical value: " << toString() << endl;
		}
		return false;
	}
	return true;
}

ConstString Option::Number::toString() const {
	return "integer | float | double";
}

bool Option::Vector::isValid(const Value& v) const {
	if (!v.isList()) {
		return v.isDouble() || v.isInt();
	}
	Bottle* b = v.asList();
	for (int i = 0; i < b->size(); i++) {
		Value& v = b->get(i);
		if (!v.isDouble() && !v.isInt()) {
			return false;
		}
	}
	return true;
}

bool Option::Vector::validate(const Value& v, ostream* const output) const {
	if (!isValid(v)) {
		if (output != NULL) {
			*output << "has to be a vector of numerical values: " << toString() << endl;
		}
		return false;
	}
	return true;
}

ConstString Option::Vector::toString() const {
	return "numeric (, numeric)*";
}

Option::Option(const ConstString& name, const ::yarp::os::ConstString& description,
		const Option::Type* const t, const ::yarp::os::Value& value) :
	name(name), description(description) {

	type = t;
	if (!type->validate(value, NULL)) {
		throw invalid_argument("The initialization value doesn't validate.");
	}
	this->value = value;
}

Option::~Option() {
}

void Option::setValue(const Value& v, ostream* const output) {
	Value v_ = v.isNull() ? Value("") : v;
	ostringstream oss;
	if (type->validate(v_, output != NULL ? &oss : NULL)) {
		value = v_;
	} else if (output != NULL) {
		*output << getName() << " " << oss.str();
	}
}

const Value& Option::getValue() const {
	return value;
}

const ConstString& Option::getName() const {
	return name;
}

const ConstString& Option::getDescription() const {
	return description;
}

ConstString Option::toString(bool shortened) const {
	ostringstream oss;
	oss << getName() << " = '" << value.toString().c_str() << "' (" << type->toString() << ")";
	return oss.str().c_str();
}

}
}
}
