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

#ifndef __VISLAB_YARP_UTIL_OPTION_H_
#define __VISLAB_YARP_UTIL_OPTION_H_

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

namespace vislab {
namespace yarp {
namespace util {

/**
 * This class represents an option that is able to validate itself by its {@link Type}.
 *
 * @author Christian Wressnegger
 * @date 2009
 */
class Option {
	static const ::yarp::os::Value DEFAULT_VALUE;

public:

	/**
	 * The basic type an {@link Option} may be of. This type accepts any value and is supposed
	 * to be extended to create more complex types.
	 *
	 * @author Christian Wressnegger
	 * @date 2009
	 */
	class Type {
	public:
		/**
		 * The default constructor.
		 */
		Type() {
		}

		/**
		 * The destructor.
		 */
		virtual ~Type() {
		}

		/**
		 * Validates the given {@link ::yarp::os::Value}.
		 * @param v The {@link ::yarp::os::Value} to be validated.
		 * @param output The stream to write possible error message to.
		 * @return An indicator wheter the given {@link ::yarp::os::Value} is valid or not.
		 */
		virtual bool validate(const ::yarp::os::Value& v, std::ostream* const output = NULL) const {
			return true;
		}

		/**
		 * Returns a human readable string that describes this object.
		 * @return A human readable string that describes this object.
		 */
		virtual ::yarp::os::ConstString toString() const {
			return "<any value>";
		}
	};

	/**
	 * This {@link Type} contains a list of values the {@link Option} may contain.
	 *
	 * @author Christian Wressnegger
	 * @date 2009
	 */
	class List: public Type {
		const std::vector< ::yarp::os::Value> options;
	public:
		/**
		 * The constructor.
		 * @param l The list of valid values.
		 * @return
		 */
		List(const std::vector< ::yarp::os::Value>& l);
		/**
		 * The destructor.
		 */
		virtual ~List();
		/**
		 * @see Type#validate(const ::yarp::os::Value&, std:ostream* const)
		 */
		virtual bool validate(const ::yarp::os::Value& v, std::ostream* const output = NULL) const;
		/**
		 * @see Type#toString()
		 */
		virtual ::yarp::os::ConstString toString() const;
	};

	/**
	 * This class implements a boolean {@link Type} for {@link Option}s.
	 *
	 * @author Christian Wressnegger
	 * @date 2009
	 */
	class Boolean: public Type {
		List* o;
	public:
		/**
		 * The constructor
		 * @param trueValue The {@link ::yarp::os::ConstString} representing the "true" value.
		 * @param falseValue The {@link ::yarp::os::ConstString} representing the "true" value.
		 * @return
		 */
		Boolean(::yarp::os::ConstString trueValue = "true", ::yarp::os::ConstString falseValue =
				"false");
		/**
		 * The destructor.
		 */
		virtual ~Boolean();
		/**
		 * @see Type#validate(const ::yarp::os::Value&, std:ostream* const)
		 */
		virtual bool validate(const ::yarp::os::Value& v, std::ostream* const output = NULL) const;
		/**
		 * @see Type#toString()
		 */
		virtual ::yarp::os::ConstString toString() const;
	};

	/**
	 * This {@link Type} is a specialization of the {@link BooleanType} in order to act as "on"/"off" switch.
	 *
	 * @author Christian Wressnegger
	 * @date 2009
	 */
	class OnOff: public Boolean {
	public:
		/**
		 * The default constructor.
		 */
		OnOff() :
			Boolean("on", "off") {
		}
	};

	/**
	 * This {@link Type} represents numeric values (int, float, double).
	 *
	 * @author Christian Wressnegger
	 * @date 2009
	 */
	class Number: public Type {
	public:
		/**
		 * The default constructor.
		 */
		Number() {
		}
		/**
		 * @see Type#validate(const ::yarp::os::Value&, std:ostream* const)
		 */
		virtual bool validate(const ::yarp::os::Value& v, std::ostream* const output = NULL) const;
		/**
		 * @see Type#toString()
		 */
		virtual ::yarp::os::ConstString toString() const;
	};

	/**
	 * This {@link Type} represents vector of numeric values (int, float, double).
	 *
	 * @author Christian Wressnegger
	 * @date 2009
	 */
	class Vector: public Type {
		bool isValid(const ::yarp::os::Value& v) const;
	public:
		/**
		 * The default constructor.
		 */
		Vector() {
		}
		/**
		 * @see Type#validate(const ::yarp::os::Value&, std:ostream* const)
		 */
		virtual bool validate(const ::yarp::os::Value& v, std::ostream* const output = NULL) const;
		/**
		 * @see Type#toString()
		 */
		virtual ::yarp::os::ConstString toString() const;
	};

	/** Accepts any value. */
	static const Type* SIMPLE;
	/** Accepts any numeric value. */
	static const Number* NUMERIC;
	/** Accepts boolean values ("true", "false"). */
	static const Boolean* BOOLEAN;
	/** Accepts "on" & "off" values. */
	static const OnOff* ON_OFF;
	/** Accepts vectors of numeric values. */
	static const Vector* VECTOR;

	/** "on" as {@link ::yarp::os::Value} */
	static const ::yarp::os::Value ON;
	/** "off" as {@link ::yarp::os::Value} */
	static const ::yarp::os::Value OFF;
	/** "true" as {@link ::yarp::os::Value} */
	static const ::yarp::os::Value TRUE_;
	/** "false" as {@link ::yarp::os::Value} */
	static const ::yarp::os::Value FALSE_;

private:

	const ::yarp::os::ConstString name;
	const ::yarp::os::ConstString description;
	const Type* type;
	::yarp::os::Value value;

	class Guard {
		~Guard() {
			delete SIMPLE;
			delete NUMERIC;
			delete BOOLEAN;
			delete ON_OFF;
			delete VECTOR;
		}
	};

public:

	/**
	 * The constructor.
	 * @param name The option's name
	 * @param description The description of this {@link Option}
	 * @param t The {@link Option::Type}/ validator of this {@link Option}.
	 * @param value The {@link ::yarp::os::Value} of this {@link Option}.
	 * @return
	 */
	Option(const ::yarp::os::ConstString& name, const ::yarp::os::ConstString& description,
			const Option::Type* const t = SIMPLE, const ::yarp::os::Value& value = DEFAULT_VALUE);
	/**
	 * The destructor.
	 */
	virtual ~Option();

	/**
	 * Returns the name of this {@link Option}.
	 * @return The name of this {@link Option}.
	 */
	const ::yarp::os::ConstString& getName() const;
	/**
	 * Returns the description of the {@link Option}.
	 * @return The description of the {@link Option}.
	 */
	const ::yarp::os::ConstString& getDescription() const;
	/**
	 * Returns the name of this {@link Option}.
	 * @return The name of this {@link Option}.
	 */
	const ::yarp::os::Value& getValue() const;
	/**
	 * Sets the value of this {@link Option}.
	 * @param v The new value of this {@link Option}.
	 * @param output Optional output for an eventual error message regarding the validation.
	 */
	void setValue(const ::yarp::os::Value& v, std::ostream* const output = NULL);

	/**
	 * Returns a human readable string that describes this object.
	 * @param shortened Flag to produce a compact description of the object
	 * @return A human readable string that describes this object.
	 */
	::yarp::os::ConstString toString(bool shortened = false) const;
};

}
}
}

#endif /* __VISLAB_YARP_UTIL_OPTION_H_ */
