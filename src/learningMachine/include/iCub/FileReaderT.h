/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Header file for template that supports reading lines of a file as class
 * instances. This is a copy of the same template in iCub/src/sendCmd/
 *
 */

#ifndef ICUB_FILEREADERT__
#define ICUB_FILEREADERT__

#include <string>
#include <fstream>
#include <stdexcept>

namespace iCub {
namespace learningmachine {


/**
 * Template class that supports reading lines of a file to object instances
 * using a fromString(char* line) method (e.g. YARP Bottles).
 *
 * \author Arjan Gijsberts
 */
template<class T>
class FileReaderT {
private:
    /**
     * Stream object for the incoming data.
     */
    std::istream* stream;

    /**
     * Internal pointer to opened file streams that need to be cleaned.
     */
    std::ifstream* f_stream;

    /**
     * Clear the data associated with the internal file stream.
     */
    void clearStream() {
        // close file stream if present
        if(this->f_stream != (std::ifstream*) 0) {
            this->f_stream->close();
        }
        // do not need to clear normal stream, as we already clear f_stream and
        // don't want to delete std::cin
        delete this->f_stream;
        this->f_stream = (std::ifstream*) 0;
    }

    /**
     * Checks whether the current file has a stream.
     *
     * @exception std::runtime_error no open stream
     */
    void checkStream() {
        if(this->stream == (std::istream*) 0) {
            throw std::runtime_error("no stream opened.");
        }
    }

public:
    /**
     * Default constructor.
     */
    FileReaderT() {
        stream = (std::istream*) 0;
        f_stream = (std::ifstream*) 0;
    }

    /**
     * Default destructor.
     */
    ~FileReaderT() {
        this->clearStream();
    }

    /**
     * Opens the given stream for reading.
     *
     * @param s the input stream
     */
    virtual void open(std::istream& s) {
        // destroy previous stream if applicable
        this->stream = &s;
    }

    /**
     * Opens a stream to the given filename for reading.
     *
     * @param f the filename
     */
    virtual void open(const std::string& f) {
        // clear previous filestream
        this->clearStream();
        // put stream on the heap!
        this->f_stream = new std::ifstream(f.c_str());
        if(!this->f_stream->is_open()) {
            this->clearStream();
            throw std::runtime_error(std::string("could not open file '" + f + "'"));
        }
        return this->open(*this->f_stream);
    }

    /**
     * Checks whether there are more items left in the stream.
     *
     * @returns true if the stream has more items left
     * @exception std::runtime_error no open stream
     */
    virtual bool hasNext() {
        this->checkStream();
        // peek so that eof bit is set
        this->stream->peek();
        return !this->stream->eof();
    }

    /**
     * Resets the stream to the beginning. Note that this may not be possible
     * for specific combinations of streams (e.g. stdin) and platforms.
     */
    virtual void reset() {
        this->stream->clear();
        this->stream->seekg(0, std::ios::beg);
    }

    /**
     * Returns a pointer to the next item in the file. Note that the caller
     * takes responsibility for cleaning the memory space associated with the
     * pointer.
     *
     * @returns a pointer to the new item
     * @exception std::runtime_error no open stream
     * @exception std::runtime_error end of stream
     */
    T* getNext() {
        // check if there are actually more items
        if(!hasNext()) {
            throw std::runtime_error("end of stream.");
        }

        // read next line
        std::string line;
        getline(*this->stream, line);

        // construct and fill object
        T* obj = new T();
        obj->fromString(line.c_str());

        return obj;
    }
};

} // learningmachine
} // iCub

#endif
