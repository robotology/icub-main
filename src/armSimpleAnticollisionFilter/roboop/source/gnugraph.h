/*
ROBOOP -- A robotics object oriented package in C++
Copyright (C) 1996-2004  Richard Gourdeau

This library is free software; you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation; either version 2.1 of the
License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


Report problems and direct all questions to:

Richard Gourdeau
Professeur Agrege
Departement de genie electrique
Ecole Polytechnique de Montreal
C.P. 6079, Succ. Centre-Ville
Montreal, Quebec, H3C 3A7

email: richard.gourdeau@polymtl.ca
-------------------------------------------------------------------------------
Revision_history:

2004/07/01: Etienne Lachance
   -Added doxygen documentation.

2004/07/01: Ethan Tira-Thompson
    -Added support for newmat's use_namespace #define, using ROBOOP namespace

2004/08/10: Etienne Lachance
    -Added class Plot3d.
    -Removed using ROBOOP namespace
-------------------------------------------------------------------------------
*/

#ifndef GNUGRAPH_H
#define GNUGRAPH_H


/*!
  @file gnugraph.h
  @brief Header file for graphics definitions.
*/

//! @brief RCS/CVS version.
static const char header_gnugraph_rcsid[] = "$Id: gnugraph.h,v 1.1 2007/07/24 16:03:09 amaldo Exp $";

#ifdef _MSC_VER                  // Microsoft
#pragma warning (disable:4786)  // Disable decorated name truncation warnings 
#pragma warning (disable:4503)  // Disable decorated name truncation warnings 
#endif

#if defined(__WIN32__) || defined(_WIN32) || defined(__NT__)  || defined(__CYGWIN__)      /* Windows 95/NT */

#define GNUPLOT "wgnuplot.exe"
#define STRICT
#include <windows.h>

#ifdef _MSC_VER 
#define snprintf	_snprintf
#endif

#else // Unix 

#define GNUPLOT "gnuplot"
#include <sys/types.h>
#include <unistd.h>
#endif

#include <stdio.h>
#include <stdexcept>

#include <boost/shared_ptr.hpp>

#define WANT_STRING                  /* include.h will get string fns */
#define WANT_STREAM                  /* include.h will get stream fns */
#define WANT_FSTREAM                 /* include.h will get fstream fns */
#define WANT_MATH                    /* include.h will get math fns */
                                     /* newmatap.h will get include.h */
#include "newmatap.h"                /* need matrix applications */
#include "newmatio.h"                /* need matrix output routines */

#ifdef use_namespace
using namespace NEWMAT;
#endif

#include <sys/stat.h>

#include <sstream>
#include <vector>


#define OUT_OF_MEMORY       -1
#define X_Y_DATA_NO_MATCH   -2
#define LABELS_NBR_NO_MATCH -3



typedef enum {
    LINES,
    DATAPOINTS,
    LINESPOINTS,
    IMPULSES,
    DOTS,
    STEPS,
    BOXES
} LineType_en;


#define NCURVESMAX  10  // maximum number of curves in the same Plot2d 

class Plot2d;

/*!
  @class GNUcurve
  @brief Object for one curve.
 */
class GNUcurve {

  public:
    GNUcurve(const std::vector<double> & x, std::vector<double> & y, 
             const std::string & label = "", LineType_en enLineType = LINES);
    GNUcurve(void);
    void dump(void);

    std::vector<double> vdX;
    std::vector<double> vdY;
    std::string clabel;       //!< string defining the curve label for the legend
    LineType_en enLineType;    //!< Line type
};

typedef boost::shared_ptr<GNUcurve> PSHR_Curve;
typedef std::vector<PSHR_Curve> VectorCurves;


/*!
  @class Plot2d
  @brief 2d plot object.
*/
class Plot2d {
public:
   Plot2d(void);
   void dump(void);
   void settitle(const std::string & t);
   void setxlabel(const std::string & t);
   void setylabel(const std::string & t);
   void addcurve(const Matrix & data, const std::string & label = "", 
                 LineType_en enLineType = DATAPOINTS);
   void gnuplot(void);
   void addcommand(const std::string & gcom);

private:
   std::string  title;        //!< Graph title.
   std::string  xlabel;       //!< Graph x axis.
   std::string  ylabel;       //!< Graph y axis.
   std::string  gnucommand;   //!< GNU plot command.

   VectorCurves vCurves;
};

/*!
  @class Plot3d
  @brief 3d plot object.
*/
class Plot3d 
{
   std::string 
     title,              //!< Graph title.
     xlabel,             //!< Graph x axis.
     ylabel,             //!< Graph y axis.
     zlabel;             //!< Graph z axis.
public:
   Plot3d(){}            //!< Default constructor.
   void settitle(const std::string & t);
   void setxlabel(const std::string & t);
   void setylabel(const std::string & t);
   void setzlabel(const std::string & t);
   void gnuplot(const Matrix & xyz);
};

#define IO_COULD_NOT_OPEN_FILE  -1
#define IO_MISMATCH_SIZE        -2
#define IO_DATA_EMPTY           -3
#define IO_MISMATCH_ELEMENT_NBR -4
#define PROBLEM_FILE_READING    -5


/*!
  @class IO_matrix_file.
  @brief Read and write data at every iterations in a file.
*/
class IO_matrix_file {
public:
   IO_matrix_file(const std::string & filename);
   short write(const std::vector<Matrix> & data);
   short write(const std::vector<Matrix> & data, const std::vector<std::string> & title);
   short read(std::vector<Matrix> & data);
   short read(std::vector<Matrix> & data, std::vector<std::string> & title);
   short read_all(std::vector<Matrix> & data, std::vector<std::string> & data_title);
private:
   int 
     position_read,       //!< Position to read the file.
     nb_iterations_write, //!< Number of iterations in writing mode.
     nb_iterations_read,  //!< Number of iterations in reading mode.
     nb_element;          //!< Number of elements to read or write.
   std::string filename;       //!< File name.
};


/*!
  @class Plot_file
  @brief Creates a graphic from a data file.
*/
class Plot_file : public IO_matrix_file, Plot2d
{
public:
   Plot_file(const std::string & filename);
   short graph(const std::string & title_graph, const std::string & label, const short x,
               const short y, const short x_start, const short y_start,
               const short y_end);
private:
   std::vector<Matrix> data_from_file;  //!< Data file.
   std::vector<std::string> data_title;      //!< Data file title.
};



short set_plot2d(const char *title_graph, const char *x_axis_title, const char *y_axis_title,
                 const char *label, LineType_en enLineType, const Matrix &xdata, const Matrix &ydata,
                 int start_y, int end_y);

short set_plot2d(const char *title_graph, const char *x_axis_title, const char *y_axis_title,
                 const vector<char *> label, LineType_en enLineType, const Matrix &xdata, 
                 const Matrix &ydata, const vector<int> & data_select);

short set_plot3d(const Matrix & xyz, const std::string & title_graph, const std::string & x_axis_title, 
		 const std::string & y_axis_title, const std::string & z_axis_title);


#endif

