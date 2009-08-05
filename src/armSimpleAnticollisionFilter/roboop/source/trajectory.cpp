/*
Copyright (C) 2002-2004  Etienne Lachance

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

email: etienne.lachance@polymtl.ca or richard.gourdeau@polymtl.ca

-------------------------------------------------------------------------------
Revision_history:

2004/06/02: Etienne Lachance
    -Added class Trajectory_Select.

2004/07/01: Ethan Tira-Thompson
    -Added support for newmat's use_namespace #define, using ROBOOP namespace.

2005/06/10: Etienne Lachance
    -Missing file warning message.

2005/11/06: Etienne Lachance
    - No need to provide a copy constructor and the assignment operator 
      (operator=) for Spl_Quaternion, Spl_Cubic and Spl_path classes. Instead we 
      use the one provide by the compiler.
-------------------------------------------------------------------------------
*/

/*! 
  @file trajectory.cpp
  @brief Trajectory member functions.
*/

//! @brief RCS/CVS version.
static const char rcsid[] = "$Id: trajectory.cpp,v 1.1 2007/07/24 16:03:10 amaldo Exp $";

#include "trajectory.h"

#ifdef use_namespace
namespace ROBOOP {
  using namespace NEWMAT;
#endif


Spl_cubic::Spl_cubic(const Matrix & pts)
/*!
  @brief Constructor
  @param pts: Matrix containing the spline data.

  The first line of the Matrix contain the sampling time
  Second line contain data (sk) to create spline i.
  Third             "                       "    i+1.
  on Nth line                                   i+N.

  The spline has the following form:
  \f[
    s = A_k(t-t_k)^3 + B_k(t-t_k)^2 + C_k(t-t_k) + D_k
  \f]
*/
{
   int N = pts.Ncols();
   bad_data = false;
   nb_path = pts.Nrows()-1;

   if(!N)
   {
      bad_data = true;
      cerr << "Spl_cubic::Spl_cubic: size of time vector is zero." << endl;
      return;
   }
   if(N <= 3)
   {
      bad_data = true;
      cerr << "Spl_cubic::Spl_cubic: need at least 4 points to produce a cubic spline." << endl;
      return;
   }
   if(!nb_path)
   {
      bad_data = true;
      cerr << "Spl_cubic::Spl_cubic: No data for each points." << endl;
      return;
   }

   ColumnVector delta(N-1), beta(N-1);
   tk = pts.SubMatrix(1,1,1,N);           // time at sampling point

   for(int i = 1; i <= N-1; i++)
   {
      delta(i) = pts(1,i+1) - pts(1,i);  // eq 5.58d Angeles

      if(!delta(i))
      {
         bad_data = true;
         cerr << "Spl_cubic::Spl_cubic: time between input points is zero" << endl;
         return;
      }
      beta(i) = 1/delta(i);              // eq 5.58e Angeles
   }

   Matrix A(N-2, N-2); A = 0;             // eq 5.59 Angeles
   A(1,1) = 2*(delta(1)+delta(2));
   A(1,2) = delta(2);
   for(int j = 2; j <= N-2; j++)
   {
      A(j,j-1) = delta(j);
      A(j,j)   = 2*(delta(j)+delta(j+1));
      if( (j+1) <= A.Ncols())
         A(j,j+1) = delta(j+1);
   }

   Matrix C(N-2, N);   C = 0;             // eq 5.58c Angeles
   for(int k = 1; k <= N-2; k++)
   {
      C(k,k) = beta(k);
      C(k,k+1) = -(beta(k)+beta(k+1));
      C(k,k+2) = beta(k+1);
   }

   Matrix _6AiC = 6*A.i()*C;        // eq 5.58a Angeles

   ColumnVector dd_s(N);   // second spline derivative at sampling points
   dd_s(1) = dd_s(N) = 0;  // second der is 0 on first and last point of natural splines.

   Ak = Matrix(nb_path, N-1);
   Bk = Matrix(nb_path, N-1);
   Ck = Matrix(nb_path, N-1);
   Dk = Matrix(nb_path, N-1);

   for(int ii = 2; ii <= nb_path+1; ii++)
   {
      dd_s.SubMatrix(2,N-1,1,1) = _6AiC*pts.SubMatrix(ii,ii,1,N).t();

      for(int jj = 1; jj < N; jj++)
      {
         // eq 5.55a - 5.55d Angeles
         Ak(ii-1,jj) = 1/(6.0*delta(jj))*(dd_s(jj+1) - dd_s(jj));
         Bk(ii-1,jj) = 1/2.0*dd_s(jj);
         Ck(ii-1,jj) = (pts(ii,jj+1)-pts(ii,jj))/delta(jj) -
                       1/6.0*delta(jj)*(dd_s(jj+1) + 2*dd_s(jj));
         Dk(ii-1,jj) = pts(ii,jj);
      }
   }
}

short Spl_cubic::interpolating(const Real t, ColumnVector & s)
//!  @brief Interpolating the spline at time t. Extrapolating is not allowed.
{
   if(bad_data)
   {
      cerr << "Spl_cubic::interpolating: data is not good. Problems occur in constructor." << endl;
      return BAD_DATA;
   }

   for(int i = 1; i < tk.Ncols(); i++)
      if( (t >= tk(i)) && (t < tk(i+1)) )
      {
         s = Ak.SubMatrix(1,nb_path,i,i)*pow(t - tk(i), 3) +
             Bk.SubMatrix(1,nb_path,i,i)*pow(t - tk(i),2) +
             Ck.SubMatrix(1,nb_path,i,i)*(t - tk(i)) +
             Dk.SubMatrix(1,nb_path,i,i);
         return 0;
      }

   cerr << "Spl_cubic::interpolating: t is out of range." << endl;
   return NOT_IN_RANGE;
}


short Spl_cubic::first_derivative(const Real t, ColumnVector & ds)
//!  @brief Spline first derivative at time t.
{
   if(bad_data)
   {
      cerr << "Spl_cubic::first_derivative: data is not good. Problems occur in constructor." << endl;
      return BAD_DATA;
   }

   for(int i = 1; i < tk.Ncols(); i++)
      if( (t >= tk(i)) && (t < tk(i+1)) )
      {
         ds = 3*Ak.SubMatrix(1,nb_path,i,i)*pow(t - tk(i), 2) +
              2*Bk.SubMatrix(1,nb_path,i,i)*(t - tk(i)) +
              Ck.SubMatrix(1,nb_path,i,i);
         return 0;
      }

   cerr << "Spl_cubic::first_derivative: t not in range." << endl;
   return NOT_IN_RANGE;
}


short Spl_cubic::second_derivative(const Real t, ColumnVector & ds)
//!  @brief Spline second derivative at time t.
{
   if(bad_data)
   {
      cerr << "Spl_cubic::second_derivative: data is not good. Problems occur in constructor." << endl;
      return BAD_DATA;
   }

   for(int i = 1; i < tk.Ncols(); i++)
      if( (t >= tk(i)) && (t < tk(i+1)) )
      {
         ds = 6*Ak.SubMatrix(1,nb_path,i,i)*(t - tk(i)) +
              2*Bk.SubMatrix(1,nb_path,i,i);
         return 0;
      }

   cerr << "Spl_cubic::second_derivative: t not in range." << endl;
   return NOT_IN_RANGE;
}

// ----------- E N D - E F F E C T O R   P A T H   W I T H   S P L I N E S  ----------

Spl_path::Spl_path(const string & filename)
/*!
  @brief Constructor.
  @param filename: Spline data.

  Here is two examples of file. # is used as a comment, the rest of line is ignored.
  The first line should contain the string CARTESIAN_SPACE or JOINT_SPACE. The following
  number (3 in first example and 6 in the second) represents the number of points on 
  each data line. 0, 0.5 and 1.0 in example 1 represents the time.

Example 1:
\verbatim
#
#
#
CARTESIAN_SPACE
3
0
0.452100 -0.150050 -0.731800 
0.5
0.561821 -0.021244 -0.537842 
1.0
0.577857 0.268650 -0.323976 
\endverbatim

Example 2:
\verbatim
#
#
#
JOINT_SPACE
6
0
0 0 0 0 0 0
1.
0.2 -0.4 0.1 0.2 0.3 0.0
2.0
0.6 -0.8 0.2 0.4 0.4 0.0
\endverbatim
*/
{
   const char *ptr_filename = filename.c_str(); //transform string to *char

   std::ifstream inpointfile(ptr_filename, std::ios::in);
   point_map pts_map;

   if(inpointfile)
   {
      double time;
      int nb_path;
      string temp;
      pts_map.clear();

      getline(inpointfile, temp);
      while(temp.substr(0,1) == "#") { // # comment
         getline(inpointfile, temp);
      }

      if(temp == "JOINT_SPACE")
	  type = JOINT_SPACE;
      else if (temp == "CARTESIAN_SPACE")
	  type = CARTESIAN_SPACE;
      else
      {
	  cerr << "Spl_path::Spl_path: wrong selection type. Should be joint space or cartesian space." << endl;
	  return;
      }
	      
      getline(inpointfile, temp);
      istringstream inputString1(temp);
      inputString1 >> nb_path;
      if(nb_path < 1)
      {
	  cerr << "Spl_path::Spl_path: number of splines should be >= 1." << endl;
	  return;
      }
      ColumnVector p(nb_path);


      while( !inpointfile.eof() )
      {
         getline(inpointfile, temp);
         istringstream inputString2(temp);

         if(temp.substr(0,1) == " ")
            break;
         inputString2 >> time;
	 final_time = time;

         getline(inpointfile, temp);
         istringstream inputString3(temp);

         for(int i = 1; i <= nb_path; i++)
            inputString3 >> p(i);

         pts_map.insert(point_map::value_type(time, p));
      }
   }
   else
      cerr << "Spl_path::Spl_path: can not open file " << filename.c_str() << endl;

   // Spl_cubic class take as input a Matrix that contain the data.
   int nb_pts, nb_path;
   nb_pts = pts_map.size();
   point_map::const_iterator iter = pts_map.begin();
   nb_path = iter->second.Nrows();

   Matrix pts(nb_path+1, nb_pts); pts = 0;
   {
      int i = 1;
      for(iter = pts_map.begin(); iter != pts_map.end(); ++iter)
      {
         pts(1,i) = iter->first;
         pts.SubMatrix(2, nb_path+1, i, i) = iter->second;
         i++;
      }
   }
   Spl_cubic::operator=(Spl_cubic(pts));
}


/*!
  @fn Spl_path::Spl_path(const Matrix & pts)
  @brief Constructor.
*/
Spl_path::Spl_path(const Matrix & pts): Spl_cubic(pts)
{
}

short Spl_path::p(const Real t, ColumnVector & p)
//!  @brief Position vector at time t.
{
   if(interpolating(t, p))
   {
      cerr << "Spl_path::p_pdot: problem with spline interpolating." << endl;
      return -4;
   }

   return 0;
}

short Spl_path::p_pdot(const Real t, ColumnVector & p, ColumnVector & pdot)
//!  @brief Position and velocity vector at time t.
{
   if(interpolating(t, p))
   {
      cerr << "Spl_path::p_pdot: problem with spline interpolating." << endl;
      return -4;
   }
   if(first_derivative(t, pdot))
   {
      cerr << "Spl_path::p_pdot: problem with spline first_derivative." << endl;
      return -4;
   }

   return 0;
}

short Spl_path::p_pdot_pddot(const Real t, ColumnVector & p, ColumnVector & pdot,
                             ColumnVector & pdotdot)
//! @brief Position, velocity and acceleration vector at time t.
{
   if(interpolating(t, p))
   {
      cerr << "Spl_path::p_pdot_pdotdot: problem with spline interpolating." << endl;
      return -4;
   }
   if(first_derivative(t, pdot))
   {
      cerr << "Spl_path::p_pdot_pdotdot: problem with spline first_derivative." << endl;
      return -4;
   }
   if(second_derivative(t, pdotdot))
   {
      cerr << "Spl_path::p_pdot_pdotdot: problem with spline first_derivative." << endl;
      return -4;
   }
   return 0;
}

// -------------------- Q U A T E R N I O N  S P L I N E S -------------------


Spl_Quaternion::Spl_Quaternion(const string & filename)
/*!
  @brief Constructor.
  @param filename: Quaternions spline data.

Here is an exemple of file. # is used as comment, the rest of the line is
ignored. 0, 0.5 and 1.0 are the time at the control points. R indicated that the 
quaternion at the control point is obtained from a rotation matrix, and the data 
(9 elements of the matrix) is on the next line. Instead of R one could use Q to 
indicate a quaternion and the following line will contain 4 elements.

\verbatim
#
#
#
0
R
1.000000 0.000000 0.000000 0.000000 -1.000000 0.000000 0.000000 -0.000000 -1.000000
0.5
R
0.999943 0.008696 -0.006149 0.009042 -0.998237 0.058659 -0.005628 -0.058711 -0.998259
1.0
R
0.961882 0.254807 0.099282 0.223474 -0.941661 0.251662 0.157616 -0.219882 -0.962709
\endverbatim
*/
{
   const char *ptr_filename = filename.c_str(); //transform string to *char

   std::ifstream inquatfile(ptr_filename, std::ios::in);

   if(inquatfile)
   {
      double time;
      string temp;
      Matrix R(3,3);
      Quaternion q;
      quat_data.clear();

      while( !inquatfile.eof() )
      {
         getline(inquatfile, temp);
         while(temp.substr(0,1) == "#") {
            getline(inquatfile, temp);
         }
         istringstream inputString(temp);
         if(temp.substr(0,1) == " ")
            break;
         inputString >> time;

         getline(inquatfile, temp);
         if( (temp.substr(0,1) == "r") || (temp.substr(0,1) == "R") )
         {
            getline(inquatfile, temp);
            istringstream inputString(temp);
            inputString >> R(1,1) >> R(1,2) >> R(1,3) >> R(2,1) >> R(2,2) >>
            R(2,3) >> R(3,1) >> R(3,2) >> R(3,3);
            q = Quaternion(R);
         }
         else if( (temp.substr(0,1) == "q") || (temp.substr(0,1) == "Q") )
         {
            getline(inquatfile, temp);
            istringstream inputString(temp);
            inputString >> R(1,1) >> R(1,2) >> R(1,3) >> R(2,1);
            q = Quaternion(R(1,1), R(1,2), R(1,3), R(2,1));
         }
         else if(temp.substr(0,1) == "")
         {
         }
         else
         {
            cerr << "Spl_Quaternion::Spl_Quaternion: format of input file "
            << filename.c_str() << " is incorrect" << endl;
         }

         quat_data.insert( quat_map::value_type(time, q));
      }
   }
   else
   {
      cerr << "Spl_Quaternion::Spl_Quaternion: can not open file "
      << filename.c_str() << endl;
   }
}


Spl_Quaternion::Spl_Quaternion(const quat_map & quat)
//!  @brief Constructor.
{
   quat_data = quat;
}

short Spl_Quaternion::quat(const Real t, Quaternion & s)
/*!
  @brief Quaternion interpollation.

  \f[
    S_n(t) = Squad(q_n,a_n, a(n+1),q(n+1),t)
  \f]
*/
{
   Real dt=0;
   Quaternion ds;
   quat_map::const_iterator iter1 = quat_data.begin(), iter2;

   if( t == iter1->first)
   {
      s = iter1->second;
      return 0;
   }

   // Point to interpolate is between the first and the second point. Use Slerp function
   // since there is not enough points for Squad. Use dt since Slerp and Squad functions
   // need t from 0 to 1.
   iter2 = iter1;
   iter2++;
   if( t < iter2->first)
   {
      dt = (t - iter1->first)/(iter2->first - iter1->first);
      s  = Slerp(iter1->second, iter2->second, dt);
      return 0;
   }


   // From second element to element N-2.
   //    for(iter1 = ++quat_data.begin(); iter1 != ----quat_data.end(); ++iter1)
   //    for(iter1 = ++iter1; iter1 != ----quat_data.end(); ++iter1)
   for(iter1 = iter2; iter1 != ----quat_data.end(); ++iter1)
   {
      iter2=iter1;
      iter2++;
      if( (t >= iter1->first) && (t < iter2->first) )
      {
         dt = (t - iter1->first)/(iter2->first - iter1->first);

         Quaternion qn_  = (--iter1)->second,  // q(n-1)
                           qn   = (++iter1)->second,  // q(n)
                                  qn_1 = (++iter1)->second,  // q(n+1)
                                         qn_2 = (++iter1)->second,  // q(n+2)
                                                q_tmp;
         ----iter1;

         // Intermediate point a(n) and a(n+1)
         Quaternion an   = qn*(((qn.i()*qn_1).Log() + (qn.i()*qn_).Log())/-4).exp(),
                           an_1 = qn_1*(((qn_1.i()*qn_2).Log() + (qn_1.i()*qn).Log())/-4).exp();

         s  = Squad(qn, an, an_1, qn_1, dt);
         return 0;
      }
   }

   // Interpolation between last two points.
   iter2 = iter1; iter2++;
   if( (t >= iter1->first) && (t <= iter2->first) )
   {
      dt = (t - iter1->first)/(iter2->first - iter1->first);
      s = Slerp(iter1->second, iter2->second, dt);
      return 0;
   }

   cerr << "Spl_Quaternion::quat_w: t not in range." << endl;
   return NOT_IN_RANGE;
}

short Spl_Quaternion::quat_w(const Real t, Quaternion & s, ColumnVector & w)
//!  @brief Quaternion interpollation and angular velocity.
{
   Real dt=0;
   Quaternion ds;
   quat_map::const_iterator iter1 = quat_data.begin(), iter2;

   if( t == iter1->first)
   {
      s = iter1->second;
      w = ColumnVector(3); w = 0.0;
      return 0;
   }

   // Point to interpolate is between the first and the second point. Use Slerp function
   // since there is not enough points for Squad. Use dt since Slerp and Squad functions
   // need t from 0 to 1.
   iter2 = iter1;
   iter2++;
   if( t < iter2->first)
   {
      dt = (t - iter1->first)/(iter2->first - iter1->first);
      s  = Slerp(iter1->second, iter2->second, dt);
      ds = Slerp_prime(iter1->second, iter2->second, dt);
      w = Omega(s, ds);

      return 0;
   }

   // From second element to element N-2.
   //    for(iter1 = ++quat_data.begin(); iter1 != ----quat_data.end(); ++iter1)
   //    for(iter1 = ++iter1; iter1 != ----quat_data.end(); ++iter1)
   for(iter1 = iter2; iter1 != ----quat_data.end(); ++iter1)
   {
      iter2=iter1;
      iter2++;
      if( (t >= iter1->first) && (t < iter2->first) )
      {
         dt = (t - iter1->first)/(iter2->first - iter1->first);

         Quaternion qn_  = (--iter1)->second,  // q(n-1)
                           qn   = (++iter1)->second,  // q(n)
                                  qn_1 = (++iter1)->second,  // q(n+1)
                                         qn_2 = (++iter1)->second,  // q(n+2)
                                                q_tmp;
         ----iter1;

         // Intermediate point a(n) and a(n+1)
         Quaternion an   = qn*(((qn.i()*qn_1).Log() + (qn.i()*qn_).Log())/-4).exp(),
                           an_1 = qn_1*(((qn_1.i()*qn_2).Log() + (qn_1.i()*qn).Log())/-4).exp();

         s  = Squad(qn, an, an_1, qn_1, dt);
         ds = Squad_prime(qn, an, an_1, qn_1, dt);
         w = Omega(s, ds);
         return 0;
      }
   }

   // Interpolation between last two points.
   iter2 = iter1; iter2++;
   if( (t >= iter1->first) && (t <= iter2->first) )
   {
      dt = (t - iter1->first)/(iter2->first - iter1->first);
      s = Slerp(iter1->second, iter2->second, dt);
      ds = Slerp_prime(iter1->second, iter2->second, dt);
      w = Omega(s, ds);
      return 0;
   }

   cerr << "Spl_Quaternion::quat_w: t not in range." << endl;
   return NOT_IN_RANGE;
}


// -----------------------------------------------------------------------------------------------


Trajectory_Select::Trajectory_Select()
//!  @brief Constructor.
{
    type = NONE;
    quaternion_active = false;
}


Trajectory_Select::Trajectory_Select(const string & filename)
/*!
  @brief Constructor.
  @param filename: File containing spline data. 

  The file can be a quaternion file (Spl_Quaternion) or path 
  file (Spl_path).
*/
{
    set_trajectory(filename);
}


Trajectory_Select & Trajectory_Select::operator=(const Trajectory_Select & x)
//!  @brief Overload = operator.
{
    type = x.type;
    if(x.quaternion_active)
    {
	quaternion_active = x.quaternion_active;
	path_quat = x.path_quat;
    }
    else
	path = x.path;

    return *this;
}


void Trajectory_Select::set_trajectory(const string & filename)
//!  @brief Trajectory selection.
{
   const char *ptr_filename = filename.c_str(); //transform string to *char

   std::ifstream inpointfile(ptr_filename, std::ios::in);

   if(inpointfile)
   {
      string temp;

      getline(inpointfile, temp);
      while(temp.substr(0,1) == "#") { // # comment
         getline(inpointfile, temp);
      }

      if( (temp == "JOINT_SPACE") || (temp == "CARTESIAN_SPACE") )
      {  
	  path = Spl_path(filename);
	  type = path.get_type();
	  quaternion_active = false;
      }
      else
      {
	  path_quat = Spl_Quaternion(filename);
	  type = CARTESIAN_SPACE;
	  quaternion_active = true;
      }
   }
   else
       cerr << "Trajectory_Select::set_trajectory: invalid input file " << filename << endl;
}

#ifdef use_namespace
}
#endif







