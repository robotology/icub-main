#include "iCub/skinDynLib/parzenWindowEstimator.h"

double gauss(const double x_0, const double sigma, const double val)
{
    double res = (1/(sqrt(2*M_PI)*sigma))*
                 exp(-0.5*( ((val - x_0)*(val - x_0))/(sigma*sigma) ));

    //           (1/(sqrt(2*pi  )*sigma))*exp(-0.5*((x(i)-x_0)/sigma)^2);
    return res;
}

double gauss2D(const double x_0, const double y_0,
               const double sigmaX, const double sigmaY,
               const double valX, const double valY)
{
    double res = (1/(sqrt(2*M_PI*sigmaX*sigmaY)))*
                 exp(-0.5*( ((valX - x_0)*(valX - x_0))/(sigmaX*sigmaX) +
                            ((valY - y_0)*(valY - y_0))/(sigmaY*sigmaY)   ));
    return res;
}

/****************************************************************/
/* PARZEN WINDOW ESTIMATOR
*****************************************************************/
    bool parzenWindowEstimator::resize(const Matrix _ext, std::vector<int> _binsNum)
    {
        if (_binsNum.size()==1 && dim==1)
        {
            _binsNum.push_back(1);
        }

        if (_ext.rows()!=dim || _ext.cols()!=2 || _binsNum.size()!=2)
        {
            yError("Resize failed! dim %i _ext size: %i %i _binsNum size: %lu\n",dim,_ext.rows(),_ext.cols(), _binsNum.size());
            return false;
        }

        ext     = _ext;
        binsNum = _binsNum;

        binWidth.clear();
        firstPosBin.clear();
        firstPosBinShift.clear();

        for (size_t i = 0; i < dim; i++)
        {
            binWidth.push_back((ext(i,1)-ext(i,0))/binsNum[i]);

            // Let's find the first bin for which we have positive values.
            // The 0 value should not be inside it
            // and its shift from zero to the start value of the firstPosBin
            double binStart=ext(0,0);
            int idx=0;

            while (binStart<0)
            {
                idx++;
                binStart+=binWidth[i];
            }
            firstPosBin.push_back(idx);
            firstPosBinShift.push_back(binStart);
                                    
            sigm.push_back(binWidth[i]);
        }

        posHist.resize(binsNum[0],binsNum[1]); posHist.zero();
        negHist.resize(binsNum[0],binsNum[1]); negHist.zero();

        return true;
    }

    yarp::sig::Matrix parzenWindowEstimator::getHist()
    {
        yarp::sig::Matrix Hist(binsNum[0],binsNum[1]);
        Hist.zero();

        for (size_t i = 0; i < binsNum[0]; i++)
        {
            for (size_t j = 0; j < binsNum[1]; j++)
            {
                Hist(i,j)=getHist(i,j);
            }
        }

        return Hist;
    }

    double parzenWindowEstimator::getHist(const int i, const int j)
    {
        if ( posHist(i,j)+negHist(i,j) < 5 )
            return 0;

        return posHist(i,j)/(posHist(i,j)+negHist(i,j));
    }

    bool parzenWindowEstimator::getIndexes(const std::vector<double> x, std::vector<int> &b)
    {
        b.clear();

        for (size_t i = 0; i < dim; i++)
        {
            if (x[i] < ext(i,0) || x[i] > ext(i,1)) return false;

            b.push_back(int((x[i]-firstPosBinShift[i])/binWidth[i]+firstPosBin[i]));
        }

        if (dim==1)
        {
            b.push_back(0);
        }

        return true;
    }

    bool parzenWindowEstimator::addSample(const std::vector<double> x)
    {
        std::vector<int> b;
        
        if (getIndexes(x,b))
        {
            printf("adding sample %i %i from %g %g\n",b[0],b[1],x[0],x[1]);
            posHist(b[0],b[1]) += 1;
            return true;
        }
        else
            return false;
    }

    bool parzenWindowEstimator::removeSample(const std::vector<double> x)
    {
        std::vector<int> b;
        
        if (getIndexes(x,b))
        {
            negHist(b[0],b[1]) += 1;
            return true;
        }
        else
            return false;
    }

    int parzenWindowEstimator::computeResponse(const std::vector<double> x)
    {
        std::vector<int> b;

        if (getIndexes(x,b))
        {
            return int(getF_X_scaled(x));
        }

        return 0;
    }

    void parzenWindowEstimator::print()
    {
        if (dim==1)
        {
            yInfo("Extension (X1 X2): %g %g",ext[0,0],ext[0,1]);
            yInfo("binsNum(X) %i\nbinWidth(X) %g",binsNum[0],binWidth[0]);
            yInfo("sigma(X) %g",sigm[0]);
        }
        else if (dim==2)
        {
            yInfo("Extension (X1 X2 Y1 Y2): %g %g %g %g",ext[0,0],ext[0,1],ext[1,0],ext[1,1]);
            yInfo("binsNum(X Y) %i %i\nbinWidth(X Y) %g %g",binsNum[0],binsNum[1],binWidth[0],binWidth[1]);
            yInfo("sigma(X Y) %g %g\n",sigm[0],sigm[1]);
        }

        yInfo("Positive histogram:\n");
        yInfo("%s\n",posHist.toString().c_str());
        yInfo("Negative histogram:\n");
        yInfo("%s\n",negHist.toString().c_str());
    }

/****************************************************************/
/* PARZEN WINDOW ESTIMATOR 1D
*****************************************************************/

    parzenWindowEstimator1D::parzenWindowEstimator1D()
    {
        dim = 1;

        Matrix eX(1,2);
        eX(0,0) = -0.1;        eX(0,1) =  0.2;

        std::vector<int> bN; bN.push_back(20);

        resize(eX,bN);
    }

    parzenWindowEstimator1D::parzenWindowEstimator1D(const Matrix _ext, const std::vector<int> _binsNum)
    {
        dim = 1;
        resize(_ext,_binsNum);
    }

    double parzenWindowEstimator1D::getF_X(const std::vector<double> x)
    {
        double f_x = 0;

        for (size_t i = 0; i < posHist.rows(); i++)
        {
            if ( posHist(i,0)>=0 )
            {
                double factor=(posHist(i,0)+negHist(i,0))>0?posHist(i,0)/(posHist(i,0)+negHist(i,0)):0;
                f_x += factor * gauss(ext(0,0)+i*binWidth[0],sigm[0],x[0]);
            }
        }

        return f_x;
    }

    double parzenWindowEstimator1D::getF_X_scaled(const std::vector<double> x)
    {
        double max      =  0;
        int granularity = 10;        

        // the granularity has been introduced to increase the precision of the process.
        for (size_t i = 0; i < binsNum[0]*granularity; i++)
        {
            std::vector<double> xx;
            xx.push_back(ext(0,0)+i*binWidth[0]*granularity);
            double func = getF_X(xx);

            max  = func>max?func:max;
        }

        double scalingfactor_max = 100/max;

        // if the histogram is empty, the maximum value is 0
        if (max == 0)
        {
            return max;
        }

        return getF_X(x)*scalingfactor_max;
    }

/****************************************************************/
/* PARZEN WINDOW ESTIMATOR 2D
*****************************************************************/

    parzenWindowEstimator2D::parzenWindowEstimator2D()
    {
        dim = 2;

        Matrix eXY(2,2);
        eXY(0,0) = -0.1;        eXY(0,1) =  0.2;
        eXY(1,0) =  0.0;        eXY(1,1) =  1.2;

        std::vector<int>    bN; bN.push_back(8);    bN.push_back(4);

        resize(eXY,bN);
    }

    parzenWindowEstimator2D::parzenWindowEstimator2D(const Matrix _ext, const std::vector<int> _binsNum)
    {
        dim = 2;
        resize(_ext,_binsNum);
    }

    double parzenWindowEstimator2D::getF_X(const std::vector<double> x)
    {
        double f_x = 0;

        for (size_t i = 0; i < posHist.rows(); i++)
        {
            for (size_t j = 0; j < posHist.cols(); j++)
            {
                if ( posHist(i,j)>=0 )
                {
                    double factor=(posHist(i,j)+negHist(i,j))>0?posHist(i,j)/(posHist(i,j)+negHist(i,j)):0;
                    f_x += factor * gauss2D(ext(0,0)+i*binWidth[0],ext(1,0)+j*binWidth[1],sigm[0],sigm[1],x[0],x[1]);
                }
            }
        }

        return f_x;
    }
    
    double parzenWindowEstimator2D::getF_X_scaled(const std::vector<double> x)
    {
        double max      = 0;
        int granularity = 2;
        // double sum  =   0;
        // int cnt     = 0;

        // the granularity has been introduced to increase the precision of the process.
        for (size_t i = 0; i < binsNum[0]*granularity; i++)
        {
            for (size_t j = 0; j < binsNum[1]*granularity; j++)
            {
                std::vector<double> xx;
                xx.push_back(ext(0,0)+i*binWidth[0]*granularity);
                xx.push_back(ext(1,0)+j*binWidth[1]*granularity);
                double func = getF_X(xx);

                max  = func>max?func:max;
                // sum += func;
                // cnt++;
            }
        }
        // double avg = sum/cnt;
        // double scalingfactor_avg = 100/avg;

        double scalingfactor_max = 255/max;

        return getF_X(x)*scalingfactor_max;
    }
        
    // pp = pp + (1/(2*M_PI*sigm))*exp(-0.5*(xxb -px(b)).^2/(sigm^2));

