// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-    
/*
\author Shiraj Sen and Stephen Hart

Copyright (C) 2010 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/
#include <NonParametricProbabilityDistribution.h>

#include <iostream>

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <math.h>
#include <stdio.h>

using namespace std;
using namespace yarp::math;
using namespace yarp::sig;

NonParametricProbabilityDistribution::NonParametricProbabilityDistribution(int count) 
    : ProbabilityDistribution(count),
      range_min(count),
      range_max(count),
      no_of_bins(count),
      bin_size(count),
      covar(count,count),
      covar_inv(count,count)
 { 
        
    dim_count = count;
    data_size = 0;
    sorted = 0;
    first = 1;
    _total_sample_count=0;
    _needsCleaning=false;
    max_val = 0;
    _count=0;
    
 }

NonParametricProbabilityDistribution::~NonParametricProbabilityDistribution() {
    
    if(_needsCleaning) {
        for(int i=0; i<data_size; i++) {
            distribution[i].clear();
            sorted_distribution[i].clear();
            cumulative_distribution[i].clear();
        }
        distribution.clear();
        sorted_distribution.clear();
        cumulative_distribution.clear();
        _needsCleaning = false;
        } 
    
    }


//Function to draw a sample from the distribution
int NonParametricProbabilityDistribution::drawSample(Vector &sample) {
    Vector sample_index(dim_count);
    if(sorted == 0)
        sort();
    getNewSample(sample_index);
    
    //Take a sample from the centre of the bin
    for(int i=0; i<dim_count; i++) {
        sample[i] = range_min[i] + bin_size[i]/2.0 + sample_index[i]*bin_size[i];
    }
    sorted = 1;
}

//Function to get the probability of a sample
double NonParametricProbabilityDistribution::getProbability(Vector sample) {
        
    int point[dim_count];
    int bin;
    double p;
    
        // find the bin the sample is in   
    for(int i=0; i<dim_count; i++)
        point[i] = (int)((sample[i]-range_min[i])/((range_max[i]-range_min[i])/no_of_bins[i]));
    bin=0;
    int i;
    for(i=0; i<dim_count-1; i++)
        bin += (int)(point[i]*pow(no_of_bins[i],dim_count-1-i));
    bin += point[i];
        
    p=distribution[bin].val/_total_sample_count;
    
    if(isinf(p)==1) {
        cout << "NonParametricProbabilityDistribution::getProbability() p=" << p << endl;
        p=0.999999;
    }
    if(isinf(p)==-1) {
        cout << "NonParametricProbabilityDistribution::getProbability() p=" << p << endl;
        p=0.999999;
    }
    if(isnan(fabs(p))) {
        cout << "NonParametricProbabilityDistribution::getProbability() p=" << p << endl;
        p=0.000001;
    }
    return p;
}

void NonParametricProbabilityDistribution::sort() {
    double max, temp, tempmax, cumulative;
    int maxindex;
    int tempindex;
    double sum=0;
    cumulative=0;
        
    for(int i=0; i<data_size; i++) {
        sorted_distribution[i].val = distribution[i].val;
        sum += sorted_distribution[i].val;
        for(int j=0; j<dim_count; j++)
            sorted_distribution[i].index[j] = distribution[i].index[j];
    }
    for(int i=0; i<data_size; i++)
        sorted_distribution[i].val = sorted_distribution[i].val / sum;
    
    int i;
    
    for(i=0; i<data_size-1; i++) {
        max = sorted_distribution[i].val;
        maxindex = i;
        for(int j=i+1; j<data_size; j++) {
            tempmax = max;
            max = (max > sorted_distribution[j].val) ? max : sorted_distribution[j].val;
            if(max != tempmax)
                maxindex = j;
        }
        if(maxindex != i) {
            temp = sorted_distribution[i].val;
            sorted_distribution[i].val = sorted_distribution[maxindex].val;
            sorted_distribution[maxindex].val = temp;
            for(int k=0; k<dim_count; k++) {
                tempindex = sorted_distribution[i].index[k];
                sorted_distribution[i].index[k] = sorted_distribution[maxindex].index[k];
                sorted_distribution[maxindex].index[k] = tempindex;
            }
        }
        
        cumulative += max;
        cumulative_distribution[i].val = cumulative;
        for(int k=0; k<dim_count; k++)
            cumulative_distribution[i].index[k] = sorted_distribution[i].index[k];
    }
    
    cumulative_distribution[i].val = 1.0;
    for(int k=0; k<dim_count; k++)
        cumulative_distribution[i].index[k] =  sorted_distribution[i].index[k];
    
}

//Helper function to get a new sample from the distribution
void NonParametricProbabilityDistribution::getNewSample(Vector &sample) {
    double coinFlip;
    time_t now;
    
    time(&now);
    srand((unsigned) now);
    coinFlip = (double)(rand()/(RAND_MAX + 1.0));
    
    for(int i=0; i<data_size-1; i++) {
        if((i==0 && coinFlip <= cumulative_distribution[0].val) || (i>0 && (coinFlip > cumulative_distribution[i-1].val) && (coinFlip <= cumulative_distribution[i].val))) {
            for(int j=0; j<dim_count; j++)
                sample[j] = cumulative_distribution[i].index[j];
        }
    }
    
}

//Function to add a sample to the distribution
int NonParametricProbabilityDistribution::addSample(Vector sample, double val) {
    int point[dim_count];
    int bin;
        
    Vector smoothing_sample(dim_count);
    double v;
    
    /*        
    // this if for just adding the single point to the histogram
    for(int i=0; i<dim_count; i++)
    printf("sample[%d]: %f\n", i, sample[i]);   
    
    for(int i=0; i<dim_count; i++)
    point[i] = (int)((sample[i]-range_min[i])/((range_max[i]-range_min[i])/no_of_bins[i]));
    
    bin=0;
    int i;
    for(i=0; i<dim_count-1; i++)
    bin += (int)(point[i]*pow(no_of_bins[i],dim_count-1-i));
    bin += point[i];
    
    distribution[bin].val += val;
    _total_sample_count += val;
    
    */
    sorted = 0;
    
    // this is for smoothing the point with a gaussian first
    for(int b=0; b<data_size; b++) {     
        getBinCenter(b, smoothing_sample);
        v = val*computeGaussianProbability(sample, smoothing_sample);            
        //            printf("%f\n", v);
        if(v > THRESHOLD) {
            distribution[b].val += v;
            _total_sample_count += v;
            max_val = max_val > distribution[b].val ? max_val : distribution[b].val;
        }
    }
    
    _count++;
    if((_count%NORMALIZE_RATE)==0) {            
        normalizeAndRescale(NORMALIZE_RATE);
        _count=0;
    }
    
    return 1;
}

int NonParametricProbabilityDistribution::getDimensionCount() {
    return(dim_count);
}

//Function to set the dimension parameters
//min : Min val of the dimension
//max : Max val of the dimension
//bins: No of bins for the dimension
void NonParametricProbabilityDistribution::setDimensionParameters(Vector min, Vector max, Vector bins) {
    
    data_size=1;
    for(int i=0; i<dim_count; i++) {
        range_min[i] = min[i];
        range_max[i] = max[i];
        no_of_bins[i] = bins[i];
        bin_size[i] = (max[i] - min[i])/bins[i];
        data_size *= (int)(no_of_bins[i]);
    }
    computeCovariance();
        
    distribution.resize(data_size);
    for(int i=0; i<data_size; i++)
        distribution[i].index.resize(dim_count);
    
    //Set the initial distribution to be uniform
    for(int i=0; i<data_size; i++)
        distribution[i].val = 1.0;
    
    _total_sample_count=data_size;
    
    sorted_distribution.resize(data_size);
    cumulative_distribution.resize(data_size);
    for(int i=0; i<data_size; i++) {
        sorted_distribution[i].index.resize(dim_count);
        cumulative_distribution[i].index.resize(dim_count);
    }
    
    _needsCleaning = true;
    setIndexParameters();
}
    
void NonParametricProbabilityDistribution::setIndexParameters() { 
   int index[dim_count];
    for(int i=0; i<data_size; i++) {
        setIndex(i,index);
        getIndex(i,index);
        for(int j=0; j<dim_count; j++)
            distribution[i].index[j] = index[j];
    }
}

void NonParametricProbabilityDistribution::setIndex(int pt, int *index) {
    if(pt == 0) {
        for(int i=0; i<dim_count; i++)
            index[i] = 0;
    }
    else
        index[dim_count-1] += 1;
}

void NonParametricProbabilityDistribution::getIndex(int pt, int *index) {
    for(int i=dim_count-1; i>=0; i--) {
        if(index[i] < no_of_bins[i])
            i=0;
        else {
            index[i] = 0;
            index[i-1]++;
        }
    }
}

//Returns the center value of a bin
void NonParametricProbabilityDistribution::getBinCenter(int pt, Vector &center) {
    for(int i=0; i<dim_count; i++) {
        center[i] = distribution[pt].index[i] * bin_size[i] + bin_size[i]/2.0 + range_min[i];
    }
}

//Function to load the distribution from a file
int NonParametricProbabilityDistribution::loadDistribution(string str) {
    
    cout << "loading distribution..." << endl;
    
    FILE *fid;
    int dims;
    double v;
    
    _total_sample_count=0;
    
    if( !(fid = fopen(str.c_str(),"r")) ) {
        cout << "NonParametricProbabilityDistribution::loadDistribution() -- can't open file=\'" << str.c_str() << "\'!!" << endl;
        return 0;
    }
    
    fscanf(fid,"%d\n",&dims);
    
    dim_count = dims;    
    data_size=1;
    for(int i=0; i<dim_count; i++) {
        fscanf(fid,"%lf\t%lf\t%lf\n",&range_min[i],&range_max[i],&no_of_bins[i]);
        data_size *= (int)(no_of_bins[i]);
        bin_size[i] = (range_max[i]-range_min[i])/no_of_bins[i];
    }
    computeCovariance();
    
    distribution.resize(data_size);
    for(int i=0; i<data_size; i++)
        distribution[i].index.resize(dim_count);
    setIndexParameters();
    
    for(int i=0; i<data_size; i++) {
        fscanf(fid,"%lf\t",&v);
        distribution[i].val = v;
        _total_sample_count += v;
        max_val = max_val > distribution[i].val ? max_val : distribution[i].val;
    }
    
    fclose(fid);
    
    sorted_distribution.resize(data_size);
    cumulative_distribution.resize(data_size);
    for(int i=0; i<data_size; i++) {
        sorted_distribution[i].index.resize(dim_count);
        cumulative_distribution[i].index.resize(dim_count);
    }
    
    _needsCleaning = true;
    return 1;
}
    
   
//Function to save the distribution to a file
int NonParametricProbabilityDistribution::saveDistribution(string str) {
    FILE *fid;
        
    cout << "saving distribution[size=" << data_size << "]: " << str.c_str() << endl;
    if( (fid = fopen(str.c_str(),"w"))==NULL) {
        cout << "NonParametricProbabilityDistribution::saveDistribution() -- can't open file=\'" << str.c_str() << "\'!!" << endl;
        return 0;
    }
    
    fprintf(fid,"%d\n",dim_count);
    
    for(int i=0; i<dim_count; i++)
        fprintf(fid,"%lf\t%lf\t%lf\n",range_min[i],range_max[i],no_of_bins[i]);
    
    for(int i=0; i<data_size; i++)
        fprintf(fid,"%lf\t",distribution[i].val);
    
        fclose(fid);
        return 1;
}
    
void NonParametricProbabilityDistribution::computeCovariance() {    
    for(int i=0; i<dim_count; i++) {
        covar[i][i] = 0.15*((range_max[i]-range_min[i])/2.0);      
    }
    covar_inv = pinv(covar);
    covar_det = det(covar);
    alpha = 1.0/(pow(2.0*M_PI,dim_count/2.0)*sqrt(covar_det));
}

double NonParametricProbabilityDistribution::computeGaussianProbability(Vector mean, Vector sample) {

    Vector diff(dim_count);
    Matrix MdiffT(1,dim_count);
    Vector s(dim_count);
    Vector m(dim_count);
    
    Matrix tmp(1,dim_count);
    Vector Vp(1);
    
    double p=0;
    double e;
    
    for(int i=0; i<dim_count; i++) {
        s[i] = sample[i];
        m[i] = mean[i];
    }
    
    diff = s - m;
    
    //printf("s: [%f %f], m: [%f %f], diff: [%f %f]\n", s[0], s[1], m[0], m[1], diff[0], diff[1]);
    for(int i=0; i<dim_count; i++)
        MdiffT[0][i] = diff[i];
    
    tmp = MdiffT * covar_inv;
    Vp = tmp * diff;
    e = exp(-0.5*Vp[0]);
    //    p = alpha*e;
    p = e; // use normalized
    
    /*
      printf("p=%.10f, alpha=%.10f, e=%.10f, det=%f, p2=%.10f\n", p, alpha, e, covar_det, p/alpha);
      
      printf("covar:\n");
      for(int i=0; i<covar.rows(); i++) {
      for(int j=0; j<covar.cols(); j++) {
      printf("%.10f ", covar[i][j]);
      }
      printf("\n");
      }
    */
    //        printf("diff:\n");
    //diff.display();
    
    //usleep(1000);
        
    return p;
    
}

void NonParametricProbabilityDistribution::normalizeAndRescale(int scale = 5) {
    double scale_factor;
    scale_factor = max_val / (double)scale;
    printf("NORMALIZE AND RESCALE: scale_factor=%f\n", scale_factor);
    sleep(10);
    if(scale_factor==0) {
        //printf("NORMALIZE AND RESCALE: scale_factor=%f\n", scale_factor);
        sleep(10);
    }
    for(int i=0; i<data_size; i++) {
        distribution[i].val /= scale_factor;
    }
    _total_sample_count /= scale_factor;
    max_val /= scale_factor;    
}
    

