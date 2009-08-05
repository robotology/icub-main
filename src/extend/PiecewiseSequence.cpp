// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2006, 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


#include "PiecewiseSequence.h"
#include "Unit.h"
#include "Sequence.h"


#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>

#include <map>


using namespace std;
using namespace boost;

#define DBG if (1) 
//#define DBG if (0) 

#define DBG2 if (1) 

double mmax(double x, double y) {
    return (x>y)?x:y;
}

void PiecewiseSequence::apply(const IUnit& unit) {
    trimLeft = 0;
    trimRight = 0;
    bool trimming = false;

    DBG cerr << "*********************************************************" << 
        endl;
    DBG cerr << "*** for " << unit.toString().substr(0,20) << " ..." << endl;
    DBG cerr << endl;

    const IUnit& past = unit; //rename

    // check the places of decimal, for cosmetic reasons
    scale = 1;
    places = 1;
    for (int i=1; i<past.size(); i++) {
        double v = past[i];
        for (int k=places; k<=6; k++) {
            if (fabs(v-(long int)(v*scale))<0.0001) {
                break;
            }
            scale *= 10;
            places++;
        }
    }  

    // ok, what kind of numbers do we have?
    bool more = true;
    bool useTh = false;
    double th = 0;
    Histogram deltas;
    Histogram mags;
    bool discrete = true;
    for (int i=1; i<past.size(); i++) {
        if (fabs((fabs(past[i])-(int)fabs(past[i])))>0.001) {
            discrete = false;
        }
    }
    while (more) {
        int adds = 0;
        deltas.clear();
        mags.clear();
        more = false;
        int len = 0;
        for (int i=1; i<past.size(); i++) {
            double v = past[i];
            double pv = past[i-1];
            //DBG cerr << past[i] << endl;
            double dv = fabs(v-pv);
            if (discrete) {
                // discrete case, if special
                dv = 1;
            }
            bool add = true;
            if (useTh) {
                add = (dv<=th)&&(dv>=0.000001);
            }
            if (add) {
                adds++;
                deltas.add(dv);
            }
            mags.add(v);
        }
        DBG cerr << "Gap width is " << deltas.getGapWidth() << endl;
        DBG cerr << "adds: " << adds << endl;
        double minFraction = 50;
        minFraction -= past.size(); // scaled for percentage
        if (minFraction<30) {
            minFraction = 30;
        }
        DBG cerr << "Min width is " << minFraction << endl;
      
        int index = deltas.getGapCenterIndex();
        double v = deltas.getValue(index);
        DBG cerr << "Gap center is " << index << " (" << v << ")" << endl;
        DBG cerr << "Estimate gap is " << v << endl;
        if (deltas.getGapWidth()>minFraction) {
            th = v;
            useTh = true;
            more = true;
        }
    }

    if (unit.size()<6) {
        DBG cerr << "*** Too short, using fixed threshold of 0.5" << endl;
        useTh = true;
        th = 0.5;
    } 



    if (useTh) {
        DBG cerr << "*** Looks roughly piece-wise continuous" << endl;
        DBG cerr << "*** Gap threshold " << th << endl;
    } else {
        DBG cerr << "*** Does not look piece-wise continuous" << endl;
        double v = deltas.getValue(deltas.size()/2);
        th = v;
        DBG cerr << "*** Arbitrary gap threshold " << th << endl;
    }


    bool lenBreak = false;
    int lenBox = 5;;
    if (unit.size()>=6) {
        double globalDev = deltas.getDeviation();
        DBG cerr << "global deviation is " << globalDev 
                 << " and threshold is " << th
                 << endl;
        if (globalDev*3>th) {
            DBG cerr << "strange conditioning on signal" << endl;
            lenBreak = true;
        }
    }


    printf("TESTING -- control threshold\n");
    th=20;

    // okay, suppose we are piece-wise continuous, with a 
    // simple threshold th
    // now let's look at the groups formed relative to th
    // then decide on their identity

    vector<Histogram> hists;
    vector<Histogram> histsMerged;
    Histogram hist;
    vector<int> segment(past.size());
    segment[0] = 0;
    int curr = -1;
    DBG2 printf("SEQ: ");
    for (int i=0; i<past.size(); i++) {
        // note: we skip first number
        double v = past[i];
        double pv = v;
        if (i>0) {
            pv = past[i-1];
        }
        double dv = fabs(v-pv);

        bool breakNow = false;
        int next = mags.getIndex(v)/lenBox;
        if (lenBreak) {
            if (i!=0 && curr!=next) {
                breakNow = true;
            }
            curr = next;
        } else {
            if (dv>=th) {
                breakNow = true;
            }
        }
        if (breakNow) {
            if (hist.size()>0) {
                hists.push_back(hist);
                hist.clear();
            }
            //DBG cerr << endl;
            if (i>0) {
                DBG2 printf("/ ");
                segment[i] = segment[i-1]+1;
            }
        } else {
            if (i>0) {
                segment[i] = segment[i-1];
            }
        }
        DBG2 printf("%g ", v);
        //DBG cerr << v << " ";
        hist.add(v);
    }
    //DBG cerr << endl;
    if (hist.size()>0) {
        hists.push_back(hist);
        hist.clear();
        DBG2 printf("/ ");
    }
    DBG2 printf("\n");
    bool predel = 0;
    if (hists.size()>=5) {
        predel = 1;
        //hists[0].clear();
        //hists[hists.size()-1].clear();
    }
    Histogram *hprev = &(hists[predel]);
    for (int i=predel; i<hists.size()-predel; i++) {
        Histogram& h = hists[i];
        //DBG cerr << "Group mean " << h.getMean() << " std " << h.getDeviation() << 
        //  endl;
        //DBG cerr << "size " << h.getLength() << endl;
        if (i>predel) {
            if (h.getLength()<=2 
                && h.getDeviation()<fabs(h.getMean())*0.1
                && fabs(hprev->getMean()-h.getMean())<0.3*mmax(hprev->getMean(),h.getMean())) {
                hprev->add(h.getMean());
                h.add(hprev->getMean());
                //h = *hprev;
                DBG cerr << "crunch" << endl;
            }
        }
        hprev = &h;
    }
    map<int, int> clusterCode;
    map<int, int> clusterCodeMerged;
    for (int i=0; i<hists.size(); i++) {
        clusterCode[i] = i;
    }
    for (int i=0; i<hists.size(); i++) {
        Histogram& h1 = hists[i];
        double m1 = h1.getMean();
        double s1 = h1.getDeviation();
        if (s1<0.5) {
            if (h1.getLength()<2) {
                s1 = 0.45;
            }
        }
        for (int j=i+1; j<hists.size(); j++) {
            Histogram& h2 = hists[j];
            double m2 = h2.getMean();
            double s2 = h2.getDeviation();
            double diff = fabs(m1-m2);
            if (s2<0.5) {
                if (h1.getLength()<2) {
                    s2 = 0.45;
                }
            }
            double scale = 2;
            if (lenBreak) {
                if (mags.getIndex(m1)/lenBox==mags.getIndex(m2)/lenBox) {
                    //DBG cerr << "Link " << i << " and " << j << endl;
                    if (clusterCode[j]>i) {
                        clusterCode[j] = i;
                    }
                }
            } else {
                if (diff<(scale*s1+0.00001)||diff<(scale*s2+0.00001)) {
                    //DBG cerr << "Link " << i << " and " << j << endl;
                    if (clusterCode[j]>i) {
                        clusterCode[j] = i;
                    }
                }
            }
        }
    } 
    map<int, int> label;
    map<int, int> revLabel;
    int ct = 0;
    int prevCode = -1;
    for (int i=0; i<hists.size(); i++) {
        int localCode = clusterCode[i];
        if (label.find(localCode)==label.end()) {
            label[localCode] = ct;
            revLabel[ct] = localCode;
            ct++;
        }
        if (localCode!=prevCode) {
            histsMerged.push_back(Histogram());
        }
        Histogram& hist = histsMerged.back();
        for (int j=0; j<hists[i].getLength(); j++) {
            hist.add(hists[i].getPast(j));
        }
        clusterCodeMerged[histsMerged.size()-1] = localCode;
        prevCode = localCode;
    }  
    int groupCount = ct;
    DBG cerr << "sequence: ";
    sym.clear();
    map<int, int> relabel;
    int relabelCt = 0;
    for (int i=0; i<past.size(); i++) {
        int nextLabel = label[clusterCode[segment[i]]];
        //int nextLabel = label[clusterCodeMerged[i]];
        DBG printf("%d ", nextLabel);
    }
    DBG printf("\n");
    for (int i=predel; i<histsMerged.size()-predel; i++) {
        int nextLabel = label[clusterCodeMerged[i]];
        if (relabel.find(nextLabel)==relabel.end()) {
            relabel[nextLabel] = relabelCt;
            relabelCt++;
        }
        nextLabel = relabel[nextLabel];
        sym.add(nextLabel);
        //DBG cerr << nextLabel << " ";
        //DBG cerr << label[clusterCodeMerged[i]] << " ";
    }
    for (int i=0; i<histsMerged.size(); i++) {
        int nextLabel = label[clusterCodeMerged[i]];
        if (relabel.find(nextLabel)==relabel.end()) {
            relabel[nextLabel] = relabelCt;
            relabelCt++;
        }
    }
    if (predel>0) {
        DBG2 printf("Check labelling: ");
        bool needLeft = true;
        for (int i=0; i<past.size(); i++) {
            int ridx = label[clusterCode[segment[i]]];
            int idx = relabel[ridx];
            if (i>0) {
                int idx2 = relabel[label[clusterCode[segment[i-1]]]];
                if (idx!=idx2) {
                    if (needLeft) {
                        needLeft = false;
                        trimLeft = i;
                    }
                    trimRight = past.size()-i;
                }
                DBG2 printf("%d:%d:%g ", idx, ridx, past[i]);
            }
        }
        trimming = true;
        DBG2 printf("\n");
    }
    DBG cerr << endl;
    DBG cerr << "Unit is: " << toString() << endl;

    DBG cerr << "(trimmed " << trimLeft << "/" << trimRight << ") Symbolic form of " << unit.toString() << " is " << 
        sym.toString() << endl;

    
    vector<Histogram> globalHists(groupCount);
    vector<Histogram> lengthHists(groupCount);
    vector<Sequence> lengthSeqs(groupCount);
    DBG cerr << "group count is " << groupCount << endl;
    for (int i=trimLeft; i<past.size()-trimRight; i++) {
        int idx = relabel[label[clusterCode[segment[i]]]];
        //DBG cerr << "  offset " << i << " segment " << segment[i] << " code "
        //<< idx 
        //<< endl;
        Histogram& h = globalHists[idx];
        h.add(past[i]);
    }


    int tlen = 0;
    int tid = -1;
    DBG2 printf("Scanning: ");
    for (int i=trimLeft; i<past.size()-trimRight; i++) {
        bool trans = false;
        int idx = relabel[label[clusterCode[segment[i]]]];
        if (i>trimLeft) {
            int idx2 = relabel[label[clusterCode[segment[i-1]]]];
            if (idx!=idx2) {
                trans = true;
            }
        }
        DBG2 printf("%d ", idx);
        if (!trans) {
            if (i==past.size()-trimRight-1) {
                tlen++;
                trans = true;
            } 
        }

        if (trans) {
            if (tid>=0) {
                DBG2 printf("[len %d for %d] ", tlen, tid);
                lengthHists[tid].add(tlen);
                lengthSeqs[tid].add(tlen);
            }
            tid = idx;
            tlen = 1;
        } else {
            tid = idx;
            tlen++;
        }
    }
    printf("\n");


    /*
    for (int i=0; i<histsMerged.size(); i++) {
        int len = histsMerged[i].getLength();
        //DBG cerr << "add len " << len << endl;
        lengthHists[label[clusterCodeMerged[i]]].add(len);
        lengthSeqs[label[clusterCodeMerged[i]]].add(len);
    }
    */

    parts = vector<Piece>(groupCount);
    Histogram global;
    for (int i=0; i<groupCount; i++) {
        Piece& p = parts[i];
        p.mean = globalHists[i].getMean();
        p.dev = globalHists[i].getDeviation();
        p.len = lengthHists[i].getMean();
        //if (lengthSeqs[i].size()==0) {
        //lengthSeqs[i].add(p.len);
        //}
        p.lengthUnit = specialize(lengthSeqs[i]);
        global.add(p.len);
        DBG cerr << "Piece " << i << " mean " << p.mean << " len " << p.len << endl;
    }
    meanLength = global.getMean();

    DBG cerr << endl;
    DBG cerr << "*** for " << unit.toString().substr(0,20) << " ..." << endl;
    DBG cerr << "*********************************************************" << 
        endl;

}


bool PiecewiseSequence::extendParts(int len, int pad) {
    bool ok = false;
    int target = 1+int(1.4*len/meanLength)+pad;
    ok = sym.extend(target);
    if (!ok) return false;
    for (int i=0; i<parts.size(); i++) {
        ok = parts[i].lengthUnit.extend(target);
        if (!ok) return false;
    }
    return ok;
}



bool PiecewiseSequence::extend(int len) {
    if (future.size()>=len) {
        return true;
    }
    if (!extendParts(len,0)) {
        return false;
    }
    int pad = 0;
    mt19937 rng;
    rng.seed((unsigned int) time(0));
    while (future.size()<len) {
        future.clear();
        map<int,int> individualCounts;
        for (int i=0; i<sym.futureSize(); i++) {
            int code = (int)(sym.predict(i)+0.5);
            if (code<0) {
                boost::uniform_real<> uni_dist(0,1);
                boost::variate_generator<mt19937&, boost::uniform_real<> >
                    uni(rng, uni_dist);
                code = int(uni()*parts.size());
                DBG cerr << "(weak random unit replacement)" << endl;
            }
            Piece& p = parts[code];
            normal_distribution<double> dist(p.mean,p.dev);
            variate_generator<mt19937&, normal_distribution<double> > 
                normal_sampler(rng, dist);
            double len = p.len;
            if (individualCounts.find(code)==individualCounts.end()) {
                individualCounts[code] = 0;
            }
            len = p.lengthUnit.predict(individualCounts[code]);
            if (i==0) {
                len -= trimRight;
            }
            DBG2 printf("<<< CODE BLOCK %d mu=%g std=%g len=%g\n", 
                        code, p.mean, p.dev, len);
            individualCounts[code]++;
            for (int j=0; j<int(len+0.5); j++) {
                double v = normal_sampler();
                v = (double)((long int)(v*scale+0.5))/scale;
                future.push_back(v);
                //printf(">>> %g\n", v);
            }
        }
        if (future.size()<len) {
            pad += 4;
            DBG cerr << "future size is " << future.size() << endl;
            DBG cerr << "extending sym to " << len+pad << endl;
            if (!extendParts(len,pad)) { 
                DBG2 printf("done piecewise extend 1\n");
                return false;
            }
        }
    }
    DBG2 printf("done piecewise extend 2\n");
    return true;
}




