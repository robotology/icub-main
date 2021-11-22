/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * Copyright (C) 2006-2010 RobotCub Consortium
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD-3-Clause license. See the accompanying LICENSE file for
 * details.
*/

#include <memory>
#include <vector>
#include <cmath>
#include <yarp/math/Math.h>
#include <iCub/ctrl/clustering.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;

namespace iCub {
    namespace ctrl {
        namespace dbscan {
            enum class PointType {
                unclassified=-1,
                noise=-2
            };

            struct Data_t {
                const vector<Vector> &points;
                const double epsilon;
                const size_t minpts;
                vector<int> ids;
                Data_t(const vector<Vector> &points_,
                       const double epsilon_,
                       const size_t minpts_) :
                       points(points_), epsilon(epsilon_), minpts(minpts_) {
                    ids.assign(points.size(),(int)PointType::unclassified);
                }
            };

            struct Node_t {
                size_t index;
                shared_ptr<Node_t> next;
            };

            struct Epsilon_neighbours_t {
                size_t num_members;
                shared_ptr<Node_t> head;
                weak_ptr<Node_t> tail;
            };

            /**********************************************************************/
            shared_ptr<Node_t> create_node(const size_t index)
            {
                shared_ptr<Node_t> node(new Node_t());
                node->index=index;
                node->next=nullptr;
                return node;
            }

            /**********************************************************************/
            void append(const size_t index, shared_ptr<Epsilon_neighbours_t> en)
            {
                shared_ptr<Node_t> node=create_node(index);
                if (en->head==nullptr)
                {
                    en->head=node;
                    en->tail=node;
                }
                else
                {
                    en->tail.lock()->next=node;
                    en->tail=node;
                }
                en->num_members++;
            }

            /**********************************************************************/
            shared_ptr<Epsilon_neighbours_t> get_epsilon_neighbours(const size_t index,
                                                                    shared_ptr<Data_t> augData)
            {
                shared_ptr<Epsilon_neighbours_t> en(new Epsilon_neighbours_t());
                for (size_t i=0; i<augData->points.size(); i++)
                {
                    double d=0.0;
                    for (size_t j=0; j<augData->points[index].length(); j++)
                    {
                        d+=pow(augData->points[index][j]-augData->points[i][j],2.0);
                    }
                    if ((i!=index) && (sqrt(d)<=augData->epsilon))
                    {
                        append(i,en);
                    }
                }
                return en;
            }

            /**********************************************************************/
            void spread(const size_t index, shared_ptr<Epsilon_neighbours_t> seeds,
                        const size_t id, shared_ptr<Data_t> augData)
            {
                shared_ptr<Epsilon_neighbours_t> spread=get_epsilon_neighbours(index,augData);
                if (spread->num_members>=augData->minpts)
                {
                    for (shared_ptr<Node_t> node=spread->head; node!=nullptr; node=node->next)
                    {
                        if ((augData->ids[node->index]==(int)PointType::noise) ||
                            (augData->ids[node->index]==(int)PointType::unclassified))
                        {
                            if (augData->ids[node->index]==(int)PointType::unclassified)
                            {
                                append(node->index,seeds);
                            }
                            augData->ids[node->index]=(int)id;
                        }
                    }
                }
            }

            /**********************************************************************/
            bool expand(const size_t index, const size_t id, shared_ptr<Data_t> augData)
            {
                shared_ptr<Epsilon_neighbours_t> seeds=get_epsilon_neighbours(index,augData);
                if (seeds->num_members<augData->minpts)
                {
                    augData->ids[index]=(int)PointType::noise;
                    return false;
                }
                else
                {
                    augData->ids[index]=(int)id;
                    for (shared_ptr<Node_t> h=seeds->head; h!=nullptr; h=h->next)
                    {
                        augData->ids[h->index]=(int)id;
                    }
                    for (shared_ptr<Node_t> h=seeds->head; h!=nullptr; h=h->next)
                    {
                        spread(h->index,seeds,id,augData);
                    }
                    return true;
                }
            }
        }
    }
}


/**********************************************************************/
map<size_t,set<size_t>> DBSCAN::cluster(const vector<Vector> &data,
                                        const Property &options)
{
    double epsilon=options.check("epsilon",Value(1.0)).asFloat64();
    size_t minpts=(size_t)options.check("minpts",Value(2)).asInt32();
    shared_ptr<dbscan::Data_t> augData(new dbscan::Data_t(data,epsilon,minpts));

    size_t id=0;
    for (size_t i=0; i<augData->points.size(); i++)
    {
        if (augData->ids[i]==(int)dbscan::PointType::unclassified)
        {
            if (dbscan::expand(i,id,augData))
            {
                id++;
            }
        }
    }

    map<size_t,set<size_t>> clusters;
    for (size_t i=0; i<augData->points.size(); i++)
    {
        if (augData->ids[i]!=(int)dbscan::PointType::noise)
        {
            clusters[augData->ids[i]].insert(i);
        }
    }
    return clusters;
}

