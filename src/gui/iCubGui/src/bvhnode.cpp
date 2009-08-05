/*
 * bvhnode.cpp
 */

#include "bvhnode.h"

const QString& BVHNode::name() const
{
    return m_name;
}

void BVHNode::setName(const QString& name)
{
    m_name=name;
}

int BVHNode::numChildren() const
{
    return children.count();
}
/*
BVHNode* BVHNode::child(int num)
{
    return children.at(num);
}
*/
void BVHNode::addChild(BVHNode* pChild)
{
    children.append(pChild);
}
/*
void BVHNode::insertChild(BVHNode* pChild,int index)
{
    children.insert(index,pChild);
}

void BVHNode::removeChild(BVHNode* pChild)
{
    children.removeAll(pChild);
}
*/
