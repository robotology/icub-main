// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Chris McCarthy
 *
 */

#include <ImpactData.h>

void packMessage(ImpactData d, Bottle *b)
{
   b->addInt(d.updated);
   b->addInt(d.imageWidth);
   b->addInt(d.imageHeight);
   b->addInt(d.interestPointX);
   b->addInt(d.interestPointY);
   b->addDouble(d.ttc);
	b->addDouble(d.approachAngle);
   b->addDouble(d.impactLocationX);
   b->addDouble(d.impactLocationY);

}


void unpackMessage(Bottle b, ImpactData *d)
{
   Value v;
   int ind = 0;
   d->updated = (b.get(ind++)).asInt();
   d->imageWidth = (b.get(ind++)).asInt();
   d->imageHeight = (b.get(ind++)).asInt();
   d->interestPointX = (b.get(ind++)).asInt();
   d->interestPointY = (b.get(ind++)).asInt();
   d->ttc = (float) (b.get(ind++)).asDouble();
	d->approachAngle = (float) (b.get(ind++)).asDouble();
   d->impactLocationX = (float) (b.get(ind++)).asDouble();
   d->impactLocationY = (float) (b.get(ind++)).asDouble();
}
