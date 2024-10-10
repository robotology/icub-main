# Copyright: (C) 2024 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Losi Jacopo, valentina Gaggero
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# rawValuePublisher.thrift

namespace yarp iCub
/**
* rawValuesDataVectorsMap
*
* IDL struct of a map of vectors to store the raw value data sent by the device
*/
struct rawValuesDataVectorsMap
{
   /**
   * contain a map of vectors of the raw data as <string, vector>
   * the user wanna send from low to higher level
   */
   1: map<string, list<i32>> vectorsMap;
}

struct rawValuesKeyMetadata
{
   1: list<string> rawValueNames;
   2: i32 size;
}

struct rawValuesKeyMetadataMap
{
   1: map<string, rawValuesKeyMetadata> metadataMap;
}

service RawValuesPublisherMetadata
{
  /**
   * Read the rawvalues metadata necessary to configure the RawValuesPublisherClient device.
   */
  rawValuesKeyMetadataMap getMetadata();
}