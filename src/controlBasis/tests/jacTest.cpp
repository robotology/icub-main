// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include <ForwardKinematics.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

using namespace KIN;

void loadConfig(string fname);

#define TORAD M_PI/180.0
#define TODEG 180.0/M_PI

Matrix DHParameters;
Vector LinkTypes;
int numDOFs;
int numLinks;
double maxSetVal;

ForwardKinematics *fk;

int main(int argc, char *argv[]) {

  string configFile = "right_arm.dh";
  numDOFs = 7;
  numLinks = 12;

  loadConfig(configFile);
  fk = new ForwardKinematics();
  fk->setLinkInfo(numLinks,numDOFs);
  fk->setDHParameters(DHParameters,LinkTypes);

  Vector q(numDOFs);
  q.zero();
  q[3] = 5.5*TORAD;
  fk->updateTransforms(q);

  Vector xpos = fk->getEndEffectorPosition();
  Matrix J = fk->getPositionJacobian();

  printf("pos: [%.3f  %.3f  %.3f]\n\n",  xpos[0], xpos[1], xpos[2]);

  printf("J:\n");
  for(int i=0; i<J.rows(); i++) {
    for(int j=0; j<J.cols(); j++) {
      printf("%.5f   ", J[i][j]); 
    }
    printf("\n");
  }
  printf("\n");
  return 1;
}


void loadConfig(string fname) {


    FILE *fp;
    char line[128];

    Matrix inTransform(4,4);

    char *linkType = (char *)malloc(64);

    if( (fp=fopen(fname.c_str(), "r")) == NULL ) {
        printf("problem opening \'%s\' for reading!!!\n", fname.c_str());
        return;
    }

    float a, d, alpha, theta;
    float t0, t1, t2, t3;
    float maxVal;
    int lnk = 0;

    while(fgets(line, 128, fp) != NULL) {
        if(!strncmp(line,"N",1)) {
            if(fgets(line, 128, fp) !=NULL) {
                sscanf (line, "%d", &numDOFs);
            }
        }
        else if(!strncmp(line,"L",1)) {
            if(fgets(line, 128, fp) !=NULL) {
                sscanf (line, "%d", &numLinks);
                
                LinkTypes.resize(numLinks);
                LinkTypes.zero();
                
                DHParameters.resize(4,numLinks);
                DHParameters.zero();
                
                printf("YARPConfigurationVariables::loadConfig() got DOFs=%d, Links=%d\n",
                       numDOFs, numLinks);
            }
        } else if(!strncmp(line,"DH",2)) {
            for(int i=0; i<numLinks; i++) {
               
                if(fgets(line, 128, fp) !=NULL) {                    
                    sscanf (line, "%f %f %f %f %s", &a, &d, &alpha, &theta, linkType);
                    DHParameters[ForwardKinematics::DH_A][i] = (double)a;
                    DHParameters[ForwardKinematics::DH_D][i] = (double)d;
                    DHParameters[ForwardKinematics::DH_ALPHA][i] = (double)alpha*TORAD;
                    DHParameters[ForwardKinematics::DH_THETA][i] = (double)theta*TORAD;
                    
                    if(!strcmp(linkType, "CONSTANT")) {
                        LinkTypes[i] = ForwardKinematics::LINK_TYPE_CONSTANT;
                    } else if(!strcmp(linkType, "PRISMATIC")) {
                        LinkTypes[i] = ForwardKinematics::LINK_TYPE_PRISMATIC;
                    } else if(!strcmp(linkType, "REVOLUTE")) {
                        LinkTypes[i] = ForwardKinematics::LINK_TYPE_REVOLUTE;
                    } else if(!strcmp(linkType, "NONINTERFERING")) {
                        LinkTypes[i] = ForwardKinematics::LINK_TYPE_NONINTERFERING;
                    } else { 
                        LinkTypes[i] = ForwardKinematics::LINK_TYPE_CONSTANT;
                    }

                    printf("Link[%d]: [%.3f %.3f %.3f %.3f], type=%.0f\n", i, 
                           DHParameters[ForwardKinematics::DH_A][i],
                           DHParameters[ForwardKinematics::DH_D][i],
                           DHParameters[ForwardKinematics::DH_ALPHA][i],
                           DHParameters[ForwardKinematics::DH_THETA][i],
                           LinkTypes[i]);
                }
            }
        }
        else if(!strncmp(line,"MAX",3)) {
            if(fgets(line, 128, fp) !=NULL) {
                sscanf (line, "%f", &maxVal);
                maxSetVal = (double)maxVal;
                printf("MaxSetVal: %f\n", maxSetVal);
            }
        }
    }

    free(linkType);        
    fclose(fp);

}
