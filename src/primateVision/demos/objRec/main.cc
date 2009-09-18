/**
 * Client of zdfserver.  
 * 
 * Query if ZDF DOG output is classified.
 * Perhaps only query when cog close to origin.
 * Hence, perhaps re-instantiate drift towards CoG 
 * in ZDFServer?
 * If so, send commands to draw it in iCubSIM
 *
 */ 


#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <qapplication.h>
//MY INCLUDES
#include <display.h>
//client of:
#include <zdfio.h>



using namespace iCub::contrib::primateVision;


int main( int argc, char **argv )
{

  QApplication *a = new QApplication(argc, argv);

 

 

  //probe ZDFServer:
  Port inPort_s;
  inPort_s.open("/objRec/input/serv_params"); 
  Network::connect("/objRec/input/serv_params", "/zdfserver/output/serv_params");
  Network::connect("/zdfserver/output/serv_params", "/objRec/input/serv_params");
  BinPortable<ZDFServerParams> server_response; 
  Bottle empty;
  inPort_s.write(empty,server_response);
  ZDFServerParams zsp = server_response.content();
  std::cout << "ZDFServer Probe Response: " << zsp.toString() << std::endl;
 

  int m_size = zsp.m_size;
  int m_psb  = zsp.m_psb;
  int t_size = zsp.t_size;
  int t_psb  = zsp.t_psb;

  IppiSize tsize={t_size,t_size};
  IppiSize msize={m_size,m_size};
  IppiSize osize={320,240};

  BufferedPort<Bottle> inPort_seg_dog;      // Create a port
  inPort_seg_dog.open("/objRec/input/seg_dog");     // Give it a name on the network.
  Network::connect("/zdfserver/output/seg_dog" , "/objRec/input/seg_dog");
  Bottle *inBot_seg_dog;
  Ipp8u  *zdf_im_seg_dog;


  iCub::contrib::primateVision::Display *d_seg_dog  = new iCub::contrib::primateVision::Display(msize,m_psb,D_8U,"ZDF_SEG_DOG");




  printf("begin..\n");

  //main event loop:
  while (1){
    
    inBot_seg_dog = inPort_seg_dog.read(false);
    

    if (inBot_seg_dog!=NULL){
      zdf_im_seg_dog = (Ipp8u*) inBot_seg_dog->get(0).asBlob();

      //DISPLAY:
      d_seg_dog->display(zdf_im_seg_dog);

      //CLASSIFY:
      printf("CHECKING CLASSIFICATION...\n");
      
      //add classification code:
 


      //OUTPUT DISPLAY TO SIM:


      printf("DONE.\n");
    }

    





    if (inBot_seg_dog==NULL){
      printf("No Input\n");
      usleep(5000);// don't blow out port
    }
   
  }
  
  //never here! 
  
}

