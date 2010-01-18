/* Recursive Least Square (RLS) adaptive FIR filter*/


#include "rls.h"
#include <iostream>
#include <fstream>
using namespace std;
RLS::RLS(int order, double mu, double *coeffs, double *init)
{	//init inizializzazione stato X
      
		int sz=sizeof(double)*(order+1);
        int sz2=sizeof(double)*(order+1)*(order+1);
        W=(double *)malloc(sz);	//There are order+1 number of coeffs
        Wfinal=(double *)malloc(sz);
		WfinalImp=(double *)malloc(sz);
		X=(double *)malloc(sz);
		XP=(double *)malloc(sz);//denominatore
		w_final_impost=false;
		G=(double *)malloc(sz);
		P=(double *)malloc(sz2);
		PXX=(double *)malloc(sz2);
		PXXP=(double *)malloc(sz2);
		PX=(double *)malloc(sz);
		Pfinal=(double *)malloc(sz2);
		PfinalX=(double *)malloc(sz);
		n=order;
        if(coeffs!=NULL) for(int i=0; i<=n; i++) W[i]=coeffs[i];
		else for(int i=0; i<=n; i++) {W[i]=0.0;WfinalImp[i]=0.0;}
        if(init!=NULL) for(int i=0; i<=n; i++) X[i]=init[i];
        ff=mu;
        for(int i=0; i<=n; i++)for(int j=0; j<=n; j++) P[i*(n+1)+j]=0.0;
        for(int i=0; i<=n; i++) P[i*(n+1)+i]=1.0;
}

void RLS::setWfinal(double *w){
	w_final_impost=true;
	
	WfinalImp[0]=w[0];
	WfinalImp[1]=w[1];
}
void RLS::resetWfinal(){
w_final_impost=false;
}
double RLS::sample(double *x, double e)/////////////RIGUARDARE  I PARAMETRI,SONO SBAGLIATI
////////////// LA X (t-1) PER W E P, MENTRE X(t) PER Y. 

//x Stato attuale X stato passo t-1
{
		 int nquadro=(n+1)*(n+1);
		
        //Iteratively calculate the inverse autocorrelation matrix P
   //   for(int row=0; row<=n; row++)
		
		 
		 for(int row=0; row<=n; row++)
        { PX[row]=0;
        	
                for(int i=0; i<=n; i++)
					PX[row]+=P[row*(n+1)+i]*X[i];
        }
		
		for(int i=0; i<=n; i++)for(int j=0; j<=n; j++) {
        	PXX[i*(n+1)+j]=0;
			
			PXX[i*(n+1)+j]=PX[i]*X[j];
		}
		
		//for(int j=0; j<=3; j++)
		//	cout<<"PXX vale  "<<PXX[j]<<endl;


		for(int j=0; j<nquadro; j++)
			PXXP[j]=0;

		for(int i=0; i<=n; i++)
			for (int y=0;y<=n;y++){
			for(int k=0; k<=n; k++){
				PXXP[i*(n+1)+y]+=PXX[i*(n+1)+k]*P[y+k*(n+1)];}//fine numeratore
		
			
		//cout<<"PXXP vale  "<<PXXP[i*(n+1)+y]<<endl;
			}
		for (int y=0;y<=n;y++)
		XP[y]=0;
		for (int y=0;y<=n;y++)
		for (int i=0; i<=n; i++)
			XP[y]+=X[i]*P[y+i*(n+1)];

		
		double xxp=0;
		for (int i=0;i<=n;i++)
		xxp+=XP[i]*X[i];
		double muxxp=ff+xxp;//fine denominatore

	

		for (int i=0;i<nquadro;i++)
			Pfinal[i]=(P[i]-(PXXP[i]/muxxp))/ff; //valore di P(t)
		
		
		
		
		//calcolo w(t)
		for (int y=0;y<=n;y++)
		PfinalX[y]=0;
		for (int y=0;y<=n;y++)
		for(int i=0;i<=n;i++)
		PfinalX[y]+=Pfinal[i+y*(n+1)]*X[i];
		
			

		for(int i=0;i<=n;i++)
		PfinalX[i]=PfinalX[i]*e;
		
		
		if(!w_final_impost){
		for(int i=0;i<=n;i++){
		Wfinal[i]=0;
		Wfinal[i]=W[i]+PfinalX[i];
		//cout<<"w vale  "<<Wfinal[i]<<endl;
		}}
		else{
		for(int i=0;i<=n;i++){
		Wfinal[i]=WfinalImp[i];
		}}
			
		
		
		
		double output=0;
		double outputPos=0;
		for(int i=0;i<=n;i++)
			output+=Wfinal[i]*x[i];
			
		
		outputPos=x[0]+x[1]*0.1;
		

		x[1]=(X[1]+x[1])/2.0;//MEDIA
		 for(int i=0; i<=n; i++) {
			 X[i]=x[i];
			 W[i]=Wfinal[i];
			}
		 /////////////////////////OCCHIO!!!!!!!
		

		fstream wuno;
		wuno.open("w1.txt",ios::app);
		wuno<< W[0] <<'\n';
	//	wuno.close;
		fstream wdue;
		wdue.open("w2.txt",ios::app);
		wdue<< W[1] <<'\n';
//		wdue.close; 
		 
		 for(int i=0; i<nquadro; i++) 
		P[i]=Pfinal[i];




		///////////////////////////////////////DA QUI
      /*  double den=ff;
        for(int row=0; row<=n; row++)den+=PX[row]*X[row];





       for(int row=0; row<=n; row++)G[row]=PX[row]/den;

        for(int col=0;col<=n;col++)for(int row=0;row<=n;row++)P[row*(n+1)+col]=(P[row*(n+1)+col]-G[row]*PX[col])/ff;

        //Update filter coefficients
        for(int i=0;i<=n;i++)W[i]=W[i]+G[i]*e;

        //Do the actual filter bit
        double output=0;
        for(int i=n; i>0; i--)X[i]=X[i-1];
        X[0]=x;
        for(int i=0;i<=n;i++)output+=W[i]*X[i];*/

        return output;
}


double * RLS::getCoeffs(double *coeffs)
{
	for(int i=0; i<=n; i++) coeffs[i]=W[i];
        return coeffs;
}


void  RLS::setX(double *valx){
	X[0]=valx[0];
	X[1]=valx[1];


}

void RLS:: setParW(double *valw){
W[0]=valw[0];
W[1]=valw[1];


}
double RLS::setW(double *valw, double *valx){
	 double output=0;
	 for(int i=0;i<=n;i++)output+=valw[i]*valx[i];
	 return output;
}
RLS::~RLS()
{
        //free(W);
        free(X);
        free(G);
        free(P);
        free(PX);
		free(PfinalX);
        free(G);
        free(XP);
        free(PXX);
}