class Filtro
{

	//eq.  do filtro
	//e(k)=alfa*e(k-1) + e(k)*(1-alfa)

	private: 
		double errorP_1;
		float alfa;
		bool initialized;


	public:
		Filtro(float );
		~Filtro();
		double filter(double );
	
};
