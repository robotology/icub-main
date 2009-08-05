

class Sensor
{

	protected: 
		int nr_of_features;
		double feature[10];

	public:
		Sensor();
		~Sensor();
		int get_nr_of_features();
		int detect_features();
		double get_feature(int feature_nr);
};
