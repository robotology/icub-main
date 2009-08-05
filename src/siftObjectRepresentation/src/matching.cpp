// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Authors: 2008 Dario Figueira
 *          Bruno Damas
 *      Institute for Systems and Robotics (IST)
 *      IST, TULisbon, Portugal
 */

#include "matching.h"

/********  Static ********/

Matching::Param Matching::param;
bool Matching::initialized = false;

bool Matching::init( string const &filename ) {
    //Default values
    param.max_images = 50;
    param.max_matches = 200;
    param.max_models = 20;

    param.bin_theta_size = 30;
    param.bin_scale_size = 2;
    param.bin_pos_size = 0.25;
    param.scale_probability = 0.5;
    param.false_match = 0.1;
    param.correct_match = 0.95;
    param.significance = 0.01;

    param.distance_ratio	= 0.8;

    param.min_models_per_bin = 3;

    initialized = true;

    if( filename == "" )
        return true;

    // Read from file
    ifstream file;
    file.open( filename.c_str() );
    if( !file.is_open() ) {
        MESSAGE("Error reading configuration file" << filename);
        return false;
    }

    string buffer;
    MESSAGE("Loading parameters from " << filename);
    while( NOT (file >> buffer).eof() ) {
        if( !strcmp(buffer.c_str(), "max_images") ) {
            if( file >> param.max_images )
                MESSAGE("\t max_images: " << param.max_images);
        } else if( !strcmp(buffer.c_str(), "max_matches") ) {
            if( file >> param.max_matches )
                MESSAGE("\t max_matches: " << param.max_matches);
        } else if( !strcmp(buffer.c_str(), "max_models") ) {
            if( file >> param.max_models )
                MESSAGE("\t max_models: " << param.max_models);
        } else if( !strcmp(buffer.c_str(), "bin_theta_size") ) {
            if( file >> param.bin_theta_size )
                MESSAGE("\t bin_theta_size: " << param.bin_theta_size);
        } else if( !strcmp(buffer.c_str(), "bin_scale_size") ) {
            if( file >> param.bin_scale_size )
                MESSAGE("\t bin_scale_size: " << param.bin_scale_size);
        } else if( !strcmp(buffer.c_str(), "bin_pos_size") ) {
            if( file >> param.bin_pos_size )
                MESSAGE("\t bin_pos_size: " << param.bin_pos_size);
        } else if( !strcmp(buffer.c_str(), "scale_probability") ) {
            if( file >> param.scale_probability )
                MESSAGE("\t scale_probability: " << param.scale_probability);
        } else if( !strcmp(buffer.c_str(), "false_match") ) {
            if( file >> param.false_match )
                MESSAGE("\t false_match: " << param.false_match);
        } else if( !strcmp(buffer.c_str(), "correct_match") ) {
            if( file >> param.correct_match )
                MESSAGE("\t correct_match: " << param.correct_match);
        } else if( !strcmp(buffer.c_str(), "significance") ) {
            if( file >> param.significance )
                MESSAGE("\t significance: " << param.significance);
        } else if( !strcmp(buffer.c_str(), "distance_ratio") ) {
            if( file >> param.distance_ratio )
                MESSAGE("\t distance_ratio: " << param.distance_ratio);
        } else if( !strcmp(buffer.c_str(), "min_models_per_bin") ) {
            if( file >> param.min_models_per_bin )
                MESSAGE("\t min_models_per_bin: " << param.min_models_per_bin);
        }

        file.clear();   //Just to override eventual garbage at the end of the line
        file.ignore(100,'\n');
    }

    file.close();


    return true;
}

void Matching::test_class() {}


/********  Public ********/

void Matching::start() {
    if( !initialized )
        init();

    training_samples.reset(param.max_images);

    matched_image = 0;

    all_features = NULL;
    number_of_all_features = 0;
    kd_image_root_all = NULL;
/*    keypoint_indexes = NULL;
    image_indexes = NULL;*/
}


void Matching::copy(Matching const &other) {
    training_samples = other.training_samples;
    matches = other.matches;
    hough_table = other.hough_table;
    hough_bins = other.hough_bins;
    models = other.models;

    image_to_match = other.image_to_match;
    matched_image = cvCloneImage(other.matched_image); //MAL?

    this->number_of_all_features = other.number_of_all_features;
    this->all_features = (feature *) calloc( this->number_of_all_features, sizeof(struct feature) );
        if(this->all_features == NULL) {cout<<"Unable to allocate memory"<<endl; exit(1);}
/*    this->keypoint_indexes = (int *) calloc( this->number_of_all_features, sizeof(int) );
        if(this->keypoint_indexes == NULL) {cout<<"Unable to allocate memory"<<endl; exit(1);}
    this->image_indexes = (int *) calloc( this->number_of_all_features, sizeof(int) );
        if(this->image_indexes == NULL) {cout<<"Unable to allocate memory"<<endl; exit(1);}*/
    for(int dur = 0; dur < this->number_of_all_features; dur++) {
        clone_feature((this->all_features + dur), (other.all_features + dur));
/*        this->keypoint_indexes[dur] = other.keypoint_indexes[dur];
        this->image_indexes[dur] = other.image_indexes[dur];*/
    }
    this->kd_image_root_all = kdtree_build( this->all_features, this->number_of_all_features );
}


void Matching::destroy() {
    cvReleaseImage(&matched_image);

    kdtree_release( this->kd_image_root_all );
    if(this->all_features != NULL) {
        for(int i = 0; i < this->number_of_all_features; i++) {
            if((this->all_features + i)->feature_data != NULL)
                free((this->all_features + i)->feature_data);
        }
        free(this->all_features);
    }
/*    if(this->keypoint_indexes != NULL) 
        free(this->keypoint_indexes);
    if(this->image_indexes != NULL) 
        free(this->image_indexes);*/
}

void Matching::restart() {
    destroy();
    start();
}

void Matching::rebuild_kd_tree(SIFT_Image image, int image_index){
//     cout<<"Rebuilding kd-tree: "<<this->number_of_all_features + image.number_of_features<<endl;
    ///copy features from database and new image to new vector
    int new_number = this->number_of_all_features + image.number_of_features;
    struct feature* new_all_features = (feature *) calloc( new_number, sizeof(struct feature) );
        if(new_all_features == NULL) {cout<<"Unable to allocate memory"<<endl; exit(1);}
//     int *new_keypoint_indexes = (int *) calloc( new_number, sizeof(int) );
//         if(new_keypoint_indexes == NULL) {cout<<"Unable to allocate memory"<<endl; exit(1);}
//     int *new_image_indexes = (int *) calloc( new_number, sizeof(int) );
//         if(new_image_indexes == NULL) {cout<<"Unable to allocate memory"<<endl; exit(1);}

    for(int dur = 0; dur < this->number_of_all_features; dur++) {///from database
        clone_feature((new_all_features + dur), (this->all_features + dur));
/*        new_keypoint_indexes[dur] = this->keypoint_indexes[dur];
        new_image_indexes[dur] = this->image_indexes[dur];*/
/*                    cout << "ANTex1 ="<<(this->all_features+dur)->keypoint_index;
                    cout << "\t ANTex2 ="<<(new_all_features+dur)->keypoint_index;
                    cout <<" \t ANTex3:"<<new_keypoint_indexes[dur];
                    cout <<" \t ANTex4:"<<this->keypoint_indexes[dur];
                    cout << "\t ANTge2="<<new_all_features[dur].image_index;
                    cout <<"\t ANTge3:"<<new_image_indexes[dur]<<endl;*/
    }
//     cout<<"Cloned old DB, "<< this->number_of_all_features <<" features."<<endl;
//     cout<<"image.number_of_features = "<<image.number_of_features<<endl;
    for(int dur = 0; dur < image.number_of_features; dur++) {///from new image
//         cout<<"i="<<dur<<" f="<<dur + this->number_of_all_features<<endl;
        clone_feature((new_all_features + dur + this->number_of_all_features), (image.features + dur));
/*        new_keypoint_indexes[dur + this->number_of_all_features] = image.keypoint_indexes[dur];
        new_image_indexes[dur + this->number_of_all_features] = image_index;*/
/*                    cout << "COPex1 ="<<(image.features + dur)->keypoint_index;
                    cout << "\t COPex2 ="<<new_all_features[dur + this->number_of_all_features].keypoint_index;
                    cout <<" \t COPex3:"<<new_keypoint_indexes[dur + this->number_of_all_features];
                    cout << "\t COPge2="<<new_all_features[dur + this->number_of_all_features].image_index;
                    cout <<"\t COPge3:"<<new_image_indexes[dur + this->number_of_all_features]<<endl;*/
    }
//     cout<<"Cloned new image features"<<endl;

    ///destroy old database
    kdtree_release( this->kd_image_root_all );
    if(this->all_features != NULL) {
        for(int i = 0; i < this->number_of_all_features; i++) {
            if((this->all_features + i)->feature_data != NULL)
                free((this->all_features + i)->feature_data);
        }
        free(this->all_features);
    }
//     if(this->keypoint_indexes != NULL) 
//         free(this->keypoint_indexes);
//     if(this->image_indexes != NULL) 
//         free(this->image_indexes);

/*    for(int dur = 0; dur < 10; dur++) {///from database
                    cout << "\t ANTex2 ="<<(new_all_features+dur)->keypoint_index;
                    cout <<" \t ANTex3:"<<new_keypoint_indexes[dur] <<endl;
    }
    cout<<"rabo??"<<endl;*/
    ///rebuild kd_tree
    this->kd_image_root_all = kdtree_build( new_all_features, new_number );
//     for(int dur = 0; dur < 10; dur++) {///from database
//                     cout << "\t ANTex2 ="<<(new_all_features+dur)->keypoint_index;
//                     cout <<" \t ANTex3:"<<new_keypoint_indexes[dur]<<endl;
//     }

    this->all_features = new_all_features;
    this->number_of_all_features = new_number;
/*    this->keypoint_indexes = new_keypoint_indexes;
    this->image_indexes = new_image_indexes;*/
}

Matching::Matching() : hough_table(HASH_MAP_SIZE) {
    start();
    MESSAGE("Started");
}

Matching::Matching(Matching const &other) {
    copy(other);
}


Matching &Matching::operator=(Matching const &other) {
    if (this != &other) {
        destroy();
        copy(other);
    }

    return *this;
}


Matching::~Matching() {
    destroy();
    MESSAGE("Terminated");
}


bool Matching::add_training_image(SIFT_Image const &img) {
    MESSAGE("Adding image to training image database...");
    insert_database_image( img );
    return true;
}


bool Matching::add_training_image(IplImage *img) {
    MESSAGE("Processing and adding image to training image database...");
    SIFT_Image new_image;

    if( img == 0 || new_image.load(img) == false)
        return false;

    insert_database_image( new_image );
}

bool Matching::add_training_image(IplImage *img, string label) {
    MESSAGE("Processing and adding image to training image database...");
    SIFT_Image new_image;

    if( img == 0 || new_image.load(img) == false)
        return false;
    new_image.label = label;

    insert_database_image( new_image );
}

bool Matching::add_training_image(string const &filename) {
    IplImage *img;
    if( (img = cvLoadImage( filename.c_str() )) == 0 ) {
        cerr << "Could not load image file: " << filename << endl;
        return false;
    }

    MESSAGE("Processing and adding " << filename << " to training image database...");

    SIFT_Image new_image;
    //	int trim=1;
    // 	if( img == 0 || new_image.load(img, trim) == false)
    // 		return false;
    if( img == 0 || new_image.load(img) == false)
        return false;

    // 	if(new_image.features == NULL) cout << "new_image.features == NULL" << endl;//DEBUG
    // 	else if(new_image.kd_image_root == NULL) cout << "kd_image_root == NULL" << endl;//DEBUG
    // 	else
    // 	{
    // 		cout <<"new_image.x="<< new_image.features->x <<" y="<< new_image.features->y <<" scl="<< new_image.features->scl <<" ori="<< new_image.features->ori << endl; //DEBUG
    //
    // 	cout << "kd_image_root->n=" << new_image.kd_image_root->n;
    // 	cout << " feature: "<<" x="<< new_image.kd_image_root->features->x <<" y="<< new_image.kd_image_root->features->y <<" scl="<< new_image.kd_image_root->features->scl <<" ori="<< new_image.kd_image_root->features->ori << endl; //DEBUG
    //
    // 	cout << "kd_image_root->kd_left->n=" << new_image.kd_image_root->kd_left->n << endl;
    // 	cout << " feat: "<<" x="<< new_image.kd_image_root->kd_left->features->x <<" y="<< new_image.kd_image_root->kd_left->features->y <<" scl="<< new_image.kd_image_root->kd_left->features->scl <<" ori="<< new_image.kd_image_root->kd_left->features->ori << endl; //DEBUG
    //
    // 	cout << "kd_image_root->kd_right->n=" << new_image.kd_image_root->kd_right->n << endl;
    // 	cout << " feat: "<<" x="<< new_image.kd_image_root->kd_right->features->x <<" y="<< new_image.kd_image_root->kd_right->features->y <<" scl="<< new_image.kd_image_root->kd_right->features->scl <<" ori="<< new_image.kd_image_root->kd_right->features->ori << endl; //DEBUG
    //
    // 	}

    insert_database_image( new_image );

    cvReleaseImage( &img );
    return true;
}

bool Matching::add_training_image(string const &filename, string label) {
    IplImage *img;
    if( (img = cvLoadImage( filename.c_str() )) == 0 ) {
        cerr << "Could not load image file: " << filename << endl;
        return false;
    }

    MESSAGE("Processing and adding " << filename << " to training image database...");

    SIFT_Image new_image;
    if( img == 0 || new_image.load(img) == false)
        return false;
    new_image.label = label;

    insert_database_image( new_image );

    cvReleaseImage( &img );
    return true;
}

void Matching::match_to_database(SIFT_Image const &img) {
//     MESSAGE( "Loading test image...");
    image_to_match = img;
/*    cout<<"rabo2?"<<endl;*/
    match_to_database();
}


void Matching::match_to_database(IplImage *img) {
//     MESSAGE( "Loading test image...");
    if( !image_to_match.load(img) )
        cerr << "Matching::match_to_database: could not load image" << endl;
    else
        match_to_database();
}


void Matching::match_to_database(string const &filename) {
    IplImage *img;
    if( (img = cvLoadImage( filename.c_str() )) == 0 ) {
        cerr << "Could not load image file: " << filename << endl;
        return;
    }

    match_to_database( img );
    MESSAGE( filename << " loaded");
    cvReleaseImage( &img );
}

void Matching::save_database() { //(string const &filename)
    char nome[100];
    string filename;
    int counter = 0;
    IplImage *img;

    MESSAGE("Saving database");

    filename = "database/database.txt\0";
    FILE * pFile;
    pFile = fopen (filename.c_str(),"w");
    if (pFile==NULL) {
        mkdir("database", S_IRWXU|S_IRWXG|S_IRWXO);
        pFile = fopen (filename.c_str(),"w");
        if (pFile==NULL) {
            MESSAGE("Not able to create/overwrite database.txt");
            return;
        }
    }
    fprintf (pFile,"%d #number of images\n", training_samples.get_size());
    printf ("%d #number of images\n", training_samples.get_size());

    for(int dbase_img = training_samples.go_to_first(); dbase_img != -1; dbase_img = training_samples.go_to_next()) {
        sprintf(nome, "%d.txt\0", dbase_img);
        string filename = "database/\0";
        filename.append(nome);
        // @return Returns 0 on success or 1 on error
        if( export_features( (char*)filename.c_str(), training_samples[dbase_img].features, training_samples[dbase_img].number_of_features ))
            cout << "ERRO exporting features of dbase_img:"<< dbase_img << endl;

        sprintf(nome, "%d.png\0", dbase_img);
        filename = "database/\0";
        filename.append(nome);

        img = (IplImage *) training_samples[dbase_img].get_image_color();
        // 		cvResetImageROI( img );
        // 		IplImage *tmp = cvCreateImage( cvSize(img->width,img->height), IPL_DEPTH_8U, 1 );
        // 		cvConvertScaleAbs( img, tmp, 255 );
        if((img->nChannels == 3 || img->nChannels == 1) && img->depth == IPL_DEPTH_8U) {
            cvSaveImage(filename.c_str(), img);
        } else
            cout << "Unable to save image "<<dbase_img<<"nChannels= "<<img->nChannels<<" depth="<<img->depth<<endl;

        ///The database assumes each label is finished by an "/n"
        if(training_samples[dbase_img].label.empty()){
            fprintf (pFile,"%d\n", counter);
            printf ("%d\n", counter);
        }else{
            fprintf (pFile,"%s", training_samples[dbase_img].label.c_str());
            printf ("Saved:%s", training_samples[dbase_img].label.c_str());
        }

        counter ++;
    }

    fclose (pFile);
}



extern Keypoint feature2keypoint(struct feature* feat);
void Matching::load_database() {
    cout<<"Loading database.."<<endl;
    FILE * pFile = NULL;
    pFile = fopen ("database/database.txt","r");
    if (pFile==NULL) {
        MESSAGE("Not able to open database.txt");
        return;
    }
    char nome[100];
    char * line = NULL;
    size_t len = 0;
    string filename;

    ssize_t read = getline(&line, &len, pFile); //the 1st line has the database size
    int num_images;
    if(sscanf(line,"%d ", &num_images)!=1) {
        MESSAGE("Failure reading database.txt");
        return;
    }
//     cout<<"Read:"<<line<<endl;

    for(int i=0; i<num_images; i++) {
        SIFT_Image new_image;

        ///loading the name
        read = getline(&line, &len, pFile); 
//         cout<<"Read:"<<line<<endl;
        if (read == -1 || read == 0) {
            cout << "Possible corruption of database.txt" << endl;
            new_image.label.clear();
        } else
            new_image.label.assign(line);
        cout<<"Loaded:"<<new_image.label<<endl;

        
        ///getting the image itself
        sprintf(nome, "%d.png\0", i);
        filename = "database/\0";
        filename.append(nome);
        IplImage* imagem = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_ANYCOLOR);
        new_image.image_color = imagem;
        new_image.image = cvCreateImage( cvGetSize(imagem), SIFT_Image::param.depth, 1 );
        if( imagem->nChannels > 1 ) {
            IplImage* tmp = cvCreateImage( cvGetSize(imagem), SIFT_Image::param.depth, imagem->nChannels );
            cvConvertScale( imagem, tmp, 1.0/255 );
            cvCvtColor( tmp, new_image.image, CV_BGR2GRAY );
            cvReleaseImage( &tmp );
        }
//         cout<<"Loaded Image"<<endl;
        // 		else
        // 			cvConvertScale( imagem, new_image.image, 1.0/255 );


        ///loading the features
        sprintf(nome, "%d.txt\0", i);
        filename = "database/\0";
        filename.append(nome);
        int num = -1;
        if((num = import_features( (char *)filename.c_str(), FEATURE_LOWE, &new_image.features )) == -1) {
            cout << "WARNING: erro a load imagem:"<<i<< endl;
        }
//         cout<<"Imported Image features"<<endl;
        new_image.number_of_features = num;

        new_image.kd_image_root = kdtree_build( new_image.features, num );
//         cout<<"Built kd-tree"<<endl;

//         new_image.keypoint_indexes = (int *) calloc( new_image.number_of_features, sizeof(int) );
//         if(new_image.keypoint_indexes == NULL) {cout<<"Unable to allocate memory"<<endl; exit(1);}

        ///loading the features into the Keypoint database
        Keypoint kp;
        struct feature *aux;
        new_image.keypoints.reset(new_image.param.max_keypoints);

        int index;
        for(int i = 0; i < num; i++ ) {
            aux = new_image.features + i;
            kp = feature2keypoint(aux);

            index = new_image.add_to_database( kp );
            aux->keypoint_index = index;
//             new_image.keypoint_indexes[i] = index;
        }
//         cout<<"Transformed features to keypoints"<<endl;

//         read = getline(&line, &len, pFile); 
//         if(read>0)
//             new_image.label = line;
//         //if(new_image.label !=NULL
//         cout<<"Loaded:"<<new_image.label<<endl;

        insert_database_image(new_image);
    }
    free(line);
    fclose (pFile);
    MESSAGE("Database loaded!");
}


void Matching::load_images(string const &filename) {
    ifstream file;
    string buffer;
    file.open( filename.c_str() );
    if( !file.is_open() ) {
        MESSAGE("Could not read" << filename);
        return;
    }

    while( NOT (file >> buffer).eof() ) {
        add_training_image(buffer);

        file.clear();   //Just to override eventual garbage at the end of the line
        file.ignore(100,'\n');
    }

    file.close();
}



Database<Matching::Model> &Matching::get_models() {
    return models;
}


Database<SIFT_Image> &Matching::get_database_images() {
    return training_samples;
}

///Making a bottle
// helper function to pass correct matrix to projectors
void convertRobMatrix(RobMatrix &robMatrix, double *matrix){
    double tmp[9];
    for (int b = 0; b < 3; b++)
        for (int a =0; a < 3; a++)
            tmp[b*3 + a] = robMatrix.M[b][a];
    // convert to 'intuitive' reference frame (axis switching)
    matrix[0] = -tmp[4];
    matrix[1] = -tmp[7];
    matrix[2] = tmp[1];
    matrix[3] = -tmp[5];
    matrix[4] = -tmp[8];
    matrix[5] = tmp[2];
    matrix[6] = -tmp[3];
    matrix[7] = -tmp[6];
    matrix[8] = tmp[0];
}

inline void transpose(double *src, double *dst){
    dst[0] = src[0]; dst[1] = src[3]; dst[2] = src[6];
    dst[3] = src[1]; dst[4] = src[4]; dst[5] = src[7];
    dst[6] = src[2]; dst[7] = src[5]; dst[8] = src[8];
}

void calcGazeVector(double *rotCamera2World, double *gaze){
    double vct[3]; 
    vct[0] = 0.0; vct[1] = 0.0; vct[2] = 1.0; // 1 0 0
    gaze[0] = rotCamera2World[0] * vct[0] + rotCamera2World[1] * vct[1] + rotCamera2World[2] * vct[2];
    gaze[1] = rotCamera2World[3] * vct[0] + rotCamera2World[4] * vct[1] + rotCamera2World[5] * vct[2];
    gaze[2] = rotCamera2World[6] * vct[0] + rotCamera2World[7] * vct[1] + rotCamera2World[8] * vct[2];  
}

//azimuth and elevation in degrees
void calcAngles(double *gaze, double &azimuth, double &elevation) {
   // THIS IS AN IMPLEMENTATION OF ATAN2, FOR OUR CASE
   if( gaze[0] > 0.0 )
      azimuth = -atan(gaze[1]/gaze[0]) * 180.0 / M_PI;
   else if (gaze[0] < 0.0)
       if (gaze[1] > 0.0)
           azimuth = -180.0 - atan(gaze[1]/gaze[0]) * 180.0 / M_PI;
       else
           azimuth = 180.0 - atan(gaze[1]/gaze[0]) * 180.0 / M_PI;
   else // gaze[0]== 0
       if (gaze[1] > 0.0)
           azimuth = -90.0;
       else if (gaze[1] < 0.0)
           azimuth = 90.0;
       else // gaze[1]==0
           azimuth = 0.0; //INDETERMINATE CASE - SHOULD NEVER HAPPEN

    elevation = -atan(gaze[2]/sqrt(pow(gaze[0],2) + pow(gaze[1],2))) * 180 / M_PI;
}

/**
Makes a bottle with the positions of the matched models. 
Expects "image_to_match" to be filled
*/
int Matching::make_bottle(yarp::os::Bottle * bawtle, double* encoders, double fx, double fy){

    Bottle size;
    size.addString("size");
    size.addInt(models.get_size());

    RobMatrix _eyeMatrix;
    RobMatrix matEyeIntuitive;
    iCubHeadKinematics _headKin;

    ///Aucustic Map expects an input of azimuth and elevation on the image only, it also has the input of the encoders and does the rest of the map there, no need to calculate absolute angles here.
    double _gaze[3];   //DO THIS VECTORS NEED TO BE FREE() ? Nop, they don't
    double _rotW2C[9]; // world to camera rotation
    double _rotC2W[9]; // camera to world rotation
    double _azimuth;
    double _elevation;
    
    //needed to calculate absolute angles to use in direct saccading
    if(models.get_size() > 0){
        // kinematics
        _eyeMatrix = _headKin.fkine(encoders, 'l'); // 'l' -> use leftt eye kinematics
        convertRobMatrix(_eyeMatrix, _rotW2C); 
        transpose(_rotW2C, _rotC2W);
        calcGazeVector(_rotC2W, _gaze);
        calcAngles(_gaze, _azimuth, _elevation);   
    }
    else{
        //return 0;
    }

    Bottle time_b;
    time_b.addString("time");
    time_b.addDouble(image_to_match.frame_time);
    Bottle time_sec;
    time_sec.addString("sec");
    time_sec.addDouble(image_to_match.sec);
    Bottle time_usec;
    time_usec.addString("usec");
    time_usec.addDouble(image_to_match.usec);

    Bottle encod_bottle;
    encod_bottle.addString("encoders");
    encod_bottle.addDouble(encoders[0]);
    encod_bottle.addDouble(encoders[1]);
    encod_bottle.addDouble(encoders[2]);
    encod_bottle.addDouble(encoders[3]);
    encod_bottle.addDouble(encoders[4]);
    encod_bottle.addDouble(encoders[5]);

    bawtle->addList() = size;
    bawtle->addList() = time_b;
    bawtle->addList() = time_sec;
    bawtle->addList() = time_usec;
    bawtle->addList() = encod_bottle;

    CvPoint left_up, middle, right_down;
    int x,y,width,height;
    int counter = 0;

    for( int i = models.go_to_first(); i != -1; i = models.go_to_next() ) {
        x = 0;
        y = 0;
        left_up.x = static_cast<int>(models[i].M[0][0]*x + models[i].M[0][1]*y + models[i].T[0][0]);
        left_up.y = static_cast<int>(models[i].M[1][0]*x + models[i].M[1][1]*y + models[i].T[1][0]);

        x = training_samples[models[i].image].get_image()->width;
        y = training_samples[models[i].image].get_image()->height;
        right_down.x = static_cast<int>(models[i].M[0][0]*x + models[i].M[0][1]*y + models[i].T[0][0]);
        right_down.y = static_cast<int>(models[i].M[1][0]*x + models[i].M[1][1]*y + models[i].T[1][0]);

        middle.x = static_cast<int>(left_up.x + right_down.x)/2;
        middle.y = static_cast<int>(left_up.y + right_down.y)/2;
        width = image_to_match.get_image()->width;
        height = image_to_match.get_image()->height;

//         cout << width <<endl;
//         cout << atan2((0 - width/2), fx)*180/M_PI << " ";
//         cout << atan2((100 - width/2), fx)*180/M_PI << " ";
//         cout << atan2((310 - width/2), fx)*180/M_PI << " ";
//         cout << atan2((320 - width/2), fx)*180/M_PI << " ";
//         cout << atan2((600 - width/2), fx)*180/M_PI << " ";
//         cout << atan2((640 - width/2), fx)*180/M_PI << " ";
//         cout << endl;

        //used to input in the objectMap, which asks for azimuth elevation relative to the image, not absolute
        Bottle azimuth;
        azimuth.addString("azimuth");
        //azimuth.addDouble(-atan2((middle.x - width/2), fx)*180/M_PI); ///i think this works for direct saccading, but not for egosphere, let me check
        azimuth.addDouble(-atan2((middle.x - width/2), fx)*180/M_PI);
        
        //cout<<middle.x<< " - "<< width/2 <<" -atan2 "<< -atan2((middle.x - width/2),fx) << " M_PI "<<M_PI<< "azimuth" << -atan2((middle.x - width/2), fx)*180/M_PI << endl;

        Bottle elevation;
        elevation.addString("elevation");
        //elevation.addDouble(atan2((height/2 - middle.y), fy)*180/M_PI); ///this works for direct saccading, but not for egosphere? weird...
        elevation.addDouble(-atan2((height/2 - middle.y), fy)*180/M_PI);
        
        /// atan(x' / fx) = alpha'      is this correct??? (fx: focal length em x; x': Origin being the center of the img; alpha': azimuth ) 

        //used for saccading directly from siftObjectRepresentationModule
        Bottle abs_azimuth;
        abs_azimuth.addString("abs_azimuth");
        abs_azimuth.addDouble(-atan2((middle.x - width/2), fx)*180/M_PI - _azimuth); 

        Bottle abs_elevation;
        abs_elevation.addString("abs_elevation");
        abs_elevation.addDouble(atan2((height/2 - middle.y), fy)*180/M_PI - _elevation);/// -_elevation BECAUSE OTHERWIZE IT GIVES WRONG RESULTS :(( WHY??
        

        Bottle x;
        x.addString("x");
        x.addInt(middle.x);

        Bottle y;
        y.addString("y");
        y.addInt(middle.y);

        Bottle label;
        label.addString("label");
        label.addString(training_samples[models[i].image].label.c_str());

        Bottle match;
        match.addInt(counter);
        match.addList() = x;
        match.addList() = y;
        match.addList() = label;
        match.addList() = azimuth;
        match.addList() = elevation;
        match.addList() = abs_azimuth;
        match.addList() = abs_elevation;

        bawtle->addList() = match;
        
        counter++;
    }
    //cout << "bawtle: "<<bawtle->toString()<<endl;

    return models.get_size();
}
///END Making a bottle

IplImage* Matching::update_matched_image() {
    //cout<<"Inicio de update_matched_image()"<< endl;

    if( image_to_match.get_image_color() == 0 )
        return NULL;
    CvScalar color = CV_RGB( 255, 0, 0 );
    //	IplImage *image = cvCloneImage( image_to_match.get_image() );
    if(matched_image != 0)
        cvReleaseImage(&matched_image);

    matched_image = cvCloneImage(image_to_match.get_image_color());
    //     matched_image = cvCreateImage( cvSize(image_to_match.get_image_color()->width,image_to_match.get_image_color()->height), IPL_DEPTH_8U,3);
    // cout<<"im color "<< image_to_match.get_image_color()->width<<" "<< image_to_match.get_image_color()->height<< " "<<image_to_match.get_image_color()->depth<< endl;
    // cout<<"im "<< image_to_match.get_image()->width<<" "<< image_to_match.get_image()->height<< " "<<image_to_match.get_image_color()->depth<<endl;
    // cout<<"matched "<< matched_image->width<<" "<< matched_image->height<< " "<<matched_image->depth<<endl;
    //     cvConvertImage( image_to_match.get_image_color(), matched_image ); //cvConvertImage( image_to_match.get_image(), image );
    // cout<< "d"<<endl;

    int red_green_blue = 0;
    CvPoint left_up, left_down, right_up, right_down;
    int x,y;

    for( int i = models.go_to_first(); i != -1; i = models.go_to_next() ) {
        x = 0;
        y = 0;
        left_up.x = static_cast<int>(models[i].M[0][0]*x + models[i].M[0][1]*y + models[i].T[0][0]);
        left_up.y = static_cast<int>(models[i].M[1][0]*x + models[i].M[1][1]*y + models[i].T[1][0]);

        x = 0;
        y = training_samples[models[i].image].get_image()->height;
        left_down.x = static_cast<int>(models[i].M[0][0]*x + models[i].M[0][1]*y + models[i].T[0][0]);
        left_down.y = static_cast<int>(models[i].M[1][0]*x + models[i].M[1][1]*y + models[i].T[1][0]);

        x = training_samples[models[i].image].get_image()->width;
        y = 0;
        right_up.x = static_cast<int>(models[i].M[0][0]*x + models[i].M[0][1]*y + models[i].T[0][0]);
        right_up.y = static_cast<int>(models[i].M[1][0]*x + models[i].M[1][1]*y + models[i].T[1][0]);

        x = training_samples[models[i].image].get_image()->width;
        y = training_samples[models[i].image].get_image()->height;
        right_down.x = static_cast<int>(models[i].M[0][0]*x + models[i].M[0][1]*y + models[i].T[0][0]);
        right_down.y = static_cast<int>(models[i].M[1][0]*x + models[i].M[1][1]*y + models[i].T[1][0]);

        cvLine( matched_image, left_up, left_down, color, 2 );
        cvLine( matched_image, left_down, right_down, color, 2 );
        cvLine( matched_image, right_down, right_up, color, 2 );
        cvLine( matched_image, right_up, left_up, color, 2 );
        
        ///Removing the expected '/n' from the end of object label so we can print to the image
        string label = training_samples[models[i].image].label;
        char* label_copy = new char[label.size()+1];
        strcpy(label_copy, label.c_str());
        char * rabo = strchr(label_copy, '\n');
        if(rabo != NULL)
            rabo[0] = ' ';
        CvFont font;
        cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX , 1.0f, 1.0f);

        ///This prints the label on top of the image, the color is not very legible
        cvPutText(matched_image, label_copy, left_up, &font, color );
//         cout << "Wrote in image:"<<label<<endl;
        switch (red_green_blue) {
            case 0:
                color = CV_RGB( 0, 255, 0 );
                red_green_blue = 1;
                break;
            case 1:
                color = CV_RGB( 0, 0, 255 );
                red_green_blue = 2;
                break;
            case 2:
                color = CV_RGB( 255, 0, 0 );
                red_green_blue = 0;
                break;
        }
        delete [] label_copy; ///'delete' ou 'delete []'
    }
    //cout<< "fim de "<<endl;


    draw_match_features(PLUS);

    return matched_image;
}

IplImage * Matching::get_matched_image(){
    return matched_image;
}

IplImage * Matching::get_original_image(){
    return image_to_match.image_color;
}

void Matching::draw_match_features(int draw_type){

    for( int i = matches.go_to_first(); i != -1; i = matches.go_to_next() ) {
        int x = (int) image_to_match.keypoints[matches[i].keypoint].x;
        int y = (int) image_to_match.keypoints[matches[i].keypoint].y;
        switch(draw_type){
            case RECTANGLE:
                cvRectangle(matched_image, cvPoint(x-1, y-1), cvPoint(x+1, y+1), CV_RGB( 255, 255, 0 ));
                break;
            case CROSS:
                cvLine(matched_image, cvPoint(x-2, y-2), cvPoint(x+2, y+2), CV_RGB( 0, 255, 255 ));
                cvLine(matched_image, cvPoint(x-2, y+2), cvPoint(x+2, y-2), CV_RGB( 0, 255, 255 ));
                break;
            case POINT:
                cvRectangle(matched_image, cvPoint(x, y), cvPoint(x, y), CV_RGB( 255, 0, 255 ));
                break;
            case PLUS:
                cvLine(matched_image, cvPoint(x-2, y), cvPoint(x+2, y), CV_RGB( 255, 0, 255 ));
                cvLine(matched_image, cvPoint(x, y+2), cvPoint(x, y-2), CV_RGB( 255, 0, 255 ));
        }
    }
}

void Matching::display_matching() {
    if( image_to_match.get_image_color() == 0 )
        return;
    //
    // 	CvScalar color = CV_RGB( 255, 0, 0 );
    // //	IplImage *image = cvCloneImage( image_to_match.get_image() );
    // 	IplImage *image = cvCreateImage( cvSize(image_to_match.get_image()->width,image_to_match.get_image()->height), IPL_DEPTH_8U,3);
    // 	cvConvertImage( image_to_match.get_image_color(), image ); //cvConvertImage( image_to_match.get_image(), image );
    //
    //
    // 	CvPoint left_up, left_down, right_up, right_down;
    // 	int x,y;
    //
    // 	for( int i = models.go_to_first(); i != -1; i = models.go_to_next() )
    // 	{
    // 		x = 0;
    // 		y = 0;
    // 		left_up.x = static_cast<int>(models[i].M[0][0]*x + models[i].M[0][1]*y + models[i].T[0][0]);
    // 		left_up.y = static_cast<int>(models[i].M[1][0]*x + models[i].M[1][1]*y + models[i].T[1][0]);
    //
    // 		x = 0;
    // 		y = training_samples[models[i].image].get_image()->height;
    // 		left_down.x = static_cast<int>(models[i].M[0][0]*x + models[i].M[0][1]*y + models[i].T[0][0]);
    // 		left_down.y = static_cast<int>(models[i].M[1][0]*x + models[i].M[1][1]*y + models[i].T[1][0]);
    //
    // 		x = training_samples[models[i].image].get_image()->width;
    // 		y = 0;
    // 		right_up.x = static_cast<int>(models[i].M[0][0]*x + models[i].M[0][1]*y + models[i].T[0][0]);
    // 		right_up.y = static_cast<int>(models[i].M[1][0]*x + models[i].M[1][1]*y + models[i].T[1][0]);
    //
    // 		x = training_samples[models[i].image].get_image()->width;
    // 		y = training_samples[models[i].image].get_image()->height;
    // 		right_down.x = static_cast<int>(models[i].M[0][0]*x + models[i].M[0][1]*y + models[i].T[0][0]);
    // 		right_down.y = static_cast<int>(models[i].M[1][0]*x + models[i].M[1][1]*y + models[i].T[1][0]);
    //
    // 		cvLine( image, left_up, left_down, color, 2 );
    // 		cvLine( image, left_down, right_down, color, 2 );
    // 		cvLine( image, right_down, right_up, color, 2 );
    // 		cvLine( image, right_up, left_up, color, 2 );
    // 	}

    IplImage *image = update_matched_image();
    cvNamedWindow( "Matches" );
    //	cvShowImage("Matches", image );
    cvShowImage("Matches", image);
    cvWaitKey(0);
    cvDestroyWindow( "Matches" );
    //	cvReleaseImage( &image );
}


inline void Matching::display_test_image() {
    image_to_match.show("Test Image");
}


void Matching::display_database_images() {
    for( int i = training_samples.go_to_first(); i != -1; i = training_samples.go_to_next() ) {
        if(training_samples[i].label.empty())
            training_samples[i].show("Database");
        else
            training_samples[i].show(training_samples[i].label);

    }
}


/*************************/

void Matching::insert_database_match( Match const &m) {
    if( matches.insert_record( m ) == -1 ) {
        MESSAGE("Resizing matches database...");
        matches.resize( 2*matches.get_capacity() );
        if( matches.insert_record( m ) == -1 ) {
            cerr << "Matching::insert_database_match: could not allocate memory. Quiting " << endl;
            exit(1);
        }
    }
}

int Matching::insert_database_image( SIFT_Image const &image) {
    int index = training_samples.insert_record( image );
    if(index == -1 ) {
        MESSAGE("Resizing image database...");
        training_samples.resize( 2*training_samples.get_capacity() );
        index = training_samples.insert_record( image );
        if( index == -1 ) {
            cerr << "Matching::insert_database_image: could not allocate memory. Quiting " << endl;
            exit(1);
        }
    }

    struct feature *aux;
    ///Set the feature's image indexs
    for(int i = 0; i < image.number_of_features; i++ ) {
        aux = image.features + i;

        aux->image_index = index;
    }

    ///Recreate the all-feature vector and kd-tree
    rebuild_kd_tree(image, index);

    // 	cout <<"image.features->x="<< image.features->x <<" y="<< image.features->y <<" scl="<< image.features->scl <<" ori="<< image.features->ori << endl; //DEBUG
    //
    // 	cout << "index=" << index << endl; //DEBUG
    //
    // 	cout <<"training_samples["<< index <<"].x="<< training_samples[index].features->x <<" y="<< training_samples[index].features->y <<" scl="<< training_samples[index].features->scl <<" ori="<< training_samples[index].features->ori << endl; //DEBUG
    //
    // 	cout << "training_samples[index].kd_image_root->n=" << training_samples[index].kd_image_root->n;
    // 	cout << " feature: "<<" x="<< training_samples[index].kd_image_root->features->x <<" y="<< training_samples[index].kd_image_root->features->y <<" scl="<< training_samples[index].kd_image_root->features->scl <<" ori="<< training_samples[index].kd_image_root->features->ori << endl; //DEBUG
    //
    // 	cout << "training_samples[index].kd_image_root->kd_left->n=" << training_samples[index].kd_image_root->kd_left->n << endl;//DEBUG
    // 	cout << " feat: "<<" x="<< training_samples[index].kd_image_root->kd_left->features->x <<" y="<< training_samples[index].kd_image_root->kd_left->features->y <<" scl="<< training_samples[index].kd_image_root->kd_left->features->scl <<" ori="<< training_samples[index].kd_image_root->kd_left->features->ori << endl; //DEBUG
    //
    // 	cout << "training_samples[index].kd_image_root->kd_right->n=" << training_samples[index].kd_image_root->kd_right->n << endl;//DEBUG
    // 	cout << " feat: "<<" x="<< training_samples[index].kd_image_root->kd_right->features->x <<" y="<< training_samples[index].kd_image_root->kd_right->features->y <<" scl="<< training_samples[index].kd_image_root->kd_right->features->scl <<" ori="<< training_samples[index].kd_image_root->kd_right->features->ori << endl; //DEBUG

    return index;
}

// int Matching::insert_database_image_and_return_index( SIFT_Image const &image) {
//     int index = training_samples.insert_record( image );
//     if(index == -1 ) {
//         MESSAGE("Resizing image database...");
//         training_samples.resize( 2*training_samples.get_capacity() );
//         if( training_samples.insert_record( image ) == -1 ) {
//             cerr << "Matching::insert_database_image: could not allocate memory. Quiting " << endl;
//             exit(1);
//         }
//     }
//     return index;
// }

void Matching::insert_model( Model const &model ) {
    if( models.insert_record( model ) == -1 ) {
        MESSAGE("Resizing models database...");
        models.resize( 2*models.get_capacity() );
        if( models.insert_record( model ) == -1 ) {
            cerr << "Matching::insert_model: could not allocate memory. Quiting " << endl;
            exit(1);
        }
    }
}



/*************************/


void Matching::match_to_database() {
//     cout<<"rabo?"<<endl;
    matches.reset(param.max_matches);
    hough_table.clear();
    hough_bins = priority_queue<HoughBin>();
    models.reset(param.max_models);

    double t_start_all = (double)cvGetTickCount();
    double t_start = (double)cvGetTickCount();

//     MESSAGE("Matching image to database...");
    get_matches();
//     MESSAGE("\t" << matches.get_size() << " match(es) found");
    
//     MESSAGE("Clustering matches...");
    cluster_matches();
    order_models();
//     MESSAGE("\t" << hough_bins.size() << " Hough bin(s) built");

//     MESSAGE("Refining models...");
    refine_models();
//     MESSAGE("\t" << models.get_size() << " model(s) found");

//     MESSAGE("Validating models...");
    validate_models();
    remove_models();
//     MESSAGE("\t" << models.get_size() << " model(s) kept after probabilistic validation");
    
    double t_end = (double)cvGetTickCount() - t_start_all;
    printf( "Matching: Matched image to database and found %d models in %gms\n", models.get_size(), t_end/(cvGetTickFrequency()*1000.) );

}



void Matching::get_matches() {
    float s_ratio = SQUARE(param.distance_ratio);

    Match nearest, nearest_feat;
    float d, nearest_d, nearest2_d;
    double d_feat= DBL_MAX, nearest_feat_d, nearest_feat2_d;

    Database<Keypoint> &image_kps = image_to_match.get_keypoints();

    int image_kp, dbase_img, dbase_kp, dbase_feat_kp;

    time_t start, end;
    double dif=0;

    //Histograma de Match/No Match
    //	Keypoint kp_mais_prox;
    // 	FILE * fpFile;
    // 	fpFile = fopen ("compare/feat.txt","a");
    // 	FILE * kpFile;
    // 	kpFile = fopen ("compare/keyp.txt","a");
    // 	if (fpFile==NULL || kpFile==NULL)
    // 	{
    // 		MESSAGE("Not able to create/overwrite feat.txt or keyp.txt");
    // 		//mkdir database; try again;
    // 		return;
    // 	}

    //--------Rob Hess way
    struct feature* feat; //features of the pics in the database
    struct feature** nbrs = NULL;
    struct feature* image_feats_root = image_to_match.features; //root to the features "vector" of the image to be matched
    struct feature* image_feat;		//features in the image just loaded to be matched
    int n = image_to_match.number_of_features;

    struct kd_node* kd_root;  //root to the kd_tree of an pic in the database



    double d0, d1;
    int k, i, rabo;
    //--------Rob Hess way
    // 			image_feats = training_samples[dbase_img].features;
    //
    // 			cout <<"dbase_img:"<< dbase_img <<"x>="<< image_feats->x <<" y="<< image_feats->y <<" scl="<< image_feats->scl <<" ori="<< image_feats->ori << endl; //DEBUG
    //
    // 			n = training_samples[dbase_img].number_of_features;
    // 						cout << "n=" << n << " exemplo: " << image_feats->x << endl;//DEBUG
    // 			kd_root = training_samples[dbase_img].kd_image_root;
    double t_start = (double)cvGetTickCount();
    for(i = 0; i < n; i++ ) //for all features in image_to_match
    {image_feat = image_feats_root + i;
        nearest.keypoint = image_feat->keypoint_index;
//         nearest.keypoint = image_to_match.keypoint_indexes[i];
//         cout<<"one:"<<image_feat->keypoint_index<<" two:"<<image_to_match.keypoint_indexes[i]<<endl;

//         nearest_feat_d = DBL_MAX;
//         nearest_feat2_d = DBL_MAX;

        if (image_feat == NULL) {
            cout << "image_feat "<< i <<" == NULL"<< endl;
            break;
        } else{
            ///correct way, one single kd-tree for all features, still in tests
            k = kdtree_bbf_knn( this->kd_image_root_all, image_feat, 2, &nbrs, KDTREE_BBF_MAX_NN_CHKS );

            if( k == 2 ) {
                if( descr_dist_sq( image_feat, nbrs[0] ) < descr_dist_sq( image_feat, nbrs[1] ) * NN_SQ_DIST_RATIO_THR ) {
//                     nearest.training_image = all_features[nbrs[0]-all_features].image_index;
//                     nearest.training_keypoint = all_features[nbrs[0]-all_features].keypoint_index;
                    //cout<<descr_dist_sq( image_feat, nbrs[0] )<<endl;
                    
                    nearest.training_image = nbrs[0]->image_index;
                    nearest.training_keypoint = nbrs[0]->keypoint_index;

/*                    nearest.training_image = image_indexes[nbrs[0]-all_features];
                    nearest.training_keypoint = keypoint_indexes[nbrs[0]-all_features];*/
//                     cout << "index1:"<<nbrs[0]->keypoint_index;
//                     cout << "\t index2 ="<<all_features[nbrs[0]-all_features].keypoint_index;
//                     cout <<" \t index3:"<<keypoint_indexes[nbrs[0]-all_features];
//                     cout << "\t image1:"<<nbrs[0]->image_index;
//                     cout << "\t image2="<<all_features[nbrs[0]-all_features].image_index;
//                     cout <<"\t image3:"<<image_indexes[nbrs[0]-all_features]<<endl;

                    insert_database_match( nearest );
/*                    cout << "index on feature vector? ="<<all_features[nbrs[0]-all_features].keypoint_index<<" index:"<<nbrs[0]->keypoint_index<<endl;*/
//                     cout << "image index? ="<<all_features[nbrs[0]-all_features].image_index<<" index:"<<nbrs[0]->image_index<<endl;
//                     cout <<"begin:"<<*all_features<<" end:"<<*(all_features + number_of_all_features) <<endl;
//                     cout << "test1:d_feat=" << descr_dist_sq( image_feat, nbrs[0] )<<"image_index img["<<nearest.training_image<<"], " "keyp["<<nearest.training_keypoint<<"]"<<endl; //DEBUG
                }
            }
            else if( k == -1){
                cout << "Empty Database."<<endl;
                free( nbrs );
                return;
            }
            
            free( nbrs );

            //wrong way, one kd-tree per image.
/*            for( dbase_img = training_samples.go_to_first(); dbase_img != -1; dbase_img = training_samples.go_to_next() ) {

                kd_root = training_samples[dbase_img].kd_image_root;
                if(kd_root == NULL) {
                    cout << "kd_root of dbase_img"<< dbase_img <<"== NULL"<< endl;
                    continue;
                }

                k = kdtree_bbf_knn( kd_root, image_feat, 2, &nbrs, KDTREE_BBF_MAX_NN_CHKS );

                // 			cout << " k=" << k << endl; //DEBUG

                if (k == -1) {
                    break;
                }
                if( k == 2 ) {
                    for(rabo = 0; rabo < k; rabo++) {
                        dbase_feat_kp = nbrs[rabo]->keypoint_index;
                        d_feat = descr_dist_sq( image_feat, nbrs[rabo] );

                        if( d_feat < nearest_feat2_d )
                            nearest_feat2_d = d_feat;
                        if( d_feat < nearest_feat_d ) {
                            nearest_feat2_d = nearest_feat_d;
                            nearest_feat_d = d_feat;

                            nearest_feat.training_image = dbase_img;
                            nearest_feat.training_keypoint = dbase_feat_kp;
                        }
                    }
                }
                free( nbrs );

/*                for(int iter=0; iter<k; iter++){
                    if(nbrs[i]->feature_data != NULL){
                        free(nbrs[i]->feature_data);
                        free(nbrs[i]);
                    }
                    free(nbrs);
                }*/
//             }*/
        // Is it a good match?
//         }
//         if( nearest_feat_d < nearest_feat2_d * NN_SQ_DIST_RATIO_THR ) {
//             // 			cout << "image_feat="<< i <<" Feat: " << nearest_feat.training_image << " " << nearest_feat.training_keypoint << endl; //DEBUG
//             // 			fprintf(fpFile, "%d %d %d\n", i, nearest_feat.training_image, nearest_feat.training_keypoint);
//             cout << "test2:d_feat=" << nearest_feat_d<<" img["<<nearest_feat.training_image<<"], " "keyp["<<nearest_feat.training_keypoint<<"]"<< endl; //DEBUG
// //             insert_database_match( nearest_feat );
        }

    }
    double t_end = (double)cvGetTickCount() - t_start;
    //     printf("MATCHING:: finding neighbors with features= %gms\n", t_end/(cvGetTickFrequency()*1000.) );
}

void Matching::get_matches_BrunoDamas()
{
    float s_ratio = SQUARE(param.distance_ratio);
    
    Match nearest;
    float d, nearest_d, nearest2_d;
    
    Database<Keypoint> &image_kps = image_to_match.get_keypoints();
    
    int image_kp, dbase_img, dbase_kp;
    
    for( image_kp = image_kps.go_to_first();  image_kp != -1; image_kp = image_kps.go_to_next() )
    {
        nearest.keypoint = image_kp;
        
        nearest_d = INF_PLUS;
        nearest2_d = INF_PLUS;
        
        for( dbase_img = training_samples.go_to_first(); dbase_img != -1; dbase_img = training_samples.go_to_next() )
        {
            Database<Keypoint> &dbase_kps = training_samples[dbase_img].get_keypoints();
            for( dbase_kp = dbase_kps.go_to_first(); dbase_kp != -1; dbase_kp = dbase_kps.go_to_next() )
            {
                d = image_kps[image_kp].square_distance( dbase_kps[dbase_kp] );

                if( d < nearest2_d )
                    nearest2_d = d;
                if( d < nearest_d )
                {
                    nearest2_d = nearest_d;
                    nearest_d = d;
                    
                    nearest.training_image = dbase_img;
                    nearest.training_keypoint = dbase_kp;
                }
            }
        }

        // Is it a good match?
        if( nearest_d < nearest2_d * s_ratio )
            insert_database_match( nearest );
    }
}


void Matching::cluster_matches()
{
    char buffer[100];
    float dx_image, dy_image, x_max, y_max;
    float cos_th, sin_th;
    int bin_th[2], bin_s[2], bin_x[2], bin_y[2];
    int n_bins_th = static_cast<int>( 360.0 / param.bin_theta_size );
    float log2_scale_size = log2(param.bin_scale_size);

    for( int i = matches.go_to_first(); i != -1; i = matches.go_to_next() )
    {
        SIFT_Image &db_image = training_samples[matches[i].training_image];
        Keypoint const &kp_image = image_to_match.get_keypoints()[matches[i].keypoint];
        Keypoint const &kp_db = db_image.get_keypoints()[matches[i].training_keypoint];
        
        // Obtaining database image position, orientation and scale relative to test image
        matches[i].s = kp_image.s / kp_db.s;
        
        matches[i].th = CLIP_DEG( kp_image.th - kp_db.th ); // -180 <= theta_image <= 180
        if( matches[i].th < 0 ) matches[i].th += 360.0; // 0 <= theta_image < 360
        
        dx_image = matches[i].s * kp_db.x;
        dy_image = matches[i].s * kp_db.y;
        cos_th = cos( matches[i].th * M_PI/180 );
        sin_th = sin( matches[i].th * M_PI/180 );
        
        matches[i].x = kp_image.x - ( cos_th * dx_image - sin_th * dy_image );
        matches[i].y = kp_image.y - ( sin_th * dx_image + cos_th * dy_image );
    
        // Predicted image size
        x_max = matches[i].s * db_image.get_image()->width;
        y_max = matches[i].s * db_image.get_image()->height;
        
        // Assigning bins
        //
        //      Scale
        bin_s[0] = static_cast<int>( cvFloor( log2(matches[i].s)/log2_scale_size ) );
        bin_s[1] = bin_s[0] + 1;
        
        //      Theta
        bin_th[0] = static_cast<int>( cvFloor( matches[i].th / param.bin_theta_size ) );
        if( bin_th[0] < n_bins_th-1 )
            bin_th[1] = bin_th[0] + 1;
        else    //Circular bins...
            bin_th[1] = 0;
        
        //      Pos
        bin_x[0] = static_cast<int>( cvFloor( matches[i].x / (x_max * param.bin_pos_size) ) );
        bin_x[1] = bin_x[0] + 1;
        bin_y[0] = static_cast<int>( cvFloor( matches[i].y / (y_max * param.bin_pos_size) ) );
        bin_y[1] = bin_y[0] + 1;
        

//cout << "Database:\tth = " << kp_db.th << ", scale = " << kp_db.s << ", pos = [" << kp_db.x << "," << kp_db.y << "]\n";
//cout << "Image:\tth = " << kp_image.th << ", scale = " << kp_image.s << ", pos = [" << kp_image.x << "," << kp_image.y << "]\n";
//cout << "IMAGE_POS:\tth = " << matches[i].th << ", scale = " << matches[i].s << ", pos = [" << matches[i].x << "," << matches[i].y << "]\n";
//cout << "IMAGE_BIN:\tth = " << bin_th[0] << ", scale = " << bin_s[0] << ", pos = [" << bin_x[0] << "," << bin_y[0] << "]\n";

        
        // Filling hash_table
        int j,k,l,m;
        for( m = 0; m < 2; m++)
            for( j = 0; j < 2; j++)
                for( k = 0; k < 2; k++)
                    for( l = 0; l < 2; l++)
        {
            snprintf(buffer,100,"%+d%+d%+d%+d%+d",matches[i].training_image, bin_s[m], bin_th[j], bin_x[k], bin_y[l]);
            hough_table.insert( pair<string, Match *>(string(buffer), &matches[i]) );
        }
    }
}


void Matching::order_models()
{
    HoughBin bin;
    
    for( bin.first = hough_table.begin(); bin.first != hough_table.end(); bin.first = bin.last )
    {
        bin.count = hough_table.count(bin.first->first);
        bin.last = hough_table.equal_range(bin.first->first).second;
        
        if( bin.count >= param.min_models_per_bin )
            hough_bins.push(bin);
    }
}


void Matching::refine_models()
{
    Model model;
    CvMat A,B,affine_tr;
    float matA[2*matches.get_size()][6];
    float matB[2*matches.get_size()][1];
    float matX[6][1];
    cvInitMatHeader( &affine_tr, 6, 1, CV_32FC1, matX );
    
    HoughBin bin;
    StringMultiHash<Match *>::iterator it;
    int i, count, n_model;
    bool outliers_found;



    for( n_model = 0, bin = hough_bins.top(); !hough_bins.empty(); hough_bins.pop(), bin = hough_bins.top() )
    {
        // This is the image the model refers to
        model.image = bin.first->second->training_image;
        
//cout << "count was " << bin.count;
        for( i = 0, it = bin.first, count = bin.count; it != bin.last && count >= param.min_models_per_bin; it++ )
        {
            if( it->second->model == -1 || it->second->model == n_model)        //Not assigned to a previous model
            {
                it->second->model = n_model;
                fill_matrices(i, *it->second, matA, matB );
                i++;
            }
            else
                count--;
        }
//cout << ", after removing assigned matches is " << count << endl;
        
        if( count < 3 )     //Discard this bin
            continue;

        do
        {
            //Solving for affine transform parameters
            cvInitMatHeader( &A, 2*count, 6, CV_32FC1, matA );
            cvInitMatHeader( &B, 2*count, 1, CV_32FC1, matB );
            cvSolve( &A, &B, &affine_tr, CV_SVD );
//cout << matX[0][0] << "\t" << matX[1][0] << "\t\t" << matX[4][0] << endl;
//cout << matX[2][0] << "\t" << matX[3][0] << "\t\t" << matX[5][0] << endl;

            //Discarding outliers
            outliers_found = false;
            for( i = 0, it = bin.first; it != bin.last && count >= param.min_models_per_bin; it++ )
                if( it->second->model == n_model)
                    if( agree( *it->second, matX ) )
            {
                fill_matrices(i, *it->second, matA, matB );
                i++;
            }
            else
            {
                it->second->model = -1;
                count--;
                outliers_found = true;
            }
        }
        while( outliers_found == true && count >= param.min_models_per_bin);
        
//cout << "And after removing outliers is " << count << endl;
        if( count < 3 )     //Discard this bin
            continue;


        //Gathering and binding other matches
        count = 0;
        for( i = matches.go_to_first(); i != -1; i = matches.go_to_next() )
            if( (matches[i].model == n_model || matches[i].model == -1) &&
                 ( matches[i].training_image == model.image )) //Must be the same image...
                if( agree( matches[i], matX ) )
        {
            matches[i].model = n_model;
            count++;
        }
        else
            matches[i].model = -1;
                
//cout << "And after looking other matches is " << count << endl;

        //Update models and proceed to next bin
        model.id = n_model;
        
        model.M[0][0] = matX[0][0];
        model.M[0][1] = matX[1][0];
        model.M[1][0] = matX[2][0];
        model.M[1][1] = matX[3][0];
        model.T[0][0] = matX[4][0];
        model.T[1][0] = matX[5][0];
        model.matches = count;
        insert_model( model );
        
        n_model++;  
    }
}


void Matching::validate_models()
{
    int i, n, k, k_reject;
    double d, l, r, s;
    double p_no_model, p_model;
    
    int total_keypoints = 0;
    for( i = training_samples.go_to_first(); i != -1; i = training_samples.go_to_next() )
        total_keypoints += training_samples[i].get_keypoints().get_size();

    for( i = models.go_to_first(); i != -1; i = models.go_to_next() )
    {
        //Number of image features within the projected outline of the model view
        n = image_range_features( image_to_match, models[i] ) - 1;
//cout << "n: " << n << endl;
        //Number of feature matches (one match was removed to build hypothesis)
        k = models[i].matches - 1;
//cout << "K: " << k << endl;       
        //Probability of randomly matching to a specific model image 
        d = static_cast<float>(training_samples[models[i].image].get_keypoints().get_size()) / total_keypoints;
        //Probability of randomly satisfying the location, scale and orientation constraints, given the image
        l = param.bin_pos_size;
        r = param.bin_theta_size / 360.0;
        s = param.scale_probability;
        
        //Probability of randomly having a match within the projected outline of the model view
        p_no_model = param.false_match*d*l*l*r*s;
//cout << "p_not_m: " << p_no_model << endl;
//p_no_model = 0.001;
//MESSAGE("HACKKK!!");
        
        //Probability of randomly having a match within the projected outline of the model view
        p_model = param.correct_match;
//cout << "p_m: " << p_model << endl;       

        //Probability of matching at least k features to the model, given that the model is NOT present
        //P-Value
        models[i].p_value = 1 - cumulative_binomial(n,p_no_model,k);

        //Critical region       
        k_reject = critical_region(n,p_no_model,param.significance);
//cout << "RC = " << k_reject << endl;      
        // Power of the test, 1-beta
        models[i].sensitivity = 1 - cumulative_binomial(n,p_model,k_reject);
        
        // Power of the test with half occlusion
        models[i].sensitivity2 = 1 - cumulative_binomial(n,p_model/2.0,k_reject);
        
        // Power of the test with 3/4 occlusion
        models[i].sensitivity4 = 1 - cumulative_binomial(n,p_model/4.0,k_reject);
        
        // Power of the test with 3/4 occlusion
        models[i].sensitivity10 = 1 - cumulative_binomial(n,p_model/10.0,k_reject);
        
//         MESSAGE("\tModel " << i << " was " << ((models[i].p_value < param.significance) ? "validated" : "rejected") <<
//                 " for a significance level of " << param.significance );
//         MESSAGE("\t\tp-value: " << models[i].p_value);
//         MESSAGE("\t\tsensitivity: " << models[i].sensitivity << "\t(no occlusion)");
//         MESSAGE("\t\t             " << models[i].sensitivity2 << "\t(50% occlusion)");
//         MESSAGE("\t\t             " << models[i].sensitivity4 << "\t(75% occlusion)");
//         MESSAGE("\t\t             " << models[i].sensitivity10 << "\t(90% occlusion)");
    }
}


void Matching::remove_models()
{
    for( int i = models.go_to_last(); i != -1; i = models.go_to_previous() )
        if( models[i].p_value > param.significance )
            models.remove_record(i);
}




double Matching::cumulative_binomial( int n, double p, int k )
{
    if( k >= n )
        return 1.0;
    else if( k < 0 )
        return 0.0;
        
    double val = pow(1-p,n);
    double c = val;

    for(int i = 1; i <= k; i++)
    {
        val *= static_cast<double>(n+1-i)/static_cast<double>(i) * p/(1-p);
        c += val;
    }
    
    return ((c > 1.0) ? 1.0 : c);
}


int Matching::critical_region( int n, double p, double significance )
{
    double val = pow(1-p,n);
    double c = val;
    
    if( c >= (1.0 - significance) )
        return 0;
    
    int k;
    for(k = 1; k <= n; k++)
    {
        val *= static_cast<double>(n+1-k)/static_cast<double>(k) * p/(1-p);
        c += val;
        if( c >= 1.0 - significance )
            return k;
    }
    
    return  n;
}


void Matching::fill_matrices( int i, Match const &match, float matA[][6], float matB[][1] )
{
    Keypoint const &kp_image = image_to_match.get_keypoints()[match.keypoint];
    Keypoint const &kp_db = training_samples[match.training_image].get_keypoints()[match.training_keypoint];

    matA[2*i][0] = matA[2*i+1][2] = kp_db.x;
    matA[2*i][1] = matA[2*i+1][3] = kp_db.y;
    matA[2*i][4] = matA[2*i+1][5] = 1.0;
    matA[2*i][2] = matA[2*i+1][0] = matA[2*i][3] = matA[2*i+1][1] = matA[2*i][5] = matA[2*i+1][4] = 0.0;
    
    matB[0][2*i] = kp_image.x; 
    matB[0][2*i+1] = kp_image.y;
}


bool Matching::agree( Match const &match, float const X[][1] )
{
    float u,v, scale, orientation;
    
    Keypoint const &kp_image = image_to_match.get_keypoints()[match.keypoint];
    Keypoint const &kp_db = training_samples[match.training_image].get_keypoints()[match.training_keypoint];

    float x_max = training_samples[match.training_image].get_image()->width;
    float y_max = training_samples[match.training_image].get_image()->height;

    // Predicted location, scale and orientation
    u = X[0][0] * kp_db.x + X[1][0] * kp_db.y + X[4][0];
    v = X[2][0] * kp_db.x + X[3][0] * kp_db.y + X[5][0];
    
    scale = sqrt(X[0][0] * X[3][0] - X[1][0] * X[2][0]) * kp_db.s;

    float cos_th = cos(M_PI/180.0*kp_db.th);
    float sin_th = sin(M_PI/180.0*kp_db.th);
    orientation = 180.0/M_PI * atan2( X[2][0]*cos_th + X[3][0]*sin_th, X[0][0]*cos_th + X[1][0]*sin_th );

//cout << "Predicted: [" << u << "," << v << "],\t scale = " << scale << ",\ttheta = " << orientation << endl;  
//cout << "Real     : [" << kp_image.x << "," << kp_image.y << "],\t scale = " << kp_image.s << ",\ttheta = " << kp_image.th << endl;   
//getchar();
    //Comparing predicted location with real location
    return(
           ( fabs(log2( scale/kp_image.s )) <= log2(param.bin_scale_size)/2.0 ) &&
            ( fabs(CLIP_DEG( orientation - kp_image.th )) <= param.bin_theta_size/2.0 ) &&
            ( fabs(u - kp_image.x) <= x_max * scale * param.bin_pos_size/2.0 ) &&
            ( fabs(v - kp_image.y) <= y_max * scale * param.bin_pos_size/2.0 ) );
}


int Matching::image_range_features( SIFT_Image &img, Model const &model )
{
    float det = model.M[0][0] * model.M[1][1] - model.M[1][0] * model.M[0][1];
    float x,y,dx,dy;
    float width = img.get_image()->width;
    float height = img.get_image()->height;
    int count = 0;
    
    for( int i = img.get_keypoints().go_to_first(); i != -1; i = img.get_keypoints().go_to_next() )
    {
        dx = img.get_keypoints()[i].x - model.T[0][0];
        dy = img.get_keypoints()[i].y - model.T[1][0];
        
        x = ( model.M[1][1] * dx - model.M[0][1] * dy) / det;
        y = (-model.M[1][0] * dx + model.M[0][0] * dy) / det;
        
        if( (x >= -param.bin_pos_size/2.0*height) && (x <= (1.0 + param.bin_pos_size/2.0)*width) &&
             (y >= -param.bin_pos_size/2.0*height) && (y <= (1.0 + param.bin_pos_size/2.0)*height) )
            count++;
    }

    return count;   
}
