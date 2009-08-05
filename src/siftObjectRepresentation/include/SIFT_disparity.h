// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Authors: 2008 Dario Figueira
 *
 *      Institute for Systems and Robotics (IST)
 *      IST, TULisbon, Portugal
 */


#ifndef SIFT_DISPARIY_HPP
#define SIFT_DISPARIY_HPP

#include "highgui.h"
#include "imgfeatures.h"
#include "sift.hpp"
#include "kdtree.h"

#include <string>
// using namespace std;


/** Testes empíricos apontam que para uma altura de 240 uma disparidade de 15 é um bom limite
        pra altura de 480, uma disparidade de 30 é um bom limite para eleminação de outliers
        Limite = Height/16;
    Calcular a média das disparidades Horizontais e verificar se é positiva ou negativa indica
        qual é a imagem esquerda ou direita. Se a média for negativa, valores de disparidade
        horizontal positivos são descartados, e vice-versa.
    
    Objecto próximo o suficiente: Uma disparidade horizontal maior que 100 para largura 640
        Horizontal Limit = Width/6.4

    
*/

class SIFT_disparity {
private:

public:
    struct Disparity_Match {
        int right_keypoint;
        int left_keypoint;
        int left_feat;

        float s;
        float th;
        float x;
        float y;

        double disparity[2]; //x, y; left-right
    };

    Database<Disparity_Match> d_matches;
    // FILE * fpFile;//DEBUG
    float HorLim;
    float VertLim;
    int MaxHeight;
    int MinHeight;
    int MaxWidth;
    int MinWidth;
    SIFT_Image left;
    SIFT_Image right;

    SIFT_Image close_obj;
    bool there_is_a_close_object;
    float average_disparity_of_object;
    int total_matches;
    int outliers;

    SIFT_disparity();
    SIFT_disparity(IplImage const *limg, IplImage const *rimg);
    SIFT_disparity(string const &left , string const &right);
    ~SIFT_disparity();
    void insert_database_disparity_match( Disparity_Match const &m);
    void go(IplImage const *limg, IplImage const *rimg);
    bool go( string const &left , string const &right);
    void go(SIFT_Image sift_limg, SIFT_Image sift_rimg);
    void go();
    void create_close_object_SIFT_Image();
    void draw_and_show_d_matches(int draw_type);

};

#endif //SIFT_DISPARIY_HPP
