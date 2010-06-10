// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino (VisLab/ISR/IST)
 */

#include <math.h>


void compute_sp_params(int input_lines, int input_cols,
                       int output_lines, int output_cols,
                       double fx, double fy,
                       double cx, double cy,
                       double k1, double k2,
                       double p1, double p2, 
                       double *fa, double *fe,
                       double *ca, double *ce);

bool check_sp_params(int input_lines, int input_cols,
                     int output_lines, int output_cols,
                     double fx, double fy,
                     double cx, double cy,
                     double k1, double k2,
                     double p1, double p2,
                     float *mapx, float *mapy);

bool compute_sp_map(int input_lines, int input_cols,
                    int output_lines, int output_cols,
                    double fx, double fy,
                    double cx, double cy,
                    double k1, double k2,
                    double p1, double p2,
                    float *mapx, float *mapy);

bool compute_egosp_map( int input_lines, int input_cols,
                        int output_lines, int output_cols,
                        double fa, double fe,
                        double ca, double ce,
                        double *R, //world to camera rotation
                        float *mapx, float *mapy);

bool compute_icub_egosp_map( int input_lines, int input_cols,
                        int output_lines, int output_cols,
                        double fx, double fy,
                        double cx, double cy,
                        double *R, //world to camera rotation
                        float *mapx, float *mapy);
