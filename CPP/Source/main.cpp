/*
 * mainCPP.cpp
 *
 *  Created on: Jun 16, 2023
 *      Author: Felipe
 */
#include "main.h"

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "icp.hpp"
#include <iostream>
#include <String>
#include <math.h>

#define MAX_IT 30
#define SCAN_SIZE 360

double PI = 3.1415;


Eigen::Matrix<double, 360, 2> scan_to_cartesian(float* scan_ranges, int size){
//	double coordinates[360][2], ang;
	double ang;
	//double coord[720];
	Eigen::Matrix<double, 360, 2> out;


	for ( int i=0 ; i< size ; i++){
		ang = i * PI * (0.0026315789);    // convertion to radians
//		coordinates[i][0] =  scan_ranges[i] * cos(ang);
//		coordinates[i][1] =  scan_ranges[i] * sin(ang);
		out(i,0) = double(scan_ranges[i] * cos(ang));
		out(i,1) = double(scan_ranges[i] * sin(ang));
	}

	return out;

}

Eigen::Matrix<double, 120, 2> drop_points_120(Eigen::Matrix<double, 360, 2> scan_2d){

	Eigen::Matrix<double, 120, 2> out;
	int idx_out = 0;

	for(int i =2; i < 360; i+=3){
		if( scan_2d(i,0) == 0.  || scan_2d(i,1) == 0.){
			for(int j = i; j < 360; j++){
				if( scan_2d(j,0) != 0. && scan_2d(j,1) != 0.){
					out(idx_out,0) = scan_2d(j, 0);
					out(idx_out,1) = scan_2d(j, 1);
					idx_out++;
					j=361;
				}
			}
		}else{
			out(idx_out,0) = scan_2d(i, 0);
			out(idx_out,1) = scan_2d(i, 1);
			idx_out++;
		}
//		if(idx_out == 119){
//			idx_out--;
//		}
	}

	return out;
}

void mainCPP(float* src_scan, float* tgt_scan){

	double error=999999., n_error=999999., rate=0.;
	Eigen::Matrix<double, 360, 2> 		source_ptcl, target_ptcl;// temp_ptcl;
	Eigen::Matrix<double, 120, 2> 		source_ptcl_d120, target_ptcl_d120, temp_ptcl_d120;
	Eigen::Matrix<double, 1  , 3>		transform, icp_transform, cumulative, final_transform;
	Eigen::Matrix<double, 1  , 2>		centroid_a, centroid_b;
	Eigen::Matrix<int   , 120, 2>		CORR;
	Eigen::Matrix<double, 3  , 3>		HOM_TRANSF_M;


	transform.setZero();
	cumulative.setZero();


	source_ptcl = scan_to_cartesian(src_scan, SCAN_SIZE);
	target_ptcl = scan_to_cartesian(tgt_scan, SCAN_SIZE);

	source_ptcl_d120 = drop_points_120(source_ptcl);
	target_ptcl_d120 = drop_points_120(target_ptcl);

	// first CENTER OF MASS CALC
	transform.block<1,2>(0,0) = CenterOfMass(target_ptcl_d120) - CenterOfMass(source_ptcl_d120 );

	temp_ptcl_d120 = computeTransform(source_ptcl_d120, transform);

	for ( int i =0; i < MAX_IT ; i++){
		CORR = FindCorrenpondences_PtP(temp_ptcl_d120, target_ptcl_d120);
		error = getError(temp_ptcl_d120, target_ptcl_d120, CORR);

		icp_transform = ICP(temp_ptcl_d120, target_ptcl_d120, 100, 0.1);
		temp_ptcl_d120 = computeTransform(temp_ptcl_d120, icp_transform);
		cumulative += icp_transform;

//		CORR = FindCorrenpondences_PtP(temp_ptcl_d120, target_ptcl_d120);
//		n_error = getError(temp_ptcl_d120, target_ptcl_d120, CORR);

//		rate = error - n_error;
	}


	final_transform << 		0	, 0  , cumulative(0,2);
	temp_ptcl_d120 = computeTransform(source_ptcl_d120, final_transform);
	final_transform.block<1,2>(0,0) = CenterOfMass(target_ptcl_d120) - CenterOfMass(temp_ptcl_d120);

	rate = 0;
//	while(1){
//
//
//	}
}

float* ICP_main_process_(float* src_scan, float* tgt_scan, float* out){
//	float out[3];
//	memset(out, 0, 3);
	double error=999999., n_error=999999., rate=0.;
	Eigen::Matrix<double, 360, 2> 		source_ptcl, target_ptcl;// temp_ptcl;
	Eigen::Matrix<double, 120, 2> 		source_ptcl_d120, target_ptcl_d120, temp_ptcl_d120;
	Eigen::Matrix<double, 1  , 3>		transform, icp_transform, cumulative, final_transform;
	Eigen::Matrix<double, 1  , 2>		centroid_a, centroid_b;
	Eigen::Matrix<int   , 120, 2>		CORR;
	Eigen::Matrix<double, 3  , 3>		HOM_TRANSF_M;


	transform.setZero();
	cumulative.setZero();


	source_ptcl = scan_to_cartesian(src_scan, SCAN_SIZE);
	target_ptcl = scan_to_cartesian(tgt_scan, SCAN_SIZE);

	source_ptcl_d120 = drop_points_120(source_ptcl);
	target_ptcl_d120 = drop_points_120(target_ptcl);

	// first CENTER OF MASS CALC
	transform.block<1,2>(0,0) = CenterOfMass(target_ptcl_d120) - CenterOfMass(source_ptcl_d120);

	temp_ptcl_d120 = computeTransform(source_ptcl_d120, transform);

	for ( int i =0; i < MAX_IT ; i++){
		CORR = FindCorrenpondences_PtP(temp_ptcl_d120, target_ptcl_d120);
		error = getError(temp_ptcl_d120, target_ptcl_d120, CORR);

		icp_transform = ICP(temp_ptcl_d120, target_ptcl_d120, 100, 0.1);
		temp_ptcl_d120 = computeTransform(temp_ptcl_d120, icp_transform);
		cumulative += icp_transform;

		//CORR = FindCorrenpondences_PtP(temp_ptcl_d120, target_ptcl_d120);
		n_error = getError(temp_ptcl_d120, target_ptcl_d120, CORR);

		rate = error - n_error;
	}


	final_transform << 		0	, 0  , cumulative(0,2);
	temp_ptcl_d120 = computeTransform(source_ptcl_d120, final_transform);
	final_transform.block<1,2>(0,0) = CenterOfMass(target_ptcl_d120) - CenterOfMass(temp_ptcl_d120);

	rate = 0;

	for (int i =0 ; i< 3; i++){
		out[i] = final_transform(0,i);
	}
	return (float*)out;
}

