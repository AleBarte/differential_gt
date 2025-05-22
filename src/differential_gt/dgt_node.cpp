
#include "../include/differential_gt/ncgt.hpp"
#include "../include/differential_gt/cgt.hpp"
#include "../include/differential_gt/arbitration.hpp"
#include <math.h>

std::vector<double> range(double min, double max, double dt) {
    std::vector<double> range;
    for(int i=0; i<max/dt; i++) {
        range.push_back(min + i*dt);
    }
    return range;
}


int main(int argc, char **argv)
{
     //* When increasing the number of dofs remember: order_of_sys = 2 * n_dofs, first n_dof elements are positions and the last n orientations
     int n_dofs=2;
     double dt = 0.01;
  
     std::vector<double> time = range(0.0,10.0,dt);
  
  
     Eigen::MatrixXd ref_h; ref_h.resize(n_dofs, time.size());
     Eigen::MatrixXd ref_r; ref_r.resize(n_dofs, time.size());

     ref_h.setZero();
     ref_r.setZero();
  
     for (int i = 0;i<time.size();i++)
     {
          ref_h.col(i) << 1, 0;//std::sin(time[i]);
          ref_r.col(i) << 1, 0;//std::sin(time[i]);
     }

     std::cout<<"ref_h: \n"<<ref_h.col(0)<<std::endl;
     std::cout<<"ref_r: \n"<<ref_r.col(0)<<std::endl;

  
     Eigen::MatrixXd Ac;
     Eigen::MatrixXd Bc;
     Eigen::MatrixXd Cc;
     Ac.resize(2*n_dofs,2*n_dofs);
     Bc.resize(2*n_dofs,n_dofs);
     Cc.resize(n_dofs,2*n_dofs);

     double m,c,k;

     m=10;
     k=0;
     c=25;
     
     Eigen::MatrixXd M = m * Eigen::MatrixXd::Identity(n_dofs,n_dofs);
     Eigen::MatrixXd C = c * Eigen::MatrixXd::Identity(n_dofs,n_dofs);
     Eigen::MatrixXd K = k * Eigen::MatrixXd::Identity(n_dofs,n_dofs);

     Ac.setZero();
     Bc.setZero();
     Cc.setZero();

     Ac.block(0, n_dofs, n_dofs, n_dofs) = Eigen::MatrixXd::Identity(n_dofs,n_dofs);
     Ac.block(n_dofs, 0, n_dofs, n_dofs) = -M.inverse() * K;
     Ac.block(n_dofs, n_dofs, n_dofs, n_dofs) = -M.inverse() * C;

     Bc.block(n_dofs, 0, n_dofs, n_dofs) = M.inverse();
     
     // Ac << 0, 1,
     //       -k/m, -c/m;

     // Bc << 0,
     //       1/m;

     // Cc << 1, 0;

     NonCoopGT ncgt(n_dofs,dt);
     ncgt.setSysParams(Ac,Bc);

     CoopGT cgt(n_dofs,dt);
     cgt.setSysParams(Ac,Bc);

     Eigen::VectorXd X=Eigen::VectorXd::Zero(2*n_dofs);
     Eigen::VectorXd dX=Eigen::VectorXd::Zero(2*n_dofs);    
     cgt.setCurrentState(X);
     ncgt.setCurrentState(X);

     // Eigen::MatrixXd Qhh; Qhh.resize(2 * n_dofs, 2 * n_dofs); 
     // Eigen::MatrixXd Qhr; Qhr.resize(2 * n_dofs, 2 * n_dofs);
     // Eigen::MatrixXd Qrr; Qrr.resize(2 * n_dofs, 2 * n_dofs);
     // Eigen::MatrixXd Qrh; Qrh.resize(2 * n_dofs, 2 * n_dofs); 

     // Qhh <<1,0,
     //      0,0.0001;
     // Qhr <<0,0,
     //      0,0.0001;

     // Qrr <<1,0,
     //      0,0.0001;
     // Qrh <<0.0001,0,
     //      0,0;

     Eigen::MatrixXd Qhh = Eigen::MatrixXd::Zero(2*n_dofs,2*n_dofs);
     Eigen::MatrixXd Qhr = Eigen::MatrixXd::Zero(2*n_dofs,2*n_dofs);
     Eigen::MatrixXd Qrr = Eigen::MatrixXd::Zero(2*n_dofs,2*n_dofs);
     Eigen::MatrixXd Qrh = Eigen::MatrixXd::Zero(2*n_dofs,2*n_dofs);

     Qhh.block(0,0,n_dofs,n_dofs) = Eigen::MatrixXd::Identity(n_dofs,n_dofs);
     Qhh.block(n_dofs,n_dofs,n_dofs,n_dofs) = 0.0001 * Eigen::MatrixXd::Identity(n_dofs,n_dofs);


     Qhr.block(n_dofs,n_dofs,n_dofs,n_dofs) = 0.0001 * Eigen::MatrixXd::Identity(n_dofs,n_dofs);

     Qrh.block(0,0,n_dofs,n_dofs) = 0.0001 * Eigen::MatrixXd::Identity(n_dofs,n_dofs);

     Qrr.block(0,0,n_dofs,n_dofs) = Eigen::MatrixXd::Identity(n_dofs,n_dofs);
     Qrr.block(n_dofs,n_dofs,n_dofs,n_dofs) = 0.0001 * Eigen::MatrixXd::Identity(n_dofs,n_dofs);

     std::cout<<"Qhh: \n"<<Qhh<<std::endl;
     std::cout<<"Qhr: \n"<<Qhr<<std::endl;
     std::cout<<"Qrr: \n"<<Qrr<<std::endl;
     std::cout<<"Qrh: \n"<<Qrh<<std::endl;




     // Eigen::MatrixXd Rh; Rh.resize(n_dofs, n_dofs); Rh<< .0001;
     // Eigen::MatrixXd Rr; Rr.resize(n_dofs, n_dofs); Rr<< .0001;

     Eigen::MatrixXd Rh = .0001 * Eigen::MatrixXd::Identity(n_dofs,n_dofs);
     Eigen::MatrixXd Rr = .0001 * Eigen::MatrixXd::Identity(n_dofs,n_dofs);
     double alpha = 0.5;

     Arbitration arbitration;
     double cos_theta;
     int decision;

     cgt.setAlpha(alpha);

     cgt.setCostsParams(Qhh,Qhr,Qrh,Qrr,Rh,Rr);

     Eigen::MatrixXd Qh;
     Eigen::MatrixXd Qr; 




     cgt.getCostMatrices(Qh,Qr,Rh,Rr);

     std::cout<<"Qh: \n"<<Qh<<std::endl;
     std::cout<<"Qr: \n"<<Qr<<std::endl;

     ncgt.setCostsParams(Qh,Qr,Rh,Rr);
  
  
     cgt.computeCooperativeGains();

     // ROS_INFO_STREAM("cgt: \n"<<cgt.getCooperativeGains());

     ncgt.computeNonCooperativeGains();

     Eigen::MatrixXd Kh,Kr;
     ncgt.getNonCooperativeGains(Kh,Kr);
     // ROS_INFO_STREAM("Kh: \n"<<Kh);
     // ROS_INFO_STREAM("Kr: \n"<<Kr);


     std::cin.get();


     Eigen::VectorXd rh = ref_h.col(0);
     Eigen::VectorXd rr = ref_r.col(0);
     cgt.setPosReference(rh,rr);
     ncgt.setPosReference(rh,rr);  

     for (int i = 0;i<100;i++)  
     {
    
          rh = ref_h.col(i);
          rr = ref_r.col(i);

          Eigen::VectorXd cgt_state = cgt.getCurrentState();
          // Eigen::VectorXd u_cgt = cgt.computeControlInputs();
          // Eigen::VectorXd u1_cgt = u_cgt.segment(0,n_dofs);
          // Eigen::VectorXd u2_cgt = u_cgt.segment(n_dofs,n_dofs);
          // arbitration.CosineSimilarity(u1_cgt, u2_cgt, cos_theta, decision);
          // std::cout<<"cos_theta: "<<cos_theta<<std::endl;
          // std::cout<<"decision: "<<decision<<std::endl;


          cgt.step(cgt_state ,rh,rr);
          std::cout<<"cgt_state: "<<cgt_state.transpose()<<std::endl;
          // ROS_INFO_STREAM("cgt_state : "<<cgt_state .transpose());

          Eigen::VectorXd ncgt_state = ncgt.getCurrentState();
          // Eigen::VectorXd u_ncgt = ncgt.computeControlInputs();
          // Eigen::VectorXd u1_ncgt = u_ncgt.segment(0,n_dofs);
          // Eigen::VectorXd u2_ncgt = u_ncgt.segment(n_dofs,n_dofs);
          // arbitration.CosineSimilarity(u1_ncgt, u2_ncgt, cos_theta, decision);
          // std::cout<<"cos_theta: "<<cos_theta<<std::endl;
          // std::cout<<"decision: "<< decision<<std::endl;
          ncgt.step(ncgt_state ,rh,rr);
          std::cout<<"ncgt_state: "<<ncgt_state.transpose()<<std::endl;
          // ROS_INFO_STREAM("ncgt_state : "<<ncgt_state .transpose());
     }
     
     
  return 0;
}





