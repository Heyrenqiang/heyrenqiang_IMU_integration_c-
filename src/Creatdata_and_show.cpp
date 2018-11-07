#include<ros/ros.h>
#include<iostream>
#include<fstream>
#include<visualization_msgs/Marker.h>
#include<geometry_msgs/PointStamped.h>
#include<tf/transform_broadcaster.h>
#include<Eigen/Dense>
#include<cmath>
#include<cstdio>
#include<stdexcept>
#include<iomanip>
#include<opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define pi 3.1415926
#define ACC_UP 0.1
#define GYR_UP 3.1415926

using namespace std;
using namespace Eigen;
using namespace cv;

std::string turtle_name;
ros::Publisher posepub;
visualization_msgs::Marker points, line_strip, line_list,line_strip2,line_strip3;
visualization_msgs::Marker line_strip4,line_strip5;
std::vector<Vector3d> acc;
std::vector<Vector3d> gyr;

std::vector<Quaterniond> poses_q;
std::vector<Vector3d> velocitys;
std::vector<Vector3d> poses_t;

std::vector<Matrix<double,15,15>> Jacobians;
std::vector<Matrix<double,15,15>> Covariances;

MatrixXd Noise=MatrixXd::Zero(12,12);

bool IFSHOWBIAS;

int z=0;

int ShOWWITCH;
int showwitchtype;
double p_initx,p_inity,p_initz,v_initx,v_inity,v_initz,q_initx,q_inity,q_initz;
double acc_biasx,acc_biasy,acc_biasz,gyr_biasx,gyr_biasy,gyr_biasz;
double acc_noise,gyr_noise,acc_bias_noise,gyr_bias_noise;

void ReadFromfile(){
    FileStorage fs("./config/config.yaml",FileStorage::READ);
    if(!fs.isOpened()){
         cerr<<"error ! can not open the fiel."<<endl;
    }
    showwitchtype=fs["showwitchtype"];
    p_initx=fs["p_initx"];
    p_inity=fs["p_inity"];
    p_initz=fs["p_initz"];
    v_initx=fs["v_initx"];
    v_inity=fs["v_inity"];
    v_initz=fs["v_initz"];
    q_initx=fs["q_initx"];
    q_inity=fs["q_initt"];
    q_initz=fs["q_initz"];
    acc_biasx=fs["acc_biasx"];
    acc_biasy=fs["acc_biasy"];
    acc_biasz=fs["acc_biasz"];
    gyr_biasx=fs["gyr_biasx"];
    gyr_biasy=fs["gyr_biasy"];
    gyr_biasz=fs["gyr_biasz"];
    acc_noise=fs["acc_noise"];
    gyr_noise=fs["gyr_noise"];
    acc_bias_noise=fs["acc_bias_noise"];
    gyr_bias_noise=fs["gyr_bias_noise"];
}

void poseCallback(const geometry_msgs::PointStamped& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg.point.x, msg.point.y, msg.point.z) );
  tf::Quaternion q;
  q.setRPY(30,60,45);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "mypose"));
  geometry_msgs::Point p;
  p.x=msg.point.x;
  p.y= msg.point.y;
  p.z= msg.point.z;
  line_strip.points.push_back(msg.point);
  posepub.publish(line_strip);
}

void show_trajectory_pose(){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    //transform.setOrigin(tf::Vector3(3,3,3));
    transform.setOrigin( tf::Vector3(2.0*sin(ros::Time::now().toSec()), 2.0*cos(ros::Time::now().toSec()), 0.0) );
    tf::Quaternion q;
    q.setRPY(30,60,45);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"world","currentframe"));

    //transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

}
void ClearAndAddFirst(){
    poses_q.clear();
    velocitys.clear();
    poses_t.clear();
    acc.clear();
    gyr.clear();

    Quaterniond q(1,q_initx,q_inity,q_initz);
    poses_q.push_back(q);
    Vector3d velocity(v_initx,v_inity,v_initz);
    velocitys.push_back(velocity);
    Vector3d pose_t(p_initx,p_inity,p_initz);
    poses_t.push_back(pose_t);
}
void GenerateIMUdata(){
    //Vecotr3d acc_temp,gyr_temp;
    for(int i=0;i<1000;i++){
        Vector3d gyr_temp(GYR_UP*i/1000,pi*i/1000,pi/3);
        gyr.push_back(gyr_temp);
    }
    for(int i=1000;i<3000;i++){
        Vector3d gyr_temp(-pi/6,pi/12,pi/12);
        gyr.push_back(gyr_temp);
    }
    for(int i=0;i<1000;i++){
        int z=1000-i-1;
        Vector3d gyr_temp(z*GYR_UP/1000,-z*GYR_UP/1000,-pi/6);
        gyr.push_back(gyr_temp);
    }

    for(int i=0;i<2000;i++){
        Vector3d acc_temp(i*ACC_UP/2000,-i*ACC_UP/2000,i*ACC_UP/2000);
        acc.push_back(acc_temp);
    }
    for(int i=0;i<2000;i++){
        int z=2000-i-1;
        Vector3d acc_temp(-z*ACC_UP/2000,-z*ACC_UP/2000,z*ACC_UP/2000);
        acc.push_back(acc_temp);
    }
}
// void ShowIMUdata(Quaterniond pose_R_q,Vector3d pose_t){
//     static tf::TransformBroadcaster br;
//     tf::Transform transform;
//     transform.setOrigin( tf::Vector3(pose_t.x(), pose_t.y(), pose_t.z()) );
//     transform.setRotation(tf::Quaternion(pose_R_q.x(),pose_R_q.y(),pose_R_q.z(),pose_R_q.w()));
//     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "mypose"));
//     geometry_msgs::Point p;
//     p.x= pose_t.x();
//     p.y= pose_t.y();
//     p.z= pose_t.z();
//     switch (ShOWWITCH)
//     {
//     case 0:
//         line_strip.points.push_back(p);
//         posepub.publish(line_strip);
//         break;
//     case 1:
//         line_strip2.points.push_back(p);
//         posepub.publish(line_strip2);
//         break;
//     case 2:
//         line_strip3.points.push_back(p);
//         posepub.publish(line_strip3);
//         break;
//     case 3:
//         line_strip4.points.push_back(p);
//         posepub.publish(line_strip4);
//         break;
//     case 4:
//         line_strip5.points.push_back(p);
//         posepub.publish(line_strip5);
//         break;
//     default:
//         /* code */
//         break;
//     }
//
// }
void ShowIMUdata(Quaterniond pose_R_q,Vector3d pose_t,visualization_msgs::Marker &line){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(pose_t.x(), pose_t.y(), pose_t.z()) );
    transform.setRotation(tf::Quaternion(pose_R_q.x(),pose_R_q.y(),pose_R_q.z(),pose_R_q.w()));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "mypose"));
    geometry_msgs::Point p;
    p.x= pose_t.x();
    p.y= pose_t.y();
    p.z= pose_t.z();
    line.points.push_back(p);
    posepub.publish(line);
}
void AddNoise(/* arguments */) {
    cv::RNG rng;
    for(size_t i=0;i<acc.size();i++){
        acc[i]=acc[i]+Vector3d(rng.gaussian(acc_noise),rng.gaussian(acc_noise),rng.gaussian(acc_noise));
        gyr[i]=gyr[i]+Vector3d(rng.gaussian(gyr_noise),rng.gaussian(gyr_noise),rng.gaussian(gyr_noise));
    }
}

void Addbias(Vector3d acc_bias,Vector3d gyr_bias){
    for(size_t i=0;i<acc.size();i++){
        // if(i<100)
        //     continue;
        acc[i]=acc[i]-acc_bias;
        gyr[i]=gyr[i]-gyr_bias;
    }
}

Matrix3d ToMat(Vector3d a){
    Matrix3d b;
    b<<0,-a(2),a(1),
    a(2),0,-a(0),
    -a(1),a(0),0;
    return b;
}
void Initialforshow(){
    cout<<"please choose a model !"<<endl;
    cout<<"the correct model include:"<<endl;
    cout<<"0: show trajectory with different bias!"<<endl;
    cout<<"1: show trajectory with different initial state v!"<<endl;
    cout<<"2: show trajectory with different noise!"<<endl;
    cout<<"3: show trajectory with bias solved by different methods(reintegration and one order jacobian similation)"<<endl;
    cout<<"4: show trajectory with different initial state p,q!"<<endl;
    line_strip.points.clear();
    line_strip2.points.clear();
    line_strip3.points.clear();
    line_strip4.points.clear();
    line_strip5.points.clear();
    p_initx=0;
    p_inity=0;
    p_initz=0;
    v_initx=0;
    v_inity=0;
    v_initz=0;
    q_initx=0;
    q_inity=0;
    q_initz=0;
    acc_biasx=0;
    acc_biasy=0;
    acc_biasz=0;
    gyr_biasx=0;
    gyr_biasy=0;
    gyr_biasz=0;
    acc_noise=0;
    gyr_noise=0;
    acc_bias_noise=0;
    gyr_bias_noise=0;

}
void Showwithoutbias(ros::Rate &r,visualization_msgs::Marker& line){
    ROS_INFO("Showwithoutbias");
    ShOWWITCH=0;
    z=0;
    line.points.clear();
    ClearAndAddFirst();
    GenerateIMUdata();
    Jacobians.clear();
    Covariances.clear();
    while(z<4001){
        MatrixXd Jacobian=MatrixXd::Identity(15,15);
        MatrixXd Covariance=MatrixXd::Zero(15,15);
        Quaterniond q[11];
          Vector3d v[11];
          Vector3d s[11];
          q[0].w()=1;
          q[0].x()=0;
          q[0].y()=0;
          q[0].z()=0;
          v->x()=0;
          v->y()=0;
          v->z()=0;
          s->x()=0;
          s->y()=0;
          s->z()=0;
          double dt=0.005;
          for(int i=1;i<11;i++){

              Matrix3d R_q(q[i-1]);

              Vector3d un_omiga=0.5*(gyr[z+i-1]+gyr[z+i]);

              Quaterniond deltaq(1,0.5*un_omiga.x()*dt,0.5*un_omiga.y()*dt,0.5*un_omiga.z()*dt);

              q[i]=q[i-1]*deltaq;

              Matrix3d R_q_2(q[i]);

              Vector3d un_acc=0.5*(R_q*acc[z+i-1]+R_q_2*acc[z+i]);

              v[i]=v[i-1]+un_acc*dt;

              s[i]=s[i-1]+v[i-1]*dt+0.5*un_acc*dt*dt;

              Matrix3d f33,f25,f24,f23,f15,f14,f13;
              f33=Matrix3d::Identity()-dt*ToMat(un_omiga);
              f25=0.5*dt*dt*R_q_2*ToMat(acc[z+i]);
              f24=-0.5*dt*(R_q+R_q_2);
              f23=-0.5*dt*((R_q*ToMat(acc[z+i-1])+R_q_2*ToMat(acc[z+i])*f33));
              f15=0.5*dt*f25;
              f14=0.5*dt*f24;
              f13=0.5*dt*f23;

              Matrix3d v11,v12,v21,v22;
              v22=-0.5*dt*(R_q+R_q_2);
              v21=0.5*dt*dt*R_q_2*ToMat(acc[z+i]);
              v12=0.5*dt*v22;
              v11=0.5*dt*v21;

            MatrixXd F=MatrixXd::Zero(15,15);
            MatrixXd V=MatrixXd::Zero(15,12);
            F.block<3,3>(0,0)=Matrix3d::Identity();
            F.block<3,3>(3,3)=Matrix3d::Identity();
            F.block<3,3>(6,6)=f33;
            F.block<3,3>(9,9)=Matrix3d::Identity();
            F.block<3,3>(12,12)=Matrix3d::Identity();
            F.block<3,3>(0,3)=Matrix3d::Identity()*dt;
            F.block<3,3>(0,6)=f13;
            F.block<3,3>(0,9)=f14;
            F.block<3,3>(0,12)=f15;
            F.block<3,3>(3,6)=f23;
            F.block<3,3>(3,9)=f24;
            F.block<3,3>(3,12)=f25;
            F.block<3,3>(6,12)=-Matrix3d::Identity()*dt;

            V.block<3,3>(0,0)=v11;
            V.block<3,3>(0,3)=v12;
            V.block<3,3>(3,0)=v21;
            V.block<3,3>(3,3)=v22;
            V.block<3,3>(6,3)=-Matrix3d::Identity()*dt;
            V.block<3,3>(9,6)=Matrix3d::Identity()*dt;
            V.block<3,3>(12,9)=Matrix3d::Identity()*dt;

            Jacobian=F*Jacobian;

            Covariance=F*Covariance*F.transpose()+V*Noise*V.transpose();

          }

          Jacobians.push_back(Jacobian);
          Covariances.push_back(Covariance);
          z=z+10;

          Quaterniond R_new;
          R_new=poses_q.back()*q[10];
          Vector3d v_new;
          v_new=velocitys.back()+poses_q.back()*v[10];
          Vector3d s_new;
          s_new=poses_t.back()+velocitys.back()*dt*10+poses_q.back()*s[10];

          ShowIMUdata(R_new,s_new,line);
          poses_q.push_back(R_new);
          velocitys.push_back(v_new);
          poses_t.push_back(s_new);

        r.sleep();
    }
}
void Showwithzeroinitial(ros::Rate &r,visualization_msgs::Marker& line){
    ROS_INFO("Showwithzeroinitial");
    ShOWWITCH=4;
    IFSHOWBIAS=false;
    z=0;
    poses_q.clear();
    velocitys.clear();
    poses_t.clear();
    Quaterniond q(1,0,0,0);
    poses_q.push_back(q);
    Vector3d velocity(0,0,0);
    velocitys.push_back(velocity);
    Vector3d pose_t(0,0,0);
    poses_t.push_back(pose_t);
    GenerateIMUdata();
    line.points.clear();
    while(z<4001){
        MatrixXd Jacobian=MatrixXd::Identity(15,15);
        MatrixXd Covariance=MatrixXd::Zero(15,15);
        Quaterniond q[11];
          Vector3d v[11];
          Vector3d s[11];
          q[0].w()=1;
          q[0].x()=0;
          q[0].y()=0;
          q[0].z()=0;
          v->x()=0;
          v->y()=0;
          v->z()=0;
          s->x()=0;
          s->y()=0;
          s->z()=0;
          double dt=0.005;
          for(int i=1;i<11;i++){

              Matrix3d R_q(q[i-1]);

              Vector3d un_omiga=0.5*(gyr[z+i-1]+gyr[z+i]);

              Quaterniond deltaq(1,0.5*un_omiga.x()*dt,0.5*un_omiga.y()*dt,0.5*un_omiga.z()*dt);

              q[i]=q[i-1]*deltaq;

              Matrix3d R_q_2(q[i]);

              Vector3d un_acc=0.5*(R_q*acc[z+i-1]+R_q_2*acc[z+i]);

              v[i]=v[i-1]+un_acc*dt;

              s[i]=s[i-1]+v[i-1]*dt+0.5*un_acc*dt*dt;

              Matrix3d f33,f25,f24,f23,f15,f14,f13;
              f33=Matrix3d::Identity()-dt*ToMat(un_omiga);
              f25=0.5*dt*dt*R_q_2*ToMat(acc[z+i]);
              f24=-0.5*dt*(R_q+R_q_2);
              f23=-0.5*dt*((R_q*ToMat(acc[z+i-1])+R_q_2*ToMat(acc[z+i])*f33));
              f15=0.5*dt*f25;
              f14=0.5*dt*f24;
              f13=0.5*dt*f23;

              Matrix3d v11,v12,v21,v22;
              v22=-0.5*dt*(R_q+R_q_2);
              v21=0.5*dt*dt*R_q_2*ToMat(acc[z+i]);
              v12=0.5*dt*v22;
              v11=0.5*dt*v21;

            MatrixXd F=MatrixXd::Zero(15,15);
            MatrixXd V=MatrixXd::Zero(15,12);
            F.block<3,3>(0,0)=Matrix3d::Identity();
            F.block<3,3>(3,3)=Matrix3d::Identity();
            F.block<3,3>(6,6)=f33;
            F.block<3,3>(9,9)=Matrix3d::Identity();
            F.block<3,3>(12,12)=Matrix3d::Identity();
            F.block<3,3>(0,3)=Matrix3d::Identity()*dt;
            F.block<3,3>(0,6)=f13;
            F.block<3,3>(0,9)=f14;
            F.block<3,3>(0,12)=f15;
            F.block<3,3>(3,6)=f23;
            F.block<3,3>(3,9)=f24;
            F.block<3,3>(3,12)=f25;
            F.block<3,3>(6,12)=-Matrix3d::Identity()*dt;

            V.block<3,3>(0,0)=v11;
            V.block<3,3>(0,3)=v12;
            V.block<3,3>(3,0)=v21;
            V.block<3,3>(3,3)=v22;
            V.block<3,3>(6,3)=-Matrix3d::Identity()*dt;
            V.block<3,3>(9,6)=Matrix3d::Identity()*dt;
            V.block<3,3>(12,9)=Matrix3d::Identity()*dt;

            Jacobian=F*Jacobian;

            Covariance=F*Covariance*F.transpose()+V*Noise*V.transpose();

          }

          Jacobians.push_back(Jacobian);
          Covariances.push_back(Covariance);

          // ROS_INFO("q: %f %f %f",q[10].x(),q[10].y(),q[10].z());
          // ROS_INFO("v: %f %f %f",v[10].x(),v[10].y(),v[10].z());
          // ROS_INFO("s: %f %f %f",s[10].x(),s[10].y(),s[10].z());
          z=z+10;

          Quaterniond R_new;
          R_new=poses_q.back()*q[10];
          Vector3d v_new;
          v_new=velocitys.back()+poses_q.back()*v[10];
          Vector3d s_new;
          s_new=poses_t.back()+velocitys.back()*dt*10+poses_q.back()*s[10];

          // ROS_INFO("q_new: %f %f %f",R_new.x(),R_new.y(),R_new.z());
          // ROS_INFO("v_new: %f %f %f",v_new.x(),v_new.y(),v_new.z());
          // ROS_INFO("s_new: %f %f %f",s_new.x(),s_new.y(),s_new.z());

          ShowIMUdata(R_new,s_new,line);
          poses_q.push_back(R_new);
          velocitys.push_back(v_new);
          poses_t.push_back(s_new);

        r.sleep();
    }
}
void Showwithbias(Vector3d acc_bias,Vector3d gyr_bias,ros::Rate &r,visualization_msgs::Marker& line){
    ROS_INFO("Showwithbias");
    ShOWWITCH=1;
    z=0;
    line.points.clear();
    ClearAndAddFirst();
    GenerateIMUdata();
    Addbias(acc_bias,gyr_bias);

    while(z<4001){
        Quaterniond q[11];
          Vector3d v[11];
          Vector3d s[11];
          q[0].w()=1;
          q[0].x()=0;
          q[0].y()=0;
          q[0].z()=0;
          v->x()=0;
          v->y()=0;
          v->z()=0;
          s->x()=0;
          s->y()=0;
          s->z()=0;
          double dt=0.005;
          for(int i=1;i<11;i++){

              Matrix3d R_q(q[i-1]);

              Vector3d un_omiga=0.5*(gyr[z+i-1]+gyr[z+i]);

              Quaterniond deltaq(1,0.5*un_omiga.x()*dt,0.5*un_omiga.y()*dt,0.5*un_omiga.z()*dt);

              q[i]=q[i-1]*deltaq;

              Matrix3d R_q_2(q[i]);

              Vector3d un_acc=0.5*(R_q*acc[z+i-1]+R_q_2*acc[z+i]);

              v[i]=v[i-1]+un_acc*dt;

              s[i]=s[i-1]+v[i-1]*dt+0.5*un_acc*dt*dt;

          }

          z=z+10;

          Quaterniond R_new;
          R_new=poses_q.back()*q[10];
          Vector3d v_new;
          v_new=velocitys.back()+poses_q.back()*v[10];
          Vector3d s_new;
          s_new=poses_t.back()+velocitys.back()*dt*10+poses_q.back()*s[10];

          ShowIMUdata(R_new,s_new,line);
          poses_q.push_back(R_new);
          velocitys.push_back(v_new);
          poses_t.push_back(s_new);

        r.sleep();
    }
}
void ShowUpdateBybias(Vector3d acc_bias,Vector3d gyr_bias,ros::Rate &r,visualization_msgs::Marker& line){

    ROS_INFO("ShowUpdateBybias");
    ShOWWITCH=2;
    z=0;
    int bantchs=0;
    line.points.clear();
    ClearAndAddFirst();
    GenerateIMUdata();
    while(z<4001){
        Quaterniond q[11];
          Vector3d v[11];
          Vector3d s[11];
          q[0].w()=1;
          q[0].x()=0;
          q[0].y()=0;
          q[0].z()=0;
          v->x()=0;
          v->y()=0;
          v->z()=0;
          s->x()=0;
          s->y()=0;
          s->z()=0;
          double dt=0.005;
          for(int i=1;i<11;i++){

              Matrix3d R_q(q[i-1]);

              Vector3d un_omiga=0.5*(gyr[z+i-1]+gyr[z+i]);

              Quaterniond deltaq(1,0.5*un_omiga.x()*dt,0.5*un_omiga.y()*dt,0.5*un_omiga.z()*dt);

              q[i]=q[i-1]*deltaq;

              Matrix3d R_q_2(q[i]);

              Vector3d un_acc=0.5*(R_q*acc[z+i-1]+R_q_2*acc[z+i]);

              v[i]=v[i-1]+un_acc*dt;

              s[i]=s[i-1]+v[i-1]*dt+0.5*un_acc*dt*dt;

          }

          z=z+10;
          Vector3d deltatheta=0.5*Jacobians[bantchs].block<3,3>(6,12)*gyr_bias;
          Quaterniond deltaq(1,deltatheta.x(),deltatheta.y(),deltatheta.z());

          q[10]=q[10]*deltaq;
          v[10]=v[10]+Jacobians[bantchs].block<3,3>(3,12)*gyr_bias+Jacobians[bantchs].block<3,3>(3,9)*acc_bias;
          s[10]=s[10]+Jacobians[bantchs].block<3,3>(0,12)*gyr_bias+Jacobians[bantchs].block<3,3>(0,9)*acc_bias;

          bantchs++;

          Quaterniond R_new;
          R_new=poses_q.back()*q[10];
          Vector3d v_new;
          v_new=velocitys.back()+poses_q.back()*v[10];
          Vector3d s_new;
          s_new=poses_t.back()+velocitys.back()*dt*10+poses_q.back()*s[10];

          ShowIMUdata(R_new,s_new,line);
          poses_q.push_back(R_new);
          velocitys.push_back(v_new);
          poses_t.push_back(s_new);

        r.sleep();
    }
}

void ShowwithNoise(ros::Rate &r,visualization_msgs::Marker& line){
    IFSHOWBIAS=true;
    ShOWWITCH=3;
    z=0;
    line.points.clear();
    ClearAndAddFirst();
    GenerateIMUdata();
    AddNoise();
    while(z<4001){
        Quaterniond q[11];
          Vector3d v[11];
          Vector3d s[11];
          q[0].w()=1;
          q[0].x()=0;
          q[0].y()=0;
          q[0].z()=0;
          v->x()=0;
          v->y()=0;
          v->z()=0;
          s->x()=0;
          s->y()=0;
          s->z()=0;
          double dt=0.005;
          for(int i=1;i<11;i++){

              Matrix3d R_q(q[i-1]);

              Vector3d un_omiga=0.5*(gyr[z+i-1]+gyr[z+i]);

              Quaterniond deltaq(1,0.5*un_omiga.x()*dt,0.5*un_omiga.y()*dt,0.5*un_omiga.z()*dt);

              q[i]=q[i-1]*deltaq;

              Matrix3d R_q_2(q[i]);

              Vector3d un_acc=0.5*(R_q*acc[z+i-1]+R_q_2*acc[z+i]);

              v[i]=v[i-1]+un_acc*dt;

              s[i]=s[i-1]+v[i-1]*dt+0.5*un_acc*dt*dt;

          }
          z=z+10;

          Quaterniond R_new;
          R_new=poses_q.back()*q[10];
          Vector3d v_new;
          v_new=velocitys.back()+poses_q.back()*v[10];
          Vector3d s_new;
          s_new=poses_t.back()+velocitys.back()*dt*10+poses_q.back()*s[10];

          ShowIMUdata(R_new,s_new,line);
          poses_q.push_back(R_new);
          velocitys.push_back(v_new);
          poses_t.push_back(s_new);

        r.sleep();
    }
}

int main(int argc, char** argv){

    ros::init(argc, argv, "my_tf_broadcaster");
    //if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
    //turtle_name = argv[1];

    ros::NodeHandle node;

    posepub=node.advertise<visualization_msgs::Marker>("visualization_marker",10);

    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id =line_strip2.header.frame_id= "world";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp =line_strip2.header.stamp= ros::Time::now();
    points.ns = line_strip.ns = line_list.ns =line_strip2.ns= "points_and_lines";
    points.action = line_strip.action =line_strip2.action= line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w =line_strip2.pose.orientation.w= line_list.pose.orientation.w = 1.0;

  points.id = 0;
  line_strip.id = 1;
  line_list.id = 2;
  line_strip2.id=3;

  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip2.type=visualization_msgs::Marker::LINE_STRIP;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.2;
  points.scale.y = 0.2;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.012;
  line_list.scale.x = 0.012;
  line_strip2.scale.x=0.012;

  // Points are green
  points.color.g = 1.0f;
  points.color.a = 1.0;

  // Line strip is red
  line_strip.color.r = 1.0;
  line_strip.color.a = 1.0;
  line_strip2.color.b=1.0;
  line_strip2.color.a=1.0;

  line_strip3=line_strip;
  line_strip3.id=4;
  line_strip3.color.r=0.7;
  line_strip3.color.b=0.9;
  line_strip3.color.a=1.0;

  line_strip4=line_strip;
  line_strip4.id=5;
  line_strip4.color.g=0.5;
  line_strip4.color.b=0.5;
  line_strip4.color.a=1.0;

  line_strip5=line_strip;
  line_strip5.id=6;
  line_strip5.color.g=0.8;
  line_strip5.color.b=0.2;
  line_strip5.color.a=1.0;
  ros::Rate r(80);
  int response;

  while(true){
      Initialforshow();
  std::cin>> response;
  if(response==9){
      cout<<"end!"<<endl;
    break;
    }
  ReadFromfile();
  ROS_INFO("showwitchtype:%d",showwitchtype);

  Vector3d acc_bias(acc_biasx,acc_biasy,acc_biasz);
  Vector3d gyr_bias(gyr_biasx,gyr_biasy,gyr_biasz);
  showwitchtype=response;
  switch (showwitchtype)
  {
  case 0://show bias and normal
      /* code */
      Showwithoutbias(r,line_strip);
      Showwithbias(acc_bias,gyr_bias,r,line_strip2);
      Showwithbias(Vector3d(0,0,0),Vector3d(0.002,0.002,0.002),r,line_strip3);

      Showwithbias(Vector3d(0,0,0),Vector3d(0.02,0.02,0.02),r,line_strip4);
      break;
  case 1://show normal and none zero initial
      Showwithzeroinitial(r,line_strip);
      Showwithoutbias(r,line_strip2);
      v_initx=-0.01;
      v_inity= 0.005;
      v_initz= -0.03;
      Showwithoutbias(r,line_strip3);
      v_initx=-0.01;
      v_inity= 0.01;
      v_initz= -0.01;
      Showwithoutbias(r,line_strip4);

      break;
  case 2://show normal and noise0.797
      Showwithoutbias(r,line_strip);
      ShowwithNoise(r,line_strip3);
      acc_noise= 0.02;
      gyr_noise= 0.0016;
      ShowwithNoise(r,line_strip4);
      acc_noise= 0.01;
      gyr_noise= 0.0008;
      ShowwithNoise(r,line_strip2);
      break;
  case 3://show normal and biaswiintegration and bias with jacobian simulization
      Showwithoutbias(r,line_strip);
      Showwithbias(acc_bias,gyr_bias,r,line_strip2);
      ShowUpdateBybias(acc_bias,gyr_bias,r,line_strip3);
      break;
  case 4://show normal and none zero initial
      //Showwithzeroinitial(r,line_strip);
      Showwithoutbias(r,line_strip2);
      p_initx=-0.5;
      p_inity= 0.5;
      p_initz= 0.5;
      Showwithoutbias(r,line_strip3);
      q_initx=0.1;
      q_inity= 0.2;
      q_initz= 0.4;
      Showwithoutbias(r,line_strip4);

      break;
  default:
      /* code */
      cout<<"please input correct model !"<<endl;
      cout<<"the correct model include:"<<endl;
      cout<<"0: show trajectory with different bias!"<<endl;
      cout<<"1: show trajectory with different initial state v!"<<endl;
      cout<<"2: show trajectory with different noise!"<<endl;
      cout<<"3: show trajectory with bias solved by different methods(reintegration and one order jacobian similation)"<<endl;
      cout<<"4: show trajectory with different initial state p,q!"<<endl;
      break;
  }
}
  // Showwithoutbias(r);
  // //cv::waitKey(0);
  // std::cin>> response;
  // ShowwithNoise(r);
  // //Vector3d acc_bias(0.001,0.001,0.001);
  // Vector3d acc_bias(0.0,0.0,0.0);
  // Vector3d gyr_bias(0.01,0.01,0.01);
  // //cv::waitKey(0);
  // std::cin>> response;
  // ShowUpdateBybias(acc_bias,gyr_bias,r);
  // //cv::waitKey(0);
  // std::cin>> response;
  // Showwithbias(acc_bias,gyr_bias,r);

  // while(true)
  //   show_trajectory_pose();

  //ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);
  //ros::Subscriber sub = node.subscribe("leica/position", 10, &poseCallback);

  ros::spin();
  return 0;
};
