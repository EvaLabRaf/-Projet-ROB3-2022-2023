/*
 *  This function is coded to reduce the GPS error of a car
 *  given its GPS(from multi satalites) and its position
 *  from other LiDARs if given
 */

#include<bits/stdc++.h>
#include"/usr/include/eigen3/Eigen/Eigen" // include the path from the eigen3 file

using namespace Eigen;
using namespace std;

class Kalman{

private:

    int i=1,n; // number of samples/readings (redundunt)

    double
            
           a,dt=-1/*time can't be neg- thus will always initialize if (line 124)*/,ti,x,y,v0,v,TH; // GPS,Speed,Acce
    
    vector<double> vec;
            

    /*
     * Our state space model taken from motion equations
     * X(i)= AX(i-1) + BU + W  (matrix format)
     * x(t+dt)= x(t)+ t v(dt) + 0.05 t^2 a(dt)
     */
    
    Matrix<double,4,4> K,P,A; 
    Matrix<double,4,2> B; 
    Matrix<double,4,1> X,Xp,Xm,Y; 
    Matrix<double,2,1> U;
    Matrix<double,2,4> H; 
    /*
     * A: zero input / state response
     * B: zero state / input response
     * X: state(updated)
     * Xm: states(measured)
     * Xp: states(predicted)
     * Y: states(measured)
     * U: acceleration input
     * States are position and velocity
     * K: kalman gain
     * P: covariance matrix
     */
   
    /*
     * Errors and noises of the Kalman
     */
    Matrix<double,4,4> R; // covariance of observation noise
    Matrix<double,4,4> Q; // covariance of process(algorithm) noise
    Matrix<double,4,1> Z ; // covariance of observation(measurement) error
  //Matrix<double,4,1> W ; // covariance of prediction error

    /*
     * Statistics (for future use)
     * row:1 GPSx, row:2 GPSy row:3 speed.
     */
    //Matrix<double,2,2> MEANS; // readings means
    //Matrix<double,2,2> VARS ; // readings variances

protected:
    
public:
    /*// constructor
    Kalman(double &t0,
           double &GPSguess,double &vguess,
           double &gpsVar,double &vVar);*/
    
    void  intialize(double t0,
                    double GPSguess,double vguess,
                    double gpsVar,double vVar);
    // next estimation
    void handleUpdate(double t, double x,double y,
                                double vx,double vy,
                                double ax,double ay);

};



/*Kalman::Kalman(double &t0,double &GPSguess,double &vguess,
               double &gpsVar,double &vVar){
    
    X<<GPSguess,GPSguess,vguess,vguess;
    ti=t0;
    // initialize the states and input coefficients matrix
    A << 1,0,t0,0, 0,1,0,t0, 0,0,1,0, 0,0,0,1;
    B << 0.5*t0*t0,0, 0,0.5*t0*t0, t0,0, 0,t0;
    // errors coefficients matrix
    R << gpsVar,0,0,0,0,gpsVar,0,0,0,0,vVar,0,0,0,0,vVar;
    P << gpsVar,0,0,0,0,gpsVar,0,0,0,0,vVar,0,0,0,0,vVar;
    K= Matrix<double,4,4>::Zero();
    // noise model
    Z.setRandom();
    Q.setRandom();
    Z*=0.1;
    Q*=0.1;

}*/

void Kalman::intialize(double t0,
                       double GPSguess,double vguess,
                       double gpsVar,double vVar){
    dt=-1;                           
    X<<GPSguess,GPSguess,vguess,vguess;
    ti=t0;
    // initialize the states and input coefficients matrix
    A << 1,0,t0,0, 0,1,0,t0, 0,0,1,0, 0,0,0,1;
    B << 0.5*t0*t0,0, 0,0.5*t0*t0, t0,0, 0,t0;
    // errors coefficients matrix
    
    P << 500,0,0,0,0,500,0,0,0,0,100,0,0,0,0,100;
    K=Matrix<double,4,4>::Zero();
    
    // noise model
    //R << gpsVar,0,0,0,0,gpsVar,0,0,0,0,vVar,0,0,0,0,vVar;
    //Z.setRandom();
    //Q.setRandom();
    //Z*=0.1;
    //Q*=0.1;

    R<< gpsVar,0,0,0, 0,gpsVar,0,0, 0,0,1,0, 0,0,0,1;
    Q<< sqrt(gpsVar),0,0,0, 0,sqrt(gpsVar),0,0, 0,0,1,0, 0,0,0,1;
    //Q=Matrix<double,4,4>::Identity();
    
    
}

void Kalman::handleUpdate(double t, 
                          double x,double y,
                          double vx,double vy,
                          double ax,double ay){

    /*
     * kalman filter algo goes here
     */

    // fill A,B if dt changes
    if(dt!=(t-ti)){ // if the new interval equal the old why update A,B
    dt=t-ti; ti=t;                             // get the new interval
    A << 1,0,dt,0, 0,1,0,dt, 0,0,1,0, 0,0,0,1; // update A matrix
    B << 0.5*dt*dt,0, 0,0.5*dt*dt, dt,0, 0,dt; // update B matrix
    }

    /*
     * update measurement + input 
     */
    Xm<<x,y,vx,vy;  // fill the new measurement data
    U<<ax,ay;       // save a as vectorized input


    /*
     * kalman algo starts here
     */ 
        // predicted state X(i)=A*X(i-1)+B*U(i)+W;
        Xp=A*X+B*U/*+W*/;
        // predicted covariance P(i)=A*X(i-1)P^(-1)+Q;
        P=(A*P)*A.transpose()+Q;
        // measurment Input
        Y=Xm;//+Z;
        // kalman gain
        K=P*(P+R).inverse();
        // update states
        X=Xp+K*(Y-Xp);
        // update covariance
        P=(Matrix<double,4,4>::Identity()-K)*P;
        // display result
        // clear vec 
        //vec.clear();
        //vec={X(0,0),X(1,0),X(2,0),X(3,0)};
        cout<<X.transpose()<<endl;
        //cout<<P<<endl;
        return; //vec;
        // next iteration for Xm,U
}




/*
 * Input Understanding
 * 1 time step
 * 0 0 initial position x y
 * 0 0 initial velocity x y
 * 2 0 acceleration x y
 * 5 5 GPS measurement error
 * 1 1 veocity measurement error
 * 4 number if readings (multi satalites)
 * 0 0 1 0 0 2 0 GPS x y SPEED x y  Acce x y
 * .... to end of readings
 *
 * check the output file too
 */

