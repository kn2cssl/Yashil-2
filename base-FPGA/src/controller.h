/*
 * controller.h
 *
 * Created: 8/14/2015 12:18:19 PM
 *  Author: QWA
 */ 



#ifndef CONTROLLER_H_
#define CONTROLLER_H_
#include <math.h>

//geometric constants

#define a1 56.31/180*M_PI    // 0.9827949017980069  rad
#define a2 135/180*M_PI      // 2.356194490192345   rad
#define a3 225/180*M_PI      // 3.926990816987241   rad
#define a4 303.69/180*M_PI   // 5.300390405381579   rad

#define g1 20.01/180*M_PI    // 0.3492403833240653  rad
#define g2 0/180*M_PI        // 0				   rad
#define g3 0/180*M_PI        // 0				   rad
#define g4 20.01/180*M_PI    // 0.3492403833240653  rad

//

#define sina1 0.8321   
#define sina2 0.7071   
#define sina3 -0.7071  
#define sina4 -0.8321  

#define cosa1 0.5547  
#define cosa2 -0.7071 
#define cosa3 -0.7071 
#define cosa4 0.5547  

#define cosg1 0.9396 
#define cosg2 1       
#define cosg3 1       
#define cosg4 0.9396  


//robot constants
#define N	76/20            //         %
#define res 1.2              //       %ohm
#define km	25.5/1000        //        %Nm/A
#define kn	374              //        %rpm/V
#define kf	0.0001           //        %unknown
#define ks	0.1              //       %unknown
#define r	28.5/1000        //         %m
#define J	0.0192           //         %kg*m2%           >>modeling needed
#define Jm	92.5/1000/10000  //        %kg*m2								//          0.00000925
#define Jw	0.0000233        //        %kg*m2        >>modeling needed		//          0.00000642  obtained from SOLIDWORKS's model
#define d	0.084            //         %m
#define M	1.5	             //          %kg         >>need measuring

#define b	60/(2*M_PI*r)


#define landa 0.99999

//run time : 4694 clk 
void setpoint_generator ( void ) ;

//run time : 12407 clk
void state_feed_back ( void ) ;

void kalman_observer ( void ) ;

double sign ( double number ) ;

extern double Vx , Vy , Wr ;

extern double x[7][1] , dx[7][1] ,xd[7][1] , du[4][1] , ud[4][1] , u[4][1] ;

extern double Yd[7] ;

extern double A [7][7] ;

extern double B [7][4] ;

extern double C [7][7] ;

//  -inv(B'*B)*B'*A	= uFx					
extern double uFx[4][7] ;
	
//k:state feed back	
extern double k_sf[4][7] ;

extern double battery_voltage ;

extern double max_ocr ;


#endif /* CONTROLLER_H_ */