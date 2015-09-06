/*
* controller.c
*
* Created: 8/14/2015 12:19:25 PM
*  Author: QWA
*/

#include "controller.h"

double Vx , Vy , Wr ;

double x[7][1] , dx[7][1] ,xd[7][1] , du[4][1] , ud[4][1] , u[4][1] ;

//Xdot = A*X + B*U
//Y = C*X + D*U

double Yd[7] ;// Y desired

double battery_voltage = 12.6 ;

double max_ocr = 4095 ;


double A [7][7] =	{	{	-53.2664	,3.7199		,0			,-0.0554	,-0.0471	,0.0471		,0.0554		}	,
						{	3.7199		,-36.0836	,0.6976		,0.0369		,-0.0471	,-0.0471	,0.0369		}	,
						{	0			,54.5001	,-185.4827	,0.4110		,0.4375		,0.4375		,0.4110		}	,
						{	-5065.0228	,3376.6732	,480.4730   ,-24.0355	,0			,0			,0			}	,
						{	-4304.4383	,-4304.4383	,511.3411	,0          ,-24.0355	,0			,0			}	,
						{	4304.4383	,-4304.4383 ,511.3411	,0			,0          ,-24.0355	,0			}	,
						{	5065.0228	,3376.6732  ,480.4730	,0			,0			,0			,-24.0355	}	};

double B [7][4] =	{	{	0			,0			,0			,0			}	,
						{	0			,0			,0			,0			}	,
						{	0			,0			,0			,0			}	,
						{	514.7574	,0			,0			,0			}	,
						{	0			,514.7574	,0			,0			}	,
						{	0			,0			,514.7574	,0			}	,
						{	0			,0			,0			,514.7574	}	}	;

double C [7][7] =	{	{	1     ,0     ,0     ,0     ,0     ,0     ,0		}	,
						{	0     ,1     ,0     ,0     ,0     ,0     ,0		}	,
						{	0     ,0     ,1     ,0     ,0     ,0     ,0		}	,
						{	0     ,0     ,0     ,1     ,0     ,0     ,0		}	,
						{	0     ,0     ,0     ,0     ,1     ,0     ,0		}	,
						{	0     ,0     ,0     ,0     ,0     ,1     ,0		}	,
						{	0     ,0     ,0     ,0     ,0     ,0     ,1		}	};

//  -inv(B'*B)*B'*A	= uFx					
double uFx[4][7] =	{	{   9.8396		,-6.5597	,-0.9334	,0.046693	,0			,0			,0			}	,
						{	8.3621		,8.3621		,-0.9936	,0			,0.046693	,0			,0			}	,
						{	-8.3621		,8.3621		,-0.9936    ,0			,0			,0.046693	,0			}	,
						{	-9.8396		,-6.5597	,-0.9334    ,0			,0			,0			,0.046693	}	}	;
							
	
//k:state feed back	
double k_sf[4][7] =   { {	-1.0379		,1.4653		,0.2593		,0.0095		,0.0036		,-0.0001	,0.0036		}	,
						{	-0.8820		,-1.0784	,0.2547		,0.0036		,0.0090		,0.0042		,-0.0001	}	,
						{	0.8820		,-1.0784    ,0.2547		,-0.0001    ,0.0042		,0.0090		,0.0036		}	,
						{	1.0379		,1.4653		,0.2593		,0.0036		,-0.0001    ,0.0036		,0.0095		}	}	;
							

#ifdef after_RLS_set_up 	

double G[7][7] =
{ { 157.15  ,0  ,0  ,-0.0009  ,-0.0008   ,0.0008   ,0.0009	}
0       ,166.78       ,170.28    ,0.0006  ,-0.0008  ,-0.0008    ,0.0006
0       ,6.8113        ,840.2   ,0  ,0  ,0   ,0
,-3792       2610.8       2372.7      0.17911     0.035114     -0.17546    -0.068673
,-3222.5      -3321.7      -2102.4     0.035114      0.17891  -4.5871e-05     -0.17546
,3222.5      -3321.7      -2102.4     -0.17546  -4.5871e-05      0.17891     0.035114
,3792       2610.8       2372.7    -0.068673     -0.17546     0.035114      0.17911	
				
double V [7][7]   =	{	{0.0000    ,0       ,0         ,0         ,0         ,0         ,0		 }	,
						{0         ,0.0000  ,0         ,0         ,0         ,0         ,0		 }	,
						{0         ,0		,0.0000    ,0         ,0         ,0         ,0		 }	,
						{0         ,0       ,0		   ,100.0000  ,0         ,0         ,0		 }	,
						{0         ,0       ,0         ,0		  ,100.0000  ,0         ,0		 }	,
						{0         ,0       ,0         ,0         ,0		 ,100.000   ,0		 }	,
						{0         ,0       ,0         ,0         ,0         ,0			,100.0000}	}	;
	
double W [7][7] =	{	{1.0000    ,0       ,0         ,0         ,0         ,0         ,0		 }	,
						{0         ,1.0000  ,0         ,0         ,0         ,0         ,0		 }	,
						{0         ,0		,1.0000    ,0         ,0         ,0         ,0		 }	,
						{0         ,0       ,0		   ,0.0100	  ,0         ,0         ,0		 }	,
						{0         ,0       ,0         ,0		  ,0.0100    ,0         ,0		 }	,
						{0         ,0       ,0         ,0         ,0		 ,0.010     ,0		 }	,
						{0         ,0       ,0         ,0         ,0         ,0			,0.010   }	}	;
#endif	
							
void setpoint_generator ( void )
{
	// 7 set points for system
	//kinematics rules that should be considered for specifying desired output
	
	// 	xd=(C'*C)\C'*Yd;%inv(C'*C)*C'*Yd; & C = I7x7		=>		xd = Yd
	xd[0][0] = Vx ;
	xd[1][0] = Vy ;
	xd[2][0] = Wr ;
	xd[3][0] = (-Vx*sina1+Vy*cosa1+Wr*cosg1*d)*b ;
	xd[4][0] = (-Vx*sina2+Vy*cosa2+Wr*cosg2*d)*b ;
	xd[5][0] = (-Vx*sina3+Vy*cosa3+Wr*cosg3*d)*b ;
	xd[6][0] = (-Vx*sina4+Vy*cosa4+Wr*cosg4*d)*b ;
	
	// data checking 1 : data is produced correctly (checked with model in MATLAB)
}

void state_feed_back ( void )
{
	// 	ud=-inv(B'*B)*B'*A*xd;
	
	for (int i=0 ; i < 4 ; i ++)
	{
		for (int j = 0 ; j < 1 ; j ++)
		{
			ud [i][j] = 0 ;
			for (int k = 0 ; k < 7 ; k ++)
			{
				ud [i][j] += uFx [i][k] * xd [k][j];
			}
		}
	}
	
	// data checking 2 : data is produced correctly (checked with model in MATLAB)

	// 	du=-K*(x-xd);%for simulating controller with unnoisy data
	// 	%du=-K*(xl-xd);%for simulating controller and observer:(x-xd)|(xh-xd)|(xl-xd)
	
	for (int i=0 ; i < 7 ; i ++)
	{
		for (int j = 0 ; j < 1 ; j ++)
		{
			
			//dx [i][j] = xd [i][j] - x [i][j] ;  // minus is here <<<<<=

		}
	}
	
	
	for (int i=0 ; i < 4 ; i ++)
	{
		for (int j = 0 ; j < 1 ; j ++)
		{
			du [i][j] = 0 ;
			for (int k = 0 ; k < 7 ; k ++)
			{
				du [i][j] += k_sf [i][k] * dx [k][j];
			}
		}
	}
	
	
	
	// 	u=ud+du;
	
	for (int i=0 ; i < 4 ; i ++)
	{
		for (int j = 0 ; j < 1 ; j ++)
		{
			u [i][j] = (du [i][j] + ud [i][j]);
			
			//emitting saturation
			if (fabs(u[i][j]) > battery_voltage)
			{
				u[i][j] = sign(u[i][j]) * battery_voltage ;
			}
		}	
	}
	
}


void kalman_observer (void)
{
	
}


//void RLS (void)
//{
	//
	//p=p/landa-(p*(xk'*xk)*p)/(landa+xk*p*xk')/landa
	//q=q+xk'*yk;
////         %x1
////         %x2
////         %x3
////         %x4
////         %x5
////    q=q+[%x6]*[y1 y2 y3 y4 y5 y6 y7 y8 y9 y10 y11]
////         %x7
////         %x8
////         %x9
////         %x10
////         %x11
	//
	//theta(:,:)=p*q;
//}


double sign (double number)
{
	if (number > 0)
	{
		return 1 ;
	}
	else if (number < 0)
	{
		return -1 ;
	}
	else
	{
		return 0 ;
	}
}





