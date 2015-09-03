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
double uFx[4][7] =	{	{   9.8396		,-6.5597	,-0.9334	,0.0467		,0			,0			,0		}	,
						{	8.3621		,8.3621		,-0.9934	,0			,0.0467		,0			,0		}	,
						{	-8.3621		,8.3621		,-0.9934    ,0			,0			,0.0467		,0		}	,
						{	-9.8396		,-6.5597	,-0.9334    ,0			,0			,0			,0.0467	}	}	;
	
//k:state feed back	
double k_sf[4][7] =   { {	-1.0379		,1.4653		,0.2593		,0.0095		,0.0036		,-0.0001	,0.0036		}	,
						{	-0.8820		,-1.0784	,0.2547		,0.0036		,0.0090		,0.0042		,-0.0001	}	,
						{	0.8820		,-1.0784    ,0.2547		,-0.0001    ,0.0042		,0.0090		,0.0036		}	,
						{	1.0379		,1.4653		,0.2593		,0.0036		,-0.0001    ,0.0036		,0.0095		}	}	;
							



void setpoint_generator ( void )
{
	// 7 set points for system
	//kinematics rules that should be considered for specifying desired output
	Yd[0] = Vx ;
	Yd[1] = Vy ;
	Yd[2] = Wr ;
	Yd[3] = (-Vx*sina1+Vy*cosa1+Wr*cosg1*d)*b ;
	Yd[4] = (-Vx*sina2+Vy*cosa2+Wr*cosg2*d)*b ;
	Yd[5] = (-Vx*sina3+Vy*cosa3+Wr*cosg3*d)*b ;
	Yd[6] = (-Vx*sina4+Vy*cosa4+Wr*cosg4*d)*b ;
}



void state_feed_back ( void )
{
	// 	xd=(C'*C)\C'*Yd;%inv(C'*C)*C'*Yd; & C = I7x7		=>		xd = Yd
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

	// 	du=-K*(x-xd);%for simulating controller with unnoisy data
	// 	%du=-K*(xl-xd);%for simulating controller and observer:(x-xd)|(xh-xd)|(xl-xd)
	
	for (int i=0 ; i < 7 ; i ++)
	{
		for (int j = 0 ; j < 1 ; j ++)
		{
			
			dx [i][j] = xd [i][j] - x [i][j] ;  // minus is here <<<<<=

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
			u [i][j] = du [i][j] + ud [i][j];
		}
	}
}





