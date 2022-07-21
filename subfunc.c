#include <stdio.h>
#include <math.h>


double	*add3( a, b, c )
	double	*a, *b, *c ;
{
	int		i ;

	for ( i = 0 ; i < 3 ; i ++ )
		*(c+i) = *(a+i) + *(b+i) ;
	return( c ) ;
}


double	*mmul31( a, b, c )
	double	a[][3], b[], c[] ;
{
	int		i, j ;

	for ( i = 0 ; i < 3 ; i ++ ) {
		c[i] = 0. ;
		for ( j = 0 ; j < 3 ; j ++ )
			c[i] += a[i][j] * b[j] ;
	}
	return( c ) ;
}


double	*mmul33( a, b, c )
	double	a[][3], b[][3], c[][3] ;
{
	int		i, j, k ;
	double	w ;

	for ( i = 0 ; i < 3 ; i ++ ) {
	    for ( j = 0 ; j < 3 ; j ++ ) {
			w = 0.0 ;
			for ( k = 0 ; k < 3 ; k ++ )
		  		w += a[i][k] * b[k][j] ;
			c[i][j] = w ;
		}
	}

	return( c[0] ) ;
}


double	*substi( n, x, y )
	int		n ;
	double	*x, *y ;
{

	int		i ;
	for ( i = 0 ; i < n ; i ++ )
		*(y+i) = *(x+i) ;
	return( y ) ;
/**
	memcpy( y, x, sizeof(double)*n ) ;
***/
}


void	Eulerian_ang(
	double	rot[3][3],
	double	Eulerian_ang[] )
{
	Eulerian_ang[0] = asin( - rot[1][2] ) ;
	Eulerian_ang[1] = atan2( rot[0][2] / cos( Eulerian_ang[0] ),
							 rot[2][2] / cos( Eulerian_ang[0] ) ) ;
	Eulerian_ang[2] = atan2( rot[1][0] / cos( Eulerian_ang[0] ),
							 rot[1][1] / cos( Eulerian_ang[0] ) ) ;
}



