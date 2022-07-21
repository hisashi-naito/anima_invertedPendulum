
#define DECLARATION

#pragma warning(disable : 4244)     // MIPS
#pragma warning(disable : 4136)     // X86
#pragma warning(disable : 4051)     // ALPHA


#define	WIDTH	1000
#define	HEIGHT	800
//使っていない？

#ifdef LSIZE
#define	WIDTH2		781
#define	HEIGHT2		581

#elif MSIZE
#define	WIDTH2		600
#define	HEIGHT2		450

#elif SSIZE
#define	WIDTH2		450
#define	HEIGHT2		338

#else 
#define	WIDTH2		781
#define	HEIGHT2		581
#endif
//#define	WIDTH2		400
//#define	HEIGHT2		300

//#define	WIDTH2		600
//#define	HEIGHT2		450

//#define	WIDTH2		500
//#define	HEIGHT2		375

//#define	WIDTH2		450
//#define	HEIGHT2		338


//ウインドウの位置
#ifdef LA
#define	WX		1
#define	WY		1

#elif LB
#define	WX		1
#define	WY		390

#elif RA
#define	WX		465
#define	WY		1

#elif RB
#define	WX		465
#define	WY		390

#else
#define	WX		1
#define	WY		1
#endif

//#define  SLOPE_ANGLE	10.0

//#define	WX		580
//#define	WY		450


#define	MAX_FORCE	300.
#define		DIM			3		/* The number of dimension */
#define XX 	0
#define YY 	1
#define ZZ 	2

#define 	PI			3.1415926535
#define		GA			9.8

#define		ON			1
#define		OFF			0
#define		NUM_LINK	50		/* The number of link */
#define		NUM_POINT		5
//#define		MAX_LINE	2000
#define		NUM_DIF			160

#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/freeglut.h>

#include <math.h>
#include <stdio.h>
//#include "robotModel.h"
#include "color.h"
#include "uglyfont.h"

long  tm, tmbef=0, tmnow, tmhz=0, itm=0, tmtbl[100] ;

static int cnt,timer = 0 ;
//static GLdouble eye[3]={2.0,-13.0,0.8};
static GLdouble eye[3]={2.0,-45.0,0.8};
static GLdouble center[3]={2.0,0.0,0.8};

int generateEPS(char *filename, int inColor, unsigned int width,
	unsigned int height);
int	get_model_from_file(char *filename);
void	Eulerian_ang() ;
void	calc_Rot() ;
void	calc_abs_co() ;
double	*mmul31() ;

float	BodyTranslate[]={ 2.0, -6.0, 0.3 };
//int		ALLROTATE[]={ -50, 10 };
int		ALLROTATE[]={ -0, 0 };
int		eps_flg = 0, trans_flg = ON ;
int		data_gc_flg = OFF ;	// 2005.10.06
int		ASC_flg = OFF, POS_flg = OFF ;

int		MARKER_flg = ON, MARKER_LINE_flg = ON ; // 2007.02.08

int		INV_FRONT_flg = OFF ;

int	 	ASC_read_flg = OFF ;
int		stick_flg = OFF ;
int		stick_read_flg = OFF ;

static int Stop=0;
int		loop_flg = OFF ;

int imax_line ;
//static double rt[MAX_LINE][NUM_LINK][DIM][DIM],tr[MAX_LINE][NUM_LINK][DIM];// rot,ori
//static double p_mkr[MAX_LINE][NUM_LINK][DIM] ; // position of marker
static double ***p_mkr ; // position of marker
//static double real_time[MAX_LINE] ; // counter time 
//static double	g_exf[MAX_LINE][2][DIM], cop[MAX_LINE][2][DIM], COG[MAX_LINE][DIM] ;
static double *real_time ; // counter time 
static double	***g_exf, ***cop ;
double sample_freq_mot, sample_freq_FP ;
static int		num_marker_line, marker_line_s[NUM_LINK], marker_line_f[NUM_LINK] ; // line bet. mkr and mkr
static int num_marker_line_white, num_marker_line_green, num_marker_line_blue ; // line color
static int instantial_axis, grf_marker_line_s[2], grf_marker_line_f[2] ; // rotational axis, grf
//static int grf_marker_line_s[2], grf_marker_line_f[2] ; // rotational axis, grf

static int num_characteristic_marker, char_marker[50] ;

double mass_coordinate[10001][DIM] ;
double psi[10001] ;
double force_vec[10001][DIM] ;
double real_time2[10001] ;

//double qXX[MAX_LINE] ;

char	model_name[100]  ;  
double	*add3( double *a, double *b, double *c ) ;


void axis (float d, float s) ;
void subaxis(float d, float s) ;
void myGround(double height) ;

void ReadSkeleton() ;
void ReadAngle() ;

// using examples from http://atlas.riken.jp/~koishi/claret.html
// mouse()
// motion()
int mouse_l = 0;
int mouse_m = 0;
int mouse_r = 0;
//int mpos[2] ;
//int trans[3], angle[3], eye_len ;
int trans[3], eye_len ;
double angle[3], mpos[2] ;

int grf_flg = 0, cog_flg = 0, mscl_flg = 1 ; // flags of Groud Reaction Force, Center of Gravity and Muscle
int flr_solid = 1 ; // flag of floor(Solid/WireFlame)
int jo_ball_flg = 1 ; // joint ball flag
int num_mkr, num_ctr ; // number of marker, number of center
int ctr[20] ; // center marker column order Number
//int num_mkr = 41 ;

double SLOPE_ANGLE = 0.0 ;

int skeletonNo[8] ;
char skeletonName[8][10] ;
double skeletonWeight[8], skeletonLength[8] ;
double skeletonCOGr[8], skeletonMomInertia[8] ;
double body_weight = 0.0 ;
//double body_cog[MAX_LINE][2] ;
double **body_cog ;


double ***mallocDouble3D ( int n0, int n1, int n2 ) {
  double ***b;
  int i,j;
  b= (double***) malloc( sizeof(double**) * n0 );
  for ( i= 0; i < n0; i++ ) {
    b[i]= (double**) malloc( sizeof(double*) * n1 );
  }
  b[0][0]= (double*) malloc( sizeof(double) * n0 * n1 * n2 );
  for ( i= 0; i < n0; i++ ) {
    for( j = 0 ; j < n1 ; j++ ){
      b[i][j]= (double*) ( b[0][0] + ( j + i*n1 )*n2 );
    }
  }
  return b;
}

void freeDouble3D ( double ***b, int n0, int n1, int n2 ) {
  int i;
  free(b[0][0]);
  for ( i=0; i < n0; i++ ) {
    free( b[i] );
  }
  free(b);
}

double **mallocDouble2D( int n0, int n1 ) {
  double **b;
  int i ;
  b= (double**) malloc( sizeof(double*) * n0 );
  for ( i= 0; i < n0; i++ ) {
    b[i]= (double*) malloc( sizeof(double) * n1 );
  }
  return b;
}

void freeDouble2D( double **b, int n0, int n1 ) {
  free( b[0] );
  free(b);
}

double *mallocDouble1D( int n0 ) {
  double *b;
  b= (double*) malloc( sizeof(double) * n0 );
  return b;
}

void freeDouble1D( double *b, int n0 ) {
  free(b);
}
/*---------------------------*/


void ReadFileInf(void)
{
	FILE *fp;
	char f_name[108],buffer[20000],*fgets();	
	int i,j,k,t,n;	
	int	d ;
	double trx, temp, tempx, tempz ;
	int num ;

	cnt = 0 ;


	  sprintf(f_name,"%s_inf.csv", model_name );
	  if ((fp=fopen(f_name,"rt"))==NULL) {
		printf("Cannnot open %s\n",f_name);
		exit(0);
	  }
	   
	for ( t = 0 ; ; t ++ ) {

		if ( fgets( buffer, 10000, fp ) == NULL ) break ;
//		printf( "%s", buffer ) ;

	    if ( buffer[0] != '#' ) {
	    if ( cnt == 0 ) {
		  sample_freq_mot = atof( strtok( buffer, "," ) ) ;
		  sample_freq_FP = atof( strtok( NULL, "," ) ) ;
	    } // buffer != "#"

	    if ( cnt == 1 ) {
		  num_mkr = atoi( strtok( buffer, "," ) ) ;
		  num_ctr = num_mkr ;
		printf( "num_mkr=%d\n", num_mkr ) ;
	    } // buffer != "#"
	    if ( cnt == 2 ) {
		 num_marker_line = atoi( strtok( buffer, "," ) ) ;
		printf( "num_marker_line=%d\n", num_marker_line ) ;
	    } // buffer != "#"
	    if ( ( cnt > 2 && cnt <= 2 + num_marker_line ) ) {
		num = atoi( strtok( buffer, "," ) ) ;
		marker_line_s[num] = atoi( strtok( NULL, "," ) ) ; 
		marker_line_f[num] = atoi( strtok( NULL, "," ) ) ;
		printf( "num=%d\n", num ) ;
	    } // buffer != "#"
			cnt ++;
	    }
	}

		for ( i = 0 ; i < num_ctr ; i ++ ) {
			ctr[i] = i ;
		}

	fclose(fp); 


}

void ReadMKR(void)
{
	FILE *fp;
	char f_name[104],buffer[20000],*fgets();	
	int i,j,k,t,n;	
	int	d ;
	double trx, temp, tempx, tempz ;
	int pelvis_num ;
	char char_temp[20] ;

	cnt = 0 ;


	  sprintf(f_name,"%s.csv", model_name );
	  if ((fp=fopen(f_name,"rt"))==NULL) {
		printf("Cannnot open %s\n",f_name);
		exit(0);
	  }

	//	printf("sprintf(f_name,.motgc, model_name );は終わったよmain.c\n");	//debug用

		fgets( buffer, 10000, fp ) ;
		  sprintf( char_temp, "%s", strtok( buffer, "," ) ) ;
		  for( i = 0 ; i < 10 ; i++ )
		  sprintf( char_temp, "%s", strtok( NULL, "," ) ) ;

		  imax_line = atoi( strtok( NULL, "," ) ) + 10 ;
		  printf( "MAX_LINE=%d\n", imax_line ) ;

		num_mkr = 3 ;
		p_mkr = mallocDouble3D( imax_line, num_mkr, DIM );
		g_exf = mallocDouble3D( imax_line, 2, DIM );
		cop = mallocDouble3D( imax_line, 2, DIM );
		body_cog = mallocDouble2D( imax_line, DIM ); 
		real_time = mallocDouble1D( imax_line ) ;

//		num_mkr = 16 ;
//		num_ctr = 16 ;

//		for ( i = 0 ; i < num_ctr ; i ++ ) {
//			ctr[i] = i ;
//		}



//	printf("instatial_axis=%d\n",instantial_axis) ;

//		printf("sprintf(f_name,.motgc, model_name );は終わったよmain.c\n");	//debug用
	for ( i = 0 ; i < 6 ; i ++ ) {
		fgets( buffer, 10000, fp ) ;
//		printf( "%s", buffer ) ;
	}

	for ( t = 0 ; ; t ++ ) {

		if ( fgets( buffer, 10000, fp ) == NULL ) break ;

	    if ( buffer[0] != '#' ) {
		  real_time[cnt] = atof( strtok( buffer, "," ) ) ;
		  real_time[cnt] = atof( strtok( NULL, "," ) ) ;
		    for( i = 0 ; i < num_mkr ; i++ ) {

		      p_mkr[cnt][i][0] = atof( strtok( NULL, "," ) ) ;
		      p_mkr[cnt][i][2] = atof( strtok( NULL, "," ) ) ;
		      p_mkr[cnt][i][1] = - 1.0 * atof( strtok( NULL, "," ) ) ;
		    }
			cnt ++;
	    } // buffer != "#"
	}
	fclose(fp); 


}

void ReadGRF(void)
{
	FILE *fp;
	char f_name[104],buffer[20000],*fgets();	
	int i,j,k,t,n;	
	int cnt2 ;
	double time, temp ;

	cnt2 = 0 ;


	  sprintf(f_name,"%s_FP.csv", model_name );
	  if ((fp=fopen(f_name,"rt"))==NULL) {
		printf("Cannnot open %s\n",f_name);
		exit(0);
	  }

	//	printf("sprintf(f_name,.motgc, model_name );は終わったよmain.c\n");	//debug用

//		g_exf = mallocDouble3D( MAX_LINE, 2, DIM );
//		cop = mallocDouble3D( MAX_LINE, 2, DIM );




//	printf("instatial_axis=%d\n",instantial_axis) ;

//		printf("sprintf(f_name,.motgc, model_name );は終わったよmain.c\n");	//debug用
	for ( i = 0 ; i < 9 ; i ++ ) {
		fgets( buffer, 10000, fp ) ;
//		printf( "%s", buffer ) ;
	}

	for ( t = 0 ; ; t ++ ) {

		if ( fgets( buffer, 10000, fp ) == NULL ) break ;

	    if ( buffer[0] != '#' ) {
		  time = atof( strtok( buffer, "," ) ) ;
		  if( time >= 1.0/ sample_freq_mot * cnt2  ) {
//			 printf( "cnt2=%d, time=%f\n", cnt2, time ) ;
		   g_exf[cnt2][0][0] = atof( strtok( NULL, "," ) ) / 1000.0;
		   g_exf[cnt2][0][1] = -1.0 * atof( strtok( NULL, "," ) ) / 1000.0;
		   g_exf[cnt2][0][2] = atof( strtok( NULL, "," ) ) / 1000.0;
		   temp = atof( strtok( NULL, "," ) ) ;
		   temp = atof( strtok( NULL, "," ) ) ;
		   temp = atof( strtok( NULL, "," ) ) ;
		   temp = atof( strtok( NULL, "," ) ) ;
		   temp = atof( strtok( NULL, "," ) ) ;
		   cop[cnt2][0][0] = -1.0 * ( atof( strtok( NULL, "," ) ) + 145.0) / 1000.0;
		   cop[cnt2][0][1] = ( atof( strtok( NULL, "," ) ) - 246.0 ) / 1000.0;
		   cop[cnt2][0][2] = 0.0 ;
		   cnt2++ ;
		  }

	    } // buffer != "#"
	    if( cnt2 >= cnt ) break ;
	}
	fclose(fp); 


}

void ReadInvPen(void)
{
	FILE *fp;
	char f_name[104],buffer[20000],*fgets();	
	int i,j,k,t,n;	
	int	d ;
	double trx, temp, tempx, tempz ;
	int pelvis_num ;
	char char_temp[20] ;

	cnt = 0 ;


	  sprintf(f_name,"%s.csv", model_name );
	  if ((fp=fopen(f_name,"rt"))==NULL) {
		printf("Cannnot open %s\n",f_name);
		exit(0);
	  }

//		printf("sprintf(f_name,.csv, model_name );は終わったよmain.c\n");	//debug用


	for ( t = 0 ; ; t ++ ) {

		if ( fgets( buffer, 10000, fp ) == NULL ) break ;

	    if ( buffer[0] != '#' ) {
		  real_time2[cnt] = atof( strtok( buffer, "," ) ) ; // time
		  psi[cnt] = atof( strtok( NULL, "," ) ) ; // psi
		  temp = atof( strtok( NULL, "," ) ) ; //dpsi
		  mass_coordinate[cnt][XX] = atof( strtok( NULL, "," ) ) ; // y
		  mass_coordinate[cnt][YY] = 0.0 ;
		  mass_coordinate[cnt][ZZ] = 0.0 ;
		  temp = atof( strtok( NULL, "," ) ) ; // dy
		  force_vec[cnt][XX] = atof( strtok( NULL, "," ) ) ; // u
		  force_vec[cnt][YY] = 0.0 ;
		  force_vec[cnt][ZZ] = 0.0 ;
			cnt ++;
	    } // buffer != "#"
	}
	fclose(fp); 

}




void myReshape(GLsizei width, GLsizei height)
{
	int i;
	GLdouble up[3];
	GLdouble vector[3];
	GLdouble norm;

	glViewport (0,0,width,height);
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity();
//	gluPerspective(18.0,(GLfloat)width/(GLfloat)height,5.0,2000.0);
	gluPerspective( 3.0,(GLfloat)width/(GLfloat)height,5.0,2000.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	for (i=0;i<3;i++){
		vector[i]=center[i]-eye[i];
	}
	up[0]=-vector[0]*vector[2];
	up[1]=-vector[1]*vector[2];
	up[2]=vector[0]*vector[0]+vector[1]*vector[1];
	norm=up[0]*up[0]+up[1]*up[1]+up[2]*up[2];
	norm=sqrt(norm);
	for (i=0;i<3;i++) up[i] /= norm;
	gluLookAt (eye[0],eye[1],eye[2],center[0],center[1],center[2],
		   up[0],up[1],up[2]);
}




/* メインの表示プログラムとなる */
void display (void) 
{
	int		i,j,m,p, k;
	double	Ang[3], tmp[DIM];
	char	filename[60] ;
	double Vector[3] ;

	extern int	clock() ;
	extern void	drawCylinderShading() ;
	extern double	veclen() ;
	float d = 0.05f, s = 2.0f;
	GLdouble color1[] =  { 0.0, 1.0, 0.0, 0.5} ;

	double v_grf[3], floor_x, floor_z ;
	int ctr_flg ;	// center marker flag
	double grf_vector_length ;


    tmbef = tmnow;
    tm = -99;
    while(tm < tmhz -1){	// リアルタイムタイマー
        tmnow = clock();
        tm = tmnow - tmbef;
    }

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushMatrix();
	glTranslatef ( BodyTranslate[0], BodyTranslate[1], BodyTranslate[2]);
	glRotatef ( (GLfloat) ALLROTATE[0], 0.0, 0.0, 1.0 );
	glRotatef ( (GLfloat) ALLROTATE[1],
		    cos((GLfloat)ALLROTATE[0] * PI/180.),
		    -sin((GLfloat)ALLROTATE[0] * PI/180.),0.0);
	
	/* floor draw */
	if ( flr_solid == ON ) { // solid
		glMaterialfv(GL_FRONT,GL_DIFFUSE,material_color4);
	  glBegin(GL_POLYGON);

//		glColor3dv(color1);
//		glNormal3f(0.,0.,1.); 
		floor_x = -9.0 * cos( SLOPE_ANGLE / 180. * PI ) ;
		floor_z = -9.0 * sin( SLOPE_ANGLE / 180. * PI ) ;
		glVertex3f(floor_x,-1.0,-floor_z);
		glVertex3f(floor_x,1.0,-floor_z);
		floor_x = 9.0 * cos( SLOPE_ANGLE / 180. * PI ) ;
		floor_z = 9.0 * sin( SLOPE_ANGLE / 180. * PI ) ;
		glVertex3f(floor_x,1.0,-floor_z);
		glVertex3f(floor_x,-1.0,-floor_z);

	  glEnd();

	} else { // wire flame
		glMaterialfv(GL_FRONT,GL_DIFFUSE,material_color4);
	  glBegin(GL_LINES);

		j = 0 ;
		/* 床が動くのを作る */

		for ( k = 0 ; k < 100 ; k ++ ) {
//		   if ( 0.5 * (double)( k ) <= ( qXX[timer] - qXX[0] ) && 0.5 * (double)( k + 1 ) > ( qXX[timer] - qXX[0] ) )
//			j = k ;
		}
		for ( i = 0 ; i < 37 ; i ++) {
//			floor_x = (-9.0 + 0.5*(double)(i+j) - ( qXX[timer] - qXX[0] )) * cos( SLOPE_ANGLE / 180. * PI ) ;
//			floor_z = (-9.0 + 0.5*(double)(i+j) - ( qXX[timer] - qXX[0] )) * sin( SLOPE_ANGLE / 180. * PI ) ;
			floor_x = (-9.0 + 0.5*(double)(i+j) ) * cos( SLOPE_ANGLE / 180. * PI ) ;
			floor_z = (-9.0 + 0.5*(double)(i+j) ) * sin( SLOPE_ANGLE / 180. * PI ) ;
			glVertex3f(floor_x,-1.0,-floor_z);
			glVertex3f(floor_x, 1.0,-floor_z);
		}

		for ( i = 0 ; i < 5 ; i ++) {
//			floor_x = (9.0 - ( qXX[timer] - qXX[0] ) + 0.5 * (double)j) * cos( SLOPE_ANGLE / 180. * PI ) ;
//			floor_z = (9.0 - ( qXX[timer] - qXX[0] ) + 0.5 * (double)j) * sin( SLOPE_ANGLE / 180. * PI ) ;
			floor_x = (9.0 + 0.5 * (double)j) * cos( SLOPE_ANGLE / 180. * PI ) ;
			floor_z = (9.0 + 0.5 * (double)j) * sin( SLOPE_ANGLE / 180. * PI ) ;
			glVertex3f(floor_x,-1.0+0.5*(double)i,-floor_z);

//			floor_x = (-9.0 - ( qXX[timer] - qXX[0] ) + 0.5 * (double)j) * cos( SLOPE_ANGLE / 180. * PI ) ;
//			floor_z = (-9.0 - ( qXX[timer] - qXX[0] ) + 0.5 * (double)j) * sin( SLOPE_ANGLE / 180. * PI ) ;
			floor_x = (-9.0 + 0.5 * (double)j) * cos( SLOPE_ANGLE / 180. * PI ) ;
			floor_z = (-9.0 + 0.5 * (double)j) * sin( SLOPE_ANGLE / 180. * PI ) ;
			glVertex3f(floor_x,-1.0+0.5*(double)i,-floor_z);
		}


	  glEnd();
	
	}

//		printf("%d,  %f\n", j, ( qXX[timer] - qXX[0] ) ) ;


     // draw BOX
		glMaterialfv(GL_FRONT,GL_DIFFUSE,material_color2 );	// green 
		glBegin(GL_POLYGON);
			glNormal3f(-1.0,0.0,0.0); // 法線の指定
			glVertex3f( mass_coordinate[timer][XX]-0.2,
				    mass_coordinate[timer][YY]+0.2,
				    mass_coordinate[timer][ZZ]
				  );
			glVertex3f( mass_coordinate[timer][XX]-0.2,
				    mass_coordinate[timer][YY]+0.2,
				    mass_coordinate[timer][ZZ]+0.2
				  );
			glVertex3f( mass_coordinate[timer][XX]-0.2,
				    mass_coordinate[timer][YY]-0.2,
				    mass_coordinate[timer][ZZ]+0.2
				  );
			glVertex3f( mass_coordinate[timer][XX]-0.2,
				    mass_coordinate[timer][YY]-0.2,
				    mass_coordinate[timer][ZZ]
				  );
		glEnd();


		glBegin(GL_POLYGON);
			glNormal3f(0.0,0.0,1.0); // 法線の指定
			glVertex3f( mass_coordinate[timer][XX]-0.2,
				    mass_coordinate[timer][YY]+0.2,
				    mass_coordinate[timer][ZZ]+0.2
				  );
			glVertex3f( mass_coordinate[timer][XX]+0.2,
				    mass_coordinate[timer][YY]+0.2,
				    mass_coordinate[timer][ZZ]+0.2
				  );
			glVertex3f( mass_coordinate[timer][XX]+0.2,
				    mass_coordinate[timer][YY]-0.2,
				    mass_coordinate[timer][ZZ]+0.2
				  );
			glVertex3f( mass_coordinate[timer][XX]-0.2,
				    mass_coordinate[timer][YY]-0.2,
				    mass_coordinate[timer][ZZ]+0.2
				  );
		glEnd();

		glBegin(GL_POLYGON);
			glNormal3f(1.0,0.0,0.0); // 法線の指定
			glVertex3f( mass_coordinate[timer][XX]+0.2,
				    mass_coordinate[timer][YY]+0.2,
				    mass_coordinate[timer][ZZ]
				  );
			glVertex3f( mass_coordinate[timer][XX]+0.2,
				    mass_coordinate[timer][YY]+0.2,
				    mass_coordinate[timer][ZZ]+0.2
				  );
			glVertex3f( mass_coordinate[timer][XX]+0.2,
				    mass_coordinate[timer][YY]-0.2,
				    mass_coordinate[timer][ZZ]+0.2
				  );
			glVertex3f( mass_coordinate[timer][XX]+0.2,
				    mass_coordinate[timer][YY]-0.2,
				    mass_coordinate[timer][ZZ]
				  );
		glEnd();

		glBegin(GL_POLYGON);
			glNormal3f(0.0,1.0,0.0); // 法線の指定
			glVertex3f( mass_coordinate[timer][XX]-0.2,
				    mass_coordinate[timer][YY]+0.2,
				    mass_coordinate[timer][ZZ]+0.2
				  );
			glVertex3f( mass_coordinate[timer][XX]+0.2,
				    mass_coordinate[timer][YY]+0.2,
				    mass_coordinate[timer][ZZ]+0.2
				  );
			glVertex3f( mass_coordinate[timer][XX]+0.2,
				    mass_coordinate[timer][YY]+0.2,
				    mass_coordinate[timer][ZZ]
				  );
			glVertex3f( mass_coordinate[timer][XX]-0.2,
				    mass_coordinate[timer][YY]+0.2,
				    mass_coordinate[timer][ZZ]
				  );
		glEnd();

		glBegin(GL_POLYGON);
			glNormal3f(0.0,1.0,0.0); // 法線の指定
			glVertex3f( mass_coordinate[timer][XX]-0.2,
				    mass_coordinate[timer][YY]-0.2,
				    mass_coordinate[timer][ZZ]+0.2
				  );
			glVertex3f( mass_coordinate[timer][XX]+0.2,
				    mass_coordinate[timer][YY]-0.2,
				    mass_coordinate[timer][ZZ]+0.2
				  );
			glVertex3f( mass_coordinate[timer][XX]+0.2,
				    mass_coordinate[timer][YY]-0.2,
				    mass_coordinate[timer][ZZ]
				  );
			glVertex3f( mass_coordinate[timer][XX]-0.2,
				    mass_coordinate[timer][YY]-0.2,
				    mass_coordinate[timer][ZZ]
				  );
		glEnd();

 // object local origin
		glMaterialfv(GL_FRONT,GL_DIFFUSE,material_color6 );	// 色を決める orange 
			drawCylinderShading( mass_coordinate[timer][XX], 
		     			     mass_coordinate[timer][YY], 
					     mass_coordinate[timer][ZZ]+0.01,
					     mass_coordinate[timer][XX],
					     mass_coordinate[timer][YY]-0.5,
					     mass_coordinate[timer][ZZ]+0.01,
		     		             0.03, 1) ; 

 // global origin
	  glPushMatrix();
		glMaterialfv(GL_FRONT,GL_DIFFUSE,material_color5 );	// 色を決める RED
	  glTranslatef(0.0, -0.3, 0.0);
		glutSolidSphere(0.02, 10, 10) ; 
	  glPopMatrix();
// draw Pendulum
		glMaterialfv(GL_FRONT,GL_DIFFUSE,material_color3); // White
			drawCylinderShading( mass_coordinate[timer][XX], 
		     			     mass_coordinate[timer][YY], 
					     mass_coordinate[timer][ZZ]+0.2,
					     mass_coordinate[timer][XX]+1.0*sin(psi[timer]),
					     mass_coordinate[timer][YY],
					     mass_coordinate[timer][ZZ]+0.2+1.0*cos(psi[timer]),
		     		             0.02, 0) ;
// draw Force arrow

		if( fabs(force_vec[timer][XX]) > 0.0001 ) {
		glMaterialfv(GL_FRONT,GL_DIFFUSE,material_colorSphVec );	// 色を決める YELLOW 
		if( force_vec[timer][XX] < 0.0 ) {
			drawCylinderShading( mass_coordinate[timer][XX]-0.2, 
		     			     mass_coordinate[timer][YY], 
					     mass_coordinate[timer][ZZ]+0.1,
					     mass_coordinate[timer][XX]-0.2+force_vec[timer][XX]/10.0,
					     mass_coordinate[timer][YY],
					     mass_coordinate[timer][ZZ]+0.1,
		     		             0.01, 0) ;
			if( fabs(force_vec[timer][XX]/10.0) > 0.05 ) {
			drawCylinderShading( mass_coordinate[timer][XX]-0.2+force_vec[timer][XX]/10.0, 
		     			     mass_coordinate[timer][YY], 
					     mass_coordinate[timer][ZZ]+0.1,
					     mass_coordinate[timer][XX]-0.2+force_vec[timer][XX]/10.0-0.05,
					     mass_coordinate[timer][YY],
					     mass_coordinate[timer][ZZ]+0.1,
		     		             0.03, 1) ;
			} else {
			drawCylinderShading( mass_coordinate[timer][XX]-0.2+force_vec[timer][XX]/10.0, 
		     			     mass_coordinate[timer][YY], 
					     mass_coordinate[timer][ZZ]+0.1,
					     mass_coordinate[timer][XX]-0.2+force_vec[timer][XX]/10.0-0.05,
					     mass_coordinate[timer][YY],
					     mass_coordinate[timer][ZZ]+0.1,
		     		             0.03*fabs(force_vec[timer][XX]/10.0/0.05), 1) ;
			}
		} else {
			if( force_vec[timer][XX] > 0.5 )
			drawCylinderShading(  
					     mass_coordinate[timer][XX]-0.2-force_vec[timer][XX]/10.0,
					     mass_coordinate[timer][YY],
					     mass_coordinate[timer][ZZ]+0.1,
		     			     mass_coordinate[timer][XX]-0.2-0.05,
					     mass_coordinate[timer][YY], 
					     mass_coordinate[timer][ZZ]+0.1,
		     		             0.01, 0) ;
			if( fabs(force_vec[timer][XX]/10.0) > 0.05 ) {
			drawCylinderShading(  
					     mass_coordinate[timer][XX]-0.2-0.05,
					     mass_coordinate[timer][YY],
					     mass_coordinate[timer][ZZ]+0.1,
		     			     mass_coordinate[timer][XX]-0.2,
					     mass_coordinate[timer][YY], 
					     mass_coordinate[timer][ZZ]+0.1,
		     		             0.03, 1) ;
			} else {
			drawCylinderShading(  
					     mass_coordinate[timer][XX]-0.2-0.05,
					     mass_coordinate[timer][YY],
					     mass_coordinate[timer][ZZ]+0.1,
		     			     mass_coordinate[timer][XX]-0.2,
					     mass_coordinate[timer][YY], 
					     mass_coordinate[timer][ZZ]+0.1,
		     		             0.03*fabs(force_vec[timer][XX]/10.0/0.05), 1) ;
			}
		}
		} // fabs() > 1.0E-8


	  glMaterialfv(GL_FRONT,GL_DIFFUSE,material_color3); // White

	  glPushMatrix();
	  glTranslatef(0.0, -1.0,-0.1);
	  glRotatef(90.0, 1.0, 0.0, 0.0);
//	  if( tmhz != 0 )
	char char_time[20] ;
	  sprintf(char_time, "Time=%8.3f", real_time2[timer] );
//	  else
//	  sprintf(char_time, "Time=?" );
	  glScalef(0.1,0.1,1.0);
	  YsDrawUglyFont( char_time ,1,0);  // center of 0/1 is centering index, 
					   //  1 is centering
//	  YsDrawUglyFont( display_name, 1,0);  // center of 0/1 is centering index, 
					   //  1 is centering
	  glPopMatrix();


//		axis(d,s);
 //		glFlush ();
	glutSwapBuffers();

//	glPopMatrix();


	glPopMatrix();

//	auxSwapBuffers();

	if ( eps_flg == 1 ) {
		sprintf( filename, "%03d.eps", timer ) ;
		generateEPS( filename,  1, WIDTH2, HEIGHT2 ); 
	}
}




void axis(float d, float s)
{
    glPushMatrix();
    subaxis(d,s);
    glPopMatrix();

    glPushMatrix();
    glRotatef(90.0, 1.0, 0.0, 0.0);
    subaxis(d,s);
    glPopMatrix();

    glPushMatrix();
    glRotatef(-90.0, 0.0, 0.0, 1.0);
    subaxis(d,s);
    glPopMatrix();
}

void subaxis(float d, float s)
{
    glTranslatef(0.0, s - 1.0, 0.0);
//    auxSolidCylinder(d, 2.0*s);
    glTranslatef(0.0, 1.0, 0.0);
    glRotatef(-90.0, 1.0, 0.0, 0.0);
//    auxSolidCone(2.0*d, 4.0*d);
    return;
}




/* 光源設定 */
void myinit (void)
{
	GLfloat light0_position[]={-5.0,0.0,5.0,0.0};
	GLfloat light1_position[]={5.0,-5.0,0.0,0.0};
	GLfloat light2_position[]={0.0,5.0,0.0,0.0};
	GLfloat light3_position[]={2.5,0.0,-5.0,0.0};

	GLfloat light0_diffuse[]={0.8,0.8,0.8,1.0};
	GLfloat light1_diffuse[]={0.8,0.8,0.8,1.0};
	GLfloat light2_diffuse[]={0.8,0.8,0.8,1.0};
	GLfloat light3_diffuse[]={0.8,0.8,0.8,1.0};

//	GLfloat light_specular[]={0.3,0.3,0.3,1.0};
	GLfloat light_specular[]={1.,1.,1.0,1.0};

	GLfloat lmodel_ambient[]={0.9,0.9,0.9,1.0};

	glLightfv(GL_LIGHT0,GL_POSITION,light0_position);
	glLightfv(GL_LIGHT0,GL_DIFFUSE,light0_diffuse);
	glLightfv(GL_LIGHT0,GL_SPECULAR,light_specular);

	glLightfv(GL_LIGHT1,GL_POSITION,light1_position);
	glLightfv(GL_LIGHT1,GL_DIFFUSE,light1_diffuse);
	glLightfv(GL_LIGHT1,GL_SPECULAR,light_specular);

	glLightfv(GL_LIGHT2,GL_POSITION,light2_position);
	glLightfv(GL_LIGHT2,GL_DIFFUSE,light2_diffuse);
	glLightfv(GL_LIGHT2,GL_SPECULAR,light_specular);

	glLightfv(GL_LIGHT3,GL_POSITION,light3_position);
	glLightfv(GL_LIGHT3,GL_DIFFUSE,light3_diffuse);
	glLightfv(GL_LIGHT3,GL_SPECULAR,light_specular);

	glLightModelfv(GL_LIGHT_MODEL_AMBIENT,lmodel_ambient);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_LIGHT2);
	glEnable(GL_LIGHT3);

	glDepthFunc(GL_LEQUAL);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_POINT_SMOOTH);
}


/* マウスの設定 */
// using examples from http://atlas.riken.jp/~koishi/claret.html
void mouse(int button, int state, int x, int y)
{
  switch (button) {
  case GLUT_LEFT_BUTTON:
    if (state == GLUT_DOWN) {
      mpos[0] = x;
      mpos[1] = y;
      mouse_l = 1;
    }
    if (state == GLUT_UP) {
      mouse_l = 0;
    }
//	printf("%d,%d,%d left \n", x, y, mouse_l ) ;		// debug用
    break;
/*
  case GLUT_MIDDLE_BUTTON:
    if (state == GLUT_DOWN) {
      mpos[0] = x;
      mpos[1] = y;
      mouse_m = 1;
    }
    if (state == GLUT_UP) {
      mouse_m = 0;
    }
	printf("%d,%d,%d middle \n", x, y, mouse_m ) ;		// debug用
    break;
*/
  case GLUT_RIGHT_BUTTON:
    if (state == GLUT_DOWN) {
      mpos[0] = x;
      mpos[1] = y;
      mouse_r = 1;
    }
    if (state == GLUT_UP) {
      mouse_r = 0;
    }
//	printf("%d,%d,%d right \n", x, y, mouse_r ) ;		// debug用
    break;
  default:
    break;
  }
}

/* マウスの動作を反映して，視点を切り替える */
// using examples from http://atlas.riken.jp/~koishi/claret.html
void motion(int x, int y)
{
  double d0;
  double len = 10.0;
/*
  len = eye_len;

  if(mouse_l == 1 && mouse_m == 1){
    trans[0] -= (double)(y-mpos[1])*len/150;
    angle[0] = -(double)(x-mpos[0])*0.2;
  } else  if(mouse_m == 1 || (mouse_l == 1 && mouse_r == 1)){
    trans[1] -= (double)(x-mpos[0])*len*.001;
    trans[2] += (double)(y-mpos[1])*len*.001;
  } else if(mouse_r == 1){
    trans[0] += (double)(y-mpos[1])*len/150;
    angle[0] =  (double)(x-mpos[0])*0.2;
  } else if(mouse_l == 1){
    d0 = len/50;
    if(d0 > 1.0) d0 = 1.0;
    angle[1] = (double)(y-mpos[1])*d0;
    angle[2] = (double)(x-mpos[0])*d0;
  }
  if(mouse_l == 1 || mouse_m == 1 || mouse_r == 1){
    mpos[0] = x;
    mpos[1] = y;
    glutPostRedisplay();
  }
*/

	if( mouse_l == 1 && mouse_r == 1 ){ // translation
//	    trans[1] -= (double)(x-mpos[0])*len*.0001;
//	    trans[2] += (double)(y-mpos[1])*len*.0001;
	    BodyTranslate[0] += (double)(x-mpos[0])*len*.001;
	    BodyTranslate[2] -= (double)(y-mpos[1])*len*.001;
	    glutPostRedisplay() ;


	} else if( mouse_r == 1 ){ // zoom in/out
//	    trans[0] += (double)(y-mpos[1])*len/150 + (double)(x-mpos[0])*len/150 ;
//	    angle[0] =  (double)(x-mpos[0])*0.2;
	    BodyTranslate[1] += (double)(y-mpos[1])*len/150 + (double)(x-mpos[0])*len/150 ;
	    glutPostRedisplay();
	} else if( mouse_l == 1 ){ // angle change aroud Y?/Z
	    d0 = len/50;
	    if(d0 > 1.0) d0 = 1.0;
	    angle[1] = (double)(y-mpos[1])*d0;
	    angle[2] = (double)(x-mpos[0])*d0;
//	printf("%f,%d,%d\n", angle[1],ALLROTATE[0],ALLROTATE[1] ) ;	// debug用
	    ALLROTATE[0] = (ALLROTATE[0] + (int)angle[2] ) % 360;
	    ALLROTATE[1] = (ALLROTATE[1] + (int)angle[1] ) % 360;
	    glutPostRedisplay();
//	printf("%f,%d,%d\n", angle[1],ALLROTATE[0],ALLROTATE[1] ) ;	// debug用
	}
	if(mouse_l == 1 || mouse_m == 1 || mouse_r == 1){
	    mpos[0] = x;
	    mpos[1] = y;
	    glutPostRedisplay() ;
//	printf("%f,%f,%d\n", mpos[0],mpos[1],ALLROTATE[1] ) ;	// debug用
	}



}


void Count_Func (void)
{
	if ( Stop && timer < cnt-1-1 ) {
		timer++ ; 
		display();
//		glutPostRedisplay();
	}
	if ( timer == cnt-1-1 ) {
		if (loop_flg == OFF ) Stop = 0;
		timer = 0;
	}
}


GLvoid *grabPixels(int inColor, unsigned int width, unsigned int height)
{
  GLvoid *buffer;
  GLint swapbytes, lsbfirst, rowlength;
  GLint skiprows, skippixels, alignment;
  GLenum format;
  unsigned int size;

  if (inColor) {
    format = GL_RGB;
    size = width * height * 3;
  } else {
    format = GL_LUMINANCE;
    size = width * height * 1;
  }

  buffer = (GLvoid *) malloc(size);
  if (buffer == NULL)
    return NULL;

  /* Save current modes. */
  glGetIntegerv(GL_PACK_SWAP_BYTES, &swapbytes);
  glGetIntegerv(GL_PACK_LSB_FIRST, &lsbfirst);
  glGetIntegerv(GL_PACK_ROW_LENGTH, &rowlength);
  glGetIntegerv(GL_PACK_SKIP_ROWS, &skiprows);
  glGetIntegerv(GL_PACK_SKIP_PIXELS, &skippixels);
  glGetIntegerv(GL_PACK_ALIGNMENT, &alignment);
  /* Little endian machines (DEC Alpha for example) could
     benefit from setting GL_PACK_LSB_FIRST to GL_TRUE
     instead of GL_FALSE, but this would require changing the
     generated bitmaps too. */
  glPixelStorei(GL_PACK_SWAP_BYTES, GL_FALSE);
  glPixelStorei(GL_PACK_LSB_FIRST, GL_FALSE);
  glPixelStorei(GL_PACK_ROW_LENGTH, 0);
  glPixelStorei(GL_PACK_SKIP_ROWS, 0);
  glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);

  /* Actually read the pixels. */
  glReadPixels(0, 0, width, height, format,
    GL_UNSIGNED_BYTE, (GLvoid *) buffer);

  /* Restore saved modes. */
  glPixelStorei(GL_PACK_SWAP_BYTES, swapbytes);
  glPixelStorei(GL_PACK_LSB_FIRST, lsbfirst);
  glPixelStorei(GL_PACK_ROW_LENGTH, rowlength);
  glPixelStorei(GL_PACK_SKIP_ROWS, skiprows);
  glPixelStorei(GL_PACK_SKIP_PIXELS, skippixels);
  glPixelStorei(GL_PACK_ALIGNMENT, alignment);
  return buffer;
}


int generateEPS(char *filename, int inColor, unsigned int width, unsigned int height)
{
  FILE *fp;
  GLvoid *pixels;
  unsigned char *curpix;
  int components, pos, i;

  pixels = grabPixels(inColor, width, height);
  if (pixels == NULL)
    return 1;
  if (inColor)
    components = 3;     /* Red, green, blue. */
  else
    components = 1;     /* Luminance. */

  fp = fopen(filename, "w");
  if (fp == NULL) {
    return 2;
  }
  fprintf(fp, "%%!PS-Adobe-2.0 EPSF-1.2\n");
  fprintf(fp, "%%%%Creator: OpenGL pixmap render output\n");
  fprintf(fp, "%%%%BoundingBox: 0 0 %d %d\n", width, height);
  fprintf(fp, "%%%%EndComments\n");
  fprintf(fp, "gsave\n");
  fprintf(fp, "/bwproc {\n");
  fprintf(fp, "    rgbproc\n");
  fprintf(fp, "    dup length 3 idiv string 0 3 0\n");
  fprintf(fp, "    5 -1 roll {\n");
  fprintf(fp, "    add 2 1 roll 1 sub dup 0 eq\n");
  fprintf(fp, "    { pop 3 idiv 3 -1 roll dup 4 -1 roll dup\n");
  fprintf(fp, "        3 1 roll 5 -1 roll put 1 add 3 0 }\n");
  fprintf(fp, "    { 2 1 roll } ifelse\n");
  fprintf(fp, "    } forall\n");
  fprintf(fp, "    pop pop pop\n");
  fprintf(fp, "} def\n");
  fprintf(fp, "systemdict /colorimage known not {\n");
  fprintf(fp, "    /colorimage {\n");
  fprintf(fp, "        pop\n");
  fprintf(fp, "        pop\n");
  fprintf(fp, "        /rgbproc exch def\n");
  fprintf(fp, "        { bwproc } image\n");
  fprintf(fp, "    } def\n");
  fprintf(fp, "} if\n");
  fprintf(fp, "/picstr %d string def\n", width * components);
  fprintf(fp, "%d %d scale\n", width, height);
  fprintf(fp, "%d %d %d\n", width, height, 8);
  fprintf(fp, "[%d 0 0 %d 0 0]\n", width, height);
  fprintf(fp, "{currentfile picstr readhexstring pop}\n");
  fprintf(fp, "false %d\n", components);
  fprintf(fp, "colorimage\n");

  curpix = (unsigned char *) pixels;
  pos = 0;
  for (i = width * height * components; i > 0; i--) {
    fprintf(fp, "%02hx", *curpix++);
    if (++pos >= 32) {
      fprintf(fp, "\n");
      pos = 0;
    }
  }
  if (pos)
    fprintf(fp, "\n");

  fprintf(fp, "grestore\n");
  free(pixels);
  fclose(fp);
  return 0;
}




void keyboard1(unsigned char key, int x, int y)
{
	char	filename[60] ;

	switch (key) {
		case 'q':
			exit(0);
		case 'Q':
			exit(0);
		case 'z':
			ALLROTATE[0] = (ALLROTATE[0] + 2) % 360 ;
			if (Stop == 0) glutPostRedisplay();
			break;
		case 'Z':
			ALLROTATE[0] = (ALLROTATE[0] - 2) % 360 ;
			if (Stop == 0) glutPostRedisplay();
			break;
		case 'x':
			ALLROTATE[1] = (ALLROTATE[1] + 2) % 360 ;
			if (Stop == 0) glutPostRedisplay();
			break;
		case 'X':
			ALLROTATE[1] = (ALLROTATE[1] - 2) % 360 ;
			if (Stop == 0) glutPostRedisplay();
			break;
		case 'i':
			BodyTranslate[1] -= 0.1;
			if (Stop == 0) glutPostRedisplay();
			break;
		case 'o':
			BodyTranslate[1] += 0.1;
			if (Stop == 0) glutPostRedisplay();
			break;
		case 's':
			if (Stop) Stop = 0;	// Stop == 1 ---> Stop = 0
			else Stop = 1; // Stop == 0 ---> Stop = 1
			break;
		case 'f':
			if ( timer >= cnt-1-1 )	timer = 0 ;
			if ( timer < cnt-1-1 )	timer++ ;
			glutPostRedisplay();
// 			printf ("%d", timer ) ;			// debug用
			break;
		case 'b':
			if ( timer <= 0 ) timer = cnt-1-1;
			if ( timer > 0 )	timer--;
			glutPostRedisplay();
			break;
		case 'p':
			sprintf( filename, "%03d.eps", timer ) ;
			generateEPS( filename,  1, WIDTH2, HEIGHT2 );
			break;
		case '\033':  /* '\033' は ESC の ASCII コード */
			exit(0);
		case 'm':
			if ( mscl_flg == ON ) mscl_flg = OFF ;
			else mscl_flg = ON ;
			glutPostRedisplay();
			break;
		case 'g':
			if ( grf_flg == ON ) grf_flg = OFF ;
			else grf_flg = ON ;
			glutPostRedisplay();
			break;
		case 'c':
			if ( cog_flg == ON ) cog_flg = OFF ;
			else cog_flg = ON ;
			glutPostRedisplay();
			break;
		case 'j':
			if ( jo_ball_flg == ON ) jo_ball_flg = OFF ;
			else jo_ball_flg = ON ;
			glutPostRedisplay();
			break;
		case 'a':
			if ( ASC_flg == OFF && ASC_read_flg == ON ) ASC_flg = ON ;
			else ASC_flg = OFF ;
			glutPostRedisplay();
			break;

		case 'A':
			if ( MARKER_flg == OFF ) MARKER_flg = ON ;
			else MARKER_flg = OFF ;
			glutPostRedisplay();
			break;
		case 'L':
			if ( MARKER_LINE_flg == OFF ) MARKER_LINE_flg = ON ;
			else MARKER_LINE_flg = OFF ;
			glutPostRedisplay();
			break;

		case 'n':
			if ( stick_flg == OFF && stick_read_flg == ON ) stick_flg = ON ;
			else stick_flg = OFF ;
			glutPostRedisplay();
			break;

		case 'w':
			if ( flr_solid == ON ) flr_solid = OFF ;
			else flr_solid = ON ;
			glutPostRedisplay();
			break;
		case '-':
			tmhz += 5 ;
			glutPostRedisplay();
			printf( "%d \n", tmhz ) ;
			break;
		case '+':
			tmhz -= 5 ;
			glutPostRedisplay();
			printf( "%d \n", tmhz ) ;
			break;
		default:
			break;

	}
}


void keyboard2(int key, int x, int y)
{

	switch (key) {

		case GLUT_KEY_LEFT:
			BodyTranslate[0] += 0.01 ;
			if (Stop == 0) glutPostRedisplay();
			break;
		case GLUT_KEY_RIGHT:
			BodyTranslate[0] -= 0.01;
			if (Stop == 0) glutPostRedisplay();
			break;
		case GLUT_KEY_UP:
			BodyTranslate[2] += 0.01 ;
			if (Stop == 0) glutPostRedisplay();
			break;
		case GLUT_KEY_DOWN:
			BodyTranslate[2] -= 0.01;
			if (Stop == 0) glutPostRedisplay();
			break;
		default:
			break;

	}
}



int main (int argc,char *argv[])
{
	int		i ;

	if(argc <= 1) {
		printf("USAGE: wvs1 -fFileName -mIntervalTime(msec)\n\n");
		exit(-1);
	}
	for (i=1; i<argc; i++) {
		if (argv[i][0] == '-') {
			switch (argv[i][1]) {
				case 'f':	strcpy(model_name, &(argv[i][2])); // file name
							break;
				case 'm':	sscanf( argv[i], "-m%d", &tmhz ); // sampling rate
							break;
				case 't':	trans_flg = OFF ;
							break ;
				case 'l':	loop_flg = ON ;
							Stop = 1; // Stop という名のフラグ Stop = 1のとき連続して動作
							break ;
				case 'p':	eps_flg = 1 ;
							break;
				case 'n':	if ( argv[i][2] == 's' )
							stick_flg = OFF ;
							break ;

				case 'c':	data_gc_flg = 1 ; // motgcというファイルを読み込む
						grf_flg = ON ;
						cog_flg = ON ;
							break;
				case 'k':	mscl_flg = OFF ;
							break;
				case 'w':	flr_solid = OFF ;
							break ;
				case 'A':	//sscanf( argv[i], "-A%d", &num_mkr ); // number_of_markers
						ASC_flg = ON ;
							break ;

				case 'P':	POS_flg = ON ;
							break ;

				case 'B':	INV_FRONT_flg = ON ;
							break ;

				default:	printf("USAGE: animation -fFileName -mIntervalTime(msec) [-t] [-p] [-A] [-w] [-l] [-B] \n\n");
							exit(-1);
							break;
			}
//				loop_flg = ON ;
//				Stop = 1; // Stop という名のフラグ Stop = 1のとき連続して動作
//				tmhz = 8 ;
				tmhz = 20 ;
						
		} 
		else { 
			printf("USAGE: animation -fFileName -mIntervalTime(msec) [-t] [-p] [-A] [-w] [-l] [-B] \n\n");
			exit(-1);
		}
	}

//		printf("read motionの前だよmain.c\n");			// debug用

//		printf("read motionは終わったよmain.c\n");		// debug用
//		ReadSkeleton() ; // read skeleton data
//		ReadAngle() ; // read angle data
//		ReadFileInf() ; // read file information data
//		ReadMKR() ; // read marker data
//		ReadGRF() ; // read grf data
		ReadInvPen() ; // inverted pendulum 
		ASC_read_flg = OFF ;




	glutInit(&argc, argv);
	glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH ); // ダブルバッファリングとデプスバッファの使用
	glutInitWindowPosition(WX, WY);
  	glutInitWindowSize(WIDTH2, HEIGHT2);
 	glutCreateWindow(argv[0]);
	myinit ();

//	glPushMatrix();
//	glTranslated(0.0,0.0,3.0);
//	YsDrawUglyFont("(0,0,3)",1);
//	glPopMatrix();


	glutMouseFunc(mouse);
  	glutMotionFunc(motion);
	glutKeyboardFunc(keyboard1);
	glutSpecialFunc(keyboard2);


	glutReshapeFunc(myReshape);
	glutIdleFunc( Count_Func );
	glutDisplayFunc(display) ;
 	glutMainLoop();

	freeDouble3D( p_mkr, imax_line, num_mkr, DIM );
	freeDouble3D( g_exf, imax_line, 2, DIM );
	freeDouble3D( cop, imax_line, 2, DIM );
	freeDouble2D( body_cog, imax_line, DIM ); 
	freeDouble1D( real_time, imax_line ) ;

 	return 0;

}


