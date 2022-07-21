/*
  任意の方向を向いた円柱データの生成
*/
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/***** iida add 2000/8 *****/
#pragma warning(disable : 4244)     // MIPS
#pragma warning(disable : 4136)     // X86
#pragma warning(disable : 4051)     // ALPHA

#define drawOneLine(x1,y1,z1,x2,y2,z2) glBegin(GL_LINES); \
 glVertex3d((x1),(y1),(z1)); glVertex3d((x2),(y2),(z2)); glEnd();

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

void tran( double x, double y, double z, double m[4][4] ) ;
void rotz( double s, double c, double m[4][4] ) ;
void concatinate( double a[4][4], double b[4][4] );
void copymx(double a[4][4], double b[4][4] ) ;
void unitmx( double v[4][4] ) ;
int unitvec( double u[] ) ;
double veclen( double u[3] ) ;    /* ベクトルの長さを求める */


/*****:::::::::::::::::*****/
	
void VRMLinit(FILE *fpout){
    fprintf(fpout,"#VRML V2.0 utf8\n");
    fprintf(fpout,"Viewpoint { position 0 0 20 }\n");
}

void VRMLshvx(FILE *fpout){
    fprintf(fpout,"Shape { appearance Appearance { material Material { \n");
    fprintf(fpout,"                  diffuseColor 0.7 1 0.2 } }\n");
    fprintf(fpout," geometry IndexedFaceSet { coord Coordinate { point [ \n");
}

void VRMLvxeg(FILE *fpout){ fprintf(fpout,"   ] }  coordIndex [ \n"); } 

void VRMLegend(FILE *fpout){ fprintf(fpout," ] creaseAngle 1.5 } }\n"); }

double veclen( u )    /* ベクトルの長さを求める */
double u[];
{
      return( sqrt(u[0]*u[0]+u[1]*u[1]+u[2]*u[2]) );
}
 
double innerproduct( double a[], double b[] )  /* ベクトルの内積を求める */
{    return( a[0]*b[0]+a[1]*b[1]+a[2]*b[2] ); }

void vecpro( double a[], double b[], double c[] )
/*   c = a X b 　ベクトルの外積を求める  */
{
      c[0] = a[1]*b[2] - a[2]*b[1];
      c[1] = a[2]*b[0] - a[0]*b[2];
      c[2] = a[0]*b[1] - a[1]*b[0];
}

void direcmat( double z[], double m[][4] ) 
{         /* 本来のｚ軸方向を、引数のｚ方向へ 回転する行列を生成　*/
      double x[3],y[3],u[]={1.,0.,0.}, v[]={0.,1.,0.};
      double a[4][4];
      int i;


      unitvec( z ); // unitベクトル化
			// zXX, zYY, zZZは円柱の
      if( fabs(innerproduct(z,u)) < fabs(innerproduct(z,v)) ){ //u:x軸の単位ベクトル／v:y軸の単位ベクトル
								// zの方向がが絶対座標のy軸に近かったら（x軸よりも）
          vecpro( z, u, y ) ; // y: 単位位置ベクトルとx軸との法線ベクトル
	  unitvec( y ) ; // 
	  vecpro( y, z, x ) ; // その法線ベクトルと単位位置ベクトルの法線ベクトル
	  unitvec( x ) ; // 
			// x, y, z 局所座標軸の絶対座標に対する各方向ベクトル
      }
      else { vecpro(v,z,x); unitvec(x); vecpro(z,x,y); unitvec(y); }
      for( i=0; i<3; i++ ) {
          a[0][i]=x[i]; a[1][i]=y[i]; a[2][i]=z[i]; a[3][i]=0.; a[i][3]=0.;
      }
	      a[3][3]=1.;
/*
		xXX, xYY, xZZ, 0
		yXX, yYY, yZZ, 0
		zXX, zYY, zZZ, 0
		  0,   0,   0, 1
*/

      concatinate( a,m ); // m = a X m

/* m
		xXX, xYY, xZZ, 0
		yXX, yYY, yZZ, 0
		zXX, zYY, zZZ, 0
		CXX, CYY, CZZ, 1
これをrotzにかけると３，４列は変化無し
*/




}              


void drawCylinderShading (double tx,double ty,double tz,
                        double bx,double by,double bz , double radius, int cone_flg )
                        /*GLfloat radius, GLint division, GLfloat DD)*/
{
#include "color.h"
        /////char filename[30]="aaa.iv";
        /////int division=20; /* 円周の分割数 */
        /////float radius=10.; /* 円柱の半径 */

 double   ln[2][3]; /* 円柱の底面の中心座標 */

//FILE *fp;
double ss,cc,s1,c1,axis[3],center[3],m[4][4],sd;
double angle=0.,height;
int i, j,ii;

    int iv,iw0,iw1,iw2,iw3;

//        GLfloat radius;
        GLint division;
	GLfloat  xx,yy,zz;
        double pv[24][3];
	GLfloat vertex_data[4][3];
	GLfloat normal[3];
	double norm;
	GLfloat vec0[3],vec1[3];

//        radius = df;	// color.hで定義

	division = div;

//		radius = rr ;

        /////printf("radius=%f   division=%d    DD=%f  \n",df,div,DD);

        ln[0][0] = tx; ln[0][1] = ty; ln[0][2] = tz; // 起始点
        ln[1][0] = bx; ln[1][1] = by; ln[1][2] = bz; // 停止点

        /////printf("Output Inventor File = %s\n",filename);

        /////if( (fp=fopen(filename,"w"))<= 0 ){
        /////    printf("File %s cannot be created \n",filename);
        /////    exit(0);
        /////}

        /////VRMLinit(fp);
        /////VRMLshvx(fp);

        /////fp = fopen(filename,"w");

        /////fprintf(fp,"#Inventor V2.0 ascii\n");
        /////fprintf(fp,"Separator {\n");
        /////fprintf(fp,"#HS         graphicRendering ON\n");
        /////fprintf(fp,"#HS         hapticRendering ON\n");
        /////fprintf(fp,"    Coordinate3 {\n");
        /////fprintf(fp,"    #HS         graphicRendering OFF\n");
        /////fprintf(fp,"    #HS         hapticRendering OFF\n");
        /////fprintf(fp,"        point [\n");

        /* 円柱両端の底面の中心座標を登録 */
        ///fprintf(fp,"                %10.4f %10.4f %10.4f,\n",
        ///                            ln[1][0],ln[1][1],ln[1][2]);
        ///fprintf(fp,"                %10.4f %10.4f %10.4f,\n",
        ///                            ln[0][0],ln[0][1],ln[0][2]);
        axis[0] = ln[1][0] - ln[0][0];    /* 円柱軸方向ベクトル */
        axis[1] = ln[1][1] - ln[0][1];
        axis[2] = ln[1][2] - ln[0][2];
        center[0] = (ln[0][0] + ln[1][0])/2.; /* 円柱の中心座標値 */
        center[1] = (ln[0][1] + ln[1][1])/2.;
        center[2] = (ln[0][2] + ln[1][2])/2.;
        height = veclen( axis )/2.;       /* 円柱の高さの半分 */
        sd = (double)division;            /* 円周の分割数 */
        ii = -1; // to line 175 

        for( i=0; i<division; i++ ){

          angle = 2.*3.141592654*(double)i/sd; 
          s1 = radius*sin( angle ); c1 = radius*cos( angle );

          unitmx( m ); // 4x4の単位行列の作成
          tran( center[0],center[1],center[2],m ); //円柱の中心点位置行列の作成
          direcmat( axis,m );      /* 円柱のローカル座標＋位置行列の作成 */   /////////////////////////// 
          tran( 0.,0.,height,m );  /* 円柱の上底面へ平行移動変換 */
          rotz( s1,c1,m );         /* 円柱の中心軸まわりにmを回転変換+拡大radius分 */

          /* 円柱上稜線上の１点を記録 */
          /////fprintf(fp,"                %10.4f %10.4f %10.4f,\n",
          /////        m[0][0]+m[3][0], m[0][1]+m[3][1], m[0][2]+m[3][2]);
          ++ii; // まず0, 2 ,4

	if ( cone_flg == 0 ) {
          pv[ii][0] = m[0][0]+m[3][0]; // ローカルx軸方向ベクトルの成分＋円柱の上底面の中心x座標
          pv[ii][1] = m[0][1]+m[3][1]; // ローカルy軸方向ベクトルの成分＋円柱の上底面の中心y座標
          pv[ii][2] = m[0][2]+m[3][2]; // ローカルz軸方向ベクトルの成分＋円柱の上底面の中心z座標
	} else {
          pv[ii][0] = m[3][0]; // ローカルx軸方向ベクトルの成分＋円柱の上底面の中心x座標
          pv[ii][1] = m[3][1]; // ローカルy軸方向ベクトルの成分＋円柱の上底面の中心y座標
          pv[ii][2] = m[3][2]; // ローカルz軸方向ベクトルの成分＋円柱の上底面の中心z座標
 	}

//          unitmx( m ); // 4x4の単位行列の作成
//          tran( center[0],center[1],center[2],m ); //円柱の中心点位置行列の作成
//          direcmat( axis,m );      /* 円柱のローカル座標＋位置行列の作成 */  ///////////////////////////////
          tran( 0.,0.,-2. * height,m ); /* 円柱の下底面へ平行移動変換 */
//          rotz( s1,c1,m );         /* 円柱の中心軸まわりにmを回転変換+拡大?*/

          /* 円柱下稜線上の１点を記録 */
          /////fprintf(fp,"                %10.4f %10.4f %10.4f,\n",
          /////        m[0][0]+m[3][0], m[0][1]+m[3][1], m[0][2]+m[3][2]);
          ++ii; // 1,3
          pv[ii][0] = m[0][0]+m[3][0]; // ローカルx軸方向ベクトルの成分＋円柱の下底面の中心x座標
          pv[ii][1] = m[0][1]+m[3][1]; // ローカルy軸方向ベクトルの成分＋円柱の下底面の中心y座標
          pv[ii][2] = m[0][2]+m[3][2]; // ローカルz軸方向ベクトルの成分＋円柱の下底面の中心z座標
        }

        /////VRMLvxeg(fp);
        /////fprintf(fp,"       ]\n");
        /////fprintf(fp,"    }\n");
        /////fprintf(fp,"    IndexedFaceSet {\n");
        /////fprintf(fp,"    #HS         graphicRendering ON\n");
        /////fprintf(fp,"    #HS         hapticRendering ON\n");
        /////fprintf(fp,"    #HS         pointNormal OFF\n");
        /////fprintf(fp,"              coordIndex [\n");

        /* surface */
        /*        surface(fp,division);   */
        /////loop(fp,division);

    for( iv=0; iv<division; iv++ ){
       iw0 = iv*2;
       iw1 = iv*2 +1;
       iw2 = iv*2 +2;
       iw3 = iv*2 +3;
       if(iw2>=(division*2))   iw2=iw2-(division*2);
       if(iw3>=(division*2+1)) iw3=iw3-(division*2);
       /////fprintf(fp,"                %d, %d, %d, %d, -1,\n",iw3,iw2,iw0,iw1); 

		/////for (i=0;i<4;++i) {
		/////	m = order[j][i];
		/////	for(n=0; n<3; n++) vertex_data[i][n] = pv[m][n];
		/////}
/*****
        vertex_data[0][0]=(float)pv[iw3][0]; 
	vertex_data[0][1]=(float)pv[iw3][1]; 
	vertex_data[0][2]=(float)pv[iw3][2]; 

        vertex_data[1][0]=(float)pv[iw2][0]; 
	vertex_data[1][1]=(float)pv[iw2][1]; 
	vertex_data[1][2]=(float)pv[iw2][2]; 

        vertex_data[2][0]=(float)pv[iw0][0]; 
	vertex_data[2][1]=(float)pv[iw0][1]; 
	vertex_data[2][2]=(float)pv[iw0][2]; 

        vertex_data[3][0]=(float)pv[iw1][0]; 
	vertex_data[3][1]=(float)pv[iw1][1]; 
	vertex_data[3][2]=(float)pv[iw1][2]; 
*****/

        vertex_data[0][0]=(float)pv[iw1][0]; 
	vertex_data[0][1]=(float)pv[iw1][1]; 
	vertex_data[0][2]=(float)pv[iw1][2]; 

        vertex_data[1][0]=(float)pv[iw0][0]; 
	vertex_data[1][1]=(float)pv[iw0][1]; 
	vertex_data[1][2]=(float)pv[iw0][2]; 

        vertex_data[2][0]=(float)pv[iw2][0]; 
	vertex_data[2][1]=(float)pv[iw2][1]; 
	vertex_data[2][2]=(float)pv[iw2][2]; 

        vertex_data[3][0]=(float)pv[iw3][0]; 
	vertex_data[3][1]=(float)pv[iw3][1]; 
	vertex_data[3][2]=(float)pv[iw3][2]; 

		for (i=0;i<3;++i) {
			vec0[i]=vertex_data[3][i]-vertex_data[1][i];
			vec1[i]=vertex_data[0][i]-vertex_data[1][i];
		}

		normal[0]=vec0[1]*vec1[2]-vec0[2]*vec1[1];
		normal[1]=vec0[2]*vec1[0]-vec0[0]*vec1[2];
		normal[2]=vec0[0]*vec1[1]-vec0[1]*vec1[0];

		norm=normal[0]*normal[0]+normal[1]*normal[1]+normal[2]*normal[2];
		norm = sqrt(norm);
		for (i=0;i<3;++i) normal[i] /= norm;


		/* GL_POLYGONの描画 */
		glBegin(GL_POLYGON);
			glNormal3f(normal[0],normal[1],normal[2]); 
			for (i=0;i<4;i++){
			glVertex3f(vertex_data[i][0],
				   vertex_data[i][1],
				   vertex_data[i][2]); // 4つの頂点
			}
		glEnd();

    }

        /////fprintf(fp,"     ]\n");
        /////fprintf(fp,"    }\n");
        /////fprintf(fp,"}\n");

        /////VRMLegend(fp);
        /////fclose( fp );
}

printUsage(void)
{
  printf("USAGE: mebius [-fInventorFileName] ");
  printf( "[-dDivision] [-rRadius] [-wWidth]\n\n");
  exit(-1);
}

void unitmx( v )
	double v[4][4];
{
	int i, j;
	for( i=0; i<4; i++ ) {
    	 for( j=0; j<4; j++ ) v[i][j]=0.;
	}
        	v[0][0]=v[1][1]=v[2][2]=v[3][3]=1.;
}

int unitvec( u )
double u[];
{
      double len;

      len = veclen( u );
      if( fabs(len) < 1.e-8 ) {
           printf("Err:   Vector length is zero / %f / %f / %f / %f /\n",
                   len, u[0], u[1], u[2] );
           u[0]=1.; u[1]=0.; u[2]=0.; return 1;
      }
      u[0] /= len; u[1] /= len; u[2] /= len;
      return 0;
}

/* 4x4行列同士の行列積 c = a X b */
void multmx( a,b,c )
	double a[4][4],b[4][4],c[4][4];
{	
	int i,j;

	for( i=0; i<4; i++ ) {
    		for( j=0; j<4; j++ )
		c[i][j]=a[i][0]*b[0][j]+a[i][1]*b[1][j]+a[i][2]*b[2][j]
			+ a[i][3]*b[3][j];
	}
}

void tran( x,y,z,m )
double x,y,z,m[4][4];
{
double a[4][4];

a[0][0]=1.; a[0][1]=0.; a[0][2]=0.; a[0][3]=0.;
a[1][0]=0.; a[1][1]=1.; a[1][2]=0.; a[1][3]=0.;
a[2][0]=0.; a[2][1]=0.; a[2][2]=1.; a[2][3]=0.;
a[3][0]=x;  a[3][1]=y;  a[3][2]=z;  a[3][3]=1.;
concatinate( a,m );
}

void rotx( s,c,m )
double s,c,m[4][4];
{
double a[4][4];

a[0][0]=1.; a[0][1]=0.; a[0][2]=0.; a[0][3]=0.;
a[1][0]=0.; a[1][1]=c;  a[1][2]=s;  a[1][3]=0.;
a[2][0]=0.; a[2][1]=-s; a[2][2]=c;  a[2][3]=0.;
a[3][0]=0.; a[3][1]=0.; a[3][2]=0.; a[3][3]=1.;
concatinate( a,m );
}

void roty( s,c,m )
double s,c,m[4][4];
{
double a[4][4];

a[0][0]=c;  a[0][1]=0.; a[0][2]=-s; a[0][3]=0.;
a[1][0]=0.; a[1][1]=1.; a[1][2]=0.; a[1][3]=0.;
a[2][0]=s;  a[2][1]=0.; a[2][2]=c;  a[2][3]=0.;
a[3][0]=0.; a[3][1]=0.; a[3][2]=0.; a[3][3]=1.;
concatinate( a,m );
}

void rotz( s,c,m )
double s,c,m[4][4];
{
double a[4][4];

a[0][0]=c;  a[0][1]=s;  a[0][2]=0.; a[0][3]=0.;
a[1][0]=-s; a[1][1]=c;  a[1][2]=0.; a[1][3]=0.;
a[2][0]=0.; a[2][1]=0.; a[2][2]=1.;  a[2][3]=0.;
a[3][0]=0.; a[3][1]=0.; a[3][2]=0.; a[3][3]=1.;
concatinate( a,m );
}


void scale( x,y,z,m )
double x,y,z,m[4][4];
{
double a[4][4];

a[0][0]=x;  a[0][1]=0.; a[0][2]=0.; a[0][3]=0.;
a[1][0]=0.; a[1][1]=y;  a[1][2]=0.; a[1][3]=0.;
a[2][0]=0.; a[2][1]=0.; a[2][2]=z;  a[2][3]=0.;
a[3][0]=0.; a[3][1]=0.; a[3][2]=0.; a[3][3]=1.;
concatinate( a,m );
}

/* 行列aと行列bの積をbで返す */
void concatinate( a,b )
double a[4][4],b[4][4];
{
double z[4][4];

multmx( a,b,z ); // z = a X b
copymx( z,b ); // b = z 代入
}

/* 行列aをbに代入 */
void copymx( a,b )
double a[4][4],b[4][4];
{
b[0][0]=a[0][0]; b[0][1]=a[0][1]; b[0][2]=a[0][2]; b[0][3]=a[0][3];
b[1][0]=a[1][0]; b[1][1]=a[1][1]; b[1][2]=a[1][2]; b[1][3]=a[1][3];
b[2][0]=a[2][0]; b[2][1]=a[2][1]; b[2][2]=a[2][2]; b[2][3]=a[2][3];
b[3][0]=a[3][0]; b[3][1]=a[3][1]; b[3][2]=a[3][2]; b[3][3]=a[3][3];
}

void writeline( fp,v,k,m )
FILE *fp;
double v[][3];
int k;
double m[][4];
{
double x1,y1,z1,x2,y2,z2;

k *= 2;
x1 = v[0][0]*m[0][0]+v[0][1]*m[1][0]+v[0][2]*m[2][0]+m[3][0];
y1 = v[0][0]*m[0][1]+v[0][1]*m[1][1]+v[0][2]*m[2][1]+m[3][1];
z1 = v[0][0]*m[0][2]+v[0][1]*m[1][2]+v[0][2]*m[2][2]+m[3][2];
x2 = v[1][0]*m[0][0]+v[1][1]*m[1][0]+v[1][2]*m[2][0]+m[3][0];
y2 = v[1][0]*m[0][1]+v[1][1]*m[1][1]+v[1][2]*m[2][1]+m[3][1];
z2 = v[1][0]*m[0][2]+v[1][1]*m[1][2]+v[1][2]*m[2][2]+m[3][2];
fprintf(fp,"                %10.4f %10.4f %10.4f,\n",x1,y1,z1);
fprintf(fp,"                %10.4f %10.4f %10.4f,\n",x2,y2,z2);
}

void loop( FILE *fp, int n)
{
    int i,j;
    int iv,iw0,iw1,iw2,iw3;

    for( iv=0; iv<n; iv++ ){

       iw0 = iv*2;
       iw1 = iv*2 +1;
       iw2 = iv*2 +2;
       iw3 = iv*2 +3;

       if(iw2>=(n*2))   iw2=iw2-(n*2);
       if(iw3>=(n*2+1)) iw3=iw3-(n*2);
       fprintf(fp,"                %d, %d, %d, %d, -1,\n",iw3,iw2,iw0,iw1); 

    }


}

void surface( fp , division)
FILE *fp;
int division;
{
int i,iv,iw0,iw1,iw2;
    for(iv=0; iv<division; iv++){
    for(i=0; i<2; i++){
      if(i==0){
       iw0 = iv*2;
       iw1 = iv*2 +1;
       iw2 = iv*2 +2;
       if(iw2>=(division*2)) iw2=iw2-(division*2-1); 
       fprintf(fp,"                %d, %d, %d, -1,\n",iw0,iw1,iw2);
      }
      else{
       iw0 = iv*2 +1;
       iw1 = iv*2 +3;
       if(iw1>=(division*2+1)) iw1=iw1-(division*2+1); 
       iw2 = iv*2 +2;
       if(iw2>=(division*2)) iw2=iw2-(division*2-1); 
       fprintf(fp,"                %d, %d, %d, -1,\n",iw0,iw1,iw2); 
      }
    }
    }
}
