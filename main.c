/*** Copyright (c) 2024, tomkawal
 * All rights reserved.
 *==============================
 * code by T.Kawala 2019
 * tried on Nordic board on NRF52832
 ******************************************************/

//ported to C from class LeastSqFit (LSQ_ )
//LSQ_Allocate2DArray() !!!first, with n = size; h=0;
//LSQ_Free2DArray()     !!!last

#ifdef M_TEST
int n;
long double **h;
//--------------------------------
// remove 0 on the diagonal by adding of equations.
bool LSQ_tozero ( int i )
{  for (int j = i+1; j < n; j++)
    { if (h[j][i] != 0)
       {  for (int k=0;  k <= n; k++ ) h[i][k] =  h[i][k] +  h[j][k] / h[j][i];
          return(true);
       }
    }
  return(false); //equation unresolvable
}//---------------------------------------------------
bool LSQ_hornerscheme()
{ int   idx;
  long double  t;
  for ( idx = 0; idx <= n-1; idx++)
    { if (( h[idx][idx] == 0 ) && ( LSQ_tozero(idx) == false))   return(false);
    // divide the equation so that the current element becomes 1
      if ( h[idx][idx] != 1 )
        {   t = 1 / h[idx][idx];
            h[idx][idx] = 1;
            for (int j = idx + 1; j <= n; j++ )  h[idx][j] = h[idx][j] * t;
        }
      for (int j = 0; j <=  n-1; j++)
        {  if ( (idx != j) && ( h[j][idx] != 0) )
            {  t = h[j][idx];
               for (int k = idx; k <= n; k++)  h[j][k] = h[j][k] - h[idx][k] * t;
            }
        }
    }
  return( true);
} //-----------------------------------------------
void LSQ_Allocate2DArray( void )
{  h = NULL; 
	 h = nrf_calloc(n,sizeof(long double));  //new long double*[n]; //n+1 rows
   for( int i=0; i < n; i++)  //Row by row
    {  h[i] = nrf_calloc(n+1,sizeof(long double));
			   //new long double[n+1];     //n elements of each column
    }
   for (int i=0; i < n; i++)  //n+1 rows
   for (int j=0; j < n+1; j++ )  h[i][j] = 0;
} //----------------------------------------------------
void LSQ_Free2DArray(void)  //free the allocated memory
{  //free each h[i] buffer and then h itself!!
   for( int i = 0 ; i < n ; i++ ) //rows
     nrf_free(h[i]);   // delete[] h[i]; //row by row
     nrf_free(h); //delete[] h;
}
//------------------------------------------------
void LSQ_AddValue( long double* v )
{   for (int i=0; i <  n; i++)
    for (int j=i; j <= n; j++) h[i][j] = h[i][j] + v[i] * v[j];
} //------------------------------------------------
bool LSQ_GetResult( long double* v )
{ bool ret;
    for (int i=1; i <= n-1; i++ )
    for (int j=0; j <= i; j++ )   h[i][j] = h[j][i];
    ret = LSQ_hornerscheme();
    for (int i=0; i <= n-1; i++ ) v[i] = h[i][n];
    return(ret);
}//------------------------------------


long double power(int a, int b)
{
     long double c=a;
	   if (b == 0) return 1;
     for (int n=b; n>1; n--) c*=a;
     if (b < 0) return(1/c); else return c;
}
//TEST function for LSQ:
//prepare test data
//check return: expected coefficients of least squares 
long double test_LSQ(void)
{	int  i,j,z;
  bool success;
	long double ret_val;  

  n = 3;  //ax^2+bx+C  equ order makes the size of the array!
  z = 3;  //nr of data points
  
  //CHECK that you have enough points for polynomial!
  //if (n < m)       return; //more points necessary!
	
	//double *y = nrf_calloc(n,sizeof(double)); //	double *y = new double[n];      
  //double *x = nrf_calloc(n,sizeof(double)); // double *x = new double[n];   
  long double *coef = nrf_calloc(n,sizeof(long double)); //long double *coef = new long double[n]; 
  long double *v = nrf_calloc(n,sizeof(long double)); //long double *v = new long double[n+1]; 
 
	static const int x[3]={12,15,18};
	static const int y[3]={32,36,44};


  LSQ_Allocate2DArray(); //with n as the size
  for (i=0; i < z; i++)        // all rows ( samples )
     { // v[0] = 1;  v[1] = x[i]; v[2] = x[i]^2;
        //if (n > 2) //more than a*x+b  polynomial
        // { 
			  for (j=0; j<n; j++)  v[j] = power(x[i],j);
        // }  
        v[n] = y[i]; //Y:expected result in last column
        LSQ_AddValue(v);
     }
    success = LSQ_GetResult(coef);
    LSQ_Free2DArray(); 
    

	 ret_val = coef[0];
 //nrf_free (y);
 //nrf_free (x);
 nrf_free (coef);
 nrf_free (v);
		 
  if (success)  return ret_val; else return 0;		 
}
#endif	

//in main():

#ifdef M_TEST
	//TODO: use memory manager:
	 if (nrf_mem_init() == NRF_SUCCESS) memory_OK = true; else memory_OK=false;
	
	 if (memory_OK) test_LSQ();
#endif	 
	
