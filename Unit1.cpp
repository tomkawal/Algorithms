//---------------------------------------------------------------------------
#include <vcl.h>
#pragma hdrstop

#include "Unit1.h"
//#include "LeastSq.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"
TForm1 *Form1;
//---------------------------------------------------------------------------
__fastcall TForm1::TForm1(TComponent* Owner)    : TForm(Owner) { }
//---------------------------------------------------------------------------
class LeastSqFit //: public TObject
{ public:
     LeastSqFit( int size );
     ~LeastSqFit(void);
     void AddValue( long double* v );
     bool GetResult( long double* v );
 private:
    int n;
    long double **h;
    void Allocate2DArray( void );
    void Free2DArray( void);
    bool tozero ( int i );
    bool hornerscheme( void );
};
//---------------------------------------------------
LeastSqFit::LeastSqFit(int size)
{    this->n = size;
     this->h=0;
     Allocate2DArray();
}//-------------------------------
LeastSqFit::~LeastSqFit(void)
{    Free2DArray();
}//--------------------------------
// remove 0 on the diagonal by adding of equations.
bool LeastSqFit::tozero ( int i )
{  for (int j = i+1; j < n; j++)
    { if (h[j][i] != 0)
       {  for (int k=0;  k <= n; k++ ) h[i][k] =  h[i][k] +  h[j][k] / h[j][i];
          return(true);
       }
    }
  return(false); //equation unresolvable
}//---------------------------------------------------
bool LeastSqFit::hornerscheme()
{ int   idx;
  long double  t;
  for ( idx = 0; idx <= n-1; idx++)
    { if (( h[idx][idx] == 0 ) && ( tozero(idx) == false))   return(false);
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
void LeastSqFit::Allocate2DArray( void )
{  h = new long double*[n]; //n+1 rows
   for( int i=0; i < n; i++)  //Row by row
    {  h[i] = new long double[n+1];     //n elements of each column
    }
   for (int i=0; i < n; i++)  //n+1 rows
   for (int j=0; j < n+1; j++ )  h[i][j] = 0;
} //----------------------------------------------------
void LeastSqFit::Free2DArray(void)  //free the allocated memory
{  //2 lines commented below because of old C++ Builder error!!
    // for( int i = 0 ; i < n ; i++ ) //rows
    //   delete[] h[i]; //row by row
   delete[] h;
}//------------------------------------------------
void LeastSqFit::AddValue( long double* v )
{   for (int i=0; i <  n; i++)
    for (int j=i; j <= n; j++) h[i][j] = h[i][j] + v[i] * v[j];
} //------------------------------------------------
bool LeastSqFit::GetResult( long double* v )
{ bool ret;
    for (int i=1; i <= n-1; i++ )
    for (int j=0; j <= i; j++ )   h[i][j] = h[j][i];
    ret = hornerscheme();
    for (int i=0; i <= n-1; i++ ) v[i] = h[i][n];
    return(ret);
}//------------------------------------
void __fastcall TForm1::Button_LRClick(TObject *Sender)
{ int  i,j,m,n,z;
  bool success;
  long double Ex; //Re-calc.  for verification
  m = Rgn->ItemIndex + 2; //equ order
  n = 0;                  //nr of data points
  z = sgy->RowCount - 1;  //nr of rows to test
  for (i = 1; i <= z; i++)
    if  ((sgy->Cells[0][i] != "") && ( Sgx->Cells[0][i] != "" ) ) n++;
      //CHECK you have enough points for polynomial!
  if (n < m)
     { MessageDlg( IntToStr(m)+ " or more points necessary!", mtError,
        TMsgDlgButtons() << mbOK,0);
       return;
     }
  /*
  double *Ex; //Re-calc. array, for verification
  int *x;     //data points
  int *y;     //results to correlate to

  double *coef; // Array for resulting coefficients
  double *v;    //vector to pass data values to LeastSqFit, m coeffs + 1 Y
  */

  //long double *Ex = new long double[n]; //SetLength(Ex,n);
  double *y = new double[n];      //SetLength(y,n);
  double *x = new double[n];    //SetLength(x,n);

  long double *coef = new long double[m]; //SetLength(coef,m);
  long double *v = new long double[m+1]; //SetLength(v, m+1);

   for ( i = 1; i <= n; i++ )
     { x[i-1] = StrToFloat( Sgx->Cells[0][i] );
       y[i-1] = StrToFloat( sgy->Cells[0][i] );
     }
 // LinFit
  LeastSqFit LSF(m);          //  LSF:=TLeastSquareFit.Create(m);
   for (i=0; i < n; i++)        // all rows ( samples )
     {  v[0] = 1;  v[1] = x[i];
         //more than a*x+b
        if (m > 2)
         { for (j=2; j<m; j++)
            v[j] =  IntPower( x[i], j);
         }  
        v[m] = y[i]; //Y:expected result
        LSF.AddValue(v);
     }
    success = LSF.GetResult(coef);
    LSF.~LeastSqFit(); //LSF.Free;
 //result
    ListCn->Enabled = false;
    ListCn->Items->Clear();
    for (z = 0; z < m; z++)
      ListCn->Items->Add( FloatToStr( coef[z] ) );
    ListCn->Enabled = true;
//recalc  on X
    sgc->Enabled = false;
    sgc->Items->Clear();
   for ( i=0; i<n; i++)   // all rows ( samples )
    {  Ex = coef[0] + coef[1] * x[i];
           //more than a*x+b
       if (m > 2)
         { for (j=2; j<m; j++)
           Ex = Ex + coef[j] * IntPower( x[i], j);
         }
        sgc->Items->Add( FloatToStrF( Ex,ffFixed,5,1) );
       //sgc->Cells[0][i+1] = FloatToStrF( Ex[i],ffFixed,5,1);
    }
      sgc->Enabled = true;

 //delete [] Ex;
 delete [] y;
 delete [] x;
 delete [] coef;
 delete [] v;
}
//---------------------------------------------------------------------------









