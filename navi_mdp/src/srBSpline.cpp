/*-----------------------------------------------------------
srBspline algorithms version 1.0    -2016.10.21 MK
-------------------------------------------------------------*/
#include <stdlib.h>
#include <math.h>
#include "utils.h"
#include "types.h"
#include "srBSpline.h"
#include <assert.h>
//////////////////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////////////////
srBSpline::srBSpline( void )
	: m_Knots(NULL)
	, m_ControlPoints(NULL)
	, m_Degree(0)
	, m_NumCPs(0)
	, m_NumKnots(0)
{

}

srBSpline::srBSpline( int Degree, int NumCPs, double* CPs, int NumKnots, double* Knots )
	: m_Knots(NULL)
	, m_ControlPoints(NULL)
	, m_Degree(0)
	, m_NumCPs(0)
	, m_NumKnots(0)
{
	Set(Degree, NumCPs, CPs, NumKnots, Knots);
}


srBSpline::srBSpline( int NumCPs, double Ti, double Tf, int Degree, bool KnotRepeat /*= true*/ )
	: m_Knots(NULL)
	, m_ControlPoints(NULL)
	, m_Degree(0)
	, m_NumCPs(0)
	, m_NumKnots(0)
{
	Set(Degree, NumCPs, Ti, Tf, KnotRepeat);
}


srBSpline::srBSpline( int Degree, int NumCPs, double* CPs, double Ti, double Tf, bool KnotRepeat /*= true */ )
	: m_Knots(NULL)
	, m_ControlPoints(NULL)
	, m_Degree(0)
	, m_NumCPs(0)
	, m_NumKnots(0)
{
	Set(Degree, NumCPs, CPs, Ti, Tf, KnotRepeat);
}

srBSpline::srBSpline( const srBSpline& T )
{
	if(&T != this)
	{
		m_Degree	= T.m_Degree;
		m_NumCPs	= T.m_NumCPs;
		m_NumKnots  = T.m_NumKnots;

		m_ControlPoints= new double [m_NumCPs];
		m_Knots = new double [m_NumKnots];

		for(int i=0 ; i< m_NumCPs;++i)
			m_ControlPoints[i]=T.m_ControlPoints[i];

		for(int i=0 ; i< m_NumKnots;++i)
			m_Knots[i]=T.m_Knots[i];
	}
}


const srBSpline& srBSpline::operator=( const srBSpline& T )
{
	if(&T != this)
	{
		m_Degree	= T.m_Degree;
		m_NumCPs	= T.m_NumCPs;
		m_NumKnots  = T.m_NumKnots;

		m_ControlPoints= new double [m_NumCPs];
		m_Knots = new double [m_NumKnots];

		for(int i=0 ; i< m_NumCPs;++i)
			m_ControlPoints[i]=T.m_ControlPoints[i];

		for(int i=0 ; i< m_NumKnots;++i)
			m_Knots[i]=T.m_Knots[i];
	}

	return (*this);
}


srBSpline::~srBSpline()
{
	_Clear();
}

void srBSpline::Set( int Degree, int NumCPs, double* CPs, int NumKnots, double* Knots )
{
	_Clear();

	int _i = 0;
	
	m_Degree = Degree;

	m_NumCPs = NumCPs;
	m_ControlPoints = new double[NumCPs];
	for (_i = 0; _i < m_NumCPs; ++_i)
		m_ControlPoints[_i] = CPs[_i];

	m_NumKnots = NumKnots;
	m_Knots = new double[NumKnots];
	for (_i = 0; _i < m_NumKnots; ++_i)
		m_Knots[_i] = Knots[_i];
}

void srBSpline::Set( int Degree, int NumCPs, double* CPs, double Ti, double Tf , bool KnotRepeat /*= true*/ )
{
	_Clear();

	int _i = 0;

	// degree
	m_Degree = Degree;

	// control points
	m_NumCPs = NumCPs;
	m_ControlPoints = new double[NumCPs];
	for (_i = 0; _i < m_NumCPs; ++_i)
		m_ControlPoints[_i] = CPs[_i];

	// knots
	m_NumKnots = NumCPs + Degree + 1;
	m_Knots = new double[m_NumKnots];
	if (KnotRepeat)
		_CalcKnot(m_Knots, m_NumKnots, Ti, Tf, m_Degree + 1, m_Degree + 1);
	else
		_CalcKnot(m_Knots, m_NumKnots, Ti, Tf);
}

void srBSpline::Set( int Degree, int NumCPs, double Ti, double Tf , bool KnotRepeat /*= true*/ )
{
	_Clear();

	int _i = 0;

	// degree
	m_Degree = Degree;

	// control points
	m_NumCPs = NumCPs;
	m_ControlPoints = new double[NumCPs];
	for (_i = 0; _i < m_NumCPs; ++_i)
		m_ControlPoints[_i] = 0.0;

	// knots
	m_NumKnots = NumCPs + Degree + 1;
	m_Knots = new double[m_NumKnots];
	if (KnotRepeat)
		_CalcKnot(m_Knots, m_NumKnots, Ti, Tf, m_Degree + 1, m_Degree + 1);
	else
		_CalcKnot(m_Knots, m_NumKnots, Ti, Tf);
}


void srBSpline::BasisFuns( OUT double* N, double u )
{
	//Original
	/*int _span = FindSpan(u);
	BasisFuns(N, _span, u);*/

	int _span;

	if(findSpan(_span,u))
	{
		BasisFuns(N, _span, u);
	}

}


/* Compute the nonvanishing basis functions */
/* Input: i,u,p,U */
/* Output: N */
void srBSpline::BasisFuns( OUT double* N, int span, double u )
{
	//assert(sizeof(N) == (m_Degree + 1));

	int _j, _r;
	double _left = 0.0;
	double _right = 0.0;
	double _saved = 0.0;
	double _temp = 0.0;

	N[0] = 1.0;
	for (_j = 1; _j <= m_Degree; ++_j)
	{
		_saved = 0.0;
		for (_r = 0; _r < _j; ++_r)
		{
			_left = _Left(span, _j - _r, u);
			_right = _Right(span, _r + 1, u);

			//std::cout<<"_left : "<<_left <<", _right :" <<_right<<std::endl;
			
			if((_right+_left)!=0)
				_temp = N[_r]/(_right + _left);

			N[_r] = _saved + _right*_temp;
			_saved = _left*_temp;
		}
		N[_j] = _saved;
	}
}

bool srBSpline::BasisFunsDers( OUT double** ders, double u, int n )
{
	//int _span = FindSpan(u);

	int _span;
	if(!findSpan(_span,u))
		return false;

	BasisFunsDers(ders, _span, u, n);

	return true;

}

bool srBSpline::BasisFunsDers( OUT double** ders, int span, double u, int n )
{
	int _j, _r, _k;
	int _s1, _s2;
	int _j1, _j2;
	int _rk;
	int _pk;

	double _saved = 0.0;
	double _left = 0.0;
	double _right = 0.0;
	double _temp = 0.0;
	double _d = 0.0;

	// to store the basis functions and knot differences
	double** _ndu = new double*[m_Degree + 1];
	for (_j = 0; _j <= m_Degree; ++_j)
		_ndu[_j] = new double[m_Degree + 1];			
	// to store (in an alternating fashion) the two most recently computed
	// rows a(k,j) and a(k-1,j)
	double** _a = new double*[2];
	for (_j = 0; _j < 2; ++_j)
		_a[_j] = new double[m_Degree+1];

	_ndu[0][0] = 1.0;
	for (_j = 1; _j <= m_Degree; ++_j)
	{
		_saved = 0.0;
		for (_r = 0; _r < _j; ++_r)
		{
			_left = _Left(span, _j - _r, u);
			_right = _Right(span, _r + 1, u);

			// Lower triangle
			_ndu[_j][_r] = _right + _left;
			_temp = _ndu[_r][_j-1]/_ndu[_j][_r];

			// Upper triangle
			_ndu[_r][_j] = _saved + _right*_temp;
			_saved = _left*_temp;
		}
		_ndu[_j][_j] = _saved;
	}

	// Load the basis functions
	for (_j = 0; _j <= m_Degree; ++_j)
		ders[0][_j] = _ndu[_j][m_Degree];

	// This section computes the derivatives (Eq. [2.9])
	for (_r = 0; _r <= m_Degree; ++_r)
	{
		_s1 = 0;
		_s2 = 1;
		_a[0][0] = 1.0;

		// Loop to compute k-th derivative
		for (_k = 1; _k <= n; ++_k)
		{
			_d = 0.0;
			_rk = _r - _k;
			_pk = m_Degree - _k;

			if (_r >= _k)
			{
				_a[_s2][0] = _a[_s1][0]/_ndu[_pk+1][_rk];
				_d = _a[_s2][0]*_ndu[_rk][_pk];
			}

			if (_rk >= -1)
				_j1 = 1;
			else
				_j1 = -_rk;

			if (_r - 1 <= _pk)
				_j2 = _k - 1;
			else
				_j2 = m_Degree - _r;

			for (_j = _j1; _j <= _j2; ++_j)
			{
				_a[_s2][_j] = (_a[_s1][_j] - _a[_s1][_j-1])/_ndu[_pk+1][_rk+_j];
				_d += _a[_s2][_j]*_ndu[_rk+_j][_pk];
			}

			if (_r <= _pk)
			{
				_a[_s2][_k] = -_a[_s1][_k-1]/_ndu[_pk+1][_r];
				_d += _a[_s2][_k]*_ndu[_r][_pk];
			}
			ders[_k][_r] = _d;

			// Switch rows
			_j = _s1;
			_s1 = _s2;
			_s2 = _j;
		}
	}

	// Multiply through by the correct factors
	// (Eq. [2.9])
	_r = m_Degree;
	for (_k = 1; _k <= n; ++_k)
	{
		for (_j = 0; _j <= m_Degree; ++_j)
			ders[_k][_j] *= _r;

		_r *= (m_Degree - _k);
	}

	// Deallocate
	for (_j = 0; _j <= m_Degree; ++_j)
		delete[] _ndu[_j];
	delete[] _ndu;
	for (_j = 0; _j < 2; ++_j)
		delete[] _a[_j];
	delete[] _a;

	return true;

}

double srBSpline::OneBasisFun( int i, double u )
{
	int _j, _k;
	double _saved = 0.0;
	double _temp = 0.0;
	double _Uleft = 0.0;
	double _Uright = 0.0;
	double _Nip = 0.0;
	double* _N = new double[m_Degree+1];

	// Special cases
	if ((i == 0 && u == m_Knots[0])
		|| (i == m_NumKnots - 1 - m_Degree - 1 && u == m_Knots[m_NumKnots - 1]))
	{
		_Nip = 1.0;
		return _Nip;
	}

	// Local property
	if (u < m_Knots[i] || u >= m_Knots[i + m_Degree + 1])
	{
		_Nip = 0.0;
		return _Nip;
	}

	// Initialize zeroth-degree functions
	for (_j = 0; _j <= m_Degree; ++_j)
	{
		if (u >= m_Knots[i + _j] && u < m_Knots[i + _j + 1])
			_N[_j] = 1.0;
		else
			_N[_j] = 0.0;
	}

	// Compute triangular table
	for (_k = 1; _k <= m_Degree; ++_k)
	{
		if (_N[0] == 0.0)
			_saved = 0.0;
		else
			_saved = (u - m_Knots[i]*_N[0])/(m_Knots[i + _k] - m_Knots[i]);

		for (_j = 0; _j < m_Degree - _k + 1; ++_j)
		{
			_Uleft = m_Knots[i + _j + 1];
			_Uright = m_Knots[i + _j + _k + 1];
			if (_N[_j + 1] == 0.0)
			{
				_N[_j] = _saved;
				_saved = 0.0;
			}
			else
			{
				_temp = _N[_j + 1]/(_Uright - _Uleft);
				_N[_j] = _saved + (_Uright - u)*_temp;
				_saved = (u - _Uleft)*_temp;
			}
		}
	}

	_Nip = _N[0];

	delete[] _N;

	return _Nip;
}


void srBSpline::OneBasisFunDers( OUT double* ders, int i, double u, int n )
{
	int _j, _k, _jj;

	double _saved = 0.0;
	double _temp = 0.0;
	double _Uleft = 0.0;
	double _URight = 0.0;

	double** _N = new double*[m_Degree+1];
	for (_j = 0; _j < m_Degree + 1; ++_j)
		_N[_j] = new double[m_Degree+1];
	double* _ND = new double[n+1];		// check!

	// Local property
	if (u < m_Knots[i] || u >= m_Knots[i+m_Degree+1])
	{
		for (_k = 0; _k <= n; ++_k)
		{
			ders[_k] = 0.0;
		}
		return;
	}

	// Initialize zeroth-degree functions
	for (_j = 0; _j <= m_Degree; ++_j)
	{
		if (u >= m_Knots[i + _j] && u < m_Knots[i + _j + 1])
			_N[_j][0] = 1.0;
		else
			_N[_j][0] = 0.0;
	}

	// Compute full triangular table
	for (_k = 1; _k <= m_Degree; ++_k)
	{
		if (_N[0][_k-1] == 0.0)
			_saved = 0.0;
		else
			_saved = ((u - m_Knots[i])*_N[0][_k-1])/(m_Knots[i+_k] - m_Knots[i]);

		for (_j = 0; _j < m_Degree - _k + 1; _j++)
		{
			_Uleft = m_Knots[i+_j+1];
			_URight = m_Knots[i+_j+_k+1];

			if (_N[_j+1][_k-1] == 0.0)
			{
				_N[_j][_k] = _saved;
				_saved = 0.0;
			}
			else
			{
				_temp = _N[_j+1][_k-1]/(_URight - _Uleft);
				_N[_j][_k] = _saved + (_URight - u)*_temp;
				_saved = (u - _Uleft)*_temp;
			}
		}
	}

	// The function value
	ders[0] = _N[0][m_Degree];

	// Compute the derivatives
	for (_k = 1; _k <= n; ++_k)
	{
		// Load appropriate column
		for (_j = 0; _j <= _k; ++_j)
			_ND[_j] = _N[_j][m_Degree-_k];

		// Compute table of width k
		for (_jj = 1; _jj <= _k; ++_jj)
		{
			if (_ND[0] == 0.0)
				_saved = 0.0;
			else
				_saved = _ND[0]/(m_Knots[i+m_Degree-_k+_jj] - m_Knots[i]);

			for (_j = 0; _j < _k - _jj + 1; ++_j)
			{
				_Uleft = m_Knots[i+_j+1];
				_URight = m_Knots[i+_j+m_Degree+_jj+1];

				if (_ND[_j+1] == 0.0)
				{
					_ND[_j] = (m_Degree - _k + _jj)*_saved;
					_saved = 0.0;
				}
				else
				{
					_temp = _ND[_j+1]/(_URight - _Uleft);
					_ND[_j] = (m_Degree - _k + _jj)*(_saved - _temp);
					_saved = _temp;
				}
			}
		}

		// k-th derivative
		ders[_k] = _ND[0];
	}


	for (_j = 0; _j < m_Degree + 1; ++_j)
		delete[] _N[_j];
	delete[] _N;

	delete[] _ND;
}

double srBSpline::OneBasisFunDers( int i, double u, int n )
{
	double* _ders = new double[n+1];
	
	OneBasisFunDers(_ders, i, u, n);
	
	double _result = _ders[n];

	// 
	delete[] _ders;
	
	return _result;
}


void srBSpline::_Clear( void )
{
	m_Degree = 0;
	m_NumKnots = 0;
	m_NumCPs = 0;

	SR_SAFE_DELETE_AR(m_Knots);
	SR_SAFE_DELETE_AR(m_ControlPoints);
}

inline double srBSpline::_Left( int i, int j, double u )
{
	return u - m_Knots[i + 1 - j];
}

inline double srBSpline::_Right( int i, int j, double u )
{
	return m_Knots[i + j] - u;
}

inline void srBSpline::_Linspace( OUT double* Array, double Ti, double Tf, int NumData )
{
	assert(NumData > 0);

	int _NumData_1 = NumData - 1;
	double _TimeStep = (Tf - Ti)/_NumData_1;

	Array[0] = Ti;
	for (int _i = 1; _i < _NumData_1; ++_i)
		Array[_i] = Array[_i - 1] + _TimeStep;
	Array[_NumData_1] = Tf;
}

inline void srBSpline::_CalcKnot( OUT double* Knot, int NumKnots, double Ti, double Tf, int NumRepeat1, int NumRepeat2 )
{
	assert(NumKnots >= NumRepeat1 + NumRepeat2);

	int _i = 0;
	int _j = 0;
	int _NumMidKnot = NumKnots - NumRepeat1 - NumRepeat2;
	double _TimeStep = (Tf - Ti)/(_NumMidKnot + 1);

	// augment knot sequence for the initial part, # of order ( degree + 1 )
	for (_j = 0; _j < NumRepeat1; ++_j){
		Knot[_i] = Ti;
		_i++;
		}
		std::cout<<"_i"<< _i <<", _j"<<_j<<std::endl;

	// uniform knot sequence for the middle part, #: NumKnot - degree - degree = NumKnot - order - order + 2
	for (_j = 0; _j < _NumMidKnot; ++_j)
		{
			//Knot[_i] = Knot[_i++ - 1] + _TimeStep;
			Knot[_i] = Knot[_i- 1] + _TimeStep;
			_i++;
			std::cout<<"_i"<< _i <<", _j"<<_j<<std::endl;
		}

	// augment knot sequence for the final part, # of order ( degree + 1 )
	for (_j = 0; _j < NumRepeat2; ++_j)
		Knot[_i++] = Tf;
	  std::cout<<"_i"<< _i <<", _j"<<_j<<std::endl;
}


void srBSpline::_CalcKnot2( OUT double* Knot, int NumKnots, double* t, int NumRepeat1 /*= 4*/, int NumRepeat2 /*= 4*/ )
{
	/*Cubic interpolation CalcKnot*/
	/*knots are at given points  */

	assert(NumKnots >= NumRepeat1 + NumRepeat2);

	int _i = 0;
	int _j = 0;
	int _NumMidKnot = NumKnots - NumRepeat1 - NumRepeat2;
	int _NumGivenPoints = _NumMidKnot+2;
	double _TimeStep =0.0;

	// augment knot sequence for the initial part, # of order ( degree + 1 )
	for (_j = 0; _j < NumRepeat1; ++_j)
	{	Knot[_i] =t[0];
		_i++;
	}


	// uniform knot sequence for the middle part, #: NumKnot - degree - degree = NumKnot - order - order + 2
	for (_j = 0; _j < _NumMidKnot; ++_j)
	{
		_TimeStep = (t[_j+1]-t[_j]);
		Knot[_i] = Knot[_i - 1] + _TimeStep;
		_i++;

	}
	// augment knot sequence for the final part, # of order ( degree + 1 )
	for (_j = 0; _j < NumRepeat2; ++_j)
	{
		Knot[_i] = t[_NumGivenPoints-1];
		_i++;
	}	

	// std::cout<<"Knots"<<std::endl;
	// for(int k=0;k<NumKnots;k++)
	// 	std::cout<<Knot[k]<<std::endl;

}


void srBSpline::_CalcKnot2( OUT double* Knot, int NumKnots, std::vector<double>& t, int NumRepeat1 /*= 4*/, int NumRepeat2 /*= 4*/ )
{
	/*Cubic interpolation CalcKnot*/
	/*knots are at given points  */

	assert(NumKnots >= NumRepeat1 + NumRepeat2);

	int _i = 0;
	int _j = 0;
	int _NumMidKnot = NumKnots - NumRepeat1 - NumRepeat2;
	int _NumGivenPoints = _NumMidKnot+2;
	double _TimeStep =0.0;

	// augment knot sequence for the initial part, # of order ( degree + 1 )
	for (_j = 0; _j < NumRepeat1; ++_j)
	{	Knot[_i] =t[0];
		_i++;
	}


	// uniform knot sequence for the middle part, #: NumKnot - degree - degree = NumKnot - order - order + 2
	for (_j = 0; _j < _NumMidKnot; ++_j)
	{
		_TimeStep = (t[_j+1]-t[_j]);
		Knot[_i] = Knot[_i - 1] + _TimeStep;
		_i++;
	}
	// augment knot sequence for the final part, # of order ( degree + 1 )
	for (_j = 0; _j < NumRepeat2; ++_j)
	{
		Knot[_i] = t[_NumGivenPoints-1];
		_i++;
	}

	// 	std::cout<<"Knots"<<std::endl;
	// for(int k=0;k<NumKnots;k++)
	// 	std::cout<<Knot[k]<<std::endl;


}

void srBSpline::_CalcKnot3( OUT double* Knot, int NumKnots, std::vector<double>& t, int NumRepeat1 /*= 4*/, int NumRepeat2 /*= 4*/ )
{
	/*Cubic interpolation CalcKnot*/
	/*knots are at given points  */

	assert(NumKnots >= NumRepeat1 + NumRepeat2);

	int _i = 0;
	int _j = 0;
	int _NumMidKnot = NumKnots - NumRepeat1 - NumRepeat2;
	int _NumGivenPoints = _NumMidKnot+2;
	double _TimeStep =0.0;

	// augment knot sequence for the initial part, # of order ( degree + 1 )
	for (_j = 0; _j < NumRepeat1; ++_j)
		Knot[_i++] =t[0];

	// uniform knot sequence for the middle part, #: NumKnot - degree - degree = NumKnot - order - order + 2
	for (_j = 0; _j < _NumMidKnot; ++_j)
	{
		_TimeStep = (t[_j+1]-t[_j]);
		Knot[_i] = Knot[_i++ - 1] + _TimeStep;
	}
	// augment knot sequence for the final part, # of order ( degree + 1 )
	for (_j = 0; _j < NumRepeat2; ++_j)
		Knot[_i++] = t[_NumGivenPoints-1];
}

bool srBSpline::getCurvePoint( double& ret, double u )
{
	int _span;

	if(u<m_Knots[0])
		u = m_Knots[0];
	else if ( u>m_Knots[m_NumKnots-1])
		u=m_Knots[m_NumKnots-1];

	if (!findSpan(_span, u))
		return false;

	double* _N = new double[m_Degree+1];

	BasisFuns(_N, _span, u);

	//mk
	// for(int k(0);k<m_Degree+1;k++)
	// {	
	// 	std::cout<<"N[k] :"<<_N[k]<<std::endl;
	// 	std::cout<<"Cp(k) :"<<m_ControlPoints[k]<<std::endl;	

 //    }
	
	//

	double _C = 0.0;
	for (int i = 0; i <= m_Degree; ++i)
		_C += _N[i]*m_ControlPoints[_span-m_Degree+i];

	delete[] _N;

	ret = _C;

	return true;
}


bool srBSpline::CurveDerivsAlg1( OUT double* CK, double u, int d )
{
	assert(d <= m_Degree);

	int _k = 0;
	int _j = 0;

	double** _nders = new double*[d+1];
	for (_k = 0; _k < d+1; ++_k)
		_nders[_k] = new double[m_Degree+1];

	int _span;
	 
	if (!findSpan(_span, u))
		return false;
		
	BasisFunsDers(_nders, _span, u, d);

	for (_k = 0; _k <= d; ++_k)
	{
		CK[_k] = 0.0;
		for (_j = 0; _j <= m_Degree; ++_j)
		{
			CK[_k] += _nders[_k][_j]*m_ControlPoints[_span-m_Degree+_j];
		}
	}

	for (_k = 0; _k < d+1; ++_k)
		delete[] _nders[_k];
	delete[] _nders;

	return true;

}

void srBSpline::CurveDerivCpts( OUT double** PK /* PK[d+1][r2-r1+1] */
	, int d /* highest derivative degree */
	, int r1 /* */
	, int r2 /* */ )
{
	int _r = r2 - r1;
	int _i = 0;
	int _k = 0;
	int _temp = 0;

	for (_i = 0; _i < _r; ++_i)
		PK[0][_i] = m_ControlPoints[r1+_i];

	for (_k = 1; _k <= d; ++_k)
	{
		_temp = m_Degree - _k + 1;
		for (_i = 0; _i <= _r; ++_i)
		{
			PK[_k][_i] = _temp*(PK[_k-1][_i+1] - PK[_k-1][_i])
				/(m_Knots[r1+_i+m_Degree+1] - m_Knots[r1+_i+_k]);
		}
	}
}

//double srBSpline::CurveDerPoint( double u, int d )
//{
//	if (d > m_Degree)
//		return 0.0;
//
//	double* _CK = new double[d+1];
//
//	CurveDerivsAlg1(_CK, u, d);
//
//	double ret = _CK[d];
//
//	delete[] _CK;
//
//	return ret;
//}


bool srBSpline::getCurveDerPoint( double& ret, double u, int d )
{
	if (d > m_Degree)
		return 0.0;

	double* _CK = new double[d+1];

	if(CurveDerivsAlg1(_CK, u, d))
	{
		ret = _CK[d];
		delete[] _CK;
		return true;
	}
	
	else
	{
		delete[] _CK;
	}
		return false;
}



//void srBSpline::CurveDerivsAlg2( OUT double* CK /* [d+1][degree+1] */
//	, double u /* global param */
//	, int d )
//{
//	assert(d <= m_Degree);
//
//	int _k = 0;
//	int _j = 0;
//
//	double** _nders = new double*[d+1];
//	for (_k = 0; _k < d+1; ++_k)
//		_nders[_k] = new double[m_Degree+1];
//
//	int _span = FindSpan(u);
//
//	AllBasis
//
//	BasisFunsDers(_nders, u, d);
//
//	for (_k = 0; _k <= d; ++_k)
//	{
//		CK[_k] = 0.0;
//		for (_j = 0; _j <= m_Degree; ++_j)
//		{
//			CK[_k] += _nders[_k][_j]*m_ControlPoints[_span-m_Degree+_j];
//		}
//	}
//
//	for (_k = 0; _k < d+1; ++_k)
//		delete[] _nders[_k];
//	delete[] _nders;
//}

void srBSpline::_CalcukChordlength(double* t, double* Q, double* U, int NumData )
{
	int i;
	double distance =0.0;
	double step=0.0;

	U[0]=0.0;

	for(i=1; i<NumData; ++i)
	{
		distance += sqrt(t[i]-t[i-1])*(t[i]-t[i-1])+(Q[i]-Q[i-1])*(Q[i]-Q[i-1]);
	}

	for(i=1; i<NumData; ++i)
	{
		step =sqrt(t[i]-t[i-1])*(t[i]-t[i-1])+(Q[i]-Q[i-1])*(Q[i]-Q[i-1]); 
		U[i]=U[i-1]+step/distance;
	}

}

void srBSpline::_AveragingAlg( double* u, double* /*_U*/, int _p, int _m )
{
	int i,k;
	int j=0;
	int NummidKnots=0;
	double Sum =0.0;
	NummidKnots = _m-2*(_p+1);

	for(i=0 ; i< _p+1; ++i)
		m_Knots[j++]=0.0;

	for(i=0 ; i< NummidKnots; ++i)
	{	
		for(k=1; k<_p+j-1;++k)
			Sum += u[k] ;
		m_Knots[j++]=Sum/_p;
	}

	for(i=0 ; i< _p+1; ++i)
		m_Knots[j++]=0.0;
}

void srBSpline::SolveTridiagonal( int _n,double* _Q,double* _P )
{
	int i;
	double* abc	= new double [4];;
	double* R	= new double [_n+1];
	double* dd	= new double [_n+1];
	double den=0.0;

	for(i=3; i<_n; ++i)
		R[i] =_Q[i-1];

	BasisFuns(abc,4,m_Knots[4]);
	den = abc[1];
	_P[2]= (_Q[1]-abc[0]*_P[1])/den;

	for(i=3; i<_n; ++i)
	{
		dd[i]=abc[2]/den;
		BasisFuns(abc,i+2,m_Knots[i+2]);
		den = abc[1]-abc[0]*dd[i];
		_P[i]=(R[i]-abc[0]*_P[i-1])/den;
	}
	dd[_n]=abc[2]/den;
	
	BasisFuns(abc,_n+2,m_Knots[_n+2]);
	den =abc[1]-abc[0]*dd[_n];
	
	_P[_n]=(_Q[_n-1]-abc[2]*_P[_n+1]-abc[0]*_P[_n-1])/den;
	
	for(i=_n-1; i>=2; i--)
		_P[i]=_P[i]-dd[i+1]*_P[i+1];


	delete []abc;
	delete []R;
	delete []dd;
}

void srBSpline::CubicSplineInterpolation(double* t,double* Q, int NumData )
{
	/* CubicSplineInterpolation
	1. degree=3
	2. multiplicity (degree+1) knots at two endpoints
	3. first and last derivatives are given
	4. mid-knot vector is same as given x-coordinates of data points
	*/
	//# given points : n+1 
	
	m_Degree =3;
	m_NumKnots = NumData+2*m_Degree;			/* #knots= #Numdata-2+2*(m_Degree+1) */
	m_NumCPs   = m_NumKnots-m_Degree-1;
	m_ControlPoints = new double[m_NumCPs];
	m_Knots   = new double [m_NumKnots];
	double der_first =0.0;
	double der_end =0.0;
	
	der_first = (Q[1]+Q[0])/(t[1]-t[0]);
	der_end =(Q[NumData-1]+Q[NumData-2])/(t[NumData-1]-t[NumData-2]);

	_CalcKnot2(m_Knots,m_NumKnots,t,m_Degree+1,m_Degree+1);
	
	m_ControlPoints[0] = Q[0];
	m_ControlPoints[1] = Q[0]+(m_Knots[4]/3)*der_first;

	m_ControlPoints[m_NumCPs-1] = Q[NumData-1];
	m_ControlPoints[m_NumCPs-2]= Q[NumData-1]-((t[NumData-1]-t[0])-m_Knots[NumData+1])/3*der_end;


	printf("Ctrl Points\n");
	for(int k=0;k<m_NumCPs;k++)
		std::cout<<m_ControlPoints[k]<<std::endl;

	SolveTridiagonal(NumData-1,Q, m_ControlPoints); 
	
}

double srBSpline::GetGrevilleAbscissa( int controlPointIndex )
{
	double Result = 0.0;
	
	// ( U[i + 1] + U[i + 2] + ... + U[i + p] ) / p
	for (int i = 0; i < m_Degree; ++i)
		Result += m_Knots[controlPointIndex + i + 1];
	
	assert(m_Degree > 0);
	return Result/m_Degree;
}

void srBSpline::SetDifferentialBSpline( srBSpline& aBSplineSource )
{
	int nCtrlPt = aBSplineSource.GetNumControlPoint();
	int Degree = aBSplineSource.GetDegree();
	int nKnot = aBSplineSource.GetNumKnots();

	double* TempCtrlPT = new double[nCtrlPt - 1];

	for(int i = 0 ; i < nCtrlPt - 1 ; i++)
		TempCtrlPT[i] = Degree/(aBSplineSource.GetKnot(i + Degree) - aBSplineSource.GetKnot(i))*(aBSplineSource.GetControlPoint(i+1) - aBSplineSource.GetControlPoint(i));

	double* dblTempKnot = new double[nKnot - 2];

	for(int i = 0 ; i < nKnot-2 ; i++)
		dblTempKnot[i] = aBSplineSource.GetKnot(i + 1);

	Set(Degree - 1, nCtrlPt - 1, TempCtrlPT, nKnot - 2, dblTempKnot);

	delete [] TempCtrlPT;
	delete [] dblTempKnot;
}

bool srBSpline::SetCtrlPts( double aXi, double aXf, const double* pCtrlPts, int aRepeatedCtrlPts_Initial, int aRepeatedCtrlPts_Final, int NumCtrlPt )
{
	if (m_NumCPs != aRepeatedCtrlPts_Initial + aRepeatedCtrlPts_Final + NumCtrlPt)
		return false;

	int i;
	int j = 0;

	for(i = 0; i < aRepeatedCtrlPts_Initial; ++i)
	{
		this->SetControlPoint(j++, aXi);
	}

	for(i=0; i< NumCtrlPt; ++i)
	{
		this->SetControlPoint(j++, pCtrlPts[i]);
	}

	for(i = 0; i < aRepeatedCtrlPts_Final; ++i)
	{
		this->SetControlPoint(j++, aXf);
	}

	return true;
}

bool srBSpline::findSpan( int& ret, double u )
{
	if (u < m_Knots[0] || m_Knots[m_NumKnots-1] < u)
		return false;

	//This case we do not consider the repeated knots
	 // if there are repeated knots, we can reduce the possible range of searching span.

	// Special case
	// u가 knots의 끝 값이 경우, 특수한 경우로서,
	// 끝에서 두 번째 knot의 index가 span이 되게 된다.
	// 이렇게 하면, 안됨
	if (SR_ISEQUAL(u, m_Knots[m_NumKnots-1]))
	{
		for (int i = m_NumKnots - 2; i > -1; --i)
		{
			if (m_Knots[i] < u && u <= m_Knots[i+1])
			{
				ret = i;
				return true;
			}
		}
		return false;
	}

	// Binary search
	int _low = 0;
	int _high = m_NumKnots - 1;
	int _mid = (_low+_high) >> 1;

	while (u < m_Knots[_mid] || u >= m_Knots[_mid + 1])
	{
		if (u < m_Knots[_mid])
			_high = _mid;
		else
			_low = _mid;
		_mid = (_low + _high) >> 1;
	}

	ret = _mid;

	return true;
}

double srBSpline::getT0( void )
{
	 return m_Knots[0];
}

double srBSpline::getTf( void )
{
	return m_Knots[m_NumKnots-1];
}

void srBSpline::CurveKnotsIns(double u, int r )
{
	/* caution :  multiplicity < degree */

	int i,j,k;
	int L=0;
	
	int np	= m_NumCPs;
	int p	=m_Degree;
	
	double alpha;
	
	double* UQ	  = new double [m_NumKnots+r];		/* NewKnots   */
	double* newC  = new double [m_NumCPs+r];		/* NewCps     */
	double* Rw	  = new double [m_Degree+1];		/* Temp Variable*/

	/* Calculate multiplicity of Knot Span */
	int s = FindMultiplicity(u);					/*multiplicity*/
	findSpan(k,u);									/*FindSpan*/
	
	/*Load new Knot vector */
	for(i=0; i < k; ++i)
		UQ[i]=m_Knots[i];
	
	if(u==m_Knots[k])
		UQ[k]=u;
	else
		UQ[k]=m_Knots[k];
	
	for(i=1 ; i<=r; ++i)
		UQ[k+i]=u;
	
	for(i=k+1; i<=m_NumKnots; i++)
		UQ[i+r]= m_Knots[i];

	/*Save unaltered Control Points */
	for(i=0; i <= k-p;i++) 
		newC[i]=m_ControlPoints[i];
	for(i=k-s; i < m_NumCPs;i++) 
		newC[i+r]=m_ControlPoints[i];
	for(i=0; i <p ;i++) 
		Rw[i]= m_ControlPoints[k-p+i];
		
	
	/*Insert the knot r times */

	if(r<3)
	{	
		for(j=1; j<=r; j++)		
		{
			L= k-p+j;
			
			for(i=0 ; i<=p-j-s; i++)
			{
				alpha = (u-m_Knots[L+i])/(m_Knots[i+k+1]-m_Knots[L+i]);
				Rw[i]=alpha*Rw[i+1]+(1.0-alpha)*Rw[i];
			}
		
			newC[L]= Rw[0];
			newC[k+r-j-s]= Rw[p-j-s];

		}

	}
	else
	{
		for(j=1; j<r; j++)		
		{
			L= k-p+j;

			for(i=0 ; i<=p-j-s; i++)
			{
				alpha = (u-m_Knots[L+i])/(m_Knots[i+k+1]-m_Knots[L+i]);
				Rw[i]=alpha*Rw[i+1]+(1.0-alpha)*Rw[i];
			}

			newC[L]= Rw[0];
			newC[k+r-j-s]= Rw[p-j-s];

		}
	}
	/*Load remaining control points */
	for(i=L+1; i<k-s; i++)	
		newC[i] = Rw[i-L];

	/*Save to member variable */
	m_NumKnots = m_NumKnots+r;
	m_NumCPs   = m_NumCPs+r;

	m_ControlPoints  = new double [m_NumCPs];
	m_Knots			 = new double [m_NumKnots];
	
	for(i=0; i< m_NumCPs;++i)
		m_ControlPoints[i] = newC[i];
			
	for(i=0; i< m_NumKnots;++i)
		m_Knots[i] = UQ[i];
	
	/*delete temp variable*/
	delete []UQ;
	delete []newC;
	delete []Rw;

}

int srBSpline::FindMultiplicity(double _u)
{
	int ret;
	int count =1;							/*default */

	findSpan(ret, _u);
	
	for(int i=ret-m_Degree ; i< ret; ++i)
	{
		if(m_Knots[i]==_u)
			count++;
		else
			return count;
	}
	
	return count;

}



void srBSpline::SolveTridiagonal( int _n, std::vector<double>& _Q,double* _P )
{
	int i;
	double* abc	= new double [4];;
	double* R	= new double [_n+1];
	double* dd	= new double [_n+1];
	double den=0.0;

	for(i=3; i<_n; ++i)
		R[i] =_Q[i-1];

	BasisFuns(abc,4,m_Knots[4]);
	den = abc[1];
	_P[2]= (_Q[1]-abc[0]*_P[1])/den;

	for(i=3; i<_n; ++i)
	{
		dd[i]=abc[2]/den;
		BasisFuns(abc,i+2,m_Knots[i+2]);
		den = abc[1]-abc[0]*dd[i];
		_P[i]=(R[i]-abc[0]*_P[i-1])/den;
	}
	dd[_n]=abc[2]/den;

	BasisFuns(abc,_n+2,m_Knots[_n+2]);
	den =abc[1]-abc[0]*dd[_n];

	_P[_n]=(_Q[_n-1]-abc[2]*_P[_n+1]-abc[0]*_P[_n-1])/den;

	for(i=_n-1; i>=2; i--)
		_P[i]=_P[i]-dd[i+1]*_P[i+1];


	delete []abc;
	delete []R;
	delete []dd;

}


void srBSpline::CubicSplineInterpolation( std::vector<double>& t,std::vector<double>& Q, int NumData )
{
	/* CubicSplineInterpolation
	1. degree=3
	2. multiplicity (degree+1) knots at two endpoints
	3. first and last derivatives are given
	4. mid-knot vector is same as given x-coordinates of data points
	*/
	//# given points : n+1 
	
	m_Degree =3;
	m_NumKnots = NumData+2*m_Degree;			/* #knots= #Numdata-2+2*(m_Degree+1) */
	m_NumCPs   = m_NumKnots-m_Degree-1;
	m_ControlPoints = new double[m_NumCPs];
	m_Knots   = new double [m_NumKnots];
	double der_first =0.0;
	double der_end =0.0;
	
	der_first = (Q[1]-Q[0])/(t[1]-t[0]);

	//std::cout<<"der_first :"<<der_first<<std::endl;

	der_end =(Q[NumData-1]-Q[NumData-2])/(t[NumData-1]-t[NumData-2]);

	//std::cout<<"der_end :"<<der_end<<std::endl;

	_CalcKnot2(m_Knots,m_NumKnots,t,m_Degree+1,m_Degree+1);
	


	m_ControlPoints[0]=Q[0];
	m_ControlPoints[1]=Q[0]+(m_Knots[4]/3)*der_first;

	m_ControlPoints[m_NumCPs-1]= Q[NumData-1];

	m_ControlPoints[m_NumCPs-2]= Q[NumData-1]-((t[NumData-1]-t[0])-(m_Knots[NumData+1]-m_Knots[0]))/3*der_end;

	// 	printf("Ctrl Points\n");
	// for(int k=0;k<m_NumCPs;k++)
	// 	std::cout<<m_ControlPoints[k]<<std::endl;

	SolveTridiagonal(NumData-1,Q, m_ControlPoints); 
}


void srBSpline::CubicSplineInterpolationwithecontinuity(srBSpline* prev_spline,double* t,double* Q, int NumData )
{
	/* CubicSplineInterpolation
	1. degree=3
	2. multiplicity (degree+1) knots at two endpoints
	3. first and last derivatives are given
	4. mid-knot vector is same as given x-coordinates of data points
	*/
	//# given points : n+1 
	
	m_Degree =3;
	m_NumKnots = NumData+2*m_Degree;			/* #knots= #Numdata-2+2*(m_Degree+1) */
	m_NumCPs   = m_NumKnots-m_Degree-1;
	m_ControlPoints = new double[m_NumCPs];
	m_Knots   = new double [m_NumKnots];
	double der_first =0.0;
	double der_end =0.0;
	
	 der_first = prev_spline->getlastDerivate();
	
	//der_first = (Q[1]+Q[0])/(t[1]-t[0]);
	der_end =(Q[NumData-1]-Q[NumData-2])/(t[NumData-1]-t[NumData-2]);

	_CalcKnot2(m_Knots,m_NumKnots,t,m_Degree+1,m_Degree+1);
	
	m_ControlPoints[0] = Q[0];
	m_ControlPoints[1] = Q[0]+((m_Knots[4]-m_Knots[0])/3)*der_first;

	m_ControlPoints[m_NumCPs-1] = Q[NumData-1];
	m_ControlPoints[m_NumCPs-2]= Q[NumData-1]-((t[NumData-1]-t[0])-(m_Knots[NumData+1]-m_Knots[0]))/3*der_end;

	SolveTridiagonal(NumData-1,Q, m_ControlPoints); 
	
}

void srBSpline::CubicSplineInterpolationwithecontinuity( srBSpline* prev_spline,std::vector<double>& t,std::vector<double>& Q, int NumData )
{
	m_Degree =3;
	m_NumKnots = NumData+2*m_Degree;			/* #knots= #Numdata-2+2*(m_Degree+1) */
	m_NumCPs   = m_NumKnots-m_Degree-1;
	m_ControlPoints = new double[m_NumCPs];
	m_Knots   = new double [m_NumKnots];
	double der_first =0.0;
	double der_end =0.0;

	der_first = prev_spline->getlastDerivate();
	//der_first = (Q[1]+Q[0])/(t[1]-t[0]);
	der_end =(Q[NumData-1]-Q[NumData-2])/(t[NumData-1]-t[NumData-2]);

	_CalcKnot2(m_Knots,m_NumKnots,t,m_Degree+1,m_Degree+1);

	m_ControlPoints[0] = Q[0];
	m_ControlPoints[1] = Q[0]+((m_Knots[4]-m_Knots[0])/3)*der_first;

	m_ControlPoints[m_NumCPs-1] = Q[NumData-1];
	m_ControlPoints[m_NumCPs-2]= Q[NumData-1]-((t[NumData-1]-t[0])-(m_Knots[NumData+1]-m_Knots[0]))/3*der_end;

	SolveTridiagonal(NumData-1,Q, m_ControlPoints); 
}

void srBSpline::connectCubicSpline( srBSpline* prev_spline,srBSpline* post_spline )
{
	m_Degree =3;
	m_NumKnots = prev_spline->GetNumKnots()+post_spline->GetNumKnots()-(m_Degree+2);
	m_NumCPs   = prev_spline->GetNumControlPoint()+post_spline->GetNumControlPoint()-1;

	m_ControlPoints = new double[m_NumCPs];
	m_Knots   = new double [m_NumKnots];

	for(int i=0 ; i< prev_spline->GetNumKnots()-1; ++i)
		m_Knots[i]=prev_spline->GetKnot(i);
	for(int i=0 ; i< post_spline->GetNumKnots()-(m_Degree+1); ++i)
		m_Knots[i+prev_spline->GetNumKnots()-1]=post_spline->GetKnot(i+m_Degree+1);

	for(int i=0 ; i< prev_spline->GetNumControlPoint(); ++i)
		m_ControlPoints[i]=prev_spline->GetControlPoint(i);
	for(int i=0 ; i< post_spline->GetNumControlPoint()-1; ++i)
		m_ControlPoints[i+prev_spline->GetNumControlPoint()]=post_spline->GetControlPoint(i+1);

}



int srBSpline::getNumCPs()
{
	int NumCps = m_NumCPs;

	return NumCps;
}

double srBSpline::getCPs(int index)
{
	double Cp;

	Cp = m_ControlPoints[index];

	return Cp;
}

double srBSpline::getlastDerivate()
{

	double temp=0.0;
	getCurveDerPoint(temp,m_Knots[m_NumKnots-1],1);

	return temp;
}

bool srBSpline::getSurfacePoint( double& ret, double u, double v )
{
	int _span_u;
	int _span_v;

	if(u<m_Knots[0])
		u = m_Knots[0];
	else if ( u>m_Knots[m_NumKnots-1])
		u=m_Knots[m_NumKnots-1];

	if(v<m_KnotsV[0])
		v = m_KnotsV[0];
	else if ( v>m_KnotsV[m_NumKnots-1])
		v=m_KnotsV[m_NumKnots-1];
	
	if (!findSpan(_span_u, u))
		return false;
	if (!findSpan(_span_v, v))
		return false;

	double* _N = new double[m_Degree+1];
	double* _M = new double[m_Degree+1];

	BasisFuns(_N, _span_u, u);
	BasisFuns(_M, _span_v, v);

	double _S = 0.0;
	double temp_=0.0;
	int uind=_span_u-u;
	int vind=_span_v-v;

	for (int i = 0; i <= m_Degree; ++i)
	{
		temp_=0.0;
		vind = _span_v-m_Degree+i;
		
		//  Control points array should be initialized before this function uses.
		for (int k=0; k<= m_Degree; k++)
			temp_=temp_+_N[k]*m_ControlPointsarray[uind+k][vind];
	
		
		_S +=_S+ _M[i]*temp_;
		
	}
	delete[] _N;
	delete[] _M;

	ret = _S;
}

void srBSpline::SurfaceDerivsA1g1( OUT double** SKL, double u,double v, int d )
{
	//Assume that p = q = m_Degree

	assert(d <= m_Degree);
	int _k = 0;
	int _j = 0;

	double** Nu = new double*[d+1];
	for (_k = 0; _k < d+1; ++_k)
		Nu[_k] = new double[m_Degree+1];

	double** Nv = new double*[d+1];
	for (_k = 0; _k < d+1; ++_k)
		Nv[_k] = new double[m_Degree+1];

	int _span_u;
	int _span_v;

	if (!findSpan(_span_u, u))
		return ;

		BasisFunsDers(Nu, _span_u, u, d);

	if (!findSpan(_span_v, v))
		return ;

		BasisFunsDers(Nv, _span_v, v, d);

	double* temp= new double [m_Degree+1];

	for (_k = 0; _k <= d; ++_k)
	{
		for (_j = 0; _j <= m_Degree; ++_j)
		{
			temp[_j] = 0.0;
			for(int _r=0;_r<=m_Degree;_r++)
				temp[_j]+=Nu[_k][_j]*m_ControlPointsarray[_span_u-m_Degree+_r][_span_v-d+_j];
		}
	

		int dd=0;
		for(int l_=0;l_<dd;l_++)
		{
			SKL[_k][l_] = 0.0;
			for (int s=0; s<=m_Degree; s++)
				SKL[_k][l_] = SKL[_k][l_] + Nv[l_][s]*temp[s];
		}
	}

	for (_k = 0; _k < d+1; ++_k)
		delete[] Nu[_k];
	delete[] Nu;

	for (_k = 0; _k < d+1; ++_k)
		delete[] Nv[_k];
	delete[] Nv;

	delete[] temp;

	return;
}

void srBSpline::SurfMeshParams( OUT double* uk, double* vl, std::vector<std::vector<double>> Datapoints , int datasize_u, int datasize_v)
{
	int n = datasize_u;
	int m = datasize_v;
	int num = n+1;
	int num_v = m+1;

	double * Uk= new double  [n+1];
	double * Vl= new double  [m+1];
	double* cds = new double [n+1];
	double* cds_v= new double [m+1];
	double total=0;
	double d=0.0;

	Uk[0]=0.0;
	uk[num]=1.0;

	for(int k=0; k<n ;k++)
		Uk[k]=0.0;

	for( int l=0; l<=m ;l++)
	{
		total=0.0;
		for(int k_=1;k_<=n;k_++)
		{
			//cds[k_]=Distance3D(Datapoints[k_][l],Datapoints[k_-1][l]);
			total+=cds[k_];
		}

		if(total==0.0)
			num=num-1;
		else
		{
			d=0.0;
			for(int k=1;k<n ; k++)
			{
				d+=cds[k];
				Uk[k]+=d/total;
			}
		}
		if(num==0)
			return;
		for(int k=1;k<n;k++)
			Uk[k]=Uk[k]/num;
	}

	// For Vl
	for(int k=0; k<m ;k++)
		Vl[k]=0.0;

	for( int l=0; l<=n ;l++)
	{
		total=0.0;
		for(int k_=1;k_<=m;k_++)
		{
		//	cds[k_]=Distance3D(Datapoints[l][k_-1],Datapoints[l][k_]);
			total+=cds[k_];
		}

		if(total==0.0)
			num=num-1;
		else
		{
			d=0.0;
			for(int k=1;k<m ; k++)
			{
				d+=cds[k];
				Vl[k]+=d/total;
			}
		}
		if(num==0)
			return;
		for(int k=1;k<m;k++)
			Vl[k]=Vl[k]/num;
	}
}
 
void srBSpline::CubicSurfaceInterpolation( std::vector<double>& t,std::vector<std::vector<double>>& Q,std::vector<double>& t2,std::vector<double>& W, int NumData_Q , int NumData_W )
{
	m_Degree   = 3; 
	m_NumKnots = NumData_Q+2*m_Degree;			/* #knots= #Numdata-2+2*(m_Degree+1) */
	m_NumCPs   = m_NumKnots-m_Degree-1;

	m_NumKnotsV=NumData_W+2*m_Degree;
	m_NumCPsV=m_NumKnotsV-m_Degree-1;

	m_ControlPointsarray = new double*[NumData_Q];
	for(int i=0;i<NumData_Q;i++)
		m_ControlPointsarray[i]=new double [m_NumCPs];

	double** m_ControlPointsarrayT = new double*[m_NumCPs];

	for(int i=0;i<m_NumCPs;i++)
		m_ControlPointsarrayT[i]=new double [NumData_Q];

	m_Knots   = new double [m_NumKnots];
	m_KnotsV   = new double [m_NumKnotsV];

	_CalcKnot2(m_Knots,m_NumKnots,t,m_Degree+1,m_Degree+1);					//calculate U
	_CalcKnot2(m_KnotsV,m_NumKnotsV,t2,m_Degree+1,m_Degree+1);              //calculate V

	double der_first = 0.0;
	double der_end   = 0.0;

	for(int j=0;j<NumData_Q;j++)
	{
		der_first = (Q[j][1]-Q[j][0])/(t[1]-t[0]);
		der_end =(Q[j][NumData_Q-1]-Q[j][NumData_Q-2])/(t[NumData_Q-1]-t[NumData_Q-2]);

		m_ControlPointsarray[j][0]=Q[j][0];
		m_ControlPointsarray[j][1]=Q[j][0]+(m_Knots[4]/3)*der_first;

		m_ControlPointsarray[j][m_NumCPs-1]= Q[j][NumData_Q-1];
		m_ControlPointsarray[j][m_NumCPs-2]= Q[j][NumData_Q-1]-((t[NumData_Q-1]-t[0])-(m_Knots[NumData_Q+1]-m_Knots[0]))/3*der_end;

		SolveTridiagonal(NumData_Q-1,Q[j], m_ControlPointsarray[j]); 
	}

	for(int i=0;i<m_NumCPs;i++)
		for(int j=0; j<NumData_Q;j++)
			m_ControlPointsarrayT[i][j]=m_ControlPointsarray[j][i];

	double** tempCps = new double* [m_NumCPs];

	for(int i=0;i<m_NumCPs;i++)
		tempCps[i] = new double [m_NumCPsV];

	for(int k=0; k< m_NumCPs;k++)
	{
		der_first = (m_ControlPointsarrayT[k][1]-m_ControlPointsarrayT[k][0])/(t2[1]-t2[0]);
		der_end =(m_ControlPointsarrayT[k][NumData_W-1]-m_ControlPointsarrayT[k][NumData_W-2])/(t2[NumData_W-1]-t[NumData_W-2]);

		tempCps[k][0]=m_ControlPointsarrayT[k][0];
		tempCps[k][1]=m_ControlPointsarrayT[k][0]+(m_KnotsV[4]/3)*der_first;

		tempCps[k][m_NumCPsV-1]= m_ControlPointsarrayT[k][NumData_W-1];
		tempCps[k][m_NumCPsV-2]= m_ControlPointsarrayT[k][NumData_W-1]-((t2[NumData_W-1]-t2[0])-(m_KnotsV[NumData_W+1]-m_KnotsV[0]))/3*der_end;

		SolveTridiagonal(NumData_W-1,m_ControlPointsarrayT[k], tempCps[k]); 
	}


	for(int i=0;i<m_NumCPs;i++)
		for(int j=0;j<m_NumCPsV;j++)
			m_ControlPointsarray[i][j]=tempCps[i][j];



}


double srBSpline::Distance3D( double* Q1, double* Q2 )
{
	double norm=0.0;

	for(int i=0;i<3;i++)
		norm+=(Q1[i]-Q2[i])*(Q1[i]-Q2[i]);

	norm =sqrt(norm);
	
	return norm;

}


