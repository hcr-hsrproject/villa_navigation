/*-----------------------------------------------------------
srBspline algorithms version 1.0    -2016.10.21 MK
-------------------------------------------------------------*/
#ifndef	___SRLIB_BSPLINE___
#define ___SRLIB_BSPLINE___

#include <vector>
#include <iostream>

#define IN
#define OUT
class srBSpline
{
public:
	/*!  */
	srBSpline( void );

	/*!  */
	srBSpline( int Degree, int NumCPs, double* CPs, int NumKnots, double* Knots );
	
	/*!  */
	srBSpline( int Degree, int NumCPs, double* CPs, double Ti, double Tf,
		bool KnotRepeat = true );

	/*!  */
	srBSpline( int NumCPs, double Ti, double Tf, int Degree, bool KnotRepeat = true);
	
	/*!  */
	const srBSpline& operator=(const srBSpline& rhs);

	/*! Copy Constructor. */
	srBSpline(const srBSpline& T);
	
	/*! Casting operator. */
	srBSpline* operator&() { return this; }

	/*! Const Casting operator. */
	const srBSpline* operator&() const { return this; }


	/*!  */
	~srBSpline();

public: // Familiar notations with the NURBS Book

	/*! Point on Curve */
	inline double C( double u ) { return CurvePoint( u ); }

	/*! Control point */
	inline double P( int IndexCP ) { return GetControlPoint( IndexCP ); }

	/*! Knots */
	inline double U( int IndexKnot ) { return GetKnot( IndexKnot ); }

	/*! Compute basis function */
	inline double N( int IndexCP, double u ) { return OneBasisFun( IndexCP, u ); }

	/*! Compute first derivative of basis function */
	inline double dN( int IndexCP, double u ) { return OneBasisFunDers( IndexCP, u, 1 ); }

	/*! Compute second derivative of basis function */
	inline double ddN( int IndexCP, double u ) { return OneBasisFunDers( IndexCP, u, 2 ); }

	/*! Degree */
	inline int p( void ) { return m_Degree; }

public:
	/*! degree, control points, knots */
	void Set(int Degree, int NumCPs, double* CPs, int NumKnots, double* Knots);

	/*! Degree, Cps, the initial and final time */
	void Set(int Degree, int NumCPs, double* CPs, double Ti, double Tf,
		bool KnotRepeat = true);

	/*! Degree, Cps, the initial and final time */
	void Set( int Degree, int NumCPs, double Ti, double Tf , bool KnotRepeat /*= true*/ );
	
	/*!  */
	int GetDegree( void ) { return m_Degree; }

	/*!   */
	int GetNumControlPoint( void ) { return m_NumCPs; }

	/*!  */
	double GetControlPoint( int Index ) { return m_ControlPoints[Index]; }

		/*! TODO: code verification  */
	double GetGrevilleAbscissa( int controlPointIndex );

	/*! TODO:  */
	int GetNumKnots( void ) { return m_NumKnots; }

	/*!  */
	double GetKnot( int Index ) { return m_Knots[Index]; }

	/*!  */
	void SetControlPoint( int Index, double NewValue ) { m_ControlPoints[Index] = NewValue; }

	/*!  */
	void SetKnot( int Index, double NewValue ) { m_Knots[Index] = NewValue; }

	/*! TODO:  */
	bool SetCtrlPts( double aXi, double aXf,
		const double* pCtrlPts,
		int aRepeatedCtrlPts_Initial,
		int aRepeatedCtrlPts_Final, 
		int NumCtrlPt );

	/*!  */
	double getT0 (void);

	/*!  */
	double getTf (void);
	
	/*! TODO:  */
	void SetDifferentialBSpline( srBSpline& aBSplineSource );

	/*! Compute Distance between two points
	*/
	double Distance3D(double* Q1, double* Q2);

public:	// The NURBS Book Algorithms

	/*! 
		Notations
		i		: index of span
		u		: global parameter
		U		: knots
		p		: degree
		m		: number of knots - 1
		n		: The maximum differential order 
		ders	: derivatives of basis functions
		Nip		: basis function(index: i, degree: p)
	*/
	
	/*!
		A2.1 Determine the knot span index
		Input: u(global param)
		Return:	the knot span index
	*/
	//int FindSpan( double u );

	bool findSpan(int& ret, double u);

	/*! A2.2 Compute the non-vanishing basis functions */
	void BasisFuns( OUT double* N, int span, double u );

	/*! A2.2 Compute the non-vanishing basis functions */

	void BasisFuns( OUT double* N, double u );

	/*!
		A2.3 Compute nonzero basis functions and their derivatives.
		First section is A2.2 modified to store functions and
		knot differences.
		OUT)
		ders: [n+1][p+1]
		IN)
		span: 
		u:
		n: highest derivative degree
	*/
	bool BasisFunsDers( OUT double** ders, int span, double u, int n );

	/*!
	    A2.3 Compute nonzero basis functions and their derivatives.
		First section is A2.2 modified to store functions and
		knot differences.
		OUT)
		ders: [n+1][p+1]
		IN)
		span: 
		u:
		n: highest derivative degree 
	*/
	bool BasisFunsDers( OUT double** ders, double u, int n );

	/*!
		A2.4 Compute the basis function Nip
		i: index of basis function
		u: global param
	*/
	double OneBasisFun( int i, double u );

	/*!
		A2.5 Compute derivatives of basis function N_{i,p}
		OUT ders: derivatives of basis function, [n+1]
		i: index of basis function
		u: global param
		n: highest derivative degree
	*/
	void OneBasisFunDers( OUT double* ders, int i, double u, int n );

	/*!
		Changed version of A2.5 - return only one differential basis function
		i: index of basis function
		u: global param
		n: derivative degree
	*/
	double OneBasisFunDers( int i, double u, int n );

	/*! A3.1 Compute curve */
	double CurvePoint( double u );

	bool getCurvePoint(double& ret, double u);

	
	/*!
		A3.2 Compute curve derivatives
		OUT CK: [d+1]
		u: global param
		d: highest derivative degree
	*/
	/*! bool function */
	bool CurveDerivsAlg1( OUT	double* CK, double u, int d );

	/*!
		Compute derivative of degree d at u
		u: 
		d: 
	*/
	bool getCurveDerPoint(double& ret, double u, int d);

	/*!
		A3.3 Compute control points of curve derivatives 
		A3.3 is a non-recursive implementation of Equation (3.8).
		It computes the control points of all derivative curves up to
		and including the d-th derivative (d <= p).
		On output, PK[k][i] is the i-th control point of the k-th derivative curve,
		where 0 <= k <= d and r1 <= i < r2 - k.
		If r1 = 0 and r2 = n, all control points are computed
		OUT PK: [d+1][r2-r1+1]
		d: highest derivative degree
		r1: 
		r2: 
	*/
	void CurveDerivCpts( OUT double** PK, int d, int r1, int r2 );

	///*! A3.4 Compute curve derivatives */
	//void CurveDerivsAlg2(OUT double* CK	// [d+1][degree+1]
	//	, double u						// global param
	//	, int d							
	//	);
	bool getSurfacePoint(double& ret, double u,double v);
	/* Compute surface point *1
	 1* Input: n,p,U,m,q,V,P,u,v *1
	 1* Output: S */
	void SurfaceDerivsA1g1(OUT	double** CK, double u,double v, int d);

	//void SurfMeshParams(OUT double* uk, double* vl, std::vector<std::vector<double>> Datapoints,int datasize);
	void SurfMeshParams( OUT double* uk, double* vl, std::vector<std::vector<double>> Datapoints , int datasize_u, int datasize_v);
	/*
		Cubic Interpolation
		Input: Data point(t:x-coord, Q: y-coord),#of Data points
	*/
	void CubicSplineInterpolation( double* t, double* Q, int NumData );
	/*
		Cubic Interpolation
		Equally knots spaced
		Input: Data point(t:x-coord, Q: y-coord),#of Data points
	*/
	void CubicSplineInterpolation(std::vector<double>& t,std::vector<double>& Q, int NumData);
	
	void CubicSurfaceInterpolation( double* t, double* Q,double* W, int NumData );
	void CubicSurfaceInterpolation(std::vector<double>& t,std::vector<std::vector<double>>& Q,std::vector<double>& t2,std::vector<double>& W, int NumData_Q , int NumData_W);

	void CubicSplineInterpolationwithecontinuity(srBSpline* prev_spline,double* t,double* Q, int NumData );
	void CubicSplineInterpolationwithecontinuity(srBSpline* prev_spline,std::vector<double>& t,std::vector<double>& Q, int NumData);
	void connectCubicSpline(srBSpline* prev_spline,srBSpline* post_spline);

	/*! TODO: ING */
	//void GlobalCurveInterp( int _n, double* t,double* Q, int _r, int _p, int _m,
	//	double* _U, double* _P, int NumData );

	/*! A5.1 CurveKnotIns 
		Input: u(knots position) , r(insert times)
	    caution : 1)  multiplicity < degree 
				  2)  cannot insert start or end points. 	*/
	void CurveKnotsIns(double _u, int r=1);
	int getNumCPs();

	double getCPs(int index);
	double getlastDerivate();
	void _Clear( void );
	void setKnotVector(double* _inputU);
	void setKnotVector2(double* _inputU);
	
protected:	// internal functions./
	/*!  */
	

	/*!  */
	double _Left( int i, int j, double u );

	/*!  */
	double _Right( int i, int j, double u );

	/*!  */
	void _CalcKnot( OUT double* Knot, int NumKnots, double Ti, double Tf, 
		int NumRepeat1 = 1, int NumRepeat2 = 1 );

	/*! Cubic interpolation CalcKnot */
	void _CalcKnot2( OUT double* Knot, int NumKnots, double* t,
					int NumRepeat1 = 4, int NumRepeat2 = 4 );

	void _CalcKnot2(OUT double* Knot, int NumKnots, std::vector<double>& t,
		int NumRepeat1 = 4, int NumRepeat2 = 4);
	void _CalcKnot3(OUT double* Knot, int NumKnots, std::vector<double>& t,
		int NumRepeat1 = 4, int NumRepeat2 = 4);

	/*!  */
	void _Linspace( OUT double* Array, double Ti, double Tf, int NumData );
	
	/*! equation (9.5) */
	void _CalcukChordlength( double* t,double* Q, double* U, int NumData );
	
	/*! equation (9.8) */
	void _AveragingAlg( double* u, double* _U, int _p, int _m );
	/*! 
		A9.2 SolveTridiagonal(n,Q,U,P)
		Input: n, Q, U, P[0],P[1],P[n+1],P[n+2]
		Output:	P
	*/
	void SolveTridiagonal( int _n, double* _Q,double* _P );
	void SolveTridiagonal(int _n, std::vector<double>& _Q,double* _P);

	/* Calculate multiplicity of Knot Span */
	/* for Knots insertion				   */
	int FindMultiplicity(double _u);

protected:
	int m_Degree;				// degree of the curve
	int m_NumKnots;				// number of knots
	int m_NumCPs;				// number of control points

	int m_NumKnotsV;			// number of knots for V 
	int m_NumCPsV;				// number of control points for V



	double* m_Knots;			// knots
	double* m_KnotsV;			// knots for V 
	double* m_ControlPoints;	// control points
	double** m_ControlPointsarray;	// control points for V 

private:


};

#endif // ___SRLIB_BSPLINE___

