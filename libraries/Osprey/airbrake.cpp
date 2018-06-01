#include <math.h>

struct Vector3D
{
  Vector3D(float a, float b, float c) : x(a), y(b), z(c) {}
  Vector3D() {}
  float x;
  float y;
  float z;
  float norm()
  { 
    return sqrt((x*x)+(y*y)+(z*z)); 
  }
};

struct StateVector {
  StateVector(float a, Vector3D s, float d) : height(a), vec(s), dragCoeff(d) {}
  StateVector() {}
  float height;
  Vector3D vec;
  float dragCoeff;
};

float const GRAV   = 9.81;
Vector3D const I_Z = Vector3D(0,0,1);

/* Calculates whether or not the satellite is in eclipse.
* input: h - height above the ellipsoid 
* output: rho - atmospheric density, in kg/m^3
*  Ref: Fundamentals of Spacecraft Attitude Determinal and Control
*       Crassidis, pg 406 & 407 (Eq. 11.10 and Table 11.1) */
float Exponentially_Decaying_Density_Model(float h)
{
   float parameters[36][5] =
   {{  0 ,  25 ,   0 , 1.225     , 8.44},
    { 25 ,  30 ,  25 , 3.899e-2  , 6.49},
    { 30 ,  35 ,  30 , 1.774e-2  , 6.75},
    { 35 ,  40 ,  35 , 8.279e-3  , 7.07},
    { 40 ,  45 ,  40 , 3.972e-3  , 7.47},
    { 45 ,  50 ,  45 , 1.995e-3  , 7.83},
    { 50 ,  55 ,  50 , 1.057e-3  , 7.95},
    { 55 ,  60 ,  55 , 5.821e-4  , 7.73},
    { 60 ,  65 ,  60 , 3.206e-4  , 7.29},
    { 65 ,  70 ,  65 , 1.718e-4  , 6.81},
    { 70 ,  75 ,  70 , 8.770e-5  , 6.33},
    { 75 ,  80 ,  75 , 4.178e-5  , 6.00},
    { 80 ,  85 ,  80 , 1.905e-5  , 5.70},
    { 85 ,  90 ,  85 , 8.337e-6  , 5.41},
    { 90 ,  95 ,  90 , 3.396e-6  , 5.38},
    { 95 , 100 ,  95 , 1.343e-6  , 5.74},
    { 100,  110,  100, 5.297e-7 ,  6.15},
    { 110,  120,  110, 9.661e-8 ,  8.06},
    { 120,  130,  120, 2.438e-8 ,  11.6},
    { 130,  140,  130, 8.484e-9 ,  16.1},
    { 140,  150,  140, 3.845e-9 ,  20.6},
    { 150,  160,  150, 2.070e-9 ,  24.6},
    { 160,  180,  160, 1.224e-9 ,  26.3},
    { 180,  200,  180, 5.464e-10,  33.2},
    { 200,  250,  200, 2.789e-10,  38.5},
    { 250,  300,  250, 7.248e-11,  46.9},
    { 300,  350,  300, 2.418e-11,  52.5},
    { 350,  400,  350, 9.158e-12,  56.4},
    { 400,  450,  400, 3.725e-12,  59.4},
    { 450,  500,  450, 1.585e-12,  62.2},
    { 500,  600,  500, 6.967e-13,  65.8},
    { 600,  700,  600, 1.454e-13,  79.0},
    { 700,  800,  700, 3.614e-14, 109.0},
    { 800,  900,  800, 1.170e-14, 164.0},
    { 900, 1000,  900, 5.245e-15, 225.0},
    {1000 , 1000, 1000, 3.019e-15, 268.0}};

  /* Initialize the atmosphere density model parameters */
  float h_0   = 0.0; // reference height, in km
  float rho_0 = 0.0; // reference density, in kg/m^3
  float H     = 0.0; // scale height, in km

  /* Compute the atmosphere density */
  for (auto counter = 0u; counter < 36; ++counter)
  { 
      if( h >= parameters[counter][0] && h <= parameters[counter][1])
      {
          h_0 = parameters[counter][2]; // reference height, in km
          rho_0 = parameters[counter][3]; //reference density, in kg/m^3
          H = parameters[counter][4]; // scale height, in km
      }
  }

  return rho_0*exp(-(h-h_0)/H); // atmospheric density, in kg/m^3
}


StateVector Truth_gravdiffeq_air_brake(StateVector x, float t)
{
  float mag_v = x.vec.norm();

  StateVector xdot;

  xdot.height = x.vec.z;

  float rho;
  
  if (x.height < 0)
  {
    rho = Exponentially_Decaying_Density_Model(0);
  }
  else
  {
    rho = Exponentially_Decaying_Density_Model(x.height/1000.0);
  }

  float newX = -(rho * x.vec.x * mag_v * x.dragCoeff);
  float newY = -(rho * x.vec.y * mag_v * x.dragCoeff);
  float newZ = -(rho * x.vec.z * mag_v * x.dragCoeff) - GRAV; 
  xdot.vec = Vector3D(newX, newY, newZ);
  xdot.dragCoeff = 0.0; 
  return xdot; 
}

StateVector mult(float c, StateVector v)
{
  return StateVector(c*v.height, Vector3D(c*v.vec.x, c*v.vec.y, c*v.vec.z), c*v.dragCoeff);
}

StateVector add(StateVector a, StateVector b)
{
  return StateVector(
      a.height + b.height,
      Vector3D(
        a.vec.x + b.vec.x,
        a.vec.y + b.vec.y,
        a.vec.z + b.vec.z
      ),
      a.dragCoeff + b.dragCoeff
  );
}

StateVector Truth_prop_state_rk45(StateVector xold, float t, float dt)
{
  float h = dt;
  float hh = h/2.0;
  float h6 = h/6.0;

  StateVector y    = xold;
  StateVector dydx = Truth_gravdiffeq_air_brake(y, t);
  StateVector yt   = add(y, mult(hh,dydx));
  StateVector dyt  = Truth_gravdiffeq_air_brake(yt, t);
  yt = add(y, mult(hh, dyt));
  StateVector dym = Truth_gravdiffeq_air_brake(yt, t);
  yt  = add(y, mult(h, dym));
  dym = add(dyt, dym);
  
  dyt = Truth_gravdiffeq_air_brake(yt, t);
  return add(y,mult(h6,(add(dydx,add(dyt,mult(2,dym))))));
}
