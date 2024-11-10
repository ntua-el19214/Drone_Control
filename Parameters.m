m=0.8;
g=9.81;
l=0.3;
Ixx=15.67e-3;
Iyy=15.67e-3;
Izz=28.34e-3;
b=192.32e-7;
d=4e-7;
Jr=0;%6e-5;


p1=(Iyy-Izz)/Ixx;
p2=Jr/Ixx;
p3=l/Ixx;
p4=(Izz-Ixx)/Iyy;
p5=Jr/Iyy;
p6=l/Iyy;
p7=(Ixx-Iyy)/Izz;
p8=1/Izz;


Omega_sq_2_U = [b  b  b  b;
                0 -b  0  b;
               -b  0  b  0;
                d -d  d -d];

U_to_Omega_sq = inv(Omega_sq_2_U);


phi_0=0;
phi_dot_0=0;
th_0=0;
th_dot_0=0;
psi_0=0.;
psi_dot_0=0;

x_0=0;
x_dot_0=0;
y_0=0;
y_dot_0=0;
z_0=1;
z_dot_0=0;



U_max=15000/60*2*pi;
U_Sq_max=U_max^2;
U1_steady_State=g*m;

theta_limit = deg2rad(15);
phi_limit = deg2rad(15);

