clear all
% puntos de baliza - ground truth (pos. conocida, y de referencia)
% trayectoria seguida - ground truth
% velocidades en el transcurso - ground truth
% un periodo de ejeción del algoritmo de T = 1 seg

%Balizas
Xbal=[0.2,0,1.5,1]';
Ybal=[1,0.5,0.2,0]';
Zbal=[0.5,0.8,1,1.8]'; 

%Recorrido ida
Xgt=[1:0.1:21]';
Ygt=[1:0.004:1.8]';
Zgt=[1:0.03:3,3.015:0.015:5.015]';
%Recorrido vuelta
Xgt2=[21.01:-0.01:19.5,19.4:-0.1:1]';
Ygt2=[1.84:-0.0025:1]';
Zgt2=[5.01:-0.01:2,2.03:-0.03:1]';

Xgt = [Xgt;Xgt2];
Ygt = [Ygt;Ygt2];
Zgt = [Zgt;Zgt2];

T=1;

Vxgt=zeros(length(Xgt),1);
Vygt=zeros(length(Xgt),1);
Vzgt=zeros(length(Xgt),1);

cont=3; % iteración donde comienza en algoritmo

for k = 2:length(Xgt)
    Vxgt(k) = (Xgt(k)- Xgt(k-1))/T;
    Vygt(k) = (Ygt(k)- Ygt(k-1))/T;
    Vzgt(k) = (Zgt(k)- Zgt(k-1))/T;
end


% desviacion tipica de medida (q), balizas radio, mas añadir ruido blanco proporcional
% se añade tambien la velocidad a los estados de estimacion
% se obtienen las velocidades a traves de una IMU
% desviacion tipica de IMU (q2)
% desviacion tipica de camara (q3)
q = 0.02;                                                  %%%%%%%%%%%%%%%%%   q
q2 = 0.03;                                                  %%%%%%%%%%%%%%%%%   q2
q3 = 2;                                                  %%%%%%%%%%%%%%%%%   q3

dmax = 15;

Pbal1 = zeros(length(Xgt),1);   %vector aleatorio
Pbal2 = zeros(length(Xgt),1);   %vector aleatorio
Pbal3 = zeros(length(Xgt),1);   %vector aleatorio
Pbal4 = zeros(length(Xgt),1);   %vector aleatorio

t = 1:length(Xgt);

d1 = zeros(length(Xgt),1);
d2 = zeros(length(Xgt),1);
d3 = zeros(length(Xgt),1);
d4 = zeros(length(Xgt),1);

for k = 1:length(Xgt)
Pbal1(k) = rand;
Pbal2(k) = rand;
Pbal3(k) = rand;
Pbal4(k) = rand;
end

%medida de distancia
for k = 1:length(Xgt)
    if(Pbal1(k)<=0.75)
        d1(k) = sqrt((Xgt(k)-Xbal(1))^2+(Ygt(k)-Ybal(1))^2+(Zgt(k)-Zbal(1))^2) + q*randn;
        if d1(k)>dmax
            d1(k)=-1;
        end
    else
        d1(k)=-1;
    end
    
    if(Pbal2(k)<=0.75)
        d2(k) = sqrt((Xgt(k)-Xbal(2))^2+(Ygt(k)-Ybal(2))^2+(Zgt(k)-Zbal(2))^2) + q*randn;
        if d2(k)>dmax
            d2(k)=-1;
        end
    else
        d2(k)=-1;
    end
    if(Pbal3(k)<=0.75)
        d3(k) = sqrt((Xgt(k)-Xbal(3))^2+(Ygt(k)-Ybal(3))^2+(Zgt(k)-Zbal(3))^2) + q*randn;
        if d3(k)>dmax
            d3(k)=-1;
        end
    else
        d3(k)=-1;
    end
    if(Pbal4(k)<=0.75)
        d4(k) = sqrt((Xgt(k)-Xbal(4))^2+(Ygt(k)-Ybal(4))^2+(Zgt(k)-Zbal(4))^2) + q*randn;
        if d4(k)>dmax
            d4(k)=-1;
        end
    else
        d4(k)=-1;
    end
end


vxr=zeros(length(Xgt),1); %velocidades con ruido de la IMU
vyr=zeros(length(Xgt),1);
vzr=zeros(length(Xgt),1);

for k = cont:length(Xgt)
    vxr(k) = Vxgt(k) + q2*randn;
    vyr(k) = Vygt(k) + q2*randn;
    vzr(k) = Vzgt(k) + q2*randn;
end

%Camara    ----------------------

%Sistema camara
xW=[1,0,0];
yW=[0,1,0];
zW=[0,0,1];

psi=0.90;%pi/4;

wTc =[ cos(psi+pi/4)  0  -sin(psi+pi/4)  Xgt(1);
       sin(psi+pi/4)  0  cos(psi+pi/4)  Ygt(1);
              0      -1              0  Zgt(1);
              0       0              0       1];
  
 Rx =[ 1          0          0    0;
       0  cos(-pi/10) -sin(-pi/10)    0;
       0  sin(-pi/10)  cos(-pi/10)    0;
       0          0          0    1]; 
           
 wTc = wTc*Rx;
             
centroC=[Xgt(1),Ygt(1),Zgt(1)];
xC=wTc*[xW,1]';
yC=wTc*[yW,1]';
zC=wTc*[zW,1]';

%Sistema balizas

centro1=[Xbal(1);Ybal(1);Zbal(1)];
xB1=[1,0,0];
yB1=[0,1,0];
zB1=[0,0,1];
centro2=[Xbal(2);Ybal(2);Zbal(2)];
xB2=xB1; yB2=yB1; zB2=zB1;
centro3=[Xbal(3);Ybal(3);Zbal(3)];
xB3=xB1; yB3=yB1; zB3=zB1;
centro4=[Xbal(4);Ybal(4);Zbal(4)];
xB4=xB1; yB4=yB1; zB4=zB1;


%Parametros intrinsecos

%distancia focal
f=0.0042; %m
%res imagen
N=1500; %ancho pix
M=1000; %alto pix
%tam sensor
w=0.00496; %ancho m
h=0.00352; %alto m
%P. princial img pix
u0=round(N/2); 
v0=round(M/2);
%dimension efectiva m/pix
rho_x = w/N;
rho_y = h/M;
%distancia focal efectva pix
fx = f/rho_x;
fy = f/rho_y;
%skew
s=0;
% Matriz de parámetros intrísecos:
Kp = [ fx  s*fx   u0
       0    fy   v0
       0     0    1 ];

% Posición del origen de {C} respecto al sistema {W}
cTw = inv(wTc);
cRw = cTw(1:3,1:3);
ctw = cTw(1:3,4);  

% Punto a proyectar %%
p1=[Xbal(1);Ybal(1);Zbal(1)];
p2=[Xbal(2);Ybal(2);Zbal(2)];
p3=[Xbal(3);Ybal(3);Zbal(3)];
p4=[Xbal(4);Ybal(4);Zbal(4)];
%balizas sistema de referencia W
wp1_ = [p1;1]; wp2_ = [p2;1]; wp3_ = [p3;1]; wp4_ = [p4;1];
%balizas sistema de referencia camara
p1_ = zeros(length(Xgt),3);
p2_ = zeros(length(Xgt),3);
p3_ = zeros(length(Xgt),3);
p4_ = zeros(length(Xgt),3);

p1_(1,1) = fx*((wp1_(1)-Xgt(1))*cRw(1,1)+(wp1_(2)-Ygt(1))*cRw(1,2)+(wp1_(3)-Zgt(1))*cRw(1,3))/((wp1_(1)-Xgt(1))*cRw(3,1)+(wp1_(2)-Ygt(1))*cRw(3,2)+(wp1_(3)-Zgt(1))*cRw(3,3))+u0;
p1_(1,2) = fy*((wp1_(1)-Xgt(1))*cRw(2,1)+(wp1_(2)-Ygt(1))*cRw(2,2)+(wp1_(3)-Zgt(1))*cRw(2,3))/((wp1_(1)-Xgt(1))*cRw(3,1)+(wp1_(2)-Ygt(1))*cRw(3,2)+(wp1_(3)-Zgt(1))*cRw(3,3))+v0;
p2_(1,1) = fx*((wp2_(1)-Xgt(1))*cRw(1,1)+(wp2_(2)-Ygt(1))*cRw(1,2)+(wp2_(3)-Zgt(1))*cRw(1,3))/((wp2_(1)-Xgt(1))*cRw(3,1)+(wp2_(2)-Ygt(1))*cRw(3,2)+(wp2_(3)-Zgt(1))*cRw(3,3))+u0;
p2_(1,2) = fy*((wp2_(1)-Xgt(1))*cRw(2,1)+(wp2_(2)-Ygt(1))*cRw(2,2)+(wp2_(3)-Zgt(1))*cRw(2,3))/((wp2_(1)-Xgt(1))*cRw(3,1)+(wp2_(2)-Ygt(1))*cRw(3,2)+(wp2_(3)-Zgt(1))*cRw(3,3))+v0;
p3_(1,1) = fx*((wp3_(1)-Xgt(1))*cRw(1,1)+(wp3_(2)-Ygt(1))*cRw(1,2)+(wp3_(3)-Zgt(1))*cRw(1,3))/((wp3_(1)-Xgt(1))*cRw(3,1)+(wp3_(2)-Ygt(1))*cRw(3,2)+(wp3_(3)-Zgt(1))*cRw(3,3))+u0;
p3_(1,2) = fy*((wp3_(1)-Xgt(1))*cRw(2,1)+(wp3_(2)-Ygt(1))*cRw(2,2)+(wp3_(3)-Zgt(1))*cRw(2,3))/((wp3_(1)-Xgt(1))*cRw(3,1)+(wp3_(2)-Ygt(1))*cRw(3,2)+(wp3_(3)-Zgt(1))*cRw(3,3))+v0;
p4_(1,1) = fx*((wp4_(1)-Xgt(1))*cRw(1,1)+(wp4_(2)-Ygt(1))*cRw(1,2)+(wp4_(3)-Zgt(1))*cRw(1,3))/((wp4_(1)-Xgt(1))*cRw(3,1)+(wp4_(2)-Ygt(1))*cRw(3,2)+(wp4_(3)-Zgt(1))*cRw(3,3))+u0;
p4_(1,2) = fy*((wp4_(1)-Xgt(1))*cRw(2,1)+(wp4_(2)-Ygt(1))*cRw(2,2)+(wp4_(3)-Zgt(1))*cRw(2,3))/((wp4_(1)-Xgt(1))*cRw(3,1)+(wp4_(2)-Ygt(1))*cRw(3,2)+(wp4_(3)-Zgt(1))*cRw(3,3))+v0;

%normalizamos
P1 = zeros(length(Xgt),2);
P2 = zeros(length(Xgt),2);
P3 = zeros(length(Xgt),2);
P4 = zeros(length(Xgt),2);
P1(1,:) = p1_(1,1:2) + q3*randn;
P2(1,:) = p2_(1,1:2) + q3*randn;
P3(1,:) = p3_(1,1:2) + q3*randn;
P4(1,:) = p4_(1,1:2) + q3*randn;
%balizas en camara
P1(1,:) = round (P1(1,:)); P2(1,:) = round (P2(1,:)); P3(1,:) = round (P3(1,:)); P4(1,:) = round (P4(1,:));

%angulo camara-balizas
%mediante producto escalar, elimina vision negativa
esc1= dot(zC(1:3)-centroC',zB1(1:3));
esc2= dot(zC(1:3)-centroC',zB2(1:3));
esc3= dot(zC(1:3)-centroC',zB3(1:3));
esc4= dot(zC(1:3)-centroC',zB4(1:3));
nC= norm(zC(1:3)-centroC');
n1= norm(zB1(1:3));
n2= norm(zB2(1:3));
n3= norm(zB3(1:3));
n4= norm(zB4(1:3));
c1=esc1/(nC*n1);
c2=esc2/(nC*n2);
c3=esc3/(nC*n3);
c4=esc4/(nC*n4);
a1=acos(c1);
a2=acos(c2);
a3=acos(c3);
a4=acos(c4);

%desviacion tipica del modelo(r) y matrices de incertidumbre de los modelos
r=0.03;                                                    %%%%%%%%%%%%%%%%%   r

Q= [q^2 0 0 0 0 0 0 0 0 0 0 0; 
    0 q^2 0 0 0 0 0 0 0 0 0 0;
    0 0 q^2 0 0 0 0 0 0 0 0 0;
    0 0 0 q^2 0 0 0 0 0 0 0 0; 
    0 0 0 0 q3^2 0 0 0 0 0 0 0;
    0 0 0 0 0 q3^2 0 0 0 0 0 0;
    0 0 0 0 0 0 q3^2 0 0 0 0 0;
    0 0 0 0 0 0 0 q3^2 0 0 0 0; 
    0 0 0 0 0 0 0 0 q3^2 0 0 0;
    0 0 0 0 0 0 0 0 0 q3^2 0 0;
    0 0 0 0 0 0 0 0 0 0 q3^2 0;
    0 0 0 0 0 0 0 0 0 0 0 q3^2];

%Q=Q(1:4,1:4);  %sin camara

R= [r^2 0 0 0 0 0; 
    0 r^2 0 0 0 0;
    0 0 r^2 0 0 0;
    0 0 0 r^2 0 0;
    0 0 0 0 r^2 0;
    0 0 0 0 0 r^2];

Xest = zeros(length(Xgt),6);


Xest(1,:) = [NaN NaN NaN NaN NaN NaN]; %si no llega la primera medida, se inicializa en instante 2
Xest(2,:) = [Xgt(2) Ygt(2) Zgt(2) vxr(2) vyr(2) vzr(2)];

Sigma_est = [Q(1:4,1:4) zeros(4,2); zeros(2,4) Q(1:2,1:2)];

Xpred=Xest;
Sigma_pred=Sigma_est;

detSigma = zeros(length(Xgt),1); % mide la incertidumbre de estimación
detSigma(1) = det(Sigma_est);

%diseño de modelos

A = [1 0 0 T 0 0;
     0 1 0 0 T 0;
     0 0 1 0 0 T;
     0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1];
 
%Inicializar H y Z

Z = zeros(length(Xgt),12);

Z(1,:) = [d1(1);d2(1);d3(1);d4(1);P1(1,1);P1(1,2);P2(1,1);P2(1,2);P3(1,1);P3(1,2);P4(1,1);P4(1,2)]; 
Z(2,:) = [d1(2);d2(2);d3(2);d4(2);P1(1,1);P1(1,2);P2(1,1);P2(1,2);P3(1,1);P3(1,2);P4(1,1);P4(1,2)];
P1(2,1)=P1(1,1);P1(2,2)=P1(1,2);P2(2,1)=P2(1,1);P2(2,2)=P2(1,2);P3(2,1)=P3(1,1);P3(2,2)=P3(1,2);P4(2,1)=P4(1,1);P4(2,2)=P4(1,2);

  
syms xk yk zk vxk vyk vzk real

d1k = sqrt((xk-Xbal(1))^2+(yk-Ybal(1))^2+(zk-Zbal(1))^2);
d2k = sqrt((xk-Xbal(2))^2+(yk-Ybal(2))^2+(zk-Zbal(2))^2);
d3k = sqrt((xk-Xbal(3))^2+(yk-Ybal(3))^2+(zk-Zbal(3))^2);
d4k = sqrt((xk-Xbal(4))^2+(yk-Ybal(4))^2+(zk-Zbal(4))^2);

    %Movimiento de camara -------------------------------------------------
    
    wTck =[ cos(psi+pi/4)  0  -sin(psi+pi/4)  xk;
            sin(psi+pi/4)  0   cos(psi+pi/4)  yk;
                  0      -1              0    zk;
                  0       0              0     1];
  
    wTck = wTck*Rx;
             
    centroCk=[xk,yk,zk];
    xCk=wTck*[xW,1]';
    yCk=wTck*[yW,1]';
    zCk=wTck*[zW,1]';
    
    cTwk = inv(wTck);
    cRwk = cTwk(1:3,1:3);
    ctwk = cTwk(1:3,4);
    
    %balizas sistema de referencia camara
    p1k1 = fx*((wp1_(1)-xk)*cRwk(1,1)+(wp1_(2)-yk)*cRwk(1,2)+(wp1_(3)-zk)*cRwk(1,3))/((wp1_(1)-xk)*cRwk(3,1)+(wp1_(2)-yk)*cRwk(3,2)+(wp1_(3)-zk)*cRwk(3,3)) +u0;
    p1k2 = fy*((wp1_(1)-xk)*cRwk(2,1)+(wp1_(2)-yk)*cRwk(2,2)+(wp1_(3)-zk)*cRwk(2,3))/((wp1_(1)-xk)*cRwk(3,1)+(wp1_(2)-yk)*cRwk(3,2)+(wp1_(3)-zk)*cRwk(3,3)) +v0;
    p2k1 = fx*((wp2_(1)-xk)*cRwk(1,1)+(wp2_(2)-yk)*cRwk(1,2)+(wp2_(3)-zk)*cRwk(1,3))/((wp2_(1)-xk)*cRwk(3,1)+(wp2_(2)-yk)*cRwk(3,2)+(wp2_(3)-zk)*cRwk(3,3)) +u0;
    p2k2 = fy*((wp2_(1)-xk)*cRwk(2,1)+(wp2_(2)-yk)*cRwk(2,2)+(wp2_(3)-zk)*cRwk(2,3))/((wp2_(1)-xk)*cRwk(3,1)+(wp2_(2)-yk)*cRwk(3,2)+(wp2_(3)-zk)*cRwk(3,3)) +v0;
    p3k1 = fx*((wp3_(1)-xk)*cRwk(1,1)+(wp3_(2)-yk)*cRwk(1,2)+(wp3_(3)-zk)*cRwk(1,3))/((wp3_(1)-xk)*cRwk(3,1)+(wp3_(2)-yk)*cRwk(3,2)+(wp3_(3)-zk)*cRwk(3,3)) +u0;
    p3k2 = fy*((wp3_(1)-xk)*cRwk(2,1)+(wp3_(2)-yk)*cRwk(2,2)+(wp3_(3)-zk)*cRwk(2,3))/((wp3_(1)-xk)*cRwk(3,1)+(wp3_(2)-yk)*cRwk(3,2)+(wp3_(3)-zk)*cRwk(3,3)) +v0;
    p4k1 = fx*((wp4_(1)-xk)*cRwk(1,1)+(wp4_(2)-yk)*cRwk(1,2)+(wp4_(3)-zk)*cRwk(1,3))/((wp4_(1)-xk)*cRwk(3,1)+(wp4_(2)-yk)*cRwk(3,2)+(wp4_(3)-zk)*cRwk(3,3)) +u0;
    p4k2 = fy*((wp4_(1)-xk)*cRwk(2,1)+(wp4_(2)-yk)*cRwk(2,2)+(wp4_(3)-zk)*cRwk(2,3))/((wp4_(1)-xk)*cRwk(3,1)+(wp4_(2)-yk)*cRwk(3,2)+(wp4_(3)-zk)*cRwk(3,3)) +v0;
    
h1 = simplify([d1k; d2k; d3k; d4k]);
h3 = simplify([p1k1;p1k2;p2k1;p2k2;p3k1;p3k2;p4k1;p4k2]);

H1 = simplify(jacobian(h1, [xk, yk, zk, vxk, vyk, vzk]));
H3 = simplify(jacobian(h3, [xk, yk, zk, vxk, vyk, vzk]));

%% 
n1=0;n2=0;n3=0;n4=0;
m1=0;m2=0;m3=0;m4=0;

%filtro


for k = cont:length(Xgt)
        
    %fase de predicción                                         //Pred
    Xpred(k,:) = A * Xest(k-1,:)' + A * ([0;0;0;vxr(k);vyr(k);vzr(k)]-[0;0;0;Xest(k-1,4:6)']); %pred más IMU
    Sigma_pred = A * Sigma_est * A' + R;
    
    %Movimiento de camara -------------------------------------------------


wTc =[ cos(psi+pi/4)  0  -sin(psi+pi/4)  Xgt(k);
       sin(psi+pi/4)  0  cos(psi+pi/4)  Ygt(k);
              0      -1              0  Zgt(k);
              0       0              0       1];
           
 wTc = wTc*Rx;
             
centroC=[Xgt(k),Ygt(k),Zgt(k)];
xC=wTc*[xW,1]';
yC=wTc*[yW,1]';
zC=wTc*[zW,1]';

% Posición del origen de {C} respecto al sistema {W}
cTw = inv(wTc);
cRw = cTw(1:3,1:3);
ctw = cTw(1:3,4);  

%balizas sistema de referencia camara
p1_(k,1) = fx*((wp1_(1)-Xgt(k))*cRw(1,1)+(wp1_(2)-Ygt(k))*cRw(1,2)+(wp1_(3)-Zgt(k))*cRw(1,3))/((wp1_(1)-Xgt(k))*cRw(3,1)+(wp1_(2)-Ygt(k))*cRw(3,2)+(wp1_(3)-Zgt(k))*cRw(3,3)) +u0;
p1_(k,2) = fy*((wp1_(1)-Xgt(k))*cRw(2,1)+(wp1_(2)-Ygt(k))*cRw(2,2)+(wp1_(3)-Zgt(k))*cRw(2,3))/((wp1_(1)-Xgt(k))*cRw(3,1)+(wp1_(2)-Ygt(k))*cRw(3,2)+(wp1_(3)-Zgt(k))*cRw(3,3)) +v0;
p2_(k,1) = fx*((wp2_(1)-Xgt(k))*cRw(1,1)+(wp2_(2)-Ygt(k))*cRw(1,2)+(wp2_(3)-Zgt(k))*cRw(1,3))/((wp2_(1)-Xgt(k))*cRw(3,1)+(wp2_(2)-Ygt(k))*cRw(3,2)+(wp2_(3)-Zgt(k))*cRw(3,3)) +u0;
p2_(k,2) = fy*((wp2_(1)-Xgt(k))*cRw(2,1)+(wp2_(2)-Ygt(k))*cRw(2,2)+(wp2_(3)-Zgt(k))*cRw(2,3))/((wp2_(1)-Xgt(k))*cRw(3,1)+(wp2_(2)-Ygt(k))*cRw(3,2)+(wp2_(3)-Zgt(k))*cRw(3,3)) +v0;
p3_(k,1) = fx*((wp3_(1)-Xgt(k))*cRw(1,1)+(wp3_(2)-Ygt(k))*cRw(1,2)+(wp3_(3)-Zgt(k))*cRw(1,3))/((wp3_(1)-Xgt(k))*cRw(3,1)+(wp3_(2)-Ygt(k))*cRw(3,2)+(wp3_(3)-Zgt(k))*cRw(3,3)) +u0;
p3_(k,2) = fy*((wp3_(1)-Xgt(k))*cRw(2,1)+(wp3_(2)-Ygt(k))*cRw(2,2)+(wp3_(3)-Zgt(k))*cRw(2,3))/((wp3_(1)-Xgt(k))*cRw(3,1)+(wp3_(2)-Ygt(k))*cRw(3,2)+(wp3_(3)-Zgt(k))*cRw(3,3)) +v0;
p4_(k,1) = fx*((wp4_(1)-Xgt(k))*cRw(1,1)+(wp4_(2)-Ygt(k))*cRw(1,2)+(wp4_(3)-Zgt(k))*cRw(1,3))/((wp4_(1)-Xgt(k))*cRw(3,1)+(wp4_(2)-Ygt(k))*cRw(3,2)+(wp4_(3)-Zgt(k))*cRw(3,3)) +u0;
p4_(k,2) = fy*((wp4_(1)-Xgt(k))*cRw(2,1)+(wp4_(2)-Ygt(k))*cRw(2,2)+(wp4_(3)-Zgt(k))*cRw(2,3))/((wp4_(1)-Xgt(k))*cRw(3,1)+(wp4_(2)-Ygt(k))*cRw(3,2)+(wp4_(3)-Zgt(k))*cRw(3,3)) +v0;
P1(k,:) = p1_(k,1:2) + q3*randn;  %error de camara
P2(k,:) = p2_(k,1:2) + q3*randn;
P3(k,:) = p3_(k,1:2) + q3*randn;
P4(k,:) = p4_(k,1:2) + q3*randn;
%balizas en camara
P1(k,:) = round (P1(k,:)); P2(k,:) = round (P2(k,:)); P3(k,:) = round (P3(k,:)); P4(k,:) = round (P4(k,:));
    %angulo camara-balizas
    %mediante producto escalar, elimina vision negativa
    esc1= dot(zC(1:3)-centroC',zB1(1:3));
    esc2= dot(zC(1:3)-centroC',zB2(1:3));
    esc3= dot(zC(1:3)-centroC',zB3(1:3));
    esc4= dot(zC(1:3)-centroC',zB4(1:3));
    nC= norm(zC(1:3)-centroC');
    n1= norm(zB1(1:3));
    n2= norm(zB2(1:3));
    n3= norm(zB3(1:3));
    n4= norm(zB4(1:3));
    c1=esc1/(nC*n1);
    c2=esc2/(nC*n2);
    c3=esc3/(nC*n3);
    c4=esc4/(nC*n4);
    a1=acos(c1);
    a2=acos(c2);
    a3=acos(c3);
    a4=acos(c4);
    
    %Balizas
    ok1(k)=0; ok2(k)=0; ok3(k)=0;
    if ((a1>=pi/2) && (a1<=pi*3/2)) || ((a1<=-pi/2) && (a1>=-pi*3/2))
        if P1(1,:)>0
            ok1(k)=1;
        end
    end
    if ((a2>=pi/2) && (a2<=pi*3/2)) || ((a2<=-pi/2) && (a2>=-pi*3/2))
        if P2(1,:)>0
            ok2(k)=1;
        end
    end
    if ((a3>=pi/2) && (a3<=pi*3/2)) || ((a3<=-pi/2) && (a3>=-pi*3/2))
        if P3(1,:)>0
            ok3(k)=1;
        end
    end
    %---------------------------------------------------------------
    


    %matriz de observación predicha y función de observación predicha (radio) y selección sensorial
    
    Z(k,:) = [d1(k);d2(k);d3(k);d4(k);P1(k,1);P1(k,2);P2(k,1);P2(k,2);P3(k,1);P3(k,2);P4(k,1);P4(k,2)];
    
      if Z(k,1) == -1    %comprobar si hay medida o no B1
         n1=1;
      end
      if Z(k,2) == -1    %comprobar si hay medida o no B2
         n2=1;
      end
      if Z(k,3) == -1    %comprobar si hay medida o no B3
         n3=1;
      end
      if Z(k,4) == -1    %comprobar si hay medida o no B4
         n4=1;
      end
      
      if P1(k,1)<0 || P1(k,1)>1500 || P1(k,2)<0 || P1(k,2)>1000 %comprobar B1 en imagen
          m1=1;
      end
      if P2(k,1)<0 || P2(k,1)>1500 || P2(k,2)<0 || P2(k,2)>1000 %comprobar B2 en imagen
          m2=1;
      end
      if P3(k,1)<0 || P3(k,1)>1500 || P3(k,2)<0 || P3(k,2)>1000 %comprobar B3 en imagen
          m3=1;
      end
      if P4(k,1)<0 || P4(k,1)>1500 || P4(k,2)<0 || P4(k,2)>1000 %comprobar B4 en imagen
          m4=1;
      end      
      
      h = -1;
      
      if (n4==0)
          %[i,j] = size(Qk); i=7-(i-1);
          h = [h1(4)]; H = [H1(4,:)]; z =Z(k,4); Qk = Q(4,4);
      end
      if (n3==0)
          if(h==-1)
              h = [h1(3)]; H = [H1(3,:)]; z =Z(k,3); Qk = Q(3,3);
          else
              [i,j] = size(Qk); i=4-(i-1);
              h = [h1(3);h]; H = [H1(3,:);H]; z =[Z(k,3) z]; Qk = [Q(3,3) Q(3,i:4); Q(i:4,3) Qk];
          end
      end
      if (n2==0)
          if(h==-1)
              h = [h1(2)]; H = [H1(2,:)]; z =Z(k,2); Qk = Q(2,2);
          else
              [i,j] = size(Qk); i=4-(i-1);
              h = [h1(2);h]; H = [H1(2,:);H]; z =[Z(k,2) z]; Qk = [Q(2,2) Q(2,i:4); Q(i:4,2) Qk];
          end
      end
      if (n1==0) 
          if(h==-1)
              h = [h1(1)]; H = [H1(1,:)]; z =Z(k,1); Qk = Q(1,1);
          else
              [i,j] = size(Qk); i=4-(i-1);
              h = [h1(1);h]; H = [H1(1,:);H]; z =[Z(k,1) z]; Qk = [Q(1,1) Q(1,i:4); Q(i:4,1) Qk];
          end
      end
      
      if m1==0
          if(h==-1)
              h = [h3(1:2)]; H = [H3(1:2,:)]; z =Z(k,5:6); Qk = Q(5:6,5:6);
          else
              [i,j] = size(Qk); i=(i+1);j=(j+2);
              h = [h;h3(1:2)]; H = [H;H3(1:2,:)]; z =[z Z(k,5:6)]; Qk = [Qk Q(1:i-1,i:j); Q(i:j,1:i-1) Q(i:j,i:j)];
          end
      end
      if m2==0
          if(h==-1)
              h = [h3(3:4)]; H = [H3(3:4,:)]; z =Z(k,7:8); Qk = Q(7:8,7:8);
          else
              [i,j] = size(Qk); i=(i+1);j=(j+2);
              h = [h;h3(3:4)]; H = [H;H3(3:4,:)]; z =[z Z(k,7:8)]; Qk = [Qk Q(1:i-1,i:j); Q(i:j,1:i-1) Q(i:j,i:j)];
          end
      end
      if m3==0
          if(h==-1)
              h = [h3(5:6)]; H = [H3(5:6,:)]; z =Z(k,9:10); Qk = Q(9:10,9:10);
          else
              [i,j] = size(Qk); i=(i+1);j=(j+2);
              h = [h;h3(5:6)]; H = [H;H3(5:6,:)]; z =[z Z(k,9:10)]; Qk = [Qk Q(1:i-1,i:j); Q(i:j,1:i-1) Q(i:j,i:j)];
          end
      end
      if m4==0
          if(h==-1)
              h = [h3(7:8)]; H = [H3(7:8,:)]; z =Z(k,11:12); Qk = Q(11:12,11:12);
          else
              [i,j] = size(Qk); i=(i+1);j=(j+2);
              h = [h;h3(7:8)]; H = [H;H3(7:8,:)]; z =[z Z(k,11:12)]; Qk = [Qk Q(1:i-1,i:j); Q(i:j,1:i-1) Q(i:j,i:j)];
          end
      end
      
%   h=-1;   %Descomentar para solo IMU
    if (h~=-1)
    h = subs(h,[xk,yk,zk,vxk,vyk,vzk],[Xpred(k,1),Xpred(k,2),Xpred(k,3),Xpred(k,4),Xpred(k,5),Xpred(k,6)]);
    h = double(h);
    H = subs(H,[xk,yk,zk,vxk,vyk,vzk],[Xpred(k,1),Xpred(k,2),Xpred(k,3),Xpred(k,4),Xpred(k,5),Xpred(k,6)]);
    H = double(H);
    
    
        %fase de actualización                                  ///Act
    K = Sigma_pred * H' * (H * Sigma_pred * H' + Qk)^-1;
    
    Xest(k,:) = Xpred(k,:)' + K * (z' - h);
    Sigma_est = (eye(6) - K * H) * Sigma_pred;
   
    else
        Xest(k,:) = Xpred(k,:)';
        Sigma_est = Sigma_pred;
    end
    
    detSigma(k)=det(Sigma_est);
    n1=0;n2=0;n3=0;n4=0;
    m1=0;m2=0;m3=0;m4=0;
end


%% Graficas ***************************

figure(1);
plot3(Xbal,Ybal,Zbal,'dr');
axis([0 25 -10 10 0 6]);
grid;

hold on
plot3(Xgt,Ygt,Zgt,'b');
hold off    

hold on    
plot3(Xest(:,1),Xest(:,2),Xest(:,3),'m');
legend('Localización balizas','Trayectoria seguida','Filtro EKF');
xlabel('Posición (m)');
ylabel('Posición (m)');
zlabel('Posición (m)');
hold off
  
figure(2);
subplot(4,1,1);
hold on
    plot(t, d1, 'r');
grid;
legend('dist a baliza 1');
hold off
subplot(4,1,2);
hold on
    plot(t, d2, 'g');
grid;
legend('dist a baliza 2');
hold off
subplot(4,1,3); 
hold on
    plot(t, d3, 'b');
grid;
legend('dist a baliza 3');
hold off
subplot(4,1,4); 
hold on
    plot(t, d4, 'm');
grid;
legend('dist a baliza 4');
hold off
    
figure(3);
plot(t,detSigma(t),'b'); % incertidumbres de estimación

%Generación imagen de camara
figure(4);

% Dibujamos marco exterior a la imagen, atendiendo a que el rango va de columnas es [1..N] y de filas [1..M]:
plot([0,0,N+1,N+1,0],[0,M+1,M+1,0,0],'Color','k', 'LineWidth',2);

set(gca,'YDir', 'reverse');
    
axis([-50,N+50,-50,M+50]);
hold on; 
%Balizas
ok1=zeros(length(Xgt),1); ok2=zeros(length(Xgt),1); ok3=zeros(length(Xgt),1);
if ((a1>=pi/2) && (a1<=pi*3/2)) || ((a1<=-pi/2) && (a1>=-pi*3/2))
if P1(1,:)>0 
    s1="\{ID_1\}";
    text(P1(1,1), P1(1,2),s1);
plot(P1(1,1), P1(1,2),'*k')
ok1(1)=1;
end
end
if ((a2>=pi/2) && (a2<=pi*3/2)) || ((a2<=-pi/2) && (a2>=-pi*3/2))
if P2(1,:)>0 
    s2="\{ID_2\}";
    text(P2(1,1), P2(1,2),s2);    
plot(P2(1,1), P2(1,2),'*k')
ok2(1)=1;
end
end
if ((a3>=pi/2) && (a3<=pi*3/2)) || ((a3<=-pi/2) && (a3>=-pi*3/2))
if P3(1,:)>0 
    s3="\{ID_3\}";
    text(P3(1,1), P3(1,2),s3);
plot(P3(1,1), P3(1,2),'*k')
ok3(1)=1;
end
end
grid on;
hold off;
