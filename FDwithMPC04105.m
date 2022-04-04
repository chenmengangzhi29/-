%角速度控制量变为线速度控制量之后
tic%计时开始
%第二色到第七色  穿料长度不相等的情况
clear;clc;
% close all;

L1=6.56;%穿料长度（m）
L2=7.08;
L3=6.08;
L4=7.91;
L5=7.33;
L6=6.89;

C=0.68;%印刷版辊周长（m） r=0.1083(m)
V=100/60;%印刷速度（m/s）

a1=V/L1;a2=V/L2;a3=V/L3;a4=V/L4;a5=V/L5;a6=V/L6;%(1/s)

Ts=C/V;%采样周期（s）

Nr1=round(L1/C);Nr2=round(L2/C);Nr3=round(L3/C);%
Nr4=round(L4/C);Nr5=round(L5/C);Nr6=round(L6/C);

r=C/(3.14159*2);%印刷版辊半径(m)
%线性系统系数
A=[1-1/Nr1 0 0 0 0 0;
   0 1-1/Nr2 0 0 0 0;
   0 0 1-1/Nr3 0 0 0;
   0 0 0 1-1/Nr4 0 0;
   0 0 0 0 1-1/Nr5 0;
   0 0 0 0 0 1-1/Nr6];
B=[1/Nr1 0 0 0 0 0;%线速度控制量1/Nr1
   0 1/Nr2 0 0 0 0;
   0 0 1/Nr3 0 0 0;
   0 0 0 1/Nr4 0 0;
   0 0 0 0 1/Nr5 0;
   0 0 0 0 0 1/Nr6];
%初始误差量
e0=[300;0;0;0;0;0];%0.01mm
%预测步长
Np=5;
%优化目标参数，加权矩阵
Q=[1 0 0 0 0 0;
   0 1 0 0 0 0;  
   0 0 1 0 0 0;
   0 0 0 1 0 0;
   0 0 0 0 1 0;
   0 0 0 0 0 1];
R=[1 0 0 0 0 0;
   0 1 0 0 0 0;
   0 0 1 0 0 0;
   0 0 0 1 0 0;
   0 0 0 0 1 0;
   0 0 0 0 0 1];
%转化为用控制量ut表示的，关于误差量的推导方程的矩阵
At=[];Bt=[];temp=[];
%转换后的加权矩阵
Qt=[];Rt=[];


%传递函数

G=tf([a2],[1,a2]);%第三色误差
G3=c2d(G,Ts);
G=tf([a1*a2],[1,a1+a2,a1*a2]);
G321=c2d(G,Ts);
G=tf([a2],[1,a2]);
G322=c2d(G,Ts);

G=tf([a3],[1,a3]);%第四色误差
G4=c2d(G,Ts);
G=tf([a1*a2*a3],[1,a1+a2+a3,a1*a2+a1*a3+a2*a3,a1*a2*a3]);
G421=c2d(G,Ts);
G=tf([a2*a3],[1,a2+a3,a2*a3]);
G422=c2d(G,Ts);
G=tf([a2*a3],[1,a2+a3,a2*a3]);
G431=c2d(G,Ts);
G=tf([a3],[1,a3]);
G432=c2d(G,Ts);

G=tf([a4],[1,a4]);%第五色误差
G5=c2d(G,Ts);
G=tf([a1*a2*a3*a4],[1,a1+a2+a3+a4,a1*a2+a1*a3+a1*a4+a2*a3+a2*a4+a3*a4,a1*a2*a3+a1*a2*a4+a1*a3*a4+a2*a3*a4,a1*a2*a3*a4]);
G521=c2d(G,Ts);
G=tf([a2*a3*a4],[1,a2+a3+a4,a2*a3+a2*a4+a3*a4,a2*a3*a4]);
G522=c2d(G,Ts);
G=tf([a2*a3*a4],[1,a2+a3+a4,a2*a3+a2*a4+a3*a4,a2*a3*a4]);
G531=c2d(G,Ts);
G=tf([a3*a4],[1,a3+a4,a3*a4]);
G532=c2d(G,Ts);
G=tf([a3*a4],[1,a3+a4,a3*a4]);
G541=c2d(G,Ts);
G=tf([a4],[1,a4]);
G542=c2d(G,Ts);

G=tf([a5],[1,a5]);%第六色误差
G6=c2d(G,Ts);
G=tf([a1*a2*a3*a4*a5],[1,a1+a2+a3+a4+a5,a1*a2+a1*a3+a1*a4+a1*a5+a2*a3+a2*a4+a2*a5+a3*a4+a3*a5+a4*a5,a1*a2*a3+a1*a2*a4+a1*a2*a5+a1*a3*a4+a1*a3*a5+a1*a4*a5+a2*a3*a4+a2*a3*a5+a2*a4*a5+a3*a4*a5,a1*a2*a3*a4+a1*a2*a3*a5+a1*a2*a4*a5+a2*a3*a4*a5+a1*a3*a4*a5,a1*a2*a3*a4*a5]);
G621=c2d(G,Ts);
G=tf([a2*a3*a4*a5],[1,a2+a3+a4+a5,a2*a3+a2*a4+a2*a5+a3*a4+a3*a5+a4*a5,a2*a3*a4+a2*a3*a5+a2*a4*a5+a3*a4*a5,a2*a3*a4*a5]);
G622=c2d(G,Ts);
G=tf([a2*a3*a4*a5],[1,a2+a3+a4+a5,a2*a3+a2*a4+a2*a5+a3*a4+a3*a5+a4*a5,a2*a3*a4+a2*a3*a5+a2*a4*a5+a3*a4*a5,a2*a3*a4*a5]);
G631=c2d(G,Ts);
G=tf([a3*a4*a5],[1,a3+a4+a5,a3*a4+a3*a5+a4*a5,a3*a4*a5]);
G632=c2d(G,Ts);
G=tf([a3*a4*a5],[1,a3+a4+a5,a3*a4+a3*a5+a4*a5,a3*a4*a5]);
G641=c2d(G,Ts);
G=tf([a4*a5],[1,a4+a5,a4*a5]);
G642=c2d(G,Ts);
G=tf([a4*a5],[1,a4+a5,a4*a5]);
G651=c2d(G,Ts);
G=tf([a5],[1,a5]);
G652=c2d(G,Ts);

G=tf([a6],[1,a6]);%第七色误差
G7=c2d(G,Ts);
G=tf([a1*a2*a3*a4*a5*a6],[1,a1+a2+a3+a4+a5+a6,a1*a2+a1*a3+a1*a4+a1*a5+a1*a6+a2*a3+a2*a4+a2*a5+a2*a6+a3*a4+a3*a5+a3*a6+a4*a5+a4*a6+a5*a6,a1*a2*a3+a1*a2*a4+a1*a2*a5+a1*a2*a6+a1*a3*a4+a1*a3*a5+a1*a3*a6+a1*a4*a5+a1*a4*a6+a1*a5*a6+a2*a3*a4+a2*a3*a5+a2*a3*a6+a2*a4*a5+a2*a4*a6+a2*a5*a6+a3*a4*a5+a3*a4*a6+a3*a5*a6+a4*a5*a6,a1*a2*a3*a4+a1*a2*a3*a5+a1*a2*a3*a6+a1*a2*a4*a5+a1*a2*a4*a6+a1*a2*a5*a6+a1*a3*a4*a5+a1*a3*a4*a6+a1*a3*a5*a6+a1*a4*a5*a6+a2*a3*a4*a5+a2*a3*a4*a6+a2*a3*a5*a6+a2*a4*a5*a6+a3*a4*a5*a6,a1*a2*a3*a4*a5+a1*a2*a3*a4*a6+a1*a2*a3*a5*a6+a1*a2*a4*a5*a6+a1*a3*a4*a5*a6+a2*a3*a4*a5*a6,a1*a2*a3*a4*a5*a6]);
G721=c2d(G,Ts);
G=tf([a2*a3*a4*a5*a6],[1,a2+a3+a4+a5+a6,a2*a3+a2*a4+a2*a5+a2*a6+a3*a4+a3*a5+a3*a6+a4*a5+a4*a6+a5*a6,a2*a3*a4+a2*a3*a5+a2*a3*a6+a2*a4*a5+a2*a4*a6+a2*a5*a6+a3*a4*a5+a3*a4*a6+a3*a5*a6+a4*a5*a6,a2*a3*a4*a5+a2*a3*a4*a6+a2*a3*a5*a6+a2*a4*a5*a6+a3*a4*a5*a6,a2*a3*a4*a5*a6]);
G722=c2d(G,Ts);
G=tf([a2*a3*a4*a5*a6],[1,a2+a3+a4+a5+a6,a2*a3+a2*a4+a2*a5+a2*a6+a3*a4+a3*a5+a3*a6+a4*a5+a4*a6+a5*a6,a2*a3*a4+a2*a3*a5+a2*a3*a6+a2*a4*a5+a2*a4*a6+a2*a5*a6+a3*a4*a5+a3*a4*a6+a3*a5*a6+a4*a5*a6,a2*a3*a4*a5+a2*a3*a4*a6+a2*a3*a5*a6+a2*a4*a5*a6+a3*a4*a5*a6,a2*a3*a4*a5*a6]);
G731=c2d(G,Ts);
G=tf([a3*a4*a5*a6],[1,a3+a4+a5+a6,a3*a4+a3*a5+a3*a6+a4*a5+a4*a6+a5*a6,a3*a4*a5+a3*a4*a6+a3*a5*a6+a4*a5*a6,a3*a4*a5*a6]);
G732=c2d(G,Ts);
G=tf([a3*a4*a5*a6],[1,a3+a4+a5+a6,a3*a4+a3*a5+a3*a6+a4*a5+a4*a6+a5*a6,a3*a4*a5+a3*a4*a6+a3*a5*a6+a4*a5*a6,a3*a4*a5*a6]);
G741=c2d(G,Ts);
G=tf([a4*a5*a6],[1,a4+a5+a6,a4*a5+a4*a6+a5*a6,a4*a5*a6]);
G742=c2d(G,Ts);
G=tf([a4*a5*a6],[1,a4+a5+a6,a4*a5+a4*a6+a5*a6,a4*a5*a6]);
G751=c2d(G,Ts);
G=tf([a5*a6],[1,a5+a6,a5*a6]);
G752=c2d(G,Ts);
G=tf([a5*a6],[1,a5+a6,a5*a6]);
G761=c2d(G,Ts);
G=tf([a6],[1,a6]);
G762=c2d(G,Ts);


%加权矩阵的计算过程，以及推导方程矩阵的叠加过程
for i=1:Np
    At=[At;A^i];
    Bt=[Bt zeros(size(Bt,1),size(B,2));
        A^(i-1)*B temp];
    temp=[A^(i-1)*B temp];
    
    Qt=[Qt zeros(size(Qt,1),size(Q,1));
        zeros(size(Q,1),size(Qt,1)) Q];
    Rt=[Rt zeros(size(Rt,1),size(R,1));
        zeros(size(R,1),size(Rt,1)) R];
end
%控制量ut的初始值
u0=zeros(6*Np,1);%0.01mm
%转换后的优化目标函数系数矩阵，循环优化函数中H后的表达式为优化目标的另一项
H=2*(Bt'*Qt*Bt+Rt);
u21(1:3)=0;%第二色的MPC控制量
u2(1:3)=0;%第二色的实际控制量
% u2(4:200)=0;
u31(1:3)=0;%第三色的MPC控制量
u3(1:3)=0;%第三色的实际控制量
% u3(4:200)=0;
u41(1:3)=0;%第四色的MPC控制量
u4(1:3)=0;%第四色的实际控制量
u51(1:3)=0;%第五色的MPC控制量
u5(1:3)=0;%第五色的实际控制量
u61(1:3)=0;%第六色的MPC控制量
u6(1:3)=0;%第六色的实际控制量
u71(1:3)=0;%第七色的MPC控制量
u7(1:3)=0;%第七色的实际控制量

u2b(1:3)=0;%对第二色的补偿量
u3b(1:3)=0;
u4b(1:3)=0;
u5b(1:3)=0;
u6b(1:3)=0;

w2(1:3)=0;%第二色原本的MPC控制量（不加积分）
w3(1:3)=0;
w4(1:3)=0;
w5(1:3)=0;
w6(1:3)=0;
w7(1:3)=0;


e2(1:3)=e0(1,:);%第二色实际误差
e3(1:3,1)=e0(2,:);%第三色实际误差
e4(1:3,1)=e0(3,:);%第四色实际误差
e5(1:3,1)=e0(4,:);%第五色实际误差
e6(1:3,1)=e0(5,:);%第六色实际误差
e7(1:3,1)=e0(6,:);%第七色实际误差
ek=e0;

for k = 3:200
    t = 0:Ts:Ts*(k-1);
%     一切准备就绪，进行二次优化
% 1.采用二次优化函数优化出ut
%     [ut,fval,exitflag]=quadprog(H,(2*Bt'*Qt'*At*ek),[],[],[],[],[],[],u0);
%     %显示求解结果是否正常
%     fprintf('%d\n',exitflag)

%2.最优控制量序列计算出ut
%     %最优控制量序列-------------------------------------------------------
%     Copt=-(Bt'*Qt*Bt+Rt)\(Bt'*Qt'*At);
%     %优化的k时刻的控制量序列
%     ut=Copt*ek;

% 3.根据最优控制量序列简化出的公式，计算出ut
%       ut(1)=-(ek(1)*(5*Nr1^9 - 25*Nr1^8 + 90*Nr1^7 - 210*Nr1^6 + 349*Nr1^5 - 417*Nr1^4 + 356*Nr1^3 - 212*Nr1^2 + 80*Nr1 - 16))/(Nr1^10 + 15*Nr1^8 - 40*Nr1^7 + 115*Nr1^6 - 192*Nr1^5 + 257*Nr1^4 - 232*Nr1^3 + 156*Nr1^2 - 64*Nr1 + 16);
%       ut(2)=-(ek(2)*(5*Nr2^9 - 25*Nr2^8 + 90*Nr2^7 - 210*Nr2^6 + 349*Nr2^5 - 417*Nr2^4 + 356*Nr2^3 - 212*Nr2^2 + 80*Nr2 - 16))/(Nr2^10 + 15*Nr2^8 - 40*Nr2^7 + 115*Nr2^6 - 192*Nr2^5 + 257*Nr2^4 - 232*Nr2^3 + 156*Nr2^2 - 64*Nr2 + 16);
%       ut(3)=-(ek(3)*(5*Nr3^9 - 25*Nr3^8 + 90*Nr3^7 - 210*Nr3^6 + 349*Nr3^5 - 417*Nr3^4 + 356*Nr3^3 - 212*Nr3^2 + 80*Nr3 - 16))/(Nr3^10 + 15*Nr3^8 - 40*Nr3^7 + 115*Nr3^6 - 192*Nr3^5 + 257*Nr3^4 - 232*Nr3^3 + 156*Nr3^2 - 64*Nr3 + 16);
%       ut(4)=-(ek(4)*(5*Nr4^9 - 25*Nr4^8 + 90*Nr4^7 - 210*Nr4^6 + 349*Nr4^5 - 417*Nr4^4 + 356*Nr4^3 - 212*Nr4^2 + 80*Nr4 - 16))/(Nr4^10 + 15*Nr4^8 - 40*Nr4^7 + 115*Nr4^6 - 192*Nr4^5 + 257*Nr4^4 - 232*Nr4^3 + 156*Nr4^2 - 64*Nr4 + 16);
%       ut(5)=-(ek(5)*(5*Nr5^9 - 25*Nr5^8 + 90*Nr5^7 - 210*Nr5^6 + 349*Nr5^5 - 417*Nr5^4 + 356*Nr5^3 - 212*Nr5^2 + 80*Nr5 - 16))/(Nr5^10 + 15*Nr5^8 - 40*Nr5^7 + 115*Nr5^6 - 192*Nr5^5 + 257*Nr5^4 - 232*Nr5^3 + 156*Nr5^2 - 64*Nr5 + 16);
%       ut(6)=-(ek(6)*(5*Nr6^9 - 25*Nr6^8 + 90*Nr6^7 - 210*Nr6^6 + 349*Nr6^5 - 417*Nr6^4 + 356*Nr6^3 - 212*Nr6^2 + 80*Nr6 - 16))/(Nr6^10 + 15*Nr6^8 - 40*Nr6^7 + 115*Nr6^6 - 192*Nr6^5 + 257*Nr6^4 - 232*Nr6^3 + 156*Nr6^2 - 64*Nr6 + 16);
      
%4.当Nri和r确定时，公式再次简化成一个常数
      ut(1)=-ek(1)*0.2876;
      ut(2)=-ek(2)*0.2876;
      ut(3)=-ek(3)*0.3000;
      ut(4)=-ek(4)*0.2636;
      ut(5)=-ek(5)*0.2754;
      ut(6)=-ek(6)*0.2876;

    %采用优化得到的MPC控制量ut的第一个元素作为第二色实际作用的MPC控制量，第二个元素作为第三色实际作用的MPC控制量，
    %分别根据得到MPC控制量算出实际控制量（补偿量+MPC控制量）
    %将实际控制量代入实际系统得到实际误差
    %再将实际误差继续二次优化出下一时刻的MPC控制量ut
    u21(k)=ut(1);%第二色MPC控制量
    u31(k)=ut(2);%第三色MPC控制量
    u41(k)=ut(3);
    u51(k)=ut(4);
    u61(k)=ut(5);
    u71(k)=ut(6);
    

    w2(k)=(u21(k)-u21(k-1));%第二色原本的MPC控制量（不加积分）
    w3(k)=(u31(k)-u31(k-1));%第三色原本的MPC控制量（不加积分）
    w4(k)=(u41(k)-u41(k-1));
    w5(k)=(u51(k)-u51(k-1));
    w6(k)=(u61(k)-u61(k-1));
    w7(k)=(u71(k)-u71(k-1));
    
    u2b(k)=(1-1/Nr1)*u2b(k-1)+u21(k)-u21(k-1);%对第二色补偿量计算
    u3b(k)=(1-1/Nr2)*u3b(k-1)+u31(k)-u31(k-1);
    u4b(k)=(1-1/Nr3)*u4b(k-1)+u41(k)-u41(k-1);
    u5b(k)=(1-1/Nr4)*u5b(k-1)+u51(k)-u51(k-1);
    u6b(k)=(1-1/Nr5)*u6b(k-1)+u61(k)-u61(k-1);
    
    u2(k)=u21(k);%第二色实际控制量
    u3(k)=u31(k)+u2b(k);%第三色实际控制量
    u4(k)=u41(k)+u2b(k)+u3b(k);
    u5(k)=u51(k)+u2b(k)+u3b(k)+u4b(k);
    u6(k)=u61(k)+u2b(k)+u3b(k)+u4b(k)+u5b(k);
    u7(k)=u71(k)+u2b(k)+u3b(k)+u4b(k)+u5b(k)+u6b(k);

    e2(k)=(1-1/Nr1)*e2(k-1)+1/Nr1*u2(k-1);%算出第二色的实际误差量  
    e3=lsim(G3,u3(1:k),t)+lsim(G321,u2(1:k),t)-lsim(G322,u2(1:k),t);%算出第三色的实际误差量
    e4=lsim(G4,u4(1:k),t)+lsim(G421,u2(1:k),t)-lsim(G422,u2(1:k),t)+lsim(G431,u3(1:k),t)-lsim(G432,u3(1:k),t);
    e5=lsim(G5,u5(1:k),t)+lsim(G521,u2(1:k),t)-lsim(G522,u2(1:k),t)+lsim(G531,u3(1:k),t)-lsim(G532,u3(1:k),t)+lsim(G541,u4(1:k),t)-lsim(G542,u4(1:k),t);
    e6=lsim(G6,u6(1:k),t)+lsim(G621,u2(1:k),t)-lsim(G622,u2(1:k),t)+lsim(G631,u3(1:k),t)-lsim(G632,u3(1:k),t)+lsim(G641,u4(1:k),t)-lsim(G642,u4(1:k),t)+lsim(G651,u5(1:k),t)-lsim(G652,u5(1:k),t);
    e7=lsim(G7,u7(1:k),t)+lsim(G721,u2(1:k),t)-lsim(G722,u2(1:k),t)+lsim(G731,u3(1:k),t)-lsim(G732,u3(1:k),t)+lsim(G741,u4(1:k),t)-lsim(G742,u4(1:k),t)+lsim(G751,u5(1:k),t)-lsim(G752,u5(1:k),t)+lsim(G761,u6(1:k),t)-lsim(G762,u6(1:k),t);
    ek=[e2(k);e3(k);e4(k);e5(k);e6(k);e7(k)];
    %对优化初始值进行修改，采用预测值的后段作为下一步的初始值
%     u0=[ut(7:6*Np);ut(6*Np);ut(6*Np);ut(6*Np);ut(6*Np);ut(6*Np);ut(6*Np)];
    
end


plot(e2(1,2:200),'LineWidth',2,'DisplayName','E2完全解耦模型预测控制');
hold all
plot(e3(2:200,1),'LineWidth',2,'DisplayName','E3完全解耦模型预测控制');
plot(e4(2:200,1),'LineWidth',2,'DisplayName','E4完全解耦模型预测控制');
plot(e5(2:200,1),'LineWidth',2,'DisplayName','E5完全解耦模型预测控制');
plot(e6(2:200,1),'LineWidth',2,'DisplayName','E6完全解耦模型预测控制');
plot(e7(2:200,1),'LineWidth',2,'DisplayName','E7完全解耦模型预测控制');
xlabel('转');
ylabel('误差（0.01mm)');


plot(u2(1,2:200),'LineWidth',2,'DisplayName','u2完全解耦模型预测控制');
hold all
plot(u3(1,3:200),'LineWidth',2,'DisplayName','u3完全解耦模型预测控制');
plot(u4(1,3:200),'LineWidth',2,'DisplayName','u4完全解耦模型预测控制');
plot(u5(1,3:200),'LineWidth',2,'DisplayName','u5完全解耦模型预测控制');
plot(u6(1,3:200),'LineWidth',2,'DisplayName','u6完全解耦模型预测控制');
plot(u7(1,3:200),'LineWidth',2,'DisplayName','u7完全解耦模型预测控制');
xlabel('转');
ylabel('控制量（0.01mm)');

P2 = 0;
for K = 2:43
    P2 = P2 + abs(u2(K-1)-u2(K));
end


% plot(u21(1,3:200),'LineWidth',2,'DisplayName','u21完全解耦模型预测控制');
% plot(u31(1,3:200),'LineWidth',2,'DisplayName','u31完全解耦模型预测控制');
% plot(w2(1,3:200),'LineWidth',2,'DisplayName','w2完全解耦模型预测控制');
% plot(u2(1,3:200),'LineWidth',2,'DisplayName','u2完全解耦模型预测控制');
% xlabel('转');
% ylabel('控制量（0.01mm)');

toc%计时结束，自动计算此时与最近一次tic之间的时间