%PROGETTO COMPLETO
%inversione con funzionale e non


clear all
close all
clc

% Inizializzazione delle variabili

q0 = [pi/9 pi/9 pi/4 pi/3];

qd(1,:) = q0;
qd_obs(1,:)=q0;

a1 = 0.08;
a2 = 0.12;
a3 = 0.05;
a4 = 0.02;

d1=0.065;
d2=0;
d3=0;
d4=0.065;


a = [a1, a2, a3,a4];
d = [d1, d2, d3,d4];
angle=[pi/2,-pi/2,pi/2,0];
lmax = sum(a);

T01 = DH_computation(d(1), a(1), angle(1), q0(1));
T12 = DH_computation(d(2), a(2), angle(2), q0(2));
T23 = DH_computation(d(3), a(3), angle(3), q0(3));
T34 = DH_computation(d(4), a(4), angle(4), q0(4));

T04 = T01*T12*T23*T34;

XY_i = DirectKinematics(T04);


tA = 0;
tB = 5;
tC = 10;
dt = 0.01;

t = tA:dt:tC;

XYd(1,:)=XY_i;
XYddot(1,:)=[0, 0, 0];
XYe(1,:)=XY_i;
XYe_man(1,:)=XY_i;

XY_B =[0.20, 0.08, 0.1525]';  
XY_f = [0.16, 0.078,0.1525]';

o = [0.14679, 0.153707, 0.162039] ;
W(1,1)= norm(XY_i-o);
W_fun(1,1)=norm(XY_i-o);


%% per ogni istante di tempo eseguo l'algoritmo di inversione cinematica
for i = 1:length(t)
    if t(i)<=tB
   
    [XYd(i,:), XYddot(i,:)]=planner_circonferenza(XY_i,XY_B, tA, tB, t(i),pi);

    else 
        [XYd(i,:), XYddot(i,:)]=planner_circonferenza(XY_B,XY_f, tB, tC, t(i),5*pi/4);

    end
    
    
    [qddot(i,:),e(i,:)] = InverseKinematics(qd(i,:),a,d,angle, XYd(i,:), XYddot(i,:),50,"i");
    [qddot_obs(i,:),e_obs(i,:)] = InverseKinematicsobs2_copia(qd_obs(i,:),a,d,angle,XYd(i,:),XYddot(i,:),50,"i",30,o)
    
    %alzando troppo ka alzo anche l'errore
    qd(i+1,:) = qd(i,:) + qddot(i,:) * dt;
    qd_obs(i+1,:) = qd_obs(i,:) + qddot_obs(i,:) * dt;

    % Calcolo la posizione corrente dell'end-effector del manipolatore
    T01 = DH_computation(d(1), a(1), angle(1), qd(i,1));
    T12 = DH_computation(d(2), a(2), angle(2), qd(i,2));
    T23 = DH_computation(d(3), a(3), angle(3), qd(i,3));
    T34 = DH_computation(d(4), a(4), angle(4), qd(i,4));

    T02 = T01*T12;
    T03 = T01*T12*T23;
    T04 = T01*T12*T23*T34;
    
    XY1(i,:) = DirectKinematics(T01);
    XY2(i,:) = DirectKinematics(T02);
    XY3(i,:) = DirectKinematics(T03);
    XYe(i,:) = DirectKinematics(T04);

    
    J = Jacobian_4dof(qd(i,:), a,d);
    J = J(1:3,:);
    W(i,1) = norm(XY3(i,:)'-o);


    % Calcolo la posizione corrente dell'end-effector del manipolatore con
    % fun 
    T01 = DH_computation(d(1), a(1), angle(1), qd_obs(i,1));
    T12 = DH_computation(d(2), a(2), angle(2), qd_obs(i,2));
    T23 = DH_computation(d(3), a(3), angle(3), qd_obs(i,3));
    T34 = DH_computation(d(4), a(4), angle(4), qd_obs(i,4));

    T02 = T01*T12;
    T03 = T01*T12*T23;
    T04 = T01*T12*T23*T34;
    
    XY1_obs(i,:) = DirectKinematics(T01);
    XY2_obs(i,:) = DirectKinematics(T02);
    XY3_obs(i,:) = DirectKinematics(T03);
    XYe_obs(i,:) = DirectKinematics(T04);

    J_obs = Jacobian_4dof(qd_obs(i,:), a,d);
    J_obs = J_obs(1:3,:);
    W_fun(i,1) = norm(XY3_obs(i,:)'-o);
end


%plot traiettoria deisderata
plot3(XYd(:,1), XYd(:,2), XYd(:,3), 'LineWidth', 3)
hold on
plot3(XYd(1,1), XYd(1,2), XYd(1,3), '.k', 'MarkerSize',20)

plot3(XYd(end,1), XYd(end,2),XYd(end,3), '.k', 'MarkerSize',20)
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
zlim([-2 20])

close all
f2 = figure(2);
f2.OuterPosition = [115,132,1200,712];
f2.Color = [1 1 1];

for i = 1:15:length(t)
    
    sgt = sgtitle('4 DoF Manipulator','Color','black');
    sgt.FontSize = 20;
    sgt.FontWeight = 'bold';
    sgt.Interpreter = 'latex';
    
    subplot(221)
    plot3([0],[0],[0],'.k','MarkerSize',20)
    hold on
    plot3([0],[0],[0],'.k','MarkerSize',20)
    hold on
    plot3([0 XY1(i,1)],[0 XY1(i,2)],[0 XY1(i,3)],'-r','Linewidth',4)
    plot3([0],[0],[0],'.k','MarkerSize',20)
    plot3([XY1(i,1) XY2(i,1)],[XY1(i,2) XY2(i,2)],[XY1(i,3) XY2(i,3)],'-b','Linewidth',4)
    plot3([XY1(i,1)],[XY1(i,2)],[XY1(i,3)],'.k','MarkerSize',20)
    plot3([XY2(i,1) XY3(i,1)],[XY2(i,2) XY3(i,2)],[XY2(i,3) XY3(i,3)],'-g','Linewidth',4)
    plot3([XY2(i,1)],[XY2(i,2)],[XY2(i,3)],'.k','MarkerSize',20)
    plot3([XY3(i,1) XYe(i,1)],[XY3(i,2) XYe(i,2)],[XY3(i,3) XYe(i,3)],'-k','Linewidth',4)
    plot3([XY3(i,1)],[XY3(i,2)],[XY3(i,3)],'.k','MarkerSize',20)
    

    plot3(o(1),o(2),o(3),'.r','MarkerSize',20)

    plot3(XYd(1:i,1),XYd(1:i,2),XYd(1:i,3),'--r','LineWidth',3)
    plot3(XYe(1:i,1),XYe(1:i,2),XYe(1:i,3),'-b','LineWidth',1.5)
    
    plot3(XY_f(1),XY_f(2),XY_f(3),'k*','MarkerSize',20)
    text(XY_f(1)+0.005,XY_f(2),XY_f(3),'$p_f$','Interpreter','latex','FontSize',16)
    plot3(XY_i(1),XY_i(2),XY_i(3),'k*','MarkerSize',20)
    text(XY_i(1)+0.005,XY_i(2),XY_i(3),'$p_i$','Interpreter','latex','FontSize',16)
    
    xlim([-0.1 0.4])
    ylim([-0.1 0.3])
    
    
    xlim([-lmax lmax])
    ylim([-lmax lmax]) 
    
    xlabel('x [m]','Interpreter','latex')
    ylabel('y [m]','Interpreter','latex')
    
    set(gca,'FontSize',16)
    hold off
    grid on
    title("SENZA FUNZIONALE",'Interpreter','latex')
    
    subplot(222)

    plot3([0],[0],[0],'.k','MarkerSize',20)
    hold on
    plot3([0],[0],[0],'.k','MarkerSize',20)
    hold on
    plot3([0 XY1_obs(i,1)],[0 XY1_obs(i,2)],[0 XY1_obs(i,3)],'-r','Linewidth',4)
    plot3([0],[0],[0],'.k','MarkerSize',20)
    plot3([XY1_obs(i,1) XY2_obs(i,1)],[XY1_obs(i,2) XY2_obs(i,2)],[XY1_obs(i,3) XY2_obs(i,3)],'-b','Linewidth',4)
    plot3([XY1_obs(i,1)],[XY1_obs(i,2)],[XY1_obs(i,3)],'.k','MarkerSize',20)
    plot3([XY2_obs(i,1) XY3_obs(i,1)],[XY2_obs(i,2) XY3_obs(i,2)],[XY2_obs(i,3) XY3_obs(i,3)],'-g','Linewidth',4)
    plot3([XY2_obs(i,1)],[XY2_obs(i,2)],[XY2_obs(i,3)],'.k','MarkerSize',20)
    plot3([XY3_obs(i,1) XYe_obs(i,1)],[XY3_obs(i,2) XYe_obs(i,2)],[XY3_obs(i,3) XYe_obs(i,3)],'-k','Linewidth',4)
    plot3([XY3_obs(i,1)],[XY3_obs(i,2)],[XY3_obs(i,3)],'.k','MarkerSize',20)

    plot3(o(1),o(2),o(3),'.r','MarkerSize',20)


    plot3(XYd(1:i,1),XYd(1:i,2),XYd(1:i,3),'--r','LineWidth',3)
    plot3(XYe_obs(1:i,1),XYe_obs(1:i,2),XYe_obs(1:i,3),'-b','LineWidth',1.5)
    
    plot3(XY_f(1),XY_f(2),XY_f(3),'k*','MarkerSize',20)
    text(XY_f(1)+0.005,XY_f(2),XY_f(3),'$p_f$','Interpreter','latex','FontSize',16)
    plot3(XY_i(1),XY_i(2),XY_i(3),'k*','MarkerSize',20)
    text(XY_i(1)+0.005,XY_i(2),XY_i(3),'$p_i$','Interpreter','latex','FontSize',16)
    
    xlim([-0.05 0.4])
    ylim([-0.05 0.3])

    xlim([-lmax lmax])
    ylim([-lmax lmax])
    
    xlabel('x [m]','Interpreter','latex')
    ylabel('y [m]','Interpreter','latex')
    
    set(gca,'FontSize',16)
    hold off
    grid on
    title("CON FUNZIONALE",'Interpreter','latex')
    
    

    subplot(2,2,[3:4])
    hold on 
    plot(t(1:i), W(1:i), 'LineWidth', 3) 
    plot(t(1:i), W_fun(1:i), 'LineWidth', 3)
    
    xlim([0 t(end)])
    %ylim([0.08 0.22])
    xlabel("Time [s]",'FontSize',18,'Interpreter','latex')
    ylabel("$\omega [m]$",'FontSize',18,'Interpreter','latex')
    set(gca,'FontSize',16)
    lg = legend("SENZA FUNZIONALE", "CON FUNZIONALE");
    lg.FontSize = 18;
    lg.Interpreter = 'latex';
    lg.Location = 'bestoutside';
    
    pause(0.01)
    
end

figure(2)
 for i=1:length(t)
     plot3((XYd(1:i,1)),XYd(1:i,2),XYd(1:i,3),'r','LineWidth',3)
    
 end

%Traiettoria desiderata so 
f = figure();
f.OuterPosition = [100 50 800 800];
subplot(3,1,1)
plot(t,XYd(:,1),'LineWidth',4)
title('Posizione desiderata SO')
xlabel('time [s]')
ylabel('x [m]')
set(gca,'FontSize',20)
subplot(3,1,2)
plot(t,XYd(:,2),'LineWidth',4)
xlabel('time [s]')
ylabel('y [m]')
set(gca,'FontSize',20)
subplot(3,1,3)
plot(t,XYd(:,3),'LineWidth',4)
xlabel('time [s]')
ylabel('z [m]')
set(gca,'FontSize',20)




%% Posizioni Desiderate ed Effettive ostacolo 

close all
f = figure();
f.OuterPosition = [100 50 800 800];
subplot(3,1,1)
plot(t,XYd(:,1),'LineWidth',4)
hold on
plot(t,XYe_obs(:,1),'LineWidth',4)
title('Posizione desiderata ed effettiva')
xlabel('time [s]')
ylabel('x [m]')
set(gca,'FontSize',20)
subplot(3,1,2)
plot(t,XYd(:,2),'LineWidth',4)
hold on
plot(t,XYe_obs(:,2),'LineWidth',4)
xlabel('time [s]')
ylabel('y [m]')
set(gca,'FontSize',20)
subplot(3,1,3)
plot(t,XYd(:,3),'LineWidth',4)
hold on
plot(t,XYe_obs(:,3),'LineWidth',4)
xlabel('time [s]')
ylabel('z [m]')
set(gca,'FontSize',20)

%% Errore di inversione cinematica

close all
f = figure();
f.OuterPosition = [100 50 800 800];
subplot(3,1,1)
plot(t,e_obs(:,1),'LineWidth',4)
title('Errore inversione cinematica con funzionale')
xlabel('time [s]')
ylabel('x [m]')
set(gca,'FontSize',20)
subplot(3,1,2)
plot(t,e_obs(:,2),'LineWidth',4)
xlabel('time [s]')
ylabel('y [m]')
set(gca,'FontSize',20)
subplot(3,1,3)
plot(t,e_obs(:,3),'LineWidth',4)
xlabel('time [s]')
ylabel('z [m]')
set(gca,'FontSize',20)

