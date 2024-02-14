import numpy as np #pacchetto per il calcolo scientifico
import matplotlib.pyplot as plt #libreria per la gestione di grafici 2D, importo il modulo di plotting


def DH_computation(d, a, alpha, theta): # posizione e orientamento relativi di due bracci consecutivi, serve per calcolare la cinematica diretta 
    T = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                 [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                 [0, np.sin(alpha), np.cos(alpha), d],
                 [0, 0, 0, 1]])
    return T

def DirectKinematics(T):#uso il quaternione unitario per ricavare la cinematica diretta (posizione e orientamento organo terminale)

    # Estraggo dalla trasformazione omogenea il vettore traslazione
    x = T[0:3,3]

    # Estraggo dalla trasformazione omogenea il quaternione unitario
    sgn_1 = np.sign(T[2,1]-T[1,2])
    sgn_2 = np.sign(T[0,2]-T[2,0])
    sgn_3 = np.sign(T[1,0]-T[0,1])

    eta = np.sqrt(T[0,0]+T[1,1]+T[2,2]+1)/2
    eps = 0.5*np.array([sgn_1*np.sqrt(T[0,0]-T[1,1]-T[2,2]+1),
                        sgn_2*np.sqrt(-T[0,0]+T[1,1]-T[2,2]+1),
                        sgn_3*np.sqrt(-T[0,0]-T[1,1]+T[2,2]+1)])

    Q = np.hstack([eta, eps]) #creo un vettore riga

    return x, Q

def planner_circonferenza(p_i, p_f, ti, tf, t, theta):
    
    s = len(p_i)
    C = (p_f-p_i)/2 + p_i #sclego un centro che dipende dalle posizioni che si trova in ingresso la funzione
    
    r=np.linalg.norm(p_i-C)
    R=np.array([[1,0,0],[0,1,0],[0, 0, 1]]) #ho calcolato a mano la matrice di rotazione tenendo conto del centro e del punto iniziale 

    if t<ti:
        pd = p_i
        pdot = np.zeros(s) #creo una matrice di zeri delle dimensioni di s

    elif t<=tf: #uso un polinomio di quinto grado perche potrei imporre anche condizioni sulla continuità della accelerazione 
        A=np.array([[ti**5, ti**4, ti**3, ti**2, ti, 1],
                   [tf**5, tf**4, tf**3, tf**2, tf, 1],
                   [5*ti**4, 4*ti**3, 3*ti**2, 2*ti, 1, 0],
                   [5*tf**4, 4*tf**3, 3*tf**2, 2*tf, 1, 0],
                   [20*ti**3, 12*ti**2, 6*ti, 2, 0, 0],
                   [20*tf**3, 12*tf**2, 6*tf, 2, 0, 0]])
        

        b = np.array([[0], [theta*r], [0], [0], [0], [0]])

        x = np.linalg.solve(A, b)#risolviamo il numero di equazioni rispetto a b, il primo argomento A è la matrice dei coefficienti del sistema,
        # mentre il secondo argomento b è il vettore dei termini noti.
       
        #scrivo l'ascissa curvilinea con i coefficienti trovati sopra 
        
        s = x[0]*t**5 + x[1]*t**4 + x[2]*t**3 + x[3]*t**2 + x[4]*t + x[5]
        s_dot = 5*x[0]*t**4 + 4*x[1]*t**3 + 3*x[2]*t**2 + 2*x[3]*t + x[4]
        #s_dot_dot = 20*x[0]*t**3 + 12*x[1]*t**2 + 6*x[2]*t + 2*x[3]

        #rappresentazione parametrica della circonferenza nel nuovo sistema di riferimento
        
        p_primo=np.array([r*np.cos(s/r), r*np.sin(s/r), 0.0])
        p_secondo=np.array([-s_dot*np.sin(s/r), s_dot*np.cos(s/r), 0.0])
        #p_terzo = np.array([-s_dot_dot*np.cos(s/r), -s_dot_dot*np.sin(s/r), 0.0])

        # trovo traiettoria, velocità e accelerazione 
           
        pd= C + np.dot(R, p_primo) # R * w, prodotto di due vettori
        pdot = np.dot(R, p_secondo)
        #pdot_dot = np.dot(R, p_terzo)
    
    else:
        pd = p_f #pd corrisponde al mio punto finale quando finisce t>tf
        pdot = np.zeros(s)
        #pdot_dot = np.zeros(s)

    return pd, pdot


    

def Jacobian(q,a,d,alpha): #jacobiano geometrico che coincide con quello analitico perche ho un compito di puro posizionamento

    T01 = DH_computation(d[0], a[0], alpha[0], q[0])
    T12 = DH_computation(d[1], a[1], alpha[1], q[1])
    T23 = DH_computation(d[2], a[2], alpha[2], q[2])
    T34 = DH_computation(d[3], a[3], alpha[3], q[3])


    T02 = np.matmul(T01,T12)
    T03 = np.matmul(T02,T23)
    T04 = np.matmul(T03,T34)
    
    # estraggo i vettori zi-1, sono vettori riga tutti 
    z0 = np.array([0,0,1])
    z1 = T01[0:3,2]
    z2 = T02[0:3,2]
    z3 = T03[0:3,2]

    # Estraggo i vettori p_i-1, vettori riga 
    p0 = np.array([0,0,0])
    p1 = T01[0:3,3]
    p2 = T02[0:3,3]
    p3 = T03[0:3,3]
    p4 = T04[0:3,3]

    # Considero solo la parte di posizionamento dello Jacobiano 
    J1 = np.cross(z0,p4-p0) #faccio un prodotto vettoriale 
    J2 = np.cross(z1,p4-p1)
    J3 = np.cross(z2,p4-p2)
    J4 = np.cross(z3,p4-p3)

    Jac = np.vstack((J1, J2, J3, J4))# 4x3 
    # metto tutto in un'unica matrice (J1 nella prima riga, J2 nella seconda...)

    return Jac.T
    #mi serve calcolare solo Jp

def PseudoInvDestra(J):#definisco la Jpinv che mi servira quando ho un manipolatore 
    #ridondante con m<n, la userò anche per il proiettore nel nullo.
    Jt=J.T
    Jprod=np.matmul(J,Jt)
    Jinversa=np.linalg.pinv(Jprod)
    Jpinv = np.matmul(Jt,Jinversa)
    return Jpinv  

#ho trovato questa funzione atrraverso matlab scrivendo il funzionale e calcolando le derivate 
def W_q_obstacle(o_x,o_y,o_z,q1,q2,q3):
    t2 = np.cos(q1)
    t3 = np.cos(q2)
    t4 = np.cos(q3)
    t5 = np.sin(q1)
    t6 = np.sin(q2)
    t7 = np.sin(q3)
    t8 = q1+q2
    t11 = -q2
    t21 = o_z*8.112963841460668e+33
    t35 = o_x*6.582018229284824e+65
    t36 = o_y*6.582018229284824e+65
    t9 = np.cos(t8)
    t10 = np.sin(t8)
    t12 = q1+t11
    t15 = t7*2.483878800010756e+16
    t18 = t6*9.735556609752802e+32
    t19 = t2*t3*2.467861557257148e+31
    t20 = t3*t5*2.467861557257148e+31
    t22 = t4*t6*4.056481920730334e+32
    t25 = -t21
    t28 = t2*t6*4.030323778211595e+47
    t29 = t5*t6*4.030323778211595e+47
    t30 = t2*t6*4.836388533853914e+48
    t32 = t5*t6*4.836388533853914e+48
    t33 = t2*6.582018229284824e+63
    t34 = t5*6.582018229284824e+63
    t37 = t2*5.265614583427859e+64
    t38 = t5*5.265614583427859e+64
    t39 = -t36
    t41 = t2*t3*7.898421875141789e+64
    t42 = t3*t5*7.898421875141789e+64
    t13 = np.cos(t12)
    t14 = np.sin(t12)
    t16 = -t15
    t17 = t3*t15
    t23 = t9*8.112963841460669e+31
    t24 = t10*8.112963841460669e+31
    t31 = -t29
    t40 = -t37
    t43 = -t41
    t49 = t20+t28+t34
    t26 = t13*8.112963841460668e+31
    t27 = t14*8.112963841460668e+31
    t50 = t19+t31+t33
    t51 = t7*t49*5.0
    t53 = t16+t17+t18+t22+t25+5.273426496949434e+32
    t44 = t23+t26
    t45 = t24+t27
    t52 = t7*t50*5.0
    t54 = t53**2
    t46 = t4*t45*2.028240960365167e+32
    t47 = t4*t44*2.028240960365167e+32
    t55 = t54*6.582018229284824e+63
    t48 = -t47
    t56 = t30+t38+t39+t42+t46+t52
    t57 = t32+t35+t40+t43+t48+t51
    t58 = t56**2
    t59 = t57**2
    t60 = t55+t58+t59
    t61 = 1.0/np.sqrt(t60)
    et1 = o_x*t5*6.931674235302037e+130
    et2 = o_y*t2*-6.931674235302037e+130
    et3 = o_x*t2*t6*6.366639498746113e+114
    et4 = o_x*t3*t5*1.039751135295306e+131
    et5 = o_x*t2*t7*4.332296397063773e+130
    et6 = o_y*t2*t3*-1.039751135295306e+131
    et7 = o_y*t5*t6*6.366639498746113e+114
    et8 = o_y*t5*t7*4.332296397063773e+130
    et9 = o_x*t2*t3*t7*1.624350975721778e+98
    et10 = o_x*t2*t4*t6*2.65276645781088e+114
    et11 = o_x*t3*t4*t5*4.332296397063773e+130
    et12 = o_x*t5*t6*t7*-2.65276645781088e+114
    et13 = o_y*t2*t3*t4*-4.332296397063773e+130
    et14 = o_y*t2*t6*t7*2.65276645781088e+114
    et15 = o_y*t3*t5*t7*1.624350975721778e+98
    et16 = o_y*t4*t5*t6*2.65276645781088e+114
    et17 = t7*(t2*t6*2.467861557257148e+31+t3*t5*4.030323778211595e+47)*5.0-t2*t3*4.836388533853914e+48
    et18 = t5*t6*7.898421875141789e+64-t4*(t23-t26)*2.028240960365167e+32
    et19 = t7*(t2*t3*4.030323778211595e+47-t5*t6*2.467861557257148e+31)*5.0+t2*t6*7.898421875141789e+64
    et20 = t3*t5*4.836388533853914e+48+t4*(t24-t27)*2.028240960365167e+32
    et21 = t61*(t56*(et17+et18)*-2.0+t57*(et19+et20)*2.0+t53*(t3*9.735556609752802e+32+t3*t4*4.056481920730334e+32-t6*t7*2.483878800010756e+16)*1.316403645856965e+64)
    et22 = 7.596454196607839e-67
    et23 = t53*(t4*2.483878800010756e+16-t3*t4*2.483878800010756e+16+t6*t7*4.056481920730334e+32)*1.316403645856965e+64-t57*(t7*t44*2.028240960365167e+32+t4*t49*5.0)*2.0
    et24 = t56*(t7*t45*2.028240960365167e+32-t4*t50*5.0)*2.0
    W_q = [(t61*(et1+et2+et3+et4+et5+et6+et7+et8+et9+et10+et11+et12+et13+et14+et15+et16))/1.316403645856965e+66,et21*et22,t61*(et23+et24)*(-7.596454196607839e-67),0.0]


    return np.array(W_q) #creo un array altrimenti ho problemi di dimensioni 

def InverseKinematicsObstacle(q,a,d,alpha,pd,pdot, kgain, algorithm, ka,o):
    J = Jacobian(q,a,d,alpha)
    Jpinv = PseudoInvDestra(J)
    # Calcolo della posa attuale
    T01 = DH_computation(d[0], a[0], alpha[0], q[0])
    T12 = DH_computation(d[1], a[1], alpha[1], q[1])
    T23 = DH_computation(d[2], a[2], alpha[2], q[2])
    T34 = DH_computation(d[3], a[3], alpha[3], q[3])
   

    T02 = np.matmul(T01,T12)
    T03 = np.matmul(T02,T23)
    T04 = np.matmul(T03,T34)
    P,_ = DirectKinematics(T04) # # DirectKinematic resistituisce P e Q ma a noi interessa solo P quindi al posto di Q mettiamo _

    #errore sull'organo terminale
    err = pd - P
    #definisco i guadagni 
    K = kgain*np.eye(3) #guadagno inversione senza funzionale 
    Err = np.matmul(K,err)
    K_0 = ka*np.eye(4) #guadagno inversione con funzionale 
    
    
    #posizione ostacolo
    o_x=o[0]
    o_y=o[1]
    o_z=o[2]
    dw = W_q_obstacle(o_x,o_y,o_z,q[0],q[1],q[3])

    if (algorithm == "t"): #se l'algoritmo è trasposta uso qddot = J.T x K x e
       
        Qdot = np.matmul(J.T,Err)
    else: #se l'algoritmo è con l'inversa uso il proiettore nel nullo per riconfigurare il manipolatore e fargli evitare l'ostacolo
        
        #Qddot = pinv(J)*(XYddot'+ K*e) + (eye(4)-(pinv(J)*J))*qa';

        dw = W_q_obstacle(o_x,o_y,o_z,q[0],q[1],q[3])
        eyeJ=(np.eye(4)- np.matmul(Jpinv,J))
        Qdot1=np.matmul(eyeJ,K_0)
        Er = np.matmul(K,err)
        Err = Er + pdot.T
        Q1D = np.matmul(Jpinv,Err) 
        Q2D=np.matmul(Qdot1,dw.T)
        Qdot = Q1D+Q2D     
         
    return Qdot, err

def InverseKinematics(q,a,d,alpha,pd,pdot, kgain, algorithm):
    # Calcolo dello Jacobiano geometrico
    J = Jacobian(q,a,d,alpha)
    Jpinv = PseudoInvDestra(J)
    # Calcolo della posa attuale
    T01 = DH_computation(d[0], a[0], alpha[0], q[0])
    T12 = DH_computation(d[1], a[1], alpha[1], q[1])
    T23 = DH_computation(d[2], a[2], alpha[2], q[2])
    T34 = DH_computation(d[3], a[3], alpha[3], q[3])

    T02 = np.matmul(T01,T12)
    T03 = np.matmul(T02,T23)
    T04 = np.matmul(T03,T34)

    P, _ = DirectKinematics(T04)
    
    # Calcolo dell'errore
    err = pd - P

    # Calcolo delle velocità desiderate SG
    K = kgain*np.eye(3) # K è diagonale e definita positiva, in ingresso dò i suoi autovalori k
    Err = np.matmul(K,err)

    if algorithm == "t":
        # Inversione con la trasposta
        Qdot = np.matmul(J.T,Err)
    else:
       # Inversione con l'inversa,scelgo il caso di assenza di singolarità ma ridondanza
        Err_ = pdot+ Err
        Qdot = np.matmul(Jpinv,Err_) 

    return Qdot, err


if __name__ == '__main__':

    
    #definisco i parametri del mio robot 
    a1 = 0.08 #m
    a2 = 0.12 #m
    a3 = 0.05 #m
    a4 = 0.02  #m
    d1 = 0.065 #m
    d4 = 0.065  #m
    lmax = a1+a2+a3+a4
    a = np.array([0.08, 0.12, 0.05, 0.02])
    d = np.array([0.065, 0, 0, 0.065])
    alpha = np.array([np.pi/2, -np.pi/2, np.pi/2,0])

    #definisco i tempi percorsi
    tA = 0 #s 
    tB = 5 #s
    tC = 10 #s
    dt = 0.01 #s

    #creo i vettori dei tempi 
    tAB = np.arange(tA,tB,dt)
    tBC = np.arange(tB,tC,dt)

     #concateno i vettori dei tempi 
    time = np.concatenate((tAB,tBC), axis=0)

    #definisco la configurazione iniziale del mio manipolatore
    q0 = np.array([np.pi/9,np.pi/9,np.pi/4,np.pi/3]) 

    T01 = DH_computation(d[0], a[0], alpha[0], q0[0])
    T12 = DH_computation(d[1], a[1], alpha[1], q0[1])
    T23 = DH_computation(d[2], a[2], alpha[2], q0[2])
    T34 = DH_computation(d[3], a[3], alpha[3], q0[3])

    T02 = np.matmul(T01,T12)
    T03 = np.matmul(T02,T23)
    T04 = np.matmul(T03,T34)

    pi, _ = DirectKinematics(T04)
    print(pi)

    

    
    #Definisco i punti A,B e C della traiettoria

    XY_A = pi
    XY_B =np.array([0.20, 0.08, 0.1525])
    XY_C = np.array([0.16, 0.078,0.1525])

    #definisco gli angoli delle due cironferenze
    theta1 = np.pi
    theta2 = np.pi*5/4

    
    # Inizializzo posizione e velocità nello spazio dei giunti, le chiamo con la Q
    Qd = np.zeros((len(time)+1,4)) #metto 4 poiché i giunti sono 4
    Qdot = np.zeros((len(time)+1,4))
    Qd[0,:] = q0
    Qdot[0,:] = np.array([0,0,0,0])
 
    Qd1=np.zeros((len(time)+1, 4))
    Qdot1=np.zeros((len(time)+1, 4))
    Qd1[0,:]=q0
    Qdot1[0,:]=np.array([0,0,0,0])

    Pd = np.zeros((len(time)+1,3))
    Pd[0,:] = pi 
    Pdot = np.zeros((len(time)+1,3))
    Pdot[0,:] = np.array([0,0,0])

    P_1 = np.zeros((len(time)+1,3))
    P_1[0,:],_ = DirectKinematics(T01)
    
    P_2 = np.zeros((len(time)+1,3))
    P_2[0,:],_ = DirectKinematics(T02)
    
    P_3 = np.zeros((len(time)+1,3))
    P_3[0,:],_ = DirectKinematics(T03)
    
    P_ee = np.zeros((len(time)+1,3))
    P_ee[0,:],_ = DirectKinematics(T04)

    P_1_fun = np.zeros((len(time)+1,3))
    P_1_fun[0,:],_ = DirectKinematics(T01)
    
    P_2_fun = np.zeros((len(time)+1,3))
    P_2_fun[0,:],_ = DirectKinematics(T02)
    
    P_3_fun = np.zeros((len(time)+1,3))
    P_3_fun[0,:],_ = DirectKinematics(T03)
    
    P_ee_fun = np.zeros((len(time)+1,3))
    P_ee_fun[0,:],_ = DirectKinematics(T04)

    errore=np.zeros((len(time)+1, 3))
    errore[0,:]=np.array([0,0,0])

    errore_fun=np.zeros((len(time)+1, 3))
    errore_fun[0,:]=np.array([0,0,0])
    
    

    # Pianificazione della traiettoria OFF-LINE
    
     #da A a B
    counter = 0
    PdAB = np.zeros((len(tAB)+1,3))
    PdotAB = np.zeros((len(tAB)+1,3))
    PdAB[0,:] = pi
    PdAB[len(tAB),:] = XY_B
    for t in tAB:
        pdAB,pdotAB=planner_circonferenza(pi, XY_B, tA, tB, t, theta1)
        PdAB[counter,:] = pdAB
        PdotAB[counter,:] = pdotAB
        counter += 1
    #da B a C
    counter = 0
    PdBC = np.zeros((len(tBC)+1,3))
    PdotBC = np.zeros((len(tBC)+1,3))
    PdBC[0,:] = XY_B
    PdBC[len(tBC),:] = XY_C
    for t in tBC:
        pdBC,pdotBC= planner_circonferenza(XY_B, XY_C, tB, tC, t, theta2)
        PdBC[counter,:] = pdBC
        PdotBC[counter,:] = pdotBC
        counter += 1
    
    Pd = np.concatenate((PdAB, PdBC), axis = 0) #vettore riga
    Pdot = np.concatenate((PdotAB, PdotBC), axis = 0)
    
    
           #circonferenza AB 
    plt.plot(tAB, PdAB[0:len(tAB),0], lw = 3, color = 'purple', label = 'desired x')
    plt.plot(tAB, PdAB[0:len(tAB),1], lw = 3, color = 'green', label = 'desired y')
    plt.plot(tAB, PdAB[0:len(tAB),2], lw = 3, color = 'yellow',label = 'desired z')

        # circonferenza BC
    plt.plot(tBC, PdBC[0:len(tBC),0], lw = 3, color = 'purple')
    plt.plot(tBC, PdBC[0:len(tBC),1], lw = 3, color = 'green')
    plt.plot(tBC, PdBC[0:len(tBC),2], lw = 3, color = 'yellow')
        
        #metto i marker
    plt.plot(tA,PdAB[0,0],marker='.',markersize=1, color = 'black')
    plt.plot(tA,PdAB[0,1],marker='.',markersize=1, color = 'black')
    plt.plot(tA,PdAB[0,2],marker='.',markersize=1, color = 'black')

    plt.plot(tB,PdAB[len(tAB),0],marker='.',markersize=5, color = 'black')
    plt.plot(tB,PdAB[len(tAB),1],marker='.',markersize=5, color = 'black')
    plt.plot(tB,PdAB[len(tAB),2],marker='.',markersize=5, color = 'black')

    plt.plot(tC,PdBC[len(tBC),0],marker='.',markersize=5, color = 'black')
    plt.plot(tC,PdBC[len(tBC),1],marker='.',markersize=5, color = 'black')
    plt.plot(tC,PdBC[len(tBC),2],marker='.',markersize=5, color = 'black')

    plt.xlim([tA, tC])
    plt.legend()
    plt.ylabel('End-effector [m]',fontsize=12)
    plt.xlabel('Time [s]',fontsize=12)

    fig = plt.subplots()

    #faccio plot in 3D con funzioni trovate online, mettendo anche i marker ad inizio e fine traiettoria 

    ax = plt.axes(projection = '3d')
    plt.plot(PdAB[0:len(tAB),0], PdAB[0:len(tAB),1], PdAB[0:len(tAB),2], lw = 4, color='green')
    plt.plot(PdAB[0,0],PdAB[0,1],PdAB[0,2],marker='.',markersize=5, color = 'black')
    plt.plot(PdAB[len(tAB),0],PdAB[len(tAB),1],PdAB[len(tAB),2],marker='.',markersize=5, color = 'black')
    plt.plot(PdBC[0:len(tBC),0], PdBC[0:len(tBC),1], PdBC[0:len(tBC),2], lw = 4, color='blue')
    plt.plot(PdBC[0,0],PdBC[0,1],PdBC[0,2],marker='.',markersize=5, color = 'black')
    plt.plot(PdBC[len(tBC),0],PdBC[len(tBC),1],PdBC[len(tBC),2],marker='.',markersize=5, color = 'black')
    


    ax.set_xlabel('x [m]', fontsize = 10)
    ax.set_ylabel('y [m]', fontsize = 10)
    ax.set_zlabel('z [m]', fontsize = 10)
    ax.set_xlim([0.05, 0.3])
    ax.set_ylim([-0.4, 0.3])
    ax.set_zlim([0, 0.4])
    ax.set_aspect('auto','box')

    plt.show()


#inversione cinematica con il funzionale (distanza ostacolo)

    W = np.zeros(len(time)+1)
    W_obs = np.zeros(len(time)+1)

    kgain = 50
    ka = 30
    
    #definisco posizione ostacolo presa sulla traiettoria del giunto 4
    O = np.array([0.14679, 0.153707, 0.162039])
    
    Ox = O[0]
    Oy = O[1]
    Oz = O[2]

    counter = 1 # posso partire da counter 1 perchè Pd in indice 0 ho già la posizione iniziale
    for t in time:
        if t <= tB :
             # Pianifiazione prima circonferenza (da A a B)
           Pd[counter,:], Pdot[counter,:] = planner_circonferenza(pi, XY_B, tA, tB, t, theta1)
        else:
            # Pianifiazione seconda circonferenza (da B a C)
           Pd[counter,:], Pdot[counter,:] = planner_circonferenza(XY_B, XY_C, tB, tC, t, theta2) 
        
        pd = Pd[counter,:] #traiettoria pianificata
        pdot = Pdot[counter,:]
        
        ## Calcolo delle velocità attuali nello SG desiderate in uscita dall'algoritmo di inversione
        qdot, err = InverseKinematics(Qd[counter-1,:], a, d, alpha, pd, pdot, kgain, "i")  #InvKin(q, a, d, pos_d, vel_d, k, algorithm):
        qdot1, err_fun = InverseKinematicsObstacle(Qd1[counter-1,:], a, d, alpha, pd, pdot, kgain, "i", ka, O)  # InvKin_distanza_finecorsa(q, a, d, pos_d, vel_d, k, algorithm, ka, q_min, q_max):

        # Ricavo la posizione nello spazio dei giunti per integrazione numerica
        # Senza funzionale di costo
        Qd[counter,:] = Qd[counter-1,:] + dt*qdot
        Qdot[counter,:] = qdot
        errore[counter,:] = err

         # Con funzionale di costo
        Qd1[counter,:]= Qd1[counter-1,:]+dt*qdot1
        Qdot1[counter,:] = qdot1
        errore_fun[counter,:] = err_fun

        # Calcolo cinematica della simulazione senza funzionale
        T01 = DH_computation(d[0], a[0], alpha[0], Qd[counter,0]) 
        T12 = DH_computation(d[1], a[1], alpha[1], Qd[counter,1])
        T23 = DH_computation(d[2], a[2], alpha[2], Qd[counter,2])
        T34 = DH_computation(d[3], a[3], alpha[3], Qd[counter,3])

        T02 = np.matmul(T01,T12)  
        T03 = np.matmul(T02,T23)
        T04 = np.matmul(T03,T34)

        # Posizione dell'end effector
        P_1[counter,:],_ = DirectKinematics(T01)
        P_2[counter,:],_ = DirectKinematics(T02)
        P_3[counter,:],_ = DirectKinematics(T03)
        P_ee[counter,:],_ = DirectKinematics(T04)

         # Calcolo cinematica della simulazione con funzionale
        T01 = DH_computation(d[0], a[0], alpha[0], Qd1[counter,0]) 
        T12 = DH_computation(d[1], a[1], alpha[1], Qd1[counter,1])
        T23 = DH_computation(d[2], a[2], alpha[2], Qd1[counter,2])
        T34 = DH_computation(d[3], a[3], alpha[3], Qd1[counter,3])

        T02 = np.matmul(T01,T12)  
        T03 = np.matmul(T02,T23)
        T04 = np.matmul(T03,T34)

        # Posizione dell'end effector
        P_1_fun[counter,:],_ = DirectKinematics(T01)
        P_2_fun[counter,:],_ = DirectKinematics(T02)
        P_3_fun[counter,:],_ = DirectKinematics(T03)
        P_ee_fun[counter,:],_ = DirectKinematics(T04)


        counter += 1
        
    np.savetxt('Qd.txt', Qd)
    np.savetxt('Qd_fun.txt', Qd1)
    np.savetxt('Qdot.txt',Qdot)
    np.savetxt('Qdot_fun.txt',Qdot1)

    
   

     ## Plot movimento manipolatore
    fig = plt.figure(figsize=((15,15)))
    # plot del tempo e della posizione dell'end-effector desiderata e di quella reale lungo x
    plt.plot(time, Pd[0:len(time),0], lw = 5, label = 'desired')
    plt.plot(time, P_ee_fun[0:len(time),0], lw = 2, label = 'real')
    plt.legend()
    plt.ylim([0, 0.7])
    plt.ylabel('x[m]',fontsize=10)
    plt.xlabel('t[s]',fontsize=10)   
    plt.title('traiettoria asse x') 
        
    # plot della poszione dell'end-effector  desiderata e reale lungo y
    fig = plt.figure(figsize=((15,15)))
    plt.plot(time, Pd[0:len(time),1], lw = 5, label = 'desired')
    plt.plot(time, P_ee_fun[0:len(time),1], lw = 2, label = 'real')
    plt.ylim([-0.8, 0.7])
    plt.ylabel('y[m]',fontsize=10)
    plt.xlabel('t[s]',fontsize=10)
    plt.legend()
    plt.title('traiettoria asse y') 

    # plot della posizione dell'end-effector desiderata e reale lungo z
    fig = plt.figure(figsize=((15,15)))
    plt.plot(time, Pd[0:len(time),2], lw = 5, label = 'desired')
    plt.plot(time, P_ee_fun[0:len(time),2], lw = 2, label = 'real')
    plt.ylim([-0.5, 0.7])
    plt.ylabel('z[m]',fontsize=10)
    plt.xlabel('t[s]',fontsize=10)
    plt.title('traiettoria asse z') 
    plt.legend()
    plt.show()
   
    
    
    # PLOT 3D - MOVIMENTO ROBOT CON FUNZIONALE, evitamento ostacolo
    viewer = plt.subplot(3,3,(2,9),projection="3d")
    fig.show()
    valori = 5

    pos_A= pi
    pos_B= XY_B
    pos_C= XY_C
    for i in range(0, counter, valori):
        viewer.clear()

        plt.plot(pos_A[0], pos_A[1], pos_A[2], marker = '.', markersize = 7, color = 'black', label = "p1")
        plt.plot(pos_B[0], pos_B[1], pos_B[2], marker = '.', markersize = 7, color = 'black', label = "p2")
        plt.plot(pos_C[0], pos_C[1], pos_C[2], marker = '.', markersize = 7, color = 'black', label = "p3")
        
        plt.plot(O[0],O[1],O[2], marker='.',markersize=12, color = 'black')


        plt.plot([0,P_1_fun[i,0]],[0,P_1_fun[i,1]], [0,P_1_fun[i,2]], lw=5, color = 'blue')
        plt.plot([0],[0],[0], marker='.',markersize=12, color = 'black')

        plt.plot([P_1_fun[i,0],P_2_fun[i,0]],[P_1_fun[i,1],P_2_fun[i,1]],[P_1_fun[i,2],P_2_fun[i,2]], lw=5, color = 'orange')
        plt.plot(P_1_fun[i,0],P_1_fun[i,1],P_1_fun[i,2], marker='.',markersize=12, color = 'black')

        plt.plot([P_2_fun[i,0],P_3_fun[i,0]],[P_2_fun[i,1],P_3_fun[i,1]],[P_2_fun[i,2],P_3_fun[i,2]], lw=5, color = 'green')
        plt.plot(P_2_fun[i,0],P_2_fun[i,1],P_2_fun[i,2], marker='.',markersize=12, color = 'black')

        plt.plot([P_3_fun[i,0],P_ee_fun[i,0]],[P_3_fun[i,1],P_ee_fun[i,1]],[P_3_fun[i,2],P_ee_fun[i,2]],lw=5, color = 'red')
        plt.plot(P_3_fun[i,0],P_3_fun[i,1],P_3_fun[i,2], marker='.',markersize=12, color = 'black')

        plt.plot(Pd[:,0],Pd[:,1],Pd[:,2],  lw = 3, c='blue', label = 'desired', linestyle='--')
        plt.plot(P_ee_fun[0:i,0],P_ee_fun[0:i,1],P_ee_fun[0:i,2], lw = 2, c='red', label = "real")
        plt.title('inseguimento di traiettoria con funzionale')

        viewer.set_xlabel('x[m]',fontsize=18)
        viewer.set_ylabel('y[m]',fontsize=18)
        viewer.set_zlabel('z[m]',fontsize=18)

       
        viewer.set_xlim([0, 0.4])
        viewer.set_ylim([-0.2, 0.3])
        viewer.set_zlim([-0.5, 0.3])

        viewer.set_aspect('auto', 'box')
        plt.pause(0.01) #.1
        fig.canvas.draw()
    
# PLOT 3D - MOVIMENTO ROBOT SENZA FUNZIONALE, prende l'ostacolo
    viewer = plt.subplot(projection="3d")
    fig.show()
    valori = 5
    for i in range(0, counter, valori):
        viewer.clear()

        plt.plot(pos_A[0], pos_A[1], pos_A[2], marker = '.', markersize = 7, color = 'black', label = "p1")
        plt.plot(pos_B[0], pos_B[1], pos_B[2], marker = '.', markersize = 7, color = 'black', label = "p2")
        plt.plot(pos_C[0], pos_C[1], pos_C[2], marker = '.', markersize = 7, color = 'black', label = "p3")
        
        plt.plot(O[0],O[1],O[2], marker='.',markersize=12, color = 'black')

        plt.plot([0,P_1[i,0]],[0,P_1[i,1]], [0,P_1[i,2]], lw=3, color = 'blue')
        plt.plot([0],[0],[0], marker='.',markersize=12, color = 'black')

        plt.plot([P_1[i,0],P_2[i,0]],[P_1[i,1],P_2[i,1]],[P_1[i,2],P_2[i,2]], lw=3, color = 'orange')
        plt.plot(P_1[i,0],P_1[i,1],P_1[i,2], marker='.',markersize=12, color = 'black')

        plt.plot([P_2[i,0],P_3[i,0]],[P_2[i,1],P_3[i,1]],[P_2[i,2],P_3[i,2]], lw=3, color = 'green')
        plt.plot(P_2[i,0],P_2[i,1],P_2[i,2], marker='.',markersize=12, color = 'black')

        plt.plot([P_3[i,0],P_ee[i,0]],[P_3[i,1],P_ee[i,1]],[P_3[i,2],P_ee[i,2]],lw=3, color = 'red')
        plt.plot(P_3[i,0],P_3[i,1],P_3[i,2], marker='.',markersize=12, color = 'black')

        plt.plot(Pd[:,0],Pd[:,1],Pd[:,2],  lw = 3, c='blue', label = 'desired', linestyle='--')
        plt.plot(P_ee[0:i,0],P_ee[0:i,1],P_ee[0:i,2], lw = 3, c='red', label = "real")
        plt.title('inseguimento di traiettoria senza funzionale')

        viewer.set_xlabel('x[m]',fontsize=18)
        viewer.set_ylabel('y[m]',fontsize=18)
        viewer.set_zlabel('z[m]',fontsize=18)

        #plt.legend(loc = "best")
        viewer.set_xlim([0, 0.4])
        viewer.set_ylim([-0.5, 0.4])
        viewer.set_zlim([0, 0.4])

        viewer.set_aspect('auto', 'box')
        plt.pause(0.01) #.1
        fig.canvas.draw()
    

 
    
    
   
