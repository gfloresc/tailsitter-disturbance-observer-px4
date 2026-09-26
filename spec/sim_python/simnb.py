import numpy as np, sys, math
from numba import njit
g=9.81; m=1.0
kp, kv = 4.0, 4.0
kq, kO = 1600.0, 60.0
k1, l1 = 300.0, 22500.0
k2, l2 = 40.0, 400.0
psid=0.0

@njit(cache=True)
def qmul(a,b):
    return np.array([a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
                     a[0]*b[1]+b[0]*a[1]+a[2]*b[3]-a[3]*b[2],
                     a[0]*b[2]+b[0]*a[2]+a[3]*b[1]-a[1]*b[3],
                     a[0]*b[3]+b[0]*a[3]+a[1]*b[2]-a[2]*b[1]])
@njit(cache=True)
def Re3(q):
    w,x,y,z=q[0],q[1],q[2],q[3]
    return np.array([2*(x*z+w*y),2*(y*z-w*x),w*w-x*x-y*y+z*z])
@njit(cache=True)
def sq(t,amp):
    for n in (5.0,10.0,15.0):
        if t>n and t<n+1.5: return amp
    return 0.0
@njit(cache=True)
def dist(t):
    F=np.zeros(3); T=np.zeros(3)
    F[0]=sq(t,5.0)
    if t>5: F[1]=2*math.sin(t)
    if t>=5: F[2]=t/3-math.floor(t/3)
    T[0]=sq(t,100.0)
    if t>7: T[1]=100*math.sin(t)
    if t>=7: T[2]=100*(t/2-math.floor(t/2))
    return F,T
@njit(cache=True)
def control(x,comp,psid,kp,kv,kq,kO):
    c=1.0 if comp else 0.0
    mu=np.empty(3)
    for i in range(3): mu[i]=m*(kp*x[i]+kv*x[3+i]+c*x[16+i])
    mu[2]+=m*g
    f=math.sqrt(mu[0]**2+mu[1]**2+mu[2]**2)
    cp,sp=math.cos(psid),math.sin(psid)
    z=np.array([(cp*mu[0]+sp*mu[1])/f,(-sp*mu[0]+cp*mu[1])/f,mu[2]/f])
    s=math.sqrt(2*(1+z[2]))
    Qr=np.array([(1+z[2])/s,-z[1]/s,z[0]/s,0.0])
    Qa=np.array([math.cos(psid/2),0.0,0.0,math.sin(psid/2)])
    Qd=qmul(Qa,Qr)
    Qdc=np.array([Qd[0],-Qd[1],-Qd[2],-Qd[3]])
    Qb=qmul(Qdc,x[6:10])
    if Qb[0]<0: Qb=-Qb
    Om=x[10:13]
    tau=np.empty(3)
    for i in range(3): tau[i]=-c*x[22+i]-kq*Qb[1+i]-kO*Om[i]   # J = I: Om x J Om = 0
    return f,tau,Qd
@njit(cache=True)
def deriv(t,x,comp,psid,kp,kv,kq,kO,k1,l1,k2,l2):
    F,T=dist(t)
    f,tau,Qd=control(x,comp,psid,kp,kv,kq,kO)
    r=Re3(x[6:10])
    dx=np.empty(25)
    for i in range(3):
        phiv=-(f/m)*r[i]+(g if i==2 else 0.0)
        dx[i]=x[3+i]; dx[3+i]=phiv+F[i]
        dx[10+i]=tau[i]+T[i]            # J = I
        ev=x[13+i]-x[3+i]; eO=x[19+i]-x[10+i]
        dx[13+i]=phiv+x[16+i]-k2*ev; dx[16+i]=-l2*ev
        dx[19+i]=tau[i]+x[22+i]-k1*eO; dx[22+i]=-l1*eO
    om=np.array([0.0,x[10],x[11],x[12]])
    dq=0.5*qmul(x[6:10],om)
    for i in range(4): dx[6+i]=dq[i]
    return dx
@njit(cache=True)
def run(comp,dt,tf,dec,psid,kp,kv,kq,kO,k1,l1,k2,l2):
    x=np.zeros(25)
    x[0]=5.0; x[1]=-2.0; x[2]=1.0
    q0=np.array([0.9435,0.3145,0.1048,0.0]); q0/=math.sqrt((q0**2).sum())
    x[6:10]=q0; x[19]=5.0; x[20]=-10.0
    N=int(round(tf/dt)); M=N//dec+1
    out=np.zeros((M,1+25+1+3+4+6)); j=0
    for k in range(N+1):
        t=k*dt
        if k%dec==0:
            f,tau,Qd=control(x,comp,psid,kp,kv,kq,kO); F,T=dist(t)
            out[j,0]=t; out[j,1:26]=x; out[j,26]=f; out[j,27:30]=tau; out[j,30:34]=Qd; out[j,34:37]=F; out[j,37:40]=T; j+=1
        if k==N: break
        a=deriv(t,x,comp,psid,kp,kv,kq,kO,k1,l1,k2,l2)
        b=deriv(t+dt/2,x+dt/2*a,comp,psid,kp,kv,kq,kO,k1,l1,k2,l2)
        c=deriv(t+dt/2,x+dt/2*b,comp,psid,kp,kv,kq,kO,k1,l1,k2,l2)
        d=deriv(t+dt,x+dt*c,comp,psid,kp,kv,kq,kO,k1,l1,k2,l2)
        x=x+dt/6*(a+2*b+2*c+d)
        nq=math.sqrt((x[6:10]**2).sum()); x[6:10]=x[6:10]/nq
    return out
if __name__=="__main__":
    for comp in (True,False):
        D=run(comp,1e-4,20.0,20,psid,kp,kv,kq,kO,k1,l1,k2,l2)
        np.save(f'res_{int(comp)}.npy',D); print('done',comp,D.shape, np.abs(D[-1,1:4]).max())
