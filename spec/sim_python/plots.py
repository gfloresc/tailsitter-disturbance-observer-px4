import numpy as np, matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
plt.rcParams.update({'font.family':'serif','font.serif':['cmr10'],'mathtext.fontset':'cm',
 'axes.unicode_minus':False,'axes.formatter.use_mathtext':True,'font.size':7,
 'axes.labelsize':7,'xtick.labelsize':6.5,'ytick.labelsize':6.5,'legend.fontsize':7,
 'xtick.direction':'in','ytick.direction':'in','xtick.top':True,'ytick.right':True,
 'axes.linewidth':0.5,'xtick.major.width':0.5,'ytick.major.width':0.5,'xtick.major.size':2.5,'ytick.major.size':2.5,
 'grid.color':'0.85','grid.linewidth':0.5,'axes.grid':True,'legend.frameon':True,
 'legend.fancybox':False,'legend.edgecolor':'k','legend.framealpha':1,'pdf.fonttype':42,'lines.dashed_pattern':[4.5,2.5]})
C=np.load('res_1.npy'); N=np.load('res_0.npy'); t=C[:,0]
def eul(Q):
    w,x,y,z=Q[:,0],Q[:,1],Q[:,2],Q[:,3]
    phi=np.degrees(np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)))
    th=np.degrees(np.arcsin(np.clip(2*(w*y-z*x),-1,1)))
    ps=np.degrees(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z)))
    return th,phi,ps
GR='0.55'
def fmt(ax,last):
    ax.set_xlim(0,20); ax.set_xticks([0,5,10,15,20])
    if last: ax.set_xlabel('time (s)')
# ---------------- position figure ----------------
fig=plt.figure(figsize=(533/72,310/72)); gs=fig.add_gridspec(4,3,height_ratios=[1,1,0.12,1],hspace=0.38,wspace=0.30,left=0.065,right=0.98,bottom=0.1,top=0.9)
axs=np.array([[fig.add_subplot(gs[r,c]) for c in range(3)] for r in (0,1,3)])
labs=[r'$x$ (m)',r'$y$ (m)',r'$z$ (m)',r'$\dot{x}$ (m/s)',r'$\dot{y}$ (m/s)',r'$\dot{z}$ (m/s)']
for i in range(3):
    for r,(off) in enumerate([1,4]):
        ax=axs[r,i]
        ax.plot(t,np.zeros_like(t),'k-',lw=0.9)
        ax.plot(t,C[:,off+i],color=GR,lw=0.7)
        ax.plot(t,N[:,off+i],'k--',lw=0.7)
        ax.set_ylabel(labs[3*r+i]); fmt(ax,False)
    ax=axs[2,i]
    ax.plot(t,C[:,34+i],'k-',lw=0.8); ax.plot(t,C[:,17+i],'--',color=GR,lw=0.8)
    ax.set_ylabel([r'$F_x$ (N)',r'$F_y$ (N)',r'$F_z$ (N)'][i]); fmt(ax,True)
h1=[Line2D([],[],color='k',lw=0.9),Line2D([],[],color=GR,lw=0.7),Line2D([],[],color='k',ls='--',lw=0.7)]
b0=axs[0,0].get_position(); b2=axs[2,0].get_position(); b21=axs[2,1].get_position()
fig.legend(h1,['Desired','Compensated','Non compensated'],loc='lower center',ncol=3,bbox_to_anchor=(0.62,b0.y1+0.005),handlelength=3,borderpad=0.3,columnspacing=1.2)
fig.text(0.24,b0.y1+0.02,'Position states',ha='center',fontsize=8)
h2=[Line2D([],[],color='k',lw=0.8),Line2D([],[],color=GR,ls='--',lw=0.8)]
fig.legend(h2,['Actual','Observed'],loc='lower center',ncol=2,bbox_to_anchor=(0.62,b2.y1+0.012),handlelength=3,borderpad=0.3)
fig.text(0.24,b2.y1+0.024,'Perturbations',ha='center',fontsize=8)
fig.savefig('sim_pos-pert.pdf')
# ---------------- attitude figure ----------------
fig=plt.figure(figsize=(524/72,310/72)); gs=fig.add_gridspec(4,3,height_ratios=[1,1,0.12,1],hspace=0.38,wspace=0.30,left=0.065,right=0.98,bottom=0.1,top=0.9)
axs=np.array([[fig.add_subplot(gs[r,c]) for c in range(3)] for r in (0,1,3)])
EC=eul(C[:,7:11]); EDC=eul(C[:,30:34]); EN=eul(N[:,7:11]); EDN=eul(N[:,30:34])
names=[r'$\theta$ (deg)',r'$\phi$ (deg)',r'$\psi$ (deg)']
for i in range(3):
    for r,(E,ED) in enumerate([(EC,EDC),(EN,EDN)]):
        ax=axs[r,i]; ax.plot(t,ED[i],'k-',lw=0.9); ax.plot(t,E[i],'--',color=GR,lw=0.8)
        ax.set_ylabel(names[i]); fmt(ax,False)
    j=[1,0,2][i]   # theta<->T_y, phi<->T_x, psi<->T_z
    ax=axs[2,i]; ax.plot(t,C[:,37+j],'k-',lw=0.8); ax.plot(t,C[:,23+j],'--',color=GR,lw=0.8)
    ax.set_ylabel([r'$T_y$ (N$\cdot$m)',r'$T_x$ (N$\cdot$m)',r'$T_z$ (N$\cdot$m)'][i]); fmt(ax,True)
    ax.set_ylim([(-150,150),(-60,140),(-20,120)][i])
b0=axs[0,0].get_position(); b2=axs[2,0].get_position()
h1=[Line2D([],[],color='k',lw=0.9),Line2D([],[],color=GR,ls='--',lw=0.8)]
fig.legend(h1,['Desired','Actual'],loc='lower center',ncol=2,bbox_to_anchor=(0.62,b0.y1+0.005),handlelength=3,borderpad=0.3)
fig.text(0.24,b0.y1+0.02,'Attitude states',ha='center',fontsize=8)
fig.legend(h2,['Actual','Observed'],loc='lower center',ncol=2,bbox_to_anchor=(0.62,b2.y1+0.012),handlelength=3,borderpad=0.3)
fig.text(0.24,b2.y1+0.024,'Perturbations',ha='center',fontsize=8)
fig.savefig('sim_att-pert.pdf')
# ---------------- inputs figure ----------------
fig,axs=plt.subplots(1,2,figsize=(612/72,111/72))
axs[0].plot(t,C[:,26],'k-',lw=0.8); axs[0].set_ylabel(r'$f$ (N)'); fmt(axs[0],True)
axs[1].plot(t,C[:,27],'k-',lw=0.8,label=r'$\tau_x$'); axs[1].plot(t,C[:,28],color=GR,lw=0.8,label=r'$\tau_y$')
axs[1].plot(t,C[:,29],'--',color=GR,lw=0.8,label=r'$\tau_z$')
axs[1].set_ylabel(r'$\tau$ (N$\cdot$m)'); axs[1].set_ylim(-400,600); fmt(axs[1],True); axs[1].legend(loc='upper right',borderpad=0.3,handlelength=2.5)
fig.tight_layout(w_pad=3)
fig.savefig('sim_forces-pert.pdf')
print('ok')
