import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import tkinter as tk
import threading, math, time

# ── Geometría (del informe) ──────────────────────────────────────────
L1, L2, D1 = 0.13, 0.13, 0.10          # metros

# ── Puntos de la tarea [cm, frame SCARA] ────────────────────────────
HOME_Q = (0.0, 0.0, 0.0)               # joints en HOME

PICK   = (13.0,  0.0,  5.0)            # recogida de pelota (CYAN)
REPO   = (-5.0, 15.0,  5.0)            # repositorio bolas  (MAGENTA)
C1     = ( 6.5, 17.48, -3.0)           # depósito 1         (AMARILLO)
C2     = ( 0.5, 19.98, -3.0)           # depósito 2         (NARANJA)
C3     = (-6.5, 17.98, -3.0)           # depósito 3         (ROJO)

Z_T = 7.0    # [cm] altura de tránsito (solo mueve q1,q2)
Z_D = -3.0   # [cm] altura de depósito / pick (prismatico abajo)


# ── Cinemática ───────────────────────────────────────────────────────

def fk_cm(q1, q2, q3_gz):
    x = (L1*math.cos(q1) + L2*math.cos(q1+q2))*100
    y = (L1*math.sin(q1) + L2*math.sin(q1+q2))*100
    z = D1*100 + q3_gz*100
    return x, y, z


def ik(x_cm, y_cm, z_cm):
    x, y = x_cm/100, y_cm/100
    r2 = x**2 + y**2;  r = math.sqrt(r2)
    cos_q2 = max(-1.0, min(1.0, (r2-L1**2-L2**2)/(2*L1*L2)))
    q2 = math.acos(cos_q2)
    arg = max(-1.0, min(1.0, (r2+L1**2-L2**2)/(2*L1*r))) if r>1e-6 else 0.0
    q1 = math.atan2(y, x) - math.acos(arg)
    q3_gz = max(-0.15, min(0.0, z_cm/100 - D1))
    return q1, q2, q3_gz


def transit(pt):
    """Misma XY que pt, altura de tránsito."""
    return (pt[0], pt[1], Z_T)


# ── Perfiles ─────────────────────────────────────────────────────────

def trapezoid(q0, qf, vmax, amax, dt):
    dq = qf-q0; sign = 1 if dq>=0 else -1; dist=abs(dq)
    if dist<1e-7: return [q0]
    ta=vmax/amax; da=0.5*amax*ta**2
    if dist<=2*da:
        ta=math.sqrt(dist/amax); tt=2*ta; vp=amax*ta; tc=0.0
    else:
        tc=(dist-2*da)/vmax; tt=2*ta+tc; vp=vmax
    pts=[]; t=0.0
    while True:
        t=min(t,tt)
        if   t<=ta:     s=0.5*amax*t**2
        elif t<=ta+tc:  s=da+vp*(t-ta)
        else:           s=dist-0.5*amax*(tt-t)**2
        pts.append(q0+sign*min(max(s,0),dist))
        if t>=tt: break
        t+=dt
    pts.append(qf); return pts


def joint_traj(qs, qe, vm, am, dt):
    """3-DOF sincronizado."""
    tots=[]
    for i in range(3):
        d=abs(qe[i]-qs[i])
        if d<1e-7: tots.append(0.0); continue
        ta=vm[i]/am[i]; da=0.5*am[i]*ta**2
        tots.append(2*ta+(d-2*da)/vm[i] if d>2*da else 2*math.sqrt(d/am[i]))
    tsync=max(tots)
    if tsync<1e-7: return [tuple(qe)]
    trajs=[]
    for i in range(3):
        d=abs(qe[i]-qs[i])
        if d<1e-7:
            trajs.append([qs[i]]*max(1,int(tsync/dt)+1)); continue
        a=am[i]
        disc=(tsync*a)**2-4*a*d
        vn=(tsync*a-math.sqrt(max(disc,0)))/2 if disc>=0 else d/tsync
        vn=min(vn,vm[i])
        pts=trapezoid(qs[i],qe[i],vn,a,dt)
        n=max(len(pts),int(tsync/dt)+1)
        while len(pts)<n: pts.append(pts[-1])
        trajs.append(pts)
    n=max(len(t) for t in trajs)
    for t in trajs:
        while len(t)<n: t.append(t[-1])
    return list(zip(*trajs))


def cart_traj(p0, p1, vc, ac, dt, q0=None):
    """Línea recta cartesiana + IK."""
    dp=[p1[i]-p0[i] for i in range(3)]
    dist=math.sqrt(sum(d**2 for d in dp))
    if dist<1e-4: return [ik(*p0)]
    sl=trapezoid(0.0,1.0,vc/dist,ac/dist,dt)
    wps=[]; qp=list(q0) if q0 else list(ik(*p0))
    for s in sl:
        p=[p0[i]+s*dp[i] for i in range(3)]
        try:
            q=ik(*p); wps.append(q); qp=list(q)
        except: wps.append(tuple(qp))
    return wps


# ── Nodo ROS ─────────────────────────────────────────────────────────

class ScaraNode(Node):
    def __init__(self):
        super().__init__('scara_trayectoria')
        self.pub={
            'q1':self.create_publisher(Float64,'/Union_1_joint/cmd_pos',10),
            'q2':self.create_publisher(Float64,'/union_2_joint/cmd_pos',10),
            'd3':self.create_publisher(Float64,'/prismatic_joint/cmd_pos',10),
        }
        self.q_cur=[0.0,0.0,0.0]; self.running=False

    def send(self,q1,q2,d3):
        self.pub['q1'].publish(Float64(data=float(q1)))
        self.pub['q2'].publish(Float64(data=float(q2)))
        self.pub['d3'].publish(Float64(data=float(d3)))
        self.q_cur=[q1,q2,d3]

    def run_wps(self,wps,dt):
        for wp in wps:
            if not self.running: break
            self.send(*wp); time.sleep(dt)
        if wps: self.q_cur=list(wps[-1])


# ── GUI ──────────────────────────────────────────────────────────────

STEPS_LOG = [
    "1  HOME",
    "2  → PICK (tránsito)",
    "3  ↓ PICK baja (prismatico)",
    "4  ↑ PICK sube (prismatico)",
    "5  → C1 (tránsito)",
    "6  ↓ C1 baja",
    "7  ↑ C1 sube",
    "8  → REPO (tránsito)",
    "9  ↓ REPO baja",
    "10 ↑ REPO sube",
    "11 → C2 (tránsito)",
    "12 ↓ C2 baja",
    "13 ↑ C2 sube",
    "14 → REPO (tránsito)",
    "15 ↓ REPO baja",
    "16 ↑ REPO sube",
    "17 → C3 (tránsito)",
    "18 ↓ C3 baja",
    "19 ↑ C3 sube",
    "20 HOME ✅",
]

class GUI:
    BG='#0d1117'; CARD='#161b22'; A1='#ffd93d'; A2='#00ff88'; BL='#58a6ff'

    def __init__(self,node):
        self.node=node
        self.root=tk.Tk()
        self.root.title('SCARA — Planeación de Trayectorias')
        self.root.configure(bg=self.BG)
        self.root.resizable(False,False)
        self._build()

    def _build(self):
        BG,CA,A1,A2,BL=self.BG,self.CARD,self.A1,self.A2,self.BL

        tk.Label(self.root,text='SCARA — Planeación de Trayectorias',
                 font=('Courier New',13,'bold'),fg=BL,bg=BG,pady=10).pack()
        tk.Frame(self.root,height=2,bg=BL).pack(fill='x',padx=16)

        # Parámetros
        pf=tk.Frame(self.root,bg=CA,padx=20,pady=8)
        pf.pack(fill='x',padx=16,pady=6)
        tk.Label(pf,text='Parámetros',font=('Courier New',9,'bold'),fg=A1,bg=CA
                 ).grid(row=0,column=0,columnspan=10,sticky='w')
        params=[('v art[rad/s]','1.5'),('a art[rad/s²]','3.0'),
                ('v cart[cm/s]','8.0'),('a cart[cm/s²]','20.0'),('dt[s]','0.05')]
        self.pv={}
        for i,(lbl,val) in enumerate(params):
            tk.Label(pf,text=lbl,font=('Courier New',8),fg='#8b949e',bg=CA
                     ).grid(row=1,column=i*2,padx=(0,2))
            v=tk.StringVar(value=val)
            tk.Entry(pf,textvariable=v,width=6,font=('Courier New',9),
                     bg='#21262d',fg='white',insertbackground='white',relief='flat'
                     ).grid(row=2,column=i*2)
            self.pv[lbl]=v

        # Botones
        bf=tk.Frame(self.root,bg=BG,pady=8); bf.pack()
        tk.Button(bf,text='▶  Espacio Articular\n(perfil trapezoidal)',
                  font=('Courier New',11,'bold'),fg='#0d1117',bg=A1,
                  relief='flat',padx=14,pady=8,width=22,cursor='hand2',
                  command=self._run_joint).grid(row=0,column=0,padx=8)
        tk.Button(bf,text='▶  Espacio Cartesiano\n(líneas rectas + IK)',
                  font=('Courier New',11,'bold'),fg='#0d1117',bg=A2,
                  relief='flat',padx=14,pady=8,width=22,cursor='hand2',
                  command=self._run_cart).grid(row=0,column=1,padx=8)
        tk.Button(bf,text='■  STOP',font=('Courier New',10,'bold'),
                  fg='white',bg='#f85149',relief='flat',padx=14,pady=6,
                  cursor='hand2',command=self._stop
                  ).grid(row=1,column=0,pady=(6,0))
        tk.Button(bf,text='⟳  HOME',font=('Courier New',10,'bold'),
                  fg='#0d1117',bg=BL,relief='flat',padx=14,pady=6,
                  cursor='hand2',command=self._home
                  ).grid(row=1,column=1,pady=(6,0))

        tk.Frame(self.root,height=1,bg='#21262d').pack(fill='x',padx=16)

        # Leyenda de puntos
        lef=tk.Frame(self.root,bg=CA,padx=16,pady=6)
        lef.pack(fill='x',padx=16,pady=(4,0))
        items=[('■','#00ccff','PICK  (13,0,5) cm'),
               ('■','#cc00cc','REPO  (-5,15,5) cm'),
               ('■','#ffdd00','C1    (6.5,17.48,-3) cm'),
               ('■','#ff7700','C2    (0.5,19.98,-3) cm'),
               ('■','#ff2222','C3    (-6.5,17.98,-3) cm')]
        for col,(sym,color,txt) in enumerate(items):
            tk.Label(lef,text=sym,fg=color,bg=CA,font=('Courier New',10,'bold')
                     ).grid(row=0,column=col*2,padx=(8,1))
            tk.Label(lef,text=txt,fg='#8b949e',bg=CA,font=('Courier New',8)
                     ).grid(row=0,column=col*2+1,padx=(0,8))

        tk.Frame(self.root,height=1,bg='#21262d').pack(fill='x',padx=16)

        # Progreso
        prf=tk.Frame(self.root,bg=CA,padx=16,pady=6)
        prf.pack(fill='x',padx=16,pady=(4,0))
        tk.Label(prf,text='Progreso',font=('Courier New',9,'bold'),fg=A2,bg=CA
                 ).grid(row=0,column=0,columnspan=4,sticky='w')
        self.step_labels=[]
        for i,s in enumerate(STEPS_LOG):
            lbl=tk.Label(prf,text=s,font=('Courier New',8),
                         fg='#484f58',bg=CA,anchor='w',width=28)
            lbl.grid(row=1+i//4,column=i%4,sticky='w',padx=4)
            self.step_labels.append(lbl)

        # Log
        lf=tk.Frame(self.root,bg=CA,padx=14,pady=6)
        lf.pack(fill='both',expand=True,padx=16,pady=(4,0))
        tk.Label(lf,text='Log',font=('Courier New',9,'bold'),fg=A2,bg=CA
                 ).pack(anchor='w')
        self.log=tk.Text(lf,height=6,font=('Courier New',8),bg='#0d1117',
                         fg='#c9d1d9',insertbackground='white',relief='flat',
                         state='disabled')
        self.log.pack(fill='both')
        self.stlbl=tk.Label(self.root,text='● Listo',
                            font=('Courier New',9),fg=A2,bg=BG,pady=5)
        self.stlbl.pack()

    # ── helpers ──────────────────────────────────────────────────────
    def _p(self,k): return float(self.pv[k].get())
    def _log(self,m):
        self.log.config(state='normal')
        self.log.insert(tk.END,m+'\n'); self.log.see(tk.END)
        self.log.config(state='disabled')
    def _st(self,m,c='#00ff88'): self.stlbl.config(text=m,fg=c)
    def _stop(self): self.node.running=False; self._st('■ Detenido','#f85149')
    def _home(self): self.node.send(0,0,0); self.node.q_cur=[0,0,0]; self._log('⟳ HOME')
    def _mark_step(self,i):
        for j,lbl in enumerate(self.step_labels):
            if j<i:    lbl.config(fg='#3fb950')   # completado
            elif j==i: lbl.config(fg='#ffd93d')   # activo
            else:      lbl.config(fg='#484f58')   # pendiente

    # ── Tarea completa (misma secuencia para ambos modos) ─────────────
    def _execute_task(self, move_fn):
        """
        Recibe move_fn(p_from, p_to, label, step_idx) que ejecuta
        el movimiento en el modo correspondiente.
        """
        dt  = self._p('dt[s]')
        q0  = self.node.q_cur[:]
        p0  = fk_cm(*q0)

        def go(pa, pb, lbl, step, q_=None):
            self.root.after(0, lambda s=step: self._mark_step(s))
            self.root.after(0, lambda l=lbl: self._log(f'  {l}'))
            return move_fn(pa, pb, dt, q_)

        # 1. HOME
        self._mark_step(0); self._log('── INICIO TAREA ──')
        q_ = q0

        # 2. HOME → PICK tránsito
        q_ = go(p0, transit(PICK), '→ PICK (tránsito)', 1, q_)
        # 3. PICK tránsito → PICK baja  (prismatico baja)
        q_ = go(transit(PICK), PICK, '↓ PICK baja', 2, q_); time.sleep(0.5)
        # 4. PICK baja → PICK tránsito  (prismatico sube)
        q_ = go(PICK, transit(PICK), '↑ PICK sube', 3, q_)

        for idx, (Ci, ci_lbl) in enumerate([(C1,'C1'),(C2,'C2'),(C3,'C3')]):
            base = 4 + idx*6

            # → Ci tránsito
            q_ = go(transit(PICK), transit(Ci), f'→ {ci_lbl} (tránsito)', base, q_)
            # ↓ Ci baja
            q_ = go(transit(Ci), Ci, f'↓ {ci_lbl} baja', base+1, q_); time.sleep(0.5)
            # ↑ Ci sube
            q_ = go(Ci, transit(Ci), f'↑ {ci_lbl} sube', base+2, q_)

            if idx < 2:   # REPO solo entre C1→C2 y C2→C3
                # → REPO tránsito
                q_ = go(transit(Ci), transit(REPO), '→ REPO (tránsito)', base+3, q_)
                # ↓ REPO baja
                q_ = go(transit(REPO), REPO, '↓ REPO baja', base+4, q_); time.sleep(0.5)
                # ↑ REPO sube
                q_ = go(REPO, transit(REPO), '↑ REPO sube', base+5, q_)

        # HOME final
        self.root.after(0, lambda: self._mark_step(19))
        go(fk_cm(*self.node.q_cur), fk_cm(*HOME_Q), '→ HOME final', 19, q_)
        self.node.running = False
        self.root.after(0, lambda: self._st('✅ Tarea completada'))
        self.root.after(0, lambda: self._log('✅ 3 pingpones colocados'))

    # ── Modo articular ────────────────────────────────────────────────
    def _run_joint(self):
        if self.node.running: return
        self._st('⏳ Articular...','#ffd93d')
        vm=[self._p('v art[rad/s]')]*3; vm[2]=0.25
        am=[self._p('a art[rad/s²]')]*3; am[2]=0.5

        def move_fn(pa, pb, dt, q_=None):
            qa = ik(*pa) if not q_ else tuple(q_)
            qb = ik(*pb)
            wps = joint_traj(qa, qb, vm, am, dt)
            self.node.run_wps(wps, dt)
            return list(wps[-1]) if wps else list(qb)

        threading.Thread(
            target=lambda: (self.node.__setattr__('running',True),
                            self._execute_task(move_fn)),
            daemon=True).start()

    # ── Modo cartesiano ───────────────────────────────────────────────
    def _run_cart(self):
        if self.node.running: return
        self._st('⏳ Cartesiana...','#00ff88')
        vc=self._p('v cart[cm/s]'); ac=self._p('a cart[cm/s²]')

        def move_fn(pa, pb, dt, q_=None):
            wps = cart_traj(pa, pb, vc, ac, dt, q_)
            self.node.run_wps(wps, dt)
            return list(wps[-1]) if wps else (list(q_) if q_ else list(ik(*pb)))

        threading.Thread(
            target=lambda: (self.node.__setattr__('running',True),
                            self._execute_task(move_fn)),
            daemon=True).start()

    def run(self): self.root.mainloop()


# ── Main ─────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node=ScaraNode()
    threading.Thread(target=rclpy.spin,args=(node,),daemon=True).start()
    GUI(node).run()
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()