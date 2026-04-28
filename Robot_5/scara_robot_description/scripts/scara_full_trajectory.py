#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

L1, L2, D1 = 0.13, 0.13, 0.0

TOPICS = {
    'q1': '/Union_1_joint/cmd_pos',
    'q2': '/union_2_joint/cmd_pos',
    'q3': '/prismatic_joint/cmd_pos',
}

HOME = (-0.0836, 0.0533,   D1  )
REPO = ( 0.13,   0.00,   -0.01 )
C1   = ( 0.065,  0.1748, -0.01 )
C2   = ( 0.005,  0.1998, -0.01 )
C3   = (-0.065,  0.1798, -0.01 )

TRANSIT_Z = D1
TF_LINE  = 12.0
TF_PRISM =  6.0
TF_HOLD  =  2.0
N_PTS    = 80
HZ       = 50

def ik(x, y):
    r2  = x*x + y*y
    r   = math.sqrt(r2)
    c2  = max(-1., min(1., (r2 - L1*L1 - L2*L2) / (2.*L1*L2)))
    q2  = math.acos(c2)
    val = max(-1., min(1., (r2 + L1*L1 - L2*L2) / (2.*L1*r))) if r > 1e-9 else 1.
    q1  = math.atan2(y, x) - math.acos(val)
    return q1, q2

def tri_profile(q0, qf, tf, n=N_PTS):
    D = qf - q0
    if abs(D) < 1e-12:
        return [(tf*i/n, q0) for i in range(n+1)]
    tq = tf/4
    th = tf/2
    t3q = 3*tf/4
    am = 8.*D/(tf*tf)
    k = am/tq
    dq_tq = .5*k*tq**2
    q_tq = q0 + (1./6.)*k*tq**3
    t2 = th - tq
    dq_th = dq_tq + am*t2 - .5*k*t2**2
    q_th = q_tq + dq_tq*t2 + .5*am*t2**2 - (1./6.)*k*t2**3
    t3 = t3q - th
    dq_t3q = dq_th - .5*k*t3**2
    q_t3q = q_th + dq_th*t3 - (1./6.)*k*t3**3

    pts = []
    for i in range(n+1):
        t = tf*i/n
        if t <= tq:
            q = q0 + (1./6.)*k*t**3
        elif t <= th:
            tau = t - tq
            q = q_tq + dq_tq*tau + .5*am*tau**2 - (1./6.)*k*tau**3
        elif t <= t3q:
            tau = t - th
            q = q_th + dq_th*tau - (1./6.)*k*tau**3
        else:
            tau = t - t3q
            q = q_t3q + dq_t3q*tau - .5*am*tau**2 + (1./6.)*k*tau**3
        pts.append((t, q))
    return pts

def seg_line(p0, p1, tf=TF_LINE, n=N_PTS):
    x0, y0, _ = p0
    x1, y1, _ = p1
    dist = math.hypot(x1-x0, y1-y0)
    prof = tri_profile(0., dist, tf, n)
    pts = []
    for t, s in prof:
        r = s/dist if dist > 1e-9 else 0.
        x = x0 + r*(x1-x0)
        y = y0 + r*(y1-y0)
        q1, q2 = ik(x, y)
        pts.append((t, q1, q2, 0.0))
    return pts

def seg_prism(q1f, q2f, z0, z1, tf=TF_PRISM, n=N_PTS):
    prof = tri_profile(z0-D1, z1-D1, tf, n)
    return [(t, q1f, q2f, q3) for t, q3 in prof]

def seg_hold(q1f, q2f, z, dur=TF_HOLD, n=10):
    q3 = z - D1
    return [(dur*i/n, q1f, q2f, q3) for i in range(n+1)]

class SCARAFullTrajectory(Node):
    def __init__(self):
        super().__init__('scara_full_trajectory')
        self.pub_q1 = self.create_publisher(Float64, TOPICS['q1'], 10)
        self.pub_q2 = self.create_publisher(Float64, TOPICS['q2'], 10)
        self.pub_q3 = self.create_publisher(Float64, TOPICS['q3'], 10)

    def send(self, q1, q2, q3):
        self.pub_q1.publish(Float64(data=float(q1)))
        self.pub_q2.publish(Float64(data=float(q2)))
        self.pub_q3.publish(Float64(data=float(q3)))

    def execute_segment(self, pts, label):
        self.get_logger().info(f'→ {label}')
        dt_step = 1.0 / HZ
        t0 = time.time()
        idx = 0
        while idx < len(pts):
            elapsed = time.time() - t0
            while idx < len(pts)-1 and pts[idx+1][0] <= elapsed:
                idx += 1
            _, q1, q2, q3 = pts[idx]
            self.send(q1, q2, q3)
            time.sleep(dt_step)
            if elapsed >= pts[-1][0]:
                break
        _, q1, q2, q3 = pts[-1]
        self.send(q1, q2, q3)
        time.sleep(0.1)

    def run(self):
        time.sleep(2.0)

        q1h, q2h = ik(*HOME[:2])
        q1r, q2r = ik(*REPO[:2])
        q1c1, q2c1 = ik(*C1[:2])
        q1c2, q2c2 = ik(*C2[:2])
        q1c3, q2c3 = ik(*C3[:2])

        self.send(q1h, q2h, 0.0)
        time.sleep(2.0)

        self.execute_segment(seg_line(HOME, REPO), 'HOME → REPO')
        self.execute_segment(seg_prism(q1r, q2r, TRANSIT_Z, REPO[2]), 'pick_1 baja')
        self.execute_segment(seg_hold(q1r, q2r, REPO[2]), 'pick_1 hold')
        self.execute_segment(seg_prism(q1r, q2r, REPO[2], TRANSIT_Z), 'pick_1 sube')
        self.execute_segment(seg_line(REPO, C1), 'REPO → C1')
        self.execute_segment(seg_prism(q1c1, q2c1, TRANSIT_Z, C1[2]), 'place_1 baja')
        self.execute_segment(seg_hold(q1c1, q2c1, C1[2]), 'place_1 hold')
        self.execute_segment(seg_prism(q1c1, q2c1, C1[2], TRANSIT_Z), 'place_1 sube')

        self.execute_segment(seg_line(C1, REPO), 'C1 → REPO')
        self.execute_segment(seg_prism(q1r, q2r, TRANSIT_Z, REPO[2]), 'pick_2 baja')
        self.execute_segment(seg_hold(q1r, q2r, REPO[2]), 'pick_2 hold')
        self.execute_segment(seg_prism(q1r, q2r, REPO[2], TRANSIT_Z), 'pick_2 sube')
        self.execute_segment(seg_line(REPO, C2), 'REPO → C2')
        self.execute_segment(seg_prism(q1c2, q2c2, TRANSIT_Z, C2[2]), 'place_2 baja')
        self.execute_segment(seg_hold(q1c2, q2c2, C2[2]), 'place_2 hold')
        self.execute_segment(seg_prism(q1c2, q2c2, C2[2], TRANSIT_Z), 'place_2 sube')

        self.execute_segment(seg_line(C2, REPO), 'C2 → REPO')
        self.execute_segment(seg_prism(q1r, q2r, TRANSIT_Z, REPO[2]), 'pick_3 baja')
        self.execute_segment(seg_hold(q1r, q2r, REPO[2]), 'pick_3 hold')
        self.execute_segment(seg_prism(q1r, q2r, REPO[2], TRANSIT_Z), 'pick_3 sube')
        self.execute_segment(seg_line(REPO, C3), 'REPO → C3')
        self.execute_segment(seg_prism(q1c3, q2c3, TRANSIT_Z, C3[2]), 'place_3 baja')
        self.execute_segment(seg_hold(q1c3, q2c3, C3[2]), 'place_3 hold')
        self.execute_segment(seg_prism(q1c3, q2c3, C3[2], TRANSIT_Z), 'place_3 sube')

        self.execute_segment(seg_line(C3, HOME), 'C3 → HOME')
        self.get_logger().info('Secuencia completa ✅')

def main(args=None):
    rclpy.init(args=args)
    node = SCARAFullTrajectory()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()