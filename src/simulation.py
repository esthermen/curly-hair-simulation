import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from perlin_noise import PerlinNoise
import numpy as np

def random_around(value, sigma=0.05, min_val=None, max_val=None):
    val = np.random.normal(loc=value, scale=sigma)
    if min_val is not None:
        val = max(val, min_val)
    if max_val is not None:
        val = min(val, max_val)
    return val

# -------------------------
# Parámetros físicos
# -------------------------
N =30                             # cantidad de mechones
M = 50                             # partículas por eje
w = 0.5
m = np.ones(M)*w/M
k = 5000*np.ones(M)       
l0 = 0.0015       
g = np.array([0, 0, -9.81])
dt = 0.001               
#amort = 0.01*np.ones(M)
amort = 0.01 + 0.04*np.linspace(0,1,M)

# Rigidez de flexión de la varilla
k_bend = 2*np.exp(-np.linspace(0,2,M))

# Curvatura natural del mechón
theta0 = 0.15   # radianes

r0 = 0.1                           #radio de los rizos aproximado
TURNS = 5
r_circle = 0.2                      #radio de la coleta

R0=np.zeros(N)
turns=np.zeros(N)
R_circle=np.zeros(N)
dtheta=np.zeros(N)
L0=np.zeros(N)
signs=np.zeros(N)
phase=np.zeros(N)                   # fase distinta por mechón
n1_prev = np.zeros((M, N, 3))
for i in range(M):
    for n in range(N):
        n1_prev[i,n] = np.array([1,0,0])


# Posiciones ejes
angles = np.random.uniform(0, 2*np.pi, N)
radii = R_circle * np.sqrt(np.random.uniform(0, 1, N))


for n in range(N):
    R0[n] = random_around(r0, sigma=0.01, min_val=0.05)
    turns[n]=random_around(TURNS, sigma=0.2, min_val=1)
    R_circle[n]=random_around(r_circle, sigma=0.05, min_val=0.05)
    dtheta[n] = 2*np.pi*turns[n]/M
    r_norm = radii[n] / R_circle[n]
    length_factor = 1.0 - 0.5 * (r_norm**2)
    L0[n] = random_around(l0 * length_factor, sigma=0.003, min_val=0.03)
    signs[n] = np.random.choice([-1, 1])  # sentido del rizo
    phase[n] = np.random.uniform(0, 2*np.pi)  # desfase temporal



x = radii * np.cos(angles)
y = radii * np.sin(angles)

# Punto fijo ejes
z_fixed = 10 * np.ones(N)

# -------------------------
# Estados iniciales
# -------------------------
r = np.zeros((M, N, 3))  # eje
v = np.zeros((M, N, 3))

R = np.zeros((M, N, 3))  # rizo geométrico

for n in range(N):
    r[0, n] = np.array([x[n], y[n], z_fixed[n]])
    for i in range(1, M):
        factor = 1 + 0.02 * i   # estiramiento por la gravedad
        r[i,n] = r[i-1,n] - np.array([0,0,L0[n]*factor])
        #r[i, n] = r[i-1, n] - np.array([0, 0, L0[n]])
        

# -------------------------
# Figura
# -------------------------
fig = plt.figure()
ax = fig.add_subplot(projection='3d')

ax.set_xlim(-0.5, 0.5)
ax.set_ylim(-0.5, 0.5)
ax.set_zlim(8, 10.5)

lines = []
points = []

for n in range(N):
    pt, = ax.plot([], [], [], '-')
    lines.append(pt)
    pt, = ax.plot([], [], [], 'o')
    points.append(pt)

# -------------------------
# Fuerzas sobre el eje
# -------------------------
def compute_forces(r, v, t=0.0):
    F = np.zeros_like(r)

    for n in range(N):
        for i in range(M):
            # gravedad + damping
            F[i,n] += m[i]*g - amort[i]*v[i,n]

            # muelle superior
            if i == 0:
                anchor = np.array([x[n], y[n], z_fixed[n]])
                d = r[i,n] - anchor
            else:
                d = r[i,n] - r[i-1,n]

            dist = np.linalg.norm(d)
            if dist > 1e-8:
                F[i,n] += -k[i]*(dist - L0[n])*d/dist

            # muelle inferior
            if i < M-1:
                d = r[i,n] - r[i+1,n]
                dist = np.linalg.norm(d)
                if dist > 1e-8:
                    F[i,n] += -k[i+1]*(dist - L0[n])*d/dist


            # ------------------------------------------------
            # FLEXIÓN DE VARILLA ELÁSTICA
            # ------------------------------------------------

            if 0 < i < M-1:

                target = 0.5*(r[i-1,n] + r[i+1,n])

                Fb = k_bend[i]*(target - r[i,n])

                F[i,n] += Fb

                
            # -------------------------
            # REPULSIÓN ENTRE MECHONES
            # -------------------------
            r_cut = 0.03      # radio efectivo del pelo
            k_rep = 1.0

            for n2 in range(N):

                if n == n2:
                    continue

                d = r[i,n] - r[i,n2]
                dist = np.linalg.norm(d)

                if dist < r_cut and dist > 1e-8:

                    dir = d/dist

                    # fuerza cuadrática suave
                    overlap = r_cut - dist

                    F[i,n] += k_rep*(overlap/r_cut)**2 * dir
            

            

    return F

# -------------------------
# VERLET
# -------------------------

def verlet_step(r, v, dt, t, anchor_pos, anchor_vel):

    a = compute_forces(r, v, t) / m[:,None,None]

    r_new = r + v*dt + 0.5*a*dt**2

    a_new = compute_forces(r_new, v, t) / m[:,None,None]

    v_new = v + 0.5*(a + a_new)*dt

    # imponer ancla 
    for n in range(N):
        r_new[0,n] = anchor_pos[n]
        v_new[0,n] = anchor_vel[n]

    return r_new, v_new

# -------------------------
# Animación
# -------------------------
t = 0.0

def update(frame):

    global r, v, R, t

    # -------------------------
    # Movimiento de cabeza
    # -------------------------

    B = 0.001
    A = 0.2
    w_head = 6.0

    X_head = A*np.sin(w_head*t)
    Y_head = B*np.sin(0.5*w_head*t)

    Vx_head = A*w_head*np.cos(w_head*t)
    Vy_head = B*0.5*w_head*np.cos(0.5*w_head*t)

    # arrays con la posición y velocidad de cada raíz
    anchor_pos = np.zeros((N,3))
    anchor_vel = np.zeros((N,3))

    for n in range(N):

        x[n] = X_head + radii[n]*np.cos(angles[n])
        y[n] = Y_head + radii[n]*np.sin(angles[n])

        anchor_pos[n] = np.array([
            x[n],
            y[n],
            z_fixed[n]
        ])

        anchor_vel[n] = np.array([
            Vx_head,
            Vy_head,
            0.0
        ])

    # -------------------------
    # Actualizar eje
    # -------------------------

    r, v = verlet_step(
        r,
        v,
        dt,
        t,
        anchor_pos,
        anchor_vel
    )

    # -------------------------
    # GENERACIÓN DEL RIZO
    # -------------------------

    for n in range(N):

        for i in range(M):

            if i > 0 and i < M-1:
                t_vec = r[i+1,n] - r[i-1,n]

            elif i > 0:
                t_vec = r[i,n] - r[i-1,n]

            else:
                t_vec = r[i+1,n] - r[i,n]

            norm = np.linalg.norm(t_vec)

            if norm > 1e-8:
                t_vec /= norm
            else:
                t_vec = np.array([0,0,1])

            n1 = n1_prev[i,n] - np.dot(
                n1_prev[i,n],
                t_vec
            )*t_vec

            norm = np.linalg.norm(n1)

            if norm < 1e-6:

                up = np.array([0,0,1])

                n1 = np.cross(t_vec, up)

                if np.linalg.norm(n1) < 1e-6:
                    up = np.array([1,0,0])
                    n1 = np.cross(t_vec, up)

            n1 /= np.linalg.norm(n1)

            n2 = np.cross(t_vec, n1)
            n2 /= np.linalg.norm(n2)

            n1_prev[i,n] = n1

            alpha = i/(M-1)

            radius = R0[n]*(0.6 + 0.4*alpha)
            radius *= (1 + 0.1*alpha**2)

            theta = signs[n]*(i*dtheta[n])

            R[i,n] = (
                r[i,n]
                + radius*(
                    np.cos(theta)*n1
                    + np.sin(theta)*n2
                )
            )

    # -------------------------
    # Dibujar
    # -------------------------

    artists = []

    for n in range(N):

        lines[n].set_data(
            R[:,n,0],
            R[:,n,1]
        )

        lines[n].set_3d_properties(
            R[:,n,2]
        )

        artists.append(lines[n])

    t += dt

    return artists

# -------------------------
# RELAJACIÓN AUTOMÁTICA
# -------------------------
max_steps = 4000
threshold = 1e-4
noise = 0.005*np.random.randn(*r.shape)
noise[:,:,2] = 0
r += noise
anchor_pos = np.zeros((N,3))
anchor_vel = np.zeros((N,3))

for n in range(N):

    anchor_pos[n] = np.array([
        x[n],
        y[n],
        z_fixed[n]
    ])

for step in range(max_steps):

    r_old = r.copy()

    r, v = verlet_step(
        r,
        v,
        dt,
        t,
        anchor_pos,
        anchor_vel
    )

    # criterio de convergencia basado en cambio de posición
    delta = np.max(np.linalg.norm(r - r_old, axis=2))

    if step % 100 == 0:
        print(f"step {step} | delta = {delta:.6f}")

        
    if delta < threshold:
        print(f"Relajado en {step} pasos")
        break
else:
    print("No convergió completamente")


t=0.0

ani = animation.FuncAnimation(
    fig,
    update,
    frames=10000,
    interval=20,
    blit=True
)

ani.save("ponytail.gif", writer="pillow", fps=60)
