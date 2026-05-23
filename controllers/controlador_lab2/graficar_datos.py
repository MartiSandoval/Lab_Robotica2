import csv
import os
import matplotlib.pyplot as plt

TIME_STEP_MS = 32  # Cambiar si usas otro TIME_STEP en Webots
Ts = TIME_STEP_MS / 1000.0

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

ESCENARIOS = {
    "simple": os.path.join(SCRIPT_DIR, "datos_sensores_simple.csv"),
    "complejo": os.path.join(SCRIPT_DIR, "datos_sensores_complejo.csv"),
}

fig, axes = plt.subplots(len(ESCENARIOS), 1, figsize=(12, 5 * len(ESCENARIOS)))
if len(ESCENARIOS) == 1:
    axes = [axes]

for ax, (nombre, archivo) in zip(axes, ESCENARIOS.items()):
    if not os.path.exists(archivo):
        ax.set_title(f"Escenario {nombre} — archivo no encontrado: {archivo}")
        ax.axis("off")
        continue

    muestras, z_crudo, z_filtrado, d_kalman = [], [], [], []
    with open(archivo, newline="") as f:
        reader = csv.DictReader(f)
        for i, row in enumerate(reader):
            muestras.append(i * Ts)
            z_crudo.append(float(row["Z_Crudo"]))
            z_filtrado.append(float(row["Z_Filtrado"]))
            d_kalman.append(float(row["D_Kalman"]))

    ax.plot(muestras, z_crudo, label="Z Crudo (sensor)", alpha=0.5, linewidth=0.8)
    ax.plot(muestras, z_filtrado, label="Z Filtrado (EMA)", linewidth=1.2)
    ax.plot(muestras, d_kalman, label="D Kalman (fusión)", linewidth=1.5)
    ax.set_title(f"Escenario {nombre.capitalize()} — señales de distancia frontal")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Distancia estimada (m)")
    ax.legend()
    ax.grid(True, alpha=0.3)

plt.tight_layout()
output_path = os.path.join(SCRIPT_DIR, "comparacion_senales.png")
plt.savefig(output_path, dpi=150)
print(f"Gráfico guardado en {output_path}")
plt.show()
