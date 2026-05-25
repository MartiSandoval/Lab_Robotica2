import csv
import os
import matplotlib.pyplot as plt

TIME_STEP_MS = 32
Ts = TIME_STEP_MS / 1000.0

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ESCENARIOS = ["simple", "complejo"]
MODOS = ["CRUDO", "FILTRADO", "KALMAN"]


def cargar_csv(escenario, modo):
    path = os.path.join(SCRIPT_DIR, f"datos_sensores_{escenario}_{modo}.csv")
    if not os.path.exists(path):
        print(f"  [aviso] No encontrado: {os.path.basename(path)}")
        return None
    datos = {"Z_Crudo": [], "Z_Filtrado": [], "D_Kalman": [], "Avance_Lineal": [], "tiempo": []}
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for i, row in enumerate(reader):
            datos["tiempo"].append(i * Ts)
            datos["Z_Crudo"].append(float(row["Z_Crudo"]))
            datos["Z_Filtrado"].append(float(row["Z_Filtrado"]))
            datos["D_Kalman"].append(float(row["D_Kalman"]))
            # Avance_Lineal puede no existir en CSVs anteriores
            datos["Avance_Lineal"].append(float(row.get("Avance_Lineal", 0)))
    return datos


# --- Figura 1: Análisis de señales (usa el modo KALMAN como referencia) ---
fig1, axes1 = plt.subplots(len(ESCENARIOS), 2, figsize=(16, 5 * len(ESCENARIOS)))
fig1.suptitle("Análisis de señales de distancia y encoder", fontsize=13)

for row_idx, escenario in enumerate(ESCENARIOS):
    ax_dist = axes1[row_idx][0]
    ax_enc  = axes1[row_idx][1]

    datos = cargar_csv(escenario, "KALMAN")
    if datos is None:
        # Intentar con cualquier modo disponible
        for modo in MODOS:
            datos = cargar_csv(escenario, modo)
            if datos:
                break

    if datos is None:
        ax_dist.set_title(f"Escenario {escenario} — sin datos")
        ax_dist.axis("off")
        ax_enc.axis("off")
        continue

    t = datos["tiempo"]
    ax_dist.plot(t, datos["Z_Crudo"],    label="Z Crudo (sensor)",  alpha=0.5, linewidth=0.8)
    ax_dist.plot(t, datos["Z_Filtrado"], label="Z Filtrado (EMA)",  linewidth=1.2)
    ax_dist.plot(t, datos["D_Kalman"],   label="D Kalman (fusión)", linewidth=1.5)
    ax_dist.set_title(f"Escenario {escenario.capitalize()} — distancia frontal")
    ax_dist.set_xlabel("Tiempo (s)")
    ax_dist.set_ylabel("Distancia (m)")
    ax_dist.legend()
    ax_dist.grid(True, alpha=0.3)

    ax_enc.plot(t, datos["Avance_Lineal"], color="darkorange", linewidth=0.9)
    ax_enc.axhline(0, color="gray", linewidth=0.6, linestyle="--")
    ax_enc.set_title(f"Escenario {escenario.capitalize()} — avance lineal (encoders)")
    ax_enc.set_xlabel("Tiempo (s)")
    ax_enc.set_ylabel("Avance por paso (m)")
    ax_enc.grid(True, alpha=0.3)

plt.tight_layout()
out1 = os.path.join(SCRIPT_DIR, "senales_distancia.png")
fig1.savefig(out1, dpi=150)
print(f"Figura 1 guardada: {out1}")


# --- Figura 2: Comparación de modos de navegación ---
fig2, axes2 = plt.subplots(len(ESCENARIOS), 1, figsize=(14, 5 * len(ESCENARIOS)))
fig2.suptitle("Comparación de modos de navegación (Z Crudo por modo)", fontsize=13)
if len(ESCENARIOS) == 1:
    axes2 = [axes2]

COLORES = {"CRUDO": "tab:red", "FILTRADO": "tab:blue", "KALMAN": "tab:green"}

for row_idx, escenario in enumerate(ESCENARIOS):
    ax = axes2[row_idx]
    algun_dato = False
    for modo in MODOS:
        datos = cargar_csv(escenario, modo)
        if datos is None:
            continue
        ax.plot(datos["tiempo"], datos["Z_Crudo"],
                label=f"Modo {modo}", color=COLORES[modo], alpha=0.75, linewidth=1.0)
        algun_dato = True

    if not algun_dato:
        ax.set_title(f"Escenario {escenario} — sin datos")
        ax.axis("off")
        continue

    ax.set_title(f"Escenario {escenario.capitalize()} — Z Crudo según modo de navegación")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Z Crudo (m)")
    ax.legend()
    ax.grid(True, alpha=0.3)

plt.tight_layout()
out2 = os.path.join(SCRIPT_DIR, "comparacion_modos.png")
fig2.savefig(out2, dpi=150)
print(f"Figura 2 guardada: {out2}")

plt.show()
