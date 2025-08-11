import cv2
import numpy as np
import time
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685

import csv
import matplotlib.pyplot as plt
from mpu6050 import mpu6050

# =============================================================================
# =              CONFIGURATIONS ET CONSTANTES (en français)                    =
# =============================================================================

ADRESSE_I2C_MPU        = 0x68
NBR_ECHANTILLONS_CALIB = 100
DELAI_ECHANTILLON      = 0.01
SEUIL_IMMOBILE         = 0.2   # pour l'accélération, qu’on considère “immobile” en ax/ay
COEFF_FILTRE_VITESSE   = 0.8
FICHIER_CSV            = "f1_data.csv"

# Paramètres du suivi de ligne
GAIN_PROPORTIONNEL = 0.1 #0.15 #0;2  APRES 5
CENTRE_X           = 640
SERVO_CENTRE       = 90
SERVO_MIN          = 60
SERVO_MAX          = 120
FACTEUR_DIRECTION  = -1  # +1 ou -1 pour inverser le sens si besoin

# Vitesse d'arrêt, vitesse min de roulage, et vitesse max
VITESSE_ARRET = 0.05
VITESSE_MIN   = 0.055 #0.059 0.060 A 5 0.055
VITESSE_MAX   = 0.068 #0.065 0.07 A 0.059

# Facteur de différentiel (roue intérieure vs. extérieure)
FACTEUR_DIFF           = 0.2  # À ajuster si nécessaire
ERREUR_SEUIL_AUTO_4WD  = 150  # px d'erreur pour basculer en 4 roues (en mode AUTO)
 #300 5
# Trois modes possibles: "RWD", "4WD" ou "AUTO"
MODE_TRANSMISSION = "RWD"

# Ramping (progressivité)
PAS_RAMPING      = 0.009 #0.009 5
vitesse_actuelle = VITESSE_MIN  # Au démarrage, on se met à la vitesse min

def formater_temps(temps_ecoule):
    """Retourne une chaîne "Xmin Ys Zms". Par ex: 1min 3s 20ms."""
    minutes = int(temps_ecoule // 60)
    secondes = int(temps_ecoule % 60)
    frac_s  = temps_ecoule - int(temps_ecoule)
    millisecondes = int(frac_s * 1000)
    return f"{minutes}min {secondes}s {millisecondes}ms"


# =============================================================================
# =                    INITIALISATION PCA, SERVO, MOTEURS                     =
# =============================================================================

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # pour servo + ESC

canal_servo = pca.channels[7]
derniere_direction_steer = 0
dernier_angle_servo      = None

canaux_moteurs = {
    'AVG': pca.channels[0],  # Avant Gauche
    'AVD': pca.channels[3],  # Avant Droite
    'ARG': pca.channels[2],  # Arrière Gauche
    'ARD': pca.channels[1],  # Arrière Droite
} 

def regler_angle_servo(canal, angle):
    global derniere_direction_steer, dernier_angle_servo
    angle = round(angle, 2)
    if dernier_angle_servo is not None and abs(angle - dernier_angle_servo) < 0.01:
        return

    dernier_angle_servo = angle
    angle = max(SERVO_MIN, min(SERVO_MAX, angle))

    impulsion_min = 500
    impulsion_max = 2500
    impulsion = impulsion_min + (angle / 180.0) * (impulsion_max - impulsion_min)
    pulse_length = int((impulsion / 1_000_000.0) * pca.frequency * 65535)
    canal.duty_cycle = pulse_length

    # Mémoriser la direction générale (pour braquage max si on perd la ligne)
    if angle > SERVO_CENTRE:
        derniere_direction_steer = 1
    elif angle < SERVO_CENTRE:
        derniere_direction_steer = -1
    else:
        derniere_direction_steer = 0


def regler_vitesse_moteur(canal, vitesse):
    """Envoie un rapport cyclique [0.0, 1.0] à l'ESC."""
    vitesse = max(0.0, min(1.0, vitesse))
    canal.duty_cycle = int(0xFFFF * vitesse)

def arret_urgence():
    """Stoppe vraiment tous les moteurs (vitesse_arret)."""
    for canal in canaux_moteurs.values():
        regler_vitesse_moteur(canal, VITESSE_ARRET)


def calcul_vitesse_de_base(erreur_ligne):
    """
    Plus le virage est grand, plus la vitesse diminue vers VITESSE_MIN.
    Plus le virage est doux, plus la vitesse augmente vers VITESSE_MAX.

    Erreur = 0 => vitesse = VITESSE_MAX
    Erreur = ERREUR_MAX => vitesse = VITESSE_MIN
    """
    ERREUR_MAX = 900.0 #300 #600
    ratio = min(abs(erreur_ligne) / ERREUR_MAX, 1.0)

    vitesse_base = VITESSE_MAX - ratio * (VITESSE_MAX - VITESSE_MIN)

    # Limiter entre VITESSE_MIN et VITESSE_MAX
    vitesse_base = max(VITESSE_MIN, min(vitesse_base, VITESSE_MAX))

    return vitesse_base


def mise_a_jour_moteurs(vitesse_base, erreur_ligne):
    """
    Calcule la vitesse à appliquer aux 4 roues en fonction:
      - du MODE_TRANSMISSION (RWD, 4WD, AUTO)
      - de l'erreur pour le différentiel (roue int / ext)
    et gère un ramping pour éviter les à-coups.
    """
    global vitesse_actuelle

    # 1) Ramping progressif
    if vitesse_actuelle < vitesse_base:
        vitesse_actuelle = min(vitesse_actuelle + PAS_RAMPING, vitesse_base)
    elif vitesse_actuelle > vitesse_base:
        vitesse_actuelle = max(vitesse_actuelle - PAS_RAMPING, vitesse_base)

    # 2) Différentiel (roue intérieure vs. extérieure)
    ERREUR_MAX = 400.0 
    intensite_virage = min(abs(erreur_ligne) / ERREUR_MAX, 1.0)

    if erreur_ligne > 0:  # virage à droite => roues droites intérieures
        vitesse_droite = vitesse_actuelle * (1 - FACTEUR_DIFF * intensite_virage)
        vitesse_gauche = vitesse_actuelle * (1 + FACTEUR_DIFF * intensite_virage)
    else:  # virage à gauche => roues gauches intérieures
        vitesse_gauche = vitesse_actuelle * (1 - FACTEUR_DIFF * intensite_virage)
        vitesse_droite = vitesse_actuelle * (1 + FACTEUR_DIFF * intensite_virage)

    # 3) Limiter les vitesses entre VITESSE_MIN et VITESSE_MAX
    vitesse_gauche = max(VITESSE_MIN, min(vitesse_gauche, VITESSE_MAX))
    vitesse_droite = max(VITESSE_MIN, min(vitesse_droite, VITESSE_MAX))

    # 4) Application selon MODE_TRANSMISSION
    if MODE_TRANSMISSION == "RWD":
        # propulsion seulement aux roues arrière
        regler_vitesse_moteur(canaux_moteurs['ARG'], vitesse_gauche)
        regler_vitesse_moteur(canaux_moteurs['ARD'], vitesse_droite)
        # roues avant à 0
        regler_vitesse_moteur(canaux_moteurs['AVG'], 0.0)
        regler_vitesse_moteur(canaux_moteurs['AVD'], 0.0)
        return (vitesse_gauche, 0.0, 0.0, vitesse_droite)

    elif MODE_TRANSMISSION == "4WD":
        # 4 roues actives
        regler_vitesse_moteur(canaux_moteurs['AVG'], vitesse_gauche)
        regler_vitesse_moteur(canaux_moteurs['ARG'], vitesse_gauche)
        regler_vitesse_moteur(canaux_moteurs['AVD'], vitesse_droite)
        regler_vitesse_moteur(canaux_moteurs['ARD'], vitesse_droite)
        return (vitesse_gauche, vitesse_gauche, vitesse_droite, vitesse_droite)

    else:  # "AUTO"
        # Si l'erreur dépasse un certain seuil, on passe en 4 roues
        if abs(erreur_ligne) > ERREUR_SEUIL_AUTO_4WD:
            regler_vitesse_moteur(canaux_moteurs['AVG'], vitesse_gauche)
            regler_vitesse_moteur(canaux_moteurs['ARG'], vitesse_gauche)
            regler_vitesse_moteur(canaux_moteurs['AVD'], vitesse_droite)
            regler_vitesse_moteur(canaux_moteurs['ARD'], vitesse_droite)
            return (vitesse_gauche, vitesse_gauche, vitesse_droite, vitesse_droite)
        else:
            # Sinon, on reste en propulsion RWD
            regler_vitesse_moteur(canaux_moteurs['ARG'], vitesse_gauche)
            regler_vitesse_moteur(canaux_moteurs['ARD'], vitesse_droite)
            regler_vitesse_moteur(canaux_moteurs['AVG'], 0.0)
            regler_vitesse_moteur(canaux_moteurs['AVD'], 0.0)
            return (vitesse_gauche, 0.0, 0.0, vitesse_droite)


# =============================================================================
# =                              PROGRAMME PRINCIPAL                           =
# =============================================================================

def main():
    # -------------------------------------------------------------------------
    # 1) CALIBRATION MPU
    # -------------------------------------------------------------------------
    print("[MPU] Calibration du MPU6050...")
    mpu = mpu6050(ADRESSE_I2C_MPU)

    offsets_accel = [0.0, 0.0, 0.0]
    offsets_gyro  = [0.0, 0.0, 0.0]
    for _ in range(NBR_ECHANTILLONS_CALIB):
        mesures_accel = mpu.get_accel_data()
        mesures_gyro  = mpu.get_gyro_data()
        offsets_accel[0] += mesures_accel['x']
        offsets_accel[1] += mesures_accel['y']
        offsets_accel[2] += mesures_accel['z']
        offsets_gyro[0]  += mesures_gyro['x']
        offsets_gyro[1]  += mesures_gyro['y']
        offsets_gyro[2]  += mesures_gyro['z']
        time.sleep(DELAI_ECHANTILLON)

    offsets_accel = [v / NBR_ECHANTILLONS_CALIB for v in offsets_accel]
    offsets_gyro  = [v / NBR_ECHANTILLONS_CALIB for v in offsets_gyro]

    print(f"[MPU] Offsets Accel : {offsets_accel}")
    print(f"[MPU] Offsets Gyro  : {offsets_gyro}")

    # -------------------------------------------------------------------------
    # 2) VARIABLES D'ÉTAT
    # -------------------------------------------------------------------------
    temps_precedent   = time.time()
    vitesse_x         = 0.0
    vitesse_y         = 0.0
    vitesse_filtree_x = 0.0
    vitesse_filtree_y = 0.0
    vitesse_max       = 0.0
    somme_vitesses    = 0.0
    compteur_vitesses = 0
    yaw               = 0.0  # orientation (°)

    # Pour grapher et logguer
    temps_log           = []
    accel_log           = []
    gyro_log            = []
    vitesses_log        = []
    yaw_log             = []
    erreur_ligne_log    = []
    servo_angle_log     = []
    correction_log      = []  # Ajout pour la correction proportionnelle
    vitesses_AVG        = []
    vitesses_AVD        = []
    vitesses_ARG        = []
    vitesses_ARD        = []

    # -------------------------------------------------------------------------
    # 3) PRÉPARATION DU CSV
    # -------------------------------------------------------------------------
    with open(FICHIER_CSV, "w", newline="", encoding="utf-8") as f:
        ecrivain = csv.writer(f, delimiter=";")
        ecrivain.writerow([
            "Temps",
            "Accel_X", "Accel_Y", "Accel_Z",
            "Gyro_X",  "Gyro_Y",  "Gyro_Z",
            "Vitesse_X", "Vitesse_Y",
            "Vitesse_Globale",
            "Yaw",
            "Erreur_Ligne",
            "Angle_Servo",
            "Correction",
            "Vitesse_AVG",
            "Vitesse_AVD",
            "Vitesse_ARG",
            "Vitesse_ARD"
        ])

    # -------------------------------------------------------------------------
    # 4) INIT SERVO + MOTEURS + CAMÉRA
    # -------------------------------------------------------------------------
    print(f"[INFO] Mode transmission : {MODE_TRANSMISSION}")
    print("[SERVO] Calibration (90°) pendant 3s...")
    regler_angle_servo(canal_servo, SERVO_CENTRE)
    fin_calibration = time.time() + 3

    # On envoie la "vitesse d'arrêt" pour armer les ESC pendant la calibration
    for c in canaux_moteurs.values():
        regler_vitesse_moteur(c, VITESSE_ARRET)

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 60)

    if not cap.isOpened():
        print("[ERREUR] Impossible d'ouvrir la caméra.")
        return

    debut = time.time()

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    sortie_video = cv2.VideoWriter('output.avi', fourcc, 20.0, (1280, 720))

    bas_vert  = np.array([40, 40, 40])
    haut_vert = np.array([80, 255, 255])
    dernier_points_detectes = []
    nbr_sections    = 6
    hauteur_section = 50

    print("[INFO] Boucle principale : Ctrl + C pour arrêter.")
    try:
        while True:
            t_actuel = time.time()
            dt = t_actuel - temps_precedent
            temps_precedent = t_actuel

            # -----------------------------------------------------------------
            # (a) LECTURE MPU
            # -----------------------------------------------------------------
            mesures_accel = mpu.get_accel_data()
            mesures_gyro  = mpu.get_gyro_data()

            ax = mesures_accel['x'] - offsets_accel[0]
            ay = mesures_accel['y'] - offsets_accel[1]
            az = mesures_accel['z'] - offsets_accel[2]
            gx = mesures_gyro['x']  - offsets_gyro[0]
            gy = mesures_gyro['y']  - offsets_gyro[1]
            gz = mesures_gyro['z']  - offsets_gyro[2]

            yaw += gz * dt

            # Si on considère qu'on est immobile quand ax et ay sont < SEUIL_IMMOBILE
            if abs(ax) < SEUIL_IMMOBILE and abs(ay) < SEUIL_IMMOBILE:
                vitesse_x, vitesse_y = 0.0, 0.0
            else:
                vitesse_x += ax * dt
                vitesse_y += ay * dt

            vitesse_filtree_x = COEFF_FILTRE_VITESSE * vitesse_filtree_x + (1 - COEFF_FILTRE_VITESSE) * vitesse_x
            vitesse_filtree_y = COEFF_FILTRE_VITESSE * vitesse_filtree_y + (1 - COEFF_FILTRE_VITESSE) * vitesse_y
            vitesse_globale = np.sqrt(vitesse_filtree_x**2 + vitesse_filtree_y**2)

            # Pour stats
            vitesse_max     = max(vitesse_max, vitesse_globale)
            somme_vitesses += vitesse_globale
            compteur_vitesses += 1

            # -----------------------------------------------------------------
            # (b) CAMERA ET DÉTECTION DE LIGNE
            # -----------------------------------------------------------------
            ok, frame = cap.read()
            if not ok:
                print("[WARN] Problème de lecture de la caméra.")
                break

            frame = cv2.flip(frame, -1)

            erreur_ligne  = 0.0
            angle_servo   = SERVO_CENTRE
            correction    = 0.0  # Initialisation de la correction

            if t_actuel < fin_calibration:
                cv2.putText(frame, "Calibration servo...", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                regler_angle_servo(canal_servo, SERVO_CENTRE)
                points_detectes = []
            else:
                hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                masque = cv2.inRange(hsv, bas_vert, haut_vert)

                points_detectes = []
                hauteur, largeur = frame.shape[:2]

                for i in range(nbr_sections):
                    top    = hauteur - (i + 1) * hauteur_section
                    bottom = hauteur - i       * hauteur_section
                    left   = int(largeur * 0.05)
                    right  = int(largeur * 0.95)
                    zone   = masque[top:bottom, left:right]
                    M      = cv2.moments(zone)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"]) + left
                        cy = (top + bottom) // 2
                        points_detectes.append((cx, cy))
                        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                if points_detectes:
                    x_coords, y_coords = zip(*points_detectes)
                    # Erreur = (X du dernier point détecté) - (CENTRE_X)
                    erreur_ligne = x_coords[-1] - CENTRE_X
                    correction   = GAIN_PROPORTIONNEL * erreur_ligne * FACTEUR_DIRECTION
                    angle_servo  = SERVO_CENTRE + correction
                    regler_angle_servo(canal_servo, angle_servo)

                    cv2.putText(frame,
                                f"Erreur: {erreur_ligne:.2f}, Correction: {correction:.2f}, Angle servo: {angle_servo:.2f}",
                                (10, 70), cv2.FONT_HERSHEY_SIMPLEX,
                                1, (255, 0, 0), 2)

                    dernier_points_detectes = points_detectes
                else:
                    # Plus de points => on applique la logique "prediction" ou braquage max
                    if dernier_points_detectes:
                        x_coords, y_coords = zip(*dernier_points_detectes)
                        point_prevu = (x_coords[-1], y_coords[-1] - hauteur_section)
                        cv2.circle(frame, point_prevu, 5, (255, 255, 0), -1)
                        cv2.putText(frame, "Prediction active", (10, 70),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                    else:
                        if derniere_direction_steer == 1:
                            regler_angle_servo(canal_servo, SERVO_MAX)
                            cv2.putText(frame, "Braquage MAX DROITE", (10, 70),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                        elif derniere_direction_steer == -1:
                            regler_angle_servo(canal_servo, SERVO_MIN)
                            cv2.putText(frame, "Braquage MAX GAUCHE", (10, 70),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                        else:
                            regler_angle_servo(canal_servo, SERVO_CENTRE)
                            cv2.putText(frame, "Position neutre", (10, 70),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

            # -----------------------------------------------------------------
            # (c) GESTION DE LA VITESSE (Calcul de base dynamique + Moteurs)
            # -----------------------------------------------------------------
            if len(points_detectes) > 0:
                # On applique la logique : plus le virage est grand => plus on descend vers VITESSE_MIN
                vitesse_base = calcul_vitesse_de_base(erreur_ligne)
            else:
                # On ne voit plus la ligne => on arrête les moteurs
                vitesse_base = VITESSE_ARRET

            # Mettre à jour les moteurs uniquement si la vitesse n'est pas à l'arrêt
            if vitesse_base > VITESSE_ARRET:
                v_avg, v_avd, v_arg, v_ard = mise_a_jour_moteurs(vitesse_base, erreur_ligne)
            else:
                # Si VITESSE_ARRET, arrêter tous les moteurs
                arret_urgence()
                v_avg, v_avd, v_arg, v_ard = (0.0, 0.0, 0.0, 0.0)

            sortie_video.write(frame)

            # -----------------------------------------------------------------
            # (d) ENREGISTREMENT ET LOGS
            # -----------------------------------------------------------------
            temps_ecoule = t_actuel - debut
            chaine_temps = formater_temps(temps_ecoule)

            temps_log.append(temps_ecoule)
            accel_log.append((ax, ay, az))
            gyro_log.append((gx, gy, gz))
            vitesses_log.append((vitesse_filtree_x, vitesse_filtree_y, vitesse_globale))
            yaw_log.append(yaw)
            erreur_ligne_log.append(erreur_ligne)
            servo_angle_log.append(angle_servo)
            correction_log.append(correction)  # Ajout de la correction proportionnelle
            vitesses_AVG.append(v_avg)
            vitesses_AVD.append(v_avd)
            vitesses_ARG.append(v_arg)
            vitesses_ARD.append(v_ard)

            with open(FICHIER_CSV, "a", newline="", encoding="utf-8") as f:
                ecrivain = csv.writer(f, delimiter=";")
                ecrivain.writerow([
                    chaine_temps,
                    f"{ax:.3f}", f"{ay:.3f}", f"{az:.3f}",
                    f"{gx:.3f}", f"{gy:.3f}", f"{gz:.3f}",
                    f"{vitesse_filtree_x:.3f}", f"{vitesse_filtree_y:.3f}",
                    f"{vitesse_globale:.3f}",
                    f"{yaw:.3f}",
                    f"{erreur_ligne:.2f}",
                    f"{angle_servo:.2f}",
                    f"{correction:.3f}",  # Ajout de la correction proportionnelle
                    f"{v_avg:.3f}",
                    f"{v_avd:.3f}",
                    f"{v_arg:.3f}",
                    f"{v_ard:.3f}",
                ])

    except KeyboardInterrupt:
        print("\n[INFO] Interruption Ctrl + C détectée, arrêt propre.")
    finally:
        print("[INFO] Fermeture des ressources...")
        cap.release()
        sortie_video.release()
        arret_urgence()
        pca.deinit()

        # ---------------------------------------------------------------------
        # (e) GRAPHIQUES FINAUX
        # ---------------------------------------------------------------------
        vitesse_moyenne = somme_vitesses / compteur_vitesses if compteur_vitesses else 0.0

        if len(accel_log) > 0:
            ax_vals, ay_vals, az_vals = zip(*accel_log)
            gx_vals, gy_vals, gz_vals = zip(*gyro_log)
            vx_vals, vy_vals, vtotal_vals = zip(*vitesses_log)

            # Graphe principal Accel / Gyro / Vitesse / Yaw
            plt.figure(figsize=(12, 12))

            # Accélérations
            plt.subplot(5, 1, 1)
            plt.plot(temps_log, ax_vals, label="Accel X")
            plt.plot(temps_log, ay_vals, label="Accel Y")
            plt.plot(temps_log, az_vals, label="Accel Z")
            plt.title("Accélérations (m/s²)")
            plt.legend()

            # Gyros
            plt.subplot(5, 1, 2)
            plt.plot(temps_log, gx_vals, label="Gyro X")
            plt.plot(temps_log, gy_vals, label="Gyro Y")
            plt.plot(temps_log, gz_vals, label="Gyro Z")
            plt.title("Gyroscope (°/s)")
            plt.legend()

            # Vitesses
            plt.subplot(5, 1, 3)
            plt.plot(temps_log, vx_vals, label="Vitesse X")
            plt.plot(temps_log, vy_vals, label="Vitesse Y")
            plt.plot(temps_log, vtotal_vals, label="Vitesse globale XY", linestyle="--")
            plt.title("Vitesse (m/s)")
            plt.legend()

            # Yaw
            plt.subplot(5, 1, 4)
            plt.plot(temps_log, yaw_log, label="Yaw (°)", color="purple")
            plt.title("Yaw (intégration Gyro)")
            plt.legend()

            # Erreur de ligne et Angle Servo
            plt.subplot(5, 1, 5)
            plt.plot(temps_log, erreur_ligne_log, label="Erreur Ligne (px)", color="red")
            plt.plot(temps_log, servo_angle_log, label="Angle Servo (°)", color="blue")
            plt.plot(temps_log, correction_log, label="Correction P (°)", color="green", linestyle="--")
            plt.title("Erreur de Ligne, Correction et Angle du Servo")
            plt.legend()

            plt.tight_layout()
            plt.figtext(
                0.5, 0.005,
                f"Vitesse moyenne : {vitesse_moyenne:.2f} m/s  |  Vitesse max : {vitesse_max:.2f} m/s",
                ha="center", fontsize=10,
                bbox={"facecolor": "lightgray", "alpha": 0.5, "pad": 5}
            )
            plt.savefig("mpu6050_donnees.png")
            plt.show()

            # Graphe Erreur de ligne + vitesses des roues
            plt.figure(figsize=(12, 6))

            # Subplot 1 : erreur de ligne
            plt.subplot(2, 1, 1)
            plt.plot(temps_log, erreur_ligne_log, label="Erreur Ligne (px)", color="red")
            plt.title("Évolution de l'erreur de ligne")
            plt.xlabel("Temps (s)")
            plt.ylabel("Erreur (px)")
            plt.legend()
            plt.grid(True)

            # Subplot 2 : vitesses des roues
            plt.subplot(2, 1, 2)
            plt.plot(temps_log, vitesses_AVG, label="AVG")
            plt.plot(temps_log, vitesses_AVD, label="AVD")
            plt.plot(temps_log, vitesses_ARG, label="ARG")
            plt.plot(temps_log, vitesses_ARD, label="ARD")
            plt.title("Vitesses des roues (rapport cyclique normalisé)")
            plt.xlabel("Temps (s)")
            plt.ylabel("Vitesse normalisée")
            plt.legend()
            plt.grid(True)

            plt.tight_layout()
            plt.savefig("erreur_ligne_et_moteurs.png")
            plt.show()

        print("[INFO] Données enregistrées dans", FICHIER_CSV)
        print(f"[INFO] Vitesse moyenne : {vitesse_moyenne:.2f} m/s | Vitesse max : {vitesse_max:.2f} m/s")
        print("[INFO] Fin du programme.")


# =============================================================================
# =                               EXÉCUTION                                    =
# =============================================================================
if __name__ == "__main__":
    main()
