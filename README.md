🏎️ Projet F1 – Véhicule autonome avec Raspberry Pi
📌 Présentation
Ce projet consiste en la conception et la réalisation d’une voiture type Formule 1 automatisée, entièrement contrôlée par un Raspberry Pi 4 et un ensemble de capteurs et modules électroniques.
L’objectif est de développer un véhicule capable de suivre une trajectoire, d’adapter sa vitesse et de réagir à son environnement en temps réel, grâce à un système embarqué optimisé.

🔧 Matériel utilisé :
-Raspberry Pi 4
-Contrôleur PCA9685 pour la gestion des ESC et des servomoteurs
-4 moteurs brushless (900KV) avec ESC
-1 servomoteur type MG90S pour la direction
-Capteur MPU6050 (accéléromètre + gyroscope)
-Caméra Raspberry Pi pour la vision embarquée
-Batterie LiPo adaptée à la puissance des moteurs
-Châssis F1 imprimé et modifié selon les besoins du projet

💡 Fonctionnalités principales :
-Contrôle des moteurs et du servomoteur via PWM
-Capture et traitement vidéo en temps réel avec OpenCV
-Détection et suivi de ligne verte sur piste
-Enregistrement vidéo automatique avec nom de fichier aléatoire
-Mesure et analyse des données d’inclinaison et d’accélération
-Gestion de la vitesse pour éviter les pertes d’adhérence

📂 Accès aux fichiers
Les fichiers sources (scripts Python, modèles 3D, ressources, etc.) sont trop volumineux pour être directement stockés sur GitHub.
Ils sont donc disponibles ici :
🔗 [Lien Google Drive du projet](https://drive.google.com/drive/folders/1nREtiOmOfcD-Jraa-cMsiBzIbDaQVjIX)



📸 Aperçu du projet
![PXL_20250518_085829850](https://github.com/user-attachments/assets/e0245448-27b5-4635-8292-e2bbc4c0118c)
![PXL_20250518_085801578](https://github.com/user-attachments/assets/4ba6caaa-c8bd-4048-8d09-aca85b206233)

🚀 Objectifs futurs :
-Amélioration de la précision du suivi de trajectoire
-Implémentation d’un contrôle PID pour optimiser la vitesse et la stabilité
-Optimisation du code pour un traitement vidéo plus rapide
