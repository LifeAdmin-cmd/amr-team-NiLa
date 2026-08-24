#!/usr/bin/env zsh

# Fange Strg+C ab und beende alle Hintergrundprozesse sauber
trap 'echo "\nBeende Prozesse..."; kill 0; exit' INT TERM

echo "=> Lade Umgebung..."
source env.bat

echo "=> Starte Gazebo Simulation im Hintergrund..."
ros2 launch robile_gazebo gazebo_4_wheel.launch.py &

echo "=> Warte 10 Sekunden, damit Gazebo hochfahren kann..."
sleep 10

echo "=> Starte Controller..."
python3 src/controller.py &

echo "=> Starte MCL Node (Localisation)..."
python3 src/localisation/mcl_node.py &

echo "\n========================================================"
echo "✅ Projekt läuft! Drücke Strg+C, um alles zu beenden."
echo "========================================================\n"

# Das Skript offen halten und auf die Hintergrundprozesse warten
wait
