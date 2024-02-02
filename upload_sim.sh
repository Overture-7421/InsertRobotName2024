#!/bin/sh

FILES=$(ls -1a | grep -iEv '^build$|^ctre_sim$|^.git$|^.gradle$|^.gitignore$|\.$')
echo $FILES
scp -r $FILES overture@photonvision.local:~/Documents/Gits/InsertRobotName2024
