$files = (Get-ChildItem -Path .).Name | Select-String -Pattern '^build$|^ctre_sim$|^.git$|^.gradle$|^gradlew$|^.gitignore$|\.$' -NotMatch
echo $files
scp -r $files overture@photonvision.local:~/Documents/Gits/InsertRobotName2024