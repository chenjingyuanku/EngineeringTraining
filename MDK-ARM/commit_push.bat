echo Y | del EngineeringTraining
cd ..
del /s /f /q *.bak
git add -A
git commit -m %1%
git push -u origin master
