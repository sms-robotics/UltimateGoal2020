#!/bin/sh

if [ "$1" = "s" ]; then
  adb push TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpMode* /storage/self/primary/FIRST/java/src/org/firstinspires/ftc/teamcode/

else
  adb push TeamCode/src/main/java/org/firstinspires/ftc/teamcode/* /storage/self/primary/FIRST/java/src/org/firstinspires/ftc/teamcode/
fi
