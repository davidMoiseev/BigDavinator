teamNumberPrefix="    \"teamNumber\": "
teamNumber=$(grep $teamNumberPrefix .wpilib/wpilib_preferences.json)
teamNumber=${teamNumber#"$teamNumberPrefix"}
teamNumberDigits=${#teamNumber}

first="0"
last="0"
if [ "$teamNumberDigits" -eq "4" ] || [ "$teamNumberDigits" -eq "3" ]
then
    i=$((teamNumberDigits-2))
    first=${teamNumber:0:i}
    last=${teamNumber:i:2}
fi
if [ "$teamNumberDigits" -eq "2" ] || [ "$teamNumberDigits" -eq "1" ]
then
    last=$teamNumber
fi

address="admin@10.$first.$last.2"

scp admin@172.22.11.2:/home/lvuser/logs/* ./Logs/
read -p "Press any key to continue..." -n1 -s