scp admin@10.1.18.2:/home/lvuser/logs/* /c/Users/Jakob/Logs/
ssh admin@10.1.18.2 'rm /home/lvuser/logs/*'
read -p "Press any key to continue..." -n1 -s