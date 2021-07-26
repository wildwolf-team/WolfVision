#!/bin/bash

echo "Please enter your root passwd:"
read PASSWORD

if ! [ -f /tmp/watch_dog.sh ]; then
    touch /tmp/watch_dog.sh
fi

cat > /tmp/watch_dog.sh <<-EOF
#!/bin.bash

fail_cnt=0

while true
do
  if [ -z \$(ps r | grep WolfVision) ]; then
    echo "WolfVision not running \$fail_cnt"
    fail_cnt=\$(expr \$fail_cnt + 1)
    sleep 1
  fi
  if [ \$fail_cnt -gt 30 ]; then
    echo "Restarting system..."
    echo $PASSWORD | sudo -S reboot
  fi
done
EOF

chmod +x /tmp/watch_dog.sh

gnome-terminal --command "bash /tmp/watch_dog.sh"

exit
