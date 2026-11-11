@echo off
wsl -d Debian -u root -- chroot /var/tmp/nomad_humble_20261108 /bin/bash /workspace/nomad_ros_test.sh
pause
