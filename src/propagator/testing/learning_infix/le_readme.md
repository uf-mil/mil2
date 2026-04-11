### i saw this, which maybe works

#!/bin/bash
# This script is run by the navigator user when the main NaviGator computer starts.
USER="navigator"

# Location of proprietary repositories
SDGPS_DIR="/home/$USER/sdgps/sdgps_current"
SONAR_DIR="/home/$USER/sonar/software"

# Command to connect to sylphase GPS/INS
SDGPS="$SDGPS_DIR/build/main"
ANTENNA_POSITION="[0.37465,-0.136525,0.587502]"
GPS_CMD="(cd $SDGPS_DIR;sudo $SDGPS sylphase-usbgpsimu2 --antenna-position '$ANTENNA_POSITION' --use-imu1 ! tracker ! kf2 --decimation 10 ! listen-solution-tcp 1234)"

i think it'd be

sudosdgps sylphase-usbgpsimu2 --antenna-position '[0,0,0]' --use-imu1 ! tracker ! kf2 --decimation 10 ! listen-solution-tcp 1234
