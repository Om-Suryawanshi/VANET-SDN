#!/bin/bash

# Usage: ./generate_sumo.sh <speed_in_m_s>
SPEED=$1
if [ -z "$SPEED" ]; then
    echo "Usage: ./generate_sumo.sh <speed_in_m_s>"
    exit 1
fi

mkdir -p mobility
echo "Generating Grid for Speed: $SPEED m/s..."

# 1. Generate Grid
netgenerate --grid --grid.x-number=6 --grid.y-number=6 --grid.length=200 --output-file grid.net.xml --no-turnarounds

# 2. Define Vehicle Type
cat > vtype.add.xml <<EOF
<additional>
    <vType id="car" maxSpeed="$SPEED" accel="2.0" decel="4.5"/>
</additional>
EOF

# 3. Generate Long Trips (Fix for high PDR at high speeds)
# --intermediate 10 : Forces cars to visit 10 random points before stopping.
# This ensures they keep moving for the full 300s simulation.
python3 $SUMO_HOME/tools/randomTrips.py \
    -n grid.net.xml \
    -e 300 \
    -p 0.5 \
    --trip-attributes='type="car"' \
    --intermediate 10 \
    -o trips.xml

# 4. Process Routes
duarouter -n grid.net.xml --route-files trips.xml --additional-files vtype.add.xml -o routes.rou.xml --ignore-errors

# 5. Config
cat > vanet.sumocfg <<EOF
<configuration>
    <input>
        <net-file value="grid.net.xml"/>
        <route-files value="routes.rou.xml"/>
    </input>
    <time>
        <begin value="0"/>
        <end value="300"/>
    </time>
</configuration>
EOF

# 6. Run SUMO
sumo -c vanet.sumocfg --fcd-output fcd.xml --max-num-vehicles 50

# 7. Convert to NS-2 Trace
python3 $SUMO_HOME/tools/traceExporter.py \
    --fcd-input fcd.xml \
    --ns2mobility-output mobility_$SPEED.tcl

mv mobility_$SPEED.tcl scratch/mobility/
echo "Success! Trace saved to scratch/mobility/mobility_$SPEED.tcl"
