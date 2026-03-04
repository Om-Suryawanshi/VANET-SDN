#!/usr/bin/env bash
set -e

if [ -z "$SUMO_HOME" ]; then
    echo "Error: SUMO_HOME environment variable is not set."
    exit 1
fi

SCENARIO=$1
if [[ ! "$SCENARIO" =~ ^(grid|highway|city|all)$ ]]; then
    echo "Usage: ./generate_sumo.sh <grid|highway|city|all>"
    exit 1
fi

SPEEDS=(5 10 15 20)
NUM_VEHICLES=40
SIM_TIME=300

# Added "extra_args" to handle the differences between city/grid and highway
generate_traces() {
    local net_file=$1
    local extra_args=$2

    for speed in "${SPEEDS[@]}"; do
        echo "  -> Speed: ${speed} m/s"
        
        # 1. Define Vehicle Type
        cat > vtype.add.xml <<EOF
<additional>
    <vType id="car" maxSpeed="${speed}" accel="2.0" decel="4.5"/>
</additional>
EOF

        # 2. Generate Trips using the specific scenario arguments
        python3 $SUMO_HOME/tools/randomTrips.py \
            -n $net_file \
            -e $SIM_TIME \
            -p 0.5 \
            --trip-attributes='type="car"' \
            $extra_args \
            -o trips.xml

        # 3. Process Routes
        duarouter -n $net_file --route-files trips.xml --additional-files vtype.add.xml -o routes.rou.xml --ignore-errors --no-warnings

        # 4. Config
        cat > vanet.sumocfg <<EOF
<configuration>
    <input>
        <net-file value="$net_file"/>
        <route-files value="routes.rou.xml"/>
    </input>
    <time>
        <begin value="0"/>
        <end value="$SIM_TIME"/>
    </time>
</configuration>
EOF

        # 5. Run SUMO
        sumo -c vanet.sumocfg --fcd-output fcd.xml --max-num-vehicles $NUM_VEHICLES > /dev/null

        # 6. Convert to NS-2 Trace
        python3 $SUMO_HOME/tools/traceExporter.py \
            --fcd-input fcd.xml \
            --ns2mobility-output mobility_${speed}.tcl
    done
}

# ==========================================
# 1. GENERATE GRID SCENARIO
# ==========================================
if [ "$SCENARIO" == "grid" ] || [ "$SCENARIO" == "all" ]; then
    echo "[*] Generating Grid Scenario..."
    mkdir -p grid && cd grid
    netgenerate --grid --grid.x-number=6 --grid.y-number=6 --grid.length=200 --output-file grid.net.xml --no-turnarounds
    generate_traces grid.net.xml "--intermediate 10"
    cd ..
fi

# ==========================================
# 2. GENERATE HIGHWAY SCENARIO (6-LANE, 7km)
# ==========================================
if [ "$SCENARIO" == "highway" ] || [ "$SCENARIO" == "all" ]; then
    echo "[*] Generating 6-Lane Highway Scenario..."
    mkdir -p highway && cd highway
    
    # Extended to 7km so fast cars don't exit the simulation early
    cat > highway.nod.xml <<EOF
<nodes>
    <node id="node1" x="0.0" y="0.0"/>
    <node id="node2" x="3000.0" y="0.0"/>
</nodes>
EOF

    cat > highway.edg.xml <<EOF
<edges>
    <edge id="edge1to2" from="node1" to="node2" numLanes="3" speed="30.0"/>
    <edge id="edge2to1" from="node2" to="node1" numLanes="3" speed="30.0"/>
</edges>
EOF

    netconvert --node-files highway.nod.xml --edge-files highway.edg.xml -o highway.net.xml --no-warnings
    # Use --allow-fringe and force a long minimum distance instead of intermediate stops
    generate_traces highway.net.xml "--allow-fringe --min-distance 2000"
    cd ..
fi

# ==========================================
# 3. GENERATE CITY SCENARIO (BHUBANESWAR)
# ==========================================
if [ "$SCENARIO" == "city" ] || [ "$SCENARIO" == "all" ]; then
    echo "[*] Generating City Scenario (Bhubaneswar)..."
    mkdir -p city && cd city
    echo "  -> Downloading OSM data..."
    wget -qO bhubaneswar.osm "https://overpass-api.de/api/map?bbox=85.8225,20.2675,85.8325,20.2775"
    netconvert --osm-files bhubaneswar.osm -o city.net.xml --geometry.remove --ramps.guess --junctions.join --tls.guess-signals --no-warnings
    generate_traces city.net.xml "--intermediate 10"
    cd ..
fi

echo "[✓] Scenario generation complete!"