# RoadRunner 3D Scene Design Specification for Unstructured Indian Roads
**MathWorks Autonomous Driving Hackathon — BharatDrive-AI Project**

This document specifies the exact 3D environment architecture, cross-sections, asset placement, actor kinematics, and MathWorks RoadRunner authoring workflows for 5 critical Indian road scenarios.

---

## IRC Standards & Indian Road Geometry Conventions
In Indian road engineering, the **Indian Roads Congress (IRC)** defines geometric standards:
- **IRC:73-1980**: Geometric Design Standards for Rural (Non-Urban) Highways.
- **IRC:86-1983**: Geometric Design Standards for Urban Roads in Plains.
- **IRC:67-2012**: Code of Practice for Road Signs.
- **IRC:99-2018**: Guidelines for Traffic Calming Measures (Speed Breakers & Rumble Strips).

Unlike western highway models, Indian driving surfaces feature **soft/earthen shoulders**, **lack of lane striping or degraded centerlines**, **heterogeneous vehicular dimensions**, and **permeable roadside boundaries** (open drains, encroachments, tea stalls, and livestock).

---

# Scenario 1: Village Road (Gramin Sadak)

### 1. Road Layout
- **Topology**: Undivided, single carriageway with gentle horizontal curves ($R = 80\text{--}150\text{ m}$), minor crest/sag vertical curves following natural rural topography.
- **Surface**: Worn, single-layer asphalt with raveling, rough aggregate exposure, edge crumbling, and recurrent potholes ($0.4\text{--}1.2\text{ m}$ diameter, $0.05\text{--}0.12\text{ m}$ depth).

### 2. Road Width
- **Carriageway**: $3.75\text{ m}$ single lane (bidirectional shared traffic requiring mutual yielding).
- **Shoulders**: $1.0\text{ m}$ to $1.2\text{ m}$ unpaved earthen/gravel shoulder (kaccha road edge) on each side with $0.15\text{ m}$ drop-off from asphalt edge.
- **Formation Width**: Total $6.0\text{ m}$.

### 3. Road Signs (IRC:67-2012)
- Warning Sign: "Narrow Road Ahead" (W-21) and "Unguarded Level / Rural Curve" (W-01).
- Informatory Sign: "Pradhan Mantri Gram Sadak Yojana (PMGSY)" green/white village marker board at $s = 0\text{ m}$.
- Speed Limit Sign: $30\text{ km/h}$ mandatory circular disc with red border.

### 4. Road Boundaries
- Open earthen stormwater ditches (unlined drains) $0.8\text{ m}$ wide, $0.5\text{ m}$ deep, running parallel along shoulders.
- Whitewashed stone guard stones / boundary stones ($0.3\text{ m} \times 0.3\text{ m} \times 0.6\text{ m}$) spaced every $10\text{ m}$ on curves.
- Dense wild shrubbery (*Lantana camara*), neem/banyan tree trunks directly abutting the unpaved shoulder with low overhanging canopy ($3.5\text{ m}$ clearance).

### 5. Buildings
- Vernacular rural single-story structures ($4\text{--}5\text{ m}$ setback): brick-and-mortar mud houses, tin-roof cattle sheds, and roadside thatched bamboo tea stall ("Chai Tapri").
- Low perimeter stone/mud boundary walls ($1.0\text{ m}$ height) with occasional wooden picket gates.

### 6. Moving Vehicles
- **Tractor with Trolley**: Moving at $18\text{ km/h}$ in opposite direction, laden with sugarcane (overhanging load extending $0.4\text{ m}$ beyond trolley side).
- **Two-Wheelers (Commuter Motorcycles/Hero Splendor)**: Moving at $30\text{ km/h}$, weaving between potholes and unpaved shoulder.
- **Bicycle with Milk Cans**: Moving at $10\text{ km/h}$ clinging to the left shoulder edge.

### 7. Pedestrians
- Farmers walking along shoulder in groups of 2.
- Villager carrying bundle of firewood on head walking along the edge of the asphalt.

### 8. Animals
- Stray dog resting near the warm asphalt edge, occasionally darting laterally.
- 1--2 free-ranging goats grazing along the grass verge.

### 9. Traffic Density
- **Density**: Low to moderate ($12\text{--}20\text{ vehicles/km}$), but high spatial contention because passing requires dropping ego left wheels onto the earthen shoulder.

---

# Scenario 2: Busy Market (Bazaar Street)

### 1. Road Layout
- **Topology**: Highly constrained, straight to undulating urban arterial running through a dense commercial ribbon. No lane disciplines; space is shared continuously by vehicles, vendors, and shoppers.
- **Surface**: Paver-block or patchy bituminous surface with water stagnation puddles and manhole covers elevated/depressed by $\pm 5\text{ cm}$.

### 2. Road Width
- **Nominal Carriageway**: $9.0\text{ m}$ total width.
- **Effective Drivable Width**: Dynamically throttled down to $3.5\text{--}4.5\text{ m}$ due to temporary encroachments, parked 2-wheelers, and vegetable handcarts.
- **Footpath**: Non-existent or completely occupied by storefront merchandise.

### 3. Road Signs (IRC:67-2012)
- "No Parking" (R-22) signs frequently ignored or partially obscured by shop awnings.
- "Pedestrian Crossing Ahead" (W-29).
- Commercial signboards, flex banners, and illuminated shop facades projecting into the roadway.

### 4. Road Boundaries
- Continuous commercial shop front plinths elevated $0.3\text{ m}$ above road grade.
- Covered RCC drains with iron grating; chaotic electrical poles with entangled overhead wiring bundles at $4.0\text{ m}$ height.

### 5. Buildings
- Continuous 2-to-3 story mixed-use commercial buildings (ground floor shops, upper floor residential) directly facing the street with zero setback.
- Colorful fabric awnings, tarpaulins, fruit stalls, and mobile kiosk umbrellas protruding $1.2\text{ m}$ into the street corridor.

### 6. Moving Vehicles
- **Auto-Rickshaws (3-wheelers)**: Making sudden, unheralded stops to drop passengers; performing tight U-turns across the corridor.
- **Delivery Mopeds / E-Rickshaws**: Slow-moving ($12\text{--}18\text{ km/h}$), wide battery carriage, frequently braking.
- **Motorcycles & Scooters**: Dynamic gap-filtering through pedestrian gaps at $15\text{--}25\text{ km/h}$.

### 7. Pedestrians
- High density: Shoppers stepping off shop steps directly into traffic path without checking blind spots.
- Street vendors pushing handcarts loaded with bananas/vegetables along traffic flow.
- Children darting between parked vehicles.

### 8. Animals
- Stray cows scavenging around vegetable market waste on the shoulder.
- Pigeons fluttering down into the carriageway.

### 9. Traffic Density
- **Density**: Extremely high spatial saturation ($60\text{--}90\text{ agents/100m}$), stop-and-go speed profile ($5\text{--}18\text{ km/h}$).

---

# Scenario 3: Urban Junction (Uncontrolled / Erratic Roundabout)

### 1. Road Layout
- **Topology**: 4-way unsignalized or semi-signalized intersection with an informal circular island / memorial statue ($D = 12\text{ m}$).
- **Flow Pattern**: Clockwise and counter-clockwise mixed circulating flow; aggressive weaving where approaching traffic fails to yield to circulating vehicles.

### 2. Road Width
- **Approaches**: 4 legs, each with $7.0\text{ m}$ carriageway (2 lanes undivided, nominal).
- **Circulating Carriageway**: $9.5\text{ m}$ wide continuous circular roadway without radial lane markings.

### 3. Road Signs (IRC:67-2012)
- "Compulsory Roundabout" (M-08) circular blue signboard on approach legs.
- Overhead overhead gantries with directional destinations in English, Hindi, and regional script.
- Blinkers: Amber flashing caution LED beacon mounted on central island mast.

### 4. Road Boundaries
- Curved concrete mountable kerbs ($0.15\text{ m}$ height) bordering central island.
- Corner pedestrian refuges with yellow/black painted bollards.
- Traffic police observation cabin / booth ("Traffic Umbrellas") on southwest quadrant.

### 5. Buildings
- Corporate office buildings, bank branches, petrol pump (gas station) on northwest corner with open ingress/egress cut-outs.
- Flyover piers (elevated metro pillars or bridge columns) obstructing sightlines across opposite approach quadrants.

### 6. Moving Vehicles
- **City Transit Buses (Ashok Leyland / Tata)**: Wide turning envelope swinging across multiple paths, long blind spots.
- **Sedans & Hatchbacks**: Aggressively asserting right-of-way.
- **Auto-Rickshaws**: Cutting diagonally across the roundabout to minimize travel distance.
- **Two-Wheelers**: Squeezing between buses and curbs on the inside radius.

### 7. Pedestrians
- Pedestrians crossing diagonally across approach mouths rather than at designated zebras.
- Traffic police officer on foot gesturing traffic flow with hand whistle.

### 8. Animals
- Occasional stray dogs navigating the central island landscaping.

### 9. Traffic Density
- **Density**: Heavy ($45\text{--}65\text{ PCU/min}$), severe lateral contention, critical requirement for intention-aware yield planning.

---

# Scenario 4: Highway Merge (Two-Lane Highway On-Ramp with Wrong-Way Traffic)

### 1. Road Layout
- **Topology**: 4-lane divided National Highway (NH) with a grade-separated or trumpet slip road merge.
- **Edge Case Dynamics**: Fast-moving highway cruising traffic ($70\text{--}90\text{ km/h}$) confronted by merging traffic, combined with **occasional local two-wheelers or tractors traveling wrong-way along the outer paved shoulder**.

### 2. Road Width
- **Mainline**: Dual $7.0\text{ m}$ carriageways (2 lanes each direction) separated by a $1.5\text{ m}$ concrete New Jersey barrier or planted median.
- **Paved Shoulder**: $1.5\text{ m}$ bitumen shoulder + $1.0\text{ m}$ unpaved gravel verge.
- **Slip Road (Merge Ramp)**: $4.5\text{ m}$ single-lane ramp with $120\text{ m}$ acceleration taper.

### 3. Road Signs (IRC:67-2012)
- "Merge Traffic from Left" (W-24) located $150\text{ m}$ upstream on mainline.
- Speed Limit: $80\text{ km/h}$ on mainline, $40\text{ km/h}$ on slip ramp.
- Green retro-reflective cantilever destination boards ($4.0\text{ m} \times 2.0\text{ m}$).

### 4. Road Boundaries
- Continuous W-beam steel crash barriers (metal beam guard rails per IRC:119) along the ramp outer curve.
- Median: Precast concrete New Jersey crash barrier ($0.81\text{ m}$ height) topped with anti-glare green plastic baffles.

### 5. Buildings
- Highway amenities: "Dhaba" (roadside restaurant) with open parking strip $20\text{ m}$ off the shoulder, tyre puncture repair stall, toll plaza canopy visible in distance.
- High-mast illumination poles ($30\text{ m}$ height) at ramp convergence points.

### 6. Moving Vehicles
- **Multi-Axle Heavy Trucks (16-wheelers)**: Occupying left/center lanes at $55\text{--}65\text{ km/h}$, slow to brake.
- **Express Passenger Cars**: Traveling in right lane at $80\text{--}100\text{ km/h}$.
- **Local Scooter Driving Wrong-Way**: Riding against traffic along the left paved shoulder at $25\text{ km/h}$ (common Indian rural-highway hazard).

### 7. Pedestrians
- Highway hitchhikers or long-distance bus passengers waiting by the roadside under merge lighting.
- Villagers attempting to sprint across high-speed mainline through gaps in median barrier.

### 8. Animals
- Stray cattle lingering on the grass verge between ramp and mainline.

### 9. Traffic Density
- **Mainline Density**: Moderate to High ($30\text{--}45\text{ vehicles/km}$), high kinetic energy requiring long-range radar/camera forward collision prediction and TTC evaluation.

---

# Scenario 5: Cattle Crossing (Sudden Obstruction on Undivided Road)

### 1. Road Layout
- **Topology**: Straight 2-lane rural / semi-urban highway passing between open agricultural fields and a village pond. High-contrast canopy shade alternating with direct sunlight.
- **Surface**: Standard bituminous macadam, but with mud tracks dragged onto the road by farm tractor tyres creating localized reduced-friction patches ($\mu \approx 0.45$).

### 2. Road Width
- **Carriageway**: $7.0\text{ m}$ total width (two $3.5\text{ m}$ lanes).
- **Shoulders**: $1.5\text{ m}$ earthen shoulders on both sides.
- **Total Formation Width**: $10.0\text{ m}$.

### 3. Road Signs (IRC:67-2012)
- Warning Sign: "Cattle Crossing Ahead" (W-27) triangle with bovine silhouette.
- "Accident Prone Area" black spot caution board.

### 4. Road Boundaries
- Open irrigation canal running parallel on left side ($2.0\text{ m}$ distance from shoulder).
- Banyan and mango tree line on right boundary with whitewashed trunks up to $1.5\text{ m}$ height.

### 5. Buildings
- Sparse roadside cowshed (Gaushala) and water trough set back $8.0\text{ m}$.
- Pump house structure with electrical transformer enclosure.

### 6. Moving Vehicles
- **Ego Autonomous Vehicle**: Approaching at $50\text{ km/h}$.
- **Oncoming Commercial Bus**: Approaching from opposite direction at $45\text{ km/h}$, narrowing the lateral swerve clearance corridor.
- **Trailing Motorbike**: Tailgating the ego vehicle at $45\text{ km/h}$ with insufficient headway ($1.2\text{ s}$ gap).

### 7. Pedestrians
- Herdsman (Gwala) carrying a wooden stick, walking behind cattle attempting to herd them across, shouting or waving hands.

### 8. Animals (Critical Hazard Actors)
- **Lead Cow (Stationary)**: Standing perpendicular across the ego lane center ($s = 65\text{ m}$), chewing cud, unresponsive to vehicle horn or approach.
- **Calf (Erratic Motion)**: Sudden sprint across from left shoulder towards the mother cow in the roadway.
- **Second Cow (Shoulder Verge)**: Grazing on right earthen shoulder, with potential to pivot onto roadway.

### 9. Traffic Density
- **Carriageway Contention**: Critical pinching point. Lateral escape corridor is blocked by oncoming bus, rear is tailgated by motorcycle, forward path is obstructed by non-deformable organic obstacle ($450\text{ kg}$ bovine). Demands emergency comfort-braking with partial lateral nudge.

---

# RoadRunner Step-by-Step Authoring Workflow

To construct these 5 scenarios in **MathWorks RoadRunner**:

### Step 1: Project Setup & GIS Projection
1. Launch **RoadRunner** (`RoadRunner.exe`).
2. Go to **File > New Project**. Set project directory to `Bharatdrive-AI/roadrunner_scenes/`.
3. Set **Elevation & Coordinate System**: Select `WGS 84 / UTM Zone 43N` (standard for Central/Northern India) or `Local Cartesian`.
4. Import regional satellite basemap tiles via **World > Imagery** to align ground truth curves.

### Step 2: Carriageway & Lane Cross-Section Authoring (Road Tool)
1. Select the **Road Tool** (`R`).
2. In the Road Attributes panel, configure custom lane cross-sections:
   - For *Village Road*: Create a single lane of $3.75\text{ m}$ width. Disable center yellow/white markings. Under **Lanes > Material**, choose `Asphalt_Worn_Cracked`.
   - For *Market & Urban Road*: Add $1.0\text{ m}$ shoulder lanes on both sides with `Dirt_Gravel` or `Paver_Blocks`.
3. Left-click to lay down path centerline spline control points according to target curvature.
4. Use the **Cross Section Tool** (`X`) to create lateral elevation crowns ($2.5\%$ camber slope per IRC specifications for drainage) and side earthen ditch slopes.

### Step 3: Road Calming & Pothole Surface Modeling
1. Select the **Surface Marking Tool** / **Decal Tool**.
2. Apply custom pothole decals (`pothole_asphalt_deep_01.png`) onto the drivable lane mesh.
3. Use the **Elevation Tool** with brush radius $0.6\text{ m}$ and falloff $0.2\text{ m}$ to depress the road mesh vertices vertically by $-0.08\text{ m}$ to $-0.12\text{ m}$, generating authentic 3D pothole geometry for vehicle suspension & camera depth estimation testing.
4. For speed breakers: Use **Road > Add Bump**; set shape to parabolic hump, height $0.10\text{ m}$, length $3.70\text{ m}$, with yellow/black thermoplastic chevron markings per IRC:99.

### Step 4: Prop & Boundary Assets Placement
1. Switch to the **Prop Tool** (`P`).
2. Open **Asset Library > Props**:
   - Place Indian telegraph poles, roadside boundary stones every $10\text{ m}$.
   - Place thatched roof stalls, kiosks, fruit vendor carts, and temporary barricades along the road edge.
   - For vegetation, use *Neem*, *Banyan*, and *Acacia* tree assets along road verges.
3. For traffic signs: Select **Sign Tool** (`S`). Place circular and triangular IRC:67 signboards on steel mounting posts at $1.5\text{ m}$ lateral offset from road edge, height $2.1\text{ m}$ clearance.

### Step 5: Junction & Roundabout Tool
1. For *Urban Junction*: Draw 4 connecting roads intersecting at central node.
2. Select **Junction Tool** (`J`). RoadRunner automatically synthesizes corner fillets and turning maneuvers.
3. Adjust corner radius to $12\text{ m}$ (standard Indian urban junction curb radius).
4. For the roundabout: Create a circular road node, adjust island diameter to $12\text{ m}$, and add concrete kerbs with yellow/black striping.

### Step 6: RoadRunner Scenario Actor Setup (Traffic & Pedestrians)
1. Open the **RoadRunner Scenario Editor** window (**Scenario > Open Scenario Editor**).
2. Drag and drop Actor 3D models onto lanes:
   - **Ego Vehicle**: Sedan / Electric SUV (Tata Nexon EV dimensions).
   - **Auto-Rickshaw (3-Wheeler)**: Assign aggressive lane-changing trajectory.
   - **Two-Wheeler (Motorbike/Scooter)**: Set path to weave along lane boundaries.
   - **Cattle (Cow/Buffalo)**: Place across lane center; define low-speed or stationary state.
   - **Pedestrians**: Assign crosswalk-free crossing trajectories intersecting the ego path at $1.2\text{ m/s}$.
3. In the Scenario Logic panel, define triggers:
   - *Cut-in trigger*: When Ego distance to Auto-Rickshaw $< 25\text{ m}$, Rickshaw executes lateral cut-in ($d = -1.8\text{ m}$) at $v = 8.5\text{ m/s}$.
   - *Cattle trigger*: Calf accelerates across lane when Ego is at $t = 3.5\text{ s}$ to collision.

### Step 7: Exporting for MathWorks Simulink & Automated Driving Toolbox
1. **Export ASAM OpenDRIVE (`.xodr`)**:
   - Go to **File > Export > ASAM OpenDRIVE (.xodr)**.
   - Enable: `Export Signals`, `Export Junction Elevation Grid`, `Generate Smooth Curves`.
   - Save to: `roadrunner_scenes/opendrive/scenario_<name>.xodr`.
2. **Export ASAM OpenSCENARIO (`.xosc`)**:
   - In Scenario window: **File > Export > ASAM OpenSCENARIO (.xosc)**.
   - Save actor trajectory logic for automated co-simulation.
3. **Import into MATLAB**:
   - In MATLAB command window:
     ```matlab
     scenario = drivingScenario;
     roadNetwork(scenario, 'OpenDRIVE', 'roadrunner_scenes/opendrive/scenario_1_village_road.xodr');
     plot(scenario);
     ```
   - Instantly brings the RoadRunner 3D Indian road network into MATLAB Automated Driving Toolbox for tracking, costmaps, and Simulink vehicle plant verification.
