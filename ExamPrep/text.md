# 🧠 DRAKE CHEAT SHEET – 6.4212 Robotic Manipulation

---

## ⚙️ SYSTEM BASICS

A **System** = inputs + outputs + (optional) internal state.

```python
class MySystem(LeafSystem):
    def __init__(self):
        super().__init__()
        self.DeclareContinuousState(2)            # [x, ẋ]
        self._u_port = self.DeclareVectorInputPort("u", 1)
        self.DeclareVectorOutputPort("x", 1, self.CalcOutput)

    def DoCalcTimeDerivatives(self, context, derivatives):
        x = context.get_continuous_state_vector().GetAtIndex(0)
        xdot = context.get_continuous_state_vector().GetAtIndex(1)
        u = self._u_port.Eval(context)[0]
        derivatives.get_mutable_vector().SetFromVector([xdot, u - x])

    def CalcOutput(self, context, output):
        x = context.get_continuous_state_vector().GetAtIndex(0)
        output.SetAtIndex(0, x)
🔹 Core subclasses
Class	Purpose
LeafSystem	Basic building block (define own dynamics)
Diagram	Collection of connected systems
VectorSystem	Simplified vector-only subclass
HardwareStation	Interface to real robot or simulated hardware

🧩 DIAGRAMS
A Diagram connects systems (like blocks in Simulink):

python
Copy code
builder = DiagramBuilder()

plant = builder.AddSystem(InvertedPendulum())
controller = builder.AddSystem(PDController())
logger = builder.AddSystem(SignalLogger(1))

builder.Connect(plant.get_output_port(), controller.get_input_port())
builder.Connect(controller.get_output_port(), plant.get_input_port())
builder.Connect(plant.get_output_port(), logger.get_input_port())

diagram = builder.Build()
Method	Function
AddSystem()	Add a block
Connect(out_port, in_port)	Wire blocks together
Build()	Finalize into a single system

🧠 CONTEXT
A Context stores the current values for:

Time

Continuous state (x)

Inputs

Parameters

python
Copy code
context = system.CreateDefaultContext()
context.SetTime(0.0)
context.SetContinuousState([θ₀, θ̇₀])
x = context.get_continuous_state_vector().CopyToVector()
Think of it as the snapshot of the system at one time.

⏩ SIMULATOR
Advances the system through time.

python
Copy code
sim = Simulator(diagram)
context = sim.get_mutable_context()
context.SetContinuousState([0.1, 0.0])   # initial θ, θ̇
sim.AdvanceTo(10.0)
Common commands
Command	Purpose
AdvanceTo(t)	Run forward in time
get_context()	Access current context
ResetStatistics()	Reset metrics before new run

📈 LOGGER
Records outputs over time.

python
Copy code
logger = builder.AddSystem(SignalLogger(1))
builder.Connect(pendulum.get_output_port(), logger.get_input_port())
After sim:

python
Copy code
t = logger.sample_times()
y = logger.data()
👀 VISUALIZATION
🟩 MeshcatVisualizer
3D visualizer (browser-based):

python
Copy code
from pydrake.geometry import MeshcatVisualizer, StartMeshcat
meshcat = StartMeshcat()
MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
🟦 PlanarSceneGraphVisualizer
2D visualizer (for pendulum, cart-pole, etc.)

python
Copy code
from pydrake.systems.planar_scenegraph_visualizer import PlanarSceneGraphVisualizer
PlanarSceneGraphVisualizer.AddToBuilder(builder, scene_graph)
🦾 MULTIBODYPLANT + SCENEGRAPH
Used for robot models and physics.

python
Copy code
from pydrake.multibody.plant import MultibodyPlant, AddMultibodyPlantSceneGraph
from pydrake.multibody.parsing import Parser

plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
Parser(plant).AddModelFromFile("iiwa14.urdf")
plant.Finalize()
Roles:
Component	Purpose
MultibodyPlant	Simulates robot dynamics
SceneGraph	Handles geometry and collisions
Parser	Loads robot models from files

🧍 HARDWARESTATION
Used in manipulation examples (simulated or real robots):

Wraps a MultibodyPlant

Defines robot joints, sensors, grippers, and frames

Often built from a .yaml config

Example:

python
Copy code
station = builder.AddSystem(HardwareStation("iiwa_station.yaml"))
builder.Connect(station.GetOutputPort("iiwa_position_command"), ...)
Ports include:

"iiwa.position_measured"

"iiwa.torque_commanded"

"camera_rgb_image"

⚖️ SYSTEM EQUATIONS
For a typical LeafSystem with state x, input u, and output y:

𝑥
˙
=
𝑓
(
𝑥
,
𝑢
,
𝑡
)
x
˙
 =f(x,u,t)
𝑦
=
𝑔
(
𝑥
,
𝑢
,
𝑡
)
y=g(x,u,t)
Example: Pendulum
𝜃
˙
=
𝜔
θ
˙
 =ω
𝜔
˙
=
−
𝑔
𝐿
sin
⁡
(
𝜃
)
+
𝜏
𝑚
𝐿
2
ω
˙
 =−
L
g
​
 sin(θ)+
mL
2

τ
​

🔄 CONNECTION SUMMARY
lua
Copy code
+-----------------------------+
|        DIAGRAM BUILDER      |
|-----------------------------|
|   +---------+               |
|   | Controller|             |
|   +---------+               |
|       | torque              |
|       v                     |
|   +----------+              |
|   | Pendulum |---θ--------> Logger
|   +----------+              |
+-----------------------------+
💡 QUICK COMMANDS REFERENCE
Function	Description
DeclareContinuousState(n)	Define continuous states
DeclareVectorInputPort(name, size)	Create input
DeclareVectorOutputPort(name, size, calc=func)	Create output
Eval(context)	Read input value
SetAtIndex(i, val)	Write output value
GetAtIndex(i)	Read element of state vector
DiagramBuilder()	Build system network
Simulator(diagram)	Create simulator
AdvanceTo(T)	Run simulation
SignalLogger()	Log data
MeshcatVisualizer	View simulation in 3D
HardwareStation	Interface with robot hardware

🧭 MENTAL MODEL SUMMARY
Concept	What it means	Analogy
System	Any unit with inputs, outputs, and dynamics	Function block
Context	Snapshot of a system’s state at a time	Simulation frame
Diagram	Collection of connected systems	Circuit diagram
Simulator	Advances time and integrates dynamics	Time engine
Logger	Stores outputs	Data recorder
Visualizer	Renders motion	Viewer window
HardwareStation	Real/sim robot interface	ROS node equivalent
