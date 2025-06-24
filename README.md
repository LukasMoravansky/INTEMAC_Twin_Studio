# INTEMAC Twin.Studio – A Digital Twin Platform for Robotic Systems
*A modular digital twin platform for simulation, visualization, and control of robotic systems.*

![TwinStudio_Teaser_1](https://github.com/user-attachments/assets/9df478e6-fca0-4ec0-810e-bb5f30ab52df)

⚠️ **Note:**  After downloading the asset package from [this link](https://drive.google.com/drive/folders/1sh54Zob04AiZA5fPUpmBnJRQQ3PGjY0-?usp=drive_link), make sure its contents are placed in the *Assets/Objects* folder to ensure the 3D scene loads correctly in Twin.Studio.

**Used software**
```bash
Unity 2022.3.45f1
```

## Project Description:

Developed as part of internal research at INTEMAC by Roman Parák and Lukáš Moravanský, Twin.Studio leverages Unity3D with HDRP to create a high-fidelity, visually realistic simulation environment. The platform supports dynamic motion simulation with precise speed and acceleration control, both forward and inverse kinematics, and script-based motion execution. It includes advanced features such as planned path visualization, realistic pick-and-place operations with gripper and gravity simulation, and real-time robot mirroring via UR-RTDE.
 
Twin.Studio has been primarily tested with the collaborative robot Universal Robots UR10e and is fully compatible with the entire UR robot family. The platform is designed with extensibility in mind, with ongoing plans to support additional collaborative and industrial robots, as well as a broader range of hardware components including PLCs, sensors, and other automation devices.
 
Future development will focus on integrating advanced motion planning algorithms and enabling automatic code generation for deployment on physical robotic systems.

### Key Features
- **Dynamic Motion Simulation** – The robot's trajectory is planned with precise control over speed and acceleration.
- **Forward Kinematics Execution** – Movement is performed based on specified target coordinates using forward kinematics.
- **Inverse Kinematics Execution** – Movement is performed based on specified target coordinates using inverse kinematics.
- **Script-Based Motion Simulation** – The robot follows a program composed of supported commands.
- **Planned Path Visualization** – Displays the precomputed trajectory for better analysis and debugging.
- **Pick and Place Application** – Includes gripper functionality with gravity simulation when releasing objects.
- **PLC Integration via OPC UA** – The digital twin can be controlled using Siemens PLCs over OPC UA, with extendability to other PLC manufacturers such as Beckhoff, B&R, etc.
- **Real-Time Robot Mirroring** – The platform can reflect the actual robot’s position via TCP/IP (requires connection to a physical UR10e).

**Used equipment**
```bash
UR10e, Robotiq 2F-85
```
### Model enviroment description:
**I. Model of real robot** \
**II. Ghost robot**  \
**III. Gripper** (Optonal)\
**IV. Viewpoint** \
**V. Plane with objects** (Optional)

![Described_env](https://github.com/user-attachments/assets/2921d02c-9b04-42e1-a946-6582a55ebffe)

## User controls:
### Forward Control Script
*Assets/Scripts/Robot_Control/Forward_Control.cs*\
It is placed on the ghost model of the robot. It also serves as the primary interface for the user to control the robot. 
- **Target_Time** : Time in which the desired movement is to be executed.
- **Execute** : Executes **MoveJ** movement of robot (I.) to the position set on the ghost robot (II.) 
- **Execute_Linear** : Executes **MoveL** linear movement of robot (I.) to the position set on the viewpoint (IV.) 
- **Go_Home** : Sends the robot (I.) to the home position. 
- **Execute_Script** : Robot (I.) starts to execute the script written in the file *Assets/Scripts/Robot_Control/Robot.cs*
- **Show_Prediction** : Renders the currently running motion/script.
- **Tool** : Parameter modifying robot kinematics (TCP distance).
- **Set_Tool** : Sets parameter value in **Tool**.



## Contact Info
Roman Parak: [parak@intemac.cz](mailto:parak@intemac.cz)

Lukas Moravansky: [moravansky@intemac.cz](mailto:moravansky@intemac.cz)

## License
[MIT](https://choosealicense.com/licenses/mit/)

