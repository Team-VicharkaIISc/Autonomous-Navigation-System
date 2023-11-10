**Autonomous Navigation System for Project URC by Team Vicharaka.**

The System is highly inspired by the works of Control Barrier Functions. Considering the challenges of the competition, we have prioritise on the safety of the rover putting its limit to the velocity. 
The Navigation Stack will consist of:
1. Base Controller(Controller.py)
2. Barrier Certificate
3. Cost Map
   Given a state and Position, Base Controller will return an array of linear velocity and angular velocity for the rover to reach a given position provided its state. Base controller consists of single integrator, unicycle_pose_controller and hybrid controller functions.
   ![Controller](https://github.com/Team-VicharkaIISc/Autonomous-Navigation-System/assets/63643403/b61d8536-1739-424c-8e4a-909b3a2ecb94)
Team Vicharaka is still working on the Barrier Certificate.

