#Teleop Web App - An RWS App#

This is a simple webapp which allows you to teleoperate Rosie using her forearm camera's to watch for obstacles and her head camera to look forward. It has been integrated with the RWS Teleoperation app for keyboard teleoperation.

This app tracks the state of Rosie's arms and provides options for changing the state of her arms including: tucking and untucking her arms, and moving her arms to point her forearm cameras near her base for navigation. When pointing Rosie's forearm cameras towards her base, her arms can either be placed in a tight (better for moving through tight passages) or a loose configuration (better for general navigation in open spaces).

To start this app, run

    roslaunch teleop_web_app app.launch

If you are running in simulation, you should first start Gazebo before launching this app.

For any questions, email: bgardon@.uw.edu