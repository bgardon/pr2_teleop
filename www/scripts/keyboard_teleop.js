var KEYBOARDTELEOP = KEYBOARDTELEOP || {
  REVISION : '2'
};

/**
 * Manages connection to the server and all interactions with ROS.
 *
 * Emits the following events:
 *   * 'change' - emitted with a change in speed occurs
 *
 * @constructor
 * @param options - possible keys include:
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic (optional) - the Twist topic to publish to, like '/cmd_vel'
 *   * throttle (optional) - a constant throttle for the speed
 */
KEYBOARDTELEOP.Teleop = function(options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/cmd_vel';
  // permanent throttle
  var throttle = options.throttle || 1.0;

  // linear x and y movement and angular z movement
  var x = 0;
  var y = 0;
  var z = 0;

  // point in fromt of the robot at which the head should point
  // distances in meters and coordinates are realtive to robot base
  var point_hor_angle = 0;
  var point_vert_angle = 0;

  var cmdVel = new ROSLIB.Topic({
    ros : ros,
    name : topic,
    messageType : 'geometry_msgs/Twist'
  });

  var head_controller = new ROSLIB.ActionClient({
    ros : ros,
    serverName : '/head_traj_controller/point_head_action',
    actionName : 'pr2_controllers_msgs::PointHeadGoal' 
  });

  // sends connection messages to the console
  ros.on('connection', function() {
    console.log('Connected to websocket server.');
  });
  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
  });
  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });

  // sets up a key listener on the page used for keyboard teleoperation
  var handleKey = function(keyCode, keyDown) {
    // used to check for changes in speed
    var oldX = x;
    var oldY = y;
    var oldZ = z;

    // used to check for changes in head goal
    var old_point_hor_angle = point_hor_angle;
    var old_point_vert_angle = point_vert_angle;

    var speed = 0;

    // throttle the speed by the slider and throttle constant
    if (keyDown === true) {
     speed = throttle;
    }
    
    // check which key was pressed
    switch (keyCode) {
      case 81: //Q key : turn left
        z = 1 * speed;
        break;
      case 87: //W key : up
        x = 0.5 * speed;
        break;
      case 38: //up arrow : up
        x = 0.5 * speed;
        break;
      case 69: //E key : turn right
        z = -1 * speed;
        break;
      case 83: //S key : down
        x = -0.5 * speed;
        break;
      case 40: //down arrow : down
        x = -0.5 * speed;
        break;
      case 68: //D key : strafe right
        y = -0.5 * speed;
        break;
      case 39: //right arrow : strafe right
        y = -0.5 * speed;
        break;
      case 65: //A key : strafe left
        y = 0.5 * speed;
        break;
      case 37: //left arrow : strafe left
        y = 0.5 * speed;
        break;

      //Keys for head movement
      case 73: //I key : look up
        point_vert_angle += (Math.PI / 36);
        if (point_vert_angle > Math.PI / 6) {
          point_vert_angle = Math.PI / 6;
        }
        break;
      case 74: //J key : look left
        point_hor_angle += (Math.PI / 36);
        if (point_hor_angle > Math.PI * 0.75) {
          point_hor_angle = Math.PI * 0.75;
        }
        break;
      case 75: //K key : look down
        point_vert_angle -= (Math.PI / 36);
        if (point_vert_angle < -Math.PI / 3) {
          point_vert_angle = -Math.PI / 3;
        }
        break;
      case 76: //L key : look right
        point_hor_angle -= (Math.PI / 36);
        if (point_hor_angle < -Math.PI * 0.75) {
          point_hor_angle = -Math.PI * 0.75;
        }
        break;
    }

    // publish a movement command
    var twist = new ROSLIB.Message({
      angular : {
        x : 0,
        y : 0,
        z : z
      },
      linear : {
        x : x,
        y : y,
        z : z
      }
    });
    cmdVel.publish(twist);

    //build a new head_goal for the head to point at
    var head_goal = new ROSLIB.Goal({
      actionClient : head_controller,
      goalMessage : {
        target : {
          header : {
            frame_id : 'base_link'
          },
          point : {
            x : (10 * Math.cos(point_hor_angle)),
            y : (10 * Math.sin(point_hor_angle)),
            z : (10 * Math.sin(point_vert_angle))
          }
        },
        pointing_frame : "high_def_frame",
        min_duration : '0.5',
        max_velocity : 1.0
      }
    });

    // check for changes in movement commands
    if (oldX !== x || oldY !== y || oldZ !== z) {
      that.emit('change', twist);
    }

    // check for changes in head goals
    if (old_point_hor_angle !== point_hor_angle 
      || old_point_vert_angle !== point_vert_angle) {
      head_goal.send();
    }
  };

  // handle the key
  var body = document.getElementsByTagName('body')[0];
  body.addEventListener('keydown', function(e) {
    handleKey(e.keyCode, true);
  }, false);
  body.addEventListener('keyup', function(e) {
    handleKey(e.keyCode, false);
  }, false);
};

KEYBOARDTELEOP.Teleop.prototype.__proto__ = EventEmitter2.prototype;