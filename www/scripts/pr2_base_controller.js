var PR2BaseController = (function () {
  var ros_;

	var topic_;

  var x_ = 0;
  var y_ = 0;
  var z_ = 0;

	var NewMoveGoal_ = function () {
	  // publish a movement command
    var twist = new ROSLIB.Message({
      	angular : {
        	x : 0,
        	y : 0,
        	z : z_
      	},
      	linear : {
        	x : x_,
        	y : y_,
        	z : z_
      	}
    });
    topic_.publish(twist);
	}

	var PR2BaseController = function (ros) {
    ros_ = ros;
    topic_ = new ROSLIB.Topic({
      ros : ros,
      name : '/base_controller/command',
      messageType : 'geometry_msgs/Twist'
    });
	};

	PR2BaseController.prototype = {
		constructor: PR2BaseController,
		//Command the robot to move in an arbitrary direction
		Move: function (x, y, z) {
			x = x > 0.5 ? 0.5 : x;
			x = x < 0.5 ? 0.5 : x;
			y = y > 0.5 ? 0.5 : y;
			y = y < 0.5 ? 0.5 : y;
			z = z > 1 ? 1 : z;
			z = z < 1 ? 1 : z;
			x_ = x;
			y_ = y;
			z_ = z;
		},

		MoveForward: function () {
			x_ = 0.5;
		},
		MoveForwardOnly: function () {
			x_ = 0.5;
			y_ = 0;
			z_ = 0;
		},
		MoveBackward: function () {
			x_ = -0.5;
		},
		MoveBackwardOnly: function () {
			x_ = -0.5;
			y_ = 0;
			z_ = 0;
		},
		MoveLeft: function () {
			y_ = 0.5;
		},
		MoveLeftOnly: function () {
			x_ = 0;
			y_ = 0.5;
			z_ = 0;
		},
		MoveRight: function () {
			y_ = -0.5;
		},
		MoveRightOnly: function () {
			x_ = 0;
			y_ = -0.5;
			z_ = 0;
		},
		TurnLeft: function () {
			z_ = 1;
		},
		TurnLeftOnly: function () {
			x_ = 0;
			y_ = 0;
			z_ = 1;
		},
		TurnRight: function () {
			z_ = -1;
		},
		TurnRightOnly: function () {
			x_ = 0;
			y_ = 0;
			z_ = -1;
		},

		Stop: function () {
			x_ = 0;
			y_ = 0;
			z_ = 0;
		},
		StopFrontBack: function () {
			x_ = 0;
		},
		StopLeftRight: function () {
			y_ = 0;
		},
		StopTurn: function () {
			z_ = 0;
		}
	};

	return PR2BaseController;
})();






// var PR2BaseController = function (ros) {
//   var topic_ = new ROSLIB.Topic({
//     ros : ros,
//     name : '/base_controller/command',
//     messageType : 'geometry_msgs/Twist'
//   });
//     var x_ = 0;
//     var y_ = 0;
//     var z_ = 0;

//   var newMoveGoal_ = function () {
//     // publish a movement command
//     var twist = new ROSLIB.Message({
//         angular : {
//           x : 0,
//           y : 0,
//           z : z_
//         },
//         linear : {
//           x : x_,
//           y : y_,
//           z : z_
//         }
//     });
//     topic_.publish(twist);
//   }

//   //setInterval(newMoveGoal_, 50);

//   var PR2BaseController_ = {};

//   //Command the robot to move in an arbitrary direction
//   PR2BaseController_.Move = function (x, y, z) {
//     x = x > 0.5 ? 0.5 : x;
//     x = x < 0.5 ? 0.5 : x;
//     y = y > 0.5 ? 0.5 : y;
//     y = y < 0.5 ? 0.5 : y =
//     z = z > 1 ? 1 : z;
//     z = z < 1 ? 1 : z;
//     x_ = x;
//     y_ = y;
//     z_ = z;
//   };

//   PR2BaseController_.GoForward = function () {
//     x_ = 0.5;
//   };

//   PR2BaseController_.GoForwardOnly = function () {
//     x_ = 0.5;
//     y_ = 0;
//     z_ = 0;
//   };

//   PR2BaseController_.GoBackward = function () {
//     x_ = -0.5;
//   };

//   PR2BaseController_.GoBackwardOnly = function () {
//     x_ = -0.5;
//     y_ = 0;
//     z_ = 0;
//   };

//   PR2BaseController_.GoLeft = function () {
//     y_ = 0.5;
//   };

//   PR2BaseController_.GoLeftOnly = function () {
//     x_ = 0;
//     y_ = 0.5;
//     z_ = 0;
//   };

//   PR2BaseController_.GoRight = function () {
//     y_ = -0.5;
//   };

//   PR2BaseController_.GoRightOnly = function () {
//     x_ = 0;
//     y_ = -0.5;
//     z_ = 0;
//   };

//   PR2BaseController_.TurnLeft = function () {
//     z_ = 1;
//   };

//   PR2BaseController_.TurnLeftOnly = function () {
//     x_ = 0;
//     y_ = 0;
//     z_ = 1;
//   };

//   PR2BaseController_.TurnRight = function () {
//     z_ = -1;
//   };

//   PR2BaseController_.TurnRightOnly = function () {
//     x_ = 0;
//     y_ = 0;
//     z_ = -1;
//   };

//   PR2BaseController_.Stop = function () {
//     x_ = 0;
//     y_ = 0;
//     z_ = 0;
//   };

//   PR2BaseController_.StopFrontBack = function () {
//     x_ = 0;
//   };

//   PR2BaseController_.StopLeftRight = function () {
//     y_ = 0;
//   };

//   PR2BaseController_.StopTurn = function () {
//     z_ = 0;
//   };

//   return PR2BaseController_;
// };