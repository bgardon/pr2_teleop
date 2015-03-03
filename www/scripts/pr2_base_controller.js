var PR2BaseController = (function (opts) {
	var topic_ = new ROSLIB.Topic({
    	ros : opts.ros,
    	name : '/base_controller/command',
    	messageType : 'geometry_msgs/Twist'
  	});
  	var x_ = 0;
  	var y_ = 0;
  	var z_ = 0;

  	var newMoveGoal_ = function () {
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

	setInterval(newMoveGoal_, 50);

  	var PR2BaseController = function () {
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

  		GoForward: function () {
  			x_ = 0.5;
  		},
  		GoForwardOnly: function () {
  			x_ = 0.5;
  			y_ = 0;
  			z_ = 0;
  		},
  		GoBackward: function () {
  			x_ = -0.5;
  		},
  		GoBackwardOnly: function () {
  			x_ = -0.5;
  			y_ = 0;
  			z_ = 0;
  		},
  		GoLeft: function () {
  			y_ = 0.5;
  		},
  		GoLeftOnly: function () {
  			x_ = 0;
  			y_ = 0.5;
  			z_ = 0;
  		},
  		GoRight: function () {
  			y_ = -0.5;
  		},
  		GoRightOnly: function () {
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