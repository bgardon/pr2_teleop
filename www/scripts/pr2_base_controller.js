var PR2BaseController = function(opts) {
	this._ros = opts.ros;
	this._topic = new ROSLIB.Topic({
    	ros : this._ros,
    	name : '/base_controller/command',
    	messageType : 'geometry_msgs/Twist'
  	});
  	this._x = 0;
  	this._y = 0;
  	this._z = 0;
}

function newMoveGoal() {
	// publish a movement command
    var twist = new ROSLIB.Message({
      	angular : {
        	x : 0,
        	y : 0,
        	z : z
      	},
      	linear : {
        	x : x / 2,
        	y : y / 2,
        	z : z
      	}
    });
    movement_controller.publish(twist);
}