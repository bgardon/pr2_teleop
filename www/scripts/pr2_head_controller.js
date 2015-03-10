var PR2HeadController = (function (actionClient) {

  var actionClient_ = actionClient;

	var horAngle_ = 0;
  var vertAngle_ = 0;

	function newLookGoal() {
    var head_goal = new ROSLIB.Goal({
      actionClient : actionClient_,
      goalMessage : {
        target : {
          header : {
            frame_id : 'base_link'
          },
          point : {
            x : (10 * Math.cos(horAngle_)),
            y : (10 * Math.sin(horAngle_)),
            z : (10 * Math.sin(vertAngle_))
          }
        },
        pointing_frame : "high_def_frame",
        pointing_axis : {
          x : 1,
          y : 0,
          z : 0,
        },
        min_duration : '0.5',
        max_velocity : 1.0
      }
    });

    head_goal.send();
  }

	var PR2HeadController = function () {
	};


	PR2HeadController.prototype = {
		constructor: PR2HeadController,
		//Command the robot to look in an arbitrary direction
		Look: function (horAngle, vertAngle) {
			horAngle_ = horAngle > (Math.PI * 0.75) ? (Math.PI * 0.75) : horAngle;
      horAngle_ = horAngle < (-Math.PI * 0.75) ? (-Math.PI * 0.75) : horAngle;
      vertAngle_ = vertAngle < (-Math.PI / 3) ? (-Math.PI / 3) : vertAngle;
      vertAngle_ = vertAngle > (Math.PI / 6) ? (Math.PI / 6) : vertAngle;
      newLookGoal();
		},

    LookUp: function () {
      Look(horAngle_, vertAngle_ + (Math.PI / 72));
    },
    LookDown: function () {
      Look(horAngle_, vertAngle_ - (Math.PI / 72));
    },
    LookLeft: function () {
      Look(horAngle_ + (Math.PI / 72), vertAngle_);
    },
    LookRight: function () {
      Look(horAngle_ - (Math.PI / 72), vertAngle_);
    },

    LookStraight: function () {
      Look(0, 0);
    },

	};
	return PR2HeadController;
})();