var JoystickInterface = (function () {

	var baseController_;
  var headController_;

  var joystick1_;
  var joystick2_;

  var headLocked_ = false;

	var JoystickInterface = function (baseController, headController) {
    baseController_ = baseController;
    headController_ = headController;

    console.log(baseController_);

    var width = window.innerWidth;
    var height = window.innerHeight;

    joystick1_ = new VirtualJoystick({
                         mouseSupport: true,
                         stationaryBase: true,
                         strokeStyle : 'grey',
                         baseX: 0.2*width,
                         baseY: 0.8*height,
                         limitStickTravel: true,
                         stickRadius: 50
                      });
    joystick1_.addEventListener('touchStartValidation', function(event){
        var touch = event.changedTouches[0];
        if( touch.pageX < window.innerWidth/2 ) {
          return false;
        }
        return true
    });

    joystick2_ = new VirtualJoystick({
                         mouseSupport: true,
                         stationaryBase: true,
                         strokeStyle : 'grey',
                         baseX: 0.8*width,
                         baseY: 0.8*height,
                         limitStickTravel: true,
                         stickRadius: 50
                      });
    joystick2_.addEventListener('touchStartValidation', function(event){
        var touch = event.changedTouches[0];
        if( touch.pageX >= window.innerWidth/2 ) {
          return false;
        }
        return true
    });
    setInterval(ControlRobot, 50);
	};

  var ControlRobot = function () {
        if (joystick1_.up()) {
          baseController_.MoveForward();
          console.log("joy-up");
        } else if (joystick1_.down()) {
          baseController_.MoveBackward();
        } else {
          baseController_.StopFrontBack();
        }

        if (joystick1_.left()) {
          baseController_.MoveLeft();
        } else if (joystick1_.right()) {
          baseController_.MoveRight();
        } else {
          baseController_.StopLeftRight();
        }

        
        if(joystick2_.up()) {
          headController_.LookUp();
        }
        if(joystick2_.left()) {
          headLocked_ ? baseController_.TurnLeft() : headController_.LookLeft();
        }
        if(joystick2_.down()) {
          headController_.LookDown();
        }
        if(joystick2_.right()) {
          headLocked_ ? baseController_.TurnLeft() : headController_.LookLeft();
        }
    }

	JoystickInterface.prototype = {
		constructor: JoystickInterface,
	};

	return JoystickInterface;
})();