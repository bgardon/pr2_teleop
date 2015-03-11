var KeyboardInterface = (function (baseController, headController) {
  
	var baseController_;
  var headController_;

	var KeyboardInterface = function (baseController, headController) {
    baseController_ = baseController;
    headController_ = headController;

    var body = document.getElementsByTagName('body')[0];
    
    body.addEventListener('keydown', function(e) {
        handleKey(e.keyCode, true);
    }, false);
    body.addEventListener('keyup', function(e) {
        handleKey(e.keyCode, false);
    }, false);
	};

  var HandleKey = function (keyCode, keyDown) {
    // check which key was pressed
    switch (keyCode) {
      case 81: //Q key : turn left
        keyDown ? baseController_.TurnLeft() : baseController_.StopTurn();
        break;
      case 87: //W key : up
        keyDown ? baseController_.MoveForward() : baseController_.StopFrontBack();
        break;
      case 38: //up arrow : up
        keyDown ? baseController_.MoveForward() : baseController_.StopFrontBack();
        break;
      case 69: //E key : turn right
        keyDown ? baseController_.TurnRight() : baseController_.StopTurn();
        break;
      case 83: //S key : down
        keyDown ? baseController_.MoveBackward() : baseController_.StopFrontBack();
        break;
      case 40: //down arrow : down
        keyDown ? baseController_.MoveBackward() : baseController_.StopFrontBack();
        break;
      case 68: //D key : strafe right
        keyDown ? baseController_.MoveRight() : baseController_.StopLeftRight();
        break;
      case 39: //right arrow : strafe right
        keyDown ? baseController_.MoveRight() : baseController_.StopLeftRight();
        break;
      case 65: //A key : strafe left
        keyDown ? baseController_.MoveLeft() : baseController_.StopLeftRight();
        break;
      case 37: //left arrow : strafe left
        keyDown ? baseController_.MoveLeft() : baseController_.StopLeftRight();
        break;

      //Keys for head movement
      case 73: //I key : look up
        if (keyDown) {
          headController_.LookUp();
        }
        break;
      case 74: //J key : look left
        if (keyDown) {
          headController_.LookLeft();
        }
        break;
      case 75: //K key : look down
        if (keyDown) {
          headController_.LookDown();
        }
        break;
      case 76: //L key : look right
        if (keyDown) {
          headController_.LookRight();
        }
        break;
    }
  }

	KeyboardInterface.prototype = {
		constructor: KeyboardInterface,
	};

	return KeyboardInterface;
})();