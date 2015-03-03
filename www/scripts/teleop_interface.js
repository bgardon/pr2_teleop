
$.get('/get_websocket_url', function(data, status) {
	var websocket_url = data;


	var ros = new ROSLIB.Ros({
	    url : websocket_url
	});


	ros.on('connection', function() {
		console.log('Connected to websocket server.');
	});
	ros.on('error', function(error) {
		console.log('Error connecting to websocket server: ', error);
	});
	ros.on('close', function() {
		console.log('Connection to websocket server closed.');
	});


	// Client for the GetArmStateAction Action Server
	var arm_state_client = new ROSLIB.ActionClient({
	    ros: ros,
	    serverName: 'get_arm_state',
	    actionName: 'pr2_teleop/GetArmStateAction'
	});

	// Client for the TuckArmsAction Action Server
	var tuck_client = new ROSLIB.ActionClient({
	    ros: ros,
	    serverName: 'tuck_arms',
	    actionName: 'pr2_common_action_msgs/TuckArmsAction'
	});

	// Client for the PointArmsAction Action Server
	var point_client = new ROSLIB.ActionClient({
	    ros: ros,
	    serverName: 'point_arms',
	    actionName: 'pr2_teleop/PointArmsAction'
	});

	// var head_controller = new ROSLIB.ActionClient({
	//     ros : ros,
	//     serverName : '/head_traj_controller/point_head_action',
	//     actionName : 'pr2_controllers_msgs/PointHeadAction' 
	// });

	// var movement_controller = new ROSLIB.Topic({
 //    	ros : ros,
 //    	name : '/base_controller/command',
 //    	messageType : 'geometry_msgs/Twist'
 //  	});

	// Disables a polymer button
	function disableBtn(btn) {
	    btn.disabled = true;
	    btn.className = "";
	}
	// Enables a polymer button
	function enableBtn(btn) {
	    btn.disabled = false;
	    btn.className = "colored";
	}

	var ArmsTucked = false;

	// Checks the current state of Rosie's arms and updates the interface to reflect this
	function updateArmState() {
	    var states = ["Unknown", "Tucked", "Untucked", "Pointed", "Pointed-Tight"]

	    var goal = new ROSLIB.Goal({
		actionClient: arm_state_client,
		goalMessage: {}
	    });

	    goal.on('result', function(result) {
			updateInterface(states[result.l_arm_state + 1], states[result.r_arm_state + 1])

			console.log('Arm State: ' 
				    + states[result.l_arm_state+1] 
				    + ', ' 
				    + states[result.r_arm_state+1]);
	    });

	    goal.send();
	}

	// Updates the interface to reflect the current state of Rosie's arms
	function updateInterface(left, right) {
	    var ToggleBtn = document.querySelector('#toggleBtn');
	    var PointBtn  = document.querySelector('#PointArmsBtn');
	    var TightCb = document.querySelector('#tightCb');

	    if(left === "Tucked" && right === "Tucked") {
			PointBtn.innerHTML = "Untuck to Point"
			disableBtn(PointBtn);
			TightCb.disabled = true;

			ToggleBtn.innerHTML = "Untuck";
			enableBtn(ToggleBtn);

			ArmsTucked = true;

	    } else if(left === "Pointed" || right === "Pointed") {
			TightCb.disabled = false;

			if(TightCb.checked) {
			    PointBtn.innerHTML = "Point Arms Tight"
			    enableBtn(PointBtn);
			} else {
			    PointBtn.innerHTML = "Pointed Arms Normal"
			    disableBtn(PointBtn);
			}

			enableBtn(ToggleBtn);

	    } else if(left === "Pointed-Tight" || right === "Pointed-Tight") {
			TightCb.disabled = false;

			if(TightCb.checked) {
			    PointBtn.innerHTML = "Pointed Arms Tight"
			    disableBtn(PointBtn);
			} else {
			    PointBtn.innerHTML = "Point Arms Normal"
			    enableBtn(PointBtn);
			}

			enableBtn(ToggleBtn);

	    } else { //if(left === "Untucked" || right === "Untucked") {
			ToggleBtn.innerHTML = "Tuck";
			enableBtn(ToggleBtn);

			PointBtn.innerHTML = "Point"
			enableBtn(PointBtn);
			TightCb.disabled = false;

			if(TightCb.checked) {
			    PointBtn.innerHTML = "Point Arms Tight"
			} else {
			    PointBtn.innerHTML = "Point Arms Normal"
			}

			ArmsTucked = false;
	    }
	}

	// linear x and y movement and angular z movement
	// var x = 0;
	// var y = 0;
	// var z = 0;

	// // Point realtive to robot base at which the head should point
	// var point_hor_angle = 0;
	// var point_vert_angle = 0;


	// function handleKey(keyCode, keyDown) {
 //      	var down = 0;
 //      	if (keyDown) {
 //      		down = 1;
 //      	}
 //      	// check which key was pressed
 //      	switch (keyCode) {
	//         case 81: //Q key : turn left
	//           	if (z != down) {
	//           		z = down;
	//           		newMoveGoal();
	//           	}
	//           	break;
	//         case 87: //W key : up
	//           	if (x != down) {
	//           		x = down;
	//           		newMoveGoal();
	//           	}
	//           	break;
	//         case 38: //up arrow : up
	//           	if (x != down) {
	//           		x = down;
	//           		newMoveGoal();
	//           	}
	//           	break;
	//         case 69: //E key : turn right
	//           	if (z != -down) {
	//           		z = -down;
	//           		newMoveGoal();
	//           	}
	//           	break;
	//         case 83: //S key : down
	//           	if (x != -down) {
	//           		x = -down;
	//           		newMoveGoal();
	//           	}
	//           	break;
	//         case 40: //down arrow : down
	//           	if (x != -down) {
	//           		x = -down;
	//           		newMoveGoal();
	//           	}
	//           	break;
	//         case 68: //D key : strafe right
	//           	if (y != -down) {
	//           		y = -down;
	//           		newMoveGoal();
	//           	}
	//           	break;
	//         case 39: //right arrow : strafe right
	//           	if (y != -down) {
	//           		y = -down;
	//           		newMoveGoal();
	//           	}
	//           	break;
	//         case 65: //A key : strafe left
	//           	if (y != down) {
	//           		y = down;
	//           		newMoveGoal();
	//           	}
	//           	break;
	//         case 37: //left arrow : strafe left
	//           	if (y != down) {
	//           		y = down;
	//           		newMoveGoal();
	//           	}
	//           	break;

	//         //Keys for head movement
	//         case 73: //I key : look up
	//           	if (keyDown) {
	//           		point_vert_angle += (Math.PI / 72);
	//           		if (point_vert_angle > Math.PI / 6) {
	//             		point_vert_angle = Math.PI / 6;
	//           		}
	//           		newLookGoal(point_hor_angle, point_vert_angle);
	//           	}
	//           	break;
	//         case 74: //J key : look left
	//           	if (keyDown) {
	//           		point_hor_angle += (Math.PI / 72);
	//           		if (point_hor_angle > Math.PI * 0.75) {
	//             		point_hor_angle = Math.PI * 0.75;
	//           		}
	//           		newLookGoal(point_hor_angle, point_vert_angle);
	//           	}
	//           	break;
	//         case 75: //K key : look down
	//          	if (keyDown) {
	//          		point_vert_angle -= (Math.PI / 72);
	//           		if (point_vert_angle < -Math.PI / 3) {
	//             		point_vert_angle = -Math.PI / 3;
	//           		}
	//           		newLookGoal(point_hor_angle, point_vert_angle);
	//           	}
	//           	break;
	//         case 76: //L key : look right
	//           	if (keyDown) {
	//           		point_hor_angle -= (Math.PI / 72);
	//           		if (point_hor_angle < -Math.PI * 0.75) {
	//             		point_hor_angle = -Math.PI * 0.75;
	//           		}
	//           		newLookGoal(point_hor_angle, point_vert_angle);
	//           	}
	//           	break;
 //   		}
	// }

	// function newMoveGoal() {
	// 	console.log(x + ", " + y + ", " + z);
	// 	// publish a movement command
	//     var twist = new ROSLIB.Message({
	//       	angular : {
	//         	x : 0,
	//         	y : 0,
	//         	z : z
	//       	},
	//       	linear : {
	//         	x : x / 2,
	//         	y : y / 2,
	//         	z : z
	//       	}
	//     });
	//     movement_controller.publish(twist);
	// }

	// setInterval(newMoveGoal, 50);

	// function newLookGoal(hor_angle, vert_angle) {
	// 	console.log(hor_angle + ", " + vert_angle);
		
	// 	var head_goal = new ROSLIB.Goal({
	//       	actionClient : head_controller,
	//       	goalMessage : {
	//         	target : {
	//           		header : {
	//             		frame_id : 'base_link'
	//           		},
	//           		point : {
	//             		x : (10 * Math.cos(hor_angle)),
	//             		y : (10 * Math.sin(hor_angle)),
	//             		z : (10 * Math.sin(vert_angle))
	//           		}
	//         	},
	//         	pointing_frame : "high_def_frame",
	//         	pointing_axis : {
	//         		x : 1,
	//         		y : 0,
	//         		z : 0,
	//         	},
	//         	min_duration : '0.5',
	//         	max_velocity : 1.0
	//      	}
	//     });

	//     head_goal.send();
	// }

	function initJoysticks() {
		var width = window.innerWidth;
		var height = window.innerHeight;

		var joystick1 = new VirtualJoystick({
		                     mouseSupport: true,
		                     stationaryBase: true,
		                     strokeStyle : 'grey',
		                     baseX: 0.2*width,
		                     baseY: 0.8*height,
		                     limitStickTravel: true,
		                     stickRadius: 50
		                  });
		joystick1.addEventListener('touchStartValidation', function(event){
		    var touch = event.changedTouches[0];
		    if( touch.pageX < window.innerWidth/2 ) {
		    	return false;
		    }
		    return true
		});

		var joystick2 = new VirtualJoystick({
		                     mouseSupport: true,
		                     stationaryBase: true,
		                     strokeStyle : 'grey',
		                     baseX: 0.8*width,
		                     baseY: 0.8*height,
		                     limitStickTravel: true,
		                     stickRadius: 50
		                  });
		joystick2.addEventListener('touchStartValidation', function(event){
		    var touch = event.changedTouches[0];
		    if( touch.pageX >= window.innerWidth/2 ) {
		    	return false;
		    }
		    return true
		});

		var debugText1 = document.getElementById("debug1");
		var debugText2 = document.getElementById("debug2");
		var debugText3 = document.getElementById("debug3");
		var debugText4 = document.getElementById("debug4");

		setInterval(control_robot, 50);

		function control_robot(){
		    debugText1.innerHTML = "Joystick X: " + joystick1.deltaX();
		    debugText2.innerHTML = "Joystick Y: " + joystick1.deltaY();
		    debugText3.innerHTML = "Joystick X: " + joystick2.deltaX();
		    debugText4.innerHTML = "Joystick Y: " + joystick2.deltaY();
		    if(joystick1.up()) {
		    	console.log("JS1: up");
		    	x = 1;
		    } else {
		    	x = 0;
		    }
		    if(joystick1.left()) {
		    	console.log("JS1: left");
		    	y = 1;
		    } else {
		    	y = 0;
		    }
		    if(joystick1.down()) {
		    	console.log("JS1: down");
		    	x = -1;
		    } else {
		    	x = 0;
		    }
		    if(joystick1.right()) {
		    	console.log("JS1: right");
		    	y = -1;
		    } else {
		    	y = 0;
		    }
		    if(joystick2.up()) {
		    	console.log("JS2: up");
		    }
		    if(joystick2.left()) {
		    	console.log("JS2: left");
		    }
		    if(joystick2.down()) {
		    	console.log("JS2: down");
		    }
		    if(joystick2.right()) {
		    	console.log("JS2: right");
		    }
		}
	}

	// Start the camera feeds
	function init() {
	    var parser = document.createElement('a');
	    parser.href = websocket_url;
	    var hostname  = parser.hostname;

	    console.log("touchscreen is", VirtualJoystick.touchScreenAvailable() ? "available" : "not available");

	    //if (VirtualJoystick.touchScreenAvailable()) {
	    	initJoysticks();
	    //} else {
		    var viewer_right = new MJPEGCANVAS.Viewer({
			divID : 'mjpeg_right',
			host : hostname,
			width : 320,
			height : 240,
			topic : '/r_forearm_cam/image_color'
		    });

		    var viewer_left = new MJPEGCANVAS.Viewer({
			divID : 'mjpeg_left',
			host : hostname,
			width : 320,
			height : 240,
			topic : '/l_forearm_cam/image_color'
		    });
		//}

	    var viewer_main = new MJPEGCANVAS.Viewer({
		divID : 'mjpeg_main',
		host : hostname,
		width : 640,
		height : 480,
		topic : '/wide_stereo/left/image_color'
	    });

	    updateArmState();

	    // assignEventHandlers();
	}

	window.onload = init;


	// Tucks or untucks Rosie's arms based on her current arm state
	function toggleArms() {
	    // Interface elements
	    var ToggleBtn = document.querySelector('#toggleBtn');
	    var PointBtn  = document.querySelector('#PointArmsBtn');
	    var TightCb = document.querySelector('#tightCb');
	    var UntuckingMsg = document.querySelector('#untucked');
	    var TuckingMsg   = document.querySelector('#tucked');

	    // Update interface state
	    ToggleBtn.innerHTML = "Working...";
	    disableBtn(ToggleBtn);
	    disableBtn(PointBtn);
	    TightCb.disabled = true;

	    //Execute tucking/untucking action
	    if(ArmsTucked) {

		UntuckingMsg.show();

		var goal = new ROSLIB.Goal({
		    actionClient: tuck_client,
		    goalMessage: {
			tuck_right: false,
			tuck_left: false
		    }
		});

		goal.on('result', function(result) {
		    updateArmState();
		});

		goal.send();

		ArmsTucked = false;

	    } else {

		TuckingMsg.show();

		var goal = new ROSLIB.Goal({
		    actionClient: tuck_client,
		    goalMessage: {
			tuck_right: true,
			tuck_left: true
		    }
		});

		goal.on('result', function(result) {
		    updateArmState();
		});

		goal.send();

		ArmsTucked = true;
	    }
	}


	// Points Rosie's arms near her base
	function pointArms() {
	    // Interface elements
	    var PointArmsBtn = document.querySelector('#PointArmsBtn');
	    var TightCb = document.querySelector('#tightCb');
	    var NormalMsg  = document.querySelector('#pointed');
	    var TightMsg  = document.querySelector('#pointed_t');

	    // Update interface state
	    PointArmsBtn.innerHTML = "Working...";
	    disableBtn(PointArmsBtn);

	    if(TightCb.checked) {
		TightMsg.show()
	    } else {
		NormalMsg.show()
	    }

	    // Execute pointing action
	    var goal = new ROSLIB.Goal({
		actionClient: point_client,
		goalMessage: {
		    point_right: true,
		    point_left: true,
		    tight: TightCb.checked
		}
	    });

	    goal.on('result', function(result) {
		updateArmState();
	    });

	    goal.send();
	}

	// function assignEventHandlers() {
	//     // Keyboard movement and looking
	//  //    var body = document.getElementsByTagName('body')[0];
	// 	// body.addEventListener('keydown', function(e) {
	// 	//   	handleKey(e.keyCode, true);
	// 	// }, false);
	// 	// body.addEventListener('keyup', function(e) {
	// 	//   	handleKey(e.keyCode, false);
	// 	// }, false);

	// 	//Getting around
	// 	document.getElementById("MoveForward").addEventListener('mousedown', function(e) {
	// 	    handleKey(87, true);
	// 	}, false);
	// 	document.getElementById("MoveForward").addEventListener('mouseup', function(e) {
	// 	    handleKey(87, false);
	// 	}, false);

	// 	document.getElementById("MoveBack").addEventListener('mousedown', function(e) {
	// 	    handleKey(83, true);
	// 	}, false);
	// 	document.getElementById("MoveBack").addEventListener('mouseup', function(e) {
	// 	    handleKey(83, false);
	// 	}, false);

	// 	document.getElementById("MoveLeft").addEventListener('mousedown', function(e) {
	// 	    handleKey(65, true);
	// 	}, false);
	// 	document.getElementById("MoveLeft").addEventListener('mouseup', function(e) {
	// 	    handleKey(65, false);
	// 	}, false);

	// 	document.getElementById("MoveRight").addEventListener('mousedown', function(e) {
	// 	    handleKey(68, true);
	// 	}, false);
	// 	document.getElementById("MoveRight").addEventListener('mouseup', function(e) {
	// 	    handleKey(68, false);
	// 	}, false);

	// 	// Looking around
	// 	document.getElementById("LookUp").addEventListener('mousedown', function(e) {
	// 	    handleKey(73, true);
	// 	}, false);
	// 	document.getElementById("LookUp").addEventListener('mouseup', function(e) {
	// 	    handleKey(73, false);
	// 	}, false);

	// 	document.getElementById("LookDown").addEventListener('mousedown', function(e) {
	// 	    handleKey(75, true);
	// 	}, false);
	// 	document.getElementById("LookDown").addEventListener('mouseup', function(e) {
	// 	    handleKey(75, false);
	// 	}, false);

	// 	document.getElementById("LookLeft").addEventListener('mousedown', function(e) {
	// 	    handleKey(74, true);
	// 	}, false);
	// 	document.getElementById("LookLeft").addEventListener('mouseup', function(e) {
	// 	    handleKey(74, false);
	// 	}, false);

	// 	document.getElementById("LookRight").addEventListener('mousedown', function(e) {
	// 	    handleKey(76, true);
	// 	}, false);
	// 	document.getElementById("LookRight").addEventListener('mouseup', function(e) {
	// 	    handleKey(76, false);
	// 	}, false);
	// }
});