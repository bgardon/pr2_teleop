var currentEnv = 'robomackerel'

// Connection to ROS - Uses port 9090 by default
var ros = new ROSLIB.Ros({
    url : 'ws://' + currentEnv + '.cs.washington.edu:9090'
});


ros.on('error', function(error) {
    console.log(error);
});


// Client for the GetArmStateAction Action Server
var arm_state_client = new ROSLIB.ActionClient({
    ros: ros,
    serverName: 'get_arm_state',
    actionName: 'teleop_web_app/GetArmStateAction'
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
    actionName: 'teleop_web_app/PointArmsAction'
});


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


// Start the camera feeds
function init() {
    var viewer_right = new MJPEGCANVAS.Viewer({
	divID : 'mjpeg_right',
	host : currentEnv + '.cs.washington.edu',
	width : 320,
	height : 240,
	topic : '/r_forearm_cam/image_color'
    });

    var viewer_left = new MJPEGCANVAS.Viewer({
	divID : 'mjpeg_left',
	host : currentEnv + '.cs.washington.edu',
	width : 320,
	height : 240,
	topic : '/l_forearm_cam/image_color'
    });

    var viewer_main = new MJPEGCANVAS.Viewer({
	divID : 'mjpeg_main',
	host : currentEnv + '.cs.washington.edu',
	width : 640,
	height : 480,
	topic : '/wide_stereo/left/image_color'
    });


    // Initialize the driving teleop.
    var teleop = new KEYBOARDTELEOP.Teleop({
	ros : ros,
	topic : '/base_controller/command'
    });

    updateArmState();
}


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