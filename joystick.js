function init() {
	console.log("touchscreen is", VirtualJoystick.touchScreenAvailable() ? "available" : "not available");
	
	var joystickL	= new VirtualJoystick({
		mouseSupport: true,
		stationaryBase: true,
        baseX: 200,
        baseY: 200,
		
		container	: document.getElementById("joystick-left"),
		strokeStyle	: 'cyan',
		stickRadius	: 120		
	});
	joystickL.addEventListener('touchStartValidation', function(event){
		var touch	= event.changedTouches[0];
		if( touch.pageX < window.innerWidth/2 )	return false;
		return true
	});

	// one on the right of the screen
	var joystickR	= new VirtualJoystick({
		mouseSupport: true,
		stationaryBase: true,
        baseX: 200,
        baseY: 200,
		
		container	: document.getElementById("joystick-right"),
		strokeStyle	: 'orange',
		stickRadius	: 120		
	});
	joystickR.addEventListener('touchStartValidation', function(event){
		var touch	= event.changedTouches[0];
		if( touch.pageX >= window.innerWidth/2 )	return false;
		return true
	});




	
	// joystickL.addEventListener('touchStart', function(){
	// 	console.log('down')
	// })
	// joystickL.addEventListener('touchEnd', function(){
	// 	console.log('up')
	// })
	// joystickR.addEventListener('touchStart', function(){
	// 	console.log('down')
	// })
	// joystickR.addEventListener('touchEnd', function(){
	// 	console.log('up')
	// })

	setInterval(function(){
		var outputEl	= document.getElementById('result');
		outputEl.innerHTML	= '<b>Result:</b> '
			+ 'Left:'
			+ ' dx:'+joystickL.deltaX()
			+ ' dy:'+joystickL.deltaY()
			+ (joystickL.right()	? ' right'	: '')
			+ (joystickL.up()	? ' up'		: '')
			+ (joystickL.left()	? ' left'	: '')
			+ (joystickL.down()	? ' down' 	: '')
			+ 'Right:'
			+ ' dx:'+joystickR.deltaX()
			+ ' dy:'+joystickR.deltaY()
			+ (joystickR.right()	? ' right'	: '')
			+ (joystickR.up()	? ' up'		: '')
			+ (joystickR.left()	? ' left'	: '')
			+ (joystickR.down()	? ' down' 	: '')	
	}, 1/30 * 1000);
}

window.onload = init;
