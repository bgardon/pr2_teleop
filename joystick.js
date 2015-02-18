function init() {
    var body = document.getElementsByTagName('body')[0];
	body.addEventListener('keydown', function(e) {
	  	handleKey(e.keyCode, true);
	}, false);
	body.addEventListener('keyup', function(e) {
	  	handleKey(e.keyCode, false);
	}, false);
}

window.onload = init;
