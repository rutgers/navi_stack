var ROS = function(url) {
	this.handlers = new Array();

	this.socket = new WebSocket(url);
	var ths = this;
	this.socket.onmessage = function(e) {
		var call = ''; 
		try {
			eval('call = ' + e.data);
		} catch(err) {
			return;
		}
    for (var i in ths.handlers[call.receiver]) {
      var handler = ths.handlers[call.receiver][i]
      handler(call.msg);
    }
	}

	var myArray = new Array(41, -5, 20, 0, 3, -11, 37);
	this.magicServices = new Array('/rosjs/topics','/rosjs/services','/rosjs/typeStringFromTopic','/rosjs/typeStringFromService','/rosjs/msgClassFromTypeString','/rosjs/reqClassFromTypeString','/rosjs/rspClassFromTypeString','/rosjs/classFromTopic','/rosjs/classesFromService');
}
ROS.prototype.callService = function(service, json, callback) {
  this.handlers[service] = new Array(callback);
	var call = '{"receiver":"' + service + '"';
	call += ',"msg":' + json + '}';
	this.socket.send(call);
}
ROS.prototype.publish = function(topic, typeStr, json) {
	typeStr.replace(/^\//,'');
	var call = '{"receiver":"' + topic + '"';
	call += ',"msg":' + json;
	call += ',"type":"' + typeStr + '"}';
	this.socket.send(call);
}
ROS.prototype.addHandler = function(topic, func) {
	if (!(topic in this.handlers)) {
		this.handlers[topic] = new Array();
	}
	this.handlers[topic].push(func);
}
ROS.prototype.setOnError = function(func) {
	this.socket.onerror = func;
}
ROS.prototype.setOnClose = function(func) {
	this.socket.onclose = func;
}
ROS.prototype.setOnOpen = function(func) {
	this.socket.onopen = func;
}
