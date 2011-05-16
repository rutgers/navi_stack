
topic_echo = function(connection, domObj){
	widget = new RosWidget(connection, domObj)	
	widget.handler = function(msg){
		widget.domObj.innerHTML = JSON.stringify(msg, null , 4)
	}
	return widget
}
