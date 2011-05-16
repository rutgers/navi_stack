//list of widgets being used on the page
ros.widgets = []


RosWidget = function(connection, domObj){
	this.domObj = domObj;
	this.connection = connection;
	this.topic = domObj.getAttribute("topic")
	log("Widget " + domObj.id + 'Created')
	
	this.init = function(){ 
		this.connection.Subscribe(this.topic, this.handler)
	}
}


ros.Connection.prototype.Subscribe = function(topic, handler){
	try{
		this.addHandler(topic, handler);
	}
	catch (error) {
		this.log('Problem registering handler for '+ topic );
		this.log(error)
		return;
    }
    log('Registered Handler for ' + topic)
   try {
		topic_arg = '["'+topic+'",0]'
		this.callService('/rosjs/subscribe', topic_arg, function(msg){});
	} catch (error) {
		log('Problem subscribing to ' + topic);
		log(error)
	}
}

//Search through the html and find the ros_widget divs
//set up each div with its widget
ros.setupWidgets = function( connection){
    
    widget = null;
    var allHTMLTags=document.getElementsByTagName("*");
    for (var i=0; i<allHTMLTags.length; i++) {
		var domObj = allHTMLTags[i];  
		this.registerWidget(connection, domObj)
	} 
	for( var i =0; i< this.widgets.length; i++){
		log("initializing " + this.widgets[i].topic)
		this.widgets[i].init()
	}
}

ros.registerWidget = function(connection, domObj){
	objtype = domObj.getAttribute("ros_widget");
	if(objtype) {
		log("Registering " + domObj.id + "of type " + objtype)
	    var clss = window[objtype];
	    if(clss) {
		var widget = new clss(connection, domObj);
		this.widgets.push(widget);
	    }
	}
}
