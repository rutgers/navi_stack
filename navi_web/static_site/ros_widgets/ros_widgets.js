//list of widgets being used on the page
ros.widgets = []

//Search through the html and find the ros_widget divs
//set up each div with its widget
ros.setupWidgets = function(){
    
    var widget = null;
    var allHTMLTags=document.getElementsByTagName("*");
    
    for (i=0; i<allHTMLTags.length; i++) {
		var domobj = allHTMLTags[i];  
		this.registerWidget(domObj)
	}
	
    for(var i=0; i<this.widgets.length; i++) {
	this.widgets[i].init();
    }
    
    var urlprefix = this.urlprefix;
}

ros.registerWidget = function(domObj){
	var objtype = domobj.getAttribute("ros_widget");
	if(objtype) {
	    var clss = window[objtype];
	    if(clss) {
		widget = new clss(domobj, this);
		this.widgets.push(widget);
	    }
	}
}
