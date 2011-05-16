image_view = function(connection, domObj){
	widget = new RosWidget(connection, domObj)	
	
	widget.init = function(){
		
		log('init img view')
		this.pix_width = domObj.getAttribute('pixel_width')
		if (! this.pix_width) {
			this.pix_width = 320
		}
		this.pix_height = domObj.getAttribute('pixel_height')
		if (! this.pix_height){
			this.pix_height= 240
		}
	
		// Setup Div element for image_view
		var canvas = document.createElement('canvas');
		this.canvas = canvas
		this.domObj.appendChild(this.canvas)
		this.canvas.width = this.pix_width
		this.canvas.height= this.pix_height

		var ctx = this.canvas.getContext('2d');
				
		//Add in some default image to show where the image_view is				
		ctx.fillStyle = "rgb(200,200,200)";
		ctx.fillRect (0, 0, this.pix_width, this.pix_height);	
		
		
		this.connection.Subscribe(this.topic, this.handler)
    }

	widget.handler=function(msg){
		var ctx = widget.canvas.getContext('2d');
		img = new Image();
		img.onload = function(){
			ctx.clearRect(0,0,this.pix_width,this.pix_height);
			ctx.save();
			ctx.scale(2,2);
			ctx.drawImage(img,0,0);
			ctx.restore();
			};		
		img.src = msg.uri;
	}
	return widget
}
