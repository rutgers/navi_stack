function image_view (div_id, pix_width, pix_height){
	this.lock = false;
	this.div_id = div_id
	this.pix_width = pix_width
	this.pix_height = pix_height
	
	// Setup Div element for image_view
	this.div = document.getElementById(div_id)
	this.canvas = document.createElement('canvas');
	this.div.appendChild(this.canvas)

	this.canvas.width = pix_width
	this.canvas.height= pix_height
	
	var ctx = this.canvas.getContext('2d');
			
	//Add in some default image to show where the image_view is				
	ctx.fillStyle = "rgb(200,200,200)";
	ctx.fillRect (0, 0, pix_width, pix_height);
	this.lock=false
	
	this.canvas.onLoad = function(){
		this.lock= false
	}			
    this.handler=function(msg){
		if (this.lock) return;
				
		this.lock = true;
		img = new Image();
		img.onload = function(){
		ctx.clearRect(0,0,this.pix_width,this.pix_height);
		ctx.save();
		ctx.scale(5,5);
		ctx.drawImage(img,0,0);
		ctx.restore();
			};
		//#log(msg.uri)
		
		this.lock=false;
		img.src = msg.uri;

    };

    
};
