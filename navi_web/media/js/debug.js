function fixedHeightDisplay (id_name, height){
		this.height = height
		this._cline =0
		this.text = ""
		this.div_id = id_name;
		this.append = function(msg) {
			if (this._cline >= this.height){
				this._cline =0;
				this.text = ""
			}
			this.text += msg + "<br/>";
			ele = document.getElementById(this.div_id)
			if (ele)
			{ele.innerHTML = this.text
			this._cline += 1;
		}
		};
	}
	
var console = new fixedHeightDisplay('console', 300)

log = function(msg) { console.append(msg)};

$(document).ready(function(){
	
	//build console div
	var console_div  = document.createElement('div')
	console_div.id = 'console'
	document.body.appendChild(console_div)
	log('Console Successfully Created')
});
