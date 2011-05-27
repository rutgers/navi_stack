
topics_display = function(connection, domObj){
	widget = new RosWidget(connection, domObj)
	widget.init = function(){
		freq = this.domObj.getAttribute('update_frequency')
		if (! freq){ freq = 0.5 }//every other second
		this.delay_period = 1/parseFloat(freq)*1000
		this.timerCB()
	}	

	widget.updateList = function(topics){
		html = '<b> Current Topics : </b><br/>'
		for (var i=0; i<topics.length; i+=1) {
			html += widget.renderTopic(topics[i])
		}
		widget.domObj.innerHTML = html
	}
	
	widget.timerCB = function(){
		setTimeout(widget.timerCB, widget.delay_period)
		widget.connection.callService('/rosjs/topics', '[]', widget.updateList)
	}
	
	widget.onTopicSelect = function(topic){}
	widget.renderTopic = function(topic) {
		return topic + '<br/>';
	}
	
	return widget
}
