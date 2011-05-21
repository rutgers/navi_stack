#! /usr/bin/env python

from django.http import HttpResponse
from django.template import Context, loader
from django.shortcuts import render_to_response

def test(request):
	rqt_str = request.__str__()
	return HttpResponse("Hello, world. You're at the Navi web index1.<br\> hose name is " + request.META['HTTP_HOST'])


def basic(request):
	address, port = request.META['HTTP_HOST'].split(":")
	r = render_to_response('basic.html',
		{ 'debug_enabled' :  True,
		   'HOST_ADDRESS' : address,
		   'WS_PORT' 	  :  '9090'
		})
	return r;
	
		 
def rosbridge(request, widget, topic):
	address, port = request.META['HTTP_HOST'].split(":")
	r = render_to_response('rosbridge.html',
		{ 'debug_enabled' :  False,
		   'HOST_ADDRESS' : address,
		   'WS_PORT' 	  :  '9090',
		   'widget_name'  :  widget,
		   'widget_topic' :  '/'+topic,
		})
	return r;
	
	return HttpResponse("Hello, world. You requested widget %s  subscribed to topic %s"%(widget, topic))
	
	
