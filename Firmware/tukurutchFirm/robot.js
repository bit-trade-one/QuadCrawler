(function(ext) {
	var device = null;
	var checkDevName = false;
	var devName = "";

	var CMD_RESET = 0x7F;

	ext.resetAll = function(){
		device.send([0xff, 0x55, 1, CMD_RESET]);
	};
	ext.runArduino = function(){
		responseValue();
	};

	//### CUSTOMIZED ###
	var remoteKey = 0;
	var remoteX = 0;
	var remoteY = 0;
	
	var CMD_CHECKREMOTEKEY = 0x80;

	ext.checkRemoteKey = function() {
		sendPackage(CMD_CHECKREMOTEKEY);
	}
	ext.isRemoteKey = function(code){
		responseValue2(0,remoteKey==code);
	}
	ext.isARemoteKey = function(code){
		responseValue2(0,remoteKey==code);
	}
	ext.getRemoteX = function(){
		responseValue2(0,remoteX);
	}
	ext.getRemoteY = function(){
		responseValue2(0,remoteY);
	}

	//### CUSTOMIZED END ###

	function sendPackage(){
		checkDevName = false;

		var bytes = [0xff, 0x55, 0];
		for(var i=0;i<arguments.length;++i)
			bytes.push(arguments[i]);
		bytes[2] = bytes.length - 3;
		device.send(bytes);
	}

	var rtype = {
		BYTE	: 1,
		SHORT	: 2,
		LONG	: 3,
		FLOAT	: 4,
		DOUBLE	: 5,
		STRING	: 6,
	};

	var _rxBuf = [];
	var _packetLen = 3;
	function processData(bytes) {
		if(checkDevName) {
			for(var index = 0; index < bytes.length; index++) {
				var c = bytes[index];
				if(c == 0x0d) {
					updateDevName(devName);
					checkDevName = false;
				} else {
					devName += String.fromCharCode(c);
				}
			}
			return;
		}

		for(var index = 0; index < bytes.length; index++){
			var c = bytes[index];
			_rxBuf.push(c);
			switch(_rxBuf.length) {
			case 1:
				_packetLen = 3;
				if(c != 0xff) 
					_rxBuf = [];
				break;
			case 2:
				if(c != 0x55) 
					_rxBuf = [];
				break;
			case 3:
				_packetLen = 3+c;
				break;
			}

			if(_rxBuf.length >= _packetLen) {
				if(_packetLen == 3) {
					responseValue();
				} else {
					var value = 0;
					switch(_rxBuf[3]) {
					case rtype.BYTE:	value = _rxBuf[4];				break;
					case rtype.SHORT:	value = readInt(_rxBuf, 4, 2);	break;
					case rtype.LONG:	value = readInt(_rxBuf, 4, 4);	break;
					case rtype.FLOAT:	value = readFloat(_rxBuf, 4);	break;
					case rtype.DOUBLE:	value = readDouble(_rxBuf, 4);	break;
					case rtype.STRING:	value = readString(_rxBuf, 5, _rxBuf[2]-3);	break;

					//### CUSTOMIZED ###
					case CMD_CHECKREMOTEKEY:
						value = _rxBuf[4];
						remoteKey = value;
						remoteX = readInt(_rxBuf, 5, 2);
						remoteY = readInt(_rxBuf, 7, 2);
						break;
					}
					responseValue(0,value);
				}
				_rxBuf = [];
			}
		}
	}
	function readInt(arr,position,count){
		var result = 0;
		for(var i=0; i<count; ++i){
			result |= arr[position+i] << (i << 3);
		}
		if(arr[position+i-1] & 0x80) {
			result -= 1 << (i << 3);
		}
		return result;
	}
	function readFloat(arr,pos){
		var f= [arr[pos+0],arr[pos+1],arr[pos+2],arr[pos+3]];
		return parseFloat(f);
	}
	function readDouble(arr,position){
		var f= [arr[pos+0],arr[pos+1],arr[pos+2],arr[pos+3],arr[pos+4],arr[pos+5],arr[pos+6],arr[pos+7]];
		return parseFloat(f);
	}
	function readString(arr,position,len){
		var value = "";
		for(var ii=0;ii<len;ii++){
			value += String.fromCharCode(arr[ii+position]);
		}
		return value;
	}

	// Extension API interactions
	ext._deviceConnected = function(dev) {
		if (dev)
			dev.open(115200, deviceOpened);
	}

	function deviceOpened(dev) {
		device = dev;
		checkDevName = true;
		devName = "";
		device.set_receive_handler(processData);
	};

	ext._deviceRemoved = function(dev) {
		if(device != dev) return;
		device = null;
	};
	ext._getStatus = function() {
		if(!device) return {status: 1, msg: 'Disconnected'};
		return {status: 2, msg: 'Connected'};
	}
	var descriptor = {};
	ScratchExtensions.register('RemoconRobo', descriptor, ext, {type: 'serial'});
})({});
