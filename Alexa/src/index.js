// 
/**
 * Particle VFD, (OIL PUMP ON/OFF) , get motor speed (Last mod June 4 2017) V1
 *
 */
 var APP_ID = "amzn1.ask.skill.xxxxxxxxx"; //replace with "amzn1.echo-sdk-ams.app.[your-unique-value-here]";

/**
 * The AlexaSkill prototype and helper functions
 */

var http = require('https');
var AlexaSkill = require('./AlexaSkill');

/*
 *
 * Particle is a child of AlexaSkill.
 *
 */
var Particle = function () {
    AlexaSkill.call(this, APP_ID);
};

// Extend AlexaSkill
Particle.prototype = Object.create(AlexaSkill.prototype);
Particle.prototype.constructor = Particle;

Particle.prototype.eventHandlers.onSessionStarted = function (sessionStartedRequest, session) {
    console.log("Particle onSessionStarted requestId: " + sessionStartedRequest.requestId + ", sessionId: " + session.sessionId);
};

Particle.prototype.eventHandlers.onLaunch = function (launchRequest, session, response) {
    console.log("Particle onLaunch requestId: " + launchRequest.requestId + ", sessionId: " + session.sessionId);
    //var speechOutput = "Welcome to the Particle Demo, you can ask me what is the temperature or humidity. You can also tell me to turn on Red or Green light.";
    var speechOutput = "Welcome to the Control Techniques  VFD Demo, you can ask me what is the drive  temperature or  what is motor pump  speed. You can also tell me to turn on or off  oil Pump .";
    response.ask(speechOutput);
};

Particle.prototype.eventHandlers.onSessionEnded = function (sessionEndedRequest, session) {
    console.log("Particle onSessionEnded requestId: " + sessionEndedRequest.requestId + ", sessionId: " + session.sessionId);
};

Particle.prototype.intentHandlers = {
    // register custom intent handlers
    ParticleIntent: function (intent, session, response) {
		var vfdSlot = intent.slots.vfd;
		var motorSlot = intent.slots.motor;
		var onoffSlot = intent.slots.onoff;

		var vfd = vfdSlot ? intent.slots.vfd.value : "";
		var motor = motorSlot ? intent.slots.motor.value : "";
		var onoff = onoffSlot ? intent.slots.onoff.value : "off";

		var speakText = "";

		console.log("Vfd = " + vfd);
		console.log("Motor = " + motor);
		console.log("OnOff = " + onoff);

		var op = "";
		var pin = "";
		var vfdcommand = "";   //varaliable holding vfd command

		// Replace these with action device id and access token
		
		var deviceid = "xxxxxxxxx";     // Tati 2 Modbus
		var accessToken = "xxxxxxxxxx"; // Tati 2 Modbus

		var sparkHst = "api.particle.io";

		console.log("Host = " + sparkHst);

		// Check slots and call appropriate Particle Functions
		if(vfd == "temperature"){
			speakText = "Stack Temperature is 69°";

			op = "Stack-Temp";  // was gettmp
		}
		else if(vfd == "speed"){
			speakText = "Motor Speed is 500 rpm";

			op = "Motor-Speed";  // was gethmd
		}
		else if(motor == "oil pump"){
			pin = "D2";
		}
		else if(motor == "water pump"){
			pin = "D6";
		}

		// User is asking for temperature/pressure
		if(op.length > 0){
			var sparkPath = "/v1/devices/" + deviceid + "/" + op;

			console.log("Path = " + sparkPath);

			makeParticleRequest(sparkHst, sparkPath, "", accessToken, function(resp){
				var json = JSON.parse(resp);

				console.log(vfd + ": " + json.return_value);

				response.tellWithCard(vfd + " is " + json.return_value + ((vfd == "temperature") ? "°" : "rpm"), "Particle", "Particle!");
			});
		}
		// User is asking to turn on/off lights
		else if(pin.length > 0){
			if(onoff == "on"){
				//pinvalue = "HIGH";
				vfdcommand = "ON";
			}
			else{
				//pinvalue = "LOW";
				vfdcommand = "OFF";
			}

			//var sparkPath = "/v1/devices/" + deviceid + "/ctrlled";   
			var sparkPath = "/v1/devices/" + deviceid + "/VFD-ONOFF";

			console.log("Path = " + sparkPath);

		//	var args = pin + "," + pinvalue;
			var args =  vfdcommand;   // for my test I just pass the value On or off to the particle funtion VFD-ONOFF Since that function have only one argument

			makeParticleRequest(sparkHst, sparkPath, args, accessToken, function(resp){
				var json = JSON.parse(resp);

				console.log(" Last updated VFD  Temperature is: " + json.return_value);

				response.tellWithCard("OK, " + motor + " motor turned " + onoff, "Particle", "Particle!");
				response.ask("Continue?");
			});
		}
		else{
			response.tell("Sorry, smart cookie I could not understand what you said");
		}
    },
    HelpIntent: function (intent, session, response) {
       // response.ask("You can ask me what is the temperature or humidity. You can also tell me to turn on Red or Green light!");
        response.ask("You can ask me what is the VFD  temperature or motor speed. You can also tell me to turn on or off    oil  pump .");
      
    }
};

// Create the handler that responds to the Alexa Request.
exports.handler = function (event, context) {
    // Create an instance of the Particle skill.
    var particleSkill = new Particle();
    particleSkill.execute(event, context);
};

function makeParticleRequest(hname, urlPath, args, accessToken, callback){
	// Particle API parameters
	var options = {
		hostname: hname,
		port: 443,
		path: urlPath,
		method: 'POST',
		headers: {
			'Content-Type': 'application/x-www-form-urlencoded',
			'Accept': '*.*'
		}
	}

	var postData = "access_token=" + accessToken + "&" + "args=" + args;

	console.log("Post Data: " + postData);

	// Call Particle API
	var req = http.request(options, function(res) {
		console.log('STATUS: ' + res.statusCode);
		console.log('HEADERS: ' + JSON.stringify(res.headers));

		var body = "";

		res.setEncoding('utf8');
		res.on('data', function (chunk) {
			console.log('BODY: ' + chunk);

			body += chunk;
		});

		res.on('end', function () {
            callback(body);
        });
	});

	req.on('error', function(e) {
		console.log('problem with request: ' + e.message);
	});

	// write data to request body
	req.write(postData);
	req.end();
}
