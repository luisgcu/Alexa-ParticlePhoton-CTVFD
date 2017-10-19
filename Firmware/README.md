# Particle Photon  CT VFD Modbus Master. 

The project basically have been created as draft that allow Control techniques VDF  drives M200, M300, M400, M600  fitted with Modbus RTU 485 interface, to be connected to a Particle -Photon  using   RS485 driver. The sample program provided in this project allow to read several VFD Modbus registers at a configurable polling time. Every time the VFD Register are  polled the data is  send to particle cloud in a json format, and also sent to a  [MQTT broker](http://mqtt.org/faq) aswell. Two particle function have been programmed allowing to Start/Stop VFD motor , get motor speed, and get VDF temperature.
The Project also inlclude a DHT11 Humidity and Temperature sensor, basicaly the sensor is located on the Photon- Modbus PCB, the data colected from the sensor is  sent in json format to particle cloud and to[MQTT broker](http://mqtt.org/faq) if you have one. 




## Getting Started

To get the projet running you need following:
1-Having the Photon hardwaired to the RS485 driver as specified on the modbus master eagle project.
2-Had previously setup  your Particle photon account and 

### Prerequisites

What things you need to install the software and how to install them

```
Give examples
```

### Installing

A step by step series of examples that tell you have to get a development env running

Say what the step will be

```
Give the example
```

And repeat

```
until finished
```

End with an example of getting some data out of the system or using it for a little demo

## Running the tests

Explain how to run the automated tests for this system

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

### And coding style tests

Explain what these tests test and why

```
Give an example
```

## Deployment

Add additional notes about how to deploy this on a live system

## Built With

* [Dropwizard](http://www.dropwizard.io/1.0.2/docs/) - The web framework used
* [Maven](https://maven.apache.org/) - Dependency Management
* [ROME](https://rometools.github.io/rome/) - Used to generate RSS Feeds

## Contributing

Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags). 

## Authors

* **Angel L ** - *Initial work* - [PurpleBooth](https://github.com/PurpleBooth)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Hat tip to anyone who's code was used
* Inspiration
* etc

