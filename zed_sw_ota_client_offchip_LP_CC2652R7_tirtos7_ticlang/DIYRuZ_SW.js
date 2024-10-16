const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const constants = require('zigbee-herdsman-converters/lib/constants');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const ota = require('zigbee-herdsman-converters/lib/ota');
//const extend = require('zigbee-herdsman-converters/lib/extend');
const e = exposes.presets;
const ea = exposes.access;
//const {calibrateAndPrecisionRoundOptions} = require('zigbee-herdsman-converters/lib/utils');
const {postfixWithEndpointName} = require('zigbee-herdsman-converters/lib/utils');


const bind = async (endpoint, target, clusters) => {
    for (const cluster of clusters) {
        await endpoint.bind(cluster, target);
    }
};

const ACCESS_STATE = 0b001, ACCESS_WRITE = 0b010, ACCESS_READ = 0b100;

const device = {
        zigbeeModel: ['DIYRuZ_SW'],
        model: 'DIYRuZ_SW',
        vendor: 'DIYRuZ',
        description: '[SW](https://github.com/koptserg/temperature)',
        supports: 'genOnOff',
        ota: ota.zigbeeOTA,
        fromZigbee: [
            fz.command_toggle,
            fz.illuminance,
            fz.temperature,
            fz.pressure,
            fz.humidity,
        ],
        toZigbee: [
            tz.factory_reset,
        ],
        meta: {
            configureKey: 1,
            multiEndpoint: true,
        },
        configure: async (device, coordinatorEndpoint) => {
            const eightEndpoint = device.getEndpoint(8);
            await bind(eightEndpoint, coordinatorEndpoint, [
//                'genOnOff',
//                'genTime',
                'msIlluminanceMeasurement',
                'msTemperatureMeasurement',
                'msPressureMeasurement',
                'msRelativeHumidity',
            ]);
        },
        exposes: [
            exposes.enum('action', ACCESS_STATE | ACCESS_WRITE, ['toggle']).withDescription('Triggered action (e.g. a button click)'), 
            exposes.numeric('illuminance', ACCESS_STATE).withLabel('Illuminance (lux)').withUnit('lx').withDescription('Measured illuminance in lux'),
            exposes.numeric('temperature_8', ACCESS_STATE).withLabel('Temperature (C)').withUnit('C').withDescription('Measured temperature in C'),
            exposes.numeric('pressure', ACCESS_STATE).withLabel('Pressure (hPa)').withUnit('hPa').withDescription('Measured pressure in hPa'),
            exposes.numeric('humidity_8', ACCESS_STATE).withLabel('Humidity (%)').withUnit('%').withDescription('Measured humidity in %'),
        ],
};

module.exports = device;