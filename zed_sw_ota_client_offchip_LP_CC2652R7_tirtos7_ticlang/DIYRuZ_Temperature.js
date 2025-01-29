//const {
//    fromZigbeeConverters,
//    toZigbeeConverters,
//    exposes
//} = require('zigbee-herdsman-converters');

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
        zigbeeModel: ['DIYRuZ_Temperature'],
        model: 'DIYRuZ_Temperature',
        vendor: 'DIYRuZ',
        description: '[Motion sensor](https://github.com/koptserg/temperature)',
        supports: 'temperature',
        fromZigbee: [
            fz.temperature,
        ],
        toZigbee: [
            tz.factory_reset,
        ],
        meta: {
            configureKey: 1,
            multiEndpoint: true,
        },
        configure: async (device, coordinatorEndpoint) => {
            const firstEndpoint = device.getEndpoint(1);
            await bind(firstEndpoint, coordinatorEndpoint, [
                'msTemperatureMeasurement',
            ]);
        const msTemperatureBindPayload = [{
            attribute: 'measuredValue',
            minimumReportInterval: 0,
            maximumReportInterval: 3600,
            reportableChange: 0,
        }];
            await firstEndpoint.configureReporting('msTemperatureMeasurement', msTemperatureBindPayload);
        },
        exposes: [
            exposes.numeric('temperature_1', ACCESS_STATE).withUnit('°C').withDescription('Measured temperature value'), 
        ],
};

module.exports = device;