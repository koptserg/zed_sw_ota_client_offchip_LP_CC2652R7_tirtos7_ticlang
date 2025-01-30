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
        zigbeeModel: ['DIYRuZ_SW7'],
        model: 'DIYRuZ_SW7',
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
            fz.battery,
        ],
        toZigbee: [
            tz.factory_reset,
        ],
        configure: async (device, coordinatorEndpoint) => {
            const eightEndpoint = device.getEndpoint(8);
            await bind(eightEndpoint, coordinatorEndpoint, [
                'genOnOff',
                'genPowerCfg',
            ]);
            const nineEndpoint = device.getEndpoint(9);
            await bind(nineEndpoint, coordinatorEndpoint, [
                'msIlluminanceMeasurement',
                'msTemperatureMeasurement',
                'msPressureMeasurement',
                'msRelativeHumidity',
            ]);
        },
        exposes: [
            e.action(['toggle']),
            e.illuminance(),
            e.temperature(),
            e.humidity(),
            e.pressure(),            
            e.battery(),
            e.battery_voltage(),
         ],
};

module.exports = device;