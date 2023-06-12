import { WPILibWSRobotBase, DigitalChannelMode } from "@wpilib/wpilib-ws-robot";

import RomiDataBuffer, { FIRMWARE_IDENT } from "./romi-shmem-buffer";
import I2CErrorDetector from "../device-interfaces/i2c/i2c-error-detector";
import LSM6 from "./devices/core/lsm6/lsm6";
import RomiConfiguration, { CustomDeviceSpec, DEFAULT_IO_CONFIGURATION, IOPinMode, PinCapability, PinConfiguration } from "./romi-config";
import RomiAccelerometer from "./romi-accelerometer";
import RomiGyro from "./romi-gyro";
import QueuedI2CBus, { QueuedI2CHandle } from "../device-interfaces/i2c/queued-i2c-bus";
import { NetworkTableInstance, NetworkTable, EntryListenerFlags } from "node-ntcore";
import LogUtil from "../utils/logging/log-util";
import { FIFOModeSelection, OutputDataRate } from "./devices/core/lsm6/lsm6-settings";
import CustomDevice, { RobotHardwareInterfaces } from "./devices/custom/custom-device";
import CustomDeviceFactory from "./devices/custom/device-library";

interface IEncoderInfo {
    reportedValue: number; // This is the reading that is reported to usercode
    reportedPeriod: number; // This is the period that is reported to usercode
    lastRobotValue: number; // The last robot-reported value
    isHardwareReversed?: boolean;
    isSoftwareReversed?: boolean;
    lastReportedTime?: number;
}

interface DevicePortMapping {
    device: CustomDevice | "romi-onboard" | "romi-external";
    port: number;
}

type OnboardDIOFunction = "general" | "encoder";
type OnboardPWMFunction = "motor";

// Hardcoded list of onboard DIO ports
const ROMI_ONBOARD_DIO: OnboardDIOFunction[] = [
    "general", // 0 - Button A
    "general", // 1 - Button B / Green LED
    "general", // 2 - Button C / Red LED
    "general", // 3 - Yellow LED
    "encoder", // 4 - Left Encoder A
    "encoder", // 5 - Left Encoder B
    "encoder", // 6 - Right Encoder A
    "encoder", // 7 - Right Encoder B
];

// Hardcoded list of onboard PWM ports
const ROMI_ONBOARD_PWM: OnboardPWMFunction[] = [
    "motor", // 0 - Pink Motor
    "motor", // 1 - Ring Motor
    "motor", // 2 - Middle Motor
    "motor", // 3 - Index Motor
]

export interface DevicePortMappingInfo {
    deviceName: string;
    port: number;
}

export interface RobotIOChannelInfo {
    dio: DevicePortMappingInfo[],
    analogIn: DevicePortMappingInfo[],
    pwm: DevicePortMappingInfo[]
}

// Supported modes for the Romi pins
const IO_CAPABILITIES: PinCapability[] = [
    { supportedModes: [IOPinMode.DIO, IOPinMode.PWM] },
    { supportedModes: [IOPinMode.DIO, IOPinMode.PWM, IOPinMode.ANALOG_IN] },
    { supportedModes: [IOPinMode.DIO, IOPinMode.PWM, IOPinMode.ANALOG_IN] },
    { supportedModes: [IOPinMode.DIO, IOPinMode.PWM, IOPinMode.ANALOG_IN] },
    { supportedModes: [IOPinMode.DIO, IOPinMode.PWM, IOPinMode.ANALOG_IN] },
];

export const NUM_CONFIGURABLE_PINS: number = 5;

const logger = LogUtil.getLogger("ROMI");

export default class WPILibWSRomiRobot extends WPILibWSRobotBase {
    private _queuedBus: QueuedI2CBus;
    private _i2cHandle: QueuedI2CHandle;
    private _i2cHandle2: QueuedI2CHandle;

    private _firmwareIdent: number = -1;

    private _batteryPct: number = 0;

    private _heartbeatTimer: NodeJS.Timeout;
    private _readTimer: NodeJS.Timeout;
    private _imuReadTimer: NodeJS.Timeout;

    private _digitalInputValues: Map<number, boolean> = new Map<number, boolean>();
    private _analogInputValues: Map<number, number> = new Map<number, number>();
    private _encoderInputValues: Map<number, IEncoderInfo> = new Map<number, IEncoderInfo>();

    // These store the HAL-registered encoder channels. -1 implies uninitialized
    private _leftEncoderChannel: number = -1;
    private _rightEncoderChannel: number = -1;

    private _ioConfiguration: PinConfiguration[] = DEFAULT_IO_CONFIGURATION;

    private _dioDevicePortMapping: DevicePortMapping[] = [];
    private _analogInDevicePortMapping: DevicePortMapping[] = [];
    private _pwmDevicePortPortMapping: DevicePortMapping[] = [];

    private _extPinConfiguration: number[] = [];
    private _onboardPinConfiguration: number[] = [1, 0, 0, 0];

    private _readyP: Promise<void>;
    private _i2cErrorDetector: I2CErrorDetector = new I2CErrorDetector(10, 500, 100);

    private _lsm6: LSM6;

    private _imuFIFOperiod: number = 0;
    private _romiAccelerometer: RomiAccelerometer;
    private _romiGyro: RomiGyro;
    private _imuReadsPaused: boolean = false;

    // Keep track of the number of active WS connections
    private _numWsConnections: number = 0;

    // Keep track of whether or not the robot is DS enabled/disabled
    private _dsEnabled: boolean = false;

    // Keep track of the DS heartbeat
    private _dsHeartbeatPresent: boolean = false;

    private _customDevices: CustomDevice[] = [];

    private _statusNetworkTable: NetworkTable;
    private _configNetworkTable: NetworkTable;

    // Take in the abstract bus, since this will allow us to
    // write unit tests more easily
    constructor(bus: QueuedI2CBus, address: number, romiConfig?: RomiConfiguration) {
        super();

        const ntInstance = NetworkTableInstance.getDefault();
        this._statusNetworkTable = ntInstance.getTable("/Romi/Status");
        this._configNetworkTable = ntInstance.getTable("/Romi/Config");

        // By default, we'll use a queued I2C bus
        this._queuedBus = bus;
        this._i2cHandle = this._queuedBus.getNewAddressedHandle(address, true);
        // this._i2cHandle2 = this._queuedBus.getNewAddressedHandle(0x15, true);

        // Set up the LSM6DS33 (and associated Romi IMU devices-s)
        this._lsm6 = new LSM6(this._queuedBus.rawBus, 0x6B);
        this._romiAccelerometer = new RomiAccelerometer(this._lsm6);
        this._romiGyro = new RomiGyro(this._lsm6);

        this.registerAccelerometer(this._romiAccelerometer);
        this.registerGyro(this._romiGyro);

        // Configure the onboard hardware
        if (romiConfig) {
            if (romiConfig.externalIOConfig) {
                if(this._verifyConfiguration(romiConfig.externalIOConfig)) {
                    this._ioConfiguration = romiConfig.externalIOConfig;
                }
                else {
                    logger.warn("Error verifying pin configuration. Reverting to default");
                }
            }

            if (romiConfig.gyroZeroOffset) {
                this._lsm6.gyroOffset = romiConfig.gyroZeroOffset;
            }

            if (romiConfig.gyroFilterWindowSize !== undefined) {
                this._romiGyro.filterWindow = romiConfig.gyroFilterWindowSize;
            }

            if (romiConfig.customDevices) {
                const robotHW: RobotHardwareInterfaces = {
                    i2cBus: bus
                };

                this._registerCustomDevices(robotHW, romiConfig.customDevices);
            }
        }

        // Set up NT interfaces
        this._configureNTInterface();

        // Set up the ready indicator
        this._readyP =
            this._configureDevices()
            .then(() => {
                // Read firmware identifier
                return this.queryFirmwareIdent();
            })
            .then(() => {
                // Verify firmware
                if (this._firmwareIdent !== FIRMWARE_IDENT) {
                    logger.error(`Firmware Identifier Mismatch. Expected ${FIRMWARE_IDENT} but got ${this._firmwareIdent}`);
                }
            })
            .then(() => {
                // Initialize LSM6
                return this._lsm6.init()
                .then(() => {
                    // Set up the LSM6
                    // Don't change these unless you (kinda) know what you're doing!
                    this._lsm6.settings.accelFIFOEnabled = true;
                    this._lsm6.settings.gyroFIFOEnabled = true;
                    this._lsm6.settings.accelODR = OutputDataRate.ODR_104_HZ;
                    this._lsm6.settings.gyroODR = OutputDataRate.ODR_104_HZ;
                    this._lsm6.settings.fifoSampleRate = OutputDataRate.ODR_104_HZ;
                    this._lsm6.settings.fifoModeSelection = FIFOModeSelection.CONTINUOUS;

                    this._imuFIFOperiod = this._lsm6.getFIFOPeriod();

                    return this._lsm6.begin()
                    .then(() => {
                        return this._lsm6.fifoStart()
                    });
                })
                .then(() => {
                    logger.info("LSM6DS33 Initialized");
                })
                .catch(err => {
                    logger.error("Failed to initialize IMU: " + err.message);
                });
            })
            .then(() => {
                this._resetToCleanState();

                // Set up the heartbeat. Only send the heartbeat if we have
                // an active WS connection, the robot is in enabled state
                // AND we have a recent-ish DS packet
                this._heartbeatTimer = setInterval(() => {this._setRomiHeartBeat();}, 100 );

                // Set up the custom device update loop (if needed)
                if (this._customDevices.length > 0) {
                    setInterval(() => {
                        this._customDevices.forEach(device => {
                            device.update();
                        });
                    }, 20);
                }

                // Set up the read timer
                this._readTimer = setInterval(() => {
                    this._bulkAnalogRead();
                    this._bulkDigitalRead();
                    this._bulkEncoderRead();

                    this._readBattery();
                }, 50);

                this._imuReadTimer = setInterval(() => {
                    if (this._imuReadsPaused) {
                        return;
                    }

                    // Pull as many frames as we can for now, and update the devices
                    const frames = this._lsm6.getNewFIFOData();

                    if (frames.length > 0) {
                        this._romiAccelerometer.updateFromFrames(frames, this._imuFIFOperiod);
                        this._romiGyro.updateFromFrames(frames, this._imuFIFOperiod);
                    }
                }, 10);

                // Set up the status check
                setInterval(() => {
                    this._i2cHandle.readByte(RomiDataBuffer.status.offset)
                    .then(val => {
                        if (val === 0) {
                            logger.warn("Status byte is 0. Assuming brown out. Rewriting IO config");
                            // If the status byte is 0, we might have browned out the romi
                            // So we write the IO configuration again
                            this._writeRomiOnboardIOConfiguration()
                            .then(() => {
                                this._writeRomiExtIOConfiguration();
                            })
                            .then(() => {
                                // While we're at it... re-query the firmware
                                // Doing this on a timeout to give the 32U4 time
                                // to finish booting
                                setTimeout(() => {
                                    this.queryFirmwareIdent()
                                    .then((fwIdent) => {
                                        logger.info("Firmware Identifier: " + fwIdent);
                                    });
                                }, 2000);
                            });

                        }
                    })
                    .catch(err => {
                        this._i2cErrorDetector.addErrorInstance();
                    });
                }, 500);
            })
            .catch(err => {
                logger.error("Failed to initialize robot: ", err);
            });
    }

    public getIMU(): LSM6 {
        return this._lsm6;
    }

    public pauseIMUReads(reason?: string): void {
        logger.info(`Pausing IMU Reads ${reason ? "(" + reason + ")" : ""}`);
        this._imuReadsPaused = true;
    }

    public resumeIMUReads(reason?: string): void {
        logger.info(`Resuming IMU Reads ${reason ? "(" + reason + ")" : ""}`);
        this._imuReadsPaused = false;
    }

    public readyP(): Promise<void> {
        return this._readyP;
    }

    public get descriptor(): string {
        return "WPILibWS Reference Robot (Romi)";
    }

    public get firmwareIdent(): number {
        return this._firmwareIdent;
    }

    public get ioChannelInfo(): RobotIOChannelInfo {
        const result: RobotIOChannelInfo = {
            dio: [],
            analogIn: [],
            pwm: []
        }

        this._dioDevicePortMapping.forEach(devicePortMapping => {
            let deviceName: string;
            if (typeof devicePortMapping.device === "string") {
                deviceName = devicePortMapping.device;
            }
            else {
                deviceName = devicePortMapping.device.identifier;
            }

            result.dio.push({
                deviceName,
                port: devicePortMapping.port
            });
        });

        this._analogInDevicePortMapping.forEach(devicePortMapping => {
            let deviceName: string;
            if (typeof devicePortMapping.device === "string") {
                deviceName = devicePortMapping.device;
            }
            else {
                deviceName = devicePortMapping.device.identifier;
            }

            result.analogIn.push({
                deviceName,
                port: devicePortMapping.port
            });
        });

        this._pwmDevicePortPortMapping.forEach(devicePortMapping => {
            let deviceName: string;
            if (typeof devicePortMapping.device === "string") {
                deviceName = devicePortMapping.device;
            }
            else {
                deviceName = devicePortMapping.device.identifier;
            }

            result.pwm.push({
                deviceName,
                port: devicePortMapping.port
            });
        });

        return result;
    }

    public getBatteryPercentage(): number {
        return this._batteryPct;
    }

    public setDigitalChannelMode(channel: number, mode: DigitalChannelMode): void {
        // For DIO 0-3, we use the builtinConfig field
        if (channel < 0) {
            return;
        }

        const devicePortMapping = this._dioDevicePortMapping[channel];
        if (!devicePortMapping) {
            return;
        }

        const channelMode = (mode === DigitalChannelMode.INPUT) ? 1 : 0;

        if (devicePortMapping.device === "romi-onboard") {
            // Onboard DIO
            if (ROMI_ONBOARD_DIO[devicePortMapping.port] === "encoder") {
                return;
            }

            // See https://github.com/wpilibsuite/wpilib-ws-robot-romi/tree/main/firmware#digital-io
            // for more information on what each port is connected to
            // Essentially, DIO 0 is hardcoded to Button A (input only)
            // DIO 3 is hardcoded to the yellow LED (Output only)
            // DIO 1 and 2 are configurable
            if (devicePortMapping.port === 1 || devicePortMapping.port === 2) {
                this._onboardPinConfiguration[devicePortMapping.port] = channelMode;
                this._writeRomiOnboardIOConfiguration();

                if (mode === DigitalChannelMode.OUTPUT) {
                    this.setDIOValue(devicePortMapping.port, true);
                }
            }
        }
        else if (devicePortMapping.device === "romi-external") {
            const ioPin = devicePortMapping.port;
            this._extPinConfiguration[ioPin] = channelMode;

            this._writeRomiExtIOConfiguration();
        }
        else {
            devicePortMapping.device.setDigitalChannelMode(devicePortMapping.port, mode);
        }

        if (mode !== DigitalChannelMode.INPUT) {
            this._digitalInputValues.delete(channel);
        }
        else if (!this._digitalInputValues.has(channel)) {
            this._digitalInputValues.set(channel, false);
        }
    }

    public setDIOValue(channel: number, value: boolean): void {
        if (channel < 0) {
            return;
        }

        const devicePortMapping = this._dioDevicePortMapping[channel];
        if (!devicePortMapping) {
            return;
        }

        if (devicePortMapping.device === "romi-onboard") {
            if (ROMI_ONBOARD_DIO[devicePortMapping.port] === "general") {
                // Use the built in DIO
                this._i2cHandle.writeByte(RomiDataBuffer.builtinDioValues.offset + devicePortMapping.port, value ? 1 : 0)
                .catch(err => {
                    this._i2cErrorDetector.addErrorInstance();
                });
            }
        }
        else if (devicePortMapping.device === "romi-external") {
            const ioIdx = devicePortMapping.port;
            this._i2cHandle.writeByte(RomiDataBuffer.extIoValues.offset + (ioIdx * 2), value ? 1 : 0)
            .catch(err => {
                this._i2cErrorDetector.addErrorInstance();
            });
        }
        else {
            devicePortMapping.device.setDIOValue(devicePortMapping.port, value);
        }
    }

    public getDIOValue(channel: number): boolean {
        if (!this._digitalInputValues.has(channel)) {
            return false;
        }

        return this._digitalInputValues.get(channel);
    }

    public setAnalogOutVoltage(channel: number, voltage: number): void {
        // no-op
    }

    public getAnalogInVoltage(channel: number): number {
        if (!this._analogInputValues.has(channel)) {
            return 0.0;
        }

        return this._analogInputValues.get(channel);
    }

    public setPWMValue(channel: number, value: number): void {
        if (channel < 0) {
            return;
        }

        const devicePortMapping = this._pwmDevicePortPortMapping[channel];
        if (!devicePortMapping) {
            return;
        }

        if (devicePortMapping.device === "romi-onboard") {
            // We get the value in the range 0-255 but the romi
            // expects -400 to 400
            // Positive values here correspond to forward motion
            const romiValue = Math.floor(((value / 255) * 800) - 400);

            // Convert back to -1 to 1
            const wpiValue = ((value * 2) / 255) - 1;

            // We need to do some trickery to get a twos-complement number
            // Essentially we'll write a 16 bit signed int to the buffer
            // and read it out as an unsigned int
            // Mainly to work around the fact that the i2c-bus library's
            // writeBlock() doesn't work...
            const tmp = Buffer.alloc(2);
            tmp.writeInt16BE(romiValue);
            logger.info(`channel: ${channel} value: ${value} wpiValue: ${wpiValue}`);

            let offset;
            if (devicePortMapping.port === 0) {
                offset = RomiDataBuffer.pinkMotor.offset;
            }
            else if (devicePortMapping.port === 1) {
                offset = RomiDataBuffer.ringMotor.offset;
            }
            else if (devicePortMapping.port === 2) {
                offset = RomiDataBuffer.middleMotor.offset;
            }
            else {
                offset = RomiDataBuffer.indexMotor.offset;
            }

            this._i2cHandle.writeWord(offset, tmp.readUInt16BE())
            .catch(err => {
                this._i2cErrorDetector.addErrorInstance();
            });

            // // Added to write to the second arduino
            // this._i2cHandle2.writeWord(offset, tmp.readUInt16BE())
            // .catch(err => {
            //     this._i2cErrorDetector.addErrorInstance();
            // });
        }
        else if (devicePortMapping.device === "romi-external") {
            // Same conversion logic as above
            const romiValue = Math.floor(((value / 255) * 800) - 400);
            const tmp = Buffer.alloc(2);
            tmp.writeInt16BE(romiValue);

            const ioIdx = devicePortMapping.port;
            const offset = RomiDataBuffer.extIoValues.offset + (ioIdx * 2);

            this._i2cHandle.writeWord(offset, tmp.readUInt16BE())
            .catch(err => {
                this._i2cErrorDetector.addErrorInstance();
            });
        }
        else {
            devicePortMapping.device.setPWMValue(devicePortMapping.port, value);
        }
    }

    public registerEncoder(encoderChannel: number, channelA: number, channelB: number) {
        // Left encoder uses dio 4/5, right uses 6/7
        // If the channels are reversed, we'll set the hardware reversed flag
        if (channelA === 4 && channelB === 5) {
            this._encoderInputValues.set(encoderChannel, {
                reportedValue: 0,
                reportedPeriod: Number.MAX_VALUE,
                lastRobotValue: 0,
                isHardwareReversed: false
            });
            this._leftEncoderChannel = encoderChannel;
        }
        else if (channelA === 5 && channelB === 4) {
            this._encoderInputValues.set(encoderChannel, {
                reportedValue: 0,
                reportedPeriod: Number.MAX_VALUE,
                lastRobotValue: 0,
                isHardwareReversed: true
            });
            this._leftEncoderChannel = encoderChannel;
        }
        else if (channelA === 6 && channelB === 7) {
            this._encoderInputValues.set(encoderChannel, {
                reportedValue: 0,
                reportedPeriod: Number.MAX_VALUE,
                lastRobotValue: 0,
                isHardwareReversed: false
            });
            this._rightEncoderChannel = encoderChannel;
        }
        else if (channelA === 7 && channelB === 6) {
            this._encoderInputValues.set(encoderChannel, {
                reportedValue: 0,
                reportedPeriod: Number.MAX_VALUE,
                lastRobotValue: 0,
                isHardwareReversed: false
            });
            this._rightEncoderChannel = encoderChannel;
        }

        // If we have the wrong combination of pins, we ignore the encoder
    }

    public getEncoderCount(channel: number): number {
        if (!this._encoderInputValues.has(channel)) {
            return 0;
        }

        return this._encoderInputValues.get(channel).reportedValue;
    }

    public getEncoderPeriod(channel: number): number {
        if (!this._encoderInputValues.has(channel)) {
            return Number.MAX_VALUE;
        }

        return this._encoderInputValues.get(channel).reportedPeriod;
    }

    public resetEncoder(channel: number, keepLast?: boolean): void {
        let offset;
        if (channel === this._leftEncoderChannel) {
            offset = RomiDataBuffer.resetLeftEncoder.offset;
        }
        else if (channel === this._rightEncoderChannel) {
            offset = RomiDataBuffer.resetRightEncoder.offset;
        }
        else {
            return;
        }

        const encoderInfo = this._encoderInputValues.get(channel);
        encoderInfo.lastRobotValue = 0;

        if (!keepLast) {
            encoderInfo.reportedValue = 0;
        }

        this._i2cHandle.writeByte(offset, 1)
        .catch(err => {
            this._i2cErrorDetector.addErrorInstance();
        });
    }

    public setEncoderReverseDirection(channel: number, reverse: boolean): void {
        const encoderInfo = this._encoderInputValues.get(channel);
        if (encoderInfo) {
            encoderInfo.isSoftwareReversed = reverse;
        }
    }

    /**
     * Called when a new WebSocket connection occurs
     */
    public onWSConnection(remoteAddrV4?: string): void {
        // If this is the first WS connection
        if (this._numWsConnections === 0) {
            // Reset the gyro. This will ensure that the gyro will
            // read 0 (or close to it) as the robot program starts up
            this._romiGyro.reset();
        }

        this._numWsConnections++;

        logger.info(`New WS Connection from ${remoteAddrV4}`);
        this.emit("wsConnection", {
            remoteAddrV4
        });
    }

    /**
     * Called when a WebSocket disconnects
     */
    public onWSDisconnection(): void {
        this._numWsConnections--;

        // If this was our last disconnection, clear out all the state
        if (this._numWsConnections === 0) {
            console.log("[ROMI] Lost all connections, resetting state");
            this._resetToCleanState();
            this.emit("wsNoConnections");
        }
    }

    public onRobotEnabled(): void {
        logger.info("Robot ENABLED");
        this._dsEnabled = true;
        // To ensure Romi will act on signals sent immediately
        this._setRomiHeartBeat();
    }

    public onRobotDisabled(): void {
        logger.info("Robot DISABLED");
        this._dsEnabled = false;
    }

    public onDSPacketTimeoutOccurred(): void {
        logger.warn("DS Packet Heartbeat Lost");
        this._dsHeartbeatPresent = false;
    }

    public onDSPacketTimeoutCleared(): void {
        logger.info("DS Packet Heartbeat Acquired");
        this._dsHeartbeatPresent = true;
    }

    public async queryFirmwareIdent(): Promise<number> {
        return this._i2cHandle.readByte(RomiDataBuffer.firmwareIdent.offset)
        .then(fwIdent => {
            this._firmwareIdent = fwIdent;
            return fwIdent;
        })
        .catch(err => {
            this._i2cErrorDetector.addErrorInstance();
            this._firmwareIdent = -1;
            return -1;
        });
    }

    private _verifyConfiguration(config: PinConfiguration[]): boolean {
        if (config.length !== IO_CAPABILITIES.length) {
            logger.warn(`Incorrect number of pin config options. Expected ${IO_CAPABILITIES.length} but got ${config.length}`);
            return false;
        }

        for (let i = 0; i < config.length; i++) {
            // For each element, make sure that we are setting a mode that is supported
            const configOption = config[i];
            if (IO_CAPABILITIES[i].supportedModes.indexOf(configOption.mode) === -1) {
                logger.warn(`Invalid mode set for pin ${i}. Supported modes are ${JSON.stringify(IO_CAPABILITIES[i].supportedModes)}`);
                return false;
            }
        }

        return true;
    }

    /**
     * Configure all devices on the Romi
     * This includes setting up the appropriate IO port->device/port pairs
     */
    private async _configureDevices(): Promise<void> {
        // Clear out the maps
        this._dioDevicePortMapping = [];
        this._analogInDevicePortMapping = [];
        this._pwmDevicePortPortMapping = [];

        // Clear out the External IO Pin config
        this._extPinConfiguration = [];

        // Set up all onboard channels
        ROMI_ONBOARD_DIO.forEach((val, idx) => {
            this._dioDevicePortMapping.push({
                device: "romi-onboard",
                port: idx
            });
        });

        ROMI_ONBOARD_PWM.forEach((val, idx) => {
            this._pwmDevicePortPortMapping.push({
                device: "romi-onboard",
                port: idx
            });
        });

        // Now configure the external IO
        this._ioConfiguration.forEach((pinConfig, ioIdx) => {
            switch (pinConfig.mode) {
                case IOPinMode.ANALOG_IN:
                    this._extPinConfiguration.push(2);

                    this._analogInDevicePortMapping.push({
                        device: "romi-external",
                        port: ioIdx // We use ioIdx here so that we know which offset to write to
                    });

                    this._analogInputValues.set(this._analogInDevicePortMapping.length - 1, 0.0);
                    break;
                case IOPinMode.DIO:
                    // Default to OUTPUT for digital pins
                    this._extPinConfiguration.push(0);

                    this._dioDevicePortMapping.push({
                        device: "romi-external",
                        port: ioIdx
                    });
                    break;
                case IOPinMode.PWM:
                    this._extPinConfiguration.push(3);
                    this._pwmDevicePortPortMapping.push({
                        device: "romi-external",
                        port: ioIdx
                    });
                    break;
            }
        });

        // Write the onboard and external IO configurations
        // and then set up the custom devices
        return this._writeRomiOnboardIOConfiguration()
        .then(() => {
            return this._writeRomiExtIOConfiguration();
        })
        .then(() => {
            // Configure any custom devices we might have
            this._customDevices.forEach(device => {
                const ioInterfaces = device.ioInterfaces;

                if (ioInterfaces.numDioPorts !== undefined) {
                    for (let i = 0; i < ioInterfaces.numDioPorts; i++) {
                        this._dioDevicePortMapping.push({
                            device,
                            port: i
                        });
                    }
                }

                if (ioInterfaces.numAnalogInPorts !== undefined) {
                    for (let i = 0; i < ioInterfaces.numAnalogInPorts; i++) {
                        this._analogInDevicePortMapping.push({
                            device,
                            port: i
                        });
                    }
                }

                if (ioInterfaces.numPwmOutPorts !== undefined) {
                    for (let i = 0; i < ioInterfaces.numPwmOutPorts; i++) {
                        this._pwmDevicePortPortMapping.push({
                            device,
                            port: i
                        });
                    }
                }

                if (ioInterfaces.simDevices !== undefined) {
                    ioInterfaces.simDevices.forEach(simDevice => {
                        this.registerSimDevice(simDevice);
                    });
                }
            });
        });
    }

    /**
     * Write the onboard IO configuration in oneshot
     */
    private async _writeRomiOnboardIOConfiguration(): Promise<void> {
        let configRegister: number = (1 << 7);
        this._onboardPinConfiguration.forEach((pinMode, ioIdx) => {
            let pinModeConfig: number = (pinMode & 0x1) << ioIdx;
            configRegister |= pinModeConfig;
        });

        return this._i2cHandle.writeByte(RomiDataBuffer.builtinConfig.offset, configRegister, 3)
        .catch(err => {
            this._i2cErrorDetector.addErrorInstance();
        });
    }

    /**
     * Do the actual configuration write to the romi
     */
    private async _writeRomiExtIOConfiguration(): Promise<void> {
        let configRegister: number = (1 << 15);

        this._extPinConfiguration.forEach((pinMode, ioIdx) => {
            let pinModeConfig: number = (pinMode & 0x3) << (13 - (2 * ioIdx));
            configRegister |= pinModeConfig;
        });

        return this._i2cHandle.writeWord(RomiDataBuffer.ioConfig.offset, configRegister, 3)
        .catch(err => {
            this._i2cErrorDetector.addErrorInstance();
        });
    }

    private _setRomiHeartBeat(): void {
        if (this._numWsConnections > 0 && this._dsEnabled && this._dsHeartbeatPresent) {
            this._i2cHandle.writeByte(RomiDataBuffer.heartbeat.offset, 1)
            .catch(err => {
                this._i2cErrorDetector.addErrorInstance();
            });
        }
    }

    private _bulkAnalogRead() {
        this._analogInDevicePortMapping.forEach((devicePortMapping, ainIdx) => {
            if (devicePortMapping.device === "romi-onboard") {
                return;
            }

            if (devicePortMapping.device === "romi-external") {
                const offset = RomiDataBuffer.extIoValues.offset + (devicePortMapping.port * 2);
                this._i2cHandle.readWord(offset)
                .then(adcVal => {
                    // The value sent over the wire is a 10-bit ADC value
                    // We'll need to convert it to 5V
                    const voltage = (adcVal / 1023.0) * 5.0;
                    this._analogInputValues.set(ainIdx, voltage);
                })
                .catch(err => {
                    this._i2cErrorDetector.addErrorInstance();
                });
            }
            else {
                devicePortMapping.device.getAnalogInVoltage(devicePortMapping.port)
                .then(voltage => {
                    this._analogInputValues.set(ainIdx, voltage);
                });
            }
        });

    }

    private _bulkDigitalRead() {
        this._digitalInputValues.forEach((val, channel) => {
            const devicePortMapping = this._dioDevicePortMapping[channel];
            if (!devicePortMapping) {
                return;
            }

            if (devicePortMapping.device === "romi-onboard") {
                const offset = RomiDataBuffer.builtinDioValues.offset + channel;
                this._i2cHandle.readByte(offset)
                .then(value => {
                    this._digitalInputValues.set(channel, value !== 0);
                })
                .catch(err => {
                    this._i2cErrorDetector.addErrorInstance();
                });
            }
            else if (devicePortMapping.device === "romi-external") {
                const offset = RomiDataBuffer.extIoValues.offset + (devicePortMapping.port * 2);
                this._i2cHandle.readByte(offset)
                .then(value => {
                    this._digitalInputValues.set(channel, value !== 0);
                })
                .catch(err => {
                    this._i2cErrorDetector.addErrorInstance();
                });
            }
            else {
                devicePortMapping.device.getDigitalInValue(devicePortMapping.port)
                .then(value => {
                    this._digitalInputValues.set(channel, value);
                });
            }
        });
    }

    private _bulkEncoderRead() {
        this._encoderInputValues.forEach((encoderInfo, channel) => {
            let offset: number;
            if (channel === this._leftEncoderChannel) {
                offset = RomiDataBuffer.leftEncoder.offset;
            }
            else if (channel === this._rightEncoderChannel) {
                offset = RomiDataBuffer.rightEncoder.offset;
            }
            else {
                // Invalid encoder channel (shouldn't happen)
                // bail out
                return;
            }

            this._i2cHandle.readWord(offset)
            .then(encoderValue => {
                // This comes in as a uint16_t
                const buf = Buffer.alloc(2);
                buf.writeUInt16LE(encoderValue);
                encoderValue = buf.readInt16LE();

                const lastValue = encoderInfo.lastRobotValue;

                // Figure out if we should be reporting flipped values
                const reverseMultiplier = (encoderInfo.isHardwareReversed ? -1 : 1) *
                                          (encoderInfo.isSoftwareReversed ? -1 : 1);
                const delta = (encoderValue - lastValue) * reverseMultiplier;

                encoderInfo.reportedValue += delta;
                encoderInfo.lastRobotValue = encoderValue;

                const currTimestamp = Date.now();

                // Calculate the period
                if (encoderInfo.lastReportedTime !== undefined) {
                    const timespanMs = currTimestamp - encoderInfo.lastReportedTime;
                    // Period = (approx) timespan / delta
                    if (delta === 0) {
                        encoderInfo.reportedPeriod = Number.MAX_VALUE;
                    }
                    else {
                        encoderInfo.reportedPeriod = (timespanMs / delta) / 1000.0;
                    }
                }

                encoderInfo.lastReportedTime = currTimestamp;

                // If we're getting close to the limits, reset the romi
                // encoder so we don't overflow
                if (Math.abs(encoderValue) > 30000) {
                    this.resetEncoder(channel, true);
                    encoderInfo.lastRobotValue = 0;
                }
            })
            .catch(err => {
                this._i2cErrorDetector.addErrorInstance();
            })
        });
    }

    private _readBattery(): void {
        this._i2cHandle.readWord(RomiDataBuffer.batteryMillivolts.offset)
        .then(battMv => {
            this._batteryPct = battMv / 9000;
        })
        .catch(err => {
            this._i2cErrorDetector.addErrorInstance();
        })
    }

    /**
     * Resets the Romi to a known clean state
     * This does NOT reset any IO configuration
     */
    private _resetToCleanState(): void {
        this._digitalInputValues.clear();
        this._encoderInputValues.clear();
        this._analogInputValues.clear();

        this._leftEncoderChannel = -1;
        this._rightEncoderChannel = -1;

        // Set up DIO 0 as an input because it's a button
        this._digitalInputValues.set(0, false);

        // Set yellow LED to be true by default since
        // DigitalOutput in wpilib defaults to true
        this.setDIOValue(3, true);

        // Reset our ds enabled state
        this._dsEnabled = false;
    }

    private _configureNTInterface() {
        const GYRO_ADD_OFFSET_X_KEY = "Gyro Runtime Offset X";
        const GYRO_ADD_OFFSET_Y_KEY = "Gyro Runtime Offset Y";
        const GYRO_ADD_OFFSET_Z_KEY = "Gyro Runtime Offset Z";
        // Set up the gyro filter window
        this._configNetworkTable.getEntry("Gyro Filter Window").setDouble(this._romiGyro.filterWindow);
        this._configNetworkTable.addEntryListener("Gyro Filter Window", (table, key, entry, value, flags) => {
            const newValue = value.getDouble();
            if (newValue !== this._romiGyro.filterWindow) {
                this._romiGyro.filterWindow = newValue;

                // Update the value on the wire
                this._configNetworkTable.getEntry("Gyro Filter Window").setDouble(this._romiGyro.filterWindow);
            }
        }, EntryListenerFlags.NEW | EntryListenerFlags.UPDATE);

        // Set up additional offsets on the gyro
        const addlOffsetXEntry = this._configNetworkTable.getEntry(GYRO_ADD_OFFSET_X_KEY);
        addlOffsetXEntry.setDouble(0.0);
        this._configNetworkTable.addEntryListener(GYRO_ADD_OFFSET_X_KEY, (table, key, entry, value, flags) => {
            const newValue = value.getDouble();
            if (newValue !== this._lsm6.gyroRuntimeOffset.x) {
                this._lsm6.setRuntimeOffsetX(newValue);
            }
        }, EntryListenerFlags.NEW | EntryListenerFlags.UPDATE);

        const addlOffsetYEntry = this._configNetworkTable.getEntry(GYRO_ADD_OFFSET_Y_KEY);
        addlOffsetYEntry.setDouble(0.0);
        this._configNetworkTable.addEntryListener(GYRO_ADD_OFFSET_Y_KEY, (table, key, entry, value, flags) => {
            const newValue = value.getDouble();
            if (newValue !== this._lsm6.gyroRuntimeOffset.y) {
                this._lsm6.setRuntimeOffsetY(newValue);
            }
        }, EntryListenerFlags.NEW | EntryListenerFlags.UPDATE);

        const addlOffsetZEntry = this._configNetworkTable.getEntry(GYRO_ADD_OFFSET_Z_KEY);
        addlOffsetZEntry.setDouble(0.0);
        this._configNetworkTable.addEntryListener(GYRO_ADD_OFFSET_Z_KEY, (table, key, entry, value, flags) => {
            const newValue = value.getDouble();
            if (newValue !== this._lsm6.gyroRuntimeOffset.z) {
                this._lsm6.setRuntimeOffsetZ(newValue);
            }
        }, EntryListenerFlags.NEW | EntryListenerFlags.UPDATE);
    }

    private _registerCustomDevices(robotHardware: RobotHardwareInterfaces, deviceSpecs: CustomDeviceSpec[]) {
        const singletonDevices: Set<string> = new Set<string>();

        deviceSpecs.forEach(deviceSpec => {
            try {
                const device = CustomDeviceFactory.createDevice(deviceSpec.type, robotHardware, deviceSpec.config);

                if (device.isSingleton) {
                    if (singletonDevices.has(deviceSpec.type)) {
                        throw new Error("Singleton device already exists");
                    }

                    singletonDevices.add(deviceSpec.type);
                }

                this._customDevices.push(device);
            }
            catch (err) {
                logger.error(`Error creating device (${deviceSpec.type}): ${err.message}`);
            }
        });
    }
}
