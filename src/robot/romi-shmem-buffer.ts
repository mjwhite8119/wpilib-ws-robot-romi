// AUTOGENERATED FILE. DO NOT MODIFY.
// Generated via `npm run gen-shmem`

// Instance: 418408b6-fd52-4d9e-b93a-0d031e7d27c0

export const FIRMWARE_IDENT: number = 192;

export enum ShmemDataType {
    BOOL,
    UINT8_T,
    INT8_T,
    UINT16_T,
    INT16_T,
}

export interface ShmemElementDefinition {
    offset: number;
    type: ShmemDataType;
    arraySize?: number;
}

const shmemBuffer: {[key: string]: ShmemElementDefinition} = {
    ioConfig: { offset: 0, type: ShmemDataType.UINT16_T},
    firmwareIdent: { offset: 2, type: ShmemDataType.UINT8_T},
    status: { offset: 3, type: ShmemDataType.UINT8_T},
    heartbeat: { offset: 4, type: ShmemDataType.BOOL},
    builtinConfig: { offset: 5, type: ShmemDataType.UINT8_T},
    builtinDioValues: { offset: 6, type: ShmemDataType.BOOL, arraySize: 4},
    extIoValues: { offset: 10, type: ShmemDataType.INT16_T, arraySize: 5},
    analog: { offset: 20, type: ShmemDataType.UINT16_T, arraySize: 2},
    pinkMotor: { offset: 24, type: ShmemDataType.INT16_T},
    ringMotor: { offset: 26, type: ShmemDataType.INT16_T},
    batteryMillivolts: { offset: 28, type: ShmemDataType.UINT16_T},
    resetLeftEncoder: { offset: 30, type: ShmemDataType.BOOL},
    resetRightEncoder: { offset: 31, type: ShmemDataType.BOOL},
    leftEncoder: { offset: 32, type: ShmemDataType.INT16_T},
    rightEncoder: { offset: 34, type: ShmemDataType.INT16_T},
};

export default Object.freeze(shmemBuffer);
