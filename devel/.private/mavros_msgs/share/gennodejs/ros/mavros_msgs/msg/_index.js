
"use strict";

let LogEntry = require('./LogEntry.js');
let DebugValue = require('./DebugValue.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let VFR_HUD = require('./VFR_HUD.js');
let CameraImageCaptured = require('./CameraImageCaptured.js');
let HilSensor = require('./HilSensor.js');
let WaypointList = require('./WaypointList.js');
let NavControllerOutput = require('./NavControllerOutput.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let Mavlink = require('./Mavlink.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let PositionTarget = require('./PositionTarget.js');
let ManualControl = require('./ManualControl.js');
let HomePosition = require('./HomePosition.js');
let ESCTelemetryItem = require('./ESCTelemetryItem.js');
let Param = require('./Param.js');
let GPSRAW = require('./GPSRAW.js');
let Altitude = require('./Altitude.js');
let ParamValue = require('./ParamValue.js');
let LandingTarget = require('./LandingTarget.js');
let Tunnel = require('./Tunnel.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let RTKBaseline = require('./RTKBaseline.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let BatteryStatus = require('./BatteryStatus.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let ESCTelemetry = require('./ESCTelemetry.js');
let StatusText = require('./StatusText.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let ActuatorControl = require('./ActuatorControl.js');
let ESCInfo = require('./ESCInfo.js');
let TerrainReport = require('./TerrainReport.js');
let Trajectory = require('./Trajectory.js');
let Waypoint = require('./Waypoint.js');
let MagnetometerReporter = require('./MagnetometerReporter.js');
let GPSINPUT = require('./GPSINPUT.js');
let RTCM = require('./RTCM.js');
let Thrust = require('./Thrust.js');
let Vibration = require('./Vibration.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let CellularStatus = require('./CellularStatus.js');
let WaypointReached = require('./WaypointReached.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let State = require('./State.js');
let ExtendedState = require('./ExtendedState.js');
let RCIn = require('./RCIn.js');
let MountControl = require('./MountControl.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let HilGPS = require('./HilGPS.js');
let RCOut = require('./RCOut.js');
let HilControls = require('./HilControls.js');
let RadioStatus = require('./RadioStatus.js');
let GPSRTK = require('./GPSRTK.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let LogData = require('./LogData.js');
let ESCStatus = require('./ESCStatus.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let FileEntry = require('./FileEntry.js');
let VehicleInfo = require('./VehicleInfo.js');
let CommandCode = require('./CommandCode.js');

module.exports = {
  LogEntry: LogEntry,
  DebugValue: DebugValue,
  PlayTuneV2: PlayTuneV2,
  VFR_HUD: VFR_HUD,
  CameraImageCaptured: CameraImageCaptured,
  HilSensor: HilSensor,
  WaypointList: WaypointList,
  NavControllerOutput: NavControllerOutput,
  TimesyncStatus: TimesyncStatus,
  Mavlink: Mavlink,
  CamIMUStamp: CamIMUStamp,
  EstimatorStatus: EstimatorStatus,
  PositionTarget: PositionTarget,
  ManualControl: ManualControl,
  HomePosition: HomePosition,
  ESCTelemetryItem: ESCTelemetryItem,
  Param: Param,
  GPSRAW: GPSRAW,
  Altitude: Altitude,
  ParamValue: ParamValue,
  LandingTarget: LandingTarget,
  Tunnel: Tunnel,
  WheelOdomStamped: WheelOdomStamped,
  RTKBaseline: RTKBaseline,
  OnboardComputerStatus: OnboardComputerStatus,
  BatteryStatus: BatteryStatus,
  CompanionProcessStatus: CompanionProcessStatus,
  ESCTelemetry: ESCTelemetry,
  StatusText: StatusText,
  HilStateQuaternion: HilStateQuaternion,
  ActuatorControl: ActuatorControl,
  ESCInfo: ESCInfo,
  TerrainReport: TerrainReport,
  Trajectory: Trajectory,
  Waypoint: Waypoint,
  MagnetometerReporter: MagnetometerReporter,
  GPSINPUT: GPSINPUT,
  RTCM: RTCM,
  Thrust: Thrust,
  Vibration: Vibration,
  ADSBVehicle: ADSBVehicle,
  CellularStatus: CellularStatus,
  WaypointReached: WaypointReached,
  AttitudeTarget: AttitudeTarget,
  OpticalFlowRad: OpticalFlowRad,
  HilActuatorControls: HilActuatorControls,
  State: State,
  ExtendedState: ExtendedState,
  RCIn: RCIn,
  MountControl: MountControl,
  GlobalPositionTarget: GlobalPositionTarget,
  ESCInfoItem: ESCInfoItem,
  HilGPS: HilGPS,
  RCOut: RCOut,
  HilControls: HilControls,
  RadioStatus: RadioStatus,
  GPSRTK: GPSRTK,
  OverrideRCIn: OverrideRCIn,
  LogData: LogData,
  ESCStatus: ESCStatus,
  ESCStatusItem: ESCStatusItem,
  FileEntry: FileEntry,
  VehicleInfo: VehicleInfo,
  CommandCode: CommandCode,
};
