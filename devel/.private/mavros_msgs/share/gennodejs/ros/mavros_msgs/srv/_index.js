
"use strict";

let CommandLong = require('./CommandLong.js')
let ParamPush = require('./ParamPush.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let WaypointClear = require('./WaypointClear.js')
let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let FileRemove = require('./FileRemove.js')
let MountConfigure = require('./MountConfigure.js')
let CommandTOL = require('./CommandTOL.js')
let LogRequestEnd = require('./LogRequestEnd.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let CommandHome = require('./CommandHome.js')
let FileOpen = require('./FileOpen.js')
let FileWrite = require('./FileWrite.js')
let SetMode = require('./SetMode.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let FileRename = require('./FileRename.js')
let FileList = require('./FileList.js')
let LogRequestList = require('./LogRequestList.js')
let FileMakeDir = require('./FileMakeDir.js')
let ParamGet = require('./ParamGet.js')
let MessageInterval = require('./MessageInterval.js')
let CommandAck = require('./CommandAck.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let VehicleInfoGet = require('./VehicleInfoGet.js')
let FileTruncate = require('./FileTruncate.js')
let WaypointPush = require('./WaypointPush.js')
let SetMavFrame = require('./SetMavFrame.js')
let ParamPull = require('./ParamPull.js')
let FileRead = require('./FileRead.js')
let StreamRate = require('./StreamRate.js')
let ParamSet = require('./ParamSet.js')
let FileClose = require('./FileClose.js')
let WaypointPull = require('./WaypointPull.js')
let LogRequestData = require('./LogRequestData.js')
let CommandInt = require('./CommandInt.js')
let CommandBool = require('./CommandBool.js')
let FileChecksum = require('./FileChecksum.js')

module.exports = {
  CommandLong: CommandLong,
  ParamPush: ParamPush,
  CommandTriggerControl: CommandTriggerControl,
  WaypointClear: WaypointClear,
  CommandTriggerInterval: CommandTriggerInterval,
  FileRemove: FileRemove,
  MountConfigure: MountConfigure,
  CommandTOL: CommandTOL,
  LogRequestEnd: LogRequestEnd,
  FileRemoveDir: FileRemoveDir,
  CommandHome: CommandHome,
  FileOpen: FileOpen,
  FileWrite: FileWrite,
  SetMode: SetMode,
  CommandVtolTransition: CommandVtolTransition,
  FileRename: FileRename,
  FileList: FileList,
  LogRequestList: LogRequestList,
  FileMakeDir: FileMakeDir,
  ParamGet: ParamGet,
  MessageInterval: MessageInterval,
  CommandAck: CommandAck,
  WaypointSetCurrent: WaypointSetCurrent,
  VehicleInfoGet: VehicleInfoGet,
  FileTruncate: FileTruncate,
  WaypointPush: WaypointPush,
  SetMavFrame: SetMavFrame,
  ParamPull: ParamPull,
  FileRead: FileRead,
  StreamRate: StreamRate,
  ParamSet: ParamSet,
  FileClose: FileClose,
  WaypointPull: WaypointPull,
  LogRequestData: LogRequestData,
  CommandInt: CommandInt,
  CommandBool: CommandBool,
  FileChecksum: FileChecksum,
};
