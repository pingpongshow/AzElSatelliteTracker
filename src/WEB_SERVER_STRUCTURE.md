# Web Server File Decomposition

The `web_server.cpp` file has been decomposed into multiple files for better maintainability:

## File Structure

### web_server.cpp (Core - ~940 lines)
Contains:
- Constructor/destructor
- Server lifecycle: `begin()`, `stop()`, `update()`
- WebSocket: `broadcastStatus()`, `sendEvent()`, `onWebSocketEvent()`, `getClientCount()`
- Route setup: `setupRoutes()`, `setupAPIRoutes()`, `setupCameraRoutes()`, `setupAdvancedRoutes()`, `setupWebSocketHandler()`
- Core handlers: `handleRoot()`, `handleNotFound()`
- Status handlers: `handleGetStatus()`, `handleGetPosition()`, `handleGetGPS()`, `handleGetConfig()`
- Control handlers: `handleSetPosition()`, `handleStop()`, `handlePark()`, `handleHome()`, `handleSetHome()`, `handleCalibrate()`
- WiFi handlers: `handleGetNetworks()`, `handleConnectWiFi()`, `handleGetWiFiStatus()`
- System handlers: `handleGetSystemInfo()`, `handleReboot()`, `handleFactoryReset()`, `handleSetConfig()`, `handleSetManualGPS()`
- Helper functions: `sendJSON()`, `sendError()`, `sendOK()`, `buildStatusJSON()`, `buildPassJSON()`

### web_handlers_tracking.cpp (~615 lines)
Contains:
- Tracking: `handleStartTracking()`, `handleStopTracking()`, `handleGetNextPass()`
- TLE management: `handleGetTLEs()`, `handleLoadTLE()`, `handleFetchTLE()`, `handleDeleteTLE()`
- Pass prediction: `handleGetPasses()`, `handleSchedulePass()`, `handleCancelPass()`, `handleGetSchedule()`, `handleClearSchedule()`
- Doppler: `handleGetDoppler()`
- External APIs: `handleSetN2YOKey()`, `handleFetchN2YOTLE()`, `handleGetVisualPasses()`, `handleSearchSatNOGS()`, `handleGetTransmitters()`, `handleFetchTLEGroup()`
- EME mode: `handleEnableEME()`, `handleDisableEME()`, `handleGetMoonInfo()`, `handleGetEMEPathLoss()`
- Geo satellites: `handlePointToGeo()`, `handleEnableGeoTracking()`, `handleDisableGeoTracking()`
- Antenna pattern: `handleGetAntennaPattern()`, `handleSetAntennaPattern()`
- Pre-positioning: `handleSetPrePosition()`

### web_handlers_camera.cpp (~170 lines)
Contains:
- Snapshots: `handleCameraSnapshot()`, `handleCameraSnapshotWithOverlay()`
- Streaming: `handleCameraStream()`
- Settings: `handleCameraSettings()`
- Detection: `handleCameraDetect()`

### web_handlers_system.cpp (~240 lines)
Contains:
- Health monitoring: `handleGetHealth()`, `handleGetHealthEvents()`, `handleClearHealthEvents()`
- SD card: `handleSDStatus()`, `handleSDListTLEs()`, `handleSDImportTLE()`, `handleSDExportTLEs()`, `handleSDBackupConfig()`, `handleSDRestoreConfig()`, `handleGetPassLog()`, `handleExportPassLog()`
- UDP broadcast: `handleUDPConfig()`, `handleUDPStatus()`

## Benefits
- Easier navigation and maintenance
- Logical grouping of related functionality
- Faster compilation (only changed files need recompiling)
- Clear separation of concerns

## Note
All handler functions remain methods of the `WebServer` class - they're just implemented in separate .cpp files. This is standard C++ practice where class method implementations can be split across multiple compilation units.
