## Architecture

* AssetManager
  + Job: Maintains connection with hermies app and make sure assets requisted is ready to use.
* ImuManager
* GnssManager
* SpeakerManager
* CameraManager

## Sequence

* Start listning to port for asset state.
* Subscribe to asset state.
* When you receive asset state, start assets.
* Stop asset when process is terminated.
