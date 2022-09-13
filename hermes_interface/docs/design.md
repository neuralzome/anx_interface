## Architecture

* AssetManager
  + Maintains connection with hermies app.
  + Update assets about state change.
  + Accept request assets to start and stop stream.
* ImuManager
  + Ensure streams requisted is streaming if available.
* GnssManager
  + Ensure streams requisted is streaming if available.
* SpeakerManager
  + Ensure streams requisted is streaming if available.
* CameraManager
  + Ensure streams requisted is streaming if available.

## Sequence

* Start listning to port for asset state.
* Subscribe to asset state.
* When you receive asset state, start assets.
* Stop asset when process is terminated.
