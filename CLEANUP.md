# Cleanup 2026-10-21

Removed synthetic tests, bench scenarios and old verification artifacts from the
working package. Runtime nodes, VINS/Kalibr, calibration recordings/configuration,
IMU/camera checks and displacement display remain. No PX4 parameters changed.

Full pre-cleanup repository and compact diagnostic reports: `/home/jetson/vio_backups/cleanup_20261021_f5a3fb79`.
The old EuRoC sample bag and failed VINS-run bags were deleted; their full sensor
recordings are not in the backup. Manifest: `/home/jetson/vio_backups/cleanup_20261021_f5a3fb79/manifest.json`.
Runtime service was not restarted during file cleanup.

Old home-directory heartbeat demo, setup scripts, source tar and topic listing
were moved to the backup legacy_home/ directory. Package build succeeded;
service remained active, adapter ACTIVE, IMU ~199.82 Hz and camera ~29.99 Hz.
