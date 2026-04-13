from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, TypedDict


@dataclass(frozen=True)
class Table:
	name: str
	columns: list[str]


SQLiteScalar = str | int | float | None


class _UWBPositionsTable(Table):
	def __init__(self, name: str = "uwb_positions"):
		super().__init__(name, ["timestamp", "tag_id", "x", "y", "z", "quality"])

	@classmethod
	def build_row(
		cls,
		timestamp: float,
		tag_id: SQLiteScalar,
		x: SQLiteScalar,
		y: SQLiteScalar,
		z: SQLiteScalar,
		quality: SQLiteScalar,
	) -> UWBPositionRow:
		return UWBPositionRow(timestamp=timestamp, tag_id=tag_id, x=x, y=y, z=z, quality=quality)


class _UWBAnchorsTable(Table):
	def __init__(self, name: str = "uwb_anchors"):
		super().__init__(name, ["timestamp", "port", "name", "id", "x", "y", "z", "range"])

	@classmethod
	def build_row(
		cls,
		timestamp: float,
		port: SQLiteScalar,
		name: SQLiteScalar,
		anchor_id: SQLiteScalar,
		x: SQLiteScalar,
		y: SQLiteScalar,
		z: SQLiteScalar,
		range_value: SQLiteScalar,
	) -> UWBAnchorRow:
		return UWBAnchorRow(
			timestamp=timestamp,
			port=port,
			name=name,
			id=anchor_id,
			x=x,
			y=y,
			z=z,
			range=range_value,
		)


class _StateEstimatorTable(Table):
	def __init__(self, name: str = "state_estimator"):
		super().__init__(name, ["timestamp", "px", "py", "pz", "vx", "vy", "vz", "yaw", "pitch", "roll"])

	@classmethod
	def build_row(
		cls,
		timestamp: float,
		px: SQLiteScalar,
		py: SQLiteScalar,
		pz: SQLiteScalar,
		vx: SQLiteScalar,
		vy: SQLiteScalar,
		vz: SQLiteScalar,
		yaw: SQLiteScalar,
		pitch: SQLiteScalar,
		roll: SQLiteScalar,
	) -> StateEstimatorRow:
		return StateEstimatorRow(
			timestamp=timestamp,
			px=px,
			py=py,
			pz=pz,
			vx=vx,
			vy=vy,
			vz=vz,
			yaw=yaw,
			pitch=pitch,
			roll=roll,
		)


class _IMUOrientationTable(Table):
	def __init__(self, name: str = "imu_orientation"):
		super().__init__(name, ["timestamp", "yaw", "pitch", "roll", "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz"])

	@classmethod
	def build_row(
		cls,
		timestamp: float,
		yaw: SQLiteScalar,
		pitch: SQLiteScalar,
		roll: SQLiteScalar,
		accel: Iterable[SQLiteScalar] | None,
		gyro: Iterable[SQLiteScalar] | None,
		mag: Iterable[SQLiteScalar] | None,
	) -> IMUOrientationRow:
		def _triplet(values: Iterable[SQLiteScalar] | None) -> tuple[SQLiteScalar, SQLiteScalar, SQLiteScalar]:
			if values is None:
				return "", "", ""

			a, b, c = tuple(values)
			return (
				"" if a is None else a,
				"" if b is None else b,
				"" if c is None else c,
			)

		ax, ay, az = _triplet(accel)
		gx, gy, gz = _triplet(gyro)
		mx, my, mz = _triplet(mag)

		return IMUOrientationRow(
			timestamp=timestamp,
			yaw=yaw,
			pitch=pitch,
			roll=roll,
			ax=ax,
			ay=ay,
			az=az,
			gx=gx,
			gy=gy,
			gz=gz,
			mx=mx,
			my=my,
			mz=mz,
		)


class _EncoderDataTable(Table):
	def __init__(self, name: str = "encoder_data"):
		super().__init__(name, ["timestamp", "count", "velocity"])

	@classmethod
	def build_row(cls, timestamp: float, count: SQLiteScalar, velocity: SQLiteScalar) -> EncoderDataRow:
		return EncoderDataRow(timestamp=timestamp, count=count, velocity=velocity)


class _ControlInputsTable(Table):
	def __init__(self, name: str = "control_inputs"):
		super().__init__(name, ["timestamp", "velocity_mps", "steering_angle_rad", "steering_angle_deg"])

	@classmethod
	def build_row(
		cls,
		timestamp: float,
		velocity_mps: SQLiteScalar,
		steering_angle_rad: SQLiteScalar,
		steering_angle_deg: SQLiteScalar,
	) -> ControlInputsRow:
		return ControlInputsRow(
			timestamp=timestamp,
			velocity_mps=velocity_mps,
			steering_angle_rad=steering_angle_rad,
			steering_angle_deg=steering_angle_deg,
		)


class _PathFollowingTable(Table):
	def __init__(self, name: str = "path_following"):
		super().__init__(name, ["timestamp", "motor_speed_mps", "steering_angle_rad", "steering_angle_deg"])

	@classmethod
	def build_row(
		cls,
		timestamp: float,
		motor_speed_mps: SQLiteScalar,
		steering_angle_rad: SQLiteScalar,
		steering_angle_deg: SQLiteScalar,
	) -> PathFollowingRow:
		return PathFollowingRow(
			timestamp=timestamp,
			motor_speed_mps=motor_speed_mps,
			steering_angle_rad=steering_angle_rad,
			steering_angle_deg=steering_angle_deg,
		)


class _YawOffsetTable(Table):
	def __init__(self, name: str = "yaw_offset"):
		super().__init__(name, ["timestamp", "source", "yaw_offset_deg", "yaw_offset_rad"])

	@classmethod
	def build_row(
		cls,
		timestamp: float,
		source: SQLiteScalar,
		yaw_offset_deg: SQLiteScalar,
		yaw_offset_rad: SQLiteScalar,
	) -> YawOffsetRow:
		return YawOffsetRow(
			timestamp=timestamp,
			source=source,
			yaw_offset_deg=yaw_offset_deg,
			yaw_offset_rad=yaw_offset_rad,
		)


class _HomePositionTable(Table):
	def __init__(self, name: str = "home_position"):
		super().__init__(name, ["x", "y", "yaw"])

	@classmethod
	def build_row(cls, x: SQLiteScalar, y: SQLiteScalar, yaw: SQLiteScalar) -> HomePositionRow:
		return HomePositionRow(x=x, y=y, yaw=yaw)


class _PathPointsTable(Table):
	def __init__(self, name: str = "path_points"):
		super().__init__(name, ["x", "y", "yaw"])

	@classmethod
	def build_row(cls, x: SQLiteScalar, y: SQLiteScalar, yaw: SQLiteScalar) -> PathPointRow:
		return PathPointRow(x=x, y=y, yaw=yaw)


UWB_POSITIONS_TABLE = _UWBPositionsTable()
UWB_ANCHORS_TABLE = _UWBAnchorsTable()
STATE_ESTIMATOR_TABLE = _StateEstimatorTable()
IMU_ORIENTATION_TABLE = _IMUOrientationTable()
ENCODER_DATA_TABLE = _EncoderDataTable()
CONTROL_INPUTS_TABLE = _ControlInputsTable()
PATH_FOLLOWING_TABLE = _PathFollowingTable()
YAW_OFFSET_TABLE = _YawOffsetTable()
HOME_POSITION_TABLE = _HomePositionTable()
PATH_POINTS_TABLE = _PathPointsTable()


class IMUOrientationRow(TypedDict):
	timestamp: float
	yaw: SQLiteScalar
	pitch: SQLiteScalar
	roll: SQLiteScalar
	ax: SQLiteScalar
	ay: SQLiteScalar
	az: SQLiteScalar
	gx: SQLiteScalar
	gy: SQLiteScalar
	gz: SQLiteScalar
	mx: SQLiteScalar
	my: SQLiteScalar
	mz: SQLiteScalar


class UWBPositionRow(TypedDict):
	timestamp: float
	tag_id: SQLiteScalar
	x: SQLiteScalar
	y: SQLiteScalar
	z: SQLiteScalar
	quality: SQLiteScalar


class UWBAnchorRow(TypedDict):
	timestamp: float
	port: SQLiteScalar
	name: SQLiteScalar
	id: SQLiteScalar
	x: SQLiteScalar
	y: SQLiteScalar
	z: SQLiteScalar
	range: SQLiteScalar


class StateEstimatorRow(TypedDict):
	timestamp: float
	px: SQLiteScalar
	py: SQLiteScalar
	pz: SQLiteScalar
	vx: SQLiteScalar
	vy: SQLiteScalar
	vz: SQLiteScalar
	yaw: SQLiteScalar
	pitch: SQLiteScalar
	roll: SQLiteScalar


class EncoderDataRow(TypedDict):
	timestamp: float
	count: SQLiteScalar
	velocity: SQLiteScalar


class ControlInputsRow(TypedDict):
	timestamp: float
	velocity_mps: SQLiteScalar
	steering_angle_rad: SQLiteScalar
	steering_angle_deg: SQLiteScalar


class PathFollowingRow(TypedDict):
	timestamp: float
	motor_speed_mps: SQLiteScalar
	steering_angle_rad: SQLiteScalar
	steering_angle_deg: SQLiteScalar


class YawOffsetRow(TypedDict):
	timestamp: float
	source: SQLiteScalar
	yaw_offset_deg: SQLiteScalar
	yaw_offset_rad: SQLiteScalar


class HomePositionRow(TypedDict):
	x: SQLiteScalar
	y: SQLiteScalar
	yaw: SQLiteScalar


class PathPointRow(TypedDict):
	x: SQLiteScalar
	y: SQLiteScalar
	yaw: SQLiteScalar
