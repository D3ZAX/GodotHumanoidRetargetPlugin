@tool
extends SkeletonModifier3D
class_name HumanoidRetargetModifier

# Source skeleton reference
var _source_skeleton: Skeleton3D
@export var source_skeleton: Skeleton3D:
	get:
		return _source_skeleton
	set(value):
		if _source_skeleton != value:
			_source_skeleton = value
			align_rest()

# Target skeleton reference
var target_skeleton: Skeleton3D

# Bone chains used for directional alignment
var body_chains := [
	["LeftShoulder", "LeftUpperArm", "LeftLowerArm", "LeftHand"],
	["RightShoulder", "RightUpperArm", "RightLowerArm", "RightHand"],

	["LeftUpperLeg", "LeftLowerLeg", "LeftFoot"],
	["RightUpperLeg", "RightLowerLeg", "RightFoot"],

	["LeftHand", "LeftMiddleProximal"],
	["RightHand", "RightMiddleProximal"],

	["LeftThumbProximal",  "LeftThumbIntermediate",  "LeftThumbDistal"],
	["LeftIndexProximal",  "LeftIndexIntermediate",  "LeftIndexDistal"],
	["LeftMiddleProximal", "LeftMiddleIntermediate", "LeftMiddleDistal"],
	["LeftRingProximal",   "LeftRingIntermediate",   "LeftRingDistal"],
	["LeftLittleProximal", "LeftLittleIntermediate", "LeftLittleDistal"],

	["RightThumbProximal",  "RightThumbIntermediate",  "RightThumbDistal"],
	["RightIndexProximal",  "RightIndexIntermediate",  "RightIndexDistal"],
	["RightMiddleProximal", "RightMiddleIntermediate", "RightMiddleDistal"],
	["RightRingProximal",   "RightRingIntermediate",   "RightRingDistal"],
	["RightLittleProximal", "RightLittleIntermediate", "RightLittleDistal"],
]

# Humanoid bone list
var humanoid_bones := [
	"Hips", "Spine", "Spine1", "Spine2", "Neck", "Head",

	"LeftShoulder", "LeftUpperArm", "LeftLowerArm", "LeftHand",
	"RightShoulder", "RightUpperArm", "RightLowerArm", "RightHand",

	"LeftUpperLeg", "LeftLowerLeg", "LeftFoot", "LeftToe",
	"RightUpperLeg", "RightLowerLeg", "RightFoot", "RightToe",

	"LeftThumbProximal",  "LeftThumbIntermediate",  "LeftThumbDistal",
	"LeftIndexProximal",  "LeftIndexIntermediate",  "LeftIndexDistal",
	"LeftMiddleProximal", "LeftMiddleIntermediate", "LeftMiddleDistal",
	"LeftRingProximal",   "LeftRingIntermediate",   "LeftRingDistal",
	"LeftLittleProximal", "LeftLittleIntermediate", "LeftLittleDistal",

	"RightThumbProximal",  "RightThumbIntermediate",  "RightThumbDistal",
	"RightIndexProximal",  "RightIndexIntermediate",  "RightIndexDistal",
	"RightMiddleProximal", "RightMiddleIntermediate", "RightMiddleDistal",
	"RightRingProximal",   "RightRingIntermediate",   "RightRingDistal",
	"RightLittleProximal", "RightLittleIntermediate", "RightLittleDistal",
]

# name → rotation offset (Quaternion)
var _bone_rotation_offset := {}

# Hip height offset after directional alignment
var _hip_pose_y_offset := 0.0


func _ready():
	target_skeleton = get_skeleton()
	align_rest()


func _process_modification_with_delta(delta: float) -> void:
	sync_from_source()


# ---------------------------------------------------------
# Reset target skeleton to rest pose and align with source
# ---------------------------------------------------------
func align_rest():
	if _source_skeleton == null or target_skeleton == null:
		return

	reset_target_pose()
	align_bones()
	record_humanoid_offsets()


# Reset all bones to rest pose
func reset_target_pose():
	for i in target_skeleton.get_bone_count():
		target_skeleton.set_bone_pose(i, target_skeleton.get_bone_rest(i))


# Align all bone chains
func align_bones():
	for chain in body_chains:
		align_chain(chain)

	align_hip_position()


# Align a chain of bones
func align_chain(chain: Array):
	for i in chain.size() - 1:
		align_bone_direction(chain[i], chain[i + 1])


# Align direction of bone A → bone B
func align_bone_direction(bone_a: String, bone_b: String):
	var src_a = _source_skeleton.find_bone(bone_a)
	var src_b = _source_skeleton.find_bone(bone_b)
	var dst_a = target_skeleton.find_bone(bone_a)
	var dst_b = target_skeleton.find_bone(bone_b)

	if src_a == -1 or src_b == -1 or dst_a == -1 or dst_b == -1:
		return

	var src_a_rest = _source_skeleton.get_bone_global_rest(src_a)
	var src_b_rest = _source_skeleton.get_bone_global_rest(src_b)
	var src_dir = (src_b_rest.origin - src_a_rest.origin).normalized()

	var dst_a_pose = target_skeleton.get_bone_global_pose(dst_a)
	var dst_b_pose = target_skeleton.get_bone_global_pose(dst_b)
	var dst_dir = (dst_b_pose.origin - dst_a_pose.origin).normalized()

	if src_dir.length() < 0.0001 or dst_dir.length() < 0.0001:
		return

	var q_dir = Quaternion(dst_dir, src_dir)
	var target_global_basis = Basis(q_dir) * dst_a_pose.basis

	var parent = target_skeleton.get_bone_parent(dst_a)
	var local_rot: Quaternion

	if parent == -1:
		local_rot = target_global_basis.get_rotation_quaternion()
	else:
		var parent_global = target_skeleton.get_bone_global_pose(parent)
		var local_basis = parent_global.basis.inverse() * target_global_basis
		local_rot = local_basis.get_rotation_quaternion()

	target_skeleton.set_bone_pose_rotation(dst_a, local_rot)


# ---------------------------------------------------------
# Hip alignment
# ---------------------------------------------------------
func align_hip_position():
	_hip_pose_y_offset = compute_hip_height_offset()

	var hip = target_skeleton.find_bone("Hips")
	if hip == -1:
		return

	var hip_rest = target_skeleton.get_bone_global_rest(hip)
	var desired_global_pos = hip_rest.origin + Vector3(0, _hip_pose_y_offset, 0)

	var parent = target_skeleton.get_bone_parent(hip)
	var local_pos: Vector3

	if parent == -1:
		local_pos = desired_global_pos
	else:
		var parent_global = target_skeleton.get_bone_global_pose(parent)
		local_pos = parent_global.affine_inverse() * desired_global_pos

	target_skeleton.set_bone_pose_position(hip, local_pos)


# Compute hip height offset after directional alignment
func compute_hip_height_offset() -> float:
	var hip = target_skeleton.find_bone("Hips")
	var foot = target_skeleton.find_bone("LeftFoot")

	if hip == -1 or foot == -1:
		return 0.0

	var hip_rest = target_skeleton.get_bone_global_rest(hip).origin

	var foot_before = target_skeleton.get_bone_global_rest(foot).origin
	var dir_before = foot_before - hip_rest
	var before_len = 0.0

	if abs(dir_before.y) > 1e-5:
		var t_before = -hip_rest.y / dir_before.y
		var ground_before = hip_rest + dir_before * t_before
		before_len = (hip_rest - ground_before).length()

	var foot_after = target_skeleton.get_bone_global_pose(foot).origin
	var dir_after = foot_after - hip_rest
	var after_len = 0.0

	if abs(dir_after.y) > 1e-5:
		var t_after = -hip_rest.y / dir_after.y
		var ground_after = hip_rest + dir_after * t_after
		after_len = (hip_rest - ground_after).length()

	return (before_len - after_len) / before_len * hip_rest.y


# ---------------------------------------------------------
# Record rest → pose rotation offsets for humanoid bones
# ---------------------------------------------------------
func record_humanoid_offsets():
	_bone_rotation_offset.clear()

	for i in target_skeleton.get_bone_count():
		var name = target_skeleton.get_bone_name(i)
		if not humanoid_bones.has(name):
			continue

		var rest_g = target_skeleton.get_bone_global_rest(i)
		var pose_g = target_skeleton.get_bone_global_pose(i)

		var offset_basis = pose_g.basis * rest_g.basis.inverse()
		_bone_rotation_offset[name] = offset_basis.get_rotation_quaternion()


# ---------------------------------------------------------
# Apply animation from source skeleton to target skeleton
# ---------------------------------------------------------
func sync_from_source():
	if _source_skeleton == null or target_skeleton == null:
		return

	for i in _source_skeleton.get_bone_count():
		var name = _source_skeleton.get_bone_name(i)
		if not _bone_rotation_offset.has(name):
			continue

		var dst = target_skeleton.find_bone(name)
		if dst == -1:
			continue

		var src_rest = _source_skeleton.get_bone_global_rest(i)
		var src_pose = _source_skeleton.get_bone_global_pose(i)

		var anim_basis = src_pose.basis * src_rest.basis.inverse()
		var q_anim = anim_basis.get_rotation_quaternion()

		var dst_rest = target_skeleton.get_bone_global_rest(dst)
		var final_basis = Basis(q_anim) * (Basis(_bone_rotation_offset[name]) * dst_rest.basis)

		var parent = target_skeleton.get_bone_parent(dst)
		var local_rot: Quaternion

		if parent == -1:
			local_rot = final_basis.get_rotation_quaternion()
		else:
			var parent_global = target_skeleton.get_bone_global_pose(parent)
			var local_basis = parent_global.basis.inverse() * final_basis
			local_rot = local_basis.get_rotation_quaternion()

		target_skeleton.set_bone_pose_rotation(dst, local_rot)

	sync_hip_position()


# Apply hip translation from source skeleton
func sync_hip_position():
	var hip = target_skeleton.find_bone("Hips")
	if hip == -1:
		return

	var hip_offset = _hip_pose_y_offset

	var dst_rest_g = target_skeleton.get_bone_global_rest(hip)
	var dst_rest_pos = dst_rest_g.origin

	var src_hip = _source_skeleton.find_bone("Hips")
	var src_rest_g = _source_skeleton.get_bone_global_rest(src_hip)
	var src_pose_g = _source_skeleton.get_bone_global_pose(src_hip)

	var src_delta = src_pose_g.origin - src_rest_g.origin
	src_delta.y *= (dst_rest_pos.y + hip_offset) / src_rest_g.origin.y

	var desired_global_pos = dst_rest_pos \
		+ Vector3(0, hip_offset, 0) \
		+ src_delta

	var parent = target_skeleton.get_bone_parent(hip)
	var local_pos: Vector3

	if parent == -1:
		local_pos = desired_global_pos
	else:
		var parent_global = target_skeleton.get_bone_global_pose(parent)
		local_pos = parent_global.affine_inverse() * desired_global_pos

	target_skeleton.set_bone_pose_position(hip, local_pos)
