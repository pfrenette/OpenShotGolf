extends CharacterBody3D

var omega := Vector3.ZERO
var on_ground := false
var floor_norm := Vector3(0.0, 1.0, 0.0)
var temperature: float = GlobalSettings.range_settings.temperature.value # Using global settings

var mass = 0.04592623
var radius = 0.021335
var A = PI*radius*radius # Cross-sectional area
var I = 0.4*mass*radius*radius # Moment of inertia
var u_k = 0.15 # kinetic friction; surface-driven
var u_kr = 0.05 # rolling friction; surface-driven
var theta_c = 0.30 # critical bounce angle in radians (~17°); surface-driven

var airDensity = Coefficients.get_air_density(0.0, temperature)
var dynamicAirViscosity = Coefficients.get_dynamic_air_viscosity(temperature)
# Spin decay time constant (seconds) - tuned to match GSPro behavior
# Note: With fixed Cl model (1.1*S + 0.05), tau = 3.0 gives correct spin decay
var spin_decay_tau = 3.0
var nu_g = 0.0005 # Grass drag viscosity; surface-driven
var drag_cf: float # Drag correction factor (set from GlobalSettings)
var lift_cf: float # Lift correction factor (set from GlobalSettings)
var surface_type: int = Enums.Surface.FAIRWAY

var state : Enums.BallState = Enums.BallState.REST

# --- NEW: shot reference for measuring downrange distance correctly ---
var shot_start_pos := Vector3.ZERO
var shot_dir := Vector3(0.0, 0.0, 1.0) # normalized horizontal-ish direction

# --- Store launch spin for bounce calculations (spin decays during flight) ---
var launch_spin_rpm := 0.0

signal rest

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	GlobalSettings.range_settings.range_units.setting_changed.connect(set_env)
	GlobalSettings.range_settings.temperature.setting_changed.connect(set_env)
	GlobalSettings.range_settings.altitude.setting_changed.connect(set_env)
	GlobalSettings.range_settings.drag_scale.setting_changed.connect(_on_drag_scale_changed)
	GlobalSettings.range_settings.lift_scale.setting_changed.connect(_on_lift_scale_changed)
	drag_cf = GlobalSettings.range_settings.drag_scale.value
	lift_cf = GlobalSettings.range_settings.lift_scale.value
	set_env(null) # Sync air properties to current settings on startup
	_apply_surface(surface_type)


func _is_ground_normal(n: Vector3) -> bool:
	# Collision normals from Godot are always normalized, so n.y == n.dot(Vector3.UP)
	return n.y > 0.7


# --- NEW: compute downrange distance along shot_dir (GSPro-like) ---
func get_downrange_yards() -> float:
	var delta: Vector3 = position - shot_start_pos
	# measure along initial shot direction, ignore sign mistakes by allowing negative if it happens
	var meters: float = delta.dot(shot_dir)
	return meters * 1.09361


func _physics_process(delta: float) -> void:
	# Track previous state for bounce detection
	var was_on_ground := on_ground
	var prev_velocity := velocity
	
	var integration_results := {}

	# Use ground state from PREVIOUS frame for force calculations
	if was_on_ground:
		integration_results = BallPhysics.integrate_ground_physics(delta, velocity, omega, floor_norm)
	else: # ball in air
		integration_results = BallPhysics.integrate_air_physics(delta, velocity, omega)
		
	velocity = integration_results["velocity"]
	omega = integration_results["omega"]

	# Safety check: prevent ball from going too far out of bounds
	if abs(position.x) > 1000.0 or abs(position.z) > 1000.0:
		print("WARNING: Ball went out of bounds at position: ", position)
		velocity = Vector3.ZERO
		omega = Vector3.ZERO
		state = Enums.BallState.REST
		emit_signal("rest")
		return

	# Safety check: prevent ball from falling through ground
	if position.y < -0.5:
		print("WARNING: Ball fell through ground at position: ", position, " - resetting to y=0.0")
		position.y = 0.0
		velocity = Vector3.ZERO
		omega = Vector3.ZERO
		state = Enums.BallState.REST
		emit_signal("rest")
		return

	# Move and detect collision
	var collision = move_and_collide(velocity * delta)

	if collision:
		var normal = collision.get_normal()

		if _is_ground_normal(normal):
			floor_norm = normal

			# Bounce if: first impact (FLIGHT) OR subsequent airborne landing
			var is_landing = (state == Enums.BallState.FLIGHT) or prev_velocity.y < -0.5

			if is_landing:
				if state == Enums.BallState.FLIGHT:
					# Keep for debugging later. 
					print("FIRST IMPACT at pos: ", position, ", downrange: %.2f yds" % get_downrange_yards())
					print("  Velocity at impact: ", velocity, " (%.2f m/s)" % velocity.length())
					print("  Spin at impact: ", omega, " (%.0f rpm)" % (omega.length()/0.10472))
					print("  Normal: ", normal)
					state = Enums.BallState.ROLLOUT
				var results = BallPhysics.bounce(velocity, omega, normal)
				velocity = results["velocity"]
				omega = results["omega"]
				print("  Velocity after bounce: ", velocity, " (%.2f m/s)" % velocity.length())
				on_ground = false
			else:
				# On ground, not bouncing
				on_ground = true
				if velocity.y < 0:
					velocity.y = 0
		else:
			# Non-ground collision (wall, etc.) - damped reflection
			on_ground = false
			floor_norm = Vector3(0.0, 1.0, 0.0)
			velocity = velocity.bounce(normal) * 0.30
	else:
		# No collision - check rolling continuity for non-flight states
		if state != Enums.BallState.FLIGHT and was_on_ground and position.y < 0.02 and velocity.y <= 0.0:
			on_ground = true
		else:
			on_ground = false
			floor_norm = Vector3(0.0, 1.0, 0.0)

	# Rest detection
	if velocity.length() < 0.1 and state != Enums.BallState.REST:
		state = Enums.BallState.REST
		velocity = Vector3.ZERO
		emit_signal("rest")





func hit():
	var data : Dictionary = {
		"Speed": 100.0,
		"VLA": 22.0,
		"HLA": -3.1,
		"TotalSpin": 6000.0,
		"SpinAxis": 3.5,
		"Temp": 25.0,
		"Altitude": 0.0
	}

	state = Enums.BallState.FLIGHT
	on_ground = false
	position = Vector3(0.0, 0.05, 0.0)

	velocity = Vector3(data["Speed"]*0.44704, 0, 0).rotated(
					Vector3(0.0, 0.0, 1.0), data["VLA"]*PI/180.0).rotated(
						Vector3(0.0, 1.0, 0.0), -data["HLA"]*PI/180.0)

	# NEW: set shot start + direction for correct distance measurement
	shot_start_pos = position
	var v_flat: Vector3 = Vector3(velocity.x, 0.0, velocity.z)
	shot_dir = v_flat.normalized() if v_flat.length() > 0.001 else Vector3(0.0, 0.0, 1.0)

	omega = Vector3(0.0, 0.0, data["TotalSpin"]*0.10472).rotated(Vector3(1.0, 0.0, 0.0), data["SpinAxis"]*PI/180.0)
	launch_spin_rpm = data["TotalSpin"]


func hit_from_data(data : Dictionary):
	var speed_mps: float = (data.get("Speed", 0.0) as float)*0.44704
	var vla_deg: float = data.get("VLA", 0.0) as float
	var hla_deg: float = data.get("HLA", 0.0) as float
	var has_backspin := data.has("BackSpin")
	var has_sidespin := data.has("SideSpin")
	var has_total := data.has("TotalSpin")
	var has_axis := data.has("SpinAxis")
	var backspin: float = (data.get("BackSpin", 0.0) as float)
	var sidespin: float = (data.get("SideSpin", 0.0) as float)
	var total_spin: float = (data.get("TotalSpin", 0.0) as float)
	var spin_axis: float = (data.get("SpinAxis", 0.0) as float)

	if total_spin == 0.0 and (has_backspin or has_sidespin):
		total_spin = sqrt(backspin*backspin + sidespin*sidespin)
	if not has_axis and (has_backspin or has_sidespin):
		spin_axis = rad_to_deg(atan2(sidespin, backspin))
	if has_total and has_axis:
		if not has_backspin:
			backspin = total_spin * cos(deg_to_rad(spin_axis))
		if not has_sidespin:
			sidespin = total_spin * sin(deg_to_rad(spin_axis))

	state = Enums.BallState.FLIGHT
	on_ground = false
	position = Vector3(0.0, 0.05, 0.0)

	velocity = Vector3(speed_mps, 0, 0).rotated(
					Vector3(0.0, 0.0, 1.0), vla_deg*PI/180.0).rotated(
						Vector3(0.0, 1.0, 0.0), -hla_deg*PI/180.0)

	# NEW: set shot start + direction for correct distance measurement
	shot_start_pos = position
	var v_flat: Vector3 = Vector3(velocity.x, 0.0, velocity.z)
	shot_dir = v_flat.normalized() if v_flat.length() > 0.001 else Vector3(0.0, 0.0, 1.0)

	if total_spin == 0.0:
		omega = Vector3(sidespin*0.10472, 0.0, backspin*0.10472)
	else:
		omega = Vector3(0.0, 0.0, total_spin*0.10472).rotated(Vector3(1.0, 0.0, 0.0), spin_axis*PI/180)

	# Store launch spin for bounce calculations (spin decays during flight)
	launch_spin_rpm = total_spin

	# Leave these in. We'll revisit again--I am sure.
	print("=== SHOT DEBUG ===")
	print("Speed: %.2f mph (%.2f m/s)" % [data.get("Speed", 0.0), speed_mps])
	print("VLA: %.2f°, HLA: %.2f°" % [vla_deg, hla_deg])
	print("Spin: %.0f rpm, Axis: %.2f°" % [total_spin, spin_axis])
	print("drag_cf: %.2f, lift_cf: %.2f" % [drag_cf, lift_cf])
	print("Air density: %.4f kg/m³" % airDensity)
	print("Dynamic viscosity: %.11f kg/(m·s)" % dynamicAirViscosity)
	var Re_initial = airDensity * speed_mps * radius * 2.0 / dynamicAirViscosity
	var spin_ratio = (total_spin * 0.10472) * radius / speed_mps if speed_mps > 0.1 else 0.0
	var Cl_initial = Coefficients.get_Cl(Re_initial, spin_ratio)
	print("Reynolds number (initial): %.0f" % Re_initial)
	print("Spin ratio (initial): %.3f" % spin_ratio)
	print("Cl (initial, before lift_cf): %.3f, after: %.3f" % [Cl_initial, Cl_initial * lift_cf])
	print("Initial velocity: ", velocity)
	print("Initial omega: ", omega, " (%.0f rpm)" % (omega.length()/0.10472))
	print("Shot dir (flat): ", shot_dir)
	print("===================")


func set_env(_value):
	airDensity = Coefficients.get_air_density(GlobalSettings.range_settings.altitude.value,
		 GlobalSettings.range_settings.temperature.value)
	dynamicAirViscosity = Coefficients.get_dynamic_air_viscosity(GlobalSettings.range_settings.temperature.value)


func reset():
	position = Vector3(0.0, 0.1, 0.0)
	velocity = Vector3.ZERO
	omega = Vector3.ZERO
	launch_spin_rpm = 0.0
	state = Enums.BallState.REST
	on_ground = false


func _on_drag_scale_changed(_value):
	drag_cf = GlobalSettings.range_settings.drag_scale.value


func _on_lift_scale_changed(_value):
	lift_cf = GlobalSettings.range_settings.lift_scale.value


func set_surface(surface: int) -> void:
	surface_type = surface as Enums.Surface
	_apply_surface(surface_type)


func _apply_surface(surface: int) -> void:
	var params := SurfaceUtil.get_params(surface as Enums.Surface)
	u_k = params["u_k"]
	u_kr = params["u_kr"]
	nu_g = params["nu_g"]
	theta_c = params["theta_c"]
