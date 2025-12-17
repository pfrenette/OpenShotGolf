extends Object
class_name BallPhysics


static var temperature: float = GlobalSettings.range_settings.temperature.value # Using global settings

const MASS = 0.04592623
const RADIUS = 0.021335
const A = PI*RADIUS*RADIUS # Cross-sectional area
const I = 0.4*MASS*RADIUS*RADIUS # Moment of inertia
const U_K = 0.15 # kinetic friction; surface-driven
const U_KR = 0.05 # rolling friction; surface-driven
const THETA_C = 0.30 # critical bounce angle in radians (~17°); surface-driven

static var airDensity = Coefficients.get_air_density(0.0, temperature)
static var dynamicAirViscosity = Coefficients.get_dynamic_air_viscosity(temperature)
# Spin decay time constant (seconds) - tuned to match GSPro behavior
# Note: With fixed Cl model (1.1*S + 0.05), tau = 3.0 gives correct spin decay
static var spin_decay_tau = 3.0
static var nu_g = 0.0005 # Grass drag viscosity; surface-driven

static func integrate_air_physics(delta: float, velocity: Vector3, omega: Vector3) -> Dictionary:
	var F_g := Vector3(0.0, -9.81*MASS, 0) # force of gravity
	var F_m := Vector3.ZERO # Magnus force
	var F_d := Vector3.ZERO # Drag force
	var F_f := Vector3.ZERO # Frictional force
	var F_gd := Vector3.ZERO # Drag force from grass

	var T_d := Vector3.ZERO # Viscous torque
	var T_f := Vector3.ZERO # Frictional torque
	var T_g := Vector3.ZERO # Grass drag torque

	var speed := velocity.length()
	var spin := 0.0
	if speed > 0.5:
		spin = omega.length()*RADIUS/speed

	var Re : float = airDensity*speed*RADIUS*2.0/dynamicAirViscosity

	# Magnus, drag, and coefficients
	var Cl = Coefficients.get_Cl(Re, spin)
	var Cd = Coefficients.get_Cd(Re)

	# Magnus force
	var om_x_vel = omega.cross(velocity)
	var omega_len = omega.length()
	if omega_len > 0.1:
		F_m = 0.5*Cl*airDensity*A*om_x_vel*velocity.length()/omega.length()
	# Spin decay torque (empirical exponential decay model)
	# T = -I * omega / tau gives exponential decay with time constant tau
	T_d = -I * omega / spin_decay_tau
	# Drag force
	F_d = -0.5*Cd*airDensity*A*velocity*speed

	# Total force
	var F : Vector3 = F_g + F_d + F_m + F_f + F_gd

	# Total torque
	var T : Vector3 = T_d + T_f + T_g

	var vel = velocity + F/MASS*delta
	var om = omega + T/I*delta
	
	return {"velocity": vel, "omega": om}


static func integrate_ground_physics(delta: float, velocity: Vector3, omega: Vector3, floor_norm: Vector3) -> Dictionary:
	var F_g := Vector3(0.0, -9.81*MASS, 0) # force of gravity
	
	# Force of viscous drag from grass
	var F_gd = velocity*(-6*PI*RADIUS*nu_g)
	F_gd.y = 0.0

	# Contact point velocity: v_contact = v_center + omega × r_contact
	# where r_contact = -floor_norm * radius (from center to ground contact)
	var v_contact : Vector3 = velocity + omega.cross(-floor_norm * RADIUS)

	# FIX: friction should act in tangent plane only (remove normal component)
	var v_tan: Vector3 = v_contact - floor_norm * v_contact.dot(floor_norm)

	var F_f := Vector3.ZERO
	if v_tan.length() < 0.05: # rolling without slipping
		var v_flat: Vector3 = velocity - floor_norm * velocity.dot(floor_norm)
		var friction_dir = v_flat.normalized() if v_flat.length() > 0.01 else Vector3.ZERO
		F_f = friction_dir * (-U_KR * MASS * 9.81)  # Use rolling friction
	else: # ball slipping - kinetic friction
		var slip_dir = v_tan.normalized()
		F_f = slip_dir * (-U_K * MASS * 9.81)  # Use kinetic friction

	# Friction torque (applies in both rolling and slipping cases)
	var T_f := Vector3.ZERO
	if F_f.length() > 0.001:
		T_f = (-floor_norm * RADIUS).cross(F_f)

	# Viscous Torque
	var T_g = -6.0*PI*nu_g*RADIUS*omega
	
	# Total force
	var F : Vector3 = F_g + F_f + F_gd

	# Total torque
	var T : Vector3 = T_f + T_g

	var vel = velocity + F/MASS*delta
	var om = omega + T/I*delta
	
	return {"velocity": vel, "omega": om}


static func bounce(velocity: Vector3, omega: Vector3, normal: Vector3) -> Dictionary:
	# component of velocity parallel to floor normal
	var vel_norm : Vector3 = velocity.project(normal)
	var speed_norm : float = vel_norm.length()
	# component of velocity orthogonal to normal
	var vel_orth : Vector3 = velocity - vel_norm
	var speed_orth : float = vel_orth.length()
	# component of angular velocity parallel to normal
	var omg_norm : Vector3 = omega.project(normal)
	# component of angular velocity orthogonal to normal
	var omg_orth : Vector3 = omega - omg_norm

	var speed : float = velocity.length()
	var theta_1 : float = velocity.angle_to(normal)
	# theta_c is the critical angle (surface-dependent, set via _apply_surface)

	# final orthogonal speed
	# FIX: use signed normal spin component (length() loses sign)
	var spin_n: float = omega.dot(normal) # signed

	# Tangential retention: theoretical is 5/7 (0.714), real golf balls ~0.55-0.60
	# High backspin reduces forward momentum on landing (ball "checks up")
	# Use current spin at impact for realistic behavior
	var current_spin_rpm = omega.length() / 0.10472
	var spin_factor = clamp(1.0 - (current_spin_rpm / 8000.0), 0.40, 1.0)
	var tangential_retention: float = 0.55 * spin_factor

	var v2_orth = tangential_retention*speed*sin(theta_1-THETA_C) - 2.0*RADIUS*abs(spin_n)/7.0

	# orthogonal restitution - handle negative v2_orth (high spin case)
	if speed_orth < 0.01 or v2_orth <= 0.0:
		vel_orth = Vector3.ZERO
	else:
		vel_orth = vel_orth.limit_length(v2_orth)

	# final orthogonal angular velocity
	var w2h : float = v2_orth / RADIUS
	# orthogonal angular restitution - handle negative w2h
	if omg_orth.length() < 0.1 or w2h <= 0.0:
		omg_orth = Vector3.ZERO
	else:
		omg_orth = omg_orth.limit_length(w2h)

	# normal restitution (coefficient of restitution)
	# Golf ball COR on turf is typically 0.4-0.6 depending on surface and impact speed
	var e : float = 0.0
	if speed_norm > 20.0:
		e = 0.25  # High speed impacts
	elif speed_norm < 2.0:
		e = 0.0  # Kill very small bounces
	else:
		# Typical COR curve for golf ball on turf
		e = 0.45 - 0.0100*speed_norm + 0.0002*speed_norm*speed_norm

	vel_norm = vel_norm*-e

	var om = omg_norm + omg_orth
	var vel = vel_norm + vel_orth
	
	return {"velocity": vel, "omega": om}
