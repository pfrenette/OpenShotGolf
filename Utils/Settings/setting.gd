class_name Setting
extends RefCounted

signal setting_changed(val)

var value : Variant
var default : Variant
var min_value : Variant = null
var max_value : Variant = null

func _init(def: Variant, minimum: Variant = null, maximum: Variant = null):
	min_value = minimum
	max_value = maximum
	value = def
	default = def
	
func reset_default():
	value = default
	emit_signal("setting_changed", value)

func set_value(val: Variant):
	if min_value and value < min_value:
		value = min_value
	elif max_value and value > max_value:
		value = max_value
	else:
		value = val
		
	emit_signal("setting_changed", value)
	
func set_default(def: Variant):
	default = def
	emit_signal("setting_changed", value)
	
