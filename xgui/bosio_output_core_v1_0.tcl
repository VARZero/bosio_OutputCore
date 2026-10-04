# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  ipgui::add_param $IPINST -name "CACHE_BYTES" -parent ${Page_0}
  ipgui::add_param $IPINST -name "CACHE_LINE_BYTES" -parent ${Page_0}
  ipgui::add_param $IPINST -name "CACHE_WAYS" -parent ${Page_0}
  ipgui::add_param $IPINST -name "C_M_AXI_ADDR_WIDTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "C_M_AXI_DATA_WIDTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "C_S_AXI_LITE_ADDR_WIDTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "C_S_AXI_LITE_DATA_WIDTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "DDR_CACHE_ENABLE" -parent ${Page_0}
  ipgui::add_param $IPINST -name "SENSOR_TDATA_WIDTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "VIDEO_TDATA_WIDTH" -parent ${Page_0}


}

proc update_PARAM_VALUE.CACHE_BYTES { PARAM_VALUE.CACHE_BYTES } {
	# Procedure called to update CACHE_BYTES when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.CACHE_BYTES { PARAM_VALUE.CACHE_BYTES } {
	# Procedure called to validate CACHE_BYTES
	return true
}

proc update_PARAM_VALUE.CACHE_LINE_BYTES { PARAM_VALUE.CACHE_LINE_BYTES } {
	# Procedure called to update CACHE_LINE_BYTES when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.CACHE_LINE_BYTES { PARAM_VALUE.CACHE_LINE_BYTES } {
	# Procedure called to validate CACHE_LINE_BYTES
	return true
}

proc update_PARAM_VALUE.CACHE_WAYS { PARAM_VALUE.CACHE_WAYS } {
	# Procedure called to update CACHE_WAYS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.CACHE_WAYS { PARAM_VALUE.CACHE_WAYS } {
	# Procedure called to validate CACHE_WAYS
	return true
}

proc update_PARAM_VALUE.C_M_AXI_ADDR_WIDTH { PARAM_VALUE.C_M_AXI_ADDR_WIDTH } {
	# Procedure called to update C_M_AXI_ADDR_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_M_AXI_ADDR_WIDTH { PARAM_VALUE.C_M_AXI_ADDR_WIDTH } {
	# Procedure called to validate C_M_AXI_ADDR_WIDTH
	return true
}

proc update_PARAM_VALUE.C_M_AXI_DATA_WIDTH { PARAM_VALUE.C_M_AXI_DATA_WIDTH } {
	# Procedure called to update C_M_AXI_DATA_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_M_AXI_DATA_WIDTH { PARAM_VALUE.C_M_AXI_DATA_WIDTH } {
	# Procedure called to validate C_M_AXI_DATA_WIDTH
	return true
}

proc update_PARAM_VALUE.C_S_AXI_LITE_ADDR_WIDTH { PARAM_VALUE.C_S_AXI_LITE_ADDR_WIDTH } {
	# Procedure called to update C_S_AXI_LITE_ADDR_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_S_AXI_LITE_ADDR_WIDTH { PARAM_VALUE.C_S_AXI_LITE_ADDR_WIDTH } {
	# Procedure called to validate C_S_AXI_LITE_ADDR_WIDTH
	return true
}

proc update_PARAM_VALUE.C_S_AXI_LITE_DATA_WIDTH { PARAM_VALUE.C_S_AXI_LITE_DATA_WIDTH } {
	# Procedure called to update C_S_AXI_LITE_DATA_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_S_AXI_LITE_DATA_WIDTH { PARAM_VALUE.C_S_AXI_LITE_DATA_WIDTH } {
	# Procedure called to validate C_S_AXI_LITE_DATA_WIDTH
	return true
}

proc update_PARAM_VALUE.DDR_CACHE_ENABLE { PARAM_VALUE.DDR_CACHE_ENABLE } {
	# Procedure called to update DDR_CACHE_ENABLE when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.DDR_CACHE_ENABLE { PARAM_VALUE.DDR_CACHE_ENABLE } {
	# Procedure called to validate DDR_CACHE_ENABLE
	return true
}

proc update_PARAM_VALUE.SENSOR_TDATA_WIDTH { PARAM_VALUE.SENSOR_TDATA_WIDTH } {
	# Procedure called to update SENSOR_TDATA_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.SENSOR_TDATA_WIDTH { PARAM_VALUE.SENSOR_TDATA_WIDTH } {
	# Procedure called to validate SENSOR_TDATA_WIDTH
	return true
}

proc update_PARAM_VALUE.VIDEO_TDATA_WIDTH { PARAM_VALUE.VIDEO_TDATA_WIDTH } {
	# Procedure called to update VIDEO_TDATA_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.VIDEO_TDATA_WIDTH { PARAM_VALUE.VIDEO_TDATA_WIDTH } {
	# Procedure called to validate VIDEO_TDATA_WIDTH
	return true
}


proc update_MODELPARAM_VALUE.C_S_AXI_LITE_DATA_WIDTH { MODELPARAM_VALUE.C_S_AXI_LITE_DATA_WIDTH PARAM_VALUE.C_S_AXI_LITE_DATA_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.C_S_AXI_LITE_DATA_WIDTH}] ${MODELPARAM_VALUE.C_S_AXI_LITE_DATA_WIDTH}
}

proc update_MODELPARAM_VALUE.C_S_AXI_LITE_ADDR_WIDTH { MODELPARAM_VALUE.C_S_AXI_LITE_ADDR_WIDTH PARAM_VALUE.C_S_AXI_LITE_ADDR_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.C_S_AXI_LITE_ADDR_WIDTH}] ${MODELPARAM_VALUE.C_S_AXI_LITE_ADDR_WIDTH}
}

proc update_MODELPARAM_VALUE.C_M_AXI_ADDR_WIDTH { MODELPARAM_VALUE.C_M_AXI_ADDR_WIDTH PARAM_VALUE.C_M_AXI_ADDR_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.C_M_AXI_ADDR_WIDTH}] ${MODELPARAM_VALUE.C_M_AXI_ADDR_WIDTH}
}

proc update_MODELPARAM_VALUE.C_M_AXI_DATA_WIDTH { MODELPARAM_VALUE.C_M_AXI_DATA_WIDTH PARAM_VALUE.C_M_AXI_DATA_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.C_M_AXI_DATA_WIDTH}] ${MODELPARAM_VALUE.C_M_AXI_DATA_WIDTH}
}

proc update_MODELPARAM_VALUE.SENSOR_TDATA_WIDTH { MODELPARAM_VALUE.SENSOR_TDATA_WIDTH PARAM_VALUE.SENSOR_TDATA_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.SENSOR_TDATA_WIDTH}] ${MODELPARAM_VALUE.SENSOR_TDATA_WIDTH}
}

proc update_MODELPARAM_VALUE.VIDEO_TDATA_WIDTH { MODELPARAM_VALUE.VIDEO_TDATA_WIDTH PARAM_VALUE.VIDEO_TDATA_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.VIDEO_TDATA_WIDTH}] ${MODELPARAM_VALUE.VIDEO_TDATA_WIDTH}
}

proc update_MODELPARAM_VALUE.DDR_CACHE_ENABLE { MODELPARAM_VALUE.DDR_CACHE_ENABLE PARAM_VALUE.DDR_CACHE_ENABLE } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.DDR_CACHE_ENABLE}] ${MODELPARAM_VALUE.DDR_CACHE_ENABLE}
}

proc update_MODELPARAM_VALUE.CACHE_LINE_BYTES { MODELPARAM_VALUE.CACHE_LINE_BYTES PARAM_VALUE.CACHE_LINE_BYTES } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.CACHE_LINE_BYTES}] ${MODELPARAM_VALUE.CACHE_LINE_BYTES}
}

proc update_MODELPARAM_VALUE.CACHE_BYTES { MODELPARAM_VALUE.CACHE_BYTES PARAM_VALUE.CACHE_BYTES } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.CACHE_BYTES}] ${MODELPARAM_VALUE.CACHE_BYTES}
}

proc update_MODELPARAM_VALUE.CACHE_WAYS { MODELPARAM_VALUE.CACHE_WAYS PARAM_VALUE.CACHE_WAYS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.CACHE_WAYS}] ${MODELPARAM_VALUE.CACHE_WAYS}
}

